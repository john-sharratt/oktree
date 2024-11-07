//! [Octree] implementation

use crate::{
    bounding::{Aabb, TUVec3, Unsigned},
    node::{Branch, Node, NodeType},
    pool::Pool,
    ElementId, NodeId, Position, TreeError,
};

use heapless::Vec as HVec;

/// Fast implementation of the octree data structure.
///
/// Helps to speed up spatial operations with stored data,
/// such as intersections, ray casting e.t.c
/// All coordinates should be positive and integer ([`Unsigned`](num::Unsigned)),
/// due to applied optimisations.
#[derive(Default)]
pub struct Octree<U, T>
where
    U: Unsigned,
    T: Position<U = U>,
{
    /// [`Pool`] of stored elements. Access it by [`ElementId`]
    pub elements: Pool<T>,

    /// [`Pool`] of tree [`Nodes`](crate::node::Node). Access it by [`NodeId`]
    pub nodes: Pool<Node<U>>,

    /// Every element caches its' [`NodeId`].
    /// Drastically speedup the elements removal.
    /// Access it by [`ElementId`]
    pub map: Pool<NodeId>,

    pub root: NodeId,
}

impl<U, T> Octree<U, T>
where
    U: Unsigned,
    T: Position<U = U>,
{
    /// Construct a tree from [`Aabb`].
    /// The root node will adopt aabb's dimensions.
    pub fn from_aabb(aabb: Aabb<U>) -> Self {
        Octree {
            elements: Default::default(),
            nodes: Pool::from_aabb(aabb),
            map: Default::default(),
            root: Default::default(),
        }
    }

    /// Construct a tree with capacity for it's pools.
    /// Helps to reduce the amount of the memory reallocations.
    pub fn with_capacity(capacity: usize) -> Self {
        Octree {
            elements: Pool::<T>::with_capacity(capacity),
            nodes: Pool::<Node<U>>::with_capacity(capacity),
            map: Pool::<NodeId>::with_capacity(capacity),
            root: Default::default(),
        }
    }

    /// Construct a tree from [`Aabb`] and capacity.
    /// Helps to reduce the amount of the memory reallocations.
    /// The root node will adopt aabb's dimensions.
    pub fn from_aabb_with_capacity(aabb: Aabb<U>, capacity: usize) -> Self {
        Octree {
            elements: Pool::<T>::with_capacity(capacity),
            nodes: Pool::<Node<U>>::from_aabb_with_capacity(aabb, capacity),
            map: Pool::<NodeId>::with_capacity(capacity),
            root: Default::default(),
        }
    }

    /// Insert an element into a tree.
    ///
    /// Recursively subdivide the space, creating new [`nodes`](crate::node::Node)
    /// Returns inserted element's [id](ElementId)
    pub fn insert(&mut self, elem: T) -> Result<ElementId, TreeError> {
        let position = elem.position();
        if self.nodes[self.root].aabb.contains(position) {
            let element = self.elements.insert(elem);
            self.map.insert(0.into());

            let mut insertions: HVec<Insertion<U>, 2> = HVec::new();
            unsafe {
                insertions.push_unchecked(Insertion {
                    element,
                    node: self.root,
                    position,
                });
            }

            while let Some(insertion) = insertions.pop() {
                match self._insert(insertion, &mut insertions) {
                    Ok(()) => (),
                    Err(err) => {
                        self.elements.remove(element);
                        self.map.remove(element);
                        return Err(err);
                    }
                }
            }

            Ok(element)
        } else {
            Err(TreeError::OutOfTreeBounds(format!(
                "{position} is outside of aabb: min: {} max: {}",
                self.nodes[self.root].aabb.min, self.nodes[self.root].aabb.max,
            )))
        }
    }

    fn _insert(
        &mut self,
        insertion: Insertion<U>,
        insertions: &mut HVec<Insertion<U>, 2>,
    ) -> Result<(), TreeError> {
        let Insertion {
            element,
            node,
            position,
        } = insertion;

        let ntype = self.nodes[node].ntype;
        match ntype {
            NodeType::Empty => {
                let n = &mut self.nodes[node];
                n.ntype = NodeType::Leaf(element);
                if let Some(parent) = n.parent {
                    match self.nodes[parent].ntype {
                        NodeType::Branch(ref mut branch) => {
                            branch.increment();
                        }
                        _ => {
                            return Err(TreeError::NotBranch(format!(
                                "Attempt to increment a node with type {}",
                                self.nodes[parent].ntype
                            )))
                        }
                    }
                }
                self.map[element] = node;
            }

            NodeType::Leaf(e) => {
                if self.nodes[node].aabb.unit() {
                    return Err(TreeError::SplitUnit(
                        "Attempt to insert element into a leaf with size 1".into(),
                    ));
                }
                let children = self.nodes.branch(node);

                let n = &mut self.nodes[node];
                n.ntype = NodeType::Branch(Branch::new(children));
                unsafe {
                    insertions.push_unchecked(insertion);
                    insertions.push_unchecked(Insertion {
                        element: e,
                        node,
                        position: self.elements[e].position(),
                    })
                }
            }

            NodeType::Branch(branch) => {
                let center = self.nodes[node].aabb.center();
                let child: NodeId = branch.find_child(position, center)?;
                unsafe {
                    insertions.push_unchecked(Insertion {
                        element,
                        node: child,
                        position,
                    })
                }
            }
        }

        Ok(())
    }

    /// Remove an element from the tree.
    /// Recursively collapse an empty [`nodes`](crate::node::Node).
    /// No memory deallocaton happening.
    /// Element is only marked as removed and could be reused.
    pub fn remove(&mut self, element: ElementId) -> Result<(), TreeError> {
        let node = self.map[element];
        let n = &mut self.nodes[node];
        let parent = n.parent;
        match n.ntype {
            NodeType::Leaf(_) => {
                self.elements.remove(element);
                self.map.remove(element);
                n.ntype = NodeType::Empty;
                if let Some((element, node)) = self.nodes.collapse(parent)? {
                    self.map[element] = node;
                }
                Ok(())
            }
            _ => Err(TreeError::NotLeaf(format!(
                "Attemt to remove element from {}",
                n.ntype
            ))),
        }
    }

    /// Consumes a tree, converting it into a [`vector`](Vec).
    pub fn to_vec(self) -> Vec<T> {
        let garbage = self.elements.garbage;
        self.elements
            .vec
            .into_iter()
            .enumerate()
            .filter_map(|(i, element)| {
                if garbage.contains(&i) {
                    None
                } else {
                    Some(element)
                }
            })
            .collect()
    }
}

#[derive(Debug)]
struct Insertion<U: Unsigned> {
    element: ElementId,
    node: NodeId,
    position: TUVec3<U>,
}
