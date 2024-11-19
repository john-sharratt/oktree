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
    /// aabb used for clearing the octree
    aabb: Option<Aabb<U>>,

    /// [`Pool`] of stored elements. Access it by [`ElementId`]
    pub(crate) elements: Pool<T>,

    /// [`Pool`] of tree [`Nodes`](crate::node::Node). Access it by [`NodeId`]
    pub(crate) nodes: Pool<Node<U>>,

    /// Every element caches its' [`NodeId`].
    /// Drastically speedup the elements removal.
    /// Access it by [`ElementId`]
    pub(crate) map: Pool<NodeId>,

    pub(crate) root: NodeId,
}

impl<U, T> Octree<U, T>
where
    U: Unsigned,
    T: Position<U = U>,
{
    /// Construct a tree from [`Aabb`].
    ///
    /// `aabb` should be positive and it's dimensions should be the power of 2.
    /// The root node will adopt aabb's dimensions.
    pub fn from_aabb(aabb: Aabb<U>) -> Self {
        Octree {
            aabb: Some(aabb),
            elements: Default::default(),
            nodes: Pool::from_aabb(aabb),
            map: Default::default(),
            root: Default::default(),
        }
    }

    /// Construct a tree with capacity for it's pools.
    ///
    /// Helps to reduce the amount of the memory reallocations.
    pub fn with_capacity(capacity: usize) -> Self {
        Octree {
            aabb: None,
            elements: Pool::<T>::with_capacity(capacity),
            nodes: Pool::<Node<U>>::with_capacity(capacity),
            map: Pool::<NodeId>::with_capacity(capacity),
            root: Default::default(),
        }
    }

    /// Construct a tree from [`Aabb`] and capacity.
    ///
    /// `aabb` should be positive and it's dimensions should be the power of 2.
    /// Helps to reduce the amount of the memory reallocations.
    /// The root node will adopt aabb's dimensions.
    pub fn from_aabb_with_capacity(aabb: Aabb<U>, capacity: usize) -> Self {
        Octree {
            aabb: Some(aabb),
            elements: Pool::<T>::with_capacity(capacity),
            nodes: Pool::<Node<U>>::from_aabb_with_capacity(aabb, capacity),
            map: Pool::<NodeId>::with_capacity(capacity),
            root: Default::default(),
        }
    }

    /// Insert an element into a tree.
    ///
    /// Recursively subdivide the space, creating new [`nodes`](crate::node::Node)
    /// Returns inserted element's [`id`](ElementId)
    ///
    /// ```ignore
    /// let mut tree = Octree::from_aabb(Aabb::new(TUVec3::splat(16), 16));
    /// let c1 = DummyCell::new(TUVec3::new(1u8, 1, 1));
    /// let c1_id = tree.insert(c1).unwrap();
    ///
    /// assert_eq!(c1_id, ElementId(0))
    /// ```
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
                let child: NodeId = branch.find_child(position, center);
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

    /// Upserts an element into a tree.
    ///
    /// Recursively subdivide the space, creating new [`nodes`](crate::node::Node)
    /// Returns inserted element's [`id`](ElementId)
    ///
    /// ```ignore
    /// let mut tree = Octree::from_aabb(Aabb::new(TUVec3::splat(16), 16));
    /// let c1 = DummyCell::new(TUVec3::new(1u8, 1, 1));
    ///
    /// let c1_id = tree.upsert(c1).unwrap();
    /// assert_eq!(c1_id, ElementId(0))
    ///
    /// let c1_id = tree.upsert(c1).unwrap();
    /// assert_eq!(c1_id, ElementId(1))
    /// ```
    pub fn upsert(&mut self, elem: T) -> Result<ElementId, TreeError> {
        if let Some(existing) = self.find(elem.position()) {
            self.remove(existing)?;
        }
        self.insert(elem)
    }

    /// Remove an element from the tree.
    ///
    /// Recursively collapse an empty [`nodes`](crate::node::Node).
    /// No memory deallocaton happening.
    /// Element is only marked as removed and could be reused.
    ///
    /// ```ignore
    /// let mut tree = Octree::from_aabb(Aabb::new(TUVec3::splat(16), 16));
    /// let c1 = DummyCell::new(TUVec3::new(1u8, 1, 1));
    /// let c1_id = tree.insert(c1).unwrap();
    ///
    /// assert_eq!(tree.remove(c1_id).is_ok())
    /// ```
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

    /// Clear all the elements in the octree and reset it to the initial state.
    ///
    /// The capacity of the octree is preserved and thus the octree can be immediately
    /// reused for new elements without causing any memory reallocations.
    pub fn clear(&mut self) {
        self.elements.clear();
        self.map.clear();
        if let Some(aabb) = self.aabb {
            self.nodes.clear_with_aabb(aabb);
        } else {
            self.nodes.clear();
        }
        self.root = Default::default();
    }

    /// Restores all the garbage elements back to real elements. Effectively
    /// this is a rollback of all the remove operations that happened
    pub fn restore_garbage(&mut self) {
        self.elements.restore_garbage();
        self.nodes.restore_garbage();
        self.map.restore_garbage();
    }

    /// Search for the element at the [`point`](TUVec3)
    ///
    /// Returns element's [`id`](ElementId) or [`None`] if elements if not found.
    ///
    /// ```ignore
    /// let mut tree = Octree::from_aabb(Aabb::new(TUVec3::splat(16), 16));
    /// let c1 = DummyCell::new(TUVec3::new(1u8, 1, 1));
    /// tree.insert(c1).unwrap();
    ///
    /// let c2 = DummyCell::new(TUVec3::new(4, 5, 6));
    /// let eid = tree.insert(c2).unwrap();
    ///
    /// assert_eq!(tree.find(TUVec3::new(4, 5, 6)), Some(eid));
    /// assert_eq!(tree.find(TUVec3::new(2, 2, 2)), None);
    /// ```
    pub fn find(&self, point: TUVec3<U>) -> Option<ElementId> {
        self.rfind(self.root, point)
    }

    fn rfind(&self, node: NodeId, point: TUVec3<U>) -> Option<ElementId> {
        let ntype = self.nodes[node].ntype;
        match ntype {
            NodeType::Empty => None,

            NodeType::Leaf(e) => {
                if self.elements[e].position() == point {
                    Some(e)
                } else {
                    None
                }
            }

            NodeType::Branch(ref branch) => {
                let child = branch.find_child(point, self.nodes[node].aabb.center());
                self.rfind(child, point)
            }
        }
    }

    /// Returns the node's [`id`](NodeId) containing the element if element exists and not garbaged.
    pub fn get_node(&self, element: ElementId) -> Option<NodeId> {
        if self.map.is_garbaged(element) {
            None
        } else {
            Some(self.map[element])
        }
    }

    /// Consumes a tree, converting it into a [`vector`](Vec).
    pub fn to_vec(self) -> Vec<T> {
        self.elements
            .vec
            .into_iter()
            .filter(|e| !e.garbage)
            .map(|e| e.item)
            .collect()
    }

    /// Returns the number of actual elements in the tree
    pub fn len(&self) -> usize {
        self.elements.len()
    }

    /// Is the tree empty
    pub fn is_empty(&self) -> bool {
        self.elements.is_empty()
    }

    /// Returns an iterator over the elements in the tree.
    pub fn iter(&self) -> impl Iterator<Item = &T> {
        self.elements.iter()
    }
}

impl<U, T> Clone for Octree<U, T>
where
    U: Unsigned,
    T: Position<U = U> + Clone,
{
    fn clone(&self) -> Self {
        Octree {
            aabb: self.aabb,
            elements: self.elements.clone(),
            nodes: self.nodes.clone(),
            map: self.map.clone(),
            root: self.root,
        }
    }
}

#[derive(Debug)]
struct Insertion<U: Unsigned> {
    element: ElementId,
    node: NodeId,
    position: TUVec3<U>,
}
