use crate::{
    bounding::{Aabb, UVec3, Unsigned},
    node::{Branch, Node, NodeType},
    pool::Pool,
    ElementId, NodeId, Position, TreeError,
};

use heapless::Vec as HVec;

#[derive(Default)]
pub struct Octree<U, T>
where
    U: Unsigned,
    T: Position<U = U>,
{
    pub elements: Pool<T>,
    pub nodes: Pool<Node<U>>,
    pub map: Pool<NodeId>,
    pub root: NodeId,
}

impl<U, T> Octree<U, T>
where
    U: Unsigned,
    T: Position<U = U>,
{
    pub fn from_aabb(aabb: Aabb<U>) -> Self {
        Octree {
            elements: Default::default(),
            nodes: Pool::from_aabb(aabb),
            map: Default::default(),
            root: Default::default(),
        }
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Octree {
            elements: Pool::<T>::with_capacity(capacity),
            nodes: Pool::<Node<U>>::with_capacity(capacity),
            map: Pool::<NodeId>::with_capacity(capacity),
            root: Default::default(),
        }
    }

    pub fn from_aabb_with_capacity(aabb: Aabb<U>, capacity: usize) -> Self {
        Octree {
            elements: Pool::<T>::with_capacity(capacity),
            nodes: Pool::<Node<U>>::from_aabb_with_capacity(aabb, capacity),
            map: Pool::<NodeId>::with_capacity(capacity),
            root: Default::default(),
        }
    }

    pub fn insert(&mut self, elem: T) -> Result<(), TreeError> {
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
            Ok(())
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
                    return Err(TreeError::SplitUnit(format!(
                        "Attempt to insert element into a leaf with size 1"
                    )));
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
}

#[derive(Debug)]
struct Insertion<U: Unsigned> {
    element: ElementId,
    node: NodeId,
    position: UVec3<U>,
}
