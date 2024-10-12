#![allow(dead_code)]
#![feature(strict_overflow_ops)]
#![feature(trait_alias)]

pub mod bounding;

use std::{
    array::from_fn,
    error::Error,
    fmt::{self},
    ops::{Index, IndexMut},
};

use bounding::{Aabb, UVec3, Unsigned};
use num::cast;

trait Translatable {
    type U: Unsigned;

    fn translation(&self) -> UVec3<Self::U>;
}

trait Nodable {
    fn set_node(&mut self, node: NodeId);

    fn get_node(&self) -> NodeId;
}
#[derive(Default)]
struct Octree<U, T>
where
    U: Unsigned,
    T: Translatable<U = U> + Nodable,
{
    elements: Pool<T>,
    nodes: Pool<Node<U>>,
    root: NodeId,
}

impl<U, T> Octree<U, T>
where
    U: Unsigned,
    T: Translatable<U = U> + Nodable,
{
    pub fn from_aabb(aabb: Aabb<U>) -> Self {
        Octree {
            elements: Default::default(),
            nodes: Pool::from_aabb(aabb),
            root: Default::default(),
        }
    }

    pub fn insert(&mut self, elem: T) -> Result<(), TreeError> {
        let position = elem.translation();
        let element = self.elements.insert(elem);
        if self.nodes[self.root].contains(position) {
            self.rinsert(element, self.root, position)?;
            Ok(())
        } else {
            Err(TreeError::OutOfTreeBounds(format!(
                "{position} is outside of aabb: min: {} max: {}",
                self.nodes[self.root].aabb.min, self.nodes[self.root].aabb.max,
            )))
        }
    }

    fn rinsert(
        &mut self,
        element: ElementId,
        node: NodeId,
        position: UVec3<U>,
    ) -> Result<(), TreeError> {
        let ntype = self.nodes[node].ntype;
        match ntype {
            NodeType::Empty => {
                let n = &mut self.nodes[node];
                n.ntype = NodeType::Leaf(element);
                if let Some(parent) = n.parent {
                    self.nodes[parent].increment()?;
                }
                self.elements[element].set_node(node);
                Ok(())
            }

            NodeType::Leaf(e) => {
                let aabb = self.nodes[node].aabb;
                let children = self.nodes.branch(node, aabb);

                let n = &mut self.nodes[node];
                n.ntype = NodeType::Branch(Branch::new(children));
                self.rinsert(e, node, self.elements[e].translation())?;
                self.rinsert(element, node, position)?;
                Ok(())
            }

            NodeType::Branch(branch) => {
                let n = &self.nodes[node];
                let child: NodeId = branch.find_child(position, n.aabb.center())?;
                self.rinsert(element, child, position)?;
                Ok(())
            }
        }
    }

    fn remove(&mut self, element: ElementId) -> Result<(), TreeError> {
        let node = self.elements[element].get_node();
        let n = &mut self.nodes[node];
        let parent = n.parent;
        match n.ntype {
            NodeType::Leaf(_) => {
                self.elements.remove(element);
                n.ntype = NodeType::Empty;
                self.nodes.collapse(parent)?;
                Ok(())
            }
            _ => Err(TreeError::NotLeaf(format!(
                "Attemt to remove element from {}",
                n.ntype
            ))),
        }
    }
}

struct Pool<T> {
    vec: Vec<T>,
    garbage: Vec<usize>,
}

impl<U: Unsigned> Default for Pool<Node<U>> {
    fn default() -> Self {
        let root = Node::default();
        let vec = vec![root];

        Pool {
            vec,
            garbage: Default::default(),
        }
    }
}

impl<T: Translatable> Default for Pool<T> {
    fn default() -> Self {
        Pool {
            vec: Default::default(),
            garbage: Default::default(),
        }
    }
}

impl<U: Unsigned> Index<NodeId> for Pool<Node<U>> {
    type Output = Node<U>;

    fn index(&self, index: NodeId) -> &Self::Output {
        debug_assert!(
            !self.garbage.contains(&index.into()),
            "Indexing garbaged node"
        );
        &self.vec[index.0 as usize]
    }
}

impl<U: Unsigned> IndexMut<NodeId> for Pool<Node<U>> {
    fn index_mut(&mut self, index: NodeId) -> &mut Self::Output {
        debug_assert!(
            !self.garbage.contains(&index.into()),
            "Mut Indexing garbaged node"
        );
        &mut self.vec[index.0 as usize]
    }
}

impl<T: Translatable> Index<ElementId> for Pool<T> {
    type Output = T;

    fn index(&self, index: ElementId) -> &Self::Output {
        debug_assert!(
            !self.garbage.contains(&index.into()),
            "Indexing garbaged element"
        );
        &self.vec[index.0 as usize]
    }
}

impl<T: Translatable> IndexMut<ElementId> for Pool<T> {
    fn index_mut(&mut self, index: ElementId) -> &mut Self::Output {
        debug_assert!(
            !self.garbage.contains(&index.into()),
            "Mut Indexing garbaged element"
        );
        &mut self.vec[index.0 as usize]
    }
}

impl<T> Pool<T> {
    fn _insert(&mut self, t: T) -> usize {
        if let Some(idx) = self.garbage.pop() {
            self.vec[idx] = t;
            idx
        } else {
            self.vec.push(t);
            self.vec.len() - 1
        }
    }

    fn len(&self) -> usize {
        self.vec.len() - self.garbage_len()
    }

    fn garbage_len(&self) -> usize {
        self.garbage.len()
    }
}

impl<U: Unsigned> Pool<Node<U>> {
    fn from_aabb(aabb: Aabb<U>) -> Self {
        let root = Node::from_aabb(aabb, None);
        let vec = vec![root];
        Pool {
            vec,
            garbage: Default::default(),
        }
    }

    fn insert(&mut self, t: Node<U>) -> NodeId {
        self._insert(t).into()
    }

    fn remove(&mut self, node: NodeId) {
        self.garbage.push(node.into());
    }

    fn branch(&mut self, parent: NodeId, aabb: Aabb<U>) -> [NodeId; 8] {
        let min = aabb.min;
        let max = aabb.max;
        let mid = aabb.center();

        from_fn(|i| self.geni_child(i, min, mid, max, parent))
    }

    fn geni_child(
        &mut self,
        i: usize,
        min: UVec3<U>,
        mid: UVec3<U>,
        max: UVec3<U>,
        parent: NodeId,
    ) -> NodeId {
        let x_mask = (i & 0b1) == 1;
        let y_mask = (i & 0b10) == 1;
        let z_mask = (i & 0b100) == 1;

        let min = UVec3::new(
            if x_mask { mid.x } else { min.x },
            if y_mask { mid.y } else { min.y },
            if z_mask { mid.z } else { min.z },
        );

        let max = UVec3::new(
            if x_mask { max.x } else { mid.x },
            if y_mask { max.y } else { mid.y },
            if z_mask { max.z } else { mid.z },
        );

        let aabb = Aabb { min, max };
        let node = Node::from_aabb(aabb, Some(parent));
        self.insert(node)
    }

    fn collapse(&mut self, parent: Option<NodeId>) -> Result<(), TreeError> {
        if let Some(parent) = parent {
            let p = &mut self[parent];
            p.decrement()?;
            let parent = p.parent;
            if p.fullness()? == 0 {
                let children = p.collapse()?;
                children.map(|child| self.remove(child));
                self.collapse(parent)?;
            }
        }

        Ok(())
    }
}

impl<T: Translatable> Pool<T> {
    fn insert(&mut self, t: T) -> ElementId {
        self._insert(t).into()
    }

    fn remove(&mut self, element: ElementId) {
        self.garbage.push(element.into());
    }
}

#[derive(Clone, Copy)]
struct Node<U: Unsigned> {
    aabb: Aabb<U>,
    ntype: NodeType,
    parent: Option<NodeId>,
}

impl<U: Unsigned> Default for Node<U> {
    fn default() -> Self {
        Node {
            aabb: Aabb {
                min: UVec3::new(cast(0).unwrap(), cast(0).unwrap(), cast(0).unwrap()),
                max: UVec3::new(cast(1).unwrap(), cast(1).unwrap(), cast(1).unwrap()),
            },
            ntype: Default::default(),
            parent: Default::default(),
        }
    }
}

impl<U: Unsigned> Node<U> {
    fn from_aabb(aabb: Aabb<U>, parent: Option<NodeId>) -> Self {
        Node {
            aabb,
            parent,
            ..Default::default()
        }
    }

    fn contains(&self, position: UVec3<U>) -> bool {
        let lemin = self.aabb.min.le(position);
        let gtmax = self.aabb.max.gt(position);

        lemin.all() && gtmax.all()
    }

    fn increment(&mut self) -> Result<(), TreeError> {
        match self.ntype {
            NodeType::Branch(ref mut branch) => {
                branch.increment();
                Ok(())
            }
            _ => Err(TreeError::NotBranch(format!(
                "Attemt to increment child count for {} node",
                self.ntype
            ))),
        }
    }

    fn decrement(&mut self) -> Result<(), TreeError> {
        match self.ntype {
            NodeType::Branch(ref mut branch) => {
                branch.decrement();
                Ok(())
            }
            _ => Err(TreeError::NotBranch(format!(
                "Attemt to decrement negative child count for {} node",
                self.ntype
            ))),
        }
    }

    fn fullness(&self) -> Result<u8, TreeError> {
        match self.ntype {
            NodeType::Branch(Branch { filled, .. }) => Ok(filled),
            _ => Err(TreeError::NotBranch(format!(
                "Attemt to get child count for {} node",
                self.ntype
            ))),
        }
    }

    fn collapse(&mut self) -> Result<[NodeId; 8], TreeError> {
        match self.ntype {
            NodeType::Branch(Branch { children, filled }) => match filled {
                0 => {
                    self.ntype = NodeType::Empty;
                    Ok(children)
                }
                _ => Err(TreeError::CollapseNonEmpty(format!(
                    "Collapsing a non empty branch"
                ))),
            },
            _ => Err(TreeError::NotBranch(format!("Collapse a {}", self.ntype))),
        }
    }
}

#[derive(Default, Clone, Copy, PartialEq, Debug)]
enum NodeType {
    #[default]
    Empty,
    Leaf(ElementId),
    Branch(Branch),
}

impl fmt::Display for NodeType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            NodeType::Empty => write!(f, "NodeType: Empty"),
            NodeType::Leaf(e) => write!(f, "NodeType: Leaf({e})"),
            NodeType::Branch(_) => write!(f, "NodeType: Branch()"),
        }
    }
}

#[derive(Default, Clone, Copy, PartialEq, Debug)]
struct Branch {
    children: [NodeId; 8],
    filled: u8,
}

impl Branch {
    fn new(children: [NodeId; 8]) -> Self {
        Branch {
            children,
            ..Default::default()
        }
    }

    fn filled(children: [NodeId; 8], filled: u8) -> Self {
        Branch { children, filled }
    }

    fn increment(&mut self) {
        self.filled = self.filled.strict_add(1);
        debug_assert!(self.filled <= 8);
    }

    fn decrement(&mut self) {
        self.filled = self.filled.strict_sub(1);
    }

    fn find_child<U: Unsigned>(
        &self,
        position: UVec3<U>,
        center: UVec3<U>,
    ) -> Result<NodeId, TreeError> {
        let x = if position.x < center.x { 0 } else { 1 };
        let y = if position.y < center.y { 0 } else { 1 };
        let z = if position.z < center.z { 0 } else { 1 };

        let idx = x | y << 1 | z << 2;

        Ok(self.children[idx])
    }
}

#[derive(Default, Clone, Copy, PartialEq, Debug)]
struct NodeId(u32);

impl From<NodeId> for usize {
    fn from(value: NodeId) -> Self {
        value.0 as usize
    }
}

impl From<usize> for NodeId {
    fn from(value: usize) -> Self {
        NodeId(value as u32)
    }
}

impl fmt::Display for NodeId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "NodeId {}", self.0)
    }
}

#[derive(Default, Clone, Copy, PartialEq, Debug)]
struct ElementId(u32);

impl From<ElementId> for usize {
    fn from(value: ElementId) -> Self {
        value.0 as usize
    }
}

impl From<usize> for ElementId {
    fn from(value: usize) -> Self {
        ElementId(value as u32)
    }
}

impl fmt::Display for ElementId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "ElementId: {}", self.0)
    }
}

#[derive(Debug)]
pub enum TreeError {
    OutOfTreeBounds(String),
    NotBranch(String),
    NotLeaf(String),
    CollapseNonEmpty(String),
}

impl Error for TreeError {}

impl fmt::Display for TreeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TreeError::OutOfTreeBounds(info) => write!(f, "Out of tree bounds. {info}"),
            TreeError::NotBranch(info) => write!(f, "Node is not a Branch. {info}"),
            TreeError::NotLeaf(info) => write!(f, "Node is not a Leaf. {info}"),
            TreeError::CollapseNonEmpty(info) => write!(f, "Collapsing non empty branch. {info}"),
        }
    }
}

#[cfg(test)]
mod tests {

    use super::*;

    struct DummyCell<U: Unsigned> {
        position: UVec3<U>,
        node: NodeId,
    }

    impl<U: Unsigned> Translatable for DummyCell<U> {
        type U = U;
        fn translation(&self) -> UVec3<U> {
            self.position
        }
    }

    impl<U: Unsigned> Nodable for DummyCell<U> {
        fn get_node(&self) -> NodeId {
            self.node
        }

        fn set_node(&mut self, node: NodeId) {
            self.node = node
        }
    }

    impl<U: Unsigned> DummyCell<U> {
        fn new(position: UVec3<U>) -> Self {
            DummyCell {
                position,
                node: Default::default(),
            }
        }
    }

    #[test]
    fn test_insert() {
        let mut tree = Octree::from_aabb(Aabb::new(UVec3::new(4, 4, 4), 4));

        assert_eq!(tree.elements.len(), 0);
        assert_eq!(tree.elements.garbage_len(), 0);

        assert_eq!(tree.nodes.len(), 1);
        assert_eq!(tree.nodes.garbage_len(), 0);

        assert_eq!(tree.nodes[0.into()].ntype, NodeType::Empty);
        assert_eq!(tree.nodes[0.into()].parent, None);

        let c1 = DummyCell::new(UVec3::new(1u8, 1, 1));
        tree.insert(c1).unwrap();

        assert_eq!(tree.elements.len(), 1);
        assert_eq!(tree.elements.garbage_len(), 0);

        assert_eq!(tree.nodes.len(), 1);
        assert_eq!(tree.nodes.garbage_len(), 0);

        assert_eq!(tree.nodes[0.into()].ntype, NodeType::Leaf(0.into()));
        assert_eq!(tree.nodes[0.into()].parent, None);

        assert_eq!(tree.elements[0.into()].get_node(), 0.into());

        let c2 = DummyCell::new(UVec3::new(7, 7, 7));
        tree.insert(c2).unwrap();

        assert_eq!(tree.elements.len(), 2);
        assert_eq!(tree.elements.garbage_len(), 0);

        assert_eq!(tree.nodes.len(), 9);
        assert_eq!(tree.nodes.garbage_len(), 0);

        assert_eq!(tree.nodes[0.into()].parent, None);

        let children = from_fn(|i| NodeId(i as u32 + 1));
        assert_eq!(
            tree.nodes[0.into()].ntype,
            NodeType::Branch(Branch::filled(children, 2))
        );

        assert_eq!(tree.nodes[1.into()].ntype, NodeType::Leaf(0.into()));
        assert_eq!(tree.nodes[1.into()].parent, Some(0.into()));
        assert_eq!(tree.nodes[8.into()].ntype, NodeType::Leaf(1.into()));
        assert_eq!(tree.nodes[8.into()].parent, Some(0.into()));
        for i in 2..8 {
            assert_eq!(tree.nodes[i.into()].ntype, NodeType::Empty);
        }

        assert_eq!(tree.elements[0.into()].get_node(), 1.into());
        assert_eq!(tree.elements[1.into()].get_node(), 8.into());
    }

    #[test]
    fn test_remove() {
        let mut tree = Octree::from_aabb(Aabb::new(UVec3::new(8u16, 8, 8), 8));

        let c1 = DummyCell::new(UVec3::new(1, 1, 1));
        tree.insert(c1).unwrap();
        let c2 = DummyCell::new(UVec3::new(2, 2, 2));
        tree.insert(c2).unwrap();
        assert_eq!(tree.nodes.len(), 25);
        assert_eq!(tree.elements.len(), 2);

        tree.remove(0.into()).unwrap();

        assert_eq!(tree.elements.len(), 1);
        assert_eq!(tree.nodes.len(), 25);

        tree.remove(1.into()).unwrap();

        assert_eq!(tree.elements.len(), 0);
        assert_eq!(tree.nodes.len(), 1);

        assert_eq!(tree.nodes[0.into()].ntype, NodeType::Empty)
    }
}
