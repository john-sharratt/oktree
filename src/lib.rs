#![allow(dead_code)]
#![feature(strict_overflow_ops)]
#![feature(trait_alias)]

#[cfg(feature = "bevy")]
pub mod bevy_integration;
pub mod bounding;

use std::{
    array::from_fn,
    error::Error,
    fmt::{self},
    ops::{Index, IndexMut},
};

use bounding::{Aabb, UVec3, Unsigned};
use num::cast;

pub trait Translatable {
    type U: Unsigned;

    fn translation(&self) -> UVec3<Self::U>;
}

pub trait Nodable {
    fn set_node(&mut self, node: NodeId);

    fn get_node(&self) -> NodeId;
}

#[derive(Default)]
pub struct Octree<U, T>
where
    U: Unsigned,
    T: Translatable<U = U> + Nodable,
{
    pub elements: Pool<T>,
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
        if self.nodes[self.root].aabb.contains(position) {
            let element = self.elements.insert(elem);
            match self.rinsert(element, self.root, position) {
                Ok(()) => Ok(()),
                Err(err) => {
                    self.elements.remove(element);
                    Err(err)
                }
            }
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
                self.elements[element].set_node(node);
                Ok(())
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
                self.rinsert(e, node, self.elements[e].translation())?;
                self.rinsert(element, node, position)?;
                Ok(())
            }

            NodeType::Branch(branch) => {
                let center = self.nodes[node].aabb.center();
                let child: NodeId = branch.find_child(position, center)?;
                self.rinsert(element, child, position)?;
                Ok(())
            }
        }
    }

    pub fn remove(&mut self, element: ElementId) -> Result<(), TreeError> {
        let node = self.elements[element].get_node();
        let n = &mut self.nodes[node];
        let parent = n.parent;
        match n.ntype {
            NodeType::Leaf(_) => {
                self.elements.remove(element);
                n.ntype = NodeType::Empty;
                if let Some((element, node)) = self.nodes.collapse(parent)? {
                    self.elements[element].set_node(node);
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

pub struct Pool<T> {
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
            "Indexing garbaged node: {index}"
        );
        self.get_unchecked(index)
    }
}

impl<U: Unsigned> IndexMut<NodeId> for Pool<Node<U>> {
    fn index_mut(&mut self, index: NodeId) -> &mut Self::Output {
        debug_assert!(
            !self.garbage.contains(&index.into()),
            "Mut Indexing garbaged node: {index}"
        );
        self.get_mut_unchecked(index)
    }
}

impl<T: Translatable> Index<ElementId> for Pool<T> {
    type Output = T;

    fn index(&self, index: ElementId) -> &Self::Output {
        debug_assert!(
            !self.garbage.contains(&index.into()),
            "Indexing garbaged element: {index}"
        );
        self.get_unchecked(index)
    }
}

impl<T: Translatable> IndexMut<ElementId> for Pool<T> {
    fn index_mut(&mut self, index: ElementId) -> &mut Self::Output {
        debug_assert!(
            !self.garbage.contains(&index.into()),
            "Mut Indexing garbaged element: {index}"
        );
        self.get_mut_unchecked(index)
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

    pub fn len(&self) -> usize {
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

    fn branch(&mut self, parent: NodeId) -> [NodeId; 8] {
        let aabbs = self[parent].aabb.split();
        from_fn(|i| self.insert(Node::from_aabb(aabbs[i], Some(parent))))
    }

    fn collapse(
        &mut self,
        parent: Option<NodeId>,
    ) -> Result<Option<(ElementId, NodeId)>, TreeError> {
        if let Some(parent) = parent {
            let mut p = self[parent];

            match p.ntype {
                NodeType::Branch(ref mut branch) => {
                    branch.decrement();
                    match branch.filled {
                        0 => {
                            let children = branch.children;
                            p.ntype = NodeType::Empty;
                            self[parent] = p;
                            children.map(|child| self.remove(child));
                            return self.collapse(p.parent);
                        }

                        1 => {
                            for child in branch.children {
                                let c = self[child];
                                match c.ntype {
                                    NodeType::Leaf(element) => {
                                        let children = branch.children;
                                        p.ntype = NodeType::Leaf(element);
                                        self[parent] = p;
                                        children.map(|child| self.remove(child));
                                        return Ok(Some((element, parent)));
                                    }
                                    NodeType::Branch(_) => break,
                                    NodeType::Empty => (),
                                }
                            }
                        }

                        _ => (),
                    }
                }
                _ => {
                    return Err(TreeError::NotBranch(format!(
                        "Attempt to collapse a node of type {}",
                        p.ntype
                    )))
                }
            }

            self[parent] = p;
        }
        Ok(None)
    }

    pub fn get(&self, node: NodeId) -> Option<&Node<U>> {
        if !self.garbage.contains(&(node.0 as usize)) {
            self.vec.get(node.0 as usize)
        } else {
            None
        }
    }

    pub fn get_mut(&mut self, node: NodeId) -> Option<&mut Node<U>> {
        if !self.garbage.contains(&(node.0 as usize)) {
            self.vec.get_mut(node.0 as usize)
        } else {
            None
        }
    }

    pub fn get_unchecked(&self, node: NodeId) -> &Node<U> {
        &self.vec[node.0 as usize]
    }

    pub fn get_mut_unchecked(&mut self, node: NodeId) -> &mut Node<U> {
        &mut self.vec[node.0 as usize]
    }
}

impl<T: Translatable> Pool<T> {
    fn insert(&mut self, t: T) -> ElementId {
        self._insert(t).into()
    }

    fn remove(&mut self, element: ElementId) {
        self.garbage.push(element.into());
    }

    pub fn get(&self, node: ElementId) -> Option<&T> {
        if !self.garbage.contains(&(node.0 as usize)) {
            self.vec.get(node.0 as usize)
        } else {
            None
        }
    }

    pub fn get_mut(&mut self, node: ElementId) -> Option<&mut T> {
        if !self.garbage.contains(&(node.0 as usize)) {
            self.vec.get_mut(node.0 as usize)
        } else {
            None
        }
    }

    pub fn get_unchecked(&self, node: ElementId) -> &T {
        &self.vec[node.0 as usize]
    }

    pub fn get_mut_unchecked(&mut self, node: ElementId) -> &mut T {
        &mut self.vec[node.0 as usize]
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

    fn fullness(&self) -> Result<u8, TreeError> {
        match self.ntype {
            NodeType::Branch(Branch { filled, .. }) => Ok(filled),
            _ => Err(TreeError::NotBranch(format!(
                "Attemt to get child count for {} node",
                self.ntype
            ))),
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
            NodeType::Branch(branch) => write!(f, "NodeType: Branch({:?})", branch),
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
pub struct NodeId(u32);

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
pub struct ElementId(u32);

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

#[derive(Debug, PartialEq)]
pub enum TreeError {
    OutOfTreeBounds(String),
    NotBranch(String),
    NotLeaf(String),
    CollapseNonEmpty(String),
    SplitUnit(String),
}

impl Error for TreeError {}

impl fmt::Display for TreeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TreeError::OutOfTreeBounds(info) => write!(f, "Out of tree bounds. {info}"),
            TreeError::NotBranch(info) => write!(f, "Node is not a Branch. {info}"),
            TreeError::NotLeaf(info) => write!(f, "Node is not a Leaf. {info}"),
            TreeError::CollapseNonEmpty(info) => write!(f, "Collapsing non empty branch. {info}"),
            TreeError::SplitUnit(info) => write!(f, "Splitting AABB with size of 1. {info}"),
        }
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    use rand::Rng;

    const RANGE: usize = 4096;

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
        assert_eq!(tree.insert(c1), Ok(()));

        assert_eq!(tree.elements.len(), 1);
        assert_eq!(tree.elements.garbage_len(), 0);

        assert_eq!(tree.nodes.len(), 1);
        assert_eq!(tree.nodes.garbage_len(), 0);

        assert_eq!(tree.nodes[0.into()].ntype, NodeType::Leaf(0.into()));
        assert_eq!(tree.nodes[0.into()].parent, None);

        assert_eq!(tree.elements[0.into()].get_node(), 0.into());

        let c2 = DummyCell::new(UVec3::new(7, 7, 7));
        assert_eq!(tree.insert(c2), Ok(()));

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
        assert_eq!(tree.insert(c1), Ok(()));
        let c2 = DummyCell::new(UVec3::new(2, 2, 2));
        assert_eq!(tree.insert(c2), Ok(()));

        assert_eq!(tree.nodes.len(), 25);

        let c2r = DummyCell::new(UVec3::new(1, 1, 1));
        assert_eq!(
            tree.insert(c2r),
            Err(TreeError::SplitUnit(
                "Attempt to insert element into a leaf with size 1".into()
            ))
        );

        assert_eq!(tree.nodes.len(), 33);
        assert_eq!(tree.elements.len(), 2);

        tree.remove(0.into()).unwrap();

        assert_eq!(tree.elements.len(), 1);
        assert_eq!(tree.nodes.len(), 17);

        tree.remove(1.into()).unwrap();

        assert_eq!(tree.elements.len(), 0);
        assert_eq!(tree.nodes.len(), 1);

        assert_eq!(tree.nodes[0.into()].ntype, NodeType::Empty)
    }

    #[test]
    fn test_insert_remove() {
        let mut tree = Octree::from_aabb(Aabb::new(UVec3::new(4u8, 4, 4), 4));

        let c1 = DummyCell::new(UVec3::new(1, 1, 1));
        assert_eq!(tree.insert(c1), Ok(()));

        let c2 = DummyCell::new(UVec3::new(2, 2, 1));
        assert_eq!(tree.insert(c2), Ok(()));

        let c3 = DummyCell::new(UVec3::new(6, 6, 1));
        assert_eq!(tree.insert(c3), Ok(()));

        let c4 = DummyCell::new(UVec3::new(7, 7, 1));
        assert_eq!(tree.insert(c4), Ok(()));

        let c5 = DummyCell::new(UVec3::new(6, 7, 1));
        assert_eq!(tree.insert(c5), Ok(()));

        assert_eq!(tree.nodes[0.into()].fullness(), Ok(2));
        assert_eq!(tree.nodes[1.into()].fullness(), Ok(2));
        assert_eq!(tree.nodes[20.into()].fullness(), Ok(3));

        assert_eq!(tree.remove(0.into()), Ok(()));

        assert_eq!(tree.nodes[0.into()].fullness(), Ok(2));
        assert_eq!(tree.nodes[1.into()].ntype, NodeType::Leaf(1.into()));
        assert_eq!(tree.nodes[20.into()].fullness(), Ok(3));

        assert_eq!(tree.remove(1.into()), Ok(()));

        assert_eq!(tree.nodes[0.into()].fullness(), Ok(1));
        assert_eq!(tree.nodes[1.into()].ntype, NodeType::Empty);
        assert_eq!(tree.nodes[20.into()].fullness(), Ok(3));

        assert_eq!(tree.remove(2.into()), Ok(()));
        assert_eq!(tree.remove(3.into()), Ok(()));

        assert_eq!(tree.nodes[0.into()].fullness(), Ok(1));
        assert_eq!(tree.nodes[1.into()].ntype, NodeType::Empty);
        assert_eq!(tree.nodes[20.into()].ntype, NodeType::Leaf(4.into()));

        assert_eq!(tree.remove(4.into()), Ok(()));

        assert_eq!(tree.nodes[0.into()].ntype, NodeType::Empty);
        assert_eq!(tree.nodes.len(), 1);
        assert_eq!(tree.elements.len(), 0);
    }

    fn random_points() -> [DummyCell<usize>; RANGE] {
        let mut rnd = rand::thread_rng();
        from_fn(|_| {
            let x = rnd.gen_range(0..=RANGE);
            let y = rnd.gen_range(0..=RANGE);
            let z = rnd.gen_range(0..=RANGE);
            let position = UVec3::new(x, y, z);
            DummyCell::new(position)
        })
    }

    #[test]
    fn test_4096() {
        let mut tree = Octree::from_aabb(Aabb::new(UVec3::splat(RANGE / 2), RANGE / 2));

        for p in random_points() {
            let _ = tree.insert(p);
        }

        assert!(tree.elements.len() > (RANGE as f32 * 0.98) as usize);

        for element in 0..tree.elements.len() {
            if let Err(err) = tree.remove(element.into()) {
                println!("{err} || {}", tree.elements[element.into()].translation());
            }
        }

        assert_eq!(tree.elements.len(), 0);
    }
}
