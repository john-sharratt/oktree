#![allow(dead_code)]

use std::{
    array::from_fn,
    fmt,
    ops::{Index, IndexMut},
};

use bevy::{
    math::{
        bounding::{Aabb3d, BoundingVolume},
        Vec3A,
    },
    prelude::*,
};

trait Translatable {
    fn translation(&self) -> UVec3;
}

struct Octree<T: Translatable> {
    elements: Pool<T>,
    nodes: Pool<Node>,
    root: NodeId,
}

impl<T: Translatable> Default for Octree<T> {
    fn default() -> Self {
        Octree {
            elements: default(),
            nodes: default(),
            root: default(),
        }
    }
}

impl<T: Translatable> Octree<T> {
    pub fn from_aabb(aabb: Aabb3d) -> Self {
        Octree {
            elements: default(),
            nodes: Pool::from_aabb(aabb),
            root: default(),
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
        position: UVec3,
    ) -> Result<(), TreeError> {
        let mut n = self.nodes[node];
        match n.ntype {
            NodeType::Empty => {
                n.ntype = NodeType::Leaf(element);
                self.nodes[node] = n;
                Ok(())
            }
            NodeType::Leaf(e) => {
                let children = self.nodes.branch(node, n.aabb);
                n.ntype = NodeType::Branch(children);
                self.nodes[node] = n;
                self.rinsert(e, node, self.elements[e].translation())?;
                self.rinsert(element, node, position)?;
                Ok(())
            }
            NodeType::Branch(_) => {
                let child: NodeId = n.child_by_pos(position)?;
                self.rinsert(element, child, position)?;
                Ok(())
            }
        }
    }
}

struct Pool<T> {
    vec: Vec<T>,
    garbage: Vec<usize>,
}

impl Default for Pool<Node> {
    fn default() -> Self {
        let root = Node::default();
        let vec = vec![root];

        Pool {
            vec,
            garbage: default(),
        }
    }
}

impl<T: Translatable> Default for Pool<T> {
    fn default() -> Self {
        Pool {
            vec: default(),
            garbage: default(),
        }
    }
}

impl Index<NodeId> for Pool<Node> {
    type Output = Node;

    fn index(&self, index: NodeId) -> &Self::Output {
        &self.vec[index.0 as usize]
    }
}

impl IndexMut<NodeId> for Pool<Node> {
    fn index_mut(&mut self, index: NodeId) -> &mut Self::Output {
        &mut self.vec[index.0 as usize]
    }
}

impl<T: Translatable> Index<ElementId> for Pool<T> {
    type Output = T;

    fn index(&self, index: ElementId) -> &Self::Output {
        &self.vec[index.0 as usize]
    }
}

impl<T: Translatable> IndexMut<ElementId> for Pool<T> {
    fn index_mut(&mut self, index: ElementId) -> &mut Self::Output {
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
        self.vec.len()
    }

    fn garbage_len(&self) -> usize {
        self.garbage.len()
    }
}

impl Pool<Node> {
    fn from_aabb(aabb: Aabb3d) -> Self {
        let root = Node::from_aabb(aabb, None);
        let vec = vec![root];
        Pool {
            vec,
            garbage: default(),
        }
    }

    fn insert(&mut self, t: Node) -> NodeId {
        self._insert(t).into()
    }

    fn remove(&mut self, id: NodeId) {
        self.garbage.push(id.into());
    }

    fn branch(&mut self, parent: NodeId, aabb: Aabb3d) -> [NodeId; 8] {
        let min = aabb.min.as_uvec3();
        let max = aabb.max.as_uvec3();
        let mid = aabb.center().as_uvec3();

        from_fn(|i| self.geni_child(i, min, mid, max, parent))
    }

    fn geni_child(
        &mut self,
        i: usize,
        min: UVec3,
        mid: UVec3,
        max: UVec3,
        parent: NodeId,
    ) -> NodeId {
        let x_mask = (i & 0b1) == 1;
        let y_mask = (i & 0b10) == 1;
        let z_mask = (i & 0b100) == 1;

        let min = Vec3A::new(
            if x_mask { mid.x as f32 } else { min.x as f32 },
            if y_mask { mid.y as f32 } else { min.y as f32 },
            if z_mask { mid.z as f32 } else { min.z as f32 },
        );

        let max = Vec3A::new(
            if x_mask { max.x as f32 } else { mid.x as f32 },
            if y_mask { max.y as f32 } else { mid.y as f32 },
            if z_mask { max.z as f32 } else { mid.z as f32 },
        );

        let aabb = Aabb3d { min, max };
        let node = Node::from_aabb(aabb, Some(parent));
        self.insert(node)
    }
}

impl<T: Translatable> Pool<T> {
    fn insert(&mut self, t: T) -> ElementId {
        self._insert(t).into()
    }

    fn remove(&mut self, id: ElementId) {
        self.garbage.push(id.into());
    }
}

#[derive(Clone, Copy)]
struct Node {
    aabb: Aabb3d,
    ntype: NodeType,
    parent: Option<NodeId>,
}

impl Default for Node {
    fn default() -> Self {
        Node {
            aabb: Aabb3d {
                min: Vec3A::ZERO,
                max: Vec3A::ONE,
            },
            ntype: default(),
            parent: default(),
        }
    }
}

impl Node {
    fn from_aabb(aabb: Aabb3d, parent: Option<NodeId>) -> Self {
        Node {
            aabb,
            parent,
            ..Default::default()
        }
    }

    fn local_child_by_pos(&self, position: UVec3) -> usize {
        let center = self.aabb.center();

        let x = if position.x < center.x as u32 { 0 } else { 1 };
        let y = if position.y < center.y as u32 { 0 } else { 1 };
        let z = if position.z < center.z as u32 { 0 } else { 1 };

        x | y << 1 | z << 2
    }

    fn child_by_pos(&self, position: UVec3) -> Result<NodeId, TreeError> {
        match self.ntype {
            NodeType::Branch(children) => {
                let idx = self.local_child_by_pos(position);
                Ok(children[idx])
            }
            _ => {
                return Err(TreeError::NotBranch(format!(
                    "Attempt to treat a node {} as a Branch",
                    self.ntype
                )))
            }
        }
    }

    fn contains(&self, position: UVec3) -> bool {
        let lemin = self.aabb.min.as_uvec3().cmple(position);
        let gtmax = self.aabb.max.as_uvec3().cmpgt(position);

        lemin.all() && gtmax.all()
    }
}

#[derive(Default, Clone, Copy, PartialEq, Debug)]
enum NodeType {
    #[default]
    Empty,
    Leaf(ElementId),
    Branch([NodeId; 8]),
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
}

impl fmt::Display for TreeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TreeError::OutOfTreeBounds(info) => write!(f, "Out of tree bounds. {info}"),
            TreeError::NotBranch(info) => write!(f, "Node is not a Branch. {info}"),
        }
    }
}

#[cfg(test)]
mod tests {

    use super::*;

    struct DummyCell {
        position: UVec3,
    }

    impl Translatable for DummyCell {
        fn translation(&self) -> UVec3 {
            self.position
        }
    }

    impl DummyCell {
        fn new(position: UVec3) -> Self {
            DummyCell { position }
        }
    }

    #[test]
    fn test_insert() {
        let mut tree = Octree::from_aabb(Aabb3d::new(
            Vec3A::new(5.0, 5.0, 5.0),
            Vec3A::new(5.0, 5.0, 5.0),
        ));

        assert_eq!(tree.elements.len(), 0);
        assert_eq!(tree.elements.garbage_len(), 0);

        assert_eq!(tree.nodes.len(), 1);
        assert_eq!(tree.nodes.garbage_len(), 0);

        assert_eq!(tree.nodes[0.into()].ntype, NodeType::Empty);

        let c1 = DummyCell::new(UVec3::new(1, 1, 1));
        tree.insert(c1).unwrap();

        assert_eq!(tree.elements.len(), 1);
        assert_eq!(tree.elements.garbage_len(), 0);

        assert_eq!(tree.nodes.len(), 1);
        assert_eq!(tree.nodes.garbage_len(), 0);

        assert_eq!(tree.nodes[0.into()].ntype, NodeType::Leaf(0.into()));

        let c2 = DummyCell::new(UVec3::new(9, 9, 9));
        tree.insert(c2).unwrap();

        assert_eq!(tree.elements.len(), 2);
        assert_eq!(tree.elements.garbage_len(), 0);

        assert_eq!(tree.nodes.len(), 9);
        assert_eq!(tree.nodes.garbage_len(), 0);

        assert_eq!(tree.nodes[1.into()].ntype, NodeType::Leaf(0.into()));
        assert_eq!(tree.nodes[8.into()].ntype, NodeType::Leaf(1.into()));
        for i in 2..8 {
            assert_eq!(tree.nodes[i.into()].ntype, NodeType::Empty);
        }
    }
}
