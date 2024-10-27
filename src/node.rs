use core::fmt;

use crate::{
    bounding::{Aabb, UVec3, Unsigned},
    ElementId, NodeId, TreeError,
};

#[derive(Clone, Copy)]
pub struct Node<U: Unsigned> {
    pub aabb: Aabb<U>,
    pub ntype: NodeType,
    pub parent: Option<NodeId>,
}

impl<U: Unsigned> Default for Node<U> {
    fn default() -> Self {
        Node {
            aabb: Aabb::<U>::default(),
            ntype: Default::default(),
            parent: Default::default(),
        }
    }
}

impl<U: Unsigned> Node<U> {
    pub(crate) fn from_aabb(aabb: Aabb<U>, parent: Option<NodeId>) -> Self {
        Node {
            aabb,
            parent,
            ..Default::default()
        }
    }

    pub fn fullness(&self) -> Result<u8, TreeError> {
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
pub enum NodeType {
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
pub struct Branch {
    pub children: [NodeId; 8],
    pub filled: u8,
}

impl Branch {
    pub(crate) fn new(children: [NodeId; 8]) -> Self {
        Branch {
            children,
            ..Default::default()
        }
    }

    pub(crate) fn from_filled(children: [NodeId; 8], filled: u8) -> Self {
        Branch { children, filled }
    }

    pub(crate) fn increment(&mut self) {
        self.filled = self.filled.strict_add(1);
        debug_assert!(self.filled <= 8);
    }

    pub(crate) fn decrement(&mut self) {
        self.filled = self.filled.strict_sub(1);
    }

    pub fn find_child<U: Unsigned>(
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
