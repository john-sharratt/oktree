//! [![Crates.io](https://img.shields.io/crates/v/oktree.svg)](https://crates.io/crates/oktree)
//! [![Docs.rs](https://docs.rs/oktree/badge.svg)](https://docs.rs/oktree)
//!
//! Fast [`octree`](tree::Octree) implementation.
//!
//! ![Example](https://raw.githubusercontent.com/exor2008/oktree/main/assets/example.gif)
//!
//! Mainly usable with Bevy game engine for fast processing of voxel data.
//!
//! Bevy integration feature if enabled by default and can be disabled by:
//!
//! ```
//! [dependencies]
//! oktree = { version = "0.1.0", default-features = false }
//! ```
//!
//! Intersection methods are not available without this feature.
//!
//! ### Optimizations:
//!
//! - `Unsigned` arithmetics, bitwise operations.
//! - Tree structure is represented by flat, reusable [`Pool`](`pool::Pool`). Removed data is marked only.
//! - Few memory allocations. [`heapless`](https://docs.rs/heapless/) structures are used.
//! - No smart pointers (RC, RefCell e.t.c)
//!
//! Compensation for the inconvenience is perfomance.
//!
//! ## Benchmark
//!
//! | Operation        | Quantity                   | Time    |
//! | ---------------- | -------------------------- | ------- |
//! | insertion        | 65536 cells                | 25 ms   |
//! | removing         | 65536 cells                | 11.2 ms |
//! | ray intersection | 4096 rays with 65536 cells | 33 ms   |
//!
//! Run benchmark:
//!
//! ```
//! cargo bench
//! ```
//!
//! ## Example
//!
//! You have to specify the type for the internal [`Octree`](`tree::Octree`) structure.
//!
//! It must be any [`Unsigned`](`num::Unsigned`) type (`u8`, `u16`, `u32`, `u64`, `u128` or `usize`).
//!
//! Implement [`Position`] for the handled type, so that it can return it's spatial coordinates.
//!
//! ```rust
//! use bevy::math::{bounding::RayCast3d, Dir3, Vec3};
//! use oktree::prelude::*;
//!
//! fn main() -> Result<(), TreeError> {
//!     let aabb = Aabb::new(TUVec3::splat(16), 16u8);
//!     let mut tree = Octree::from_aabb_with_capacity(aabb, 10);
//!
//!     let c1 = DummyCell::new(TUVec3::splat(1u8));
//!     let c2 = DummyCell::new(TUVec3::splat(8u8));
//!
//!     tree.insert(c1)?;
//!     tree.insert(c2)?;
//!
//!     let ray = RayCast3d::new(Vec3::new(1.5, 7.0, 1.9), Dir3::NEG_Y, 100.0);
//!     assert_eq!(
//!         tree.ray_cast(&ray),
//!         HitResult {
//!             element: Some(ElementId(0)),
//!             distance: 5.0
//!         }
//!     );
//!
//!     assert_eq!(tree.remove(ElementId(0)), Ok(()));
//!     assert_eq!(
//!         tree.ray_cast(&ray),
//!         HitResult {
//!             element: None,
//!             distance: 0.0
//!         }
//!     );
//!     Ok(())
//! }
//!
//! struct DummyCell {
//!     position: TUVec3<u8>,
//! }
//!
//! impl Position for DummyCell {
//!     type U = u8;
//!     fn position(&self) -> TUVec3<u8> {
//!         self.position
//!     }
//! }
//!
//! impl DummyCell {
//!     fn new(position: TUVec3<u8>) -> Self {
//!         DummyCell { position }
//!     }
//! }
//! ```
//!
//! Run bevy visual example:
//!
//! ```
//! cargo run --release --example bevy_tree
//! ```

#![allow(dead_code)]
#![feature(strict_overflow_ops)]
#![feature(trait_alias)]

#[cfg(feature = "bevy")]
pub mod bevy_integration;
pub mod bounding;
pub mod node;
pub mod pool;
pub mod prelude;
pub mod tree;

use bounding::{TUVec3, Unsigned};
use std::{
    error::Error,
    fmt::{self},
};

// Implement on stored type to inform a tree
// about object's spatial coordinates.
pub trait Position {
    type U: Unsigned;

    fn position(&self) -> TUVec3<Self::U>;
}

/// Index [`tree.nodes`](pool::Pool) with it.
///
/// ```no_run
/// let node: Node<u16> = tree.nodes[NodeId(0)]
/// ```
#[derive(Default, Clone, Copy, PartialEq, Debug)]
pub struct NodeId(pub u32);

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

/// Index [`tree.elements`](pool::Pool) with it.
/// Stored type element will be returned.
///
/// ```no_run
/// let element = tree.elements[ElementId(0)]
/// ```
#[derive(Default, Clone, Copy, PartialEq, Debug)]
pub struct ElementId(pub u32);

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

/// Enum of all possible errors of the octree's operations.
#[derive(Debug, PartialEq)]
pub enum TreeError {
    /// Object is out of bounds of tree's [`Aabb`](bounding::Aabb).
    OutOfTreeBounds(String),

    /// Attempt to treat a [`Node`](node::Node) of different type
    /// as a [`Branch`](node::NodeType::Branch).
    NotBranch(String),

    /// Attempt to treat a [`Node`](node::Node) of different type
    /// as a [`Leaf`](node::NodeType::Leaf).
    NotLeaf(String),

    /// Only a [`Branch`](node::NodeType::Branch) [`Node`](node::Node) can be collapsed.
    CollapseNonEmpty(String),

    /// Attempt to split a [`Node`](node::Node) with size of 1.
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
    use bounding::Aabb;
    use node::{Branch, NodeType};
    use rand::Rng;
    use std::array::from_fn;
    use tree::Octree;

    const RANGE: usize = 65536;

    #[derive(Debug, PartialEq)]
    struct DummyCell<U: Unsigned> {
        position: TUVec3<U>,
        node: NodeId,
    }

    impl<U: Unsigned> Position for DummyCell<U> {
        type U = U;
        fn position(&self) -> TUVec3<U> {
            self.position
        }
    }

    impl<U: Unsigned> DummyCell<U> {
        fn new(position: TUVec3<U>) -> Self {
            DummyCell {
                position,
                node: Default::default(),
            }
        }
    }

    #[test]
    fn test_insert() {
        let mut tree = Octree::from_aabb(Aabb::new(TUVec3::new(4, 4, 4), 4));

        assert_eq!(tree.elements.len(), 0);
        assert_eq!(tree.elements.garbage_len(), 0);

        assert_eq!(tree.nodes.len(), 1);
        assert_eq!(tree.nodes.garbage_len(), 0);

        assert_eq!(tree.map.len(), 0);
        assert_eq!(tree.map.garbage_len(), 0);

        assert_eq!(tree.nodes[0.into()].ntype, NodeType::Empty);
        assert_eq!(tree.nodes[0.into()].parent, None);

        let c1 = DummyCell::new(TUVec3::new(1u8, 1, 1));
        assert_eq!(tree.insert(c1), Ok(ElementId(0)));

        assert_eq!(tree.elements.len(), 1);
        assert_eq!(tree.elements.garbage_len(), 0);

        assert_eq!(tree.nodes.len(), 1);
        assert_eq!(tree.nodes.garbage_len(), 0);

        assert_eq!(tree.map.len(), 1);
        assert_eq!(tree.map.garbage_len(), 0);
        assert_eq!(tree.map[0.into()], 0.into());

        assert_eq!(tree.nodes[0.into()].ntype, NodeType::Leaf(0.into()));
        assert_eq!(tree.nodes[0.into()].parent, None);

        let c2 = DummyCell::new(TUVec3::new(7, 7, 7));
        assert_eq!(tree.insert(c2), Ok(ElementId(1)));

        assert_eq!(tree.elements.len(), 2);
        assert_eq!(tree.elements.garbage_len(), 0);

        assert_eq!(tree.nodes.len(), 9);
        assert_eq!(tree.nodes.garbage_len(), 0);

        assert_eq!(tree.map.len(), 2);
        assert_eq!(tree.map.garbage_len(), 0);
        assert_eq!(tree.map[0.into()], 1.into());
        assert_eq!(tree.map[1.into()], 8.into());

        assert_eq!(tree.nodes[0.into()].parent, None);

        let children = from_fn(|i| NodeId(i as u32 + 1));
        assert_eq!(
            tree.nodes[0.into()].ntype,
            NodeType::Branch(Branch::from_filled(children, 2))
        );

        assert_eq!(tree.nodes[1.into()].ntype, NodeType::Leaf(0.into()));
        assert_eq!(tree.nodes[1.into()].parent, Some(0.into()));
        assert_eq!(tree.nodes[8.into()].ntype, NodeType::Leaf(1.into()));
        assert_eq!(tree.nodes[8.into()].parent, Some(0.into()));
        for i in 2..8 {
            assert_eq!(tree.nodes[i.into()].ntype, NodeType::Empty);
        }

        assert_eq!(tree.map.len(), 2);
        assert_eq!(tree.map.garbage_len(), 0);
        assert_eq!(tree.map[0.into()], 1.into());
        assert_eq!(tree.map[1.into()], 8.into());
    }

    #[test]
    fn test_remove() {
        let mut tree = Octree::from_aabb(Aabb::new(TUVec3::new(8u16, 8, 8), 8));

        let c1 = DummyCell::new(TUVec3::new(1, 1, 1));
        assert_eq!(tree.insert(c1), Ok(ElementId(0)));
        assert_eq!(tree.map[0.into()], 0.into());
        let c2 = DummyCell::new(TUVec3::new(2, 2, 2));
        assert_eq!(tree.insert(c2), Ok(ElementId(1)));
        assert_eq!(tree.map[0.into()], 17.into());
        assert_eq!(tree.map[1.into()], 24.into());
        assert_eq!(tree.nodes[17.into()].ntype, NodeType::Leaf(0.into()));

        assert_eq!(tree.nodes.len(), 25);

        let c2r = DummyCell::new(TUVec3::new(1, 1, 1));
        assert_eq!(
            tree.insert(c2r),
            Err(TreeError::SplitUnit(
                "Attempt to insert element into a leaf with size 1".into()
            ))
        );

        assert_eq!(tree.nodes.len(), 33);
        assert_eq!(tree.elements.len(), 2);
        assert_eq!(tree.map.len(), 2);

        tree.remove(0.into()).unwrap();

        assert_eq!(tree.elements.len(), 1);
        assert_eq!(tree.map.len(), 1);
        assert_eq!(tree.nodes.len(), 17);

        tree.remove(1.into()).unwrap();

        assert_eq!(tree.elements.len(), 0);
        assert_eq!(tree.map.len(), 0);
        assert_eq!(tree.nodes.len(), 1);

        assert_eq!(tree.nodes[0.into()].ntype, NodeType::Empty)
    }

    #[test]
    fn test_insert_remove() {
        let mut tree = Octree::from_aabb(Aabb::new(TUVec3::new(4u8, 4, 4), 4));

        let c1 = DummyCell::new(TUVec3::new(1, 1, 1));
        assert_eq!(tree.insert(c1), Ok(ElementId(0)));

        let c2 = DummyCell::new(TUVec3::new(2, 2, 1));
        assert_eq!(tree.insert(c2), Ok(ElementId(1)));

        let c3 = DummyCell::new(TUVec3::new(6, 6, 1));
        assert_eq!(tree.insert(c3), Ok(ElementId(2)));

        let c4 = DummyCell::new(TUVec3::new(7, 7, 1));
        assert_eq!(tree.insert(c4), Ok(ElementId(3)));

        let c5 = DummyCell::new(TUVec3::new(6, 7, 1));
        assert_eq!(tree.insert(c5), Ok(ElementId(4)));

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
        assert_eq!(tree.map.len(), 0);
    }

    fn random_point() -> DummyCell<usize> {
        let mut rnd = rand::thread_rng();

        let x = rnd.gen_range(0..=RANGE);
        let y = rnd.gen_range(0..=RANGE);
        let z = rnd.gen_range(0..=RANGE);
        let position = TUVec3::new(x, y, z);
        DummyCell::new(position)
    }

    #[test]
    fn test_65536() {
        let mut tree = Octree::from_aabb(Aabb::new(TUVec3::splat(RANGE / 2), RANGE / 2));

        for _ in 0..RANGE {
            let p = random_point();
            let _ = tree.insert(p);
        }

        assert!(tree.elements.len() > (RANGE as f32 * 0.98) as usize);
        assert!(tree.map.len() > (RANGE as f32 * 0.98) as usize);

        for element in 0..tree.elements.len() {
            if let Err(err) = tree.remove(element.into()) {
                println!("{err} || {}", tree.elements[element.into()].position());
            }
        }

        assert_eq!(tree.elements.len(), 0);
        assert_eq!(tree.map.len(), 0);
        assert_eq!(tree.nodes.len(), 1);
    }

    #[test]
    fn test_iterator() {
        let mut tree = Octree::from_aabb(Aabb::new(TUVec3::splat(16), 16));

        for i in 0..16u32 {
            assert_eq!(
                tree.insert(DummyCell::new(TUVec3::splat(i))),
                Ok(ElementId(i))
            );
            assert_eq!(tree.elements.len(), (i + 1) as usize);
            assert_eq!(tree.elements.vec.len(), (i + 1) as usize);
            assert_eq!(tree.elements.garbage_len(), 0);
        }

        for i in 0..16u32 {
            assert_eq!(
                tree.elements.iter().next().unwrap().position,
                TUVec3::splat(i)
            );

            assert_eq!(tree.remove(ElementId(i)), Ok(()));
            assert_eq!(tree.elements.len(), (15 - i) as usize);
            assert_eq!(tree.elements.vec.len(), 16);
            assert_eq!(tree.elements.garbage_len(), (i + 1) as usize);
        }

        for i in 0..16u32 {
            assert_eq!(
                tree.insert(DummyCell::new(TUVec3::splat(i))),
                Ok(ElementId(15 - i))
            );
            assert_eq!(tree.elements.len(), (i + 1) as usize);
            assert_eq!(tree.elements.vec.len(), 16);
            assert_eq!(tree.elements.garbage_len(), (15 - i) as usize);
        }

        for i in 0..16u32 {
            assert_eq!(
                tree.elements.iter().next().unwrap().position,
                TUVec3::splat(15 - i)
            );

            assert_eq!(tree.remove(ElementId(i)), Ok(()));
            assert_eq!(tree.elements.len(), (15 - i) as usize);
            assert_eq!(tree.elements.vec.len(), 16);
            assert_eq!(tree.elements.garbage_len(), (i + 1) as usize);
        }
    }

    #[test]
    fn test_constructors() {
        let aabb = Aabb::default();

        let tree: Octree<u8, DummyCell<u8>> = Octree::from_aabb(aabb);
        assert_eq!(tree.elements.len(), 0);
        assert_eq!(tree.elements.garbage_len(), 0);
        assert_eq!(tree.nodes.len(), 1);
        assert_eq!(tree.nodes.garbage_len(), 0);
        assert_eq!(tree.nodes[0.into()].aabb, aabb);

        let tree: Octree<u8, DummyCell<u8>> = Octree::with_capacity(100);
        assert_eq!(tree.elements.len(), 0);
        assert_eq!(tree.elements.garbage_len(), 0);
        assert_eq!(tree.elements.vec.capacity(), 100);
        assert_eq!(tree.nodes.len(), 1);
        assert_eq!(tree.nodes.garbage_len(), 0);
        assert_eq!(tree.nodes.vec.capacity(), 100);
        assert_eq!(tree.nodes[0.into()].aabb, Aabb::default());

        let tree: Octree<u8, DummyCell<u8>> = Octree::from_aabb_with_capacity(aabb, 50);
        assert_eq!(tree.elements.len(), 0);
        assert_eq!(tree.elements.garbage_len(), 0);
        assert_eq!(tree.elements.vec.capacity(), 50);
        assert_eq!(tree.nodes.len(), 1);
        assert_eq!(tree.nodes.garbage_len(), 0);
        assert_eq!(tree.nodes.vec.capacity(), 50);
        assert_eq!(tree.nodes[0.into()].aabb, aabb);
    }

    #[test]
    fn test_to_vec() {
        let mut tree = Octree::from_aabb(Aabb::new(TUVec3::splat(16), 16));
        assert_eq!(
            tree.insert(DummyCell::new(TUVec3::splat(1u8))),
            Ok(ElementId(0))
        );
        assert_eq!(
            tree.insert(DummyCell::new(TUVec3::splat(2u8))),
            Ok(ElementId(1))
        );
        assert_eq!(
            tree.insert(DummyCell::new(TUVec3::splat(3u8))),
            Ok(ElementId(2))
        );

        assert_eq!(tree.remove(1.into()), Ok(()));

        assert_eq!(
            tree.to_vec(),
            vec![
                DummyCell::new(TUVec3::splat(1u8)),
                DummyCell::new(TUVec3::splat(3u8))
            ]
        )
    }
}
