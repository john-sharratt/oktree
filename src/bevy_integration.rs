//! [Bevy](https://docs.rs/bevy/) game engine integrations.
//!
//! Adds the [Bevy](https://docs.rs/bevy/) game engine as a dependency.
//!
//! ### Intersections:
//! - [ray](RayCast3d) [intersection](Octree::ray_cast)
//!
//! ```no_run
//! let ray = RayCast3d::new(Vec3A::new(7.0, 5.9, 1.01), Dir3A::NEG_X, 10.0);
//! assert_eq!(
//!   tree.ray_cast(&ray),
//!   HitResult {
//!     element: Some(1.into()),
//!     distance: 5.0
//!   }
//! );
//! ```

use bevy::math::{
    bounding::{Aabb3d, BoundingSphere, IntersectsVolume, RayCast3d},
    Vec3, Vec3A,
};
use num::cast;

use crate::{
    bounding::{Aabb, TUVec3, Unsigned},
    node::{Branch, NodeType},
    tree::Octree,
    ElementId, NodeId, Position,
};

impl<U, T> Octree<U, T>
where
    U: Unsigned,
    T: Position<U = U>,
{
    /// Intersects an [Octree] with the [RayCast3d].
    ///
    /// Returns a [HitResult] with [ElementId] and the doistance to
    /// the intersection if any.
    pub fn ray_cast(&self, ray: &RayCast3d) -> HitResult {
        let mut hit = HitResult::default();
        self.recursive_ray_cast(self.root, ray, &mut hit);
        hit
    }

    fn recursive_ray_cast(&self, node: NodeId, ray: &RayCast3d, hit: &mut HitResult) {
        let n = &self.nodes[node];

        if n.ntype == NodeType::Empty {
            return;
        }

        let aabb: Aabb3d = n.aabb.into();
        if ray.intersects(&aabb) {
            match n.ntype {
                NodeType::Branch(Branch { children, .. }) => {
                    children.map(|child| self.recursive_ray_cast(child, ray, hit));
                }

                NodeType::Leaf(element) => {
                    let min = self.elements[element].position();
                    let max = TUVec3::new(
                        min.x + cast(1).unwrap(),
                        min.y + cast(1).unwrap(),
                        min.z + cast(1).unwrap(),
                    );
                    let aabb = Aabb3d {
                        min: min.into(),
                        max: max.into(),
                    };

                    if let Some(dist) = ray.aabb_intersection_at(&aabb) {
                        match hit.element {
                            Some(_) => {
                                if hit.distance > dist {
                                    hit.element = Some(element);
                                    hit.distance = dist;
                                }
                            }
                            None => {
                                hit.element = Some(element);
                                hit.distance = dist;
                            }
                        }
                    }
                }

                NodeType::Empty => (),
            }
        }
    }
}

/// Intersection result.
///
/// Contains `Some(`[ElementId]`)` in case of intersection,
/// [None] otherwise.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct HitResult {
    pub element: Option<ElementId>,
    pub distance: f32,
}

impl<U: Unsigned> From<Aabb<U>> for Aabb3d {
    fn from(value: Aabb<U>) -> Self {
        Aabb3d {
            min: value.min.into(),
            max: value.max.into(),
        }
    }
}

impl<U: Unsigned> From<TUVec3<U>> for Vec3A {
    fn from(value: TUVec3<U>) -> Self {
        Vec3A::new(
            cast(value.x).unwrap(),
            cast(value.y).unwrap(),
            cast(value.z).unwrap(),
        )
    }
}

impl<U: Unsigned> From<TUVec3<U>> for Vec3 {
    fn from(value: TUVec3<U>) -> Self {
        Vec3::new(
            cast(value.x).unwrap(),
            cast(value.y).unwrap(),
            cast(value.z).unwrap(),
        )
    }
}

impl<U, T> IntersectsVolume<Aabb3d> for Octree<U, T>
where
    U: Unsigned,
    T: Position<U = U>,
{
    /// Check if a [Aabb3d] volume intersects with the [Octree] root node.
    fn intersects(&self, volume: &Aabb3d) -> bool {
        let aabb: Aabb3d = self.nodes[self.root].aabb.into();
        volume.intersects(&aabb)
    }
}

impl<U, T> IntersectsVolume<BoundingSphere> for Octree<U, T>
where
    U: Unsigned,
    T: Position<U = U>,
{
    /// Check if a [BoundingSphere] volume intersects with the [Octree] root node.
    fn intersects(&self, volume: &BoundingSphere) -> bool {
        let aabb: Aabb3d = self.nodes[self.root].aabb.into();
        volume.intersects(&aabb)
    }
}

#[cfg(test)]
mod tests {

    use bevy::math::Dir3A;

    use super::*;

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
    fn test_ray_intersection() {
        let aabb = Aabb::new(TUVec3::new(4u16, 4, 4), 4);
        assert!(aabb.is_ok());
        let mut tree = Octree::from_aabb(aabb.unwrap());

        let c1 = DummyCell::new(TUVec3::new(3, 1, 1));
        assert_eq!(tree.insert(c1), Ok(ElementId(0)));

        let c2 = DummyCell::new(TUVec3::new(1, 5, 1));
        assert_eq!(tree.insert(c2), Ok(ElementId(1)));

        // hit 2nd
        let ray = RayCast3d::new(Vec3A::new(1.5, 1.5, 1.5), Dir3A::Y, 10.0);
        assert_eq!(
            tree.ray_cast(&ray),
            HitResult {
                element: Some(1.into()),
                distance: 3.5
            }
        );

        // miss
        let ray = RayCast3d::new(Vec3A::ZERO, Dir3A::Y, 10.0);
        assert_eq!(
            tree.ray_cast(&ray),
            HitResult {
                element: None,
                distance: 0.0
            }
        );

        // hit 1st
        let ray = RayCast3d::new(Vec3A::new(0.0, 1.05, 1.05), Dir3A::X, 10.0);
        assert_eq!(
            tree.ray_cast(&ray),
            HitResult {
                element: Some(0.into()),
                distance: 3.0
            }
        );

        // miss
        let ray = RayCast3d::new(Vec3A::new(40.0, 40.0, 40.0), Dir3A::X, 10.0);
        assert_eq!(
            tree.ray_cast(&ray),
            HitResult {
                element: None,
                distance: 0.0
            }
        );

        // miss
        let ray = RayCast3d::new(Vec3A::new(7.0, 5.9, 1.01), Dir3A::NEG_X, 10.0);
        assert_eq!(
            tree.ray_cast(&ray),
            HitResult {
                element: Some(1.into()),
                distance: 5.0
            }
        );

        // miss
        let ray = RayCast3d::new(Vec3A::new(1.01, 1.01, 1.01), Dir3A::NEG_X, 10.0);
        assert_eq!(
            tree.ray_cast(&ray),
            HitResult {
                element: None,
                distance: 0.0
            }
        );

        // hit 1st
        let ray = RayCast3d::new(Vec3A::new(3.05, 10.0, 1.05), Dir3A::NEG_Y, 10.0);
        assert_eq!(
            tree.ray_cast(&ray),
            HitResult {
                element: Some(0.into()),
                distance: 8.0
            }
        );
    }
}
