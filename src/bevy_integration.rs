use bevy::math::{
    bounding::{Aabb3d, IntersectsVolume, RayCast3d},
    Vec3, Vec3A,
};
use num::cast;

use crate::{
    bounding::{Aabb, UVec3, Unsigned},
    Branch, ElementId, Nodable, NodeId, NodeType, Octree, Translatable,
};

impl<U, T> Octree<U, T>
where
    U: Unsigned,
    T: Translatable<U = U> + Nodable,
{
    pub fn ray_cast(&self, ray: &RayCast3d) -> Option<ElementId> {
        let mut hit = HitResult::default();
        self.recursive_ray_cast(self.root, &ray, &mut hit);
        hit.element
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
                    let min = self.elements[element].translation();
                    let max = UVec3::new(
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
                                }
                            }
                            None => {
                                hit.element = Some(element);
                                hit.distance = dist
                            }
                        }
                    }
                }

                NodeType::Empty => (),
            }
        }
    }
}

#[derive(Default, Clone, Copy)]
struct HitResult {
    element: Option<ElementId>,
    distance: f32,
}

impl<U: Unsigned> From<Aabb<U>> for Aabb3d {
    fn from(value: Aabb<U>) -> Self {
        Aabb3d {
            min: value.min.into(),
            max: value.max.into(),
        }
    }
}

impl<U: Unsigned> From<UVec3<U>> for Vec3A {
    fn from(value: UVec3<U>) -> Self {
        Vec3A::new(
            cast(value.x).unwrap(),
            cast(value.y).unwrap(),
            cast(value.z).unwrap(),
        )
    }
}

impl<U: Unsigned> From<UVec3<U>> for Vec3 {
    fn from(value: UVec3<U>) -> Self {
        Vec3::new(
            cast(value.x).unwrap(),
            cast(value.y).unwrap(),
            cast(value.z).unwrap(),
        )
    }
}

#[cfg(test)]
mod tests {

    use bevy::math::Dir3A;

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
    fn test_ray_intersection() {
        let mut tree = Octree::from_aabb(Aabb::new(UVec3::new(4u16, 4, 4), 4));

        let c1 = DummyCell::new(UVec3::new(1, 1, 1));
        assert_eq!(tree.insert(c1), Ok(()));

        let c2 = DummyCell::new(UVec3::new(1, 5, 1));
        assert_eq!(tree.insert(c2), Ok(()));

        let ray = RayCast3d::new(Vec3A::new(1.5, 1.5, 1.5), Dir3A::Y, 10.0);
        assert_eq!(tree.ray_cast(&ray), Some(0.into()));

        let ray = RayCast3d::new(Vec3A::ZERO, Dir3A::Y, 10.0);
        assert_eq!(tree.ray_cast(&ray), None);

        let ray = RayCast3d::new(Vec3A::ONE, Dir3A::Y, 10.0);
        assert_eq!(tree.ray_cast(&ray), Some(0.into()));

        let ray = RayCast3d::new(Vec3A::new(40.0, 40.0, 40.0), Dir3A::X, 10.0);
        assert_eq!(tree.ray_cast(&ray), None);

        let ray = RayCast3d::new(Vec3A::new(1.0, 1.0, 1.0), Dir3A::NEG_X, 10.0);
        assert_eq!(tree.ray_cast(&ray), Some(0.into()));

        let ray = RayCast3d::new(Vec3A::new(1.0, 5.9, 1.0), Dir3A::NEG_X, 10.0);
        assert_eq!(tree.ray_cast(&ray), Some(1.into()));
    }
}
