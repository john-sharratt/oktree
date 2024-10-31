use bevy::math::{bounding::RayCast3d, Dir3, Vec3};
use oktree::prelude::*;

fn main() -> Result<(), TreeError> {
    let aabb = Aabb::new(TUVec3::splat(16), 16u8);
    let mut tree = Octree::from_aabb_with_capacity(aabb, 10);

    let c1 = DummyCell::new(TUVec3::splat(1u8));
    let c2 = DummyCell::new(TUVec3::splat(8u8));

    tree.insert(c1)?;
    tree.insert(c2)?;

    let ray = RayCast3d::new(Vec3::new(1.5, 7.0, 1.9), Dir3::NEG_Y, 100.0);
    assert_eq!(
        tree.ray_cast(&ray),
        HitResult {
            element: Some(ElementId(0)),
            distance: 5.0
        }
    );

    assert_eq!(tree.remove(ElementId(0)), Ok(()));
    assert_eq!(
        tree.ray_cast(&ray),
        HitResult {
            element: None,
            distance: 0.0
        }
    );
    Ok(())
}

struct DummyCell<U: Unsigned> {
    position: TUVec3<U>,
}

impl<U: Unsigned> Position for DummyCell<U> {
    type U = U;
    fn position(&self) -> TUVec3<U> {
        self.position
    }
}

impl<U: Unsigned> DummyCell<U> {
    fn new(position: TUVec3<U>) -> Self {
        DummyCell { position }
    }
}
