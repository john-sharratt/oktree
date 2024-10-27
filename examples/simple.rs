use bevy::math::{bounding::RayCast3d, Dir3, Vec3};
use oktree::{prelude::*, ElementId};

fn main() -> Result<(), TreeError> {
    let aabb = Aabb::new(TUVec3::splat(16), 16u8);
    let mut tree = Octree::from_aabb_with_capacity(aabb, 10);

    let c1 = DummyCell::new(TUVec3::splat(1u8));
    let c2 = DummyCell::new(TUVec3::splat(8u8));

    tree.insert(c1)?;
    tree.insert(c2)?;

    let ray = RayCast3d::new(Vec3::splat(1.5), Dir3::X, 100.0);
    assert_eq!(tree.ray_cast(&ray), Some(ElementId(0)));

    assert_eq!(tree.remove(ElementId(0)), Ok(()));
    assert_eq!(tree.ray_cast(&ray), None);
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
