# Oktree

[![Crates.io](https://img.shields.io/crates/v/oktree.svg)](https://crates.io/crates/oktree)
[![Docs.rs](https://docs.rs/oktree/badge.svg)](https://docs.rs/oktree)

Fast octree implementation.

![Example](/assets/example.gif)

Mainly usable with Bevy game engine for fast processing of voxel data.

Bevy integration feature if enabled by default and can be disabled by:

```
[dependencies]
oktree = { version = "0.1.0", default-features = false }
```

Intersection methods are not available without this feature.

### Optimizations:

- `Unsigned` arithmetics, bitwise operations.
- Tree structure is represented by flat, reusable pools. Removed data is marked only.
- Few memory allocations. Heapless structures are used.
- No smart pointers (RC, RefCell e.t.c)

Compensation for the inconvenience is perfomance.

## Benchmark

| Operation        | Quantity                   | Time    |
| ---------------- | -------------------------- | ------- |
| insertion        | 65536 cells                | 25 ms   |
| removing         | 65536 cells                | 11.2 ms |
| ray intersection | 4096 rays with 65536 cells | 33 ms   |

Run benchmark:

```
cargo bench
```

## Example

You have to specify the type for the internal tree structure.

It must be any `Unsigned` type (`u8`, `u16`, `u32`, `u64`, `u128` or `usize`).

Implement `Position` for the handled type, so that it can return it's spatial coordinates.

```rust
use bevy::math::{bounding::RayCast3d, Dir3, Vec3};
use oktree::prelude::*;

fn main() -> Result<(), TreeError> {
    let aabb = Aabb::new(TUVec3::splat(16), 16u8);
    let mut tree = Octree::from_aabb_with_capacity(aabb, 10);

    let c1 = DummyCell::new(TUVec3::splat(1u8));
    let c2 = DummyCell::new(TUVec3::splat(8u8));

    let c1_id = tree.insert(c1)?;
    let c2_id = tree.insert(c2)?;

    // Searching by point
    assert_eq!(tree.find(TUVec3::new(1, 1, 1)), Some(c1_id));
    assert_eq!(tree.find(TUVec3::new(8, 8, 8)), Some(c2_id));
    assert_eq!(tree.find(TUVec3::new(1, 2, 8)), None);
    assert_eq!(tree.find(TUVec3::splat(100)), None);

    // Searching for the ray intersection
    let ray = RayCast3d::new(Vec3::new(1.5, 7.0, 1.9), Dir3::NEG_Y, 100.0);

    // Hit!
    assert_eq!(
        tree.ray_cast(&ray),
        HitResult {
            element: Some(ElementId(0)),
            distance: 5.0
        }
    );

    assert_eq!(tree.remove(ElementId(0)), Ok(()));

    // Miss!
    assert_eq!(
        tree.ray_cast(&ray),
        HitResult {
            element: None,
            distance: 0.0
        }
    );
    Ok(())
}

struct DummyCell {
    position: TUVec3<u8>,
}

impl Position for DummyCell {
    type U = u8;
    fn position(&self) -> TUVec3<u8> {
        self.position
    }
}

impl DummyCell {
    fn new(position: TUVec3<u8>) -> Self {
        DummyCell { position }
    }
}
```

Run bevy visual example:

```
cargo run --release --example bevy_tree
```
