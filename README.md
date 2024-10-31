# Oktree

[![Crates.io](https://img.shields.io/crates/v/oktree.svg)](https://crates.io/crates/oktree)
[![Docs.rs](https://docs.rs/oktree/badge.svg)](https://docs.rs/oktree)

Fast octree implementation.

![Example](/assets/example.gif)

Mainly usable with Bevy game engine for fast processing of voxel data.

### Optimizations:

- `Unsigned` arithmetics, bitwise operations.
- Tree structure is represented by flat, reusable pools. Removed data is marked only as removed.
- Few memory allocations. Heapless structures are used.
- No smart pointers (RC, RefCell e.t.c)

Compensation for the inconvenience is perfomance.

## Benchmark

| Operation        | Quantity | Time   |
| ---------------- | -------- | ------ |
| insertion        | 4096     | 1 ms   |
| removing         | 4096     | 0.3 ms |
| ray intersection | 4096     | 9 ms   |

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
```

Run bevy visual example:

```
cargo run --release --example bevy_tree
```
