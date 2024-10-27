use std::{array::from_fn, time::Duration};

use bevy::math::{bounding::RayCast3d, Dir3A, Vec3A};
use criterion::{criterion_group, criterion_main, Criterion};
use oktree::{
    bounding::{Aabb, UVec3, Unsigned},
    tree::Octree,
    Position,
};
use rand::Rng;

const RANGE: usize = 4096;

#[derive(Clone, Copy)]
struct DummyCell<U: Unsigned> {
    position: UVec3<U>,
}

impl<U: Unsigned> Position for DummyCell<U> {
    type U = U;
    fn position(&self) -> UVec3<U> {
        self.position
    }
}

impl<U: Unsigned> DummyCell<U> {
    fn new(position: UVec3<U>) -> Self {
        DummyCell { position }
    }
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

fn random_rays() -> [RayCast3d; RANGE] {
    let mut rnd = rand::thread_rng();
    from_fn(|_| {
        let x = rnd.gen_range(0.0..=RANGE as f32);
        let y = rnd.gen_range(0.0..=RANGE as f32);
        let z = rnd.gen_range(0.0..=RANGE as f32);
        let origin = Vec3A::new(x, y, z);

        let x_dir = rnd.gen_range(0.0..=1.0);
        let y_dir = rnd.gen_range(0.0..=1.0);
        let z_dir = rnd.gen_range(0.0..=1.0);
        let direction = Vec3A::new(x_dir, y_dir, z_dir);
        let direction = Dir3A::new(direction).unwrap();

        RayCast3d::new(origin, direction, RANGE as f32)
    })
}

fn octree_insert(points: &[DummyCell<usize>; RANGE]) {
    let mut tree =
        Octree::from_aabb_with_capacity(Aabb::new(UVec3::splat(RANGE / 2), RANGE / 2), RANGE);

    for p in points {
        let _ = tree.insert(*p);
    }
}

fn octree_remove(points: &[DummyCell<usize>; RANGE]) {
    let mut tree =
        Octree::from_aabb_with_capacity(Aabb::new(UVec3::splat(RANGE / 2), RANGE / 2), RANGE);

    for p in points {
        let _ = tree.insert(*p);
    }

    for element in 0..tree.elements.len() {
        let _ = tree.remove(element.into());
    }
}

fn octree_intersection(points: &[DummyCell<usize>; RANGE], rays: &[RayCast3d; RANGE]) {
    let mut tree =
        Octree::from_aabb_with_capacity(Aabb::new(UVec3::splat(RANGE / 2), RANGE / 2), RANGE);

    for p in points {
        let _ = tree.insert(*p);
    }

    for ray in rays {
        let _ = tree.ray_cast(ray);
    }
}

fn criterion_benchmark(c: &mut Criterion) {
    let mut group = c.benchmark_group("main");
    group.measurement_time(Duration::from_secs(10));
    let points = random_points();
    let rays = random_rays();

    group.bench_function("octree insert", |b| b.iter(|| octree_insert(&points)));

    group.bench_function("octree remove", |b| b.iter(|| octree_remove(&points)));

    group.bench_function("octree intersection", |b| {
        b.iter(|| octree_intersection(&points, &rays))
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
