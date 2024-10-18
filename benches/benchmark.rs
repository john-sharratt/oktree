use std::array::from_fn;

use criterion::{criterion_group, criterion_main, Criterion};
use oktree::{
    bounding::{Aabb, UVec3, Unsigned},
    Nodable, NodeId, Octree, Translatable,
};
use rand::Rng;

const RANGE: usize = 4096;

#[derive(Clone, Copy)]
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

fn random_points() -> [DummyCell<usize>; RANGE] {
    let mut rnd = rand::thread_rng();
    from_fn(|_| {
        let x = rnd.gen_range(0..RANGE);
        let y = rnd.gen_range(0..RANGE);
        let z = rnd.gen_range(0..RANGE);
        let position = UVec3::new(x, y, z);
        DummyCell::new(position)
    })
}

fn octree_insert(points: [DummyCell<usize>; RANGE]) {
    let mut tree = Octree::from_aabb(Aabb::new(UVec3::splat(RANGE / 2), RANGE / 2));

    for p in points {
        let _ = tree.insert(p);
    }
}

fn octree_remove(points: [DummyCell<usize>; RANGE]) {
    let mut tree = Octree::from_aabb(Aabb::new(UVec3::splat(RANGE / 2), RANGE / 2));

    for p in points {
        let _ = tree.insert(p);
    }

    for element in 0..tree.elements.len() {
        let _ = tree.remove(element.into());
    }
}

fn criterion_benchmark(c: &mut Criterion) {
    let points = random_points();

    c.bench_function("octree insert", |b| b.iter(|| octree_insert(points)));

    c.bench_function("octree remove", |b| b.iter(|| octree_remove(points)));
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
