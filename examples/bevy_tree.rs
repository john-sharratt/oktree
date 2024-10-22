use std::time::Duration;

use bevy::prelude::*;
use oktree::{
    bounding::{Aabb, UVec3 as TUVec3, Unsigned},
    Nodable, NodeId, NodeType, Octree, Translatable,
};
use rand::Rng;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_systems(Startup, setup)
        .add_systems(FixedUpdate, spawn_points)
        .add_systems(Update, (draw_nodes, draw_elements))
        .run();
}

fn setup(mut commands: Commands) {
    commands.insert_resource(Tree(Octree::from_aabb(Aabb::new(TUVec3::splat(32), 32))));
    commands.insert_resource(SpawnTimer {
        timer: Timer::new(Duration::from_secs(1), TimerMode::Repeating),
    });

    let position = Transform::from_xyz(-100.0, 0.0, 32.0).looking_at(Vec3::splat(32.0), Vec3::Z);
    commands.spawn((
        Camera3dBundle {
            transform: position,
            ..default()
        },
        Camera,
    ));
}

fn draw_nodes(mut gizmos: Gizmos, tree: Res<Tree>) {
    for node in tree.0.nodes.iter() {
        let color = match node.ntype {
            NodeType::Empty => Color::srgb(0.7, 0.7, 0.7),
            NodeType::Leaf(_) => Color::srgb(0.9, 0.45, 0.0),
            NodeType::Branch(_) => Color::srgb(0.9, 0.9, 0.9),
        };
        let scale = node.aabb.size() as f32;
        let transform =
            Transform::from_translation(node.aabb.center().into()).with_scale(Vec3::splat(scale));

        gizmos.cuboid(transform, color);
    }
}

fn draw_elements(mut gizmos: Gizmos, tree: Res<Tree>) {
    for element in tree.0.elements.iter() {
        gizmos.sphere(
            element.position.into(),
            Quat::IDENTITY,
            1.0,
            Color::srgb(0.1, 0.45, 0.9),
        );
    }
}

fn spawn_points(mut tree: ResMut<Tree>, time: Res<Time>, mut timer: ResMut<SpawnTimer>) {
    timer.timer.tick(time.delta());

    if timer.timer.finished() {
        let mut rnd = rand::thread_rng();
        let position = TUVec3 {
            x: rnd.gen_range(0..64),
            y: rnd.gen_range(0..64),
            z: rnd.gen_range(0..64),
        };
        let c = DummyCell::new(position);
        let _ = tree.0.insert(c);
    }
}

#[derive(Resource)]
struct Tree(Octree<u32, DummyCell<u32>>);

#[derive(Resource)]
struct SpawnTimer {
    timer: Timer,
}

#[derive(Component)]
struct Camera;

struct DummyCell<U: Unsigned> {
    position: TUVec3<U>,
    node: NodeId,
}

impl<U: Unsigned> Translatable for DummyCell<U> {
    type U = U;
    fn translation(&self) -> TUVec3<U> {
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
    fn new(position: TUVec3<U>) -> Self {
        DummyCell {
            position,
            node: Default::default(),
        }
    }
}
