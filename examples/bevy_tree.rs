use std::time::Duration;

use bevy::{color::palettes::css::RED, prelude::*};
use oktree::prelude::*;
use rand::Rng;

const SIZE: u32 = 256;
const COUNTER: usize = 1024;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_systems(Startup, setup)
        .add_systems(FixedUpdate, spawn_points)
        .add_systems(Update, (draw_nodes, draw_elements))
        .run();
}

fn setup(mut commands: Commands) {
    commands.insert_resource(Tree(Octree::from_aabb(Aabb::new_unchecked(
        TUVec3::splat(SIZE / 2),
        SIZE / 2,
    ))));

    commands.insert_resource(SpawnTimer {
        timer: Timer::new(Duration::from_millis(10), TimerMode::Repeating),
    });

    commands.insert_resource(Mode::Insert);

    commands.insert_resource(Counter(0));

    let position = Transform::from_xyz(-(SIZE as f32), 0.0, (SIZE / 2) as f32)
        .looking_at(Vec3::splat((SIZE / 2) as f32), Vec3::Z);
    commands.spawn((
        Camera3dBundle {
            transform: position,
            ..default()
        },
        Camera,
    ));
}

fn draw_nodes(mut gizmos: Gizmos, tree: Res<Tree>) {
    for node in tree.0.iter_nodes() {
        let scale = node.aabb.size() as f32;
        let transform =
            Transform::from_translation(node.aabb.center().into()).with_scale(Vec3::splat(scale));

        match node.ntype {
            NodeType::Empty => gizmos.cuboid(transform, Color::srgb(0.7, 0.7, 0.7)),
            NodeType::Leaf(_) => gizmos.cuboid(transform, Color::srgb(0.9, 0.45, 0.0)),
            NodeType::Branch(_) => (),
        };
    }
}

fn draw_elements(mut gizmos: Gizmos, tree: Res<Tree>) {
    for element in tree.0.iter() {
        gizmos.sphere(element.position.into(), Quat::IDENTITY, 1.0, RED);
    }
}

fn spawn_points(
    mut tree: ResMut<Tree>,
    time: Res<Time>,
    mut timer: ResMut<SpawnTimer>,
    mut mode: ResMut<Mode>,
    mut counter: ResMut<Counter>,
) {
    timer.timer.tick(time.delta());

    if timer.timer.finished() {
        match *mode {
            Mode::Insert => {
                let mut rnd = rand::thread_rng();
                let position = TUVec3 {
                    x: rnd.gen_range(0..SIZE),
                    y: rnd.gen_range(0..SIZE),
                    z: rnd.gen_range(0..SIZE),
                };
                let c = DummyCell::new(position);
                let _ = tree.0.insert(c);
                counter.0 += 1;
                if counter.0 >= COUNTER {
                    *mode = Mode::Remove;
                }
            }
            Mode::Remove => {
                counter.0 -= 1;
                let _ = tree.0.remove(counter.0.into());
                if counter.0 == 0 {
                    *mode = Mode::Insert;
                }
            }
        }
    }
}

#[derive(Resource)]
struct Tree(Octree<u32, DummyCell<u32>>);

#[derive(Resource)]
struct SpawnTimer {
    timer: Timer,
}

#[derive(Resource)]
enum Mode {
    Insert,
    Remove,
}

#[derive(Resource)]
struct Counter(usize);

#[derive(Component)]
struct Camera;

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
