//! A simple example showing how to use oxidized_navigation with a custom component using parry3d colliders.

use bevy::prelude::*;
use oxidized_navigation::{
    colliders::OxidizedCollider,
    debug_draw::{DrawNavMesh, OxidizedNavigationDebugDrawPlugin},
    NavMeshAffector, NavMeshSettings, OxidizedNavigationPlugin,
};
use parry3d::shape::SharedShape;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            OxidizedNavigationPlugin::<MyParryCollider>::new(NavMeshSettings {
                cell_width: 0.25,
                cell_height: 0.1,
                tile_width: 100,
                world_half_extents: 10.0,
                world_bottom_bound: -1.0,
                max_traversable_slope_radians: (40.0_f32 - 0.1).to_radians(),
                walkable_height: 20,
                walkable_radius: 1,
                step_height: 3,
                min_region_area: 100,
                merge_region_area: 500,
                max_contour_simplification_error: 1.1,
                max_edge_length: 80,
                max_tile_generation_tasks: Some(9),
            }),
            OxidizedNavigationDebugDrawPlugin,
        ))
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (toggle_nav_mesh_system, spawn_or_despawn_affector_system),
        )
        .run();
}

#[derive(Component)]
struct MyParryCollider {
    collider: SharedShape,
}

impl OxidizedCollider for MyParryCollider {
    fn oxidized_into_typed_shape(&self) -> parry3d::shape::TypedShape {
        self.collider.as_typed_shape()
    }

    fn oxidized_compute_local_aabb(&self) -> parry3d::bounding_volume::Aabb {
        self.collider.compute_local_aabb()
    }
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    print_controls();

    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(4.0, 10.0, 15.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });

    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -1.0, -0.5, 0.0)),
        ..default()
    });

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(shape::Plane::from_size(20.0).into()),
            material: materials.add(Color::rgb(0.3, 0.5, 0.3).into()),
            ..default()
        },
        MyParryCollider {
            collider: SharedShape::cuboid(10.0, 0.1, 10.0),
        },
        NavMeshAffector,
    ));
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(shape::Cube { size: 2.0 }.into()),
            material: materials.add(Color::rgb(0.4, 0.5, 0.9).into()),
            transform: Transform::from_xyz(2.0, 1.0, -3.0),
            ..default()
        },
        MyParryCollider {
            collider: SharedShape::cuboid(1.0, 1.0, 1.0),
        },
        NavMeshAffector,
    ));

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(shape::Cube { size: 0.1 }.into()),
            material: materials.add(Color::rgb(0.4, 0.8, 0.9).into()),
            transform: Transform::from_xyz(-3.0, 0.6, 3.0).with_scale(Vec3::new(30.0, 12.0, 1.0)),
            ..default()
        },
        MyParryCollider {
            collider: SharedShape::cuboid(1.5, 0.6, 0.05),
        },
        NavMeshAffector,
    ));
}

//
//  Toggle drawing Nav-mesh.
//  Press M to toggle drawing the navmesh.
//
fn toggle_nav_mesh_system(keys: Res<Input<KeyCode>>, mut show_navmesh: ResMut<DrawNavMesh>) {
    if keys.just_pressed(KeyCode::M) {
        show_navmesh.0 = !show_navmesh.0;
    }
}

fn spawn_or_despawn_affector_system(
    keys: Res<Input<KeyCode>>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut spawned_entity: Local<Option<Entity>>,
) {
    if !keys.just_pressed(KeyCode::X) {
        return;
    }

    if let Some(entity) = *spawned_entity {
        commands.entity(entity).despawn_recursive();
        *spawned_entity = None;
    } else {
        let entity = commands
            .spawn((
                PbrBundle {
                    mesh: meshes.add(Mesh::from(bevy::prelude::shape::Cube { size: 2.5 })),
                    material: materials.add(Color::rgb(1.0, 0.1, 0.5).into()),
                    transform: Transform::from_xyz(5.0, 0.8, 5.0),
                    ..default()
                },
                MyParryCollider {
                    collider: SharedShape::cuboid(1.25, 1.25, 1.25),
                },
                NavMeshAffector, // Only entities with a NavMeshAffector component will contribute to the nav-mesh.
            ))
            .id();

        *spawned_entity = Some(entity);
    }
}

fn print_controls() {
    info!("=========================================");
    info!("| Press M to draw nav-mesh.             |");
    info!("| Press X to spawn or despawn red cube. |");
    info!("=========================================");
}
