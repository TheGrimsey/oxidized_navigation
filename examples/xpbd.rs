//! A simple example showing how to use oxidized_navigation with xpbd.

use bevy::{math::primitives, prelude::*};
use bevy_xpbd_3d::prelude::{Collider, PhysicsPlugins};
use oxidized_navigation::{
    debug_draw::{DrawNavMesh, OxidizedNavigationDebugDrawPlugin},
    NavMeshAffector, NavMeshSettings, OxidizedNavigationPlugin,
};

fn main() {
    App::new()
        // Default Plugins
        .add_plugins((
            DefaultPlugins,
            OxidizedNavigationPlugin::<Collider>::new(
                NavMeshSettings::from_agent_and_bounds(0.5, 1.9, 250.0, -1.0),
            ),
            OxidizedNavigationDebugDrawPlugin,
            PhysicsPlugins::default(),
        ))
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (toggle_nav_mesh_system, spawn_or_despawn_affector_system),
        )
        .run();
}

//
//  Toggle drawing Nav-mesh.
//  Press M to toggle drawing the navmesh.
//
fn toggle_nav_mesh_system(keys: Res<ButtonInput<KeyCode>>, mut show_navmesh: ResMut<DrawNavMesh>) {
    if keys.just_pressed(KeyCode::KeyM) {
        show_navmesh.0 = !show_navmesh.0;
    }
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    print_controls();

    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(10.0, 10.0, 15.0)
            .looking_at(Vec3::new(0.0, 2.0, 0.0), Vec3::Y),
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

    // Plane
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(primitives::Rectangle::from_size(Vec2::new(50.0, 50.0))),
            material: materials.add(Color::rgb(0.3, 0.5, 0.3)),
            transform: Transform::IDENTITY,
            ..default()
        },
        Collider::cuboid(25.0, 0.1, 25.0),
        NavMeshAffector, // Only entities with a NavMeshAffector component will contribute to the nav-mesh.
    ));

    // Cube
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(primitives::Cuboid::new(2.5, 2.5, 2.5)),
            material: materials.add(Color::rgb(0.1, 0.1, 0.5)),
            transform: Transform::from_xyz(-5.0, 0.8, -5.0),
            ..default()
        },
        Collider::cuboid(1.25, 1.25, 1.25),
        NavMeshAffector, // Only entities with a NavMeshAffector component will contribute to the nav-mesh.
    ));

    // Thin wall
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Mesh::from(primitives::Cuboid::new(0.1, 0.1, 0.1))),
            material: materials.add(Color::rgb(0.1, 0.1, 0.5)),
            transform: Transform::from_xyz(-3.0, 0.8, 5.0).with_scale(Vec3::new(50.0, 15.0, 1.0)),
            ..default()
        },
        Collider::cuboid(0.05, 0.05, 0.05),
        NavMeshAffector, // Only entities with a NavMeshAffector component will contribute to the nav-mesh.
    ));
}

fn spawn_or_despawn_affector_system(
    keys: Res<ButtonInput<KeyCode>>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut spawned_entity: Local<Option<Entity>>,
) {
    if !keys.just_pressed(KeyCode::KeyX) {
        return;
    }

    if let Some(entity) = *spawned_entity {
        commands.entity(entity).despawn_recursive();
        *spawned_entity = None;
    } else {
        let entity = commands
            .spawn((
                PbrBundle {
                    mesh: meshes.add(primitives::Cuboid::new(2.5, 2.5, 2.5)),
                    material: materials.add(Color::rgb(1.0, 0.1, 0.5)),
                    transform: Transform::from_xyz(5.0, 0.8, 0.0),
                    ..default()
                },
                Collider::cuboid(2.5, 2.5, 2.5),
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
