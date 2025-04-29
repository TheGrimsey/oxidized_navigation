//! A simple example showing how to use oxidized_navigation with Avian3d.
//! Press M to draw nav-mesh.
//! Press X to spawn or despawn red cube.

use avian3d::prelude::{Collider, PhysicsPlugins};
use bevy::{math::primitives, prelude::*};
use oxidized_navigation::{
    debug_draw::{DrawNavMesh, OxidizedNavigationDebugDrawPlugin},
    NavMeshAffector, NavMeshSettings, OxidizedNavigationPlugin,
};
use oxidized_navigation_avian::AvianCollider;

fn main() {
    App::new()
        // Default Plugins
        .add_plugins((
            DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    title: "Oxidized Navigation: Avian3d".to_owned(),
                    ..default()
                }),
                ..default()
            }),
            OxidizedNavigationPlugin::<AvianCollider>::new(NavMeshSettings::from_agent_and_bounds(
                0.5, 1.9, 250.0, -1.0,
            )),
            OxidizedNavigationDebugDrawPlugin,
            PhysicsPlugins::default(),
        ))
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (toggle_nav_mesh_debug_draw, spawn_or_despawn_affector_system),
        )
        .run();
}

fn toggle_nav_mesh_debug_draw(
    keys: Res<ButtonInput<KeyCode>>,
    mut show_navmesh: ResMut<DrawNavMesh>,
) {
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

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(10.0, 10.0, 15.0).looking_at(Vec3::new(0.0, 2.0, 0.0), Vec3::Y),
    ));

    // Directional light
    commands.spawn((
        DirectionalLight {
            shadows_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -1.0, -0.5, 0.0)),
    ));

    // Ground plane
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(25.0, 25.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.3, 0.5, 0.3))),
        Transform::IDENTITY,
        Collider::cuboid(25.0, 0.1, 25.0),
        NavMeshAffector, // Only entities with a NavMeshAffector component will contribute to the nav-mesh.
    ));

    // Cube
    commands.spawn((
        Mesh3d(meshes.add(primitives::Cuboid::new(2.5, 2.5, 2.5))),
        MeshMaterial3d(materials.add(Color::srgb(0.1, 0.1, 0.5))),
        Transform::from_xyz(-5.0, 0.8, -5.0),
        Collider::cuboid(1.25, 1.25, 1.25),
        NavMeshAffector, // Only entities with a NavMeshAffector component will contribute to the nav-mesh.
    ));

    // Thin wall
    commands.spawn((
        Mesh3d(meshes.add(Mesh::from(primitives::Cuboid::new(0.1, 0.1, 0.1)))),
        MeshMaterial3d(materials.add(Color::srgb(0.1, 0.1, 0.5))),
        Transform::from_xyz(-3.0, 0.8, 5.0).with_scale(Vec3::new(50.0, 15.0, 1.0)),
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
                Mesh3d(meshes.add(primitives::Cuboid::new(2.5, 2.5, 2.5))),
                MeshMaterial3d(materials.add(Color::srgb(1.0, 0.1, 0.5))),
                Transform::from_xyz(5.0, 0.8, 0.0),
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
