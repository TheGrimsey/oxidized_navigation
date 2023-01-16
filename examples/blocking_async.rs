//! Nav-mesh set up example for both blocking & async pathfinding.
//!
//! Press A to run async path finding.
//!
//! Press B to run blocking path finding.
//!

use std::sync::{Arc, RwLock};

use bevy::{
    prelude::*,
    tasks::{AsyncComputeTaskPool, Task},
    DefaultPlugins,
};
use bevy_rapier3d::prelude::Collider;
use futures_lite::future;
use oxidized_navigation::{
    query::{find_path, perform_string_pulling_on_path},
    tiles::NavMeshTiles,
    NavMesh, NavMeshAffector, NavMeshSettings, OxidizedNavigationPlugin,
};

fn main() {
    App::new()
        .insert_resource(NavMeshSettings {
            cell_width: 0.25,
            cell_height: 0.1,
            tile_width: 100,
            world_half_extents: 250.0,
            world_bottom_bound: -100.0,
            max_traversable_slope_radians: (40.0_f32 - 0.1).to_radians(),
            walkable_height: 20,
            walkable_radius: 2,
            step_height: 3,
            min_region_area: 100,
            merge_region_area: 500,
            max_contour_simplification_error: 1.3,
            max_edge_length: 80,
        })
        .insert_resource(AsyncPathfindingTasks::default())
        // Default Plugins
        .add_plugins(DefaultPlugins)
        .add_plugin(OxidizedNavigationPlugin)
        .add_startup_system(setup_world_system)
        .add_startup_system(info_system)
        .add_system(run_blocking_pathfinding)
        .add_system(run_async_pathfinding)
        .add_system(poll_pathfinding_tasks_system)
        .run();
}

//
//  Blocking Pathfinding.
//  Press B to run.
//
//  Running pathfinding in a system.
//
fn run_blocking_pathfinding(
    keys: Res<Input<KeyCode>>,
    nav_mesh_settings: Res<NavMeshSettings>,
    nav_mesh: Res<NavMesh>,
) {
    if !keys.just_pressed(KeyCode::B) {
        return;
    }

    // Get the underlying nav_mesh.
    if let Ok(nav_mesh) = nav_mesh.get().read() {
        let start_pos = Vec3::new(5.0, 1.0, 5.0);
        let end_pos = Vec3::new(-15.0, 1.0, -15.0);

        // Run pathfinding to get a polygon path.
        match find_path(&nav_mesh, &nav_mesh_settings, start_pos, end_pos, None) {
            Ok(path) => {
                info!("Path found (BLOCKING): {:?}", path);

                // Convert polygon path to a path of Vec3s.
                match perform_string_pulling_on_path(&nav_mesh, start_pos, end_pos, &path) {
                    Ok(string_path) => {
                        info!("String path (BLOCKING): {:?}", string_path);
                    }
                    Err(error) => error!("Error with string path: {:?}", error),
                };
            }
            Err(error) => error!("Error with pathfinding: {:?}", error),
        }
    }
}

//
//  Async Pathfinding.
//  Press A to run.
//
//  Running pathfinding in a task without blocking the frame.
//  Also check out Bevy's async compute example.
//  https://github.com/bevyengine/bevy/blob/main/examples/async_tasks/async_compute.rs
//

// Holder resource for tasks.
#[derive(Default, Resource)]
struct AsyncPathfindingTasks {
    tasks: Vec<Task<Option<Vec<Vec3>>>>,
}

// Queue up pathfinding tasks.
fn run_async_pathfinding(
    keys: Res<Input<KeyCode>>,
    nav_mesh_settings: Res<NavMeshSettings>,
    nav_mesh: Res<NavMesh>,
    mut pathfinding_task: ResMut<AsyncPathfindingTasks>,
) {
    if !keys.just_pressed(KeyCode::A) {
        return;
    }

    let thread_pool = AsyncComputeTaskPool::get();

    let nav_mesh_lock = nav_mesh.get();
    let start_pos = Vec3::new(5.0, 1.0, 5.0);
    let end_pos = Vec3::new(-15.0, 1.0, -15.0);

    let task = thread_pool.spawn(async_path_find(
        nav_mesh_lock,
        nav_mesh_settings.clone(),
        start_pos,
        end_pos,
        None,
    ));

    pathfinding_task.tasks.push(task);
}

// Poll existing tasks.
fn poll_pathfinding_tasks_system(mut pathfinding_task: ResMut<AsyncPathfindingTasks>) {
    let mut i = 0;
    while i < pathfinding_task.tasks.len() {
        let task = &mut pathfinding_task.tasks[i];
        if let Some(string_path) = future::block_on(future::poll_once(task)).unwrap_or(None) {
            info!("Async path task finished with result: {:?}", string_path);
            let _ = pathfinding_task.tasks.swap_remove(i);
        } else {
            i += 1;
        }
    }
}

/// Async wrapper function for path finding.
async fn async_path_find(
    nav_mesh_lock: Arc<RwLock<NavMeshTiles>>,
    nav_mesh_settings: NavMeshSettings,
    start_pos: Vec3,
    end_pos: Vec3,
    position_search_radius: Option<f32>,
) -> Option<Vec<Vec3>> {
    // Get the underlying nav_mesh.
    let Ok(nav_mesh) = nav_mesh_lock.read() else {
        return None;
    };

    // Run pathfinding to get a polygon path.
    match find_path(
        &nav_mesh,
        &nav_mesh_settings,
        start_pos,
        end_pos,
        position_search_radius,
    ) {
        Ok(path) => {
            info!("Path found (ASYNC): {:?}", path);
            // Convert polygon path to a path of Vec3s.
            match perform_string_pulling_on_path(&nav_mesh, start_pos, end_pos, &path) {
                Ok(string_path) => {
                    info!("String path (ASYNC): {:?}", string_path);
                    return Some(string_path);
                }
                Err(error) => error!("Error with string path: {:?}", error),
            };
        }
        Err(error) => error!("Error with pathfinding: {:?}", error),
    }

    None
}

fn setup_world_system(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Plane
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Mesh::from(bevy::prelude::shape::Plane { size: 50.0 })),
            material: materials.add(Color::rgb(0.3, 0.5, 0.3).into()),
            transform: Transform::IDENTITY,
            ..default()
        },
        Collider::cuboid(25.0, 0.1, 25.0),
        NavMeshAffector::default(), // Only entities with a NavMeshAffector component will contribute to the nav-mesh.
    ));

    // Cube
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Mesh::from(bevy::prelude::shape::Cube { size: 2.5 })),
            material: materials.add(Color::rgb(0.1, 0.1, 0.5).into()),
            transform: Transform::from_xyz(-5.0, 0.8, -5.0),
            ..default()
        },
        Collider::cuboid(1.25, 1.25, 1.25),
        NavMeshAffector::default(), // Only entities with a NavMeshAffector component will contribute to the nav-mesh.
    ));

    // light
    commands.spawn(PointLightBundle {
        transform: Transform::from_xyz(4.0, 8.0, 4.0),
        ..default()
    });

    // Camera
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(10.0, 5.0, 15.0)
            .looking_at(Vec3::new(0.0, 2.0, 0.0), Vec3::Y),
        ..default()
    });
}

fn info_system() {
    info!("=========================================");
    info!("| Press A to run ASYNC path finding.    |");
    info!("| Press B to run BLOCKING path finding. |");
    info!("=========================================");
}
