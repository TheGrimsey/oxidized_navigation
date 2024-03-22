use std::{num::NonZeroU16, time::Duration};

use bevy::prelude::*;
use bevy_xpbd_3d::prelude::{Collider, PhysicsPlugins};
use oxidized_navigation::{
    query::find_path, ActiveGenerationTasks, NavMesh, NavMeshAffector, NavMeshSettings,
    OxidizedNavigationPlugin,
};

const TIMEOUT_DURATION: Duration = Duration::new(15, 0);
const SLEEP_DURATION: Duration = Duration::from_millis(2);

fn setup_world_system(mut commands: Commands) {
    // Plane
    commands.spawn((
        TransformBundle::IDENTITY,
        Collider::cuboid(25.0, 0.1, 25.0),
        NavMeshAffector,
    ));

    // Cube
    commands.spawn((
        TransformBundle::from_transform(Transform::from_xyz(-5.0, 0.8, -5.0)),
        Collider::cuboid(1.25, 1.25, 1.25),
        NavMeshAffector,
    ));

    // Tall Cube
    commands.spawn((
        TransformBundle::from_transform(
            Transform::from_xyz(-0.179, 18.419, -27.744).with_scale(Vec3::new(15.0, 15.0, 15.0)),
        ),
        Collider::cuboid(1.25, 1.25, 1.25),
        NavMeshAffector,
    ));

    // Thin wall
    commands.spawn((
        TransformBundle::from_transform(
            Transform::from_xyz(-3.0, 0.8, 5.0).with_scale(Vec3::new(50.0, 15.0, 1.0)),
        ),
        Collider::cuboid(0.05, 0.05, 0.05),
        NavMeshAffector,
    ));
}

fn setup_app(app: &mut App) {
    app.add_plugins((
        MinimalPlugins,
        TransformPlugin,
        OxidizedNavigationPlugin::<Collider>::new(NavMeshSettings {
            cell_width: 0.25,
            cell_height: 0.1,
            tile_width: 100,
            world_half_extents: 250.0,
            world_bottom_bound: -100.0,
            max_traversable_slope_radians: (40.0_f32 - 0.1).to_radians(),
            walkable_height: 20,
            walkable_radius: 1,
            step_height: 3,
            min_region_area: 100,
            max_region_area_to_merge_into: 500,
            max_contour_simplification_error: 1.1,
            max_edge_length: 80,
            max_tile_generation_tasks: NonZeroU16::new(8), // Github Actions are limited to 7 GB.
        }),
        PhysicsPlugins::default(),
    ));
}

fn wait_for_generation_to_finish(app: &mut App) {
    loop {
        app.update();

        if app.world.resource::<ActiveGenerationTasks>().is_empty() {
            break;
        } else if app.world.resource::<Time>().elapsed() >= TIMEOUT_DURATION {
            panic!("Generation timed out.");
        }

        std::thread::sleep(SLEEP_DURATION);
    }
}

#[test]
fn test_simple_navigation() {
    let mut app = App::new();

    setup_app(&mut app);

    app.add_systems(Startup, setup_world_system);

    wait_for_generation_to_finish(&mut app);

    let nav_mesh_settings = app.world.resource::<NavMeshSettings>();
    let nav_mesh = app.world.resource::<NavMesh>().get();
    let nav_mesh = nav_mesh.read().expect("Failed to get nav-mesh lock.");

    let start_pos = Vec3::new(5.0, 1.0, 5.0);
    let end_pos = Vec3::new(-15.0, 1.0, -15.0);

    // Run pathfinding to get a polygon path.
    let path = find_path(&nav_mesh, nav_mesh_settings, start_pos, end_pos, None, None);

    if let Err(error) = path {
        panic!("Pathfinding failed: {error:?}");
    }
}
