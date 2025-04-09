use std::{num::NonZeroU16, time::Duration};

use avian3d::prelude::{Collider, PhysicsPlugins};
use bevy::{ecs::system::RunSystemOnce, prelude::*};
use oxidized_navigation::{
    query::{find_path, FindPathError},
    tiles::NavMeshTiles,
    ActiveGenerationTasks, NavMesh, NavMeshAffector, NavMeshSettings, OxidizedNavigationPlugin,
};

const TIMEOUT_DURATION: Duration = Duration::new(15, 0);
const SLEEP_DURATION: Duration = Duration::from_millis(2);

#[test]
fn test_simple_navigation() {
    let path = setup_app().setup_world().run_pathfinding();

    if let Err(error) = path {
        panic!("Pathfinding failed: {error:?}");
    }
}

#[test]
fn nav_mesh_is_cleared() {
    let path = setup_app().setup_world().clear_world().run_pathfinding();

    assert!(path.is_err());
}

#[test]
fn nav_mesh_is_deterministic() {
    let mut app = setup_app();

    let nav_mesh_one = app.setup_world().get_nav_mesh();
    app.clear_world();
    let nav_mesh_two = app.setup_world().get_nav_mesh();

    assert_eq!(nav_mesh_one.tiles, nav_mesh_two.tiles);
}

#[test]
fn compound_colliders_create_same_navmesh_as_individual_colliders() {
    let mut app = setup_app();

    let nav_mesh_one = app.setup_world().get_nav_mesh();
    app.clear_world();
    let nav_mesh_two = app.setup_compound_world().get_nav_mesh();

    assert_eq!(nav_mesh_one.tiles, nav_mesh_two.tiles);
}

fn setup_world_system(mut commands: Commands) {
    // Plane
    commands.spawn((
        Transform::IDENTITY,
        Collider::cuboid(25.0, 0.1, 25.0),
        NavMeshAffector,
    ));

    // Cube
    commands.spawn((
        Transform::from_xyz(-5.0, 0.8, -5.0),
        Collider::cuboid(1.25, 1.25, 1.25),
        NavMeshAffector,
    ));

    // Tall Cube
    commands.spawn((
        Transform::from_xyz(-0.179, 18.419, -27.744).with_scale(Vec3::new(15.0, 15.0, 15.0)),
        Collider::cuboid(1.25, 1.25, 1.25),
        NavMeshAffector,
    ));

    // Rotated Cube
    commands.spawn((
        Transform::from_xyz(0.0, 0.0, 0.0)
            .with_rotation(Quat::from_rotation_y(std::f32::consts::TAU / 8.0)),
        Collider::cuboid(1.25, 1.25, 1.25),
        NavMeshAffector,
    ));

    // Scaled and rotated cube
    commands.spawn((
        Transform::from_xyz(0.0, 0.0, 0.0)
            .with_rotation(Quat::from_rotation_y(std::f32::consts::TAU / 8.0))
            .with_scale(Vec3::new(2.0, 2.0, 2.0)),
        Collider::cuboid(1.25, 1.25, 1.25),
        NavMeshAffector,
    ));

    // Thin wall
    commands.spawn((
        Transform::from_xyz(-3.0, 0.8, 5.0).with_scale(Vec3::new(50.0, 15.0, 1.0)),
        Collider::cuboid(0.05, 0.05, 0.05),
        NavMeshAffector,
    ));
}

fn setup_compound_world_system(mut commands: Commands) {
    commands.spawn((
        Transform::IDENTITY,
        Collider::compound(vec![
            // Plane
            (
                Vec3::ZERO,
                Quat::IDENTITY,
                Collider::cuboid(25.0, 0.1, 25.0),
            ),
            // Cube
            (
                Vec3::new(-5.0, 0.8, -5.0),
                Quat::IDENTITY,
                Collider::cuboid(1.25, 1.25, 1.25),
            ),
            // Tall Cube
            (
                Vec3::new(-0.179, 18.419, -27.744),
                Quat::IDENTITY,
                Collider::cuboid(1.25, 1.25, 1.25).scaled_by(Vec3::new(15.0, 15.0, 15.0)),
            ),
            // Rotated Cube
            (
                Vec3::new(0.0, 0.0, 0.0),
                Quat::from_rotation_y(std::f32::consts::TAU / 8.0),
                Collider::cuboid(1.25, 1.25, 1.25),
            ),
            // Scaled and rotated cube
            (
                Vec3::new(0.0, 0.0, 0.0),
                Quat::from_rotation_y(std::f32::consts::TAU / 8.0),
                Collider::cuboid(1.25, 1.25, 1.25).scaled_by(Vec3::new(2.0, 2.0, 2.0)),
            ),
            // Thin wall
            (
                Vec3::new(-3.0, 0.8, 5.0),
                Quat::IDENTITY,
                Collider::cuboid(0.05, 0.05, 0.05),
            ),
        ]),
        NavMeshAffector,
    ));
}

trait TestApp {
    fn wait_for_generation_to_finish(&mut self) -> &mut Self;
    fn setup_world(&mut self) -> &mut Self;
    fn setup_compound_world(&mut self) -> &mut Self;
    fn clear_world(&mut self) -> &mut Self;
    fn run_pathfinding(&self) -> Result<Vec<Vec3>, FindPathError>;
    fn get_nav_mesh(&self) -> NavMeshTiles;
}

impl TestApp for App {
    fn wait_for_generation_to_finish(&mut self) -> &mut Self {
        loop {
            self.update();

            if self.world().resource::<ActiveGenerationTasks>().is_empty() {
                break;
            } else if self.world().resource::<Time>().elapsed() >= TIMEOUT_DURATION {
                panic!("Generation timed out.");
            }

            std::thread::sleep(SLEEP_DURATION);
        }

        self
    }

    fn setup_world(&mut self) -> &mut Self {
        self.world_mut()
            .run_system_once(setup_world_system)
            .unwrap();

        self.wait_for_generation_to_finish();

        self
    }

    fn setup_compound_world(&mut self) -> &mut Self {
        self.world_mut()
            .run_system_once(setup_compound_world_system)
            .unwrap();

        self.wait_for_generation_to_finish();

        self
    }

    fn clear_world(&mut self) -> &mut Self {
        self.world_mut()
            .run_system_once(
                |q_transform: Query<Entity, With<Transform>>, mut commands: Commands| {
                    for entity in q_transform.iter() {
                        commands.entity(entity).despawn_recursive();
                    }
                },
            )
            .unwrap();

        self.wait_for_generation_to_finish();

        self
    }

    fn run_pathfinding(&self) -> Result<Vec<Vec3>, FindPathError> {
        let nav_mesh_settings = self.world().resource::<NavMeshSettings>();
        let nav_mesh = self.get_nav_mesh();

        let start_pos = Vec3::new(5.0, 1.0, 5.0);
        let end_pos = Vec3::new(-15.0, 1.0, -15.0);

        // Run pathfinding to get a polygon path.
        find_path(&nav_mesh, nav_mesh_settings, start_pos, end_pos, None, None)
    }

    fn get_nav_mesh(&self) -> NavMeshTiles {
        self.world()
            .resource::<NavMesh>()
            .get()
            .read()
            .expect("Failed to get nav-mesh lock.")
            .clone()
    }
}

fn setup_app() -> App {
    let mut app = App::new();

    app.add_plugins((
        MinimalPlugins,
        TransformPlugin,
        OxidizedNavigationPlugin::<Collider>::new(NavMeshSettings {
            cell_width: 0.25,
            cell_height: 0.1,
            tile_width: NonZeroU16::new(100).unwrap(),
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
            experimental_detail_mesh_generation: None,
        }),
        PhysicsPlugins::default(),
        HierarchyPlugin,
    ));

    app
}

trait ScaledCollider {
    fn scaled_by(self, scale: Vec3) -> Self;
}

impl ScaledCollider for Collider {
    fn scaled_by(mut self, scale: Vec3) -> Self {
        self.set_scale(scale, 0);
        self
    }
}
