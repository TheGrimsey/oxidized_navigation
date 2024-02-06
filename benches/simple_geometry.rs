use bevy::prelude::{UVec2, Transform, Vec3};
use criterion::{Criterion, criterion_group, criterion_main, black_box};
use oxidized_navigation::{build_tile_sync, NavMeshSettings, conversion::{GeometryToConvert, GeometryCollection, ColliderType}};
use parry3d::shape::Cuboid;


fn generate_single_primitive_geometry() {
    let tile_coord = UVec2::new(0, 0);
    let heightfields = Box::default();
    

    let geometry_collections = vec![
        GeometryCollection {
            transform: Transform::from_xyz(0.0, 0.0, 0.0),
            geometry_to_convert: GeometryToConvert::Collider(ColliderType::Cuboid(Cuboid::new(Vec3::new(10.0, 0.2, 10.0).into()))),
            area: None
        }
    ];
    let nav_mesh_settings = NavMeshSettings {
        cell_width: 0.25,
        cell_height: 0.1,
        tile_width: 100,
        world_half_extents: 12.5,
        world_bottom_bound: -100.0,
        max_traversable_slope_radians: (40.0_f32 - 0.1).to_radians(),
        walkable_height: 20,
        walkable_radius: 1,
        step_height: 3,
        min_region_area: 100,
        merge_region_area: 500,
        max_contour_simplification_error: 1.1,
        max_edge_length: 80,
        max_tile_generation_tasks: Some(1),
    };

    black_box(build_tile_sync(geometry_collections, tile_coord, heightfields, &nav_mesh_settings));
}

fn generate_many_primitive_geometry() {
    let tile_coord = UVec2::new(0, 0);
    let heightfields = Box::default();
    

    let geometry_collections = vec![
        GeometryCollection {
            transform: Transform::from_xyz(0.0, 0.0, 0.0),
            geometry_to_convert: GeometryToConvert::Collider(ColliderType::Cuboid(Cuboid::new(Vec3::new(10.0, 0.2, 10.0).into()))),
            area: None
        },
        GeometryCollection {
            transform: Transform::from_xyz(5.0, 1.0, 0.0),
            geometry_to_convert: GeometryToConvert::Collider(ColliderType::Cuboid(Cuboid::new(Vec3::new(1.0, 1.0, 1.0).into()))),
            area: None
        },
        GeometryCollection {
            transform: Transform::from_xyz(-5.0, 1.0, 2.0),
            geometry_to_convert: GeometryToConvert::Collider(ColliderType::Cuboid(Cuboid::new(Vec3::new(4.0, 1.0, 1.0).into()))),
            area: None
        },
        GeometryCollection {
            transform: Transform::from_xyz(-2.5, 2.0, 2.0),
            geometry_to_convert: GeometryToConvert::Collider(ColliderType::Cuboid(Cuboid::new(Vec3::new(1.0, 2.0, 1.0).into()))),
            area: None
        },
        GeometryCollection {
            transform: Transform::from_xyz(-2.5, 2.0, -2.0),
            geometry_to_convert: GeometryToConvert::Collider(ColliderType::Cuboid(Cuboid::new(Vec3::new(1.0, 2.0, 1.0).into()))),
            area: None
        }
    ];
    let nav_mesh_settings = NavMeshSettings {
        cell_width: 0.25,
        cell_height: 0.1,
        tile_width: 100,
        world_half_extents: 12.5,
        world_bottom_bound: -100.0,
        max_traversable_slope_radians: (40.0_f32 - 0.1).to_radians(),
        walkable_height: 20,
        walkable_radius: 1,
        step_height: 3,
        min_region_area: 100,
        merge_region_area: 500,
        max_contour_simplification_error: 1.1,
        max_edge_length: 80,
        max_tile_generation_tasks: Some(1),
    };

    black_box(build_tile_sync(geometry_collections, tile_coord, heightfields, &nav_mesh_settings));
}

fn criterion_benchmark(c: &mut Criterion) {
    c.bench_function("Generate Single Primitive Geometry", |b| b.iter(generate_single_primitive_geometry));
    c.bench_function("Generate Many Primitive Geometry", |b| b.iter(generate_many_primitive_geometry));
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);