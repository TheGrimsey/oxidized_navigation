use std::num::NonZeroU16;

use bevy::{
    prelude::{Transform, UVec2, Vec3},
    platform::collections::HashMap,
};
use criterion::{black_box, criterion_group, criterion_main, Criterion};
use oxidized_navigation::{
    build_tile_sync,
    conversion::{ColliderType, GeometryCollection, GeometryToConvert},
    query::find_path,
    tiles::{NavMeshTile, NavMeshTiles},
    NavMeshSettings,
};
use parry3d::shape::Cuboid;

fn generate_single_primitive_geometry(nav_mesh_settings: &NavMeshSettings) -> NavMeshTile {
    let tile_coord = UVec2::new(0, 0);
    let heightfields = Box::default();

    let geometry_collections = vec![GeometryCollection {
        transform: Transform::from_xyz(0.0, 0.0, 0.0),
        geometry_to_convert: GeometryToConvert::Collider(ColliderType::Cuboid(Cuboid::new(
            Vec3::new(10.0, 0.2, 10.0).into(),
        ))),
        area: None,
    }];

    build_tile_sync(
        geometry_collections,
        tile_coord,
        heightfields,
        nav_mesh_settings,
    )
}

fn generate_many_primitive_geometry(nav_mesh_settings: &NavMeshSettings) -> NavMeshTile {
    let tile_coord = UVec2::new(0, 0);
    let heightfields = Box::default();

    let geometry_collections = vec![
        GeometryCollection {
            transform: Transform::from_xyz(0.0, 0.0, 0.0),
            geometry_to_convert: GeometryToConvert::Collider(ColliderType::Cuboid(Cuboid::new(
                Vec3::new(10.0, 0.2, 10.0).into(),
            ))),
            area: None,
        },
        GeometryCollection {
            transform: Transform::from_xyz(5.0, 1.0, 0.0),
            geometry_to_convert: GeometryToConvert::Collider(ColliderType::Cuboid(Cuboid::new(
                Vec3::new(1.0, 1.0, 1.0).into(),
            ))),
            area: None,
        },
        GeometryCollection {
            transform: Transform::from_xyz(-5.0, 1.0, 2.0),
            geometry_to_convert: GeometryToConvert::Collider(ColliderType::Cuboid(Cuboid::new(
                Vec3::new(4.0, 1.0, 1.0).into(),
            ))),
            area: None,
        },
        GeometryCollection {
            transform: Transform::from_xyz(-2.5, 2.0, 2.0),
            geometry_to_convert: GeometryToConvert::Collider(ColliderType::Cuboid(Cuboid::new(
                Vec3::new(1.0, 2.0, 1.0).into(),
            ))),
            area: None,
        },
        GeometryCollection {
            transform: Transform::from_xyz(-2.5, 2.0, -2.0),
            geometry_to_convert: GeometryToConvert::Collider(ColliderType::Cuboid(Cuboid::new(
                Vec3::new(1.0, 2.0, 1.0).into(),
            ))),
            area: None,
        },
    ];

    build_tile_sync(
        geometry_collections,
        tile_coord,
        heightfields,
        nav_mesh_settings,
    )
}

fn criterion_benchmark(c: &mut Criterion) {
    let nav_mesh_settings = NavMeshSettings {
        cell_width: 0.25,
        cell_height: 0.1,
        tile_width: NonZeroU16::new(100).unwrap(),
        world_half_extents: 12.5,
        world_bottom_bound: -100.0,
        max_traversable_slope_radians: (40.0_f32 - 0.1).to_radians(),
        walkable_height: 20,
        walkable_radius: 1,
        step_height: 3,
        min_region_area: 100,
        max_region_area_to_merge_into: 500,
        max_contour_simplification_error: 1.1,
        max_edge_length: 80,
        max_tile_generation_tasks: NonZeroU16::new(1),
        experimental_detail_mesh_generation: None,
    };

    let simple_tiles = NavMeshTiles {
        tiles: vec![(
            UVec2::ZERO,
            generate_single_primitive_geometry(&nav_mesh_settings),
        )]
        .into_iter()
        .collect(),
        tile_generations: HashMap::default(),
    };
    let many_tiles = NavMeshTiles {
        tiles: vec![(
            UVec2::ZERO,
            generate_many_primitive_geometry(&nav_mesh_settings),
        )]
        .into_iter()
        .collect(),
        tile_generations: HashMap::default(),
    };

    c.bench_function("Simple Navigation", |b| {
        b.iter(|| {
            black_box(find_path(
                &simple_tiles,
                &nav_mesh_settings,
                Vec3::new(5.0, 0.0, 5.0),
                Vec3::new(0.0, 0.0, 0.0),
                None,
                None,
            ))
        })
    });
    c.bench_function("Many Navigation", |b| {
        b.iter(|| {
            black_box(find_path(
                &many_tiles,
                &nav_mesh_settings,
                Vec3::new(5.0, 0.0, 5.0),
                Vec3::new(0.0, 0.0, 0.0),
                None,
                None,
            ))
        })
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
