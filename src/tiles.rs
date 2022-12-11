use bevy::prelude::{Resource, UVec2, UVec3};

use super::mesher::PolyMesh;

pub type PolygonRef = u64;

struct MeshTile {
    vertices: Vec<UVec3>,
    indices: Vec<u32>,
}

#[derive(Default, Resource)]
struct NavMesh {}

impl NavMesh {
    pub fn add_tile(&mut self, tile_coord: UVec2, tile: &PolyMesh) {}
}

fn create_nav_mesh_data_from_poly_mesh(mesh: &PolyMesh) {
    let (min_height, max_height) = mesh.vertices.iter().fold((u32::MAX, 0), |(min, max), val| {
        (min.min(val.y), max.max(val.y))
    });
}
