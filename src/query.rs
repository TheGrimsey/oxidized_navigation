use std::sync::{Arc, RwLock};

use bevy::prelude::Vec3;

use crate::{tiles::NavMesh, NavMeshSettings};


struct NavMeshPathNode {
    pos: Vec3,
    cost: f32,
    total_cost: f32,
}

struct NavMeshPath {
    nodes: Vec<NavMeshPathNode>
}

async fn find_path(
    nav_mesh: Arc<RwLock<NavMesh>>,
    nav_mesh_settings: NavMeshSettings,
    start_pos: Vec3,
    end_pos: Vec3,
) -> Option<NavMeshPath> {
    let Ok(nav_mesh) = nav_mesh.read() else {
        return None;
    };

    let Some((start_tile, start_poly, start_pos)) = nav_mesh.find_closest_polygon_in_box(&nav_mesh_settings, start_pos, 5.0) else {
        return None;
    };

    let Some((end_tile, end_poly, end_pos)) = nav_mesh.find_closest_polygon_in_box(&nav_mesh_settings, end_pos, 5.0) else {
        return None;
    };

    None
}