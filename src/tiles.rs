use bevy::prelude::{Resource, UVec2, UVec3};

use crate::mesher::{EdgeConnectionType, EdgeConnectionDirection, VERTICES_PER_POLYGON};

use super::mesher::PolyMesh;

pub type PolygonIdentifier = u64;

pub enum Link {
    Internal {
        polygon: u32,
        edge: u8
    },
    OffMesh {
        polygon: u32,
        edge: u8,
        neighbour: PolygonIdentifier,
        direction: EdgeConnectionDirection,
    }
}

struct NavMeshTile {
    vertices: Vec<UVec3>,
    polygons: Vec<[u32; VERTICES_PER_POLYGON]>,

    links: Vec<Link>,

    min_height: u32,
    max_height: u32,
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

    let (edge_count, portal_count) = mesh.edges.iter().fold((0, 0), |(edges, portals), value| {
        let mut linked_edges_in_polygon = 0;
        let mut portals_in_polygon = 0;

        for edge_connection_type in value.iter() {
            match edge_connection_type {
                EdgeConnectionType::None => {},
                EdgeConnectionType::Internal(_) => {
                    linked_edges_in_polygon += 1;
                },
                EdgeConnectionType::OffMesh(_) => {
                    linked_edges_in_polygon += 1;
                    portals_in_polygon += 1;
                },
            }
        }

        (edges + linked_edges_in_polygon, portals + portals_in_polygon)
    });

    let total_links = edge_count + portal_count * 2;


}
