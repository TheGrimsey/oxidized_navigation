use bevy::{prelude::{Resource, UVec2, UVec3}, utils::HashMap};
use smallvec::SmallVec;

use crate::mesher::{EdgeConnection, EdgeConnectionDirection, VERTICES_PER_POLYGON};

use super::mesher::PolyMesh;

#[derive(Clone, Copy)]
pub enum Link {
    Internal {
        edge: u8,
        neighbour_polygon: u16,
    },
    OffMesh {
        edge: u8,
        neighbour_polygon: u16,
        direction: EdgeConnectionDirection, // Knowing the direction of the other tile is enough to figure out the tile from our tile.
    },
}

struct Polygon {
    indices: [u32; VERTICES_PER_POLYGON],
    links: SmallVec<[Link; VERTICES_PER_POLYGON * 2]>, // This becomes a mess memory wise with a ton of different small objects around.
}

/*
*   Polygons make up a form of graph, linking to other polygons (which could be on another mesh)
*/
pub(super) struct NavMeshTile {
    salt: u32,
    vertices: Vec<UVec3>,
    polygons: Vec<Polygon>,
    edges: Vec<[EdgeConnection; VERTICES_PER_POLYGON]>,

    min_height: u32,
    max_height: u32,
    
}

#[derive(Default, Resource)]
pub(super) struct NavMesh {
    tiles: HashMap<UVec2, NavMeshTile>
}

impl NavMesh {
    pub fn add_tile(&mut self, tile_coord: UVec2, mut tile: NavMeshTile) {
        // Get an incremented salt.
        let previous_tile_existed = if let Some(old_tile) = self.tiles.get(&tile_coord) {
            tile.salt = old_tile.salt + 1;
            true
        } else {
            tile.salt = 0;
            false
        };

        // Connect neighbours.
        if tile_coord.x > 0 {
            let neighbour_coord = UVec2::new(tile_coord.x - 1, tile_coord.y);

            if let Some(neighbour) = self.tiles.get_mut(&neighbour_coord) {
                connect_external_links(&mut tile, neighbour, EdgeConnectionDirection::XPositive, previous_tile_existed);
            }
        }
    }
}

fn connect_external_links(new_tile: &mut NavMeshTile, neighbour_tile: &mut NavMeshTile, opposite_direction: EdgeConnectionDirection, tile_existed: bool) {
    for polygon in neighbour_tile.polygons.iter_mut() { // TODO: What if we just store a list of edge polygons?
        if tile_existed { // Remove existing links to this tile.
            let mut i = 0;
            while i < polygon.links.len() {
                if let Link::OffMesh { direction, .. } = polygon.links[i] {
                    if direction == opposite_direction {
                        polygon.links.swap_remove(i);
                    }
                }

                i += 1;
            }
        }

        
    }
}

pub(super) fn create_nav_mesh_data_from_poly_mesh(mesh: &PolyMesh) -> NavMeshTile {
    let (min_height, max_height) = mesh.vertices.iter().fold((u32::MAX, 0), |(min, max), val| {
        (min.min(val.y), max.max(val.y))
    });

    // Slight worry that the compiler won't optimize this but damn, it's cool.
    let polygons = mesh.polygons.iter().zip(mesh.edges.iter()).map(|(indices, edges)| {
        let links = edges.iter().enumerate().filter_map(|(i, edge)| {
            let EdgeConnection::Internal(other_polygon) = edge else {
                return None;
            };

            Some(Link::Internal { 
                edge: i as u8,
                neighbour_polygon: *other_polygon
            })
        }).collect();

        Polygon {
            links,
            indices: *indices,
        }
    }).collect();

    NavMeshTile {
        salt: 0,
        vertices: mesh.vertices.clone(),
        edges: mesh.edges.clone(),
        polygons,
        min_height,
        max_height,
    }
}
