use std::sync::{Arc, RwLock};

use bevy::{
    prelude::{Resource, UVec2, Vec2, Vec3},
    utils::HashMap,
};
use smallvec::SmallVec;

use crate::{
    mesher::{EdgeConnection, EdgeConnectionDirection, VERTICES_PER_POLYGON},
    NavMeshSettings,
};

use super::mesher::PolyMesh;

#[derive(Clone, Copy, Debug)]
pub enum Link {
    Internal {
        edge: u8,
        neighbour_polygon: u16,
    },
    OffMesh {
        edge: u8,
        neighbour_polygon: u16,
        direction: EdgeConnectionDirection, // Knowing the direction of the other tile is enough to figure out the tile from our tile.
        bound_min: u8,                      // % bound of edge that links to this.
        bound_max: u8, // For example: 10% -> 50% = the connected edge covers 10% from vertex A to B to 50%.
    },
}

#[derive(Debug)]
pub(super) struct Polygon {
    indices: [u32; VERTICES_PER_POLYGON],
    links: SmallVec<[Link; VERTICES_PER_POLYGON * 2]>, // This becomes a mess memory wise with a ton of different small objects around.
}

/*
*   Polygons make up a form of graph, linking to other polygons (which could be on another mesh)
*/
pub(super) struct NavMeshTile {
    salt: u32,
    vertices: Vec<Vec3>,
    pub(super) polygons: Vec<Polygon>,
    edges: Vec<[EdgeConnection; VERTICES_PER_POLYGON]>,
}

#[derive(Default, Resource)]
pub(super) struct NavMeshTiles {
    pub(super) nav_mesh: Arc<RwLock<NavMesh>>
}

#[derive(Default)]
pub(super) struct NavMesh {
    pub(super) tiles: HashMap<UVec2, NavMeshTile>,
}

impl NavMesh {
    pub fn add_tile(
        &mut self,
        tile_coord: UVec2,
        mut tile: NavMeshTile,
        nav_mesh_settings: &NavMeshSettings,
    ) {
        // Get an incremented salt.
        let previous_tile_existed = if let Some(old_tile) = self.tiles.get(&tile_coord) {
            tile.salt = old_tile.salt + 1;
            true
        } else {
            tile.salt = 0;
            false
        };

        // Connect neighbours.
        let step_height = nav_mesh_settings.step_height as f32 * nav_mesh_settings.cell_height;
        // X-Negative
        if tile_coord.x > 0 {
            let neighbour_coord = UVec2::new(tile_coord.x - 1, tile_coord.y);

            if let Some(neighbour) = self.tiles.get_mut(&neighbour_coord) {
                let direction = EdgeConnectionDirection::XNegative;
                let opposite_direction = EdgeConnectionDirection::XPositive;

                connect_external_links(
                    &mut tile,
                    neighbour,
                    direction,
                    opposite_direction,
                    false,
                    step_height,
                );
                connect_external_links(
                    neighbour,
                    &tile,
                    opposite_direction,
                    direction,
                    previous_tile_existed,
                    step_height,
                );
            }
        }
        // X-Positive
        if tile_coord.x < u32::MAX {
            let neighbour_coord = UVec2::new(tile_coord.x + 1, tile_coord.y);

            if let Some(neighbour) = self.tiles.get_mut(&neighbour_coord) {
                let direction = EdgeConnectionDirection::XPositive;
                let opposite_direction = EdgeConnectionDirection::XNegative;

                connect_external_links(
                    &mut tile,
                    neighbour,
                    direction,
                    opposite_direction,
                    false,
                    step_height,
                );
                connect_external_links(
                    neighbour,
                    &tile,
                    opposite_direction,
                    direction,
                    previous_tile_existed,
                    step_height,
                );
            }
        }
        // Z-Negative
        if tile_coord.y > 0 {
            let neighbour_coord = UVec2::new(tile_coord.x, tile_coord.y - 1);

            if let Some(neighbour) = self.tiles.get_mut(&neighbour_coord) {
                let direction = EdgeConnectionDirection::ZNegative;
                let opposite_direction = EdgeConnectionDirection::ZPositive;

                connect_external_links(
                    &mut tile,
                    neighbour,
                    direction,
                    opposite_direction,
                    false,
                    step_height,
                );
                connect_external_links(
                    neighbour,
                    &tile,
                    opposite_direction,
                    direction,
                    previous_tile_existed,
                    step_height,
                );
            }
        }
        // Z-Positive
        if tile_coord.y < u32::MAX {
            let neighbour_coord = UVec2::new(tile_coord.x, tile_coord.y + 1);

            if let Some(neighbour) = self.tiles.get_mut(&neighbour_coord) {
                let direction = EdgeConnectionDirection::ZPositive;
                let opposite_direction = EdgeConnectionDirection::ZNegative;

                connect_external_links(
                    &mut tile,
                    neighbour,
                    direction,
                    opposite_direction,
                    false,
                    step_height,
                );
                connect_external_links(
                    neighbour,
                    &tile,
                    opposite_direction,
                    direction,
                    previous_tile_existed,
                    step_height,
                );
            }
        }

        self.tiles.insert(tile_coord, tile);
    }
}

fn connect_external_links(
    tile: &mut NavMeshTile,
    neighbour: &NavMeshTile,
    neighbour_direction: EdgeConnectionDirection,
    neighbour_to_self_direction: EdgeConnectionDirection,
    remove_existing_links: bool,
    step_height: f32,
) {
    for (poly_index, polygon) in tile.polygons.iter_mut().enumerate() {
        // TODO: What if we just store a list of edge polygons? Allows us to skip majority of polygons.
        if remove_existing_links {
            // Remove existing links to neighbour.
            let mut i = 0;
            while i < polygon.links.len() {
                if let Link::OffMesh { direction, .. } = polygon.links[i] {
                    if direction == neighbour_direction {
                        polygon.links.swap_remove(i);
                    }
                }

                i += 1;
            }
        }

        for (edge_index, edge) in tile.edges[poly_index].iter().enumerate() {
            let EdgeConnection::OffMesh(edge_direction) = edge else {
                continue;
            };
            if *edge_direction != neighbour_direction {
                continue;
            }

            let vertex_a = tile.vertices[polygon.indices[edge_index] as usize];
            let vertex_b =
                tile.vertices[polygon.indices[(edge_index + 1) % polygon.indices.len()] as usize];

            let (connection_count, connected_polys, connection_areas) =
                find_connecting_polygons_in_tile(
                    &vertex_a,
                    &vertex_b,
                    neighbour,
                    neighbour_to_self_direction,
                    step_height,
                );

            polygon.links.reserve(connection_count);
            for i in 0..connection_count {
                let (neighbour_polygon, edge) = connected_polys[i];
                let area = connection_areas[i];

                let (bound_min, bound_max) = if neighbour_to_self_direction
                    == EdgeConnectionDirection::XNegative
                    || neighbour_to_self_direction == EdgeConnectionDirection::XPositive
                {
                    let mut min = (area.x - vertex_a.z) / (vertex_b.z - vertex_a.z);
                    let mut max = (area.y - vertex_a.z) / (vertex_b.z - vertex_a.z);

                    if min > max {
                        std::mem::swap(&mut min, &mut max);
                    }

                    (min, max)
                } else {
                    let mut min = (area.x - vertex_a.x) / (vertex_b.x - vertex_a.x);
                    let mut max = (area.y - vertex_a.x) / (vertex_b.x - vertex_a.x);

                    if min > max {
                        std::mem::swap(&mut min, &mut max);
                    }

                    (min, max)
                };

                let min_byte = (bound_min.clamp(0.0, 1.0) * 255.0).round() as u8;
                let max_byte = (bound_max.clamp(0.0, 1.0) * 255.0).round() as u8;

                polygon.links.push(Link::OffMesh {
                    edge,
                    neighbour_polygon,
                    direction: neighbour_direction,
                    bound_min: min_byte,
                    bound_max: max_byte,
                });
            }
            break; // We can only have one edge parallel to the direction in a triangle.
        }
    }
}

fn calculate_slab_end_points(
    vertex_a: &Vec3,
    vertex_b: &Vec3,
    side: EdgeConnectionDirection,
) -> (Vec2, Vec2) {
    if side == EdgeConnectionDirection::XNegative || side == EdgeConnectionDirection::XPositive {
        if vertex_a.z < vertex_b.z {
            let min = Vec2::new(vertex_a.z, vertex_a.y);
            let max = Vec2::new(vertex_b.z, vertex_b.y);

            (min, max)
        } else {
            let min = Vec2::new(vertex_b.z, vertex_b.y);
            let max = Vec2::new(vertex_a.z, vertex_a.y);

            (min, max)
        }
    } else if vertex_a.z < vertex_b.z {
        let min = Vec2::new(vertex_a.x, vertex_a.y);
        let max = Vec2::new(vertex_b.x, vertex_b.y);

        (min, max)
    } else {
        let min = Vec2::new(vertex_b.x, vertex_b.y);
        let max = Vec2::new(vertex_a.x, vertex_a.y);

        (min, max)
    }
}

fn get_slab_position(vertex: &Vec3, side: EdgeConnectionDirection) -> f32 {
    match side {
        EdgeConnectionDirection::XNegative => vertex.x,
        EdgeConnectionDirection::ZPositive => vertex.z,
        EdgeConnectionDirection::XPositive => vertex.x,
        EdgeConnectionDirection::ZNegative => vertex.z,
    }
}

fn check_slabs_overlap(
    a_min: Vec2,
    a_max: Vec2,
    b_min: Vec2,
    b_max: Vec2,
    edge_shrink: f32,
    allowed_step: f32,
) -> bool {
    let min_edge = (a_min.x + edge_shrink).max(b_min.x + edge_shrink);
    let max_edge = (a_max.x - edge_shrink).min(b_max.x - edge_shrink);
    if min_edge > max_edge {
        return false;
    }

    let a_d = (a_max.y - a_min.y) / (a_max.x - a_max.y);
    let a_k = a_min.y - a_d * a_min.x;

    let b_d = (b_max.y - b_min.y) / (b_max.x - b_max.y);
    let b_k = b_min.y - a_d * b_min.x;

    let a_min_y = a_d * min_edge + a_k;
    let a_max_y = a_d * max_edge + a_k;

    let b_min_y = b_d * min_edge + b_k;
    let b_max_y = b_d * max_edge + b_k;

    let delta_min = b_min_y - a_min_y;
    let delta_max = b_max_y - a_max_y;

    if delta_min * delta_max < 0.0 {
        return false;
    }

    let threshold = (allowed_step * 2.0).powi(2);

    delta_min * delta_min <= threshold || delta_max * delta_max <= threshold
}

const MAX_CONNECTING_POLYGONS: usize = 8;

fn find_connecting_polygons_in_tile(
    vertex_a: &Vec3,
    vertex_b: &Vec3,
    tile: &NavMeshTile,
    side: EdgeConnectionDirection,
    step_height: f32,
) -> (
    usize,
    [(u16, u8); MAX_CONNECTING_POLYGONS],
    [Vec2; MAX_CONNECTING_POLYGONS],
) {
    let mut connecting_polys = [(0, 0); MAX_CONNECTING_POLYGONS];
    let mut connection_area = [Vec2::ZERO; MAX_CONNECTING_POLYGONS];
    let mut count = 0;

    let (in_min, in_max) = calculate_slab_end_points(vertex_a, vertex_b, side);
    let in_pos = get_slab_position(vertex_a, side);

    for (poly_index, polygon) in tile.polygons.iter().enumerate() {
        // TODO: What if we just store a list of edge polygons?
        for (edge_index, edge) in tile.edges[poly_index].iter().enumerate() {
            let EdgeConnection::OffMesh(direction) = edge else {
                continue;
            };
            if *direction != side {
                continue;
            }

            let vertex_c = tile.vertices[polygon.indices[edge_index] as usize];
            let vertex_d =
                tile.vertices[polygon.indices[(edge_index + 1) % polygon.indices.len()] as usize];

            let edge_pos = get_slab_position(&vertex_c, side);

            if (in_pos - edge_pos).abs() > 0.01 {
                continue;
            }
            let (edge_min, edge_max) = calculate_slab_end_points(&vertex_c, &vertex_d, side);

            if check_slabs_overlap(in_min, in_max, edge_min, edge_max, 0.01, step_height) {
                continue;
            }

            if count < connecting_polys.len() {
                connecting_polys[count] = (poly_index as u16, edge_index as u8);
                connection_area[count] =
                    Vec2::new(in_min.x.max(edge_min.x), in_max.x.min(edge_max.x));
                count += 1;
            }
            break;
        }
    }

    (count, connecting_polys, connection_area)
}

pub(super) fn create_nav_mesh_data_from_poly_mesh(
    mesh: &PolyMesh,
    tile_coords: UVec2,
    nav_mesh_settings: &NavMeshSettings,
) -> NavMeshTile {
    // Slight worry that the compiler won't optimize this but damn, it's cool.
    let polygons = mesh
        .polygons
        .iter()
        .zip(mesh.edges.iter())
        .map(|(indices, edges)| {
            // Pre build internal links.
            let links = edges
                .iter()
                .enumerate()
                .filter_map(|(i, edge)| {
                    let EdgeConnection::Internal(other_polygon) = edge else {
                return None;
            };

                    Some(Link::Internal {
                        edge: i as u8,
                        neighbour_polygon: *other_polygon,
                    })
                })
                .collect();

            Polygon {
                links,
                indices: *indices,
            }
        })
        .collect();

    let tile_min_bound = nav_mesh_settings.get_tile_min_bound(tile_coords);
    let vertices = mesh
        .vertices
        .iter()
        .map(|vertex| {
            Vec3::new(
                tile_min_bound.x + vertex.x as f32 * nav_mesh_settings.cell_width,
                nav_mesh_settings.world_bottom_bound
                    + vertex.y as f32 * nav_mesh_settings.cell_height,
                tile_min_bound.y + vertex.z as f32 * nav_mesh_settings.cell_width,
            )
        })
        .collect();

    NavMeshTile {
        salt: 0,
        vertices,
        edges: mesh.edges.clone(),
        polygons,
    }
}
