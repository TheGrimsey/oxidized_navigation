use bevy::{
    math::Vec3Swizzles,
    prelude::{UVec2, Vec2, Vec3},
    utils::HashMap,
};
use smallvec::SmallVec;

use crate::{
    mesher::{EdgeConnection, EdgeConnectionDirection, VERTICES_IN_TRIANGLE},
    NavMeshSettings, Area,
};

use super::mesher::PolyMesh;

/// Representation of a link between different polygons either internal to the tile or external (crossing over to another tile).
#[derive(Clone, Copy, Debug)]
pub enum Link {
    Internal {
        /// Edge on self polygon.
        edge: u8,
        /// Index of polygon this polygon is linked to.
        neighbour_polygon: u16,
    },
    External {
        /// Edge on self polygon.
        edge: u8,
        /// Index of polygon this polygon is linked to.
        neighbour_polygon: u16,
        /// Direction toward the neighbour polygon's tile.
        direction: EdgeConnectionDirection,
        /// Min % of this edge that connects to the linked polygon.
        bound_min: u8, // % bound of edge that links to this.
        // MAx % of this edge that connects to the linked polygon.
        bound_max: u8, // For example: 10% -> 50% = the connected edge covers 10% from vertex A to B to 50%.
    },
}

/// A polygon within a nav-mesh tile.
#[derive(Debug)]
pub struct Polygon {
    pub indices: [u32; VERTICES_IN_TRIANGLE],
    pub links: SmallVec<[Link; VERTICES_IN_TRIANGLE]>, // This becomes a mess memory wise with a ton of different small objects around.
    pub area: Area,
}

/*
*   Polygons make up a form of graph, linking to other polygons (which could be on another mesh)
*/

/// A single nav-mesh tile.
#[derive(Debug)]
pub struct NavMeshTile {
    /// Vertices in world space.
    pub vertices: Vec<Vec3>,
    pub polygons: Vec<Polygon>,
    pub edges: Vec<[EdgeConnection; VERTICES_IN_TRIANGLE]>,
}
impl NavMeshTile {
    /// Returns the closest point on ``polygon`` to ``position``.
    pub fn get_closest_point_in_polygon(&self, polygon: &Polygon, position: Vec3) -> Vec3 {
        let vertices = polygon.indices.map(|index| self.vertices[index as usize]);

        if let Some(height) = get_height_in_triangle(&vertices, position) {
            return Vec3::new(position.x, height, position.z);
        }

        closest_point_on_edges(&vertices, position)
    }
}

/// Container for all nav-mesh tiles. Used for pathfinding queries.
///
/// Call [crate::query::find_path] to run pathfinding algorithm.
#[derive(Default)]
pub struct NavMeshTiles {
    pub(super) tiles: HashMap<UVec2, NavMeshTile>,
    pub(super) tile_generations: HashMap<UVec2, u64>,
}

impl NavMeshTiles {
    /// Returns a [HashMap] containing all tiles in the nav-mesh.
    pub fn get_tiles(&self) -> &HashMap<UVec2, NavMeshTile> {
        &self.tiles
    }

    pub(super) fn add_tile(
        &mut self,
        tile_coord: UVec2,
        mut tile: NavMeshTile,
        nav_mesh_settings: &NavMeshSettings,
    ) {
        let previous_tile_existed = self.tiles.contains_key(&tile_coord);

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

        // Insert tile.
        self.tiles.insert(tile_coord, tile);
    }

    pub(super) fn remove_tile(&mut self, tile_coord: UVec2) {
        if tile_coord.x > 0 {
            let direction = EdgeConnectionDirection::XNegative;
            let neighbour_coord = direction.offset(tile_coord);

            if let Some(neighbour) = self.tiles.get_mut(&neighbour_coord) {
                remove_links_to_direction(neighbour, EdgeConnectionDirection::XPositive);
            }
        }

        if tile_coord.x < u32::MAX {
            let direction = EdgeConnectionDirection::XPositive;
            let neighbour_coord = direction.offset(tile_coord);

            if let Some(neighbour) = self.tiles.get_mut(&neighbour_coord) {
                remove_links_to_direction(neighbour, EdgeConnectionDirection::XNegative);
            }
        }

        if tile_coord.y > 0 {
            let direction = EdgeConnectionDirection::ZNegative;
            let neighbour_coord = direction.offset(tile_coord);

            if let Some(neighbour) = self.tiles.get_mut(&neighbour_coord) {
                remove_links_to_direction(neighbour, EdgeConnectionDirection::ZPositive);
            }
        }

        if tile_coord.y < u32::MAX {
            let direction = EdgeConnectionDirection::ZPositive;
            let neighbour_coord = direction.offset(tile_coord);

            if let Some(neighbour) = self.tiles.get_mut(&neighbour_coord) {
                remove_links_to_direction(neighbour, EdgeConnectionDirection::ZNegative);
            }
        }

        self.tiles.remove(&tile_coord);
    }

    /// Returns the closest polygon in a box around ``center`` as a tuple of (tile coordinate, polygon index, position on triangle).
    pub fn find_closest_polygon_in_box(
        &self,
        nav_mesh_settings: &NavMeshSettings,
        center: Vec3,
        half_extents: f32,
    ) -> Option<(UVec2, u16, Vec3)> {
        let min = center - half_extents;
        let max = center + half_extents;

        let min_tile = nav_mesh_settings.get_tile_containing_position(min.xz());
        let max_tile = nav_mesh_settings.get_tile_containing_position(max.xz());

        let mut out_polygon = None;
        let mut out_distance = f32::INFINITY;
        for x in min_tile.x..=max_tile.x {
            for y in min_tile.y..=max_tile.y {
                let tile_coords = UVec2::new(x, y);
                if let Some(tile) = self.tiles.get(&tile_coords) {
                    for (poly_i, polygon) in tile.polygons.iter().enumerate() {
                        let closest_point = tile.get_closest_point_in_polygon(polygon, center);
                        let closest_distance = closest_point.distance_squared(center);

                        if closest_distance < out_distance {
                            out_distance = closest_distance;
                            out_polygon = Some((tile_coords, poly_i as u16, closest_point));
                        }
                    }
                }
            }
        }

        out_polygon
    }
}

fn get_height_in_triangle(vertices: &[Vec3; VERTICES_IN_TRIANGLE], position: Vec3) -> Option<f32> {
    if !in_polygon(vertices, position) {
        return None;
    }

    if let Some(height) =
        closest_height_in_triangle(vertices[0], vertices[1], vertices[2], position)
    {
        return Some(height);
    }

    // We only hit this if we are ON an edge. Unlikely to happen.
    let closest = closest_point_on_edges(vertices, position);

    Some(closest.y)
}

fn closest_height_in_triangle(a: Vec3, b: Vec3, c: Vec3, position: Vec3) -> Option<f32> {
    let v0 = c - a;
    let v1 = b - a;
    let v2 = position - a;

    let mut denom = v0.x * v1.z - v0.z * v1.x;
    const EPS: f32 = 0.000001;
    if denom.abs() < EPS {
        return None;
    }

    let mut u = v1.z * v2.x - v1.x * v2.z;
    let mut v = v0.x * v2.z - v0.z * v2.x;

    if denom < 0.0 {
        denom = -denom;
        u = -u;
        v = -v;
    }

    if u >= 0.0 && v >= 0.0 && (u + v) <= denom {
        return Some(a.y + (v0.y * u + v1.y * v) / denom);
    }

    None
}

fn closest_point_on_edges(vertices: &[Vec3; VERTICES_IN_TRIANGLE], position: Vec3) -> Vec3 {
    let mut d_min = f32::INFINITY;
    let mut t_min = 0.0;

    let mut edge_min = Vec3::ZERO;
    let mut edge_max = Vec3::ZERO;

    for i in 0..vertices.len() {
        let prev = (vertices.len() + i - 1) % vertices.len();

        let (d, t) = distance_point_to_segment_2d(position, vertices[prev], vertices[i]);
        if d < d_min {
            d_min = d;
            t_min = t;
            edge_min = vertices[prev];
            edge_max = vertices[i];
        }
    }

    edge_min.lerp(edge_max, t_min)
}

fn distance_point_to_segment_2d(point: Vec3, seg_a: Vec3, seg_b: Vec3) -> (f32, f32) {
    let ba_x = seg_b.x - seg_a.x;
    let ba_z = seg_b.z - seg_a.z;

    let dx = point.x - seg_a.x;
    let dz = point.z - seg_a.z;

    let d = ba_x * ba_x + ba_z * ba_z;
    let mut t = ba_x * dx + ba_z * dz;
    if d > 0.0 {
        t /= d;
    }
    t = t.clamp(0.0, 1.0);

    let dx = seg_a.x + t * ba_x - point.x;
    let dz = seg_a.z + t * ba_z - point.z;

    (dx * dx + dz * dz, t)
}

fn in_polygon(vertices: &[Vec3; VERTICES_IN_TRIANGLE], position: Vec3) -> bool {
    let mut inside = false;

    for i in 0..vertices.len() {
        let prev = (vertices.len() + i - 1) % vertices.len();

        let a = vertices[i];
        let b = vertices[prev];
        if ((a.z > position.z) != (b.z > position.z))
            && (position.x < (b.x - a.x) * (position.z - a.z) / (b.z - a.z) + a.x)
        {
            inside = !inside;
        }
    }

    inside
}

fn remove_links_to_direction(tile: &mut NavMeshTile, remove_direction: EdgeConnectionDirection) {
    for polygon in tile.polygons.iter_mut() {
        polygon.links.retain(|link| match link {
            Link::Internal { .. } => true,
            Link::External { direction, .. } => *direction != remove_direction,
        });
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
        if remove_existing_links {
            polygon.links.retain(|link| match link {
                Link::Internal { .. } => true,
                Link::External { direction, .. } => *direction != neighbour_direction,
            });
        }

        for (edge_index, edge) in tile.edges[poly_index].iter().enumerate() {
            let EdgeConnection::External(edge_direction) = edge else {
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
                let neighbour_polygon = connected_polys[i];
                let area = connection_areas[i];

                let (mut bound_min, mut bound_max) = if neighbour_to_self_direction
                    == EdgeConnectionDirection::XNegative
                    || neighbour_to_self_direction == EdgeConnectionDirection::XPositive
                {
                    let min = (area.x - vertex_a.z) / (vertex_b.z - vertex_a.z);
                    let max = (area.y - vertex_a.z) / (vertex_b.z - vertex_a.z);

                    (min, max)
                } else {
                    let min = (area.x - vertex_a.x) / (vertex_b.x - vertex_a.x);
                    let max = (area.y - vertex_a.x) / (vertex_b.x - vertex_a.x);

                    (min, max)
                };

                if bound_min > bound_max {
                    std::mem::swap(&mut bound_min, &mut bound_max);
                }

                let min_byte = (bound_min.clamp(0.0, 1.0) * 255.0).round() as u8;
                let max_byte = (bound_max.clamp(0.0, 1.0) * 255.0).round() as u8;

                polygon.links.push(Link::External {
                    edge: edge_index as u8,
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
            let min = vertex_a.zy();
            let max = vertex_b.zy();

            (min, max)
        } else {
            let min = vertex_b.zy();
            let max = vertex_a.zy();

            (min, max)
        }
    } else if vertex_a.x < vertex_b.x {
        let min = vertex_a.xy();
        let max = vertex_b.xy();

        (min, max)
    } else {
        let min = vertex_b.xy();
        let max = vertex_a.xy();

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

    let a_d = (a_max.y - a_min.y) / (a_max.x - a_min.x);
    let a_k = a_min.y - a_d * a_min.x;

    let b_d = (b_max.y - b_min.y) / (b_max.x - b_min.x);
    let b_k = b_min.y - a_d * b_min.x;

    let a_min_y = a_d * min_edge + a_k;
    let a_max_y = a_d * max_edge + a_k;

    let b_min_y = b_d * min_edge + b_k;
    let b_max_y = b_d * max_edge + b_k;

    let delta_min = b_min_y - a_min_y;
    let delta_max = b_max_y - a_max_y;

    if delta_min * delta_max < 0.0 {
        return true;
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
    [u16; MAX_CONNECTING_POLYGONS],
    [Vec2; MAX_CONNECTING_POLYGONS],
) {
    let mut connecting_polys = [0; MAX_CONNECTING_POLYGONS];
    let mut connection_area = [Vec2::ZERO; MAX_CONNECTING_POLYGONS];
    let mut count = 0;

    let (in_min, in_max) = calculate_slab_end_points(vertex_a, vertex_b, side);
    let in_pos = get_slab_position(vertex_a, side);

    for (poly_index, polygon) in tile.polygons.iter().enumerate() {
        for (edge_index, edge) in tile.edges[poly_index].iter().enumerate() {
            let EdgeConnection::External(direction) = edge else {
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

            if !check_slabs_overlap(in_min, in_max, edge_min, edge_max, 0.01, step_height) {
                continue;
            }

            if count < connecting_polys.len() {
                connecting_polys[count] = poly_index as u16;
                connection_area[count] =
                    Vec2::new(in_min.x.max(edge_min.x), in_max.x.min(edge_max.x));
                count += 1;
            }
            break;
        }
    }

    (count, connecting_polys, connection_area)
}

pub(super) fn create_nav_mesh_tile_from_poly_mesh(
    poly_mesh: PolyMesh,
    tile_coord: UVec2,
    nav_mesh_settings: &NavMeshSettings,
) -> NavMeshTile {
    // Slight worry that the compiler won't optimize this but damn, it's cool.
    let polygons = poly_mesh
        .polygons
        .iter()
        .zip(poly_mesh.edges.iter())
        .zip(poly_mesh.areas.iter())
        .map(|((indices, edges), area)| {
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
                area: *area,
            }
        })
        .collect();

    let tile_origin = nav_mesh_settings.get_tile_origin_with_border(tile_coord);
    let vertices = poly_mesh
        .vertices
        .iter()
        .map(|vertex| {
            Vec3::new(
                tile_origin.x + vertex.x as f32 * nav_mesh_settings.cell_width,
                nav_mesh_settings.world_bottom_bound
                    + vertex.y as f32 * nav_mesh_settings.cell_height,
                tile_origin.y + vertex.z as f32 * nav_mesh_settings.cell_width,
            )
        })
        .collect();

    NavMeshTile {
        vertices,
        edges: poly_mesh.edges,
        polygons,
    }
}
