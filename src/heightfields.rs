use std::{cmp::Ordering, ops::Div, sync::Arc};

use bevy::{prelude::*, math::Vec3A};
use parry3d::shape::HeightField;
use smallvec::SmallVec;

use crate::{conversion::Triangles, Area};

use super::{get_neighbour_index, NavMeshSettings};

#[derive(Default, Clone, Debug)]
struct HeightSpan {
    min: u16,
    max: u16,
    traversable: bool,
    area: Option<Area>,
}

#[derive(Default, Clone)]
struct VoxelCell {
    spans: SmallVec<[HeightSpan; 2]>, // Bottom to top.
}

#[derive(Default)]
pub struct VoxelizedTile {
    cells: Box<[VoxelCell]>, // len = tiles_along_width^2. Laid out X to Y
}

#[derive(Default, Clone, Debug)]
pub(super) struct OpenCell {
    pub(super) spans: SmallVec<[OpenSpan; 1]>,
}

// Like a HeightSpan but representing open walkable areas (empty space with floor & height >= walkable_height
#[derive(Default, Clone, Copy, Debug)]
pub(super) struct OpenSpan {
    pub(super) min: u16,
    pub(super) max: Option<u16>,
    pub(super) neighbours: [Option<u16>; 4],
    pub(super) tile_index: usize, // The index of this span in the whole tile.
    pub(super) region: u16,       // Region if non-zero.
    area: Option<Area>,           // TODO: Ideally we don't want store this here. It's only here to be copied over to [OpenTile::areas] & bumps up the OpenSpan size from 32b to 40b.
}

#[derive(Default, Debug)]
pub struct OpenTile {
    pub(super) cells: Vec<OpenCell>, // len = tiles_along_width^2. Laid out X to Y
    pub(super) distances: Box<[u16]>, // Distances used in watershed. One per span. Use tile_index to go from span to distance.
    pub(super) areas: Box<[Option<Area>]>,
    pub(super) max_distance: u16,
    pub(super) span_count: usize, // Total spans in all cells.
    pub(super) max_regions: u16,
}

pub(super) struct TriangleCollection {
    pub(super) transform: Transform,
    pub(super) triangles: Triangles,
    pub(super) area: Option<Area>,
}

pub struct HeightFieldCollection {
    pub transform: Transform,
    pub heightfield: Arc<HeightField>,
    pub area: Option<Area>,
}

pub(super) fn build_heightfield_tile(
    tile_coord: UVec2,
    triangle_collections: &[TriangleCollection],
    heightfields: &[HeightFieldCollection],
    nav_mesh_settings: &NavMeshSettings,
) -> VoxelizedTile {
    let tile_side = nav_mesh_settings.get_tile_side_with_border();
    let mut voxel_tile = VoxelizedTile {
        cells: vec![VoxelCell::default(); tile_side.pow(2)].into_boxed_slice(),
    };

    let tile_max_bound = IVec3::new((tile_side - 1) as i32, 0, (tile_side - 1) as i32);

    let tile_origin = nav_mesh_settings.get_tile_origin_with_border(tile_coord);
    let tile_origin = Vec3::new(
        tile_origin.x,
        nav_mesh_settings.world_bottom_bound,
        tile_origin.y,
    );

    let mut translated_vertices = Vec::default();

    for collection in triangle_collections.iter() {
        // TODO: This might be wrong for xpbd or custom parry3d colliders, but I can't figure out a nice way to know whether or not we're actually dealing with a rapier3d collider.
        let transform = collection.transform.with_scale(Vec3::ONE); // The collider returned from rapier already has scale applied to it, so we reset it here.

        match &collection.triangles {
            Triangles::Triangle(vertices) => {
                let translated_vertices =
                    vertices.map(|vertex| transform.transform_point(vertex) - tile_origin);

                process_triangle(
                    Vec3A::from(translated_vertices[0]),
                    Vec3A::from(translated_vertices[1]),
                    Vec3A::from(translated_vertices[2]),
                    nav_mesh_settings,
                    tile_max_bound,
                    tile_side,
                    &mut voxel_tile.cells,
                    collection.area,
                );
            }
            Triangles::TriMesh(vertices, triangles) => {
                translated_vertices.clear();
                translated_vertices.extend(
                    vertices
                        .iter()
                        .map(|vertex| transform.transform_point(*vertex) - tile_origin),
                ); // Transform vertices.

                for triangle in triangles.iter() {
                    let a = Vec3A::from(translated_vertices[triangle[0] as usize]);
                    let b = Vec3A::from(translated_vertices[triangle[1] as usize]);
                    let c = Vec3A::from(translated_vertices[triangle[2] as usize]);

                    process_triangle(
                        a,
                        b,
                        c,
                        nav_mesh_settings,
                        tile_max_bound,
                        tile_side,
                        &mut voxel_tile.cells,
                        collection.area,
                    );
                }
            }
        }
    }

    for collection in heightfields.iter() {
        // TODO: This might be wrong for xpbd or custom parry3d colliders, but I can't figure out a nice way to know whether or not we're actually dealing with a rapier3d collider.
        let transform = collection.transform.with_scale(Vec3::ONE); // The collider returned from rapier already has scale applied to it, so we reset it here.

        for triangle in collection.heightfield.triangles() {
            let a = Vec3A::from(transform.transform_point(Vec3::new(triangle.a.x, triangle.a.y, triangle.a.z))
                - tile_origin);
            let b = Vec3A::from(transform.transform_point(Vec3::new(triangle.b.x, triangle.b.y, triangle.b.z))
                - tile_origin);
            let c = Vec3A::from(transform.transform_point(Vec3::new(triangle.c.x, triangle.c.y, triangle.c.z))
                - tile_origin);

            process_triangle(
                a,
                b,
                c,
                nav_mesh_settings,
                tile_max_bound,
                tile_side,
                &mut voxel_tile.cells,
                collection.area,
            );
        }
    }

    voxel_tile
}

fn process_triangle(
    a: Vec3A,
    b: Vec3A,
    c: Vec3A,
    nav_mesh_settings: &NavMeshSettings,
    tile_max_bound: IVec3,
    tile_side: usize,
    voxel_cells: &mut [VoxelCell],
    area: Option<Area>,
) {
    let min_bound = a.min(b).min(c).div(nav_mesh_settings.cell_width).as_ivec3();
    let max_bound = a.max(b).max(c).div(nav_mesh_settings.cell_width).as_ivec3();

    // Check if triangle is completely outside the tile.
    if max_bound.x < 0
        || max_bound.z < 0
        || min_bound.x > tile_max_bound.x
        || min_bound.z > tile_max_bound.z
    {
        return;
    }

    let clamped_bound_min = min_bound.max(IVec3::ZERO);
    let clamped_bound_max = max_bound.min(tile_max_bound);
    let traversable = is_triangle_traversable(a, b, c, nav_mesh_settings);
    let vertices = [a, b, c];

    // For cache reasons we go.
    // --> X
    // Z
    // |
    // V
    // X is column. Z is row.
    // Which means we iterate Z first.
    for z in clamped_bound_min.z..=clamped_bound_max.z {
        let row_clip_min = z as f32 * nav_mesh_settings.cell_width;
        let row_clip_max = row_clip_min + nav_mesh_settings.cell_width;

        // Clip polygon to the row.
        // TODO: This is awful & too complicated.
        let (row_min_clip_vert_count, row_min_clip_verts) = divide_polygon(&vertices, row_clip_min, 2, false);
        let (row_vert_count, row_verts) = divide_polygon(&row_min_clip_verts[..row_min_clip_vert_count],row_clip_max, 2, true);
        if row_vert_count < 3 {
            continue;
        }

        // Calculate the column footprint of the row.
        let mut column_min_vert_x = row_verts[0].x;
        let mut column_max_vert_x = row_verts[0].x;
        for vertex in row_verts.iter().take(row_vert_count).skip(1) {
            column_min_vert_x = column_min_vert_x.min(vertex.x);
            column_max_vert_x = column_max_vert_x.max(vertex.x);
        }
        let column_min = ((column_min_vert_x / nav_mesh_settings.cell_width) as i32).max(0);
        let column_max =
            ((column_max_vert_x / nav_mesh_settings.cell_width) as i32).min((tile_side - 1) as i32);

        for x in column_min..=column_max {
            let column_clip_min = x as f32 * nav_mesh_settings.cell_width;
            let column_clip_max = column_clip_min + nav_mesh_settings.cell_width;

            // Clip polygon to column.
            let (column_min_clip_vert_count, column_min_clip_verts) = divide_polygon(&row_verts[..row_vert_count], column_clip_min, 0, false);
            let (column_vert_count, column_verts) = divide_polygon(&column_min_clip_verts[..column_min_clip_vert_count],column_clip_max, 0, true);
            if column_vert_count < 3 {
                continue;
            }

            let mut square_min_height = column_verts[0].y;
            let mut square_max_height = column_verts[0].y;
            for vertex in column_verts.iter().take(column_vert_count).skip(1) {
                square_min_height = square_min_height.min(vertex.y);
                square_max_height = square_max_height.max(vertex.y);
            }

            square_min_height = square_min_height.max(0.0);
            if square_max_height < 0.0 {
                continue;
            }

            let min_height = (square_min_height / nav_mesh_settings.cell_height) as u16;
            let max_height = (square_max_height / nav_mesh_settings.cell_height) as u16;

            let index = x as usize + z as usize * tile_side;
            let cell = &mut voxel_cells[index];

            let mut new_span = HeightSpan {
                min: min_height,
                max: max_height,
                traversable,
                area,
            };

            if cell.spans.is_empty() {
                cell.spans.push(new_span);
                continue;
            }
            // We need to go over all existing ones.
            let mut i = 0;
            while i < cell.spans.len() {
                let existing_span = &cell.spans[i];
                if existing_span.min > new_span.max {
                    // i is beyond the new span. We can insert!
                    break;
                } else if existing_span.max < new_span.min {
                    // i is before the new span. Continue until we hit one that isn't.
                    i += 1;
                    continue;
                } else {
                    match existing_span.max.cmp(&new_span.max) {
                        Ordering::Greater => {
                            new_span.traversable = existing_span.traversable;
                            new_span.area = existing_span.area;
                        }
                        Ordering::Equal => {
                            new_span.traversable |= existing_span.traversable;
                            // Higher area number has higher priority.
                            new_span.area = new_span.area.max(existing_span.area);
                        }
                        Ordering::Less => {}
                    }

                    // Extend new span to existing span's size.
                    if existing_span.min < new_span.min {
                        new_span.min = existing_span.min;
                    }
                    if existing_span.max > new_span.max {
                        new_span.max = existing_span.max;
                    }

                    cell.spans.remove(i);
                }
            }
            cell.spans.insert(i, new_span);
        }
    }
}

fn is_triangle_traversable(
    a: Vec3A,
    b: Vec3A,
    c: Vec3A,
    nav_mesh_settings: &NavMeshSettings,
) -> bool {
    let ab = b - a;
    let ac = c - a;
    let normal = ab.cross(ac).normalize();
    let slope = normal.dot(Vec3A::Y).acos();

    slope < nav_mesh_settings.max_traversable_slope_radians
}

/*
*   This function takes in a polygon (of max 7 vertices), an line on which to divide it, and an axis.
*   It then returns the left polygon's vertex count, the left polygon's vertices,
*   the right polygon's vertex count, and the right polygon's vertices.
*/
fn divide_polygon(
    vertices: &[Vec3A],
    clip_line: f32,
    axis: usize,
    keep_left: bool
) -> (usize, [Vec3A; 7]) {
    let mut delta_from_line = [0.0; 7];
    // This loop determines which side of the line the vertex is on.
    for (i, vertex) in vertices.iter().enumerate() {
        delta_from_line[i] = clip_line - vertex[axis];
    }

    // TODO: We always use one of these options. Does it make sense to even return the other?
    let mut polygon_left = [Vec3A::ZERO; 7];
    let mut polygon_right = [Vec3A::ZERO; 7];

    let mut verts_left = 0;
    let mut verts_right = 0;

    for i in 0..vertices.len() {
        let previous = (vertices.len() - 1 + i) % vertices.len(); // j is i-1 wrapped.

        let in_a = delta_from_line[previous] >= 0.0;
        let in_b = delta_from_line[i] >= 0.0;

        // Check if both vertices are on the same side of the line.
        if in_a != in_b {
            // We slide the vertex along to the edge.
            let slide = delta_from_line[previous] / (delta_from_line[previous] - delta_from_line[i]);

            polygon_left[verts_left] = vertices[previous] + (vertices[i] - vertices[previous]) * slide;
            polygon_right[verts_right] = polygon_left[verts_left];
            verts_left += 1;
            verts_right += 1;

            if delta_from_line[i] > 0.0 {
                polygon_left[verts_left] = vertices[i];
                verts_left += 1;
            } else if delta_from_line[i] < 0.0 {
                polygon_right[verts_right] = vertices[i];
                verts_right += 1;
            }
        } else {
            if delta_from_line[i] >= 0.0 {
                polygon_left[verts_left] = vertices[i];
                verts_left += 1;

                if delta_from_line[i] != 0.0 {
                    continue;
                }
            }
            polygon_right[verts_right] = vertices[i];
            verts_right += 1;
        }
    }

    if keep_left {
        (verts_left, polygon_left)
    } else {
        (verts_right, polygon_right)
    }
}

pub fn build_open_heightfield_tile(
    voxelized_tile: VoxelizedTile,
    nav_mesh_settings: &NavMeshSettings,
) -> OpenTile {
    let mut cells = vec![OpenCell::default(); voxelized_tile.cells.len()];
    let mut span_count = 0;

    // First we create open spaces.
    for (i, cell) in voxelized_tile
        .cells
        .iter()
        .enumerate()
        .filter(|(_, cell)| !cell.spans.is_empty())
    {
        let open_spans = &mut cells[i].spans;

        let mut iter = cell.spans.iter().peekable();
        while let Some(span) = iter.next() {
            let area = if span.traversable { span.area } else { None };

            if let Some(next_span) = iter.peek() {
                // Need to check if space is large enough.
                if next_span.min - span.max >= nav_mesh_settings.walkable_height {
                    open_spans.push(OpenSpan {
                        min: span.max,
                        max: Some(next_span.min),
                        area,
                        ..Default::default()
                    });
                }
            } else {
                // None above. This is an unbounded open space.
                open_spans.push(OpenSpan {
                    min: span.max,
                    max: None,
                    area,
                    ..Default::default()
                });
            }
        }
        span_count += open_spans.len();
    }

    // Create Open Tile.
    let mut open_tile = OpenTile {
        cells,
        distances: vec![u16::MAX; span_count].into_boxed_slice(),
        areas: vec![None; span_count].into_boxed_slice(),
        max_distance: 0,
        span_count,
        max_regions: 0,
    };

    // Assign tile_index & copy over areas.
    let mut tile_index = 0;
    for cell in open_tile.cells.iter_mut() {
        for span in cell.spans.iter_mut() {
            span.tile_index = tile_index;

            open_tile.areas[tile_index] = span.area;

            tile_index += 1;
        }
    }
    
    {
        #[cfg(feature = "trace")]
        let _span = info_span!("Link neighbours").entered();
        link_neighbours(&mut open_tile, nav_mesh_settings);
    }

    open_tile
}

fn link_neighbours(open_tile: &mut OpenTile, nav_mesh_settings: &NavMeshSettings) {
    let mut neighbour_spans = Vec::with_capacity(3);

    let tile_side = nav_mesh_settings.get_tile_side_with_border();
    for i in 0..open_tile.cells.len() {
        if open_tile.cells[i].spans.is_empty() {
            continue;
        }

        let row = i / tile_side;
        let column = i % tile_side;

        let neighbour_index = [
            if column > 0 { Some(i - 1) } else { None },
            if row < (tile_side - 1) { Some(i + tile_side) } else { None },
            if column < (tile_side - 1) { Some(i + 1)} else { None },
            if row > 0 { Some(i - tile_side) } else { None }
        ];

        // For each direct neighbour.
        for (neighbour, neighbour_index) in neighbour_index.into_iter().enumerate().filter_map(|(i, index)| Some(i).zip(index)) {
            neighbour_spans.clear();
            neighbour_spans.extend(
                open_tile.cells[neighbour_index]
                    .spans
                    .iter()
                    .map(|span| (span.min, span.max)),
            );

            for span in open_tile.cells[i].spans.iter_mut() {
                for (i, (min, max)) in neighbour_spans.iter().enumerate() {
                    if let Some((max, span_max)) = max.zip(span.max) {
                        let gap = span_max.min(max).abs_diff(span.min.max(*min));
                        if gap < nav_mesh_settings.walkable_height {
                            continue;
                        }
                    }
    
                    if min.abs_diff(span.min) < nav_mesh_settings.step_height {
                        span.neighbours[neighbour] = Some(i as u16);
                        break;
                    }
                }
            }
        }
    }
}

pub fn erode_walkable_area(open_tile: &mut OpenTile, nav_mesh_settings: &NavMeshSettings) {
    let tile_side = nav_mesh_settings.get_tile_side_with_border();
    // Mark boundary cells.
    for (i, cell) in open_tile.cells.iter().enumerate() {
        for span in cell.spans.iter() {
            let area = open_tile.areas[span.tile_index];

            if area.is_none() {
                open_tile.distances[span.tile_index] = 0;
                continue;
            }

            let all_neighbours = span.neighbours.iter().enumerate().all(|(dir, neighbour)| {
                if let Some(neighbour) = neighbour {
                    let neighbour_index = get_neighbour_index(tile_side, i, dir);
                    let neighbour = &open_tile.cells[neighbour_index].spans[*neighbour as usize];

                    open_tile.areas[neighbour.tile_index].is_some() // Any neighbour not marked as unwalkable.
                } else {
                    false
                }
            });

            open_tile.distances[span.tile_index] = if all_neighbours { u16::MAX } else { 0 };
        }
    }

    filter_tile(open_tile, nav_mesh_settings);

    // Any cell within 2*walkable_radius is considered unwalkable. This ensures characters won't clip into walls.
    let threshold = nav_mesh_settings.walkable_radius * 2;
    for i in 0..open_tile.span_count {
        if open_tile.distances[i] < threshold {
            open_tile.areas[i] = None;
        }
    }
}

pub fn calculate_distance_field(open_tile: &mut OpenTile, nav_mesh_settings: &NavMeshSettings) {
    let tile_side = nav_mesh_settings.get_tile_side_with_border();
    // Mark boundary cells.
    for (i, cell) in open_tile.cells.iter().enumerate() {
        for span in cell.spans.iter() {
            let area = open_tile.areas[span.tile_index];

            let all_neighbours = span.neighbours.iter().enumerate().all(|(dir, neighbour)| {
                if let Some(neighbour) = neighbour {
                    let neighbour_index = get_neighbour_index(tile_side, i, dir);
                    let neighbour = &open_tile.cells[neighbour_index].spans[*neighbour as usize];

                    open_tile.areas[neighbour.tile_index] == area // Only neighbours of same area.
                } else {
                    false
                }
            });

            open_tile.distances[span.tile_index] = if all_neighbours { u16::MAX } else { 0 };
        }
    }

    filter_tile(open_tile, nav_mesh_settings);

    open_tile.max_distance = *open_tile.distances.iter().max().unwrap_or(&0);

    // Box blur. If you're reading this, why?
    let threshold = 2;

    let mut blurred = vec![0; open_tile.distances.len()].into_boxed_slice();

    for (i, cell) in open_tile.cells.iter().enumerate() {
        for span in cell.spans.iter() {
            let distance = open_tile.distances[span.tile_index];
            if distance <= threshold {
                blurred[span.tile_index] = distance;
                continue;
            }

            let mut d = distance;
            for dir in 0..4 {
                let Some(index) = span.neighbours[dir] else {
                    d += distance * 2;
                    continue;
                };

                let other_cell_index = get_neighbour_index(tile_side, i, dir);
                let other_span = &open_tile.cells[other_cell_index].spans[index as usize];

                d += open_tile.distances[other_span.tile_index];

                let next_dir = (dir + 1) & 0x3;
                let Some(index) = other_span.neighbours[next_dir] else {
                    d += distance;
                    continue;
                };

                let other_cell_index =
                    get_neighbour_index(tile_side, other_cell_index, next_dir);

                let other_span = &open_tile.cells[other_cell_index].spans[index as usize];

                d += open_tile.distances[other_span.tile_index];
            }

            // Apply distance change.
            blurred[span.tile_index] = (d + 5) / 9;
        }
    }

    open_tile.distances = blurred;
    // End Box Blur
}

fn filter_tile(open_tile: &mut OpenTile, nav_mesh_settings: &NavMeshSettings) {
    let tile_side = nav_mesh_settings.get_tile_side_with_border();
    // Pass 1.
    for (i, cell) in open_tile.cells.iter().enumerate() {
        for span in cell.spans.iter() {
            let mut distance = open_tile.distances[span.tile_index];

            if let Some(span_index) = span.neighbours[0] {
                // (-1, 0)
                let other_cell_index = i - 1;
                let other_span = &open_tile.cells[other_cell_index].spans[span_index as usize];

                let other_distance = open_tile.distances[other_span.tile_index] + 2;
                if other_distance < distance {
                    distance = other_distance;
                }

                // (-1, -1)
                if let Some(span_index) = other_span.neighbours[3] {
                    let other_cell_index = other_cell_index - tile_side;
                    let other_span = &open_tile.cells[other_cell_index].spans[span_index as usize];

                    let other_distance = open_tile.distances[other_span.tile_index] + 3;
                    if other_distance < distance {
                        distance = other_distance;
                    }
                }
            }

            if let Some(span_index) = span.neighbours[3] {
                // (0, -1)
                let other_cell_index = i - tile_side;
                let other_span = &open_tile.cells[other_cell_index].spans[span_index as usize];

                let other_distance = open_tile.distances[other_span.tile_index] + 2;
                if other_distance < distance {
                    distance = other_distance;
                }

                // (1, -1)
                if let Some(span_index) = other_span.neighbours[2] {
                    let other_cell_index = other_cell_index + 1;
                    let other_span = &open_tile.cells[other_cell_index].spans[span_index as usize];

                    let other_distance = open_tile.distances[other_span.tile_index] + 3;
                    if other_distance < distance {
                        distance = other_distance;
                    }
                }
            }

            // Apply distance change.
            open_tile.distances[span.tile_index] = distance;
        }
    }

    // Pass 2
    for (i, cell) in open_tile.cells.iter().enumerate().rev() {
        for span in cell.spans.iter() {
            let mut distance = open_tile.distances[span.tile_index];

            if let Some(span_index) = span.neighbours[2] {
                // (1, 0)
                let other_cell_index = i + 1;
                let other_span = &open_tile.cells[other_cell_index].spans[span_index as usize];

                let other_distance = open_tile.distances[other_span.tile_index] + 2;
                if other_distance < distance {
                    distance = other_distance;
                }

                // (1, 1)
                if let Some(span_index) = other_span.neighbours[1] {
                    let other_cell_index = other_cell_index + tile_side;
                    let other_span = &open_tile.cells[other_cell_index].spans[span_index as usize];

                    let other_distance = open_tile.distances[other_span.tile_index] + 3;
                    if other_distance < distance {
                        distance = other_distance;
                    }
                }
            }

            if let Some(span_index) = span.neighbours[1] {
                // (0, 1)
                let other_cell_index = i + tile_side;
                let other_span = &open_tile.cells[other_cell_index].spans[span_index as usize];

                let other_distance = open_tile.distances[other_span.tile_index] + 2;
                if other_distance < distance {
                    distance = other_distance;
                }

                // (-1, 1)
                if let Some(span_index) = other_span.neighbours[0] {
                    let other_cell_index = other_cell_index - 1;
                    let other_span = &open_tile.cells[other_cell_index].spans[span_index as usize];

                    let other_distance = open_tile.distances[other_span.tile_index] + 3;
                    if other_distance < distance {
                        distance = other_distance;
                    }
                }
            }

            // Apply distance change.
            open_tile.distances[span.tile_index] = distance;
        }
    }
}
