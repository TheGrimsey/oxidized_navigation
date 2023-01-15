use std::{cmp::Ordering, ops::Div};

use bevy::{
    prelude::{GlobalTransform, IVec3, UVec2, Vec3},
};

use super::{
    get_neighbour_index, NavMeshSettings
};

#[derive(Default, Clone, Debug)]
struct HeightSpan {
    min: u16,
    max: u16,
    traversable: bool,
}

#[derive(Default, Clone)]
struct VoxelCell {
    spans: Vec<HeightSpan>, // Bottom to top.
}

#[derive(Default)]
pub struct VoxelizedTile {
    cells: Vec<VoxelCell>, // len = tiles_along_width^2. Laid out X to Y
}

#[derive(Default, Clone, Debug)]
pub(super) struct OpenCell {
    pub(super) spans: Vec<OpenSpan>,
}

// Like a HeightSpan but representing open walkable areas (empty space with floor & height >= walkable_height
#[derive(Default, Clone, Copy, Debug)]
pub(super) struct OpenSpan {
    pub(super) min: u16,
    pub(super) max: Option<u16>,
    pub(super) neighbours: [Option<u16>; 4],
    pub(super) tile_index: usize, // The index of this span in the whole tile.
    pub(super) region: u16, // Region if non-zero. We could use option for this if we had some optimization for size.
}

#[derive(Default, Debug)]
pub struct OpenTile {
    pub(super) cells: Vec<OpenCell>, // len = tiles_along_width^2. Laid out X to Y
    pub(super) distances: Vec<u16>, // Distances used in watershed. One per span. Use tile_index to go from span to distance.
    pub(super) max_distance: u16,
    pub(super) span_count: usize, // Total spans in all cells.
    pub(super) max_regions: u16,
}

pub fn build_heightfield_tile(
    tile_coord: UVec2,
    triangle_collections: Vec<(GlobalTransform, Vec<Vec3>, Vec<[u32; 3]>)>,
    nav_mesh_settings: &NavMeshSettings,
) -> VoxelizedTile {
    let mut voxel_tile = VoxelizedTile {
        cells: vec![VoxelCell::default(); nav_mesh_settings.tile_width as usize * nav_mesh_settings.tile_width as usize],
    };

    let tile_min = nav_mesh_settings.get_tile_min_bound(tile_coord);

    let tile_max_bound = IVec3::new(
        (nav_mesh_settings.tile_width - 1).into(),
        0,
        (nav_mesh_settings.tile_width - 1).into(),
    );

    let max_vertices = triangle_collections.iter().fold(0, |acc, val| acc.max(val.1.len()));
    let mut translated_vertices = Vec::with_capacity(max_vertices);

    for (transform, vertices, triangles) in triangle_collections.iter() {
        let transform = transform.compute_transform().with_scale(Vec3::ONE); // The collider returned from rapier already has scale applied to it, so we reset it here.
        
        translated_vertices.clear();
        translated_vertices.extend(vertices.iter().map(|vertex| {
            transform.transform_point(*vertex) - Vec3::new(tile_min.x, nav_mesh_settings.world_bottom_bound, tile_min.y)
        })); // Transform vertices.

        for triangle in triangles.iter() {
            let a = translated_vertices[triangle[0] as usize];
            let b = translated_vertices[triangle[1] as usize];
            let c = translated_vertices[triangle[2] as usize];

            let min_bound = a.min(b).min(c).div(nav_mesh_settings.cell_width).as_ivec3();
            let max_bound = a.max(b).max(c).div(nav_mesh_settings.cell_width).as_ivec3();

            // Check if triangle is completely outside the tile.
            if max_bound.x < 0
                || max_bound.z < 0
                || min_bound.x > nav_mesh_settings.tile_width.into()
                || min_bound.z > nav_mesh_settings.tile_width.into()
            {
                continue;
            }

            let clamped_bound_min = min_bound.max(IVec3::ZERO);
            let clamped_bound_max = max_bound.min(tile_max_bound);

            let ab = b - a;
            let ac = c - a;
            let normal = ab.cross(ac).normalize();
            let slope = normal.dot(Vec3::Y).acos();
            let traversable = slope < nav_mesh_settings.max_traversable_slope_radians;

            let vertices = [a, b, c, Vec3::ZERO, Vec3::ZERO, Vec3::ZERO, Vec3::ZERO];

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
                let (_, _, row_min_clip_vert_count, row_min_clip_verts) =
                    divide_polygon(&vertices, 3, row_clip_min, 2);
                let (row_vert_count, row_verts, _, _) = divide_polygon(
                    &row_min_clip_verts,
                    row_min_clip_vert_count,
                    row_clip_max,
                    2,
                );
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
                let column_min =
                    ((column_min_vert_x / nav_mesh_settings.cell_width) as i32).max(0);
                let column_max = ((column_max_vert_x / nav_mesh_settings.cell_width) as i32)
                    .min((nav_mesh_settings.tile_width - 1).into());

                for x in column_min..=column_max {
                    let column_clip_min = x as f32 * nav_mesh_settings.cell_width;
                    let column_clip_max = column_clip_min + nav_mesh_settings.cell_width;

                    // Clip polygon to column.
                    let (_, _, column_min_clip_vert_count, column_min_clip_verts) =
                        divide_polygon(&row_verts, row_vert_count, column_clip_min, 0);
                    let (column_vert_count, column_verts, _, _) = divide_polygon(
                        &column_min_clip_verts,
                        column_min_clip_vert_count,
                        column_clip_max,
                        0,
                    );
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

                    let index = (x + z * nav_mesh_settings.tile_width as i32) as usize;
                    let cell = &mut voxel_tile.cells[index];

                    let mut new_span = HeightSpan {
                        min: min_height,
                        max: max_height,
                        traversable,
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
                        }
                        // An overlap!
                        match existing_span.max.cmp(&new_span.max) {
                            Ordering::Greater => {
                                new_span.traversable = existing_span.traversable;
                                new_span.max = existing_span.max;
                            }
                            Ordering::Equal => {
                                new_span.traversable |= existing_span.traversable;
                            }
                            Ordering::Less => {}
                        }
                        cell.spans.remove(i);
                    }
                    cell.spans.insert(i, new_span);
                }
            }
        }
    }

    voxel_tile
}

/*
*   This function takes in a polygon (of max 7 vertices), an line on which to divide it, and an axis.
*   It then returns the left polygon's vertex count, the left polygon's vertices,
*   the right polygon's vertex count, and the right polygon's vertices.
*/
fn divide_polygon(
    vertices: &[Vec3; 7], // TODO: Is it even possible to have more than 4 vertices as a result of a single triangle?
    vertex_count_in: usize,
    clip_line: f32,
    axis: usize,
) -> (usize, [Vec3; 7], usize, [Vec3; 7]) { // TODO: We always use one of these options. Does it make sense to even return the other?
    let mut polygon_a = [Vec3::ZERO; 7];
    let mut polygon_b = [Vec3::ZERO; 7];

    let mut delta_from_line = [0.0; 12];
    // This loop determines which side of the line the vertex is on.
    for i in 0..vertex_count_in {
        delta_from_line[i] = clip_line - vertices[i][axis];
    }

    let mut verts_a = 0;
    let mut verts_b = 0;

    for i in 0..vertex_count_in {
        let j = (vertex_count_in - 1 + i) % vertex_count_in; // j is i-1 wrapped.

        let in_a = delta_from_line[j] >= 0.0;
        let in_b = delta_from_line[i] >= 0.0;

        // Check if both vertices are on the same side of the line.
        if in_a != in_b {
            // We slide the vertex along to the edge.
            let slide = delta_from_line[j] / (delta_from_line[j] - delta_from_line[i]);

            polygon_a[verts_a] = vertices[j] + (vertices[i] - vertices[j]) * slide;
            polygon_b[verts_b] = polygon_a[verts_a];
            verts_a += 1;
            verts_b += 1;

            if delta_from_line[i] > 0.0 {
                polygon_a[verts_a] = vertices[i];
                verts_a += 1;
            } else if delta_from_line[i] < 0.0 {
                polygon_b[verts_b] = vertices[i];
                verts_b += 1;
            }
        } else {
            if delta_from_line[i] >= 0.0 {
                polygon_a[verts_a] = vertices[i];
                verts_a += 1;

                if delta_from_line[i] != 0.0 {
                    continue;
                }
            }
            polygon_b[verts_b] = vertices[i];
            verts_b += 1;
        }
    }
    (verts_a, polygon_a, verts_b, polygon_b)
}

pub fn build_open_heightfield_tile(
    voxelized_tile: &VoxelizedTile,
    nav_mesh_settings: &NavMeshSettings,
) -> OpenTile {
    let mut open_tile = OpenTile {
        cells: vec![OpenCell::default(); nav_mesh_settings.tile_width as usize * nav_mesh_settings.tile_width as usize],
        distances: Vec::new(),
        max_distance: 0,
        span_count: 0,
        max_regions: 0,
    };

    // First we create open spaces.
    for (i, cell) in voxelized_tile
    .cells
    .iter()
    .enumerate()
    .filter(|(_, cell)| !cell.spans.is_empty())
    {
        let open_spans = &mut open_tile.cells[i].spans;
        open_spans.clear();

        let mut iter = cell.spans.iter().peekable();
        while let Some(span) = iter.next() {
            if !span.traversable { // Skip untraversable. Not filtered because we still need to peek at them so.
                continue;
            }

            if let Some(next_span) = iter.peek() {
                // Need to check if space is large enough.
                if next_span.min - span.max >= nav_mesh_settings.walkable_height {
                    open_spans.push(OpenSpan {
                        min: span.max,
                        max: Some(next_span.min),
                        ..Default::default()
                    });
                }
            } else {
                // None above. This is an unbounded open space.
                open_spans.push(OpenSpan {
                    min: span.max,
                    max: None,
                    ..Default::default()
                });
            }
        }
        open_tile.span_count += open_spans.len();
    }

    open_tile.distances.resize(open_tile.span_count, u16::MAX);

    open_tile
}

pub fn link_neighbours(
    open_tile: &mut OpenTile,
    nav_mesh_settings: &NavMeshSettings,
) {
    let cells_count = nav_mesh_settings.tile_width as usize * nav_mesh_settings.tile_width as usize;

    let mut x_positive = Vec::with_capacity(3);
    let mut x_negative = Vec::with_capacity(3);
    let mut z_positive = Vec::with_capacity(3);
    let mut z_negative = Vec::with_capacity(3);

    for i in 0..cells_count {
        if open_tile.cells[i].spans.is_empty() {
            continue;
        }

        let row = i / nav_mesh_settings.tile_width as usize;
        let column = i % nav_mesh_settings.tile_width as usize;

        let x_positive_contained = column < (nav_mesh_settings.tile_width - 1).into();
        let x_negative_contained = column > 0;
        let z_positive_contained = row < (nav_mesh_settings.tile_width - 1).into();
        let z_negative_contained = row > 0;

        x_positive.clear();
        x_negative.clear();
        z_positive.clear();
        z_negative.clear();

        if x_positive_contained {
            x_positive.extend(open_tile.cells[i + 1].spans.iter().map(|span| (span.min, span.max)));
        }
        if x_negative_contained {
            x_negative.extend(open_tile.cells[i - 1].spans.iter().map(|span| (span.min, span.max)));
        }
        if z_positive_contained {
            z_positive.extend(
                open_tile.cells[i + nav_mesh_settings.tile_width as usize]
                    .spans
                    .iter()
                    .map(|span| (span.min, span.max)),
            );
        }
        if z_negative_contained {
            z_negative.extend(
                open_tile.cells[i - nav_mesh_settings.tile_width as usize]
                    .spans
                    .iter()
                    .map(|span| (span.min, span.max)),
            );
        }

        for span in open_tile.cells[i].spans.iter_mut() {
            for (i, (min, max)) in x_negative.iter().enumerate() {
                if max.is_some() && span.max.is_some() {
                    let max = max.unwrap();
                    let span_max = span.max.unwrap();

                    let gap = span_max.min(max).abs_diff(span.min.max(*min));
                    if gap < nav_mesh_settings.walkable_height {
                        continue;
                    }
                }

                if min.abs_diff(span.min) < nav_mesh_settings.step_height {
                    span.neighbours[0] = Some(i as u16);
                    break;
                }
            }

            for (i, (min, max)) in z_positive.iter().enumerate() {
                if max.is_some() && span.max.is_some() {
                    let max = max.unwrap();
                    let span_max = span.max.unwrap();

                    let gap = span_max.min(max).abs_diff(span.min.max(*min));
                    if gap < nav_mesh_settings.walkable_height {
                        continue;
                    }
                }

                if min.abs_diff(span.min) < nav_mesh_settings.step_height {
                    span.neighbours[1] = Some(i as u16);
                    break;
                }
            }

            for (i, (min, max)) in x_positive.iter().enumerate() {
                if max.is_some() && span.max.is_some() {
                    let max = max.unwrap();
                    let span_max = span.max.unwrap();

                    let gap = span_max.min(max).abs_diff(span.min.max(*min));
                    if gap < nav_mesh_settings.walkable_height {
                        continue;
                    }
                }

                if min.abs_diff(span.min) < nav_mesh_settings.step_height {
                    span.neighbours[2] = Some(i as u16);
                    break;
                }
            }

            for (i, (min, max)) in z_negative.iter().enumerate() {
                if max.is_some() && span.max.is_some() {
                    let max = max.unwrap();
                    let span_max = span.max.unwrap();

                    let gap = span_max.min(max).abs_diff(span.min.max(*min));
                    if gap < nav_mesh_settings.walkable_height {
                        continue;
                    }
                }
                
                if min.abs_diff(span.min) < nav_mesh_settings.step_height {
                    span.neighbours[3] = Some(i as u16);
                    break;
                }
            }
        }
    }
}

pub fn calculate_distance_field(
    open_tile: &mut OpenTile,
    nav_mesh_settings: &NavMeshSettings,
) {
    // Assign tile_index.
    let mut tile_index = 0;
    for cell in open_tile.cells.iter_mut() {
        for span in cell.spans.iter_mut() {
            span.tile_index = tile_index;
            tile_index += 1;
        }
    }

    // Mark boundary cells.
    for cell in open_tile.cells.iter() {
        for span in cell.spans.iter() {
            let neighbours = span.neighbours.iter().filter(|neighbour| neighbour.is_some()).count();

            if neighbours != 4 {
                open_tile.distances[span.tile_index] = 0;
            }
        }
    }

    // Pass 1.
    for (i, cell) in open_tile.cells.iter().enumerate() {
        for span in cell.spans.iter() {
            let mut distance = open_tile.distances[span.tile_index];

            if let Some(span_index) = span.neighbours[0] {
                // (-1, 0)
                let other_cell_index = i - 1;
                let other_span = &open_tile.cells[other_cell_index].spans[span_index as usize];
                
                let other_distance = open_tile.distances[other_span.tile_index];
                if (other_distance + 2) < distance {
                    distance = other_distance + 2;
                }

                // (-1, -1)
                if let Some(span_index) = other_span.neighbours[3] {
                    let other_cell_index = other_cell_index - nav_mesh_settings.tile_width as usize;
                    let other_span = &open_tile.cells[other_cell_index].spans[span_index as usize];

                    let other_distance = open_tile.distances[other_span.tile_index];
                    if (other_distance + 3) < distance {
                        distance = other_distance + 3;
                    }
                }
            }

            if let Some(span_index) = span.neighbours[3] {
                // (0, -1)
                let other_cell_index = i - nav_mesh_settings.tile_width as usize;
                let other_span = &open_tile.cells[other_cell_index].spans[span_index as usize];

                let other_distance = open_tile.distances[other_span.tile_index];
                if (other_distance + 2) < distance {
                    distance = other_distance + 2;
                }

                // (1, -1)
                if let Some(span_index) = other_span.neighbours[2] {
                    let other_cell_index = other_cell_index + 1;
                    let other_span = &open_tile.cells[other_cell_index].spans[span_index as usize];

                    let other_distance = open_tile.distances[other_span.tile_index];
                    if (other_distance + 3) < distance {
                        distance = other_distance + 3;
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

                let other_distance = open_tile.distances[other_span.tile_index];
                if (other_distance + 2) < distance {
                    distance = other_distance + 2;
                }

                // (1, 1)
                if let Some(span_index) =
                    other_span.neighbours[1]
                {
                    let other_cell_index = other_cell_index + nav_mesh_settings.tile_width as usize;
                    let other_span = &open_tile.cells[other_cell_index].spans[span_index as usize];

                    let other_distance = open_tile.distances[other_span.tile_index];
                    if (other_distance + 3) < distance {
                        distance = other_distance + 3;
                    }
                }
            }

            if let Some(span_index) = span.neighbours[1] {
                // (0, 1)
                let other_cell_index = i + nav_mesh_settings.tile_width as usize;
                let other_span = &open_tile.cells[other_cell_index].spans[span_index as usize];

                let other_distance = open_tile.distances[other_span.tile_index];
                if (other_distance + 2) < distance {
                    distance = other_distance + 2;
                }

                // (-1, 1)
                if let Some(span_index) =
                    other_span.neighbours[0]
                {
                    let other_cell_index = other_cell_index - 1;
                    let other_span = &open_tile.cells[other_cell_index].spans[span_index as usize];

                    let other_distance = open_tile.distances[other_span.tile_index];
                    if (other_distance + 3) < distance {
                        distance = other_distance + 3;
                    }
                }
            }

            // Apply distance change.
            open_tile.distances[span.tile_index] = distance;
        }
    }
    
    open_tile.max_distance = *open_tile.distances.iter().max().unwrap_or(&0);

    // Box blur. If you're reading this, why?
    let threshold = 2;

    let mut blurred = vec![0; open_tile.distances.len()];

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

                let other_cell_index = get_neighbour_index(nav_mesh_settings, i, dir);
                let other_span =
                    &open_tile.cells[other_cell_index].spans[index as usize];

                d += open_tile.distances[other_span.tile_index];

                let next_dir = (dir + 1) & 0x3;
                let Some(index) = other_span.neighbours[next_dir] else {
                    d += distance;
                    continue;
                };

                let other_cell_index = get_neighbour_index(nav_mesh_settings, other_cell_index, next_dir);

                let other_span =
                    &open_tile.cells[other_cell_index].spans[index as usize];

                d += open_tile.distances[other_span.tile_index];
            }

            // Apply distance change.
            blurred[span.tile_index] = (d + 5) / 9;
        }
    }

    open_tile.distances = blurred;
    // End Box Blur
}
