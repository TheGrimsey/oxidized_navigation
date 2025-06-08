use std::cmp::Ordering;

use bevy::{
    prelude::{IVec2, UVec2, UVec4},
};

use crate::{
    get_neighbour_index,
    heightfields::{OpenSpan, OpenTile},
    Area,
};

use super::math::{in_cone, intersect};
use super::{NavMeshSettings, FLAG_BORDER_VERTEX, MASK_CONTOUR_REGION};

#[derive(Default, Clone, Debug)]
pub struct Contour {
    pub vertices: Vec<UVec4>,
    pub region: u16,
    /// Unlike [OpenSpan] this can't be ``None`` as ``None`` spans are ignored when generating contours.  
    pub area: Area,
}

#[derive(Default)]
pub struct ContourSet {
    pub contours: Vec<Contour>,
}

#[derive(Default, Clone)]
struct ContourHole {
    contour: Contour,
    min_x: u32,
    min_z: u32,
    left_most_vertex: u32,
}

#[derive(Default, Clone)]
struct ContourRegion {
    outline: Option<Contour>,
    holes: Vec<ContourHole>,
}

pub fn build_contours(open_tile: &OpenTile, nav_mesh_settings: &NavMeshSettings) -> ContourSet {
    let max_contours = open_tile.max_regions.max(8);
    let mut contour_set = ContourSet {
        contours: Vec::with_capacity(max_contours.into()),
    };
    let tile_side = nav_mesh_settings.get_tile_side_with_border();

    // Mark boundaries.
    let mut boundry_flags = vec![0u8; open_tile.span_count];
    for (cell_index, cell) in open_tile.cells.iter().enumerate() {
        for span in cell.spans.iter() {
            let mut res = 0;

            for dir in 0..4 {
                let mut other_region = 0;
                if let Some(span_index) = span.neighbours[dir] {
                    let other_span = &open_tile.cells
                        [get_neighbour_index(tile_side, cell_index, dir)]
                    .spans[span_index as usize];
                    other_region = other_span.region;
                }

                if span.region == other_region {
                    res |= 1 << dir;
                }
            }

            boundry_flags[span.tile_index] = res ^ 0b1111; // Flip so we mark unconnected sides.
        }
    }

    let mut vertices = Vec::with_capacity(256);
    let mut simplified_vertices = Vec::with_capacity(64);

    for (cell_index, cell) in open_tile.cells.iter().enumerate() {
        for (span_index, span) in cell.spans.iter().enumerate() {
            if boundry_flags[span.tile_index] == 0 || boundry_flags[span.tile_index] == 0b1111 {
                boundry_flags[span.tile_index] = 0;
                continue;
            }
            if span.region == 0 {
                continue;
            }
            let Some(area) = open_tile.areas[span.tile_index] else {
                continue;
            };

            vertices.clear();
            simplified_vertices.clear();

            // Walk contour
            walk_contour(
                cell_index,
                span_index,
                open_tile,
                nav_mesh_settings,
                &mut boundry_flags,
                &mut vertices,
            );

            // Simplify contour
            simplify_contour(
                &vertices,
                &mut simplified_vertices,
                nav_mesh_settings.max_contour_simplification_error,
                nav_mesh_settings.max_edge_length,
            );

            // Remove degenerate segments.
            remove_degenerate_segments(&mut simplified_vertices);

            if simplified_vertices.len() >= 3 {
                let new_contour = Contour {
                    vertices: simplified_vertices.clone(),
                    region: span.region,
                    area,
                };

                contour_set.contours.push(new_contour);
            }
        }
    }

    // handle holes.
    if !contour_set.contours.is_empty() {
        #[derive(Clone, Copy)]
        enum Winding {
            Outline,
            Hole,
        }

        let mut winding = vec![Winding::Hole; contour_set.contours.len()];
        let mut num_holes = 0;
        for (i, contour) in contour_set.contours.iter().enumerate() {
            let contour_winding = calc_area_of_polygon_2d(&contour.vertices);

            if contour_winding < 0 {
                num_holes += 1;
                winding[i] = Winding::Hole;
            } else {
                winding[i] = Winding::Outline;
            }
        }

        if num_holes > 0 {
            let num_regions = open_tile.max_regions + 1;
            let mut regions = vec![ContourRegion::default(); num_regions.into()];

            for (contour, winding) in contour_set.contours.iter().zip(winding) {
                match winding {
                    Winding::Outline => {
                        regions[contour.region as usize].outline = Some(contour.clone());
                    }
                    Winding::Hole => {
                        regions[contour.region as usize].holes.push(ContourHole {
                            contour: contour.clone(),
                            min_x: contour.vertices[0].x,
                            min_z: contour.vertices[0].z,
                            left_most_vertex: 0,
                        });
                    }
                }
            }

            for region in regions
                .iter_mut()
                .filter(|region| !region.holes.is_empty() && region.outline.is_some())
            {
                merge_region_holes(region);
            }
        }
    }

    contour_set
}

#[derive(Default, Clone, Copy)]
struct PotentialDiagonal {
    vertex: u32,
    distance: u32,
}

fn merge_region_holes(region: &mut ContourRegion) {
    // Find left-most vertex
    for hole in region.holes.iter_mut() {
        for (i, vertex) in hole.contour.vertices.iter().enumerate() {
            if vertex.x < hole.min_x || (vertex.x == hole.min_x && vertex.z < hole.min_z) {
                hole.min_x = vertex.x;
                hole.min_z = vertex.z;
                hole.left_most_vertex = i as u32;
            }
        }
    }

    region.holes.sort_by(|a, b| match a.min_x.cmp(&b.min_x) {
        Ordering::Less => Ordering::Less,
        Ordering::Equal => a.min_z.cmp(&b.min_z),
        Ordering::Greater => Ordering::Greater,
    });

    let max_vertices = region
        .outline
        .as_ref()
        .map_or(0, |outline| outline.vertices.len())
        + region
            .holes
            .iter()
            .fold(0, |value, hole| value + hole.contour.vertices.len());

    let mut diagonals = Vec::with_capacity(max_vertices);

    let outline = region.outline.as_mut().unwrap();

    for (hole_i, hole) in region.holes.iter().enumerate() {
        let mut index = None;
        let mut best_vertex = hole.left_most_vertex;

        for _ in 0..hole.contour.vertices.len() {
            // Find potential diagonals.
            //
            diagonals.clear();
            let corner_vertex = hole.contour.vertices[best_vertex as usize];
            for i in 0..outline.vertices.len() {
                if in_cone(i, &outline.vertices, corner_vertex) {
                    let delta_x = outline.vertices[i].x.abs_diff(corner_vertex.x);
                    let delta_z = outline.vertices[i].z.abs_diff(corner_vertex.z);
                    let distance = delta_x * delta_x + delta_z * delta_z;
                    diagonals.push(PotentialDiagonal {
                        vertex: i as u32,
                        distance,
                    });
                }
            }

            diagonals.sort_by(|a, b| a.distance.cmp(&b.distance));

            // Find non-intersecting diagonals.
            index = None;

            for potential_diagonal in diagonals.iter() {
                let vertex = outline.vertices[potential_diagonal.vertex as usize];
                let mut intersects = intersect_segment_contour(
                    vertex,
                    corner_vertex,
                    potential_diagonal.vertex as usize,
                    &outline.vertices,
                );

                for other_hole in region.holes.iter().skip(hole_i) {
                    intersects |= intersect_segment_contour_no_vertex(
                        vertex,
                        corner_vertex,
                        &other_hole.contour.vertices,
                    );

                    if intersects {
                        break;
                    }
                }

                if !intersects {
                    index = Some(potential_diagonal.vertex);
                    break;
                }
            }

            if index.is_some() {
                break;
            }

            best_vertex = (best_vertex + 1) % hole.contour.vertices.len() as u32;
        }

        let Some(index) = index else {
            continue;
        };

        merge_contours(outline, &hole.contour, index as usize, best_vertex as usize);
    }
}

fn merge_contours(
    target_contour: &mut Contour,
    source_contour: &Contour,
    index_a: usize,
    index_b: usize,
) {
    let mut vertices =
        Vec::with_capacity(target_contour.vertices.len() + source_contour.vertices.len());

    for i in 0..target_contour.vertices.len() {
        let vertex = &target_contour.vertices[(index_a + i) % target_contour.vertices.len()];
        vertices.push(*vertex);
    }

    for i in 0..source_contour.vertices.len() {
        let vertex = &source_contour.vertices[(index_b + i) % source_contour.vertices.len()];
        vertices.push(*vertex);
    }

    target_contour.vertices = vertices;
}

fn calc_area_of_polygon_2d(vertices: &[UVec4]) -> i32 {
    let mut area = 0;
    for i in 0..vertices.len() {
        let previous = vertices[i].as_ivec4();
        let next = vertices[(i + 1) % vertices.len()].as_ivec4();

        area += next.x * previous.z - previous.x * next.z;
    }

    (area + 1) / 2
}

fn intersect_segment_contour(
    point: UVec4,
    corner: UVec4,
    diagonal_vertex: usize,
    outline_vertices: &[UVec4],
) -> bool {
    for i in 0..outline_vertices.len() {
        let next = (i + 1) % outline_vertices.len();

        if i == diagonal_vertex || next == diagonal_vertex {
            continue;
        }

        let point_i = outline_vertices[i];
        let point_next = outline_vertices[next];

        if (point.x == point_i.x && point.z == point_i.z)
            || (point_next.x == point_i.x && point_next.z == point_i.z)
            || (point_next.x == point.x && point_next.z == point.z)
        {
            continue;
        }

        if intersect(
            point.as_ivec4(),
            corner.as_ivec4(),
            point_i.as_ivec4(),
            point_next.as_ivec4(),
        ) {
            return true;
        }
    }

    false
}

fn intersect_segment_contour_no_vertex(
    point: UVec4,
    corner: UVec4,
    outline_vertices: &[UVec4],
) -> bool {
    for i in 0..outline_vertices.len() {
        let next = (i + 1) % outline_vertices.len();

        let point_i = outline_vertices[i];
        let point_next = outline_vertices[next];

        if (point.x == point_i.x && point.z == point_i.z)
            || (point_next.x == point_i.x && point_next.z == point_i.z)
            || (point_next.x == point.x && point_next.z == point.z)
        {
            continue;
        }

        if intersect(
            point.as_ivec4(),
            corner.as_ivec4(),
            point_i.as_ivec4(),
            point_next.as_ivec4(),
        ) {
            return true;
        }
    }

    false
}

fn walk_contour(
    mut cell_index: usize,
    mut span_index: usize,
    tile: &OpenTile,
    nav_mesh_settings: &NavMeshSettings,
    boundry_flags: &mut [u8],
    contour: &mut Vec<u32>,
) {
    let mut dir = 0u8;
    while (boundry_flags[tile.cells[cell_index].spans[span_index].tile_index] & (1 << dir)) == 0 {
        dir += 1;
    }
    let start_direction = dir;
    let start_cell = cell_index;
    let start_span = span_index;
    let tile_side = nav_mesh_settings.get_tile_side_with_border();

    loop {
        let row = cell_index / nav_mesh_settings.get_tile_side_with_border();
        let column = cell_index % nav_mesh_settings.get_tile_side_with_border();

        let span = &tile.cells[cell_index].spans[span_index];
        if boundry_flags[span.tile_index] & (1 << dir) > 0 {
            // Check if this direction is unconnected.
            let height = get_corner_height(cell_index, span, tile, nav_mesh_settings, dir);

            let mut bordering_region = 0u32;
            if let Some(span_index) = span.neighbours[dir as usize] {
                let other_span = &tile.cells
                    [get_neighbour_index(tile_side, cell_index, dir.into())]
                .spans[span_index as usize];
                bordering_region = other_span.region.into();
            }

            let px = match dir {
                1 => column + 1,
                2 => column + 1,
                _ => column,
            } as u32;
            let py = height as u32;
            let pz = match dir {
                0 => row + 1,
                1 => row + 1,
                _ => row,
            } as u32;
            contour.extend_from_slice(&[px, py, pz, bordering_region]);

            boundry_flags[span.tile_index] &= !(1 << dir);
            dir = (dir + 1) & 0x3; // Rotate clock-wise.
        } else {
            // Direction is connected.
            if let Some(index) = span.neighbours[dir as usize] {
                span_index = index.into();
            } else {
                panic!("Incorrectly flagged boundry! This should not happen.");
            }

            cell_index = get_neighbour_index(tile_side, cell_index, dir.into());
            dir = (dir + 3) & 0x3; // Rotate COUNTER clock-wise.
        }

        if start_cell == cell_index && start_span == span_index && start_direction == dir {
            break;
        }
    }
}

fn get_corner_height(
    cell_index: usize,
    span: &OpenSpan,
    tile: &OpenTile,
    nav_mesh_settings: &NavMeshSettings,
    dir: u8,
) -> u16 {
    let tile_side = nav_mesh_settings.get_tile_side_with_border();
    let next_dir = (dir + 1) & 0x3;
    let mut regions = [0; 4];

    let mut height = span.min;

    regions[0] = span.region;

    if let Some(span_index) = span.neighbours[dir as usize] {
        let other_cell_index = get_neighbour_index(tile_side, cell_index, dir.into());
        let other_span = &tile.cells[other_cell_index].spans[span_index as usize];

        height = height.max(other_span.min);
        regions[1] = other_span.region;

        if let Some(span_index) = other_span.neighbours[next_dir as usize] {
            let other_cell_index =
                get_neighbour_index(tile_side, other_cell_index, next_dir.into());
            let other_span = &tile.cells[other_cell_index].spans[span_index as usize];

            height = height.max(other_span.min);
            regions[2] = other_span.region;
        }
    }

    if let Some(span_index) = span.neighbours[next_dir as usize] {
        let other_cell_index = get_neighbour_index(tile_side, cell_index, next_dir.into());
        let other_span = &tile.cells[other_cell_index].spans[span_index as usize];

        height = height.max(other_span.min);
        regions[3] = other_span.region;

        if let Some(span_index) = other_span.neighbours[dir as usize] {
            let other_cell_index = get_neighbour_index(tile_side, other_cell_index, dir.into());
            let other_span = &tile.cells[other_cell_index].spans[span_index as usize];

            height = height.max(other_span.min);
            regions[2] = other_span.region;
        }
    }

    height
}

fn simplify_contour(
    points: &[u32],
    simplified: &mut Vec<UVec4>,
    max_error: f32,
    max_edge_len: u16,
) {
    let has_connections = {
        let mut has_connections = false;

        let mut i = 0;
        while i < points.len() {
            if (points[i + 3] & MASK_CONTOUR_REGION) != 0 {
                has_connections = true;
                break;
            }
            i += 4;
        }

        has_connections
    };

    if has_connections {
        let length = points.len() / 4;
        for i in 0..length {
            let next = ((i + 1) % length) * 4;
            let i_pre_mul = i * 4;

            let regions_differ = (points[i_pre_mul + 3] & MASK_CONTOUR_REGION)
                != (points[next + 3] & MASK_CONTOUR_REGION);
            if regions_differ {
                simplified.push(UVec4 {
                    x: points[i_pre_mul],
                    y: points[i_pre_mul + 1],
                    z: points[i_pre_mul + 2],
                    w: i as u32,
                });
            }
        }
    } else {
        let mut lower_left_x = points[0];
        let mut lower_left_y = points[1];
        let mut lower_left_z = points[2];
        let mut lower_left_i = 0;

        let mut upper_right_x = points[0];
        let mut upper_right_y = points[1];
        let mut upper_right_z = points[2];
        let mut upper_right_i = 0;

        let length = points.len() / 4;
        for i in 0..length {
            let i_pre_mul = i * 4;
            let x = points[i_pre_mul];
            let y = points[i_pre_mul + 1];
            let z = points[i_pre_mul + 2];
            if x < lower_left_x || (x == lower_left_x && z < lower_left_z) {
                lower_left_x = x;
                lower_left_y = y;
                lower_left_z = z;
                lower_left_i = i as u32;
            }
            if x > upper_right_x || (x == upper_right_x && z > upper_right_z) {
                upper_right_x = x;
                upper_right_y = y;
                upper_right_z = z;
                upper_right_i = i as u32;
            }
        }

        simplified.push(UVec4 {
            x: lower_left_x,
            y: lower_left_y,
            z: lower_left_z,
            w: lower_left_i,
        });

        simplified.push(UVec4 {
            x: upper_right_x,
            y: upper_right_y,
            z: upper_right_z,
            w: upper_right_i,
        });
    }

    let point_count = points.len() / 4;
    let mut i = 0;
    while i < simplified.len() {
        let next = (i + 1) % simplified.len();

        let mut a = simplified[i];
        let mut b = simplified[next];

        let (mut c_i, c_increments, end_i) = if b.x > a.x || (b.x == a.x && b.z > a.z) {
            let c_increments = 1;
            let c_i = (a.w + c_increments) % point_count as u32;
            let end_i = b.w;
            (c_i, c_increments, end_i)
        } else {
            let c_increments = (point_count - 1) as u32;
            let c_i = (b.w + c_increments) % point_count as u32;
            let end_i = a.w;

            std::mem::swap(&mut a.x, &mut b.x);
            std::mem::swap(&mut a.z, &mut b.z);
            (c_i, c_increments, end_i)
        };

        let mut max_deviation = 0.0;
        let mut max_i = None;

        if (points[(c_i * 4 + 3) as usize] & MASK_CONTOUR_REGION) == 0 {
            // Checking if region is 0. We only tesellate unconnected edges.
            while c_i != end_i {
                let deviation = point_distance_from_segment(
                    UVec2::new(points[(c_i * 4) as usize], points[(c_i * 4 + 2) as usize])
                        .as_ivec2(),
                    UVec2::new(a.x, a.z).as_ivec2(),
                    UVec2::new(b.x, b.z).as_ivec2(),
                );
                if deviation > max_deviation {
                    max_deviation = deviation;
                    max_i = Some(c_i);
                }
                c_i = (c_i + c_increments) % point_count as u32;
            }
        }

        match (max_i, max_deviation > (max_error * max_error)) {
            (Some(max_i), true) => {
                simplified.insert(
                    i + 1,
                    UVec4 {
                        x: points[(max_i * 4) as usize],
                        y: points[(max_i * 4 + 1) as usize],
                        z: points[(max_i * 4 + 2) as usize],
                        w: max_i,
                    },
                );
            }
            _ => {
                i += 1;
            }
        }
    }

    // SPLIT LONG EDGES.
    {
        let mut i = 0;
        while i < simplified.len() {
            let a = simplified[i];
            let b = simplified[(i + 1) % simplified.len()];

            let next_original_point_index = (a.w + 1) as usize % point_count;
            let should_tesselate =
                points[next_original_point_index * 4 + 3] & MASK_CONTOUR_REGION == 0;

            let mut max_i = None;
            if should_tesselate {
                let delta_x = b.x.abs_diff(a.x);
                let delta_z = b.z.abs_diff(a.z);

                if delta_x * delta_x + delta_z * delta_z > max_edge_len as u32 * max_edge_len as u32
                {
                    let n = if b.w < a.w {
                        b.w as isize + point_count as isize - a.w as isize
                    } else {
                        b.w as isize - a.w as isize
                    };

                    if n > 1 {
                        if b.x > a.x || (b.x == a.x && b.z > a.z) {
                            max_i = Some((a.w as usize + (n / 2) as usize) % point_count);
                        } else {
                            max_i = Some((a.w as usize + ((n + 1) / 2) as usize) % point_count)
                        }
                    }
                }
            }

            if let Some(max_i) = max_i {
                simplified.insert(
                    i + 1,
                    UVec4::new(
                        points[max_i * 4],
                        points[max_i * 4 + 1],
                        points[max_i * 4 + 2],
                        max_i as u32,
                    ),
                );
            } else {
                i += 1;
            }
        }
    }

    for point in simplified.iter_mut() {
        let next = (point.w + 1) % point_count as u32;
        let current = point.w;
        point.w = (points[(next * 4 + 3) as usize] & MASK_CONTOUR_REGION)
            | (points[(current * 4 + 3) as usize] & FLAG_BORDER_VERTEX);
    }
}

pub fn point_distance_from_segment(point: IVec2, seg_a: IVec2, seg_b: IVec2) -> f32 {
    let segment_delta = (seg_b - seg_a).as_vec2();
    let point_delta = (point - seg_a).as_vec2();

    let d = segment_delta.x * segment_delta.x + segment_delta.y * segment_delta.y;
    let mut t = segment_delta.x * point_delta.x + segment_delta.y * point_delta.y;
    if d > 0.0 {
        t /= d;
    }
    t = t.clamp(0.0, 1.0);

    let delta_x = seg_a.x as f32 + t * segment_delta.x - point.x as f32;
    let delta_y = seg_a.y as f32 + t * segment_delta.y - point.y as f32;

    delta_x * delta_x + delta_y * delta_y
}

fn remove_degenerate_segments(simplified: &mut Vec<UVec4>) {
    // Remove adjacent vertices which are equal on xz-plane,
    let mut i = 0;
    while i < simplified.len() {
        let next = (i + 1) % simplified.len();

        let a = simplified[i];
        let b = simplified[next];

        if a.x == b.x && a.z == b.z {
            simplified.remove(i);
        }
        i += 1;
    }
}
