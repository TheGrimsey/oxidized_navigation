use std::{cmp::Ordering, ops::Div};

use bevy::{
    prelude::{info, GlobalTransform, IVec3, Query, Res, ResMut, Resource, UVec2, Vec3, With},
    utils::HashMap,
};
use bevy_rapier3d::prelude::Collider;

use super::{
    cell_move_back_column, cell_move_back_row, cell_move_forward_column, cell_move_forward_row,
    get_cell_offset, DirtyTiles, NavMeshAffector, NavMeshSettings, NeighbourConnection, OpenCell,
    OpenSpan, OpenTile, TileAffectors, TilesOpen,
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
struct VoxelizedTile {
    cells: Vec<VoxelCell>, // len = tiles_along_width^2. Laid out X to Y
}

#[derive(Default, Resource)]
pub(super) struct TilesVoxelized {
    map: HashMap<UVec2, VoxelizedTile>,
}

pub(super) fn rebuild_heightfields_system(
    nav_mesh_settings: Res<NavMeshSettings>,
    tile_affectors: Res<TileAffectors>,
    mut tiles: ResMut<TilesVoxelized>,
    dirty_tiles: Res<DirtyTiles>,
    collider_query: Query<(&Collider, &GlobalTransform), With<NavMeshAffector>>,
) {
    if dirty_tiles.0.is_empty() {
        return;
    }

    for tile_coord in dirty_tiles.0.iter() {
        let Some(affectors) = tile_affectors.map.get(tile_coord) else {
            tiles.map.remove(tile_coord);
            continue;
        };
        if affectors.is_empty() {
            tiles.map.remove(tile_coord);
            continue;
        }

        if !tiles.map.contains_key(tile_coord) {
            tiles.map.insert(*tile_coord, VoxelizedTile::default());
        }

        let voxel_tile = tiles.map.get_mut(tile_coord).unwrap();

        voxel_tile.cells.clear();
        voxel_tile.cells.resize(
            (nav_mesh_settings.tile_width * nav_mesh_settings.tile_width) as usize,
            VoxelCell::default(),
        );

        let tile_min = nav_mesh_settings.get_tile_min_bound(*tile_coord);

        let mut iter = collider_query.iter_many(affectors.iter());
        while let Some((collider, transform)) = iter.fetch_next() {
            let mesh = match collider.as_typed_shape() {
                bevy_rapier3d::prelude::ColliderView::Ball(ball) => ball.raw.to_trimesh(5, 5),
                bevy_rapier3d::prelude::ColliderView::Cuboid(cuboid) => cuboid.raw.to_trimesh(),
                bevy_rapier3d::prelude::ColliderView::Capsule(capsule) => {
                    capsule.raw.to_trimesh(5, 5)
                }
                bevy_rapier3d::prelude::ColliderView::TriMesh(trimesh) => {
                    (trimesh.raw.vertices().to_vec(), trimesh.indices().to_vec())
                }
                bevy_rapier3d::prelude::ColliderView::HeightField(heightfield) => {
                    heightfield.raw.to_trimesh()
                }
                bevy_rapier3d::prelude::ColliderView::ConvexPolyhedron(polyhedron) => {
                    polyhedron.raw.to_trimesh()
                }
                bevy_rapier3d::prelude::ColliderView::Cylinder(cylinder) => {
                    cylinder.raw.to_trimesh(5)
                }
                bevy_rapier3d::prelude::ColliderView::Cone(cone) => cone.raw.to_trimesh(5),
                bevy_rapier3d::prelude::ColliderView::RoundCuboid(round_cuboid) => {
                    round_cuboid.raw.inner_shape.to_trimesh()
                }
                bevy_rapier3d::prelude::ColliderView::RoundCylinder(round_cylinder) => {
                    round_cylinder.raw.inner_shape.to_trimesh(5)
                }
                bevy_rapier3d::prelude::ColliderView::RoundCone(round_cone) => {
                    round_cone.raw.inner_shape.to_trimesh(5)
                }
                bevy_rapier3d::prelude::ColliderView::RoundConvexPolyhedron(round_polyhedron) => {
                    round_polyhedron.raw.inner_shape.to_trimesh()
                }
                // TODO: All the following ones are more complicated :)
                bevy_rapier3d::prelude::ColliderView::Segment(_) => todo!(), /* ??? */
                bevy_rapier3d::prelude::ColliderView::Triangle(_) => todo!(), /* ??? */
                bevy_rapier3d::prelude::ColliderView::RoundTriangle(_) => todo!(), /* ??? */
                bevy_rapier3d::prelude::ColliderView::Compound(_) => todo!(), /* ??? */
                // These ones do not make sense in this.
                bevy_rapier3d::prelude::ColliderView::HalfSpace(_) => continue, /* This is like an infinite plane? We don't care. */
                bevy_rapier3d::prelude::ColliderView::Polyline(_) => continue, /* This is a line. */
            };
            // Transform points into world space and then relative to tile. This is gonna be slowwww with more vertices. We could change this to be Vec3A.
            let vertices = mesh
                .0
                .iter()
                .map(|point| {
                    transform.transform_point(Vec3::new(point.x, point.y, point.z))
                        - Vec3::new(tile_min.x, 0.0, tile_min.y)
                })
                .collect::<Vec<Vec3>>();

            for triangle in mesh.1 {
                let a = vertices[triangle[0] as usize];
                let b = vertices[triangle[1] as usize];
                let c = vertices[triangle[2] as usize];

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
                let clamped_bound_max = max_bound.min(IVec3::new(
                    (nav_mesh_settings.tile_width - 1).into(),
                    0,
                    (nav_mesh_settings.tile_width - 1).into(),
                ));

                let ab = b - a;
                let ac = c - a;
                let normal = ab.cross(ac).normalize();
                let slope = normal.dot(Vec3::Y).acos();
                let traversable = slope < nav_mesh_settings.max_traversable_slope;

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
                        square_min_height -= nav_mesh_settings.world_bottom_bound;
                        square_max_height -= nav_mesh_settings.world_bottom_bound;

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
    }
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
) -> (usize, [Vec3; 7], usize, [Vec3; 7]) {
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

/*#[test]
fn test_clip_polygon() {
    let mut vertices = [Vec3::ZERO; 7];
    vertices[0] = Vec3::new(0.0, 0.0, 0.0);
    vertices[1] = Vec3::new(7.5, 0.0, 0.0);
    vertices[2] = Vec3::new(7.5, 0.0, 7.5);

    let (_, _, verts_b, polygon_b) = divide_polygon(&vertices, 3, 2.5, 0);
    println!("B: {}, Verts: {:?}", verts_b, polygon_b);

    let (verts_c, polygon_c, _, _) = divide_polygon(&polygon_b, verts_b, 5.0, 0);
    println!("C: {}, Verts: {:?}", verts_c, polygon_c);
}*/

pub(super) fn construct_open_heightfields_system(
    nav_mesh_settings: Res<NavMeshSettings>,
    voxel_tiles: Res<TilesVoxelized>,
    mut open_tiles: ResMut<TilesOpen>,
    dirty_tiles: Res<DirtyTiles>,
) {
    if dirty_tiles.0.is_empty() {
        return;
    }

    for tile_coord in dirty_tiles.0.iter() {
        let Some(voxel_tile) = voxel_tiles.map.get(tile_coord) else {
            open_tiles.map.remove(tile_coord);
            continue;
        };

        if !open_tiles.map.contains_key(tile_coord) {
            open_tiles.map.insert(*tile_coord, OpenTile::default());
        }
        let open_tile = open_tiles.map.get_mut(tile_coord).unwrap();
        open_tile.cells.clear();
        open_tile.cells.resize(
            (nav_mesh_settings.tile_width * nav_mesh_settings.tile_width) as usize,
            OpenCell::default(),
        );
        open_tile.span_count = 0;

        // First we create open spaces.
        for (i, cell) in voxel_tile
            .cells
            .iter()
            .enumerate()
            .filter(|(_, cell)| !cell.spans.is_empty())
        {
            let open_spans = &mut open_tile.cells[i].spans;
            open_spans.clear();

            let mut iter = cell.spans.iter().peekable();
            while let Some(span) = iter.next() {
                if !span.traversable {
                    // Skip untraversable. Not filtered because we still need to peek at them so.
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
                    continue;
                }
                // None above. This is an unbounded open space.
                open_spans.push(OpenSpan {
                    min: span.max,
                    max: None,
                    ..Default::default()
                });
            }
            open_tile.span_count += open_spans.len();
        }

        open_tile.distances.clear();
        open_tile.distances.resize(open_tile.span_count, u16::MAX);
    }
}

pub(super) fn create_neighbour_links_system(
    nav_mesh_settings: Res<NavMeshSettings>,
    mut open_tiles: ResMut<TilesOpen>,
    dirty_tiles: Res<DirtyTiles>,
) {
    let cells_count = nav_mesh_settings.tile_width as usize * nav_mesh_settings.tile_width as usize;

    for tile_coord in dirty_tiles.0.iter() {
        if !open_tiles.map.contains_key(tile_coord) {
            continue;
        }

        // Then we process neighbouring ones.
        let mut distance_index = 0;

        // Work around borrow checker :)
        let mut x_positive = Vec::with_capacity(3);
        let mut x_negative = Vec::with_capacity(3);
        let mut z_positive = Vec::with_capacity(3);
        let mut z_negative = Vec::with_capacity(3);

        for i in 0..cells_count {
            let row = i / nav_mesh_settings.tile_width as usize;
            let column = i % nav_mesh_settings.tile_width as usize;

            let x_positive_contained = column < (nav_mesh_settings.tile_width - 1).into();
            let x_negative_contained = column > 0;
            let z_positive_contained = row < (nav_mesh_settings.tile_width - 1).into();
            let z_negative_contained = row > 0;

            {
                let tile = open_tiles.map.get(tile_coord).unwrap();

                if tile.cells[i].spans.is_empty() {
                    continue;
                }
                x_positive.clear();
                x_negative.clear();
                z_positive.clear();
                z_negative.clear();

                if x_positive_contained {
                    x_positive.extend(tile.cells[i + 1].spans.iter().map(|span| span.min));
                }
                if x_negative_contained {
                    x_negative.extend(tile.cells[i - 1].spans.iter().map(|span| span.min));
                }
                if z_positive_contained {
                    z_positive.extend(
                        tile.cells[i + nav_mesh_settings.tile_width as usize]
                            .spans
                            .iter()
                            .map(|span| span.min),
                    );
                }
                if z_negative_contained {
                    z_negative.extend(
                        tile.cells[i - nav_mesh_settings.tile_width as usize]
                            .spans
                            .iter()
                            .map(|span| span.min),
                    );
                }
            }

            let tile = open_tiles.map.get_mut(tile_coord).unwrap();
            for span in tile.cells[i].spans.iter_mut() {
                // TODO: Also check if we won't hit our head when stepping up or down. Height of lower needs to be at least (walkable_height + difference in minimum)

                let mut neighbours = 0;
                for (i, x_min) in x_negative.iter().enumerate() {
                    if x_min.abs_diff(span.min) < nav_mesh_settings.step_height {
                        span.neighbours[0] = NeighbourConnection::Connected { index: i as u16 };
                        neighbours += 1;
                        break;
                    }
                }

                for (i, z_min) in z_positive.iter().enumerate() {
                    if z_min.abs_diff(span.min) < nav_mesh_settings.step_height {
                        span.neighbours[1] = NeighbourConnection::Connected { index: i as u16 };
                        neighbours += 1;
                        break;
                    }
                }

                for (i, x_min) in x_positive.iter().enumerate() {
                    if x_min.abs_diff(span.min) < nav_mesh_settings.step_height {
                        span.neighbours[2] = NeighbourConnection::Connected { index: i as u16 };
                        neighbours += 1;
                        break;
                    }
                }

                for (i, z_min) in z_negative.iter().enumerate() {
                    if z_min.abs_diff(span.min) < nav_mesh_settings.step_height {
                        span.neighbours[3] = NeighbourConnection::Connected { index: i as u16 };
                        neighbours += 1;
                        break;
                    }
                }

                if neighbours != 4 {
                    tile.distances[distance_index] = 0;
                }
                span.tile_index = distance_index;
                distance_index += 1;
            }
        }
    }
}

pub(super) fn create_distance_field_system(
    nav_mesh_settings: Res<NavMeshSettings>,
    mut open_tiles: ResMut<TilesOpen>,
    dirty_tiles: Res<DirtyTiles>,
) {
    // Pass 1.
    // Crimes most terrible be committed here, against ye' borrow checkin' machine.
    for tile_coord in dirty_tiles.0.iter() {
        let tile_cell_count = {
            let Some(tile) = open_tiles.map.get(tile_coord) else {
                continue;
            };

            tile.cells.len()
        };

        for i in 0..tile_cell_count {
            let cell_span_count = open_tiles.map[tile_coord].cells[i].spans.len();

            for span_i in 0..cell_span_count {
                let (distance, distance_index) = {
                    let tile = &open_tiles.map[tile_coord];
                    let span = &tile.cells[i].spans[span_i];
                    let mut distance = tile.distances[span.tile_index];

                    if let NeighbourConnection::Connected { index } = span.neighbours[0] {
                        // (-1, 0)
                        let (other_span, other_cell_index) =
                            cell_move_back_column(tile, i, index.into());

                        let other_distance = tile.distances[other_span.tile_index];
                        if (other_distance + 2) < distance {
                            distance = other_distance + 2;
                        }

                        // (-1, -1)
                        if let NeighbourConnection::Connected { index } = other_span.neighbours[3] {
                            let (other_span, _) = cell_move_back_row(
                                tile,
                                &nav_mesh_settings,
                                other_cell_index,
                                index.into(),
                            );

                            let other_other_distance = tile.distances[other_span.tile_index];
                            if (other_other_distance + 3) < distance {
                                distance = other_other_distance + 3;
                            }
                        }
                    }

                    if let NeighbourConnection::Connected { index } = span.neighbours[3] {
                        // (0, -1)
                        let (other_span, other_cell_index) =
                            cell_move_back_row(tile, &nav_mesh_settings, i, index.into());

                        let other_distance = tile.distances[other_span.tile_index];
                        if (other_distance + 2) < distance {
                            distance = other_distance + 2;
                        }

                        // (1, -1)
                        if let NeighbourConnection::Connected { index } = other_span.neighbours[2] {
                            let (other_span, _) =
                                cell_move_forward_column(tile, other_cell_index, index.into());

                            let other_other_distance = tile.distances[other_span.tile_index];
                            if (other_other_distance + 3) < distance {
                                distance = other_other_distance + 3;
                            }
                        }
                    }

                    (distance, span.tile_index)
                };

                // Apply distance change.
                let tile = open_tiles.map.get_mut(tile_coord).unwrap();
                tile.distances[distance_index] = distance;
            }
        }
    }

    // Pass 2
    for tile_coord in dirty_tiles.0.iter() {
        let tile_cell_count = {
            let Some(tile) = open_tiles.map.get(tile_coord) else {
                continue;
            };

            tile.cells.len()
        };

        for i in (0..tile_cell_count).rev() {
            let cell_span_count = open_tiles.map[tile_coord].cells[i].spans.len();

            for span_i in 0..cell_span_count {
                let (distance, distance_index) = {
                    let tile = &open_tiles.map[tile_coord];
                    let span = &tile.cells[i].spans[span_i];
                    let mut distance = tile.distances[span.tile_index];

                    if let NeighbourConnection::Connected { index, .. } = span.neighbours[2] {
                        // (1, 0)
                        let (other_span, other_cell_index) =
                            cell_move_forward_column(tile, i, index.into());

                        let other_distance = tile.distances[other_span.tile_index];
                        if (other_distance + 2) < distance {
                            distance = other_distance + 2;
                        }

                        // (1, 1)
                        if let NeighbourConnection::Connected { index, .. } =
                            other_span.neighbours[1]
                        {
                            let (other_span, _) = cell_move_forward_row(
                                tile,
                                &nav_mesh_settings,
                                other_cell_index,
                                index as usize,
                            );

                            let other_other_distance = tile.distances[other_span.tile_index];
                            if (other_other_distance + 3) < distance {
                                distance = other_other_distance + 3;
                            }
                        }
                    }

                    if let NeighbourConnection::Connected { index, .. } = span.neighbours[1] {
                        // (0, 1)
                        let (other_span, other_cell_index) =
                            cell_move_forward_row(tile, &nav_mesh_settings, i, index as usize);

                        let other_distance = tile.distances[other_span.tile_index];
                        if (other_distance + 2) < distance {
                            distance = other_distance + 2;
                        }

                        // (-1, 1)
                        if let NeighbourConnection::Connected { index, .. } =
                            other_span.neighbours[0]
                        {
                            let (other_span, _) =
                                cell_move_back_column(tile, other_cell_index, index.into());

                            let other_other_distance = tile.distances[other_span.tile_index];
                            if (other_other_distance + 3) < distance {
                                distance = other_other_distance + 3;
                            }
                        }
                    }

                    (distance, span.tile_index)
                };

                // Apply distance change.
                let tile = open_tiles.map.get_mut(tile_coord).unwrap();
                tile.distances[distance_index] = distance;
            }

            let tile = open_tiles.map.get_mut(tile_coord).unwrap();
            tile.max_distance = *tile.distances.iter().max().unwrap_or(&0);
        }

        // Box blur. If you're reading this, why?
        let threshold = 2;

        for tile_coord in dirty_tiles.0.iter() {
            let tile_cell_count = {
                let Some(tile) = open_tiles.map.get(tile_coord) else {
                    continue;
                };

                tile.cells.len()
            };

            for i in 0..tile_cell_count {
                let cell_span_count = open_tiles.map[tile_coord].cells[i].spans.len();

                for span_i in 0..cell_span_count {
                    let (distance, distance_index) = {
                        let tile = &open_tiles.map[tile_coord];
                        let span = &tile.cells[i].spans[span_i];
                        let mut distance = tile.distances[span.tile_index];
                        let mut d = distance;

                        if distance <= threshold {
                            distance = threshold;
                        } else {
                            for dir in 0..4 {
                                let NeighbourConnection::Connected { index } = span.neighbours[dir] else {
                                    d += distance * 2;
                                    continue;
                                };

                                let other_cell_index = (i as isize
                                    + get_cell_offset(&nav_mesh_settings, dir))
                                    as usize;
                                let other_span =
                                    &tile.cells[other_cell_index].spans[index as usize];

                                d += tile.distances[other_span.tile_index];

                                let next_dir = (dir + 1) % 4;
                                let NeighbourConnection::Connected { index, .. } = span.neighbours[next_dir] else {
                                    d += distance;
                                    continue;
                                };

                                let other_cell_index = (other_cell_index as isize
                                    + get_cell_offset(&nav_mesh_settings, next_dir))
                                    as usize;
                                let other_span =
                                    &tile.cells[other_cell_index].spans[index as usize];

                                d += tile.distances[other_span.tile_index];
                            }

                            distance = (d + 5) / 9;
                        }

                        (distance, span.tile_index)
                    };

                    // Apply distance change.
                    let tile = open_tiles.map.get_mut(tile_coord).unwrap();
                    tile.distances[distance_index] = distance;
                }
            }
        }
        // End Box Blur
    }
}
