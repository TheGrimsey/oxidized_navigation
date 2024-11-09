use bevy::{log::warn, math::{U16Vec2, U16Vec3, UVec3, Vec3, Vec3Swizzles}, utils::hashbrown::HashMap};

#[cfg(feature = "trace")]
use bevy::log::info_span;

use crate::{get_neighbour_index, heightfields::OpenTile, mesher::{PolyMesh, VERTICES_IN_TRIANGLE}, NavMeshSettings};

#[derive(Debug)]
struct HeightPatch {
    min_x: u16,
    min_y: u16,
    width: u16,
    height: u16,

    /// Heights of the area covered by the patch extracted from OpenTile. 
    heights: Vec<u16>
}

/// Builds a height corrected "detail" mesh from the original poly-mesh.
/// 
/// Adding vertices at points where the height difference compared to the OpenTile is too great.
pub fn build_detail_mesh(
    nav_mesh_settings: &NavMeshSettings,
    open_tile: &OpenTile,
    poly_mesh: &PolyMesh
) -> Option<PolyMesh> {
    let Some(detail_mesh_settings) = &nav_mesh_settings.detail_mesh_generation else {
        return None;
    };

    #[cfg(feature = "trace")]
    let _span = info_span!("Build Detail Mesh").entered();

    let mut max_bounds = U16Vec2::ZERO;

    let polygon_bounds = poly_mesh.polygons.iter().map(|polygon| {
        let mut min = U16Vec2::splat(nav_mesh_settings.tile_width.get());
        let mut max = U16Vec2::ZERO;

        for i in polygon {
            let vertex = poly_mesh.vertices[*i as usize].xz();

            min = min.min(vertex);
            max = max.max(vertex);
        }

        min = min.saturating_sub(U16Vec2::ONE);
        max = (max + U16Vec2::ONE).min(U16Vec2::splat(nav_mesh_settings.tile_width.get()));

        max_bounds = max_bounds.max(max.saturating_sub(min));

        (
            min,
            max
        )
    }).collect::<Vec<_>>();

    let mut height_patch = HeightPatch {
        min_x: 0,
        min_y: 0,
        width: 0,
        height: 0,
        heights: vec![0u16; (max_bounds.x * max_bounds.y) as usize],
    };

    let mut vertices_to_index = HashMap::with_capacity(poly_mesh.vertices.len());
    let mut high_detail_poly_mesh = PolyMesh {
        vertices: Vec::with_capacity(poly_mesh.vertices.len()),
        polygons: Vec::with_capacity(poly_mesh.polygons.len()),
        edges: Vec::with_capacity(poly_mesh.edges.len()),
        areas: Vec::with_capacity(poly_mesh.areas.len()),
        regions: vec![],
    };

    let mut edges = Vec::with_capacity(64);
    let mut polygons = Vec::with_capacity(512);
    let mut samples = Vec::with_capacity(512);
    let mut verts = Vec::with_capacity(256);
    let mut queue = Vec::with_capacity(512);

    for (((polygon, (min, max)), region), area) in poly_mesh.polygons.iter().zip(polygon_bounds.iter()).zip(poly_mesh.regions.iter()).zip(poly_mesh.areas.iter()) {
        let vertices = [
            poly_mesh.vertices[polygon[0] as usize],
            poly_mesh.vertices[polygon[1] as usize],
            poly_mesh.vertices[polygon[2] as usize],
        ];

        height_patch.min_x = min.x as u16;
        height_patch.min_y = min.y as u16;
        height_patch.width = max.x.saturating_sub(min.x) as u16;
        height_patch.height = max.y.saturating_sub(min.y) as u16;

        extract_height_data(nav_mesh_settings, open_tile, &vertices, *region, &mut height_patch, &mut queue);
    
        if !build_poly_detail(&height_patch, &vertices, 2, &mut verts, &mut polygons, &mut edges, &mut samples, detail_mesh_settings.max_height_error.get() as f32, 3, detail_mesh_settings.sample_step.get() as usize) {
            return None;
        }

        // Merge vertices into the high detail poly mesh.
        let mut resolve_vertex = | vertex: U16Vec3 | if let Some(i) = vertices_to_index.get(&vertex) {
            *i
        } else {
            let i = high_detail_poly_mesh.vertices.len() as u32;
            high_detail_poly_mesh.vertices.push(vertex);

            vertices_to_index.insert(vertex, i);

            i
        };

        high_detail_poly_mesh.polygons.extend(polygons.iter().map(|[a,b,c]| {
            let vertex_a = resolve_vertex(verts[*a as usize]);
            let vertex_b = resolve_vertex(verts[*b as usize]);
            let vertex_c = resolve_vertex(verts[*c as usize]);

            [
                vertex_a,
                vertex_b,
                vertex_c
            ]
        }));

        high_detail_poly_mesh.areas.extend([*area].repeat(polygons.len()));
    }

    Some(high_detail_poly_mesh)
}

fn extract_height_data(
    nav_mesh_settings: &NavMeshSettings,
    open_tile: &OpenTile,
    triangle_vertices: &[U16Vec3],
    region: u16,
    height_patch: &mut HeightPatch,
    queue: &mut Vec<(usize, usize)>,
) {
    queue.clear();

    height_patch.heights.fill(u16::MAX);

    let tile_side = nav_mesh_settings.get_tile_side_with_border();
    let height_patch_width = height_patch.width as usize;
    
    let mut empty = true;
    for y in 0..height_patch.height {
        // Including walkable radius because it acts as a buffer zone around the tile
        // but this is not included in the poly mesh.
        let cell_y = y + height_patch.min_y + nav_mesh_settings.walkable_radius;

        for x in 0..height_patch.width {
            let cell_x = x + height_patch.min_x + nav_mesh_settings.walkable_radius;
            let cell_i = cell_x as usize + cell_y as usize * tile_side;
            let cell = &open_tile.cells[cell_i];

            for (span_i, span) in cell.spans.iter().enumerate() {
                if span.region == region {
                    height_patch.heights[x as usize + y as usize * height_patch.width as usize] = span.min;
                    empty = false;

                    let border = span.neighbours.iter()
                        .enumerate().filter_map(|(i, neighbour)| Some(i).zip(*neighbour))
                        .any(|(i, neighbour)| {
                            let neighbour_i = get_neighbour_index(tile_side, cell_i, i);

                            open_tile.cells[neighbour_i].spans[neighbour as usize].region != region
                        });

                    if border {
                        queue.push((cell_i, span_i));
                    }
                    break;
                }
            }
        }
    }

    // If no seed points were found, fall back to the polygon center
    if empty {
        seed_array_with_poly_center(open_tile, triangle_vertices, nav_mesh_settings, queue, height_patch);
    }

    // If we go over this, we clear out the first retract_size elements in the queue.
    let retract_size = 256;
    let mut head = 0;

    while head < queue.len() {
        let (cell_i, span_i) = queue[head];
        
        head += 1;

        if head >= retract_size {
            queue.drain(..retract_size);
            head = 0;
        }

        let open_cell = &open_tile.cells[cell_i];
        let open_span = &open_cell.spans[span_i];

        for (i, neighbour) in open_span.neighbours.iter().enumerate().filter_map(|(i, neighbour)| Some(i).zip(*neighbour)) {
            let neighbour_i = get_neighbour_index(tile_side, cell_i, i);
            
            let x = neighbour_i % tile_side;
            let y = neighbour_i / tile_side;

            let height_patch_x = x as isize - height_patch.min_x as isize - nav_mesh_settings.walkable_radius as isize;
            let height_patch_y = y as isize - height_patch.min_y as isize - nav_mesh_settings.walkable_radius as isize;

            if height_patch_x < 0 || height_patch_y < 0 || height_patch_y >= height_patch.height as isize || height_patch_x >= height_patch.width as isize {
                continue;
            }

            if height_patch.heights[height_patch_x as usize + height_patch_y as usize * height_patch.width as usize] != u16::MAX {
                continue;
            }

            let neighbour_span = &open_tile.cells[neighbour_i].spans[neighbour as usize];

            height_patch.heights[height_patch_x as usize + height_patch_y as usize * height_patch.width as usize] = neighbour_span.min;

            queue.push((neighbour_i, neighbour as usize));
        }
    }
}

fn seed_array_with_poly_center(
    open_tile: &OpenTile,
    vertices: &[U16Vec3],
    nav_mesh_settings: &NavMeshSettings,
    queue: &mut Vec<(usize, usize)>,
    height_patch: &mut HeightPatch
) {
    const OFFSETS: [(i16, i16); 9] = [
        (0, 0), (-1, -1), (0, -1), (1, -1), (1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0),
    ];

    let mut start_cell = None;
    let mut span_height_distance_to_vertex = u16::MAX;

    let min_x = height_patch.min_x;
    let max_x = height_patch.min_x + height_patch.width;
    
    let min_y = height_patch.min_y;
    let max_y = height_patch.min_y + height_patch.height;

    let tile_side = nav_mesh_settings.get_tile_side_with_border();

    for &vertex in vertices {
        for &(offset_x, offset_y) in &OFFSETS {
            let ax = vertex.x.saturating_add_signed(offset_x);
            let ay = vertex.y;
            let az = vertex.z.saturating_add_signed(offset_y);

            if ax < min_x
                || ax >= max_x
                || az < min_y
                || az >= max_y
            {
                continue;
            }

            let cell_i = (ax + nav_mesh_settings.walkable_radius) as usize + (az + nav_mesh_settings.walkable_radius) as usize * tile_side;
            let cell = &open_tile.cells[cell_i];
            for (span_i, open_span) in cell.spans.iter().enumerate() {
                let height_difference = ay.abs_diff(open_span.min);
                if height_difference < span_height_distance_to_vertex {
                    start_cell = Some((cell_i, span_i));
                    span_height_distance_to_vertex = height_difference;
                    
                    if span_height_distance_to_vertex == 0 {
                        break;
                    }
                }
            }
            
            if span_height_distance_to_vertex == 0 {
                break;
            }
        }
        
        if span_height_distance_to_vertex == 0 {
            break;
        }
    }

    let Some(start_cell) = start_cell else {
        return;
    };

    // Initialize the DFS stack with the start cell
    queue.clear();
    queue.push(start_cell);
}

fn distance_pt_seg(point: Vec3, va: Vec3, vb: Vec3) -> f32 {
    let ab = vb - va;
    let ap = point - va;
    let proj = ap.dot(ab) / ab.length_squared();
    let closest = if proj < 0.0 {
        va
    } else if proj > 1.0 {
        vb
    } else {
        va + ab * proj
    };
    (point - closest).length_squared()
}

fn build_poly_detail(
    height_patch: &HeightPatch,
    // Vertices of the polygon we are currently building detail for.
    poly: &[U16Vec3],
    sample_distance: u32,
    verts: &mut Vec<U16Vec3>,
    triangles: &mut Vec<[u32; VERTICES_IN_TRIANGLE]>,
    edges: &mut Vec<u32>,
    samples: &mut Vec<U16Vec3>,
    sample_max_error: f32,
    search_radius: u16,
    sample_step: usize
) -> bool {
    const MAX_VERTS: usize = 127;
    const MAX_VERTS_PER_EDGE: usize = 32;
    let mut edge = [U16Vec3::ZERO; (MAX_VERTS_PER_EDGE+1)];
    let mut hull = Vec::with_capacity(MAX_VERTS);

    verts.clear();
    verts.extend(poly.iter().cloned());
    edges.clear();
    triangles.clear();
    samples.clear();

    let min_extent = poly_min_extent(verts);

    // Tesselate outlines.
    if sample_distance > 0 {
        for i in 0..poly.len() {
            let j = (i + poly.len() - 1) % poly.len();
            let mut vertex_j = poly[j];
            let mut vertex_i = poly[i];
            let mut swapped = false;

            if vertex_j.x.abs_diff(vertex_i.x) == 0 {
                if vertex_j.z > vertex_i.z {
                    std::mem::swap(&mut vertex_j, &mut vertex_i);
                    swapped = true;
                }
            } else if vertex_j.x > vertex_i.x {
                std::mem::swap(&mut vertex_j, &mut vertex_i);
                swapped = true;
            }

            let delta = vertex_i.as_vec3() - vertex_j.as_vec3();
            let d = (delta.x * delta.x + delta.z * delta.z).sqrt();

            let mut nn = (1 + (d / sample_distance as f32).floor() as usize).min(MAX_VERTS_PER_EDGE - 1);
            if verts.len() + nn >= MAX_VERTS {
                nn = (MAX_VERTS - 1).saturating_sub(verts.len());
            }

            for k in 0..=nn {
                let t = k as f32 / nn as f32;
                let mut pos = vertex_j.as_vec3().lerp(vertex_i.as_vec3(), t).floor().as_uvec3();
                pos.y = get_height(pos.x, pos.y, pos.z, search_radius, height_patch) as u32;
                
                edge[k] = pos.as_u16vec3();
            }

            // Simplify samples
            let mut idx = vec![0, nn];
            let mut k = 0;
            while k < idx.len() - 1 {
                let a = idx[k];
                let b = idx[k + 1];
                let vertex_a = edge[a];
                let vertex_b = edge[b];
            
                // Find maximum deviation along the segment
                let mut max_dev = 0.0;
                let mut max_i = None;
                for m in a + 1..b {
                    let dev = distance_pt_seg(edge[m].as_vec3(), vertex_a.as_vec3(), vertex_b.as_vec3());
                    if dev > max_dev {
                        max_dev = dev;
                        max_i = Some(m);
                    }
                }
            
                // Add new point if deviation is greater than sample_max_error
                if let Some(max_i) = max_i {
                    if max_dev > sample_max_error * sample_max_error {
                        idx.insert(k + 1, max_i);
                    } else {
                        k += 1;
                    }
                } else {
                    k += 1;
                }
            }
            
            // Record the hull
            hull.push(j);
            
            // Add new vertices
            if swapped {
                for &k in idx.iter().rev().skip(1).take(idx.len() - 2) {
                    verts.push(edge[k]);
                    hull.push(verts.len() - 1);
                }
            } else {
                for &k in idx.iter().skip(1).take(idx.len() - 2) {
                    verts.push(edge[k]);
                    hull.push(verts.len() - 1);
                }
            }
        }
    }

    triangulate_hull(&verts, &hull, poly.len(), triangles);

    if min_extent < (sample_distance * 2) as f32 || triangles.is_empty() {
        return true;
    }

    if sample_distance > 0 {
        // Except for Y this is stored in the min & max of the height_patch :thinking:
        let mut min_bounds = poly[0];
        let mut max_bounds = poly[0];

        for vertex in poly.iter().skip(1) {
            min_bounds = min_bounds.min(*vertex);
            max_bounds = max_bounds.max(*vertex);
        }
        let y = ((max_bounds.y as f32 + min_bounds.y as f32) * 0.5).floor() as u16;

        for z in (min_bounds.z..max_bounds.z).step_by(sample_step) {
            for x in (min_bounds.x..max_bounds.x).step_by(sample_step) {
                let point = U16Vec3::new(
                    x,
                    y,
                    z,
                );
    
                // Make sure the samples are not too close to the edges.
                let distance = dist_to_poly(poly, point.as_vec3());
                if distance > -(sample_distance as f32) / 2.0 {
                    continue;
                }
    
                let y = get_height(point.x.into(), point.y.into(), point.z.into(), search_radius, height_patch);

                // Coordinates can't be greater than a u16 since we limit a tile to max u16 in a side.
                samples.push(point.with_y(y));
            }
        }

        // Make sure there is at least one sample at the center of the polygon.
        if samples.is_empty() {
            let point_center = poly.iter().fold(UVec3::ZERO, |acc, entry| acc + entry.as_uvec3()) / poly.len() as u32;

            let y = get_height(point_center.x, point_center.y, point_center.z, search_radius, height_patch);

            samples.push(point_center.as_u16vec3().with_y(y));
        }

        // Find and add samples with the largest errors
        let nsamples = samples.len();
        for _ in 0..nsamples {
            if verts.len() >= MAX_VERTS {
                break;
            }

            // Find the sample with the most error
            let mut best_point = U16Vec3::ZERO;
            let mut best_distance = 0.0;
            let mut best_i = None;

            for (i, sample) in samples.iter().enumerate() {
                // Calculate distance to the mesh.
                // Can't fail because we only tesselate the inside of the mesh & we purge all outside points earlier.
                let Some(d) = dist_to_tri_mesh(sample.as_vec3(), &verts, &triangles) else {
                    continue;
                };

                if d > best_distance {
                    best_distance = d;
                    best_i = Some(i);
                    best_point = *sample;
                }
            }

            // Stop tessellating if error is within the threshold or no sample found
            if best_distance <= sample_max_error {
                break;
            }
            let Some(best_i) = best_i else {
                break;
            };

            samples.swap_remove(best_i);

            // Add the new sample point to verts
            verts.push(best_point);

            // Rebuild triangulation
            edges.clear();
            triangles.clear();
            delaunay_hull(&verts, &hull, triangles, edges);
        }
    }

    // If we failed to add more points, let's just triangulate with the hull again.
    if triangles.is_empty() {
        triangulate_hull(&verts, &hull, poly.len(), triangles);
    }

    return true;
}

fn get_height(
    fx: u32,
    fy: u32,
    fz: u32,
    radius: u16,
    height_patch: &HeightPatch,
) -> u16 {
    // Convert coordinates to cell indices
    let mut initial_x = fx;
    let mut initial_z = fz;
    initial_x = initial_x.saturating_sub(height_patch.min_x.into()).clamp(0, (height_patch.width - 1).into());
    initial_z = initial_z.saturating_sub(height_patch.min_y.into()).clamp(0, (height_patch.height - 1).into());

    let mut h = height_patch.heights[(initial_x + initial_z * height_patch.width as u32) as usize];
    if h == u16::MAX {
        // Special case: search adjacent cells in a spiral up to `radius` for valid height data
        let mut x = 1;
        let mut z = 0;
        let mut dx = 1;
        let mut dz = 0;
        let max_size = radius * 2 + 1;
        let max_iter = max_size * max_size - 1;

        let mut next_ring_iter_start = 8;
        let mut next_ring_iters = 16;

        let mut min_distance = u32::MAX;
        for i in 0..max_iter {
            let nx = initial_x as i32 + x;
            let nz = initial_z as i32 + z;

            if nx >= 0 && nz >= 0 && nx < height_patch.width.into() && nz < height_patch.height.into() {
                let new_height = height_patch.heights[(nx + nz * height_patch.width as i32) as usize];
                if new_height != u16::MAX {
                    let d = (new_height as u32).abs_diff(fy);
                    if d < min_distance {
                        h = new_height;
                        min_distance = d;
                    }
                }
            }

            // If we are about to enter the next ring and have found a valid height, we stop the search
            if i + 1 == next_ring_iter_start {
                if h != u16::MAX {
                    break;
                }
                next_ring_iter_start += next_ring_iters;
                next_ring_iters += 8;
            }

            // Update x and z to move in a spiral pattern, adjusting dx and dz as unsigned
            if x == z || (x < 0 && x == -z) || (x > 0 && x == (1 - z)) {
                let temp = dx;
                dx = -dz;
                dz = temp;
            }
            x += dx;
            z += dz;
        }
    }
    h
}

fn dist_to_poly(poly: &[U16Vec3], p: Vec3) -> f32 {
    let mut dmin = f32::MAX;
    let mut c = false;
    let nvert = poly.len();

    for i in 0..nvert {
        let vi = poly[i].as_vec3();
        let vj = poly[(i + nvert - 1) % nvert].as_vec3();

        // Check if point `p` is inside the polygon (even-odd rule)
        if ((vi.z > p.z) != (vj.z > p.z)) &&
            (p.x < (vj.x - vi.x) * (p.z - vi.z) / (vj.z - vi.z) + vi.x) {
            c = !c;
        }

        // Calculate the minimum distance from point `p` to the polygon edges
        dmin = dmin.min(distance_pt_seg_2d(p, vj, vi));
    }

    if c { -dmin } else { dmin }
}

// Computes the minimum distance from point `p` to the triangle mesh defined by `verts` and `tris`.
fn dist_to_tri_mesh(p: Vec3, verts: &[U16Vec3], tris: &[[u32; VERTICES_IN_TRIANGLE]]) -> Option<f32> {
    let mut minimum_distance = None;

    for [a, b, c] in tris.iter() {
        let va = verts[*a as usize].as_vec3();
        let vb = verts[*b as usize].as_vec3();
        let vc = verts[*c as usize].as_vec3();

        // Calculate the distance from point `p` to the triangle `va, vb, vc`
        let distance = dist_point_to_triangle(p, va, vb, vc);
        if minimum_distance.is_none_or(|minimum_distance| distance.is_some_and(|distance| distance < minimum_distance)) {
            minimum_distance = distance;
        }
    }

    minimum_distance
}

// Helper function to calculate the 2D distance from a point `p` to a segment `a-b` in the XZ plane.
fn distance_pt_seg_2d(pt: Vec3, p: Vec3, q: Vec3) -> f32 {
    let pqx = q.x - p.x;
    let pqz = q.z - p.z;

    let dx = pt.x - p.x;
    let dz = pt.z - p.z;

    let d = pqx * pqx + pqz * pqz;
    let mut t = pqx * dx + pqz * dz;

    if d > 0.0 {
        t /= d;
    }
    t = t.clamp(0.0, 1.0);

    let dx = p.x + t * pqx - pt.x;
    let dz = p.z + t * pqz - pt.z;

    dx * dx + dz * dz
}

// Calculates the distance from a point `p` to a triangle `a, b, c`.
fn dist_point_to_triangle(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Option<f32> {
    // Calculate vectors
    let v0 = c - a;
    let v1 = b - a;
    let v2 = p - a;

    // Compute dot products in the XZ plane
    let dot00 = v0.x * v0.x + v0.z * v0.z;
    let dot01 = v0.x * v1.x + v0.z * v1.z;
    let dot02 = v0.x * v2.x + v0.z * v2.z;
    let dot11 = v1.x * v1.x + v1.z * v1.z;
    let dot12 = v1.x * v2.x + v1.z * v2.z;

    // Compute barycentric coordinates
    let inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    let u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
    let v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

    // Check if point is inside the triangle
    const EPS: f32 = 1e-4;
    if u >= -EPS && v >= -EPS && (u + v) <= 1.0 + EPS {
        // Calculate interpolated y-coordinate
        let y = a.y + u * v0.y + v * v1.y;
        Some((y - p.y).abs()) // Return the vertical distance from `p` to the triangle plane
    } else {
        None // Point is outside the triangle
    }
}

fn prev(i: usize, len: usize) -> usize {
    (i + len - 1) % len
}

// `next` function for moving to the next index in the hull
fn next(i: usize, len: usize) -> usize {
    (i + 1) % len
}

// Function to triangulate the hull
fn triangulate_hull(verts: &[U16Vec3], hull: &[usize], nin: usize, tris: &mut Vec<[u32; VERTICES_IN_TRIANGLE]>) {
    let mut start = 0;
    let mut left = 1;
    let mut right = hull.len() - 1;

    // Start from an ear with the shortest perimeter
    let mut min_perimeter = f32::MAX;
    for i in 0..hull.len() {
        if hull[i] >= nin { continue; } // Skip segments on edges, only process original vertices as middle of triangles

        let pi = prev(i, hull.len());
        let ni = next(i, hull.len());

        let pv = verts[hull[pi]].as_vec3();
        let cv = verts[hull[i]].as_vec3();
        let nv = verts[hull[ni]].as_vec3();

        let perimeter = pv.xz().distance(cv.xz()) + cv.xz().distance(nv.xz()) + nv.xz().distance(pv.xz());
        
        if perimeter < min_perimeter {
            start = i;
            left = ni;
            right = pi;
            min_perimeter = perimeter;
        }
    }

    // Add the first triangle
    tris.push([
        hull[start] as u32,
        hull[left] as u32,
        hull[right] as u32
    ]);

    // Triangulate the polygon by moving left or right based on shortest perimeter
    while next(left, hull.len()) != right {
        let nleft = next(left, hull.len());
        let nright = prev(right, hull.len());

        let cv_left = verts[hull[left]].as_vec3();
        let nv_left = verts[hull[nleft]].as_vec3();
        let cv_right = verts[hull[right]].as_vec3();
        let nv_right = verts[hull[nright]].as_vec3();

        let dleft = cv_left.xz().distance(nv_left.xz()) + nv_left.xz().distance(cv_right.xz());
        let dright = cv_right.xz().distance(nv_right.xz()) + cv_left.xz().distance(nv_right.xz());

        if dleft < dright {
            tris.push([
                hull[left] as u32,
                hull[nleft] as u32,
                hull[right] as u32
            ]);
            left = nleft;
        } else {
            tris.push([
                hull[left] as u32,
                hull[nright] as u32,
                hull[right] as u32
            ]);
            right = nright;
        }
    }
}

fn delaunay_hull(
    vertices: &[U16Vec3],
    hull: &[usize],
    triangles: &mut Vec<[u32; VERTICES_IN_TRIANGLE]>,
    edges: &mut Vec<u32>,
) {
    let mut num_faces = 0;
    let mut num_edges = 0;
    let max_edges = vertices.len() * 10;
    edges.resize(max_edges * 4, u32::MAX);

    // Initialize hull edges
    for i in 0..hull.len() {
        let j = if i == 0 { hull.len() - 1 } else { i - 1 };
        add_edge(edges, &mut num_edges, max_edges, hull[j] as u32, hull[i] as u32, u32::MAX, u32::MAX);
    }

    // Complete facets
    let mut current_edge = 0;
    while current_edge < num_edges {
        if edges[current_edge * 4 + 2] == u32::MAX {
            complete_facet(vertices, edges, &mut num_edges, max_edges, &mut num_faces, current_edge);
        }
        if edges[current_edge * 4 + 3] == u32::MAX {
            complete_facet(vertices, edges, &mut num_edges, max_edges, &mut num_faces, current_edge);
        }
        current_edge += 1;
    }

    // Initialize triangles
    triangles.resize(num_faces, [u32::MAX; VERTICES_IN_TRIANGLE]);

    // Populate triangles from edges
    for i in 0..num_edges {
        let e = &edges[i * 4..(i + 1) * 4];
        if e[3] != u32::MAX {
            // Left face
            let t = &mut triangles[e[3] as usize];
            if t[0] == u32::MAX {
                t[0] = e[0];
                t[1] = e[1];
            } else if t[0] == e[1] {
                t[2] = e[0];
            } else if t[1] == e[0] {
                t[2] = e[1];
            }
        }
        if e[2] != u32::MAX {
            // Right face
            let t = &mut triangles[e[2] as usize];
            if t[0] == u32::MAX {
                t[0] = e[1];
                t[1] = e[0];
            } else if t[0] == e[0] {
                t[2] = e[1];
            } else if t[1] == e[1] {
                t[2] = e[0];
            }
        }
    }

    // Remove dangling faces
    triangles.retain(|triangle| triangle[0] != u32::MAX && triangle[1] != u32::MAX && triangle[2] != u32::MAX);
}


fn complete_facet(
    vertices: &[U16Vec3],
    edges: &mut [u32],
    nedges: &mut usize,
    max_edges: usize,
    nfaces: &mut usize,
    e: usize,
) {
    const EPS: f32 = 1e-5;

    let edge = &mut edges[e * 4..(e + 1) * 4];

    // Cache `s` and `t`
    let (s, t) = if edge[2] == u32::MAX {
        (edge[0], edge[1])
    } else if edge[3] == u32::MAX {
        (edge[1], edge[0])
    } else {
        // Edge already completed
        return;
    };

    // Find the best point on the left of the edge
    let mut pt = vertices.len();
    let mut c = Vec3::ZERO;
    let mut r = -1.0;
    for u in 0..vertices.len() {
        if u == s as usize || u == t as usize {
            continue;
        }
        if vcross2(vertices[s as usize].as_vec3(), vertices[t as usize].as_vec3(), vertices[u].as_vec3()) > EPS {
            if r < 0.0 {
                // The circumcircle is not updated yet, do it now
                pt = u;
                circum_circle(vertices[s as usize].as_vec3(), vertices[t as usize].as_vec3(), vertices[u].as_vec3(), &mut c, &mut r);
                continue;
            }
            let d = c.xz().distance(vertices[u].as_vec3().xz());
            let tol = 0.001;
            if d > r * (1.0 + tol) {
                // Outside current circumcircle, skip
                continue;
            } else if d < r * (1.0 - tol) {
                // Inside safe circumcircle, update circle
                pt = u;
                circum_circle(vertices[s as usize].as_vec3(), vertices[t as usize].as_vec3(), vertices[u].as_vec3(), &mut c, &mut r);
            } else {
                // Inside epsilon circumcircle, do extra tests to ensure edge validity
                if overlap_edges(vertices, edges, *nedges, s, u as u32) {
                    continue;
                }
                if overlap_edges(vertices, edges, *nedges, t, u as u32) {
                    continue;
                }
                // Edge is valid
                pt = u;
                circum_circle(vertices[s as usize].as_vec3(), vertices[t as usize].as_vec3(), vertices[u].as_vec3(), &mut c, &mut r);
            }
        }
    }

    // Add new triangle or update edge info if s-t is on hull
    if pt < vertices.len() {
        // Update face information of edge being completed
        update_left_face(&mut edges[e * 4..(e + 1) * 4], s, t, *nfaces as u32);

        // Add new edge or update face info of old edge
        let e = find_edge(edges, *nedges, pt as u32, s);
        if let Some(e) = e {
            update_left_face(&mut edges[e as usize * 4..(e as usize + 1) * 4], pt as u32, s, *nfaces as u32);
        } else {
            add_edge(edges, nedges, max_edges, pt as u32, s, *nfaces as u32, u32::MAX);
        }

        // Add new edge or update face info of old edge
        let e = find_edge(edges, *nedges, t, pt as u32);
        if let Some(e) = e {
            update_left_face(&mut edges[e as usize * 4..(e as usize + 1) * 4], t, pt as u32, *nfaces as u32);
        } else {
            add_edge(edges, nedges, max_edges, t, pt as u32, *nfaces as u32, u32::MAX);
        }

        *nfaces += 1;
    } else {
        update_left_face(&mut edges[e * 4..(e + 1) * 4], s, t, u32::MAX);
    }
}

fn circum_circle(p1: Vec3, p2: Vec3, p3: Vec3, c: &mut Vec3, r: &mut f32) -> bool {
    const EPS: f32 = 1e-6;

    // Calculate vectors relative to p1 to avoid precision issues.
    let v1 = Vec3::ZERO;
    let v2 = p2 - p1;
    let v3 = p3 - p1;

    let cp = vcross2(v1, v2, v3);
    if cp.abs() > EPS {
        let v1_sq = v1.xz().length_squared();
        let v2_sq = v2.xz().length_squared();
        let v3_sq = v3.xz().length_squared();

        c.x = (v1_sq * (v2.z - v3.z) + v2_sq * (v3.z - v1.z) + v3_sq * (v1.z - v2.z)) / (2.0 * cp);
        c.y = 0.0;
        c.z = (v1_sq * (v3.x - v2.x) + v2_sq * (v1.x - v3.x) + v3_sq * (v2.x - v1.x)) / (2.0 * cp);

        *r = c.xz().distance(v1.xz());
        *c += p1;

        true
    } else {
        *c = p1;
        *r = 0.0;
        false
    }
}

fn overlap_seg_seg_2d(a: Vec3, b: Vec3, c: Vec3, d: Vec3) -> bool {
    let a1 = vcross2(a, b, d);
    let a2 = vcross2(a, b, c);
    if a1 * a2 < 0.0 {
        let a3 = vcross2(c, d, a);
        let a4 = a3 + a2 - a1;
        if a3 * a4 < 0.0 {
            return true;
        }
    }
    false
}

fn overlap_edges(vertices: &[U16Vec3], edges: &[u32], nedges: usize, s1: u32, t1: u32) -> bool {
    for i in 0..nedges {
        let s0 = edges[i * 4];
        let t0 = edges[i * 4 + 1];
        
        // Skip if edges are the same or connected
        if s0 == s1 || s0 == t1 || t0 == s1 || t0 == t1 {
            continue;
        }

        // Check if the edges overlap in 2D
        if overlap_seg_seg_2d(vertices[s0 as usize].as_vec3(), vertices[t0 as usize].as_vec3(), vertices[s1 as usize].as_vec3(), vertices[t1 as usize].as_vec3()) {
            return true;
        }
    }
    false
}

// Cross product in the XZ plane to determine if the point is on the left of the edge
fn vcross2(p1: Vec3, p2: Vec3, p3: Vec3) -> f32 {
    let u1 = p2.x - p1.x;
    let v1 = p2.z - p1.z;
    let u2 = p3.x - p1.x;
    let v2 = p3.z - p1.z;
    u1 * v2 - v1 * u2
}

// Update the left face of an edge
fn update_left_face(edge: &mut [u32], s: u32, t: u32, f: u32) {
    if edge[0] == s && edge[1] == t && edge[2] == u32::MAX {
        edge[2] = f;
    } else if edge[1] == s && edge[0] == t && edge[3] == u32::MAX {
        edge[3] = f;
    }
}

fn find_edge(edges: &[u32], nedges: usize, s: u32, t: u32) -> Option<u32> {
    for i in 0..nedges {
        let e = &edges[i * 4..(i + 1) * 4];
        if (e[0] == s && e[1] == t) || (e[0] == t && e[1] == s) {
            return Some(i as u32);
        }
    }
    None
}

fn add_edge(
    edges: &mut [u32],
    num_edges: &mut usize,
    max_edges: usize,
    s: u32,
    t: u32,
    l: u32,
    r: u32,
) -> Option<u32> {
    if *num_edges >= max_edges {
        warn!("addEdge: Too many edges ({}/{})", *num_edges, max_edges);
        return None;
    }

    // Add edge if not already in the triangulation
    let e = find_edge(edges, *num_edges, s, t);
    if e.is_none() {
        let edge = &mut edges[*num_edges * 4..(*num_edges + 1) * 4];
        edge[0] = s;
        edge[1] = t;
        edge[2] = l;
        edge[3] = r;
        *num_edges += 1;
        
        Some(*num_edges as u32 - 1)
    } else {
        None
    }
}

fn poly_min_extent(
    vertices: &[U16Vec3]
) -> f32 {
    let mut min_distance = f32::MAX;
    for i in 0..vertices.len() {
        let next_i = (i + 1) % vertices.len();
        let vertex = vertices[i];
        let next_vertex = vertices[next_i];

        let mut max_edge_distance = 0.0_f32;
        for j in 0..vertices.len() {
            if j == i || j == next_i {
                continue;
            }

            let distance = distance_pt_seg_2d(vertices[j].as_vec3(), vertex.as_vec3(), next_vertex.as_vec3());

            max_edge_distance = max_edge_distance.max(distance);
        }

        min_distance = min_distance.min(max_edge_distance);
    }

    min_distance.sqrt()
}