use bevy::{
    prelude::{info, Res, ResMut, Resource, UVec2, UVec3, UVec4},
    utils::HashMap,
};

use super::{
    intersect_prop, intersect_segment, left, left_on, DirtyTiles, NavMeshSettings, TileContours,
};

#[derive(Default)]
pub struct PolyMesh {
    pub vertices: Vec<UVec3>,
    pub indices: Vec<[u32; VERTICES_PER_POLYGON]>, //
    edges: Vec<[EdgeConnectionType; VERTICES_PER_POLYGON]>, // For each polygon edge points to a polygon (if any) that shares the edge.
    regions: Vec<u16>, // Region Id for each polygon. TODO: Is this usefull beyond debugging?
    portal_edge_count: usize,
}

#[derive(Default, Resource)]
pub(super) struct TilePolyMesh {
    map: HashMap<UVec2, PolyMesh>,
}

const VERTEX_BUCKET_COUNT: usize = 1 << 12; // 4 096
const VERTICES_PER_POLYGON: usize = 3; // Don't change this. The mesher can't make anything other than triangles.

pub(super) fn build_poly_mesh_system(
    nav_mesh_settings: Res<NavMeshSettings>,
    contours: Res<TileContours>,
    mut poly_meshes: ResMut<TilePolyMesh>,
    dirty_tiles: Res<DirtyTiles>,
) {
    for tile_coord in dirty_tiles.0.iter() {
        let Some(contour_set) = contours.map.get(tile_coord) else {
            continue;
        };

        let mut max_vertices = 0;
        let mut max_tris = 0;
        let mut max_verts_per_contour = 0;

        for contour in &contour_set.contours {
            if contour.vertices.len() < 3 {
                continue;
            }

            max_vertices += contour.vertices.len();
            max_tris += contour.vertices.len() - 2;
            max_verts_per_contour = contour.vertices.len().max(max_verts_per_contour);
        }

        let mut poly_mesh = PolyMesh {
            vertices: Vec::with_capacity(max_vertices),
            regions: Vec::with_capacity(max_tris),
            indices: Vec::with_capacity(max_tris),
            edges: Vec::with_capacity(max_tris),
            portal_edge_count: 0,
        };

        let mut first_vertex = vec![-1; VERTEX_BUCKET_COUNT];
        let mut next_vertex = vec![0; max_vertices];

        let mut indices = Vec::with_capacity(max_verts_per_contour);
        let mut triangles = Vec::with_capacity(max_verts_per_contour * 3);
        let mut polygons = Vec::with_capacity((max_verts_per_contour + 1) * VERTICES_PER_POLYGON);

        for contour in &contour_set.contours {
            if contour.vertices.len() < 3 {
                continue;
            }

            indices.clear();
            triangles.clear();
            polygons.clear();

            indices.extend((0..contour.vertices.len()).map(|val| val as u32));

            if !triangulate(&contour.vertices, &mut indices, &mut triangles) {
                info!(
                    "Triangulation failed, skipping contour for region {}.",
                    contour.region
                );
                continue;
            }

            for vertex in contour.vertices.iter() {
                let index = add_vertex(
                    vertex.truncate(),
                    &mut poly_mesh.vertices,
                    &mut first_vertex,
                    &mut next_vertex,
                );
                indices.push(index);
            }

            let triangle_count = triangles.len() / 3;
            for i in 0..triangle_count {
                let a = triangles[i * 3];
                let b = triangles[i * 3 + 1];
                let c = triangles[i * 3 + 2];

                if a != b && a != c && b != c {
                    polygons.push([
                        indices[a as usize],
                        indices[b as usize],
                        indices[c as usize],
                    ]);
                }
            }

            if polygons.is_empty() {
                continue;
            }

            // Store polygons.
            poly_mesh.indices.extend(polygons.iter());
            for _ in 0..polygons.len() {
                poly_mesh.regions.push(contour.region);
            }
        }

        // For each edge, find other polygon that shares that edge.
        build_mesh_adjacency(
            &poly_mesh.indices,
            poly_mesh.vertices.len(),
            &mut poly_mesh.edges,
        );

        // Fix portal edges.
        let polygon_count = poly_mesh.indices.len() / VERTICES_PER_POLYGON;
        for i in 0..polygon_count {
            let indices = &poly_mesh.indices[i];
            for index in 0..indices.len() {
                // Connect to edges that don't have an internal edge connection.
                let EdgeConnectionType::None = poly_mesh.edges[i * VERTICES_PER_POLYGON][index] else {
                    continue;
                };

                let vertex_a = poly_mesh.vertices[indices[index] as usize];
                let vertex_b = poly_mesh.vertices[indices[(index + 1) % indices.len()] as usize];

                // Only edges parallel to the tile edge.
                if vertex_a.x == 0 && vertex_b.x == 0 {
                    poly_mesh.edges[i][index] =
                        EdgeConnectionType::OffMesh(EdgeConnectionDirection::XNegative);
                    poly_mesh.portal_edge_count += 1;
                } else if vertex_a.z == nav_mesh_settings.tile_width as u32
                    && vertex_b.z == nav_mesh_settings.tile_width as u32
                {
                    poly_mesh.edges[i][index] =
                        EdgeConnectionType::OffMesh(EdgeConnectionDirection::ZPositive);
                    poly_mesh.portal_edge_count += 1;
                } else if vertex_a.x == nav_mesh_settings.tile_width as u32
                    && vertex_b.x == nav_mesh_settings.tile_width as u32
                {
                    poly_mesh.edges[i][index] =
                        EdgeConnectionType::OffMesh(EdgeConnectionDirection::XPositive);
                    poly_mesh.portal_edge_count += 1;
                } else if vertex_a.z == 0 && vertex_b.z == 0 {
                    poly_mesh.edges[i][index] =
                        EdgeConnectionType::OffMesh(EdgeConnectionDirection::ZNegative);
                    poly_mesh.portal_edge_count += 1;
                }
            }
        }

        info!("Mesher Output for {:?}:", tile_coord);
        info!("Vertices: {:?}", poly_mesh.vertices);
        info!("Indices: {:?}", poly_mesh.indices);
        info!("Edges: {:?}", poly_mesh.edges);
        poly_meshes.map.insert(*tile_coord, poly_mesh);
    }
}

#[derive(Clone, Copy, Debug)]
pub enum EdgeConnectionDirection {
    XNegative,
    ZPositive,
    XPositive,
    ZNegative,
}

#[derive(Clone, Copy, Debug)]
pub enum EdgeConnectionType {
    None,
    Internal(u16),
    OffMesh(EdgeConnectionDirection),
}

#[derive(Debug)]
struct Edge {
    // The vertices that make up this edge.
    vertices: [u32; 2],
    // The index of the edge in respective polygon.
    edge_in_polygon: [usize; 2],
    // The polygons that this edge makes up.
    polygon: [usize; 2],
}

fn build_mesh_adjacency(
    polygons: &[[u32; VERTICES_PER_POLYGON]],
    vertex_count: usize,
    in_edges: &mut Vec<[EdgeConnectionType; VERTICES_PER_POLYGON]>,
) {
    let max_edge_count = polygons.len() * VERTICES_PER_POLYGON;

    let mut first_edge = vec![None; vertex_count];
    let mut next_edge = vec![None; max_edge_count];
    let mut edges = Vec::with_capacity(max_edge_count);

    for (i, indices) in polygons.iter().enumerate() {
        for (j, current) in indices.iter().enumerate() {
            let next = indices[(j + 1) % indices.len()];
            if *current < next {
                let edge = Edge {
                    vertices: [*current, next],
                    edge_in_polygon: [j, 0],
                    polygon: [i, i],
                };

                next_edge[edges.len()] = first_edge[*current as usize];
                first_edge[*current as usize] = Some(edges.len());
                edges.push(edge);
            }
        }
    }

    for (i, indices) in polygons.iter().enumerate() {
        for (j, current) in indices.iter().enumerate() {
            let next = indices[(j + 1) % indices.len()];
            if *current > next {
                let mut edge_iter = first_edge[next as usize];
                while let Some(edge_index) = edge_iter {
                    let edge = &mut edges[edge_index];
                    if edge.vertices[1] == *current && edge.polygon[0] == edge.polygon[1] {
                        edge.polygon[1] = i;
                        edge.edge_in_polygon[1] = j;
                        break;
                    }
                    edge_iter = next_edge[edge_index];
                }
            }
        }
    }

    in_edges.clear();
    in_edges.resize(
        polygons.len(),
        [EdgeConnectionType::None; VERTICES_PER_POLYGON],
    );
    for edge in edges.iter() {
        if edge.polygon[0] != edge.polygon[1] {
            let polygon_one = edge.polygon[0];
            let polygon_two = edge.polygon[1];
            in_edges[polygon_one][edge.edge_in_polygon[0]] =
                EdgeConnectionType::Internal(edge.polygon[1] as u16);
            in_edges[polygon_two][edge.edge_in_polygon[1]] =
                EdgeConnectionType::Internal(edge.polygon[0] as u16);
        }
    }
}

fn compute_vertex_hash(x: u64, z: u64) -> u64 {
    // I am not sure if this is completely necessary.
    const HASH_X: u64 = 0x8da6b343; // Multipliers from Recast's version. "Large multiplicative constants"
    const HASH_Z: u64 = 0xcb1ab31f; // "here arbitrarily chosen primes"

    let hash = x * HASH_X + z * HASH_Z;

    hash & (VERTEX_BUCKET_COUNT - 1) as u64 // Wrap it.
}

fn add_vertex(
    vertex: UVec3,
    vertices: &mut Vec<UVec3>,
    first_vertex: &mut [i32],
    next_vertex: &mut [i32],
) -> u32 {
    let bucket = compute_vertex_hash(vertex.x.into(), vertex.z.into());
    let mut i = first_vertex[bucket as usize];

    while i != -1 {
        let other_vertex = vertices[i as usize];
        if other_vertex.x == vertex.x
            && other_vertex.y.abs_diff(vertex.y) <= 1
            && other_vertex.z == vertex.z
        {
            return i as u32;
        }
        i = next_vertex[i as usize];
    }

    let i = vertices.len();
    vertices.push(vertex);
    next_vertex[i] = first_vertex[bucket as usize];
    first_vertex[bucket as usize] = i as i32;
    i as u32
}

fn triangulate(vertices: &[UVec4], indices: &mut Vec<u32>, triangles: &mut Vec<u32>) -> bool {
    for i in 0..vertices.len() {
        let next = (i + 1) % vertices.len();
        let next_next = (next + 1) % vertices.len();

        if diagonal(i, next_next, vertices, indices) {
            indices[next] |= 0x80000000;
        }
    }

    while indices.len() > 3 {
        let mut min_len = u32::MAX;
        let mut min_index = usize::MAX;

        for i in 0..indices.len() {
            let next = (i + 1) % indices.len();
            if indices[next] & 0x80000000 != 0 {
                let point = vertices[(indices[i] & 0x0fffffff) as usize];
                let point_next = vertices[(indices[next] & 0x0fffffff) as usize];

                let delta_x = point_next.x.abs_diff(point.x);
                let delta_z = point_next.z.abs_diff(point.z);

                let square_length = delta_x * delta_x + delta_z * delta_z;

                if square_length < min_len {
                    min_len = square_length;
                    min_index = i;
                }
            }
        }

        if min_index == usize::MAX {
            for i in 0..indices.len() {
                let next = (i + 1) % indices.len();
                let next_next = (next + 1) % indices.len();
                if diagonal_loose(i, next_next, vertices, indices) {
                    let point = vertices[(indices[i] & 0x0fffffff) as usize];
                    let point_next = vertices[(indices[next] & 0x0fffffff) as usize];

                    let delta_x = point_next.x.abs_diff(point.x);
                    let delta_z = point_next.z.abs_diff(point.z);

                    let square_length = delta_x * delta_x + delta_z * delta_z;

                    if square_length < min_len {
                        min_len = square_length;
                        min_index = i;
                    }
                }
            }

            if min_index == usize::MAX {
                info!("Failed to triangulate contour. Contour was probably simplified too much.");
                return false;
            }
        }

        let next = {
            let i = min_index;
            let next = (i + 1) % indices.len();
            let next_next = (next + 1) % indices.len();

            triangles.push(indices[i] & 0x0fffffff);
            triangles.push(indices[next] & 0x0fffffff);
            triangles.push(indices[next_next] & 0x0fffffff);

            indices.remove(next);

            if next >= indices.len() {
                0
            } else {
                next
            }
        };

        let i = (indices.len() + next - 1) % indices.len();
        let prev = (indices.len() + i - 1) % indices.len();
        let next_next = (next + 1) % indices.len();

        if diagonal(prev, next, vertices, indices) {
            indices[i] |= 0x80000000;
        } else {
            indices[i] &= 0x0fffffff;
        }

        if diagonal(i, next_next, vertices, indices) {
            indices[next] |= 0x80000000;
        } else {
            indices[next] &= 0x0fffffff;
        }
    }

    triangles.push(indices[0] & 0x0fffffff);
    triangles.push(indices[1] & 0x0fffffff);
    triangles.push(indices[2] & 0x0fffffff);
    indices.clear();

    true
}

fn vec_equal(a: UVec4, b: UVec4) -> bool {
    a.x == b.x && a.z == b.z
}

fn in_cone(i: usize, j: usize, vertices: &[UVec4], indices: &[u32]) -> bool {
    let point_i = vertices[(indices[i] & 0x0fffffff) as usize];
    let point_j = vertices[(indices[j] & 0x0fffffff) as usize];
    let point_i_next = vertices[(indices[(i + 1) % indices.len()] & 0x0fffffff) as usize];
    let point_i_previous =
        vertices[(indices[(indices.len() + j - 1) % indices.len()] & 0x0fffffff) as usize];

    if left_on(
        point_i_previous.as_ivec4(),
        point_i.as_ivec4(),
        point_i_next.as_ivec4(),
    ) {
        return left(
            point_i.as_ivec4(),
            point_j.as_ivec4(),
            point_i_previous.as_ivec4(),
        ) && left(
            point_j.as_ivec4(),
            point_i.as_ivec4(),
            point_i_next.as_ivec4(),
        );
    }

    !left_on(
        point_i.as_ivec4(),
        point_j.as_ivec4(),
        point_i_next.as_ivec4(),
    ) && left_on(
        point_j.as_ivec4(),
        point_i.as_ivec4(),
        point_i_previous.as_ivec4(),
    )
}

fn diagonalie(i: usize, j: usize, vertices: &[UVec4], indices: &[u32]) -> bool {
    let diagonal_one = vertices[(indices[i] & 0x0fffffff) as usize];
    let diagonal_two = vertices[(indices[j] & 0x0fffffff) as usize];

    for edge in 0..vertices.len() {
        let next_edge = (edge + 1) % vertices.len();

        if !(edge == i || next_edge == i || edge == j || next_edge == j) {
            let point_one = vertices[(indices[edge] & 0x0fffffff) as usize];
            let point_two = vertices[(indices[next_edge] & 0x0fffffff) as usize];

            if vec_equal(diagonal_one, point_one)
                || vec_equal(diagonal_two, point_one)
                || vec_equal(diagonal_one, point_two)
                || vec_equal(diagonal_two, point_two)
            {
                continue;
            }

            if intersect_prop(
                diagonal_one.as_ivec4(),
                diagonal_two.as_ivec4(),
                point_one.as_ivec4(),
                point_two.as_ivec4(),
            ) {
                return false;
            }
        }
    }

    true
}

fn diagonal(i: usize, j: usize, vertices: &[UVec4], indices: &[u32]) -> bool {
    in_cone(i, j, vertices, indices) && diagonalie(i, j, vertices, indices)
}

fn in_cone_loose(i: usize, j: usize, vertices: &[UVec4], indices: &[u32]) -> bool {
    let point_i = vertices[(indices[i] & 0x0fffffff) as usize];
    let point_j = vertices[(indices[j] & 0x0fffffff) as usize];
    let point_i_next = vertices[(indices[(i + 1) % indices.len()] & 0x0fffffff) as usize];
    let point_i_previous =
        vertices[(indices[(indices.len() + j - 1) % indices.len()] & 0x0fffffff) as usize];

    if left_on(
        point_i_previous.as_ivec4(),
        point_i.as_ivec4(),
        point_i_next.as_ivec4(),
    ) {
        // only difference between in_cone is this being left_on instead of left:
        return left_on(
            point_i.as_ivec4(),
            point_j.as_ivec4(),
            point_i_previous.as_ivec4(),
        ) && left_on(
            point_j.as_ivec4(),
            point_i.as_ivec4(),
            point_i_next.as_ivec4(),
        );
    }

    !left_on(
        point_i.as_ivec4(),
        point_j.as_ivec4(),
        point_i_next.as_ivec4(),
    ) && left_on(
        point_j.as_ivec4(),
        point_i.as_ivec4(),
        point_i_previous.as_ivec4(),
    )
}

fn diagonalie_loose(i: usize, j: usize, vertices: &[UVec4], indices: &[u32]) -> bool {
    let diagonal_one = vertices[(indices[i] & 0x0fffffff) as usize];
    let diagonal_two = vertices[(indices[j] & 0x0fffffff) as usize];

    for edge in 0..vertices.len() {
        let next_edge = (edge + 1) % vertices.len();

        if !(edge == i || next_edge == i || edge == j || next_edge == j) {
            let point_one = vertices[(indices[edge] & 0x0fffffff) as usize];
            let point_two = vertices[(indices[next_edge] & 0x0fffffff) as usize];

            if vec_equal(diagonal_one, point_one)
                || vec_equal(diagonal_two, point_one)
                || vec_equal(diagonal_one, point_two)
                || vec_equal(diagonal_two, point_two)
            {
                continue;
            }

            // loose uses prop instead of regular.
            if intersect_segment(
                diagonal_one.as_ivec4(),
                diagonal_two.as_ivec4(),
                point_one.as_ivec4(),
                point_two.as_ivec4(),
            ) {
                return false;
            }
        }
    }

    true
}

fn diagonal_loose(i: usize, j: usize, vertices: &[UVec4], indices: &[u32]) -> bool {
    in_cone_loose(i, j, vertices, indices) && diagonalie_loose(i, j, vertices, indices)
}
