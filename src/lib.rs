use bevy::prelude::{IntoSystemDescriptor, SystemSet, SystemLabel};
use bevy::{
    ecs::system::Resource,
    prelude::{
        App, Changed, Component, Entity, GlobalTransform, IVec4, Plugin, Query, Res,
        ResMut, UVec2, UVec4, Vec2,
    },
    utils::{HashMap, HashSet},
};
use bevy_rapier3d::{na::Vector3, prelude::Collider, rapier::prelude::Isometry};
use smallvec::SmallVec;
use tiles::{create_nav_mesh_data_from_poly_mesh, NavMeshTiles};

use self::{
    contour::{build_contours_system, TileContours},
    heightfields::{
        construct_open_heightfields_system, create_distance_field_system,
        create_neighbour_links_system, rebuild_heightfields_system, TilesVoxelized,
    },
    mesher::{build_poly_mesh_system, TilePolyMesh},
    regions::build_regions_system,
};

pub mod contour;
mod heightfields;
mod mesher;
mod regions;
pub mod tiles;
pub mod query;

#[derive(SystemLabel)]
pub struct OxidizedGeneration;

pub struct OxidizedNavigationPlugin;
impl Plugin for OxidizedNavigationPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(TilesVoxelized::default())
            .insert_resource(TileAffectors::default())
            .insert_resource(TilesOpen::default())
            .insert_resource(TileContours::default())
            .insert_resource(DirtyTiles::default())
            .insert_resource(TilePolyMesh::default())
            .insert_resource(NavMeshTiles::default());

        app.add_system_set(SystemSet::new()
            .label(OxidizedGeneration)
            .with_system(update_navmesh_affectors_system)
            .with_system(rebuild_heightfields_system.after(update_navmesh_affectors_system))
            .with_system(construct_open_heightfields_system.after(rebuild_heightfields_system))
            .with_system(create_neighbour_links_system.after(construct_open_heightfields_system))
            .with_system(create_distance_field_system.after(create_neighbour_links_system))
            .with_system(build_regions_system.after(create_distance_field_system))
            .with_system(build_contours_system.after(build_regions_system))
            .with_system(build_poly_mesh_system.after(build_contours_system))

            .with_system(insert_updated_tile_system.after(build_poly_mesh_system))
            .with_system(clear_dirty_tiles_system.after(insert_updated_tile_system))
        );
    }
}

const FLAG_BORDER_VERTEX: u32 = 0x10000;
const MASK_CONTOUR_REGION: u32 = 0xffff; // Masks out the above value.

#[derive(Component, Default)]
pub struct NavMeshAffector(SmallVec<[UVec2; 4]>);

#[derive(Default, Clone, Debug)]
pub struct OpenCell {
    spans: Vec<OpenSpan>,
}

/*
*   Neighbours:
*   0: (-1, 0),
*   1: (0, 1),
*   2: (1, 0),
*   3: (0, -1)
*/

// Like a HeightSpan but representing open walkable areas (empty space with floor & height >= walkable_height
#[derive(Default, Clone, Copy, Debug)]
struct OpenSpan {
    min: u16,
    max: Option<u16>,
    neighbours: [Option<u16>; 4],
    tile_index: usize, // The index of this span in the whole tile.
    region: u16, // Region if non-zero. We could use option for this if we had some optimization for size.
}

#[derive(Default, Debug)]
pub struct OpenTile {
    cells: Vec<OpenCell>, // len = tiles_along_width^2. Laid out X to Y
    distances: Vec<u16>, // Distances used in watershed. One per span. Use tile_index to go from span to distance.
    max_distance: u16,
    span_count: usize, // Total spans in all cells.
    max_regions: u16,
}

#[derive(Default, Resource)]
struct TilesOpen {
    map: HashMap<UVec2, OpenTile>,
}

#[derive(Default, Resource)]
struct TileAffectors {
    map: HashMap<UVec2, HashSet<Entity>>,
}

#[derive(Default, Resource)]
struct DirtyTiles(HashSet<UVec2>);

#[derive(Resource, Clone)]
pub struct NavMeshSettings {
    // Suggestion: Set this to 1/2 of character radius.
    pub cell_width: f32,
    // Suggestion: Set this to 1/2 of cell_width.
    pub cell_height: f32,

    // Length of a tile size in cells.
    // Higher means more to update each time something within the tile changes, smaller means you will have more overhead from connecting the edges to other tiles & generating the tile itself.
    pub tile_width: u16, 
    
    // Set this to a value that keeps the entirety of the world you wish to cover with navmesh within it as measured from the world origin (0,0).
    // This is added onto any calculation to figure out which tile we are in. Exists because without it we'd be in a big mess around 0,0.
    pub world_half_extents: f32,
    // Minium Y position of anything in the world that should be covered by the nav mesh.
    pub world_bottom_bound: f32,
    
    // Maximum slope traversable when navigating in radians.
    pub max_traversable_slope_radians: f32,
    // Minimum open height for an area to be considered walkable in cell_height(s).
    pub walkable_height: u16,
    // Theoretically minimum width of an area to be considered walkable, not quite used for that yet, in cell_width(s). 
    pub walkable_radius: u16,
    // Maximum height difference that is still considered traversable in cell_height(s). (Think, stair steps)
    pub step_height: u16,

    // Minimum size of a region, anything smaller than this will be removed. This is used to filter out smaller regions that might appear on tables.
    pub min_region_area: usize,
    // Maximum size of a region to merge other regions into.
    pub merge_region_area: usize,

    // Maximum length of an edge before it's split. Suggestion: Start high and reduce it if there are issues.
    pub max_edge_length: u32,
    // Maximum difference allowed for simplified contour generation on the XZ-plane. Suggested value range: [1.1, 1.5]
    pub max_contour_simplification_error: f32, 
}
impl NavMeshSettings {
    #[inline]
    pub fn get_tile_size(&self) -> f32 {
        self.cell_width * self.tile_width as f32
    }

    #[inline]
    pub fn get_tile_position(&self, world_pos: Vec2) -> UVec2 {
        let tile_size = self.get_tile_size();

        let offset_world = world_pos + self.world_half_extents;

        (offset_world / tile_size).as_uvec2()
    }

    #[inline]
    pub fn get_tile_min_bound(&self, tile: UVec2) -> Vec2 {
        let tile_size = self.get_tile_size();

        tile.as_vec2() * tile_size - self.world_half_extents
    }

    #[inline]
    pub fn get_tile_bounds(&self, tile: UVec2) -> (Vec2, Vec2) {
        let tile_size = self.get_tile_size();

        let min_bound = tile.as_vec2() * tile_size - self.world_half_extents;
        let max_bound = min_bound + tile_size;

        (min_bound, max_bound)
    }
}

fn update_navmesh_affectors_system(
    nav_mesh_settings: Res<NavMeshSettings>,
    mut tile_affectors: ResMut<TileAffectors>,
    mut dirty_tiles: ResMut<DirtyTiles>,
    mut query: Query<
        (Entity, &mut NavMeshAffector, &Collider, &GlobalTransform),
        Changed<GlobalTransform>,
    >,
) {
    for (e, mut affector, collider, global_transform) in query.iter_mut() {
        let transform = global_transform.compute_transform();
        let iso = Isometry::new(
            transform.translation.into(),
            transform.rotation.to_scaled_axis().into(),
        );
        let local_aabb = collider.raw.compute_local_aabb();
        let aabb = local_aabb
            .scaled(&Vector3::new(
                transform.scale.x,
                transform.scale.y,
                transform.scale.z,
            ))
            .transform_by(&iso);

        let min_vec = Vec2::new(aabb.mins.x, aabb.mins.z);
        let min_tile = nav_mesh_settings.get_tile_position(min_vec);

        let max_vec = Vec2::new(aabb.maxs.x, aabb.maxs.z);
        let max_tile = nav_mesh_settings.get_tile_position(max_vec);

        // Remove from previous.
        for old_tile in affector.0.iter().filter(|tile_coord| {
            min_tile.x > tile_coord.x
                || min_tile.y > tile_coord.y
                || max_tile.x < tile_coord.x
                || max_tile.y < tile_coord.y
        }) {
            if let Some(affectors) = tile_affectors.map.get_mut(old_tile) {
                affectors.remove(&e);
                dirty_tiles.0.insert(*old_tile);
            }
        }
        affector.0.clear();

        for x in min_tile.x..=max_tile.x {
            for y in min_tile.y..=max_tile.y {
                let tile_coord = UVec2::new(x, y);

                if !tile_affectors.map.contains_key(&tile_coord) {
                    tile_affectors.map.insert(tile_coord, HashSet::default());
                }

                let affectors = tile_affectors.map.get_mut(&tile_coord).unwrap();
                affectors.insert(e);

                affector.0.push(tile_coord);
                dirty_tiles.0.insert(tile_coord);
            }
        }
    }
}

fn insert_updated_tile_system(
    dirty_tiles: Res<DirtyTiles>,
    poly_meshes: Res<TilePolyMesh>,
    nav_mesh_settings: Res<NavMeshSettings>,
    nav_mesh: ResMut<NavMeshTiles>,
) {
    if !dirty_tiles.0.is_empty() {
        let mut nav_mesh = nav_mesh.nav_mesh.write().unwrap();
        for tile in dirty_tiles.0.iter() {
            let Some(poly_mesh) = poly_meshes.map.get(tile) else {
                continue;
            };
            let nav_mesh_tile =
                create_nav_mesh_data_from_poly_mesh(poly_mesh, *tile, &nav_mesh_settings);
    
            nav_mesh.add_tile(*tile, nav_mesh_tile, &nav_mesh_settings);
        }
    }
}

fn clear_dirty_tiles_system(mut dirty_tiles: ResMut<DirtyTiles>) {
    dirty_tiles.0.clear();
}

fn get_cell_offset(nav_mesh_settings: &NavMeshSettings, index: usize, dir: usize) -> usize {
    match dir {
        0 => index - 1,
        1 => index + nav_mesh_settings.tile_width as usize,
        2 => index + 1,
        3 => index - nav_mesh_settings.tile_width as usize,
        _ => panic!("Not a valid direction"),
    }
}

fn intersect_prop(a: IVec4, b: IVec4, c: IVec4, d: IVec4) -> bool {
    if collinear(a, b, c) || collinear(a, b, d) || collinear(c, d, a) || collinear(c, d, b) {
        return false;
    }

    (left(a, b, c) ^ left(a, b, d)) && (left(c, d, a) ^ left(c, d, b))
}

fn between(a: IVec4, b: IVec4, c: IVec4) -> bool {
    if !collinear(a, b, c) {
        return false;
    }

    if a.x != b.x {
        return (a.x <= c.x && c.x <= b.x) || (a.x >= c.x && c.x >= b.x);
    }

    (a.z <= c.z && c.z <= b.z) || (a.z >= c.z && c.z >= b.z)
}

fn intersect(a: IVec4, b: IVec4, c: IVec4, d: IVec4) -> bool {
    intersect_prop(a, b, c, d)
        || between(a, b, c)
        || between(a, b, d)
        || between(c, d, a)
        || between(c, d, b)
}

fn area_sqr(a: IVec4, b: IVec4, c: IVec4) -> i32 {
    (b.x - a.x) * (c.z - a.z) - (c.x - a.x) * (b.z - a.z)
}

fn collinear(a: IVec4, b: IVec4, c: IVec4) -> bool {
    area_sqr(a, b, c) == 0
}

fn left(a: IVec4, b: IVec4, c: IVec4) -> bool {
    area_sqr(a, b, c) < 0
}
fn left_on(a: IVec4, b: IVec4, c: IVec4) -> bool {
    area_sqr(a, b, c) <= 0
}

fn in_cone(i: usize, outline_vertices: &[UVec4], point: UVec4) -> bool {
    let point_i = outline_vertices[i];
    let point_next = outline_vertices[(i + 1) % outline_vertices.len()];
    let point_previous =
        outline_vertices[(outline_vertices.len() + i - 1) % outline_vertices.len()];

    if left_on(point_i.as_ivec4(), point.as_ivec4(), point_next.as_ivec4()) {
        return left(
            point_i.as_ivec4(),
            point.as_ivec4(),
            point_previous.as_ivec4(),
        ) && left(point.as_ivec4(), point_i.as_ivec4(), point_next.as_ivec4());
    }

    !left_on(point_i.as_ivec4(), point.as_ivec4(), point_next.as_ivec4())
        && left_on(
            point.as_ivec4(),
            point_i.as_ivec4(),
            point_previous.as_ivec4(),
        )
}
