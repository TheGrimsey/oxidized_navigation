use bevy::prelude::IntoSystemDescriptor;
use bevy::{
    ecs::system::Resource,
    prelude::{
        App, Changed, Component, CoreStage, Entity, GlobalTransform, IVec4, Plugin, Query, Res,
        ResMut, UVec2, UVec4, Vec2,
    },
    utils::{HashMap, HashSet},
};
use bevy_rapier3d::{na::Vector3, prelude::Collider, rapier::prelude::Isometry};
use smallvec::SmallVec;

use self::{
    contour::{build_contours_system, TileContours},
    heightfields::{
        construct_open_heightfields_system, create_distance_field_system,
        create_neighbour_links_system, rebuild_heightfields_system, TilesVoxelized,
    },
    mesher::{build_poly_mesh_system, TilePolyMesh},
    regions::build_regions_system,
};

mod contour;
mod heightfields;
mod mesher;
mod regions;
mod tiles;

pub struct OxidizedNavigationPlugin;
impl Plugin for OxidizedNavigationPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(TilesVoxelized::default())
            .insert_resource(TileAffectors::default())
            .insert_resource(TilesOpen::default())
            .insert_resource(TileContours::default())
            .insert_resource(DirtyTiles::default())
            .insert_resource(TilePolyMesh::default());

        app.add_system(update_navmesh_affectors_system)
            .add_system(rebuild_heightfields_system.after(update_navmesh_affectors_system))
            .add_system(construct_open_heightfields_system.after(rebuild_heightfields_system))
            .add_system(create_neighbour_links_system.after(construct_open_heightfields_system))
            .add_system(create_distance_field_system.after(create_neighbour_links_system))
            .add_system(build_regions_system.after(create_distance_field_system))
            .add_system(build_contours_system.after(build_regions_system))
            .add_system(build_poly_mesh_system.after(build_contours_system))
            .add_system_to_stage(CoreStage::PostUpdate, clear_dirty_tiles_system);
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
#[derive(Default, Clone, Copy, Debug)]
enum NeighbourConnection {
    // TODO: This might be overkill and could just be replaced by Option<u16>.
    #[default]
    Unconnected,
    Connected {
        index: u16, // Index of the span in the neighbour cell.
    },
}

// Like a HeightSpan but representing open walkable areas (empty space with floor & height >= walkable_height
#[derive(Default, Clone, Copy, Debug)]
struct OpenSpan {
    min: u16,
    max: Option<u16>,
    neighbours: [NeighbourConnection; 4],
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

#[derive(Resource)]
pub struct NavMeshSettings {
    cell_width: f32,  // Recast recommends having this be 1/2 of character radius.
    cell_height: f32, // Recast recommends having this be 1/2 of cell_width.

    tile_width: u16, // As a multiple of cell_width

    world_bound: f32, // Keep this as small as possible whilst fitting your world between -world_bound & world_bound on X & Z. This is added onto any calculation to figure out which tile we are in. Exists because without it we'd be in a big mess around 0,0.
    world_bottom_bound: f32, // Minium height of anything in the world: For example: -100.

    max_traversable_slope: f32, // In Radians.
    walkable_height: u16, // Minimum open height for a cell to be considered walkable. Size in cell_height(s).
    walkable_radius: u16, // Theoretically minimum width of an area to be considered walkable, not quite used for that yet. Size in cell_widths.
    step_height: u16, // Maximum height difference that is still considered traversable. (Think, stair steps)

    min_region_area: usize, // Minimum area of a region for it to not be removed in cells.
    merge_region_size: usize, // Maximum size of a region to merge other regions into.

    max_contour_simplification_error: f32, // Maximum difference allowed for the contour generation on the XZ-plane in cell_widths. Recast suggests keeping this in the range of [1.1, 1.5]
}
impl NavMeshSettings {
    pub fn get_tile_size(&self) -> f32 {
        self.cell_width * self.tile_width as f32
    }

    pub fn get_tile_position(&self, world_pos: Vec2) -> UVec2 {
        let tile_size = self.get_tile_size();

        let offset_world = world_pos + self.world_bound;

        (offset_world / tile_size).as_uvec2()
    }

    pub fn get_tile_bounds(&self, tile: UVec2) -> (Vec2, Vec2) {
        let tile_size = self.get_tile_size();

        let min_bound = tile.as_vec2() * tile_size - self.world_bound;
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

fn clear_dirty_tiles_system(mut dirty_tiles: ResMut<DirtyTiles>) {
    dirty_tiles.0.clear();
}

fn cell_move_back_row<'a>(
    tile: &'a OpenTile,
    nav_mesh_settings: &NavMeshSettings,
    cell_index: usize,
    target_span_index: usize,
) -> (&'a OpenSpan, usize) {
    let other_cell_index = cell_index - nav_mesh_settings.tile_width as usize;
    let other_cell = &tile.cells[other_cell_index];
    (&other_cell.spans[target_span_index], other_cell_index)
}

fn cell_move_forward_row<'a>(
    tile: &'a OpenTile,
    nav_mesh_settings: &NavMeshSettings,
    cell_index: usize,
    target_span_index: usize,
) -> (&'a OpenSpan, usize) {
    let other_cell_index = cell_index + nav_mesh_settings.tile_width as usize;
    let other_cell = &tile.cells[other_cell_index];
    (&other_cell.spans[target_span_index], other_cell_index)
}

fn cell_move_back_column(
    tile: &OpenTile,
    cell_index: usize,
    target_span_index: usize,
) -> (&OpenSpan, usize) {
    let other_cell = &tile.cells[cell_index - 1];
    (&other_cell.spans[target_span_index], cell_index - 1)
}

fn cell_move_forward_column(
    tile: &OpenTile,
    cell_index: usize,
    target_span_index: usize,
) -> (&OpenSpan, usize) {
    let other_cell = &tile.cells[cell_index + 1];
    (&other_cell.spans[target_span_index], cell_index + 1)
}

fn get_cell_offset(nav_mesh_settings: &NavMeshSettings, dir: usize) -> isize {
    match dir {
        0 => -1,
        1 => nav_mesh_settings.tile_width as isize,
        2 => 1,
        3 => -(nav_mesh_settings.tile_width as isize),
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
        return a.x <= c.x && c.x <= b.x || a.x >= c.x && c.x >= b.x;
    }

    a.z <= c.z && c.z <= b.z || a.z >= c.z && c.z >= b.z
}

fn intersect_segment(a: IVec4, b: IVec4, c: IVec4, d: IVec4) -> bool {
    intersect_prop(a, b, c, d)
        || between(a, b, c)
        || between(a, b, d)
        || between(c, d, a)
        || between(c, d, b)
}

fn area_sqr(a: IVec4, b: IVec4, c: IVec4) -> i32 {
    (b.x - a.x) * (c.z - a.z) * (c.x - a.x) * (b.z - a.z)
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
