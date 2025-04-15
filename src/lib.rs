//! Tiled **Runtime** Nav-mesh Generation for 3D worlds in [Bevy].
//!
//! Takes in colliders that implement the `OxidizedCollider` trait from entities with the [NavMeshAffector] component and **asynchronously** generates tiles of navigation meshes based on [NavMeshSettings]. Nav-meshes can then be queried using [query::find_path].
//!
//! ## Quick-start:
//! **Nav-mesh generation:**
//! 1. Choose which backend you're going to use (bevy_rapier3d, avian3d, or custom parry3d based colliders) and enable the relevant crate features ("rapier" or "avian" features, custom parry3d colliders don't require enabling any features).
//! 2. If you opted for custom parry3d colliders, implement the `OxidizedCollider` trait for your collider component that wraps a `parry3d::shape::SharedShape`. This is already done for `bevy_rapier3d` and `avian3d`.
//! 3. Add ``OxidizedNavigationPlugin`` as a plugin. (eg. for avian `OxidizedNavigationPlugin::<Collider>::new(NavMeshSettings {...}`)
//! 4. Attach a ``NavMeshAffector`` component and a collider that implements the `OxidizedCollider` trait (already implemented for `bevy_rapier3d` and `avian3d`) to any entity you want to affect the nav-mesh.
//!
//! *At this point nav-meshes will be automatically generated whenever the collider or ``GlobalTransform`` of any entity with a ``NavMeshAffector`` is changed.*
//!
//! **Querying the nav-mesh / Pathfinding:**
//! 1. Your system needs to take in the ``NavMesh`` resource.
//! 2. Get the underlying data from the nav-mesh using ``NavMesh::get``. This data is wrapped in an ``RwLock``.
//! 3. To access the data call ``RwLock::read``. *This will block until you get read acces on the lock. If a task is already writing to the lock it may take time.*
//! 4. Call ``query::find_path`` with the ``NavMeshTiles`` returned from the ``RwLock``.
//!
//! *Also see the [examples](https://github.com/TheGrimsey/oxidized_navigation/tree/master/examples) for how to run pathfinding in an async task which may be preferable.*
//!
//! ## FAQ
//!
//! > I added the `OxidizedNavigationPlugin` to my app and now it won't compile.
//!
//! You need to use `OxidizedNavigationPlugin::<Collider>::new(NavMeshSettings {...}`, where `Collider` is either a rapier or avian `Collider`, or your own custom collider that implements the `OxidizedCollider` trait. This is necessary to allow us to be generic over different `Collider` components.
//!
//! > I don't want to use the Rapier3d or XPBD3d physics engines just to generate a navmesh. How do I create my own `parry3d` wrapper component?
//!
//! You need to create a component that contains a parry3d `SharedShape`, then implement the `OxidizedCollider` trait. See the [parry3d example](./examples/parry3d.rs) for a basic example.
//!
//! > Can I use this with the builtin bevy shapes, or my own custom shapes?
//!
//! Currently only `parry3d` colliders are supported, or crates using `parry3d` colliders. You'd have to write a function to convert your shapes/bevy shapes into `parry3d` colliders.
//!
//! > Why aren't my Parry3d colliders scaled properly?
//!
//! You need to manually apply your transform's scale to the Parry3d collider's shape.
//!
//! [Bevy]: https://crates.io/crates/bevy
//! [Bevy Rapier3D]: https://crates.io/crates/bevy_rapier3d
//! [Avian]: https://crates.io/crates/avian3d
//! [Bevy Rapier3D]: https://crates.io/crates/bevy_rapier3d
//! [examples]: https://github.com/TheGrimsey/oxidized_navigation/blob/master/examples

use std::marker::PhantomData;
use std::num::{NonZeroU16, NonZeroU8};
use std::sync::{Arc, RwLock};

use bevy::ecs::entity::EntityHashMap;
use bevy::tasks::futures_lite::future;
use bevy::tasks::{AsyncComputeTaskPool, Task};
use bevy::{
    ecs::system::Resource,
    ecs::{intern::Interned, schedule::ScheduleLabel},
    prelude::*,
    utils::{HashMap, HashSet},
};
use colliders::OxidizedCollider;
use contour::build_contours;
use conversion::{
    convert_geometry_collections, ColliderType, GeometryCollection, GeometryToConvert,
};
use heightfields::{
    build_heightfield_tile, build_open_heightfield_tile, calculate_distance_field,
    erode_walkable_area, HeightFieldCollection,
};
use mesher::build_poly_mesh;
use parry3d::shape::HeightField;
use parry3d::{math::Isometry, na::Vector3, shape::TypedShape};
use regions::build_regions;
use smallvec::SmallVec;
use tiles::{create_nav_mesh_tile_from_poly_mesh, NavMeshTile, NavMeshTiles};

pub mod colliders;
mod contour;
pub mod conversion;
#[cfg(feature = "debug_draw")]
pub mod debug_draw;
mod detail_mesh;
mod heightfields;
mod math;
mod mesher;
pub mod query;
mod regions;
pub mod tiles;

/// System sets containing the crate's systems.
#[derive(SystemSet, Debug, PartialEq, Eq, Hash, Clone)]
pub enum OxidizedNavigation {
    /// Systems handling dirty marking when a NavMeshAffector component is removed.
    /// Separated to make sure that even if Main is throttled the removal events will be caught.
    RemovedComponent,
    /// Main systems, this creates the tile generation tasks & handles reacting to NavMeshAffector changes.
    Main,
}

pub struct OxidizedNavigationPlugin<ColliderComponent> {
    pub settings: NavMeshSettings,
    schedule: Interned<dyn ScheduleLabel>,
    _collider_type: PhantomData<fn() -> ColliderComponent>,
}

impl<C> OxidizedNavigationPlugin<C>
where
    C: OxidizedCollider,
{
    #[must_use]
    pub fn new(settings: NavMeshSettings) -> OxidizedNavigationPlugin<C> {
        OxidizedNavigationPlugin::<C> {
            settings,
            schedule: RunFixedMainLoop.intern(),
            _collider_type: PhantomData,
        }
    }

    /// Sets the schedule for running the plugin. Defaults to
    /// [`RunFixedMainLoop`].
    #[must_use]
    pub fn in_schedule(mut self, schedule: impl ScheduleLabel) -> Self {
        self.schedule = schedule.intern();
        self
    }
}

impl<C: OxidizedCollider> Plugin for OxidizedNavigationPlugin<C> {
    fn build(&self, app: &mut App) {
        app.insert_resource(self.settings.clone());

        app.init_resource::<TileAffectors>()
            .init_resource::<DirtyTiles>()
            .init_resource::<NavMesh>()
            .init_resource::<GenerationTicker>()
            .init_resource::<NavMeshAffectorRelations>()
            .init_resource::<ActiveGenerationTasks>();

        app.configure_sets(
            self.schedule,
            (
                OxidizedNavigation::RemovedComponent,
                OxidizedNavigation::Main,
            )
                .chain()
                // Configure our systems to run before physics engines.
                .in_set(RunFixedMainLoopSystem::BeforeFixedMainLoop),
        );

        app.add_systems(
            self.schedule,
            handle_removed_affectors_system
                .run_if(any_component_removed::<NavMeshAffector>)
                .in_set(OxidizedNavigation::RemovedComponent),
        );

        app.add_systems(
            self.schedule,
            (
                (remove_finished_tasks, update_navmesh_affectors_system::<C>),
                send_tile_rebuild_tasks_system::<C>.run_if(can_generate_new_tiles),
            )
                .chain()
                .in_set(OxidizedNavigation::Main),
        );

        app.register_type::<NavMeshAffector>()
            .register_type::<NavMeshAreaType>();

        app.add_event::<TileGenerated>();
    }
}

const FLAG_BORDER_VERTEX: u32 = 0x10000;
const MASK_CONTOUR_REGION: u32 = 0xffff; // Masks out the above value.

#[derive(Resource, Default)]
struct NavMeshAffectorRelations(EntityHashMap<SmallVec<[UVec2; 4]>>);

#[derive(Resource, Default)]
pub struct ActiveGenerationTasks(Vec<Task<Option<UVec2>>>);
impl ActiveGenerationTasks {
    pub fn len(&self) -> usize {
        self.0.len()
    }
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

/// Component for entities that should affect the nav-mesh.
#[derive(Component, Reflect, Default)]
#[reflect(Component)]
pub struct NavMeshAffector;

/// Optional component to define the area type of an entity. Setting this to ``None`` means that the entity isn't walkable.
///
/// Any part of the nav-mesh generated from this entity will have this area type. Overlapping areas will prefer the higher area type.
#[derive(Component, Reflect, Default)]
#[reflect(Component)]
pub struct NavMeshAreaType(pub Option<Area>);

#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Reflect)]
pub struct Area(pub u16);

/*
*   Neighbours:
*   0: (-1, 0),
*   1: (0, 1),
*   2: (1, 0),
*   3: (0, -1)
*/

/// Generation ticker for tiles.
///
/// Used to keep track of if the existing tile is newer than the one we are trying to insert in [build_tile]. This could happen if we go from having a lot of triangles to very few.
#[derive(Default, Resource)]
struct GenerationTicker(u64);

#[derive(Default, Resource, Deref, DerefMut)]
struct TileAffectors(HashMap<UVec2, HashSet<Entity>>);

/// Set of all tiles that need to be rebuilt.
#[derive(Default, Resource)]
struct DirtyTiles(HashSet<UVec2>);

/// Settings for generating height-corrected detail meshes.
#[derive(Clone)]
pub struct DetailMeshSettings {
    /// The maximum acceptible error in height between the nav-mesh polygons & the true world (in cells).
    pub max_height_error: NonZeroU16,
    /// Determines how often (in cells) to sample the height when generating the height-corrected nav-mesh.
    ///
    /// This greatly affects generation performance. Higher values reduce samples by half to the previous one.
    /// Ex. 1.0, 0.5, 0.25, 0.125.
    ///
    /// **Suggested value:** >=2. Start high & reduce as needed.  
    pub sample_step: NonZeroU8,
}

/// Settings for nav-mesh generation.
#[derive(Resource, Clone)]
pub struct NavMeshSettings {
    /// The horizontal resolution of the voxelized tile.
    ///
    /// **Suggested value**: 1/2 of character radius.
    ///
    /// Smaller values will increase tile generation times with diminishing returns in nav-mesh detail.
    pub cell_width: f32,
    /// The vertical resolution of the voxelized tile.
    ///
    /// **Suggested value**: 1/2 of cell_width.
    ///
    /// Smaller values will increase tile generation times with diminishing returns in nav-mesh detail.
    pub cell_height: f32,

    /// Length of a tile's side in cells. Resulting size in world units is ``tile_width * cell_width``.
    ///
    /// **Suggested value**: ???
    ///
    /// Higher means more to update each time something within the tile changes, smaller means you will have more overhead from connecting the edges to other tiles & generating the tile itself.
    pub tile_width: NonZeroU16,

    /// Extents of the world as measured from the world origin (0.0, 0.0) on the XZ-plane.
    ///
    /// **Suggested value**: As small as possible whilst still keeping the entire world within it.
    ///
    /// This exists because figuring out which tile we are in around the world origin would not work without it.
    pub world_half_extents: f32,
    /// Bottom extents of the world on the Y-axis. The top extents is capped by ``world_bottom_bound + cell_height * u16::MAX``.
    ///
    /// **Suggested value**: Minium Y position of anything in the world that should be covered by the nav mesh.
    pub world_bottom_bound: f32,

    /// Maximum incline/slope traversable when navigating in radians.
    pub max_traversable_slope_radians: f32,
    /// Minimum open height for an area to be considered walkable in cell_height(s).
    ///
    /// **Suggested value**: The height of character * ``cell_height``, rounded up.
    pub walkable_height: u16,
    /// This will "pull-back" the nav-mesh from edges, meaning anywhere on the nav-mesh will be walkable for a character with a radius of ``walkable_radius * cell_width``.
    ///
    /// **Suggested value**: ``ceil(character_radius / cell_width)`` (2-3 if `cell_width` is 1/2 of ``character_radius``)  
    pub walkable_radius: u16,
    /// Maximum height difference that is still considered traversable in cell_height(s). This smooths out stair steps and small ledges.
    pub step_height: u16,

    /// Minimum size of a region in cells, anything smaller than this will be removed. This is used to filter out smaller disconnected island that may appear on surfaces like tables.
    pub min_region_area: u32,
    /// Maximum size of a region in cells we can merge other regions into.
    pub max_region_area_to_merge_into: u32,

    /// Maximum length of an edge before it's split.
    ///
    /// **Suggested value**: Start high and reduce if there are issues.
    pub max_edge_length: u16,
    /// Maximum difference allowed for simplified contour generation on the XZ-plane in cell_width(s).
    ///
    /// **Suggested value range**: `[1.1, 1.5]`
    pub max_contour_simplification_error: f32,

    /// Max tiles to generate in parallel at once. A value of ``None`` will result in no limit.
    ///
    /// Adjust this to control memory & CPU usage. More tiles generating at once will have a higher memory footprint.
    pub max_tile_generation_tasks: Option<NonZeroU16>,

    /// When not None, height correct nav-mesh polygons where the surface height differs too much from the surface in cells. This is very useful for bumpy terrain.
    ///
    /// Helps on bumpy shapes like terrain but comes at a performance cost.
    /// **Experimental**: This may have issues at the edges of regions.
    pub experimental_detail_mesh_generation: Option<DetailMeshSettings>,
}
impl NavMeshSettings {
    /// Helper function for creating nav-mesh settings with reasonable defaults from the size of your navigation agent and bounds of your world.
    ///
    ///
    #[inline]
    pub fn from_agent_and_bounds(
        agent_radius: f32,
        agent_height: f32,
        world_half_extents: f32,
        world_bottom_bound: f32,
    ) -> Self {
        let cell_width = agent_radius / 2.0;
        let cell_height = agent_radius / 4.0;

        let walkable_height = (agent_height / cell_height) as u16;

        Self {
            cell_width,
            cell_height,
            tile_width: NonZeroU16::new(120).unwrap(),
            world_half_extents: world_half_extents.abs(),
            world_bottom_bound,
            max_traversable_slope_radians: 50.0_f32.to_radians(),
            walkable_height,
            walkable_radius: 2,
            step_height: 3,
            min_region_area: 100,
            max_region_area_to_merge_into: 500,
            max_edge_length: 80,
            max_contour_simplification_error: 1.1,
            max_tile_generation_tasks: NonZeroU16::new(8),
            experimental_detail_mesh_generation: None,
        }
    }
    /// Setter for [`NavMeshSettings::walkable_radius`]
    pub fn with_walkable_radius(mut self, walkable_radius: u16) -> Self {
        self.walkable_radius = walkable_radius;

        self
    }
    /// Setter for [`NavMeshSettings::tile_width`]
    pub fn with_tile_width(mut self, tile_width: NonZeroU16) -> Self {
        self.tile_width = tile_width;

        self
    }
    /// Setter for [`NavMeshSettings::max_traversable_slope_radians`]
    pub fn with_traversible_slope(mut self, traversible_slope: f32) -> Self {
        self.max_traversable_slope_radians = traversible_slope;

        self
    }
    /// Setter for [`NavMeshSettings::max_tile_generation_tasks`]
    pub fn with_max_tile_generation_tasks(
        mut self,
        max_tile_generation_tasks: Option<NonZeroU16>,
    ) -> Self {
        self.max_tile_generation_tasks = max_tile_generation_tasks;

        self
    }
    /// Setter for [`NavMeshSettings::step_height`]
    pub fn with_step_height(mut self, step_height: u16) -> Self {
        self.step_height = step_height;

        self
    }
    /// Setter for [`NavMeshSettings::min_region_area`] & [`NavMeshSettings::max_region_area_to_merge_into`]
    pub fn with_region_area(
        mut self,
        min_region_area: u32,
        max_region_area_to_merge_into: u32,
    ) -> Self {
        self.min_region_area = min_region_area;
        self.max_region_area_to_merge_into = max_region_area_to_merge_into;

        self
    }
    /// Setter for [`NavMeshSettings::max_contour_simplification_error`]
    pub fn with_max_contour_simplification_error(
        mut self,
        max_contour_simplification_error: f32,
    ) -> Self {
        self.max_contour_simplification_error = max_contour_simplification_error;

        self
    }
    /// Setter for [`NavMeshSettings::max_edge_length`]
    pub fn with_max_edge_length(mut self, max_edge_length: u16) -> Self {
        self.max_edge_length = max_edge_length;

        self
    }

    /// Setter for [`NavMeshSettings::experimental_detail_mesh_generation`]
    ///
    /// **Experimental**: This may have issues at the edges of regions.
    pub fn with_experimental_detail_mesh_generation(
        mut self,
        detail_mesh_generation_settings: DetailMeshSettings,
    ) -> Self {
        self.experimental_detail_mesh_generation = Some(detail_mesh_generation_settings);

        self
    }

    /// Returns the length of a tile's side in world units.
    #[inline]
    pub fn get_tile_size(&self) -> f32 {
        self.cell_width * f32::from(self.tile_width.get())
    }
    #[inline]
    pub fn get_border_size(&self) -> f32 {
        f32::from(self.walkable_radius) * self.cell_width
    }

    /// Returns the tile coordinate that contains the supplied ``world_position``.
    #[inline]
    pub fn get_tile_containing_position(&self, world_position: Vec2) -> UVec2 {
        let offset_world = world_position + self.world_half_extents;

        (offset_world / self.get_tile_size()).as_uvec2()
    }

    /// Returns the minimum bound of a tile on the XZ-plane.
    #[inline]
    pub fn get_tile_origin(&self, tile: UVec2) -> Vec2 {
        tile.as_vec2() * self.get_tile_size() - self.world_half_extents
    }

    /// Returns the origin of a tile on the XZ-plane including the border area.
    #[inline]
    pub fn get_tile_origin_with_border(&self, tile: UVec2) -> Vec2 {
        self.get_tile_origin(tile) - self.get_border_size()
    }

    #[inline]
    pub fn get_tile_side_with_border(&self) -> usize {
        usize::from(self.tile_width.get()) + usize::from(self.walkable_radius) * 2
    }
    #[inline]
    pub fn get_border_side(&self) -> usize {
        // Not technically useful currently but in case.
        self.walkable_radius.into()
    }

    /// Returns the minimum & maximum bound of a tile on the XZ-plane.
    #[inline]
    pub fn get_tile_bounds(&self, tile: UVec2) -> (Vec2, Vec2) {
        let tile_size = self.get_tile_size();

        let min_bound = tile.as_vec2() * tile_size - self.world_half_extents;
        let max_bound = min_bound + tile_size;

        (min_bound, max_bound)
    }
}

/// Wrapper around the nav-mesh data.
///
/// The underlying [NavMeshTiles] must be retrieved using [NavMesh::get]
#[derive(Default, Resource)]
pub struct NavMesh(Arc<RwLock<NavMeshTiles>>);

impl NavMesh {
    pub fn get(&self) -> Arc<RwLock<NavMeshTiles>> {
        self.0.clone()
    }
}

type NavmeshAffectorChangedQueryFilter<C> = (
    Or<(
        Changed<GlobalTransform>,
        Changed<C>,
        Changed<NavMeshAffector>,
    )>,
    With<NavMeshAffector>,
);

#[expect(clippy::type_complexity)]
fn update_navmesh_affectors_system<C: OxidizedCollider>(
    nav_mesh_settings: Res<NavMeshSettings>,
    mut tile_affectors: ResMut<TileAffectors>,
    mut affector_relations: ResMut<NavMeshAffectorRelations>,
    mut dirty_tiles: ResMut<DirtyTiles>,
    query: Query<
        (Entity, &C::Component, &GlobalTransform),
        NavmeshAffectorChangedQueryFilter<C::Component>,
    >,
) {
    // Expand by 2 * walkable_radius to match with erode_walkable_area.
    let border_expansion =
        f32::from(nav_mesh_settings.walkable_radius * 2) * nav_mesh_settings.cell_width;

    query.iter().for_each(|(e, collider, global_transform)| {
        let transform = global_transform.compute_transform();
        let iso = Isometry::new(
            transform.translation.into(),
            transform.rotation.to_scaled_axis().into(),
        );
        let local_aabb = C::oxidized_compute_local_aabb(collider);
        let aabb = local_aabb
            .scaled(&Vector3::new(
                transform.scale.x,
                transform.scale.y,
                transform.scale.z,
            ))
            .transform_by(&iso);

        let min_vec = Vec2::new(
            aabb.mins.x - border_expansion,
            aabb.mins.z - border_expansion,
        );
        let min_tile = nav_mesh_settings.get_tile_containing_position(min_vec);

        let max_vec = Vec2::new(
            aabb.maxs.x + border_expansion,
            aabb.maxs.z + border_expansion,
        );
        let max_tile = nav_mesh_settings.get_tile_containing_position(max_vec);

        let relation = if let Some(relation) = affector_relations.0.get_mut(&e) {
            // Remove from previous.
            for old_tile in relation.iter().filter(|tile_coord| {
                min_tile.x > tile_coord.x
                    || min_tile.y > tile_coord.y
                    || max_tile.x < tile_coord.x
                    || max_tile.y < tile_coord.y
            }) {
                if let Some(affectors) = tile_affectors.get_mut(old_tile) {
                    affectors.remove(&e);
                    dirty_tiles.0.insert(*old_tile);
                }
            }
            relation.clear();

            relation
        } else {
            affector_relations
                .0
                .insert_unique_unchecked(e, SmallVec::default())
                .1
        };

        for x in min_tile.x..=max_tile.x {
            for y in min_tile.y..=max_tile.y {
                let tile_coord = UVec2::new(x, y);

                let affectors = if let Some(affectors) = tile_affectors.get_mut(&tile_coord) {
                    affectors
                } else {
                    tile_affectors
                        .insert_unique_unchecked(tile_coord, HashSet::default())
                        .1
                };
                affectors.insert(e);

                relation.push(tile_coord);
                dirty_tiles.0.insert(tile_coord);
            }
        }
    });
}

fn handle_removed_affectors_system(
    mut removed_affectors: RemovedComponents<NavMeshAffector>,
    mut affector_relations: ResMut<NavMeshAffectorRelations>,
    mut dirty_tiles: ResMut<DirtyTiles>,
) {
    for relations in removed_affectors
        .read()
        .filter_map(|removed| affector_relations.0.remove(&removed))
    {
        for tile in relations {
            dirty_tiles.0.insert(tile);
        }
    }
}

fn can_generate_new_tiles(
    active_generation_tasks: Res<ActiveGenerationTasks>,
    dirty_tiles: Res<DirtyTiles>,
    nav_mesh_settings: Res<NavMeshSettings>,
) -> bool {
    nav_mesh_settings
        .max_tile_generation_tasks
        .is_none_or(|max_tile_generation_tasks| {
            active_generation_tasks.0.len() < max_tile_generation_tasks.get().into()
        })
        && !dirty_tiles.0.is_empty()
}

#[allow(clippy::too_many_arguments)]
#[expect(clippy::type_complexity)]
fn send_tile_rebuild_tasks_system<C: OxidizedCollider>(
    mut active_generation_tasks: ResMut<ActiveGenerationTasks>,
    mut generation_ticker: ResMut<GenerationTicker>,
    mut dirty_tiles: ResMut<DirtyTiles>,
    mut tiles_to_generate: Local<Vec<UVec2>>,
    mut heightfields: Local<EntityHashMap<Arc<HeightFieldCollection>>>,
    nav_mesh_settings: Res<NavMeshSettings>,
    nav_mesh: Res<NavMesh>,
    tile_affectors: Res<TileAffectors>,
    collider_query: Query<
        (
            Entity,
            &C::Component,
            &GlobalTransform,
            Option<&NavMeshAreaType>,
        ),
        With<NavMeshAffector>,
    >,
) {
    let thread_pool = AsyncComputeTaskPool::get();

    let max_task_count = (nav_mesh_settings
        .max_tile_generation_tasks
        .unwrap_or(NonZeroU16::MAX)
        .get() as usize)
        .saturating_sub(active_generation_tasks.0.len());
    tiles_to_generate.extend(dirty_tiles.0.iter().take(max_task_count));

    for tile_coord in tiles_to_generate.drain(..) {
        dirty_tiles.0.remove(&tile_coord);

        generation_ticker.0 += 1;

        let Some(affectors) = tile_affectors.get(&tile_coord) else {
            // Spawn task to remove tile.
            thread_pool
                .spawn(remove_tile(
                    generation_ticker.0,
                    tile_coord,
                    nav_mesh.0.clone(),
                ))
                .detach();
            continue;
        };
        if affectors.is_empty() {
            // Spawn task to remove tile.
            thread_pool
                .spawn(remove_tile(
                    generation_ticker.0,
                    tile_coord,
                    nav_mesh.0.clone(),
                ))
                .detach();
            continue;
        }

        // Step 1: Gather data.
        let mut geometry_collections = Vec::with_capacity(affectors.len());
        // Storing heightfields separately because they are massive.
        let mut heightfield_collections = Vec::new();

        let mut collider_iter = collider_query.iter_many(affectors.iter());
        while let Some((entity, collider, global_transform, nav_mesh_affector)) =
            collider_iter.fetch_next()
        {
            let area = nav_mesh_affector.map_or(Some(Area(0)), |area_type| area_type.0);

            let geometry_result = get_geometry_type(C::oxidized_into_typed_shape(collider));
            let transform = global_transform.compute_transform();
            handle_geometry_result(
                geometry_result,
                entity,
                transform,
                area,
                &mut geometry_collections,
                &mut heightfield_collections,
                &mut heightfields,
            );
        }

        // Step 2: Acquire nav_mesh lock
        let nav_mesh = nav_mesh.0.clone();

        // Step 3: Make it a task.
        let task = thread_pool.spawn(build_tile(
            generation_ticker.0,
            tile_coord,
            nav_mesh_settings.clone(),
            geometry_collections,
            heightfield_collections.into_boxed_slice(),
            nav_mesh,
        ));

        active_generation_tasks.0.push(task);
    }
    heightfields.clear();
}

fn handle_geometry_result(
    type_to_convert: GeometryResult,
    entity: Entity,
    global_transform: Transform,
    area: Option<Area>,
    geometry_collections: &mut Vec<GeometryCollection>,
    heightfield_collections: &mut Vec<Arc<HeightFieldCollection>>,
    heightfields: &mut EntityHashMap<Arc<HeightFieldCollection>>,
) {
    match type_to_convert {
        GeometryResult::GeometryToConvert(geometry_to_convert) => {
            geometry_collections.push(GeometryCollection {
                transform: global_transform,
                geometry_to_convert,
                area,
            });
        }
        GeometryResult::Heightfield(heightfield) => {
            // Deduplicate heightfields.
            let heightfield = if let Some(heightfield) = heightfields.get(&entity) {
                heightfield.clone()
            } else {
                let heightfield = Arc::new(HeightFieldCollection {
                    transform: global_transform,
                    heightfield: heightfield.clone(),
                    area,
                });

                heightfields.insert(entity, heightfield.clone());

                heightfield
            };

            heightfield_collections.push(heightfield);
        }
        GeometryResult::Compound(results) => {
            for (isometry, result) in results {
                let translation = Vec3::from(isometry.translation);
                let rotation = Quat::from(isometry.rotation);
                let mut transform = global_transform;
                transform.translation += translation;
                transform.rotation *= rotation;
                handle_geometry_result(
                    result,
                    entity,
                    transform,
                    area,
                    geometry_collections,
                    heightfield_collections,
                    heightfields,
                );
            }
        }
        GeometryResult::Unsupported => {}
    }
}

enum GeometryResult<'a> {
    Compound(Vec<(Isometry<f32>, GeometryResult<'a>)>),
    GeometryToConvert(GeometryToConvert),
    Heightfield(&'a HeightField),
    Unsupported,
}

impl From<GeometryToConvert> for GeometryResult<'_> {
    fn from(value: GeometryToConvert) -> Self {
        GeometryResult::GeometryToConvert(value)
    }
}

fn get_geometry_type(collider: TypedShape) -> GeometryResult {
    match collider {
        TypedShape::Ball(ball) => GeometryToConvert::Collider(ColliderType::Ball(*ball)).into(),
        TypedShape::Cuboid(cuboid) => {
            GeometryToConvert::Collider(ColliderType::Cuboid(*cuboid)).into()
        }
        TypedShape::Capsule(capsule) => {
            GeometryToConvert::Collider(ColliderType::Capsule(*capsule)).into()
        }
        TypedShape::TriMesh(trimesh) => GeometryToConvert::ParryTriMesh(
            // Can't turn these into boxed slices instantly because they are references.. So we need a copy.
            trimesh.vertices().to_vec().into_boxed_slice(),
            trimesh.indices().to_vec().into_boxed_slice(),
        )
        .into(),
        TypedShape::HeightField(heightfield) => GeometryResult::Heightfield(heightfield),
        TypedShape::ConvexPolyhedron(polyhedron) => {
            let tri = polyhedron.to_trimesh();

            GeometryToConvert::ParryTriMesh(tri.0.into_boxed_slice(), tri.1.into_boxed_slice())
                .into()
        }
        TypedShape::Cylinder(cylinder) => {
            GeometryToConvert::Collider(ColliderType::Cylinder(*cylinder)).into()
        }
        TypedShape::Cone(cone) => GeometryToConvert::Collider(ColliderType::Cone(*cone)).into(),
        TypedShape::RoundCuboid(round_cuboid) => {
            GeometryToConvert::Collider(ColliderType::Cuboid(round_cuboid.inner_shape)).into()
        }
        TypedShape::RoundCylinder(round_cylinder) => {
            GeometryToConvert::Collider(ColliderType::Cylinder(round_cylinder.inner_shape)).into()
        }
        TypedShape::RoundCone(round_cone) => {
            GeometryToConvert::Collider(ColliderType::Cone(round_cone.inner_shape)).into()
        }
        TypedShape::RoundConvexPolyhedron(round_polyhedron) => {
            let tri = round_polyhedron.inner_shape.to_trimesh();

            GeometryToConvert::ParryTriMesh(tri.0.into_boxed_slice(), tri.1.into_boxed_slice())
                .into()
        }
        TypedShape::Triangle(triangle) => {
            GeometryToConvert::Collider(ColliderType::Triangle(*triangle)).into()
        }
        TypedShape::RoundTriangle(triangle) => {
            GeometryToConvert::Collider(ColliderType::Triangle(triangle.inner_shape)).into()
        }
        TypedShape::Compound(colliders) => {
            let results = colliders
                .shapes()
                .iter()
                .map(|(isometry, shape)| (*isometry, get_geometry_type(shape.0.as_typed_shape())))
                .collect();

            GeometryResult::Compound(results)
        }
        // These ones do not make sense in this.
        TypedShape::HalfSpace(_) => GeometryResult::Unsupported, /* This is like an infinite plane? We don't care. */
        TypedShape::Polyline(_) => GeometryResult::Unsupported,  /* This is a line. */
        TypedShape::Segment(_) => GeometryResult::Unsupported,   /* This is a line segment. */
        TypedShape::Custom(_) => {
            warn!(
                "Custom shapes are not yet supported for nav-mesh generation, skipping for now.."
            );
            GeometryResult::Unsupported
        }
    }
}

/// Event containing the tile coordinate of a generated/regenerated tile.
///
/// Emitted when a tile has been updated.
#[derive(Event)]
pub struct TileGenerated(pub UVec2);

fn remove_finished_tasks(
    mut active_generation_tasks: ResMut<ActiveGenerationTasks>,
    mut event: EventWriter<TileGenerated>,
) {
    active_generation_tasks.0.retain_mut(|task| {
        if let Some(tile) = future::block_on(future::poll_once(task)) {
            if let Some(tile) = tile {
                event.send(TileGenerated(tile));
            }

            false
        } else {
            true
        }
    });
}

async fn remove_tile(
    generation: u64, // This is the max generation we remove. Should we somehow strangely be executing this after a new tile has arrived we won't remove it.
    tile_coord: UVec2,
    nav_mesh: Arc<RwLock<NavMeshTiles>>,
) {
    let Ok(mut nav_mesh) = nav_mesh.write() else {
        error!("Nav-Mesh lock has been poisoned. Generation can no longer be continued.");
        return;
    };

    if nav_mesh.tile_generations.get(&tile_coord).unwrap_or(&0) < &generation {
        nav_mesh.tile_generations.insert(tile_coord, generation);
        nav_mesh.remove_tile(tile_coord);
    }
}
async fn build_tile(
    generation: u64,
    tile_coord: UVec2,
    nav_mesh_settings: NavMeshSettings,
    geometry_collections: Vec<GeometryCollection>,
    heightfields: Box<[Arc<HeightFieldCollection>]>,
    nav_mesh: Arc<RwLock<NavMeshTiles>>,
) -> Option<UVec2> {
    #[cfg(feature = "trace")]
    let _span = info_span!("Async build Tile").entered();

    let nav_mesh_tile = build_tile_sync(
        geometry_collections,
        tile_coord,
        heightfields,
        &nav_mesh_settings,
    );

    let Ok(mut nav_mesh) = nav_mesh.write() else {
        error!("Nav-Mesh lock has been poisoned. Generation can no longer be continued.");
        return None;
    };

    if nav_mesh.tile_generations.get(&tile_coord).unwrap_or(&0) < &generation {
        nav_mesh.tile_generations.insert(tile_coord, generation);

        nav_mesh.add_tile(tile_coord, nav_mesh_tile, &nav_mesh_settings);

        Some(tile_coord)
    } else {
        None
    }
}

pub fn build_tile_sync(
    geometry_collections: Vec<GeometryCollection>,
    tile_coord: UVec2,
    heightfields: Box<[Arc<HeightFieldCollection>]>,
    nav_mesh_settings: &NavMeshSettings,
) -> NavMeshTile {
    let triangle_collection = {
        #[cfg(feature = "trace")]
        let _span = info_span!("Convert Geometry Collections").entered();
        convert_geometry_collections(geometry_collections)
    };

    let voxelized_tile = {
        #[cfg(feature = "trace")]
        let _span = info_span!("Build Heightfield Tile").entered();
        build_heightfield_tile(
            tile_coord,
            &triangle_collection,
            &heightfields,
            nav_mesh_settings,
        )
    };

    let mut open_tile = {
        #[cfg(feature = "trace")]
        let _span = info_span!("Build Open Heightfield Tile").entered();
        build_open_heightfield_tile(voxelized_tile, nav_mesh_settings)
    };

    // Remove areas that are too close to a wall.
    {
        #[cfg(feature = "trace")]
        let _span = info_span!("Erode walkable area").entered();
        erode_walkable_area(&mut open_tile, nav_mesh_settings);
    }

    {
        #[cfg(feature = "trace")]
        let _span = info_span!("Calculate distance field").entered();
        calculate_distance_field(&mut open_tile, nav_mesh_settings);
    }
    {
        #[cfg(feature = "trace")]
        let _span = info_span!("Build regions").entered();
        build_regions(&mut open_tile, nav_mesh_settings);
    }

    let contour_set = {
        #[cfg(feature = "trace")]
        let _span = info_span!("Build contours").entered();
        build_contours(&open_tile, nav_mesh_settings)
    };

    let poly_mesh = {
        #[cfg(feature = "trace")]
        let _span = info_span!("Build poly mesh").entered();
        build_poly_mesh(contour_set, nav_mesh_settings, &open_tile)
    };

    {
        #[cfg(feature = "trace")]
        let _span = info_span!("Create nav-mesh tile from poly mesh").entered();

        create_nav_mesh_tile_from_poly_mesh(poly_mesh, tile_coord, nav_mesh_settings)
    }
}

fn get_neighbour_index(tile_size: usize, index: usize, dir: usize) -> usize {
    match dir {
        0 => index - 1,
        1 => index + tile_size,
        2 => index + 1,
        3 => index - tile_size,
        _ => panic!("Not a valid direction"),
    }
}
