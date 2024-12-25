
## 0.12.0 (Unreleased)
- Add `TileGenerated` event which is emitted when a nav-mesh tile has been updated.
- Added support for `Avian3d`
- Removed support for `bevy_xpbd`, use avian instead.
- Updated to Bevy 0.15

## 0.11.0 (2024-07-15)

- Added `NavMeshSettings::from_agent_and_bounds` helper for creating a nav-mesh with 'reasonable' defaults.
- Added many `NavMeshSettings::with_*` functions to adjust the 'reasonable' defaults.
- Added gizmo config group `NavigationGroup` for debug draw.
- `NavMeshSettings::max_tile_generation_tasks` is now an `Option<NonZeroU16>`
- Switch to using an EntityHashMap for storing tile relations.
- Renamed `NavMeshSettings::merge_region_area` to `NavMeshSettings::max_region_area_to_merge_into`
- `NavMeshSettings::max_edge_length` is now a `u16` instead of `u32`. An edge can't be longer than `tile_length` anyway which is a `u16`.
- Updated to Bevy 0.14
- Support multiple Parry3d versions (adopted from PR by @BaronVonScrub)

## 0.10.0 (2024-02-23)
- Update to Bevy 0.13

## 0.9.0 (2024-02-17)
- Switch to using boxed slices where applicable resulting in ~1-2% performance improvements in generation & pathfinding.
- Update to `bevy_rapier3d` 0.24 (Authored by @janhohenheim)

## 0.8.1 (2024-01-16)
- Fix only the lowest points of the nav-mesh being generated.
- Fix using the wrong area in contour generation.
- Fix broken docs build.

## 0.8.0 (2023-11-09)
- Update to Bevy 0.12
- XPBD colliders no longer need to be manually scaled.

## 0.7.0
- Added support for using XPBD & wrapped Parry3d components for Nav-Mesh generation (Authored by @Elabajaba)
- Add benchmarks.
- Optimizations in tile building.

## 0.6.0 (2023-07-11)
- Update to Bevy 0.11. (Authored by @Elabajaba)
- Integrated nav-mesh debug draw in ``OxidizedNavigationDebugDrawPlugin`` (Updated debug drawing authored by @Elabajaba)

## 0.5.2 (2023-07-08)

- Fix an oversight causing inconsistencies in voxel tile generation.

## 0.5.1 (2023-06-29)

- Minor update to README. Otherwise identical to 0.5.0

## 0.5.0 (2023-06-29)

- ``OxidizedNavigationPlugin`` now takes a settings parameter containing ``NavMeshSettings``
- ``find_path`` has been renamed ``find_polygon_path``
- A new ``find_path`` function has been added that does both polygon path finding and stringpulling in one.
- Defer much of geometry handling to async task, frame should now be blocked less.
- Added ``max_tile_generation_tasks`` to optionally limit how many tiles can be generated concurrently.

## 0.4.0 (2023-04-11)

- Nav-Mesh generation now correctly reacts to removing the ``NavMeshAffector`` component.
- ``NavMeshAffector`` is now an empty type, the data has been moved to a resource to enable the above change.

## 0.3.0 (2023-03-07)

- Updated to Bevy 0.10
- Removed ``NavMeshGenerationState``, you can replicate the same functionality by using ``configure_set`` to add a run condition to the ``OxidizedNavigation`` SystemSet.

## 0.2.0 (2023-02-13)

- Implemented ``walkable_radius``. This will "pull-back" the nav-mesh from edges, which means anywhere on the nav-mesh should be fine to stand on for a character with a radius of ``walkable_radius * cell_width``
- Added area cost multipliers.
- Added ``NavMeshAreaType`` component, allowing changing the area type of an entity from the default 0, where a value of ``None`` is unwalkable.
- Added ``NavMeshGenerationState`` allowing you to pause nav-mesh tile generation.

- Added debug draw to example.
- Disabled compiling default Bevy features.
- Refactored code to be more rust-y.
- Update to ``bevy_rapier`` 0.20

## 0.1.1 (2023-01-18)

- Fix minor bug in contour generation.

## 0.1.0 (2023-01-16)

- Initial release.