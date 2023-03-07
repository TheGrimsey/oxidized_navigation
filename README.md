# Oxidized Navigation
[![Crates.io](https://img.shields.io/crates/v/oxidized_navigation)](https://crates.io/crates/oxidized_navigation/)
![Crates.io](https://img.shields.io/crates/l/oxidized_navigation)

Tiled **Runtime** Nav-mesh generation for 3D worlds in [Bevy](https://bevyengine.org/). Based on [Recast's Nav-mesh generation](https://github.com/recastnavigation/recastnavigation/) but in Rust.

Takes in [Bevy Rapier3D](https://crates.io/crates/bevy_rapier3d) colliders from entities with the ``NavMeshAffector`` component and **asynchronously** generates tiles of navigation meshes based on ``NavMeshSettings``.

## Quick-start:
**Nav-mesh generation:**
1. Insert an instance of the ``NavMeshSettings`` resource into your Bevy app.
2. Add ``OxidizedNavigationPlugin`` as a plugin.
3. Attach a ``NavMeshAffector`` component and a rapier collider to any entity you want to affect the nav-mesh.

*At this point nav-meshes will be automatically generated whenever the collider or ``GlobalTransform`` of any entity with a ``NavMeshAffector`` is changed.*

**Querying the nav-mesh / Pathfinding:**
1. Your system needs to take in the ``NavMesh`` resource.
2. Get the underlying data from the nav-mesh using ``NavMesh::get``. This data is wrapped in an ``RwLock``.
3. To access the data call ``RwLock::read``. *This will block until you get read acces on the lock. If a task is already writing to the lock it may take time.*
4. Call ``query::find_path`` with the ``NavMeshTiles`` returned from the ``RwLock``. 

*Also see the [examples](https://github.com/TheGrimsey/oxidized_navigation/tree/master/examples) for how to run pathfinding in an async task which may be preferable.*

## Supported versions

| Crate Version | Bevy Version | Bevy Rapier 3D Version |
| ------------- | ------------ | ---------------------- |
| 0.2.0         | 0.10.0       | 0.21                   |
| 0.2.0         | 0.9.X        | 0.20                   |
| 0.1.X         | 0.9.X        | 0.19                   |

## Non-exhaustive TODO-list:

- [X] Generate poly meshes for single tiles.
- [X] Handle linking tiles together.
- [X] Pathfinding across tiles.
- [X] Generate tiles asynchronously
- [X] Clean up intermediate representations when we finish processing a tile (Voxelized, Open cells, etc. Only keeping polymesh).
- [X] Quick "How to use" guide.

- [X] Mark areas too close to walls as unwalkable based on character width.
- [X] Implement areas allowing to specify differing cost of traveling.
- [ ] Allow creating nav-mesh from meshes (probably add an option to ``NavMeshAffector``).
- [ ] Rebuild all tiles when ``NavMeshSettings`` are changed. *Ideally, one would want to have the ability to change the resolution of the nav-mesh when changing level & the resolution of individual tiles.* 
- [ ] Built-in nav-mesh debug draw.
- [ ] Nav-mesh "layers" for different sized agents.

- [ ] Remove ``create_nav_mesh_tile_from_poly_mesh`` in favor of simply creating the data in the right format from the start.
- [ ] Code Tests.
- [ ] Benchmarks for tile generation & pathfinding. 
- [ ] Optimize linking tiles. (At a cost of memory we can save a lot of time linking by saving indices of polygons with OffMesh links)
- [ ] Adjust memory representation for cache usage. (Some data we only need when linking tiles and not pathfinding)

## Debug draw.

Whilst not included in the plugin currently, you can use [Bevy Prototype Debug Lines](https://crates.io/crates/bevy_prototype_debug_lines) and the ``draw_nav_mesh_system`` in the ``blocking_async`` example:
