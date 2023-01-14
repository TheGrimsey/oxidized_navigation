# Oxidized Navigation

Tiled **Runtime** Nav-mesh generation for 3D worlds in [Bevy](https://bevyengine.org/). Based on [Recast's Nav-mesh generation](https://github.com/recastnavigation/recastnavigation/) but in Rust. *This is not a 1:1 port of Recast but should produce close enough results.*

Takes in [Bevy Rapier3D](https://crates.io/crates/bevy_rapier3d) colliders from entities with the ``NavMeshAffector`` component and **asynchronously** generates tiles of navigation meshes based on ``NavMeshSettings``.
## Non-exhaustive TODO-list:

- [X] Generate poly meshes for single tiles.
- [X] Handle linking tiles together.
- [X] Pathfinding across tiles.
- [X] Generate tiles asynchronously
- [X] Clean up intermediate representations when we finish processing a tile (Voxelized, Open cells, etc. Only keeping polymesh).
- [ ] Quick "How to use" guide.
- [ ] Implement areas allowing to specify differing cost of traveling.
- [ ] Mark areas too close to walls as unwalkable based on character width.
- [ ] Allow creating nav-mesh from meshes (probably add an option to ``NavMeshAffector``).
- [ ] Switch to the Bevy 0.9 plugin settings system.
- [ ] Code Tests.
- [ ] Benchmarks for tile generation & pathfinding. 
- [ ] Optimize linking tiles. (At a cost of memory we can save a lot of time finding OffMesh links by just... saving indices of the polygons with OffMesh links)
- [ ] Adjust memory representation for cache usage. (Some data we only need when linking tiles and not pathfinding)

## How to use:

1. Add a ``NavMeshSettings`` resource.
2. Add the ``OxidizedNavigationPlugin`` plugin.
3. Attach a ``NavMeshAffector`` to any entity that should affect the Nav-Mesh, the Nav-mesh will automatically update when this entity moves.

## Nav Mesh Tiles.

The world is divided into tiles of ``cell_width * tile_width`` size with 0,0 in tile coordinates being at ``(-world_half_extents, world_bound_bottom, -world_half_extents)``
