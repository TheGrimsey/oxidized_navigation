# Oxidized Navigation

### This is in no way production ready. You can't navigate with this.

An experimental Nav-Mesh generation plugin based on [Recast's Nav-mesh generation](https://github.com/recastnavigation/recastnavigation/) for [Bevy](https://bevyengine.org/). *This is not a 1:1 port of Recast but should produce close enough results.*

One of the main ideas is that this should allow runtime generation of tiled navigation meshes. It will gather all entities within a 2D grid square and rasterize their Rapier3D colliders into a poly mesh. 

## Non-exhaustive TODO-list:

- [X] Generate poly meshes for single tiles.
- [X] Handle linking tiles together.
- [ ] Pathfinding across tiles.
- [ ] Optimize linking tiles. (At a cost of memory we can save a lot of time finding OffMesh links by just... saving indices of the polygons with OffMesh links)
- [ ] Adjust memory representation for cache usage. (Some data we only need when linking tiles and not pathfinding)
- [ ] Clean up intermediate representations when we finish processing a tile (Voxelized, Open cells, etc. Only keeping polymesh).
- [ ] Switch to more Rust-y implementations where it makes sense (Option<>, enums, etc).
- [ ] Switch to async tasks for mesh generation instead of systems. We really just need to get the data once at the start and then it can run by itself independently of frames.
- [ ] Switch to the Bevy 0.9 plugin settings system.

## How to use:

### This is in no way production ready. You can't navigate with this.

1. Add a ``NavMeshSettings`` resource.
2. Add the ``OxidizedNavigationPlugin`` plugin.
3. Attach a ``NavMeshAffector`` to any entity that should affect the Nav-Mesh and whenever it moves it will be added/remove to the relevant tiles and those tiles will be rebuilt.

## Nav Mesh Tiles.

The world is divided into tiles of ``cell_width * tile_width`` size with 0,0 in tile coordinates being at ``(-world_bound, world_bound_bottom, -world_bound)``

## Why not just make Rust bindings for Recast?

That would probably have been a good idea. But this seemed more fun and I like not having to deal with external C++ code.