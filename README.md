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

*Also see the [examples] for how to run pathfinding in an async task which may be preferable.*

## Supported versions

| Crate Version | Bevy Version | Bevy Rapier 3D Version |
| ------------- | ------------ | ---------------------- |
| 0.1.0         | 0.9.1        | 0.19                   |

*Note: These are the versions the crate is built against. It may work with newer versions.*

## Non-exhaustive TODO-list:

- [X] Generate poly meshes for single tiles.
- [X] Handle linking tiles together.
- [X] Pathfinding across tiles.
- [X] Generate tiles asynchronously
- [X] Clean up intermediate representations when we finish processing a tile (Voxelized, Open cells, etc. Only keeping polymesh).
- [X] Quick "How to use" guide.

- [ ] Built-in nav-mesh debug draw.
- [ ] Implement areas allowing to specify differing cost of traveling.
- [ ] Mark areas too close to walls as unwalkable based on character width.
- [ ] Allow creating nav-mesh from meshes (probably add an option to ``NavMeshAffector``).
- [ ] Switch to the Bevy 0.9 plugin settings system.

- [ ] Code Tests.
- [ ] Benchmarks for tile generation & pathfinding. 
- [ ] Optimize linking tiles. (At a cost of memory we can save a lot of time finding OffMesh links by just... saving indices of the polygons with OffMesh links)
- [ ] Adjust memory representation for cache usage. (Some data we only need when linking tiles and not pathfinding)

## Debug draw.

Whilst not included in the plugin currently, you can use [Bevy Prototype Debug Lines](https://crates.io/crates/bevy_prototype_debug_lines) and the following snippet in a system to draw up the nav-mesh:

```rust
fn draw_nav_mesh(
    nav_mesh_settings: Res<NavMeshSettings>,
    nav_mesh: Res<NavMesh>,
    mut lines: ResMut<DebugLines>
) {
    // Probably want to add in a trigger key here to make it not always draw.

    if let Ok(nav_mesh) = nav_mesh.get().read() {
        for (_, tile) in nav_mesh.get_tiles().iter() {
            // Draw polygons.
            for poly in tile.polygons.iter() {
                let indices = &poly.indices;
                for i in 0..indices.len() {
                    let a = tile.vertices[indices[i] as usize];
                    let b = tile.vertices[indices[(i + 1) % indices.len()] as usize];

                    lines.line(a, b, 15.0);
                }
            }

            // Draw vertex points.
            for vertex in tile.vertices.iter() {
                lines.line(*vertex, *vertex + Vec3::Y, 15.0);
            }
        }
    }
}
```
