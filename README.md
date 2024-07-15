# Oxidized Navigation
[![Crates.io](https://img.shields.io/crates/v/oxidized_navigation)](https://crates.io/crates/oxidized_navigation/)
![Crates.io](https://img.shields.io/crates/l/oxidized_navigation)

Tiled **Runtime** Nav-mesh generation for 3D worlds in [Bevy](https://bevyengine.org/). Based on [Recast's Nav-mesh generation](https://github.com/recastnavigation/recastnavigation/) but in Rust.

Takes in [Parry3d](https://crates.io/crates/parry3d) colliders that implement the ``OxidizedCollider`` trait from entities with the ``NavMeshAffector`` component and **asynchronously** generates tiles of navigation meshes based on ``NavMeshSettings``. ``OxidizedCollider`` implementations for [Bevy Rapier3D](https://crates.io/crates/bevy_rapier3d) and [Bevy Xpbd_3D](https://crates.io/crates/bevy_xpbd_3d) are included under the `rapier` and `xpbd` features.

## Quick-start:
**Nav-mesh generation:**
1. Choose which backend you're going to use (bevy_rapier3d, bevy_xpbd_3d, or custom parry3d based colliders) and enable the relevant crate features ("rapier", "xpbd", or "parry_016" features).
2. If you opted for custom parry3d colliders, implement the `OxidizedCollider` trait for your collider component that wraps a `parry3d::shape::SharedShape`. This is already done for `bevy_rapier3d` and `bevy_xpbd_3d`.
3. Add ``OxidizedNavigationPlugin`` as a plugin. (eg. for xpbd `OxidizedNavigationPlugin::<Collider>::new(NavMeshSettings {...}`)
4. Attach a ``NavMeshAffector`` component and a collider that implements the `OxidizedCollider` trait (already implemented for `bevy_rapier3d` and `bevy_xpbd_3D`) to any entity you want to affect the nav-mesh.

*At this point nav-meshes will be automatically generated whenever the collider or ``GlobalTransform`` of any entity with a ``NavMeshAffector`` is changed.*

**Querying the nav-mesh / Pathfinding:**
1. Your system needs to take in the ``NavMesh`` resource.
2. Get the underlying data from the nav-mesh using ``NavMesh::get``. This data is wrapped in an ``RwLock``.
3. To access the data call ``RwLock::read``. *This will block until you get read acces on the lock. If a task is already writing to the lock it may take time.*
4. Call ``query::find_path`` with the ``NavMeshTiles`` returned from the ``RwLock``. 

*Also see the [examples](https://github.com/TheGrimsey/oxidized_navigation/tree/master/examples) for how to run pathfinding in an async task which may be preferable.*

## FAQ

> I added the `OxidizedNavigationPlugin` to my app and now it won't compile.

You need to use `OxidizedNavigationPlugin::<Collider>::new(NavMeshSettings {...}`, where `Collider` is either a rapier or xpbd `Collider`, or your own custom collider that implements the `OxidizedCollider` trait. This is necessary to allow us to be generic over different `Collider` components.

> I don't want to use the Rapier3d or XPBD3d physics engines just to generate a navmesh. How do I create my own `parry3d` wrapper component?

You need to create a component that contains a parry3d `SharedShape`, then implement the `OxidizedCollider` trait. See the [parry3d example](./examples/parry3d.rs) for a basic example.

> Can I use this with the builtin bevy shapes, or my own custom shapes?

Currently only `parry3d` colliders are supported, or crates using `parry3d` colliders. You'd have to write a function to convert your shapes/bevy shapes into `parry3d` colliders.

> Why aren't my Xpbd/Parry3d colliders scaled properly?

You need to manually apply your transform's scale to the Xpbd/Parry3d collider's shape.

> My physics crate updated and now my nav-meshes won't generate.

This is due to how dependencies are handled, Oxidized Navigation will only interact with the versions specified in [Supported Versions](#supported-versions). If you want to use other versions you can [make cargo use a different version](https://doc.rust-lang.org/cargo/reference/overriding-dependencies.html#the-patch-section).

> How do I draw the Nav-mesh for debugging?

Debug draw is available behind the ``debug_draw`` feature and using the ``OxidizedNavigationDebugDrawPlugin`` see usage in examples.

## Supported versions

| Crate Version | Bevy Version | Bevy Rapier 3D Version | Bevy Xpbd 3D Version | Parry3d Version |
| ------------- | ------------ | ---------------------- | -------------------- | --------------- |
| 0.11.0        | 0.14         | 0.27                   | 0.5                  | 0.15/0.16       |
| 0.10.0        | 0.13         | 0.25                   | 0.4                  | 0.13            |
| 0.9.0         | 0.12         | 0.24                   | 0.3                  | 0.13            |
| 0.8.0         | 0.12         | 0.23                   | 0.3                  | 0.13            |
| 0.7.0         | 0.11         | 0.22                   | 0.2                  | 0.13            |
| 0.6.0         | 0.11         | 0.22                   | unsupported          | unsupported     |
| 0.5.X         | 0.10.X       | 0.21                   | unsupported          | unsupported     |
| 0.4.0         | 0.10.X       | 0.21                   | unsupported          | unsupported     |
| 0.3.0         | 0.10.0       | 0.21                   | unsupported          | unsupported     |
| 0.2.0         | 0.9.X        | 0.20                   | unsupported          | unsupported     |
| 0.1.X         | 0.9.X        | 0.19                   | unsupported          | unsupported     |

**Using an unsupported Rapier, Xpbd, or parry3d version will cause Oxidized Navigation to fail as it tries to get the wrong version of components.**
In this case you may be able to [override which version Oxidized Navigation depends on](https://doc.rust-lang.org/cargo/reference/overriding-dependencies.html).

## Non-exhaustive TODO-list:

- [ ] Allow creating nav-mesh from meshes (probably add an option to ``NavMeshAffector``).
- [ ] Rebuild all tiles when ``NavMeshSettings`` are changed.

- [ ] Nav-mesh "layers" using different ``NavMeshSettings``.
- [ ] Pathfinding ticket system (Call to pathfinding returns a ticket that one can check later, controlling async pathfinding like this allows us to limit the amount of parallel tasks & prioritize them)
- [ ] Remove ``create_nav_mesh_tile_from_poly_mesh`` in favor of creating data in the right format from the start.

- [ ] Add local nav-mesh sub-grids that can be used for moving objects (platforms, ships, etc) without needing to regenerate it's interior every update.