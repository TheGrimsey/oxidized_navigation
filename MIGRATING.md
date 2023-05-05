## 0.5

## ``OxidizedNavigationPlugin`` now takes a settings parameter containing ``NavMeshSettings``

You no longer have to remember to separately insert the ``NavMeshSettings`` resource.

```rust
// 0.4
app.insert_resource(NavMeshSettings {
    // etc...
});
app.add_plugin(OxidizedNavigationPlugin);

// 0.5
app.add_plugin(OxidizedNavigationPlugin {
    settings: NavMeshSettings {
        // etc..
    }
});
```


## ``find_path`` now performs string pulling as well as path finding.

``find_polygon_path`` acts as ``find_path`` did previously if you only want the polygon path.

```rust
// 0.4
match find_path(
        &nav_mesh,
        &nav_mesh_settings,
        start_pos,
        end_pos,
        position_search_radius,
        area_cost_multiplier,
    ) {
        Ok(path) => {
            // Convert polygon path to a path of Vec3s.
            match perform_string_pulling_on_path(&nav_mesh, start_pos, end_pos, &path) {
                Ok(string_path) => {
                    return Some(string_path);
                }
                Err(error) => // Handle error
            };
        }
        Err(error) => // Handle error
    }
// 0.5
match find_path(
        &nav_mesh,
        &nav_mesh_settings,
        start_pos,
        end_pos,
        position_search_radius,
        area_cost_multiplier,
    ) {
        Ok(string_path) => {
            return Some(string_path);
        }
        Err(error) => // Handle error
    }
```


## 0.4

## ``OxidizedNavigation`` system set is now an enum.

To continue with previous behaviour you should configure the ``OxidizedNavigation::Main`` set. The ``RemovedComponent`` set should **not** be throttled as it reacts to removing navmesh affectors. Throttling it may cause it to miss affectors being removed.

## ``NavMeshAffector`` component is now an empty type.

Remove ``::default()`` from when inserting the component.

## 0.1 to 0.2

### ``OxidizedNavigationPlugin`` now takes a ``NavMeshGenerationState``.
This allows determining the starting state of generation, allowing pausing it until the world has loaded in fully.

```rust
// 0.1
app.add_plugin(OxidizedNavigationPlugin);

// 0.2
app.add_plugin(OxidizedNavigationPlugin {
    starting_state: NavMeshGenerationState::Running,
});
// Or alternatively it will default to Running
app.add_plugin(OxidizedNavigationPlugin::default());
```

### ``find_path`` now takes an optional slice of f32s
These determine the cost multiplier for crossing an area.

```rust
// 0.1
find_path(
        &nav_mesh,
        &nav_mesh_settings,
        start_pos,
        end_pos,
        position_search_radius,
);

// 0.2
// Replicating 0.1 behaviour
find_path(
        &nav_mesh,
        &nav_mesh_settings,
        start_pos,
        end_pos,
        position_search_radius,
        None // All areas will have a cost multiplier of 1.0.
);

// Or with different costs.
find_path(
        &nav_mesh,
        &nav_mesh_settings,
        start_pos,
        end_pos,
        position_search_radius,
        Some(&[1.0, 0.5]), // Areas of type 0 have a cost multiplier of 1.0. Whilst areas of type 1 have a cost of 0.5. Type 1 areas will be considered cheaper to traverse. Any areas above type 1 will have a multiplier  of 1.0.
);
```
