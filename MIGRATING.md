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
