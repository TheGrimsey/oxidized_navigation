# Oxidized Navigation: Avian

Avian integration for [Oxidized Navigation](https://crates.io/crates/oxidized_navigation/)

## Version Mapping

| Oxidized Navigation | Integration | Avian3D |
| ------------------  | ----------- | -------- |
| 0.13 | 0.1 | 0.2

## Usage

Add both this crate & `oxidized_navigation` as dependencies, then add the OxidizedNavigationPlugin with `AvianCollider`.
```rust
OxidizedNavigationPlugin::<AvianCollider>::new(nav_mesh_settings),
```
