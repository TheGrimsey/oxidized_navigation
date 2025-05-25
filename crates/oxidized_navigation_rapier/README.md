# Oxidized Navigation: Rapier

Rapier integration for [Oxidized Navigation](https://crates.io/crates/oxidized_navigation/)

## Version Mapping

| Oxidized Navigation | Integration | Rapier3D |
| ------------------  | ----------- | -------- |
| 0.13 | 0.1 | 0.28

## Usage

Add both this crate & `oxidized_navigation` as dependencies, then add the OxidizedNavigationPlugin with `RapierCollider`.
```rust
OxidizedNavigationPlugin::<RapierCollider>::new(nav_mesh_settings),
```