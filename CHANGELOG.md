## 0.2.0 (Unreleased)

- Implemented ``walkable_radius``. This will "pull-back" the nav-mesh from edges, which means anywhere on the nav-mesh should be fine to stand on for a character with a radius of ``walkable_radius * cell_width`` 
- Added area cost multipliers.
- Added ``NavMeshAreaType`` component, allowing changing the area type of an entity from the default 0, where a value of ``None`` is unwalkable.
- Added ``NavMeshGenerationState`` allowing you to pause nav-mesh tile generation.

- Added debug draw to example.
- Disabled compiling default Bevy features.
- Refactored code to be more rust-y.

## 0.1.1 (2023-01-18)

- Fix minor bug in contour generation.

## 0.1.0 (2023-01-16)

- Initial release.