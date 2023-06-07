## 0.5.0 (Unreleased)

- ``find_path`` has been renamed ``find_polygon_path``
- A new ``find_path`` function has been added that does both polygon path finding and stringpulling in one.
- ``OxidizedNavigationPlugin`` now takes a settings parameter containing ``NavMeshSettings``
- Refactors to geometry gathering, the game should now be blocked less when tile rebuilding starts. 

## 0.4.0 (2023-04-11)

- Nav-Mesh generation now correctly reacts to removing the ``NavMeshAffector`` component.
- ``NavMeshAffector`` is now an empty type, the data has been moved to a resource to enable the above change.

## 0.3.0 (2023-03-07)

- Updated to Bevy 0.10
- Removed ``NavMeshGenerationState``, you can replicate the same functionality by using ``configure_set`` to add a run condition to the ``OxidizedNavigation`` SystemSet. 

## 0.2.0 (2023-02-13)

- Implemented ``walkable_radius``. This will "pull-back" the nav-mesh from edges, which means anywhere on the nav-mesh should be fine to stand on for a character with a radius of ``walkable_radius * cell_width`` 
- Added area cost multipliers.
- Added ``NavMeshAreaType`` component, allowing changing the area type of an entity from the default 0, where a value of ``None`` is unwalkable.
- Added ``NavMeshGenerationState`` allowing you to pause nav-mesh tile generation.

- Added debug draw to example.
- Disabled compiling default Bevy features.
- Refactored code to be more rust-y.
- Update to ``bevy_rapier`` 0.20 

## 0.1.1 (2023-01-18)

- Fix minor bug in contour generation.

## 0.1.0 (2023-01-16)

- Initial release.