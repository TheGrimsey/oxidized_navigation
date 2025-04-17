use parry3d::{bounding_volume::Aabb, shape::TypedShape};

use super::OxidizedCollider;

pub struct AvianCollider;

/// This is only compiled and available when the "avian" feature is enabled.
impl OxidizedCollider for AvianCollider {
    type Component = avian3d::prelude::Collider;

    fn oxidized_into_typed_shape(collider: &avian3d::prelude::Collider) -> TypedShape {
        collider.shape_scaled().as_typed_shape()
    }

    fn oxidized_compute_local_aabb(collider: &avian3d::prelude::Collider) -> Aabb {
        collider.shape_scaled().compute_local_aabb()
    }
}
