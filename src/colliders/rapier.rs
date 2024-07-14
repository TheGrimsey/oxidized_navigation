use crate::parry::parry3d::{bounding_volume::Aabb, shape::TypedShape};

use super::OxidizedCollider;

/// This is only compiled and available when the "rapier" feature is enabled.
impl OxidizedCollider for bevy_rapier3d::prelude::Collider {
    fn oxidized_into_typed_shape(&self) -> TypedShape {
        self.raw.as_typed_shape()
    }

    fn oxidized_compute_local_aabb(&self) -> Aabb {
        self.raw.compute_local_aabb()
    }
}
