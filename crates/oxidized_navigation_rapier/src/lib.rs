use parry3d::{bounding_volume::Aabb, shape::TypedShape};

use super::OxidizedCollider;

pub struct RapierCollider;

/// This is only compiled and available when the "rapier" feature is enabled.
impl OxidizedCollider for RapierCollider {
    type Component = bevy_rapier3d::prelude::Collider;

    fn oxidized_into_typed_shape(collider: &bevy_rapier3d::prelude::Collider) -> TypedShape {
        collider.raw.as_typed_shape()
    }

    fn oxidized_compute_local_aabb(collider: &bevy_rapier3d::prelude::Collider) -> Aabb {
        collider.raw.compute_local_aabb()
    }
}
