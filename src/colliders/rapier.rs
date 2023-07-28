use bevy_rapier3d::prelude::Collider as RapierCollider;
use parry3d::{bounding_volume::Aabb, shape::TypedShape};

use super::OxidizedCollider;

impl OxidizedCollider for RapierCollider {
    fn into_typed_shape(&self) -> TypedShape {
        self.raw.as_typed_shape()
    }

    fn t_compute_local_aabb(&self) -> Aabb {
        self.raw.compute_local_aabb()
    }
}
