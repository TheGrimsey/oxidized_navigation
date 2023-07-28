use bevy_xpbd_3d::prelude::Collider as XpbdCollider;
use parry3d::{bounding_volume::Aabb, shape::TypedShape};

use super::OxidizedCollider;

impl OxidizedCollider for XpbdCollider {
    fn into_typed_shape(&self) -> TypedShape {
        self.as_typed_shape()
    }

    fn t_compute_local_aabb(&self) -> Aabb {
        self.compute_local_aabb()
    }
}
