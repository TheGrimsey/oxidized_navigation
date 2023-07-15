use bevy_rapier3d::prelude::Collider as RCollider;
use parry3d::{bounding_volume::Aabb, math::Real, na::Point3, shape::TypedShape};

use super::Collider;

impl Collider for RCollider {
    // fn into_trimesh(&self) -> (Vec<Point3<Real>>, Vec<[u32; 3]>) {
    //     self.as_trimesh()
    // }

    fn as_typed_shape(&self) -> TypedShape {
        self.raw.as_typed_shape()
    }

    fn compute_local_aabb(&self) -> Aabb {
        self.raw.compute_local_aabb()
    }
}
