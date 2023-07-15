use parry3d::{bounding_volume::Aabb, math::Real, na::Point3, shape::TypedShape};

#[cfg(feature = "rapier")]
pub mod rapier;

pub trait Collider {
    // fn into_trimesh(&self) -> (Vec<Point3<Real>>, Vec<[u32; 3]>);

    fn as_typed_shape(&self) -> TypedShape;

    fn compute_local_aabb(&self) -> Aabb;
}
