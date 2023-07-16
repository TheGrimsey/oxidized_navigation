use parry3d::{bounding_volume::Aabb, shape::TypedShape};

#[cfg(feature = "rapier")]
pub mod rapier;
#[cfg(feature = "xpbd")]
pub mod xpbd;

pub trait Collider {
    fn into_typed_shape(&self) -> TypedShape;

    fn t_compute_local_aabb(&self) -> Aabb;
}
