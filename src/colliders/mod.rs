use bevy::prelude::Component;

#[cfg(feature = "rapier")]
pub mod rapier;
#[cfg(feature = "xpbd")]
pub mod xpbd;

/// The trait that is require to implement for the collider component that you want to use with oxidized-navigation.
/// Essentially it allows you to use any bevy component that contains a `parry3d::shape::SharedShape` as a collider.
///
/// This trait is already implemented for `bevy_rapier3d::prelude::Collider` and `bevy_xpbd_3d::prelude::Collider`.
///
/// See the parry3d example for how to implement this trait for a custom component that wraps a `parry3d::shape::SharedShape`.
pub trait OxidizedCollider: Component {
    // Names are changed to avoid conflicting with the function calls on a `parry3d::shape::SharedShape`.
    fn oxidized_into_typed_shape(&self) -> parry3d::shape::TypedShape;

    fn oxidized_compute_local_aabb(&self) -> parry3d::bounding_volume::Aabb;
}
