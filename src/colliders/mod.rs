use bevy::ecs::component::Component;

#[cfg(feature = "avian")]
pub mod avian;
#[cfg(feature = "rapier")]
pub mod rapier;

/// The trait that is required to implement for the collider that you want to use with oxidized-navigation.
/// Essentially it allows you to use any bevy component that contains a `parry3d::shape::SharedShape` as a collider.
///
/// This trait may be implemented directly on the component (though `OxidizedCollider::Component` must still be specified),
/// or may be implemented on a different (foreign) type. This is already done with `avian3d` and `bevy_rapier3d`` colliders,
/// with AvianCollider and RapierCollider respectively.
///
/// See the parry3d example for how to implement this trait for a custom component that wraps a `parry3d::shape::SharedShape`.
pub trait OxidizedCollider: 'static {
    type Component: Component;

    // Names are changed to avoid conflicting with the function calls on a `parry3d::shape::SharedShape`.
    fn oxidized_into_typed_shape(item: &Self::Component) -> parry3d::shape::TypedShape;

    fn oxidized_compute_local_aabb(item: &Self::Component) -> parry3d::bounding_volume::Aabb;
}
