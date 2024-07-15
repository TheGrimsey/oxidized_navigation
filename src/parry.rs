
#[cfg(feature = "parry_016")]
pub use parry3d_016 as parry3d;

#[cfg(feature = "parry_015")]
pub use parry3d_015 as parry3d;

#[cfg(all(feature = "parry_016", feature = "parry_015"))]
compile_error!("You must pick a single parry3d feature.");

#[cfg(all(not(feature = "parry_016"), not(feature = "parry_015")))]
compile_error!("You must enabled a Parry3d feature matching your version. Either 0.16 or 0.15.");
