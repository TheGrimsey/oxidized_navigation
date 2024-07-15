
cfg_if::cfg_if! {
    if #[cfg(all(feature = "parry_016", feature = "parry_015"))] {
        compile_error!("You must pick a single parry3d feature.");
    } else if #[cfg(feature = "parry_015")] {
        pub use parry3d_015 as parry3d;
    } else if #[cfg(feature = "parry_016")] {
        pub use parry3d_016 as parry3d;
    } else {
        compile_error!("You must enabled either the `parry_015` or `parry_016` feature matching, whichever matches your Parry3d version.");
    }
}
