#[macro_export]
macro_rules! use_appropriate_parry3d {
    () => {
        cfg_if::cfg_if! {
            if #[cfg(all(feature = "xpbd", feature = "rapier"))] {
                compile_error!("Features 'xpbd' and 'rapier' cannot be enabled at the same time.");
            } else if #[cfg(all(feature = "xpbd", feature = "parry_standalone"))] {
                compile_error!("Features 'xpbd' and 'parry_standalone' cannot be enabled at the same time.");
            } else if #[cfg(all(feature = "rapier", feature = "parry_standalone"))] {
                compile_error!("Features 'rapier' and 'parry_standalone' cannot be enabled at the same time.");
            } else if #[cfg(feature = "xpbd")] {
                use parry3d_xpbd as parry3d;
            } else if #[cfg(feature = "rapier")] {
                use parry3d_rapier as parry3d;
            } else if #[cfg(feature = "parry_standalone")] {
                use parry3d_rapier as parry3d;
            } else {
                compile_error!("One of the features 'xpbd', 'rapier', or 'parry_standalone' must be enabled.");
            }
        }
    };
}