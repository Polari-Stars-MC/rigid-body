#![allow(clippy::missing_safety_doc)]

pub extern crate rapier3d;
pub mod helper;
pub mod rapier;

pub use rapier::ffi::*;

/// Re-export the JNI-facing types and utilities that `mps-jni` needs.
pub mod jni_api {
    pub use crate::helper::*;
    pub use crate::rapier::{
        bounds, bridge, collider, compat, controller, crbtree, dop,
        error, events, joints, neural, query, rigid_body, rtree,
        spaceflight, voxel, world,
        aerodynamics as aero, fluid as fl, trajectory as traj, molecular as mol,
    };
    pub use crate::rapier::ffi::{self, *};
    #[cfg(feature = "anvilkit-bridge")]
    pub use crate::rapier::anvilkit;
}

#[cfg(feature = "anvilkit-bridge")]
pub use rapier::ffi::AnvilKitAppHandle;