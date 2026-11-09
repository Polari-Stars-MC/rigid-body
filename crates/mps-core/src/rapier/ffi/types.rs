pub use mps_formula::ffi::types::*;

/// Magic stamped into every live [`WorldHandle`] so a stale or forged `jlong`
/// can be detected instead of dereferenced (use-after-free / wild pointer).
pub const WORLD_HANDLE_MAGIC: u64 = 0x4D50535F_57524C44; // "MPS_WRLD"

pub struct WorldHandle {
    /// Must equal [`WORLD_HANDLE_MAGIC`] while the handle is alive; zeroed by
    /// `world_destroy` before the allocation is freed.
    pub magic: u64,
    pub inner: crate::rapier::world::PhysicsWorld,
}

impl WorldHandle {
    /// Returns `true` if `ptr` points to a live world handle (correct magic).
    ///
    /// # Safety
    /// `ptr` must be null or a pointer previously returned by `world_create`.
    pub unsafe fn is_valid(ptr: *const WorldHandle) -> bool {
        if ptr.is_null() {
            return false;
        }
        // SAFETY: caller guarantees ptr is null or from world_create; reading the
        // magic word of a live allocation is fine, and for a freed handle the
        // read happens before any other deref — worst case we read freed memory
        // once, which is why world_destroy zeroes the magic instead of relying
        // on the allocator.
        unsafe { (*ptr).magic == WORLD_HANDLE_MAGIC }
    }
}

#[cfg(feature = "anvilkit-bridge")]
pub struct AnvilKitAppHandle {
    pub(crate) inner: crate::rapier::anvilkit::AnvilKitAppState,
}

pub struct RigidBodyBuilderHandle {
    pub(crate) inner: rapier3d::prelude::RigidBodyBuilder,
}

/// Opaque FFI wrapper for a **built** collider — the pointer returned by
/// `collider_builder_build` / `world_copy_collider` and consumed by
/// `world_insert_collider*` / `collider_destroy_raw`.
///
/// Keeps the Rapier `Collider` type name out of `rigid_body.h` so the header
/// stays self-contained C (same pattern as `RigidBodyBuilderHandle` /
/// `ColliderBuilderHandle`): cbindgen only emits
/// `typedef struct ColliderMemHandle ColliderMemHandle;`. A single-field repr(Rust)
/// struct has the same size/alignment as the wrapped collider, so boxing this
/// is layout-equivalent to boxing the `rapier3d::prelude::Collider` directly.
pub struct ColliderMemHandle {
    pub(crate) inner: rapier3d::prelude::Collider,
}

/// Opaque FFI wrapper for a **built** rigid body — the pointer returned by
/// `rigid_body_builder_build` / `world_copy_rigid_body` and consumed by
/// `world_insert_rigid_body` / `rigid_body_destroy_raw`.
///
/// Keeps the Rapier `RigidBody` type name out of `rigid_body.h` so the header
/// stays self-contained C. Named `RigidBodyMemHandle` (not `RigidBodyHandle`,
/// which is Rapier's index handle imported all over this crate) and follows
/// the `memory_handle` parameter name used by the FFI docs. A single-field
/// repr(Rust) struct has the same size/alignment as the wrapped body, so
/// boxing this is layout-equivalent to boxing the `rapier3d::dynamics::RigidBody`
/// directly.
pub struct RigidBodyMemHandle {
    pub(crate) inner: rapier3d::dynamics::RigidBody,
}

pub struct ColliderBuilderHandle {
    pub(crate) inner: rapier3d::prelude::ColliderBuilder,
    /// Retained voxel source grid + options when this builder was created from
    /// a voxel (so the resulting collider can be edited in place later).
    pub(crate) voxel_source: Option<crate::rapier::voxel::VoxelCache>,
}

pub struct JointBuilderHandle {
    pub(crate) inner: crate::rapier::joints::JointBuilderKind,
}

pub struct CharacterControllerHandle {
    pub(crate) inner: crate::rapier::controller::CharacterControllerState,
}

pub struct RTreeHandle {
    pub(crate) inner: rapier3d::geometry::user_index::GenericAabbIndex,
}

pub struct CRbTreeHandle {
    pub(crate) inner: crate::rapier::crbtree::CRbTreeIndex,
}
