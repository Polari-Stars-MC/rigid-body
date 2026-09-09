//! Default collider selection for Java/JNI and C/FFM callers.
//! Changing the policy affects only `world_insert_default_collider`; explicit
//! collider APIs and previously inserted colliders retain their behavior.

use super::error::{
    ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, clear_error, ffi_guard, set_error,
};
use super::ffi::{
    Bool, ColliderBuilderHandle, ColliderHandleRaw, RigidBodyHandleRaw, Vec3, WorldHandle,
    pack_collider_handle, unpack_rigid_body_handle,
};

/// Policy used when inserting a collider through `world_insert_default_collider`.
///
/// cbindgen:prefix-with-name
#[repr(u32)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum WorldCollisionMode {
    /// No collider is inserted. Set body mass explicitly if needed.
    None = 0,
    /// Select the non-compound builder supplied by the caller.
    #[default]
    Simple = 1,
    /// Select the compound builder supplied by the caller.
    Compound = 2,
    /// Select compound when available, otherwise simple.
    Adaptive = 3,
}

impl WorldCollisionMode {
    fn from_raw(mode: u32) -> Option<Self> {
        match mode {
            0 => Some(Self::None),
            1 => Some(Self::Simple),
            2 => Some(Self::Compound),
            3 => Some(Self::Adaptive),
            _ => None,
        }
    }
}

/// Creates a world with a default collider policy (0=None, 1=Simple, 2=Compound, 3=Adaptive).
/// Invalid mode returns null. Gravity follows `world_create` semantics.
#[unsafe(no_mangle)]
pub extern "C" fn world_create_with_collision_mode(gravity: Vec3, mode: u32) -> *mut WorldHandle {
    ffi_guard(std::ptr::null_mut(), || {
        let Some(mode) = WorldCollisionMode::from_raw(mode) else {
            set_error(ERR_INVALID_ARGUMENT, "invalid collision mode");
            return std::ptr::null_mut();
        };
        let world = super::world::world_create(gravity);
        if let Some(w) = unsafe { world.as_mut() } {
            w.inner.default_collision_mode = mode;
            clear_error();
        }
        world
    })
}

/// Changes the default policy for future default insertions only.
/// # Safety
/// `world` must be live and exclusively accessible, including relative to step.
#[unsafe(no_mangle)]
pub extern "C" fn world_set_default_collision_mode(world: *mut WorldHandle, mode: u32) -> Bool {
    ffi_guard(Bool::FALSE, || {
        let Some(w) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return Bool::FALSE;
        };
        let Some(mode) = WorldCollisionMode::from_raw(mode) else {
            set_error(ERR_INVALID_ARGUMENT, "invalid collision mode");
            return Bool::FALSE;
        };
        w.inner.default_collision_mode = mode;
        clear_error();
        Bool::TRUE
    })
}

/// Returns the policy or `u32::MAX` on error.
/// # Safety
/// `world` must be live, with no concurrent mutation.
#[unsafe(no_mangle)]
pub extern "C" fn world_get_default_collision_mode(world: *const WorldHandle) -> u32 {
    ffi_guard(u32::MAX, || {
        let Some(w) = (unsafe { world.as_ref() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return u32::MAX;
        };
        clear_error();
        w.inner.default_collision_mode as u32
    })
}

/// Inserts a collider selected by the world's default policy. Builders are
/// borrowed, not consumed, and can be reused. None mode returns 0 with ERR_OK;
/// errors return 0 with a nonzero last error. Only the selected builder is read.
/// Compound mode requires a compound shape; Simple rejects compound shapes.
/// Repeated calls add colliders; existing colliders are never removed/replaced.
/// # Safety
/// `world` must be live and exclusively accessible. The selected builder must
/// be live, aligned, and not concurrently mutated; unused builders may be null.
#[unsafe(no_mangle)]
pub extern "C" fn world_insert_default_collider(
    world: *mut WorldHandle,
    body: RigidBodyHandleRaw,
    simple_builder: *const ColliderBuilderHandle,
    compound_builder: *const ColliderBuilderHandle,
) -> ColliderHandleRaw {
    ffi_guard(0, || {
        let Some(w) = (unsafe { world.as_mut() }) else {
            set_error(ERR_NULL_POINTER, "world is null");
            return 0;
        };
        let parent = unpack_rigid_body_handle(body);
        if !w.inner.bodies.contains(parent) {
            set_error(ERR_NOT_FOUND, "rigid body not found");
            return 0;
        }
        let mode = w.inner.default_collision_mode;
        let (selected, selected_is_compound) = match mode {
            WorldCollisionMode::None => {
                clear_error();
                return 0;
            }
            WorldCollisionMode::Simple => (simple_builder, false),
            WorldCollisionMode::Compound => (compound_builder, true),
            WorldCollisionMode::Adaptive if !compound_builder.is_null() => (compound_builder, true),
            WorldCollisionMode::Adaptive => (simple_builder, false),
        };
        if selected.is_null() {
            set_error(ERR_NULL_POINTER, "selected builder is null");
            return 0;
        }
        if !selected.is_aligned() {
            set_error(ERR_INVALID_ARGUMENT, "misaligned builder");
            return 0;
        }
        let builder = unsafe { &*selected };
        let collider = builder.inner.build();
        if collider.shape().as_compound().is_some() != selected_is_compound {
            set_error(
                ERR_INVALID_ARGUMENT,
                "builder shape does not match collision mode",
            );
            return 0;
        }
        let handle = w
            .inner
            .colliders
            .insert_with_parent(collider, parent, &mut w.inner.bodies);
        let raw = pack_collider_handle(handle);
        if let Some(source) = &builder.voxel_source {
            w.inner.voxel_grids.insert(raw, source.clone());
        }
        clear_error();
        raw
    })
}
