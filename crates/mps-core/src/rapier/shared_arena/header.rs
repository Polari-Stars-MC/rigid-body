//! `shared_arena::header` submodule — header field accessors, flag
//! manipulation, integration-params and force-summary region writes.
//!
//! Split out of the original 1028-line `shared_arena.rs` per OPTIMIZATION.md
//! §N5.  All methods here extend [`super::SharedPhysicsArena`] and address
//! raw header bytes via the `OFF_*` constants re-exported from `super`.

use std::sync::atomic::{AtomicU32, Ordering};

impl super::SharedPhysicsArena {
    // -----------------------------------------------------------------------
    // Header accessors
    // -----------------------------------------------------------------------

    pub fn header_u32(&self, offset: usize) -> u32 {
        use std::sync::atomic::{AtomicU32, Ordering};
        debug_assert_eq!(offset % std::mem::align_of::<AtomicU32>(), 0);
        unsafe { (&*(self.ptr.add(offset) as *const AtomicU32)).load(Ordering::Acquire) }
    }

    pub(super) fn set_header_u32(&self, offset: usize, value: u32) {
        use std::sync::atomic::{AtomicU32, Ordering};
        debug_assert_eq!(offset % std::mem::align_of::<AtomicU32>(), 0);
        unsafe { (&*(self.ptr.add(offset) as *const AtomicU32)).store(value, Ordering::Release) }
    }

    pub fn header_u64(&self, offset: usize) -> u64 {
        use std::sync::atomic::{AtomicU64, Ordering};
        debug_assert_eq!(offset % std::mem::align_of::<AtomicU64>(), 0);
        unsafe { (&*(self.ptr.add(offset) as *const AtomicU64)).load(Ordering::Acquire) }
    }

    /// Set flags in the header atomically.
    pub fn set_flags(&self, flags: u32) {
        let ptr = unsafe { self.ptr.add(12) as *mut AtomicU32 };
        unsafe {
            (*ptr).fetch_or(flags, Ordering::Release);
        }
    }

    /// Clear flags in the header atomically.
    pub fn clear_flags(&self, flags: u32) {
        let ptr = unsafe { self.ptr.add(12) as *mut AtomicU32 };
        unsafe {
            (*ptr).fetch_and(!flags, Ordering::Release);
        }
    }

    // -----------------------------------------------------------------------
    // Integration parameters (zero-JNI read/write by Java)
    // -----------------------------------------------------------------------

    /// Flush integration parameters into the arena's integration_params region.
    pub fn flush_integration_params(
        &self,
        dt: f64,
        solver_iterations: u32,
        ccd_substeps: u32,
        gravity: &rapier3d::prelude::Vector,
    ) {
        let base = self.integration_params_offset;
        use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
        unsafe {
            (&*(self.ptr.add(base) as *const AtomicU64)).store(dt.to_bits(), Ordering::Relaxed);
            (&*(self.ptr.add(base + 8) as *const AtomicU32))
                .store(solver_iterations, Ordering::Relaxed);
            (&*(self.ptr.add(base + 12) as *const AtomicU32))
                .store(ccd_substeps, Ordering::Relaxed);
            (&*(self.ptr.add(base + 16) as *const AtomicU64))
                .store(gravity.x.to_bits(), Ordering::Relaxed);
            (&*(self.ptr.add(base + 24) as *const AtomicU64))
                .store(gravity.y.to_bits(), Ordering::Relaxed);
            (&*(self.ptr.add(base + 32) as *const AtomicU64))
                .store(gravity.z.to_bits(), Ordering::Relaxed);
        }
    }

    /// Flush the aggregate force summary into the arena's force_summary region.
    pub fn flush_force_report(
        &self,
        max_reynolds: f64,
        total_external_force: &crate::rapier::ffi::Vec3,
        total_drag_force: &crate::rapier::ffi::Vec3,
        drag_body_count: u32,
        ext_force_body_count: u32,
    ) {
        let base = self.force_summary_offset;
        use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
        unsafe {
            (&*(self.ptr.add(base) as *const AtomicU64))
                .store(max_reynolds.to_bits(), Ordering::Relaxed);
            (&*(self.ptr.add(base + 8) as *const AtomicU64))
                .store(total_external_force.x.to_bits(), Ordering::Relaxed);
            (&*(self.ptr.add(base + 16) as *const AtomicU64))
                .store(total_external_force.y.to_bits(), Ordering::Relaxed);
            (&*(self.ptr.add(base + 24) as *const AtomicU64))
                .store(total_external_force.z.to_bits(), Ordering::Relaxed);
            (&*(self.ptr.add(base + 32) as *const AtomicU64))
                .store(total_drag_force.x.to_bits(), Ordering::Relaxed);
            (&*(self.ptr.add(base + 40) as *const AtomicU64))
                .store(total_drag_force.y.to_bits(), Ordering::Relaxed);
            (&*(self.ptr.add(base + 48) as *const AtomicU64))
                .store(total_drag_force.z.to_bits(), Ordering::Relaxed);
            (&*(self.ptr.add(base + 56) as *const AtomicU32))
                .store(drag_body_count, Ordering::Relaxed);
            (&*(self.ptr.add(base + 60) as *const AtomicU32))
                .store(ext_force_body_count, Ordering::Relaxed);
        }
    }
}
