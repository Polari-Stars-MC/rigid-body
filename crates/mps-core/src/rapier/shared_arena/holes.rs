//! `shared_arena::holes` submodule — body slots, collider slots, body handle
//! map, and per-ForceLawType force breakdown.
//!
//! Split out of the original 1028-line `shared_arena.rs` per OPTIMIZATION.md
//! §N5.  The "holes" naming follows the OPTIMIZATION.md suggestion: the body
//! and collider slot arrays are the variably-populated "holes" inside the
//! arena's contiguous layout, written by Rust after each `world_step`.

use std::sync::atomic::{AtomicU64, Ordering};

use rapier3d::prelude::RigidBodyType;

use super::{BODY_SLOT_STRIDE, COLLIDER_SLOT_STRIDE, OFF_FORCE_LAW_COUNT};

impl super::SharedPhysicsArena {
    // -----------------------------------------------------------------------
    // Body slot access
    // -----------------------------------------------------------------------

    /// Get pointer to a body slot.
    pub fn body_slot_ptr(&self, index: u32) -> *mut u8 {
        unsafe {
            self.ptr
                .add(self.body_slots_offset + index as usize * BODY_SLOT_STRIDE as usize)
        }
    }

    /// Flush a single body's state to its arena slot.
    ///
    /// Called after `world_step` for each active body.
    #[allow(clippy::too_many_arguments)] // layout-frozen slot writer
    pub fn flush_body(
        &self,
        index: u32,
        pos_x: f64,
        pos_y: f64,
        pos_z: f64,
        vel_x: f64,
        vel_y: f64,
        vel_z: f64,
        angvel_x: f64,
        angvel_y: f64,
        angvel_z: f64,
        body_type: u32,
        sleeping: u32,
        user_data: u64,
    ) {
        if index >= self.max_bodies {
            return;
        }

        let slot = self.body_slot_ptr(index);

        unsafe {
            // Increment generation to odd (writing)
            let gen_ptr = &*(slot as *const AtomicU64);
            let current_gen = gen_ptr.load(Ordering::Relaxed);
            gen_ptr.store(current_gen.wrapping_add(1) | 1, Ordering::Release);

            // Write data (atomic stores, mirroring mps-cosmos so the seqlock
            // payload is not a Rust data race on weakly-ordered targets).
            use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
            (&*(slot.add(8) as *const AtomicU64)).store(pos_x.to_bits(), Ordering::Relaxed);
            (&*(slot.add(16) as *const AtomicU64)).store(pos_y.to_bits(), Ordering::Relaxed);
            (&*(slot.add(24) as *const AtomicU64)).store(pos_z.to_bits(), Ordering::Relaxed);
            (&*(slot.add(32) as *const AtomicU64)).store(vel_x.to_bits(), Ordering::Relaxed);
            (&*(slot.add(40) as *const AtomicU64)).store(vel_y.to_bits(), Ordering::Relaxed);
            (&*(slot.add(48) as *const AtomicU64)).store(vel_z.to_bits(), Ordering::Relaxed);
            (&*(slot.add(56) as *const AtomicU64)).store(angvel_x.to_bits(), Ordering::Relaxed);
            (&*(slot.add(64) as *const AtomicU64)).store(angvel_y.to_bits(), Ordering::Relaxed);
            (&*(slot.add(72) as *const AtomicU64)).store(angvel_z.to_bits(), Ordering::Relaxed);
            (&*(slot.add(80) as *const AtomicU32)).store(body_type, Ordering::Relaxed);
            (&*(slot.add(84) as *const AtomicU32)).store(sleeping, Ordering::Relaxed);
            (&*(slot.add(88) as *const AtomicU64)).store(user_data, Ordering::Relaxed);

            // Increment generation to even (done writing)
            gen_ptr.store(current_gen.wrapping_add(2), Ordering::Release);
        }
    }

    /// Mark a body slot as empty (no longer in use).
    pub fn clear_body_slot(&self, index: u32) {
        if index >= self.max_bodies {
            return;
        }
        let slot = self.body_slot_ptr(index);
        unsafe {
            // Set generation to 0 (Java side treats gen=0 as "empty slot")
            (&*(slot as *const AtomicU64)).store(0, Ordering::Release);
        }
    }

    // -----------------------------------------------------------------------
    // Body handle map — arena index → Rapier RigidBodyHandle
    // -----------------------------------------------------------------------

    /// Flush all active bodies to their arena slots.
    ///
    /// Called after `world_step` completes.
    ///
    /// M3 优化（性能分析.MD §11.3）：尾部不再 `for i in index..max_bodies:
    /// clear_body_slot(i)` 把全部 `max_bodies - index` 个空 slot 都写 gen=0。
    /// 改为只回收 `[ curr_count .. prev_count ]` 区间——即"上次被填过、
    /// 这次变少"的那一部分；slot `[ prev_count .. max_bodies ]` 从未被填，
    /// 上一帧首调时已是 gen=0（`alloc_zeroed` 兜底），保留即可。
    ///
    /// 如此稳态（计数不变）情况下每个 slot 的 gen 写入正好等于前一次，无
    /// 多余清零；1000 active / max_bodies=4096 这种典型 MC 拓扑，每帧把
    /// `3096 × 8B` 的清零写带宽一次直接抹掉，约 ~24KB/tick。
    pub fn flush_all_bodies(&self, bodies: &rapier3d::prelude::RigidBodySet) {
        let mut index = 0u32;
        for (handle, body) in bodies.iter() {
            if index >= self.max_bodies {
                break;
            }

            // Write body handle map (arena index → Rapier handle)
            self.write_body_handle(index, handle.into_raw_parts().0 as u64);

            let pos = body.translation();
            let vel = body.linvel();
            let angvel = body.angvel();

            let body_type = match body.body_type() {
                RigidBodyType::Dynamic => 0u32,
                RigidBodyType::Fixed => 1u32,
                RigidBodyType::KinematicVelocityBased => 2u32,
                RigidBodyType::KinematicPositionBased => 3u32,
                // Upstream v0.36 soft-body proxy: reported as dynamic, matching
                // `RigidBodyType::is_dynamic`.
                RigidBodyType::SoftFrame => 0u32,
            };

            let sleeping = if body.is_sleeping() { 1u32 } else { 0u32 };
            let user_data = (body.user_data & 0xFFFF_FFFF_FFFF_FFFF) as u64;

            self.flush_body(
                index, pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, angvel.x, angvel.y, angvel.z,
                body_type, sleeping, user_data,
            );

            index += 1;
        }

        // Update body count in header
        self.set_header_u32(32, index);

        // 回收 [ curr_count .. prev_count ]——仅上次被填过的缩水部分。
        // slot [ prev_count .. max_bodies ] 已为 gen=0，不动。
        let prev = self.prev_body_active_count.load(Ordering::Relaxed);
        if index < prev {
            for i in index..prev {
                self.clear_body_slot(i);
            }
        }
        self.prev_body_active_count.store(index, Ordering::Relaxed);
    }

    /// Write a body handle into the handle map.
    fn write_body_handle(&self, index: u32, handle_raw: u64) {
        if self.body_handle_map_offset == 0 || index >= self.max_bodies {
            return;
        }
        unsafe {
            let slot = self
                .ptr
                .add(self.body_handle_map_offset + index as usize * 8);
            use std::sync::atomic::{AtomicU64, Ordering};
            (&*(slot as *const AtomicU64)).store(handle_raw, Ordering::Relaxed);
        }
    }

    // -----------------------------------------------------------------------
    // Force report — per-ForceLawType contributions (32 slots)
    // -----------------------------------------------------------------------

    /// Flush the per-frame ForceReport to the arena's force_report region.
    ///
    /// Writes up to 32 ForceLawType contributions so Java can read which
    /// force types are active and how much force each contributed.
    pub fn flush_force_breakdown(&self, report: &crate::rapier::forces::ForceReport) {
        if self.force_report_offset == 0 {
            return;
        }

        let mut count = 0u32;
        for (law_type, contrib) in &report.contributions {
            if count >= 32 {
                break;
            }
            let type_tag = crate::rapier::ffi::force_law_type_tag(law_type);
            let offset = self.force_report_offset + count as usize * 32;
            unsafe {
                use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
                (&*(self.ptr.add(offset) as *const AtomicU64))
                    .store(contrib.total_force.x.to_bits(), Ordering::Relaxed);
                (&*(self.ptr.add(offset + 8) as *const AtomicU64))
                    .store(contrib.total_force.y.to_bits(), Ordering::Relaxed);
                (&*(self.ptr.add(offset + 16) as *const AtomicU64))
                    .store(contrib.total_force.z.to_bits(), Ordering::Relaxed);
                (&*(self.ptr.add(offset + 24) as *const AtomicU32))
                    .store(contrib.body_count, Ordering::Relaxed);
                (&*(self.ptr.add(offset + 28) as *const AtomicU32))
                    .store(type_tag, Ordering::Relaxed);
            }
            count += 1;
        }

        // Update header: force_law_count
        self.set_header_u32(OFF_FORCE_LAW_COUNT, count);

        // Clear remaining slots
        for i in count..32 {
            let offset = self.force_report_offset + i as usize * 32;
            unsafe {
                (self.ptr.add(offset) as *mut f64).write_unaligned(0.0);
                (self.ptr.add(offset + 8) as *mut f64).write_unaligned(0.0);
                (self.ptr.add(offset + 16) as *mut f64).write_unaligned(0.0);
                (self.ptr.add(offset + 24) as *mut u32).write_unaligned(0);
            }
        }
    }

    // -----------------------------------------------------------------------
    // Collider slot access
    // -----------------------------------------------------------------------

    fn collider_slot_ptr(&self, index: u32) -> *mut u8 {
        unsafe {
            self.ptr
                .add(self.collider_slots_offset + index as usize * COLLIDER_SLOT_STRIDE as usize)
        }
    }

    /// Flush a single collider's state.  Layout (80 bytes):
    ///   offset 0 : generation (u64)
    ///   offset 8 : parent_body_index (u32) + padding (4)
    ///   offset 16: pos_x, pos_y, pos_z (3 × f64)
    ///   offset 40: friction (f64)
    ///   offset 48: restitution (f64)
    ///   offset 56: density (f64)
    ///   offset 64: sensor (u32), active_events (u32)
    ///   offset 72: collision_groups_memberships (u32), filter (u32)
    #[allow(clippy::too_many_arguments)] // layout-frozen slot writer
    pub fn flush_collider(
        &self,
        index: u32,
        parent_body_index: u32,
        pos_x: f64,
        pos_y: f64,
        pos_z: f64,
        friction: f64,
        restitution: f64,
        density: f64,
        sensor: u32,
        active_events: u32,
        collision_groups_memberships: u32,
        collision_groups_filter: u32,
    ) {
        if index >= self.max_colliders {
            return;
        }
        let slot = self.collider_slot_ptr(index);
        unsafe {
            let gen_ptr = &*(slot as *const AtomicU64);
            let current_gen = gen_ptr.load(Ordering::Relaxed);
            gen_ptr.store(current_gen.wrapping_add(1) | 1, Ordering::Release);

            use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
            (&*(slot.add(8) as *const AtomicU32)).store(parent_body_index, Ordering::Relaxed);
            (&*(slot.add(16) as *const AtomicU64)).store(pos_x.to_bits(), Ordering::Relaxed);
            (&*(slot.add(24) as *const AtomicU64)).store(pos_y.to_bits(), Ordering::Relaxed);
            (&*(slot.add(32) as *const AtomicU64)).store(pos_z.to_bits(), Ordering::Relaxed);
            (&*(slot.add(40) as *const AtomicU64)).store(friction.to_bits(), Ordering::Relaxed);
            (&*(slot.add(48) as *const AtomicU64)).store(restitution.to_bits(), Ordering::Relaxed);
            (&*(slot.add(56) as *const AtomicU64)).store(density.to_bits(), Ordering::Relaxed);
            (&*(slot.add(64) as *const AtomicU32)).store(sensor, Ordering::Relaxed);
            (&*(slot.add(68) as *const AtomicU32)).store(active_events, Ordering::Relaxed);
            (&*(slot.add(72) as *const AtomicU32))
                .store(collision_groups_memberships, Ordering::Relaxed);
            (&*(slot.add(76) as *const AtomicU32))
                .store(collision_groups_filter, Ordering::Relaxed);

            gen_ptr.store(current_gen.wrapping_add(2), Ordering::Release);
        }
    }

    fn clear_collider_slot(&self, index: u32) {
        if index >= self.max_colliders {
            return;
        }
        let slot = self.collider_slot_ptr(index);
        unsafe {
            (&*(slot as *const AtomicU64)).store(0, Ordering::Release);
        }
    }

    /// Flush all colliders after world_step.
    ///
    /// L4 优化（性能分析.MD §12.4）：与 M3 同形——尾部不再
    /// `for i in index..max_colliders: clear_collider_slot(i)`，改为只
    /// 回收 `[ curr_count .. prev_count ]`。稳态下 collider 数稳定时无
    /// 任何多余清零写，常见 500 active / max_colliders=4096 把每帧
    /// ~3596 × 8B ≈ 29KB/tick 的写带宽直接抹掉。
    pub fn flush_all_colliders(&self, colliders: &rapier3d::prelude::ColliderSet) {
        let mut index = 0u32;
        for (_handle, collider) in colliders.iter() {
            if index >= self.max_colliders {
                break;
            }
            let pos = collider.translation();
            let parent = collider.parent().map_or(u32::MAX, |h| h.into_raw_parts().0);
            self.flush_collider(
                index,
                parent,
                pos.x,
                pos.y,
                pos.z,
                collider.friction(),
                collider.restitution(),
                collider.density(),
                if collider.is_sensor() { 1 } else { 0 },
                collider.active_events().bits(),
                collider.collision_groups().memberships.bits(),
                collider.collision_groups().filter.bits(),
            );
            index += 1;
        }
        self.set_header_u32(36, index);
        // 回收 [ curr_count .. prev_count ]——仅上次被填过的缩水部分。
        let prev = self.prev_collider_active_count.load(Ordering::Relaxed);
        if index < prev {
            for i in index..prev {
                self.clear_collider_slot(i);
            }
        }
        self.prev_collider_active_count
            .store(index, Ordering::Relaxed);
    }
}
