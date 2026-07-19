#[cfg(test)]
mod tests {
    use smallvec::SmallVec;
    use mps_core::rapier::shared_arena::*;
    use mps_core::rapier::ffi::*;

    #[test]
    fn arena_create_and_drop() {
        let arena = SharedPhysicsArena::new(16, 32, 64, 128);
        assert!(!arena.as_ptr().is_null());
        let expected_size = HEADER_SIZE
            + 16 * BODY_SLOT_STRIDE as usize
            + 32 * COLLIDER_SLOT_STRIDE as usize
            + 16 * 8 // body_handle_map
            + 32 * 32 // force_report
            + 128 * CMD_SLOT_STRIDE as usize
            + 64 * EVENT_SLOT_STRIDE as usize;
        assert_eq!(arena.size(), expected_size,
            "expected {} got {}", expected_size, arena.size());

        // Check header magic
        let magic = arena.header_u64(0);
        assert_eq!(magic, ARENA_MAGIC);

        let version = arena.header_u32(8);
        assert_eq!(version, ARENA_VERSION);
    }

    #[test]
    fn body_flush_and_readback() {
        let arena = SharedPhysicsArena::new(8, 0, 0, 0);

        arena.flush_body(0, 1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.01, 0.02, 0.03, 0, 0, 42);

        // Read back from the slot
        let slot = arena.body_slot_ptr(0);
        unsafe {
            let generation_val = (slot as *const u64).read_unaligned();
            assert!(generation_val > 0, "generation should be non-zero");
            assert_eq!(generation_val & 1, 0, "generation should be even (stable)");

            let pos_x = (slot.add(8) as *const f64).read_unaligned();
            assert!((pos_x - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn command_drain_empty() {
        let arena = SharedPhysicsArena::new(8, 0, 0, 16);
        let cmds = arena.drain_commands();
        assert!(cmds.is_empty());
    }

    #[test]
    fn event_push_and_reset() {
        let arena = SharedPhysicsArena::new(0, 0, 8, 0);

        arena.push_collision_event(true, 1, 2, false, false);
        arena.push_collision_event(false, 3, 4, true, true);

        // Event count in header should be 2
        let count = arena.header_u32(40);
        assert_eq!(count, 2);

        arena.reset_event_ring();
        let count = arena.header_u32(40);
        assert_eq!(count, 0);
    }
}







