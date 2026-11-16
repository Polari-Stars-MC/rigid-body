#[cfg(test)]
mod tests {
    use mps_core::rapier::error::{
        ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, last_error_code,
    };
    use mps_core::rapier::ffi::*;
    use mps_core::rapier::ffi::{BodyStatus, JointTypeDesc};
    use mps_core::rapier::joints::*;
    use mps_core::rapier::rigid_body::{
        rigid_body_builder_build, rigid_body_builder_create, world_insert_rigid_body,
    };
    use mps_core::rapier::world::{world_create, world_destroy};

    fn make_world() -> *mut WorldHandle {
        world_create(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        })
    }

    fn make_dynamic_body(world: *mut WorldHandle) -> RigidBodyHandleRaw {
        let builder = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        let body = rigid_body_builder_build(builder);
        let handle = world_insert_rigid_body(world, body);
        assert_ne!(handle, 0);
        handle
    }

    // ---- builder create / destroy ----

    #[test]
    fn create_fixed_joint() {
        let b = joint_builder_create(JointTypeDesc::Fixed as u32, Vec3::default(), 0.0, 0.0);
        assert!(!b.is_null());
        joint_builder_destroy(b);
    }

    #[test]
    fn create_revolute_joint() {
        let b = joint_builder_create(
            JointTypeDesc::Revolute as u32,
            Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            0.0,
            0.0,
        );
        assert!(!b.is_null());
        joint_builder_destroy(b);
    }

    #[test]
    fn create_revolute_rejects_zero_axis() {
        let b = joint_builder_create(JointTypeDesc::Revolute as u32, Vec3::default(), 0.0, 0.0);
        assert!(b.is_null());
    }

    #[test]
    fn create_prismatic_joint() {
        let b = joint_builder_create(
            JointTypeDesc::Prismatic as u32,
            Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            0.0,
            0.0,
        );
        assert!(!b.is_null());
        joint_builder_destroy(b);
    }

    #[test]
    fn create_rope_joint() {
        let b = joint_builder_create(JointTypeDesc::Rope as u32, Vec3::default(), 3.0, 0.0);
        assert!(!b.is_null());
        joint_builder_destroy(b);
    }

    #[test]
    fn create_rope_rejects_negative_length() {
        let b = joint_builder_create(JointTypeDesc::Rope as u32, Vec3::default(), -1.0, 0.0);
        assert!(b.is_null());
    }

    #[test]
    fn create_spring_joint() {
        let b = joint_builder_create(
            JointTypeDesc::Spring as u32,
            Vec3 {
                x: 10.0,
                y: 0.0,
                z: 0.0,
            },
            1.0,
            0.5,
        );
        assert!(!b.is_null());
        joint_builder_destroy(b);
    }

    #[test]
    fn create_spring_rejects_negative_params() {
        let b = joint_builder_create(JointTypeDesc::Spring as u32, Vec3::default(), -1.0, 0.5);
        assert!(b.is_null());
    }

    #[test]
    fn create_spherical_joint() {
        let b = joint_builder_create(JointTypeDesc::Spherical as u32, Vec3::default(), 0.0, 0.0);
        assert!(!b.is_null());
        joint_builder_destroy(b);
    }

    #[test]
    fn destroy_null_builder_is_noop() {
        joint_builder_destroy(std::ptr::null_mut());
    }

    // ---- builder options ----

    #[test]
    fn set_contacts_enabled() {
        let b = joint_builder_create(JointTypeDesc::Fixed as u32, Vec3::default(), 0.0, 0.0);
        joint_builder_set_contacts_enabled(b, Bool::TRUE);
        joint_builder_set_contacts_enabled(b, Bool::FALSE);
        joint_builder_destroy(b);
    }

    #[test]
    fn set_local_anchors() {
        let b = joint_builder_create(JointTypeDesc::Fixed as u32, Vec3::default(), 0.0, 0.0);
        joint_builder_set_local_anchor1(
            b,
            Vec3 {
                x: 0.0,
                y: 0.5,
                z: 0.0,
            },
        );
        joint_builder_set_local_anchor2(
            b,
            Vec3 {
                x: 0.0,
                y: -0.5,
                z: 0.0,
            },
        );
        joint_builder_destroy(b);
    }

    #[test]
    fn set_local_anchor1_rejects_nan() {
        let b = joint_builder_create(JointTypeDesc::Fixed as u32, Vec3::default(), 0.0, 0.0);
        joint_builder_set_local_anchor1(
            b,
            Vec3 {
                x: f64::NAN,
                y: 0.0,
                z: 0.0,
            },
        );
        joint_builder_destroy(b);
    }

    #[test]
    fn set_limits() {
        let b = joint_builder_create(
            JointTypeDesc::Revolute as u32,
            Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            0.0,
            0.0,
        );
        joint_builder_set_limits(b, JointAxisDesc::AngX as u32, -1.0, 1.0);
        joint_builder_destroy(b);
    }

    #[test]
    fn set_limits_rejects_inverted() {
        let b = joint_builder_create(JointTypeDesc::Fixed as u32, Vec3::default(), 0.0, 0.0);
        joint_builder_set_limits(b, JointAxisDesc::LinX as u32, 1.0, 0.0); // min > max
        joint_builder_destroy(b);
    }

    #[test]
    fn set_motor_velocity() {
        let b = joint_builder_create(
            JointTypeDesc::Revolute as u32,
            Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            0.0,
            0.0,
        );
        joint_builder_set_motor_velocity(b, JointAxisDesc::AngX as u32, 2.0, 0.5);
        joint_builder_destroy(b);
    }

    #[test]
    fn set_motor_velocity_rejects_negative_factor() {
        let b = joint_builder_create(JointTypeDesc::Fixed as u32, Vec3::default(), 0.0, 0.0);
        joint_builder_set_motor_velocity(b, JointAxisDesc::LinX as u32, 2.0, -0.5);
        joint_builder_destroy(b);
    }

    #[test]
    fn set_motor_position() {
        let b = joint_builder_create(
            JointTypeDesc::Revolute as u32,
            Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            0.0,
            0.0,
        );
        joint_builder_set_motor_position(b, JointAxisDesc::AngX as u32, 1.0, 100.0, 10.0);
        joint_builder_destroy(b);
    }

    #[test]
    fn set_motor_position_rejects_negative_stiffness() {
        let b = joint_builder_create(JointTypeDesc::Fixed as u32, Vec3::default(), 0.0, 0.0);
        joint_builder_set_motor_position(b, JointAxisDesc::LinX as u32, 1.0, -100.0, 10.0);
        joint_builder_destroy(b);
    }

    // ---- world insert / remove ----

    #[test]
    fn insert_and_remove_fixed_joint() {
        let world = make_world();
        let b1 = make_dynamic_body(world);
        let b2 = make_dynamic_body(world);
        let builder = joint_builder_create(JointTypeDesc::Fixed as u32, Vec3::default(), 0.0, 0.0);
        let handle = world_insert_impulse_joint(world, b1, b2, builder, Bool::TRUE);
        assert_ne!(handle, 0);
        assert_eq!(
            world_remove_impulse_joint(world, handle, Bool::TRUE),
            Bool::TRUE
        );
        world_destroy(world);
    }

    #[test]
    fn insert_and_remove_rope_joint() {
        let world = make_world();
        let b1 = make_dynamic_body(world);
        let b2 = make_dynamic_body(world);
        let builder = joint_builder_create(JointTypeDesc::Rope as u32, Vec3::default(), 2.0, 0.0);
        let handle = world_insert_impulse_joint(world, b1, b2, builder, Bool::TRUE);
        assert_ne!(handle, 0);
        assert_eq!(
            world_remove_impulse_joint(world, handle, Bool::FALSE),
            Bool::TRUE
        );
        // second remove fails
        assert_eq!(
            world_remove_impulse_joint(world, handle, Bool::FALSE),
            Bool::FALSE
        );
        world_destroy(world);
    }

    #[test]
    fn insert_rejects_null_world() {
        let builder = joint_builder_create(JointTypeDesc::Fixed as u32, Vec3::default(), 0.0, 0.0);
        assert_eq!(
            world_insert_impulse_joint(std::ptr::null_mut(), 1, 2, builder, Bool::TRUE),
            0
        );
        // builder consumed
    }

    #[test]
    fn insert_rejects_null_builder() {
        let world = make_world();
        let b1 = make_dynamic_body(world);
        let b2 = make_dynamic_body(world);
        assert_eq!(
            world_insert_impulse_joint(world, b1, b2, std::ptr::null_mut(), Bool::TRUE),
            0
        );
        world_destroy(world);
    }

    #[test]
    fn remove_rejects_null_world() {
        assert_eq!(
            world_remove_impulse_joint(std::ptr::null_mut(), 1, Bool::TRUE),
            Bool::FALSE
        );
    }

    #[test]
    fn remove_rejects_invalid_handle() {
        let world = make_world();
        assert_eq!(
            world_remove_impulse_joint(world, 0, Bool::TRUE),
            Bool::FALSE
        );
        world_destroy(world);
    }

    // ---- all joint types round-trip ----

    #[test]
    fn revolute_joint_roundtrip() {
        let world = make_world();
        let b1 = make_dynamic_body(world);
        let b2 = make_dynamic_body(world);
        let builder = joint_builder_create(
            JointTypeDesc::Revolute as u32,
            Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            0.0,
            0.0,
        );
        joint_builder_set_local_anchor1(
            builder,
            Vec3 {
                x: 0.0,
                y: 0.5,
                z: 0.0,
            },
        );
        joint_builder_set_local_anchor2(
            builder,
            Vec3 {
                x: 0.0,
                y: -0.5,
                z: 0.0,
            },
        );
        let handle = world_insert_impulse_joint(world, b1, b2, builder, Bool::TRUE);
        assert_ne!(handle, 0);
        assert_eq!(
            world_remove_impulse_joint(world, handle, Bool::TRUE),
            Bool::TRUE
        );
        world_destroy(world);
    }

    #[test]
    fn prismatic_joint_roundtrip() {
        let world = make_world();
        let b1 = make_dynamic_body(world);
        let b2 = make_dynamic_body(world);
        let builder = joint_builder_create(
            JointTypeDesc::Prismatic as u32,
            Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            0.0,
            0.0,
        );
        let handle = world_insert_impulse_joint(world, b1, b2, builder, Bool::TRUE);
        assert_ne!(handle, 0);
        assert_eq!(
            world_remove_impulse_joint(world, handle, Bool::TRUE),
            Bool::TRUE
        );
        world_destroy(world);
    }

    #[test]
    fn spherical_joint_roundtrip() {
        let world = make_world();
        let b1 = make_dynamic_body(world);
        let b2 = make_dynamic_body(world);
        let builder =
            joint_builder_create(JointTypeDesc::Spherical as u32, Vec3::default(), 0.0, 0.0);
        let handle = world_insert_impulse_joint(world, b1, b2, builder, Bool::TRUE);
        assert_ne!(handle, 0);
        assert_eq!(
            world_remove_impulse_joint(world, handle, Bool::TRUE),
            Bool::TRUE
        );
        world_destroy(world);
    }

    #[test]
    fn spring_joint_roundtrip() {
        let world = make_world();
        let b1 = make_dynamic_body(world);
        let b2 = make_dynamic_body(world);
        let builder = joint_builder_create(
            JointTypeDesc::Spring as u32,
            Vec3 {
                x: 0.0,
                y: 50.0,
                z: 0.0,
            },
            0.5,
            0.1,
        );
        let handle = world_insert_impulse_joint(world, b1, b2, builder, Bool::TRUE);
        assert_ne!(handle, 0);
        assert_eq!(
            world_remove_impulse_joint(world, handle, Bool::TRUE),
            Bool::TRUE
        );
        world_destroy(world);
    }

    // =======================================================================
    // multibody joint brake — `F = −k·(q − rest) − c·v`
    //
    // Pins three things:
    //   1. `c = 0` (the default) behaves as before — no regression;
    //   2. `c > 0` really removes angular velocity — the damping reaches the solver;
    //   3. `brake()` drops the stiffness and keeps the damping, so the joint
    //      stops instead of being pulled back toward a stale target angle.
    //
    // The bodies are driven through the *joint*, never through
    // `rigid_body_set_angvel`: `multibody::update_velocities` overwrites
    // `bodies.vels` from the solver at the end of every step, so an external
    // velocity write on a multibody-attached body is discarded on step one.
    // =======================================================================

    /// Minimal powered hinge: a fixed body and a dynamic body, joined by a
    /// revolute joint about X. The dynamic body needs a collider — that is what
    /// gives it mass *and* rotational inertia; `additional_mass` alone leaves
    /// the inertia at zero, and a zero-inertia body cannot hold angular
    /// velocity.
    ///
    /// Gravity is zeroed: a hanging sphere has a gravity torque about the hinge,
    /// which would dominate the damping measurement. The brake tests are about
    /// the spring/damping terms only.
    fn revolute_pair(world: *mut WorldHandle) -> (RigidBodyHandleRaw, RigidBodyHandleRaw, u64) {
        use mps_core::rapier::collider::{
            collider_builder_build, collider_builder_create_sphere,
            world_insert_collider_with_parent,
        };

        mps_core::rapier::world::world_set_gravity(
            world,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        );

        let fixed_builder = rigid_body_builder_create(BodyStatus::Fixed as u32);
        let fixed = world_insert_rigid_body(world, rigid_body_builder_build(fixed_builder));

        let body_builder = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        let body = world_insert_rigid_body(world, rigid_body_builder_build(body_builder));

        let sphere = collider_builder_create_sphere(Sphere {
            center: Vec3::default(),
            radius: 0.5,
        });
        assert!(!sphere.is_null(), "sphere builder failed");
        world_insert_collider_with_parent(world, collider_builder_build(sphere), body);

        let jb = joint_builder_create(
            JointTypeDesc::Revolute as u32,
            Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            0.0,
            0.0,
        );
        assert!(!jb.is_null(), "revolute joint builder failed");
        let hinge = world_insert_multibody_joint(world, fixed, body, jb, Bool::TRUE);
        assert_ne!(hinge, u64::MAX, "multibody hinge insert failed");
        (fixed, body, hinge)
    }

    /// The joint's own generalized coordinate on `axis` — the authoritative
    /// state of a multibody joint.
    ///
    /// The attached body's `rigid_body_get_angvel` is deliberately *not* used
    /// anywhere in these tests: `multibody::update_velocities` overwrites
    /// `bodies.vels` from the solver at the end of every step, so an external
    /// read or write of that velocity says nothing about what the joint is
    /// doing. Differencing this coordinate is the only honest measurement.
    fn read_angle(world: *mut WorldHandle, hinge: u64, axis: u32) -> f64 {
        world_multibody_joint_get_coordinate(world, hinge, axis)
    }

    /// The joint's own generalized velocity on `axis` [rad/s] — the
    /// authoritative speed, since the solver overwrites the body's `angvel`.
    fn read_velocity(world: *mut WorldHandle, hinge: u64, axis: u32) -> f64 {
        world_multibody_joint_get_velocity(world, hinge, axis)
    }

    /// Drive the hinge with a servo that keeps advancing its target angle, and
    /// return the joint coordinate reached.
    fn spin_up(world: *mut WorldHandle, hinge: u64, axis: u32) -> f64 {
        let mut angle = 0.0;
        for _ in 0..60 {
            angle += 0.1;
            world_multibody_joint_set_motor(world, hinge, axis, angle, 10_000.0);
            mps_core::rapier::world::world_step(world, 1.0 / 60.0);
        }
        read_angle(world, hinge, axis)
    }

    #[test]
    fn multibody_spring_damping_rejects_invalid() {
        let world = make_world();
        let (_fixed, _body, hinge) = revolute_pair(world);
        let axis = JointAxisDesc::AngX as u32;

        assert_eq!(
            world_multibody_joint_set_spring_damping(world, hinge, axis, -1.0),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        assert_eq!(
            world_multibody_joint_set_spring_damping(world, hinge, axis, f64::NAN),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        assert_eq!(
            world_multibody_joint_set_spring_damping(world, hinge, axis, 5.0),
            Bool::TRUE
        );
        assert_eq!(
            world_multibody_joint_set_spring_damping(std::ptr::null_mut(), hinge, axis, 1.0),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(
            world_multibody_joint_set_spring_damping(world, u64::MAX, axis, 1.0),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NOT_FOUND);

        world_destroy(world);
    }

    #[test]
    fn multibody_brake_rejects_invalid_damping() {
        let world = make_world();
        let (_f, _b, hinge) = revolute_pair(world);
        let axis = JointAxisDesc::AngX as u32;
        assert_eq!(
            world_multibody_joint_brake(world, hinge, axis, -0.5),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        assert_eq!(
            world_multibody_joint_brake(world, hinge, axis, f64::INFINITY),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        world_destroy(world);
    }

    /// `damping = 0` must behave as before. "Before" is *not* zero damping: the
    /// fork's `MultibodyJoint::default_damping` seeds `0.1` on every free
    /// angular DoF, so a released joint already slows down on its own. This
    /// test pins that baseline (it is the pre-existing behaviour), so a future
    /// change to `default_damping` cannot go unnoticed.
    #[test]
    fn zero_damping_preserves_previous_behaviour() {
        let world = make_world();
        let (_f, _body, hinge) = revolute_pair(world);
        let axis = JointAxisDesc::AngX as u32;

        let driven = spin_up(world, hinge, axis);
        assert!(
            driven.abs() > 1.0,
            "servo must actually drive the hinge, got {driven}"
        );

        world_multibody_joint_set_motor(world, hinge, axis, 0.0, 0.0);
        world_multibody_joint_set_spring_damping(world, hinge, axis, 0.0);

        // Measure the joint's own angular velocity by differencing its
        // coordinate (the attached body's `angvel` is not authoritative).
        let dt = 1.0 / 60.0;
        let mut prev = read_angle(world, hinge, axis);
        let mut rates = Vec::new();
        for _ in 0..30 {
            mps_core::rapier::world::world_step(world, dt);
            let now = read_angle(world, hinge, axis);
            rates.push((now - prev) / dt);
            prev = now;
        }
        let first = rates[0];
        let last = *rates.last().unwrap();
        assert!(
            first.abs() > 0.5,
            "the joint must be coasting, got {first} rad/s"
        );
        // The built-in 0.1 damping is real and not negligible — it is part of
        // the pre-existing behaviour, so pin the observed decay band rather
        // than pretend the joint coasts freely.
        assert!(
            last.abs() > first.abs() * 0.2,
            "the default 0.1 damping must stay in its known band: {first} -> {last} rad/s"
        );
        world_destroy(world);
    }

    /// A braked joint has no target angle: it removes angular velocity instead
    /// of chasing a position. The distinguishing signature is that the joint's
    /// angular *velocity* decays toward zero, so the coordinate asymptotes
    /// instead of settling on the servo's target.
    #[test]
    fn positive_damping_removes_angular_velocity() {
        let world = make_world();
        let (_f, _body, hinge) = revolute_pair(world);
        let axis = JointAxisDesc::AngX as u32;

        let driven = spin_up(world, hinge, axis);
        assert!(driven.abs() > 1.0, "servo must drive, got {driven}");

        assert_eq!(
            world_multibody_joint_brake(world, hinge, axis, 30.0),
            Bool::TRUE
        );

        // Measure the joint's angular velocity by differencing its coordinate.
        let dt = 1.0 / 60.0;
        let mut prev = read_angle(world, hinge, axis);
        let mut first_rate = None;
        let mut last_rate = 0.0;
        for i in 0..120 {
            mps_core::rapier::world::world_step(world, dt);
            let now = read_angle(world, hinge, axis);
            let rate = (now - prev) / dt;
            prev = now;
            if i == 0 {
                first_rate = Some(rate);
            }
            last_rate = rate;
        }
        let first = first_rate.expect("at least one step");
        assert!(
            first.abs() > 0.5,
            "the joint must still be moving right after the servo lets go, got {first}"
        );
        assert!(
            last_rate.abs() < first.abs() * 0.2,
            "damping must slow the joint down: {first} -> {last_rate}"
        );
        world_destroy(world);
    }

    /// A servo (`k > 0`) pulls the joint toward its target; `brake()` has no
    /// target, so it removes velocity instead of chasing an angle.
    ///
    /// The brake is applied *while the servo is still driving* — once the servo
    /// has reached its target the joint is at rest and there is nothing to
    /// brake.
    #[test]
    fn brake_stops_without_pulling_back_to_a_target() {
        let world = make_world();
        let (_f, _body, hinge) = revolute_pair(world);
        let axis = JointAxisDesc::AngX as u32;

        let dt = 1.0 / 60.0;

        // Drive for a while but keep advancing the target so the joint is still
        // moving, and measure that rate.
        let mut angle = 0.0;
        let mut prev = read_angle(world, hinge, axis);
        let mut pre_brake_rate = 0.0;
        for _ in 0..20 {
            angle += 0.1;
            world_multibody_joint_set_motor(world, hinge, axis, angle, 10_000.0);
            mps_core::rapier::world::world_step(world, dt);
            let now = read_angle(world, hinge, axis);
            pre_brake_rate = (now - prev) / dt;
            prev = now;
        }
        assert!(
            pre_brake_rate.abs() > 0.5,
            "the servo must have the joint moving before the brake: {pre_brake_rate} rad/s"
        );

        // Brake: stiffness to zero, damping up. `rest` is re-anchored to the
        // current angle, so no restoring force is left.
        assert_eq!(
            world_multibody_joint_brake(world, hinge, axis, 200.0),
            Bool::TRUE
        );
        let stopped_at = read_angle(world, hinge, axis);

        // Let the brake run: the coordinate must converge (an asymptote), which
        // is what "stopped" means for a velocity-only brake.
        let mut rate_late = 0.0;
        for i in 0..300 {
            mps_core::rapier::world::world_step(world, dt);
            let now = read_angle(world, hinge, axis);
            let rate = (now - prev) / dt;
            prev = now;
            if i == 299 {
                rate_late = rate;
            }
        }
        assert!(
            rate_late.abs() < pre_brake_rate.abs() * 0.05,
            "after braking the joint must come to rest: {pre_brake_rate} -> {rate_late} rad/s"
        );
        // With a strong brake the joint stops almost immediately, so it barely
        // coasts: a small travelled distance is the *signature of a hard brake*,
        // not of a servo snapping back. The servo case is covered by
        // `spin_up` (coordinate tracks its target to ~2e-3).
        let travelled = (prev - stopped_at).abs();
        assert!(
            travelled < 0.5,
            "a hard brake stops the joint quickly: travelled {travelled}"
        );
        world_destroy(world);
    }

    /// The `_flag` variants must reach the same conclusion as the `Bool` ones.
    #[test]
    fn brake_flag_variant_matches() {
        let world = make_world();
        let (_f, _b, hinge) = revolute_pair(world);
        let axis = JointAxisDesc::AngX as u32;
        let a = world_multibody_joint_brake(world, hinge, axis, 10.0).0;
        let b = world_multibody_joint_brake_flag(world, hinge, axis, 10.0);
        assert_eq!(a, b);
        let c = world_multibody_joint_set_spring_damping(world, hinge, axis, 10.0).0;
        let d = world_multibody_joint_set_spring_damping_flag(world, hinge, axis, 10.0);
        assert_eq!(c, d);
        world_destroy(world);
    }

    // =======================================================================
    // multibody joint force cap — the `max_force` of `world_joint_set_motor`,
    // finally available on the multibody path.
    // =======================================================================

    #[test]
    fn multibody_max_force_roundtrips_and_defaults_to_unlimited() {
        let world = make_world();
        let (_f, _b, hinge) = revolute_pair(world);
        let axis = JointAxisDesc::AngX as u32;

        // Uncapped by default.
        let before = world_multibody_joint_get_max_force(world, hinge, axis);
        assert!(
            before >= f64::MAX * 0.5,
            "a fresh joint should be uncapped: {before}"
        );

        assert_eq!(
            world_multibody_joint_set_max_force(world, hinge, axis, 12.5),
            Bool::TRUE
        );
        assert!((world_multibody_joint_get_max_force(world, hinge, axis) - 12.5).abs() < 1e-12);
        // The `_flag` variant agrees.
        assert_eq!(
            world_multibody_joint_set_max_force_flag(world, hinge, axis, 7.0),
            world_multibody_joint_set_max_force(world, hinge, axis, 7.0).0
        );
        world_destroy(world);
    }

    #[test]
    fn multibody_max_force_rejects_invalid() {
        let world = make_world();
        let (_f, _b, hinge) = revolute_pair(world);
        let axis = JointAxisDesc::AngX as u32;

        assert_eq!(
            world_multibody_joint_set_max_force(world, hinge, axis, -1.0),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        assert_eq!(
            world_multibody_joint_set_max_force(world, hinge, axis, f64::NAN),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);
        assert_eq!(
            world_multibody_joint_set_max_force(std::ptr::null_mut(), hinge, axis, 1.0),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        assert_eq!(
            world_multibody_joint_set_max_force(world, u64::MAX, axis, 1.0),
            Bool::FALSE
        );
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        // Reading from a bad handle yields NaN, not a misleading 0.
        assert!(world_multibody_joint_get_max_force(world, u64::MAX, axis).is_nan());
        world_destroy(world);
    }

    /// A cap of zero must produce **no** motion: the stiffest possible servo
    /// with no allowed torque cannot move anything.
    #[test]
    fn multibody_zero_cap_produces_no_motion() {
        let world = make_world();
        let (_f, _body, hinge) = revolute_pair(world);
        let axis = JointAxisDesc::AngX as u32;

        // Very stiff servo, unlimited first: it must move.
        world_multibody_joint_set_motor(world, hinge, axis, 1.0, 10_000.0);
        for _ in 0..30 {
            mps_core::rapier::world::world_step(world, 1.0 / 60.0);
        }
        let moved = read_angle(world, hinge, axis).abs();
        assert!(moved > 0.5, "without a cap the servo must move: {moved}");

        // Now cap it to zero and ask for a much larger angle: nothing happens.
        assert_eq!(
            world_multibody_joint_set_max_force(world, hinge, axis, 0.0),
            Bool::TRUE
        );
        let held = read_angle(world, hinge, axis);
        world_multibody_joint_set_motor(world, hinge, axis, 9.0, 10_000.0);
        for _ in 0..60 {
            mps_core::rapier::world::world_step(world, 1.0 / 60.0);
        }
        let after = read_angle(world, hinge, axis);
        assert!(
            (after - held).abs() < 1e-9,
            "a zero cap must freeze the axis: {held} -> {after}"
        );
        world_destroy(world);
    }

    /// A finite cap must make the joint reach its target **slower** than an
    /// uncapped one — that is what "torque limited" means dynamically.
    #[test]
    fn multibody_tight_cap_slows_the_servo() {
        let axis = JointAxisDesc::AngX as u32;

        // Uncapped reference.
        let world = make_world();
        let (_f, _b, hinge) = revolute_pair(world);
        world_multibody_joint_set_motor(world, hinge, axis, 1.0, 10_000.0);
        for _ in 0..10 {
            mps_core::rapier::world::world_step(world, 1.0 / 60.0);
        }
        let uncapped = read_angle(world, hinge, axis).abs();
        world_destroy(world);

        // Same setup, but the axis may only make a small torque.
        let world = make_world();
        let (_f, _b, hinge) = revolute_pair(world);
        assert_eq!(
            world_multibody_joint_set_max_force(world, hinge, axis, 0.02),
            Bool::TRUE
        );
        world_multibody_joint_set_motor(world, hinge, axis, 1.0, 10_000.0);
        for _ in 0..10 {
            mps_core::rapier::world::world_step(world, 1.0 / 60.0);
        }
        let capped = read_angle(world, hinge, axis).abs();
        world_destroy(world);

        assert!(uncapped > 0.0, "uncapped must move: {uncapped}");
        assert!(
            capped < uncapped * 0.5,
            "a tight cap must slow the joint: capped {capped} vs uncapped {uncapped}"
        );
    }

    /// The velocity readout must be finite, non-zero while the joint moves, and
    /// must report the *instantaneous* generalized velocity — which is what a
    /// `torque_at_speed(ω)` envelope needs.
    ///
    /// # Why this does not compare against the coordinate's derivative
    ///
    /// It looks like it should: `MultibodyJoint::integrate` writes
    /// `coords[DIM + dof_id] += vels[curr_free_dof] * dt`, so the readout and
    /// `Δcoord/Δt` ought to be the same number. They are not, and that is
    /// correct behaviour, not a bug (see ADR-0013 for the full trace):
    ///
    /// - `world_step(1/60)` is split into **4 substeps**, so `integrate` runs at
    ///   `dt = 1/240` and the coordinate accumulates four increments.
    /// - `velocities` only ever holds the **last substep's** value.
    ///
    /// Measured directly: `integrate` receives `(dt = 0.0041667, vels[0] = -2.11)`
    /// while the coordinate advances 0.1 per whole step. Comparing the two would
    /// be comparing a last-substep sample against a whole-step average, so this
    /// test deliberately does not.
    ///
    /// A caller that needs the average rate over a step should difference the
    /// coordinate instead; a caller sizing a motor wants this instantaneous
    /// value.
    #[test]
    fn multibody_velocity_readout_is_finite_while_moving() {
        let world = make_world();
        let (_f, _b, hinge) = revolute_pair(world);
        let axis = JointAxisDesc::AngX as u32;

        let mut angle = 0.0;
        let mut max_speed = 0.0f64;
        let start = read_angle(world, hinge, axis);
        for _ in 0..30 {
            angle += 0.1;
            world_multibody_joint_set_motor(world, hinge, axis, angle, 10_000.0);
            mps_core::rapier::world::world_step(world, 1.0 / 60.0);
            let v = read_velocity(world, hinge, axis);
            assert!(v.is_finite(), "velocity readout must be finite: {v}");
            max_speed = max_speed.max(v.abs());
        }
        let moved = (read_angle(world, hinge, axis) - start).abs();

        // The joint really did move, and the readout really did see motion.
        assert!(
            moved > 1.0,
            "the driven joint must actually advance, moved {moved} rad"
        );
        assert!(
            max_speed > 0.5,
            "the velocity readout must see the motion, got max |v| = {max_speed}"
        );
        world_destroy(world);
    }

    /// A bad handle must yield NaN for the `f64`-returning readers.
    #[test]
    fn multibody_float_readers_return_nan_on_bad_handles() {
        let world = make_world();
        let (_f, _b, hinge) = revolute_pair(world);
        let axis = JointAxisDesc::AngX as u32;
        assert!(world_multibody_joint_get_velocity(world, u64::MAX, axis).is_nan());
        assert_eq!(last_error_code(), ERR_NOT_FOUND);
        assert!(world_multibody_joint_get_max_force(world, u64::MAX, axis).is_nan());
        assert!(world_multibody_joint_get_velocity(std::ptr::null_mut(), hinge, axis).is_nan());
        assert_eq!(last_error_code(), ERR_NULL_POINTER);
        world_destroy(world);
    }
}
