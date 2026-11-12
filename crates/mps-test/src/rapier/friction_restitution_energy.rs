//! Real-physics validation: friction & restitution energetics through the FFI
//! surface only (no production-code changes). Three scenarios:
//!
//! 1. Coulomb friction cone on an inclined plane: critical angle tanθ_c = μ;
//!    steady-state sliding acceleration g(sinθ − μcosθ).
//! 2. Restitution energy ratio: rebound/impact speed ratio ≈ e (height ratio
//!    ≈ e²) with Max combine rule on both contacts.
//! 3. Static friction hold: horizontal push below μ·m·g produces zero drift;
//!    above the threshold the body slides.

#[cfg(test)]
mod tests {
    use mps_core::rapier::collider::{
        collider_builder_build, collider_builder_create_ex, collider_builder_set_friction,
        collider_builder_set_restitution, collider_set_restitution_combine_rule,
        world_insert_collider_with_parent,
    };
    use mps_core::rapier::ffi::{BodyStatus, Bool, RigidBodyHandleRaw, ShapeDesc, ShapeType, Vec3};
    use mps_core::rapier::rigid_body::{
        rigid_body_add_force, rigid_body_builder_build, rigid_body_builder_create,
        rigid_body_builder_set_additional_mass, rigid_body_builder_set_enabled_rotations,
        rigid_body_builder_set_rotation, rigid_body_builder_set_translation, rigid_body_get_linvel,
        rigid_body_get_translation, world_insert_rigid_body,
    };
    use mps_core::rapier::world::{
        world_apply_runtime_settings, world_create, world_destroy, world_step,
    };

    const G: f64 = 9.81;
    const DT: f64 = 1.0 / 60.0;
    /// Max combine rule discriminator understood by
    /// `collider_set_restitution_combine_rule` (3 => CoefficientCombineRule::Max).
    const COMBINE_MAX: u32 = 3;

    fn make_world() -> *mut mps_core::rapier::ffi::WorldHandle {
        world_create(Vec3 {
            x: 0.0,
            y: -G,
            z: 0.0,
        })
    }

    fn v(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3 { x, y, z }
    }

    fn cuboid(half_x: f64, half_y: f64, half_z: f64) -> ShapeDesc {
        ShapeDesc {
            shape_type: ShapeType::Cuboid as u32,
            a: half_x,
            b: half_y,
            c: half_z,
            ..Default::default()
        }
    }

    fn ball(radius: f64) -> ShapeDesc {
        ShapeDesc {
            shape_type: ShapeType::Ball as u32,
            a: radius,
            ..Default::default()
        }
    }

    /// Fixed body with one cuboid collider; friction applied to the collider.
    fn fixed_cuboid(
        world: *mut mps_core::rapier::ffi::WorldHandle,
        center: Vec3,
        z_rotation: f64,
        shape: ShapeDesc,
        friction: f64,
        restitution: f64,
        max_combine: bool,
    ) -> (RigidBodyHandleRaw, u64) {
        let builder = rigid_body_builder_create(BodyStatus::Fixed as u32);
        rigid_body_builder_set_translation(builder, center);
        rigid_body_builder_set_rotation(builder, v(0.0, 0.0, z_rotation));
        let body = world_insert_rigid_body(world, rigid_body_builder_build(builder));
        let cb = collider_builder_create_ex(shape);
        collider_builder_set_friction(cb, friction);
        collider_builder_set_restitution(cb, restitution);
        let collider_handle =
            world_insert_collider_with_parent(world, collider_builder_build(cb), body);
        if max_combine {
            assert_eq!(
                collider_set_restitution_combine_rule(world, collider_handle, COMBINE_MAX),
                Bool::TRUE
            );
        }
        (body, collider_handle)
    }

    /// Dynamic cuboid with translations locked (no tumbling) and an explicit
    /// additional mass so the mass used in the friction threshold is known.
    fn sliding_block(
        world: *mut mps_core::rapier::ffi::WorldHandle,
        center: Vec3,
        z_rotation: f64,
        shape: ShapeDesc,
        friction: f64,
        restitution: f64,
    ) -> RigidBodyHandleRaw {
        let builder = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_translation(builder, center);
        rigid_body_builder_set_rotation(builder, v(0.0, 0.0, z_rotation));
        rigid_body_builder_set_enabled_rotations(builder, Bool::FALSE, Bool::FALSE, Bool::FALSE);
        let body = world_insert_rigid_body(world, rigid_body_builder_build(builder));
        let cb = collider_builder_create_ex(shape);
        collider_builder_set_friction(cb, friction);
        collider_builder_set_restitution(cb, restitution);
        world_insert_collider_with_parent(world, collider_builder_build(cb), body);
        body
    }

    /// 1a. θ well below arctan(μ): the block must not move (Coulomb cone).
    #[test]
    fn friction_cone_incline_static_below_critical_angle() {
        let mu: f64 = 0.4;
        // θ = 0.5 · arctan(μ) → tanθ ≈ 0.192 < μ.
        let theta = 0.5 * mu.atan();
        let world = make_world();

        // Ramp: long cuboid rotated by θ about z; its top face is the slope.
        let (ramp, _c) = fixed_cuboid(
            world,
            v(0.0, 0.0, 0.0),
            theta,
            cuboid(10.0, 0.2, 1.0),
            mu,
            0.0,
            false,
        );
        assert_ne!(ramp, 0);

        // Block placed on the slope surface, up-slope from the pivot.
        let (u_x, u_y) = (theta.cos(), theta.sin());
        let (n_x, n_y) = (-theta.sin(), theta.cos());
        let s0 = 2.0; // up-slope offset along the surface
        let half_block = 0.1;
        let start = v(
            s0 * u_x + 0.2 * n_x,
            s0 * u_y + (0.2 + half_block) * n_y,
            0.0,
        );
        let block = sliding_block(
            world,
            start,
            theta,
            cuboid(half_block, half_block, half_block),
            mu,
            0.0,
        );

        // Prime the broad phase, settle 100 steps (initial contact resolution),
        // then measure the drift over the next 100 steps only.
        world_step(world, DT);
        for _ in 0..100 {
            world_step(world, DT);
        }
        let settled = rigid_body_get_translation(world, block);
        for _ in 0..100 {
            world_step(world, DT);
        }

        let p = rigid_body_get_translation(world, block);
        let drift = ((p.x - settled.x).powi(2) + (p.y - settled.y).powi(2)).sqrt();
        let total = ((p.x - start.x).powi(2) + (p.y - start.y).powi(2)).sqrt();
        println!(
            "FRICTION_CONE_STATIC mu={mu} theta_deg={:.2} drift_m={drift:.6} total_settle_m={total:.6}",
            theta.to_degrees()
        );
        assert!(
            drift < 1.0e-3,
            "block must hold below critical angle: drift {drift} m"
        );
        world_destroy(world);
    }

    /// 1b. θ = 1.5 · arctan(μ): block slides with a ≈ g(sinθ − μcosθ) (±10%).
    #[test]
    fn friction_cone_incline_slides_at_predicted_acceleration() {
        let mu: f64 = 0.4;
        let theta = 1.5 * mu.atan(); // tanθ ≈ 0.639 > μ
        let world = make_world();

        let (ramp, _c) = fixed_cuboid(
            world,
            v(0.0, 0.0, 0.0),
            theta,
            cuboid(10.0, 0.2, 1.0),
            mu,
            0.0,
            false,
        );
        assert_ne!(ramp, 0);

        let half_block = 0.1;
        let s0 = 3.0;
        let start = v(
            s0 * theta.cos() + 0.2 * (-theta.sin()),
            s0 * theta.sin() + (0.2 + half_block) * theta.cos(),
            0.0,
        );
        let block = sliding_block(
            world,
            start,
            theta,
            cuboid(half_block, half_block, half_block),
            mu,
            0.0,
        );

        world_step(world, DT);
        // 100 settle steps, then measure Δv over the next 100 steps.
        for _ in 0..100 {
            world_step(world, DT);
        }
        let v1 = rigid_body_get_linvel(world, block);
        for _ in 0..100 {
            world_step(world, DT);
        }
        let v2 = rigid_body_get_linvel(world, block);
        let p = rigid_body_get_translation(world, block);

        // Project onto the slope direction; downhill is −u.
        let u = [theta.cos(), theta.sin()];
        let down1 = -(v1.x * u[0] + v1.y * u[1]);
        let down2 = -(v2.x * u[0] + v2.y * u[1]);
        let measured = (down2 - down1) / (100.0 * DT);
        let expected = G * (theta.sin() - mu * theta.cos());
        let rel_err = ((measured - expected) / expected).abs();
        println!(
            "FRICTION_CONE_SLIDE theta_deg={:.2} expected_a={expected:.4} measured_a={measured:.4} rel_err={rel_err:.3} drift_m={:.3} v_down1={down1:.3} v_down2={down2:.3}",
            theta.to_degrees(),
            ((p.x - start.x).powi(2) + (p.y - start.y).powi(2)).sqrt()
        );
        assert!(
            rel_err < 0.10,
            "sliding acceleration off: {measured} vs {expected}"
        );
        assert!(down2 > 1.0, "block must actually slide: v={down2}");
        world_destroy(world);
    }

    /// 2. Rebound/impact speed ratio ≈ e with Max combine on both colliders
    ///    (height ratio then ≈ e²). Speed ratio is the stabler discretization.
    #[test]
    fn restitution_bounce_speed_ratio_matches_e() {
        let e: f64 = 0.6;
        let world = make_world();

        // Floor: top surface at y = 0.
        let (floor, _cf) = fixed_cuboid(
            world,
            v(0.0, -0.5, 0.0),
            0.0,
            cuboid(5.0, 0.5, 5.0),
            0.5,
            e,
            true,
        );
        assert_ne!(floor, 0);

        // Ball dropped from center height 1.0 (0.8 m of fall).
        let radius = 0.2;
        let builder = rigid_body_builder_create(BodyStatus::Dynamic as u32);
        rigid_body_builder_set_translation(builder, v(0.0, 1.0, 0.0));
        rigid_body_builder_set_enabled_rotations(builder, Bool::FALSE, Bool::FALSE, Bool::FALSE);
        let ball_body = world_insert_rigid_body(world, rigid_body_builder_build(builder));
        let cb = collider_builder_create_ex(ball(radius));
        collider_builder_set_friction(cb, 0.5);
        collider_builder_set_restitution(cb, e);
        let ch = world_insert_collider_with_parent(world, collider_builder_build(cb), ball_body);
        assert_eq!(
            collider_set_restitution_combine_rule(world, ch, COMBINE_MAX),
            Bool::TRUE
        );

        // Fine time resolution for clean peak sampling.
        let dt = 1.0 / 240.0;
        world_step(world, dt);
        let mut v_impact = 0.0_f64; // most negative vy before the reversal
        let mut v_rebound = 0.0_f64; // max vy after the reversal
        let mut bounced = false;
        let mut prev_vy = 0.0_f64;
        for _ in 0..600 {
            world_step(world, dt);
            let vel = rigid_body_get_linvel(world, ball_body);
            if !bounced {
                if vel.y < 0.0 {
                    v_impact = v_impact.min(vel.y);
                } else if prev_vy < 0.0 && vel.y >= 0.0 {
                    bounced = true;
                }
            } else {
                v_rebound = v_rebound.max(vel.y);
            }
            prev_vy = vel.y;
        }
        let ratio = v_rebound / v_impact.abs();
        println!(
            "RESTITUTION e={e} v_impact={:.4} v_rebound={:.4} speed_ratio={ratio:.4} (height_ratio={:.4}, e^2={:.4})",
            v_impact.abs(),
            v_rebound,
            ratio,
            ratio * ratio,
        );
        assert!(bounced, "ball never bounced");
        assert!(
            (ratio - e).abs() <= 0.1 * e,
            "rebound/impact speed ratio {ratio} deviates from e={e} by more than 10%"
        );
        world_destroy(world);
    }

    /// 3. Static friction hold: F < μ·m·g keeps the block pinned; F above the
    ///    threshold starts motion.
    #[test]
    fn static_friction_holds_below_threshold_then_slides() {
        let mu: f64 = 0.5;
        // Block: 0.5³ cuboid, density 1.0 → 0.125 kg, plus 2.0 kg additional
        // mass → m ≈ 2.125 kg. Threshold μ·m·g ≈ 10.4 N.
        let mass_est = 2.0 + 0.125;
        let threshold = mu * mass_est * G;

        let run = |push_factor: f64| -> (f64, f64) {
            let w = make_world();
            // Crank the solver: contact friction under a sustained push needs
            // more iterations than the default 4 to converge to full stick.
            assert_eq!(
                world_apply_runtime_settings(w, 64, 4, 0, 0, 1, 1),
                Bool::TRUE
            );
            let (_floor, _cf) = fixed_cuboid(
                w,
                v(0.0, -0.5, 0.0),
                0.0,
                cuboid(20.0, 0.5, 20.0),
                mu,
                0.0,
                false,
            );
            let half = 0.25;
            let builder = rigid_body_builder_create(BodyStatus::Dynamic as u32);
            rigid_body_builder_set_translation(builder, v(0.0, half, 0.0));
            rigid_body_builder_set_enabled_rotations(
                builder,
                Bool::FALSE,
                Bool::FALSE,
                Bool::FALSE,
            );
            rigid_body_builder_set_additional_mass(builder, 2.0);
            let body = world_insert_rigid_body(w, rigid_body_builder_build(builder));
            let cb = collider_builder_create_ex(cuboid(half, half, half));
            collider_builder_set_friction(cb, mu);
            collider_builder_set_restitution(cb, 0.0);
            world_insert_collider_with_parent(w, collider_builder_build(cb), body);

            let force = push_factor * threshold;
            // Settle onto the floor first (drop-in compression), then push.
            world_step(w, DT);
            for _ in 0..20 {
                world_step(w, DT);
            }
            let settled = rigid_body_get_translation(w, body);
            let mut trace = Vec::new();
            for i in 0..200 {
                assert_eq!(
                    rigid_body_add_force(w, body, v(force, 0.0, 0.0), Bool::TRUE),
                    Bool::TRUE
                );
                world_step(w, DT);
                if (i + 1) % 40 == 0 {
                    let p = rigid_body_get_translation(w, body);
                    let vl = rigid_body_get_linvel(w, body);
                    trace.push(format!(
                        "step={} dx={:.6} vx={:.4}",
                        i + 1,
                        p.x - settled.x,
                        vl.x
                    ));
                }
            }
            eprintln!("STATIC_TRACE force={force:.3} [{}]", trace.join("; "));
            let p = rigid_body_get_translation(w, body);
            let vel = rigid_body_get_linvel(w, body);
            world_destroy(w);
            (p.x - settled.x, vel.x)
        };

        let (dx_low, vx_low) = run(0.3);
        println!(
            "STATIC_FRICTION threshold_N={threshold:.3} sub_threshold_force_N={:.3} drift_m={dx_low:.6} vx={vx_low:.4}",
            0.3 * threshold
        );
        assert!(
            dx_low.abs() < 1.0e-3,
            "block must stay pinned below μ·m·g: drift {dx_low} m"
        );

        let (dx_high, vx_high) = run(3.0);
        println!(
            "STATIC_FRICTION super_threshold_force_N={:.3} drift_m={dx_high:.3} vx={vx_high:.3}",
            3.0 * threshold
        );
        assert!(
            dx_high > 0.5,
            "block must slide above μ·m·g: drift {dx_high} m"
        );
    }
}
