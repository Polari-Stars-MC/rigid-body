//! Tests for mps-core's parallel per-frame work (`mps-core::rapier::parallel`).
//!
//! The force-law fills and the snapshot export are order-preserving parallel
//! maps, so above the parallel threshold their results must match a serial
//! reference computed in these tests (to floating-point rounding); below the
//! threshold the code runs serially and must match exactly the same reference.
//! The chunked pairwise-gravity decomposition is additionally checked for
//! run-to-run determinism.

// The module only defines test fixtures + `#[test]` fns; compile to nothing
// outside `cargo test` so the lib target has no dead code.
#![cfg(test)]

use rapier3d::prelude::{ColliderBuilder, RigidBodyBuilder, Vector};

use mps_core::rapier::ffi::{
    AirDragLaw, Bool, ExternalForceLaw, NewtonGravityLaw, PulsarMagneticDipoleLaw, Vec3,
    WorldHandle,
};
use mps_core::rapier::terrain_gravity::lunar_mascon_gravity;

/// Bodies per test world — comfortably above every parallel threshold
/// (`GRAVITY_PAR_MIN_BODIES` = 256 is the largest).
const PARALLEL_N: u32 = 300;
/// Below `PAR_MIN_ITEMS` (128): the serial fast path must produce the same
/// results as the parallel path.
const SERIAL_N: u32 = 100;

const DT: f64 = 0.5;

use mps_core::rapier::collision_mode::WorldCollisionMode as ColliderMode;

#[test]
fn extreme_concurrent_independent_world_steps_remain_finite() {
    let workers = std::thread::available_parallelism()
        .map_or(2, |n| n.get())
        .min(4);
    let rounds = 8;
    let mut joins = Vec::with_capacity(workers);
    for _worker in 0..workers {
        joins.push(std::thread::spawn(move || {
            const EXTREME_BODIES: u32 = 100_000;
            let world = make_world(EXTREME_BODIES, true);
            for _round in 0..rounds {
                mps_core::rapier::world::world_step(world, DT);
                let w = unsafe { &(*world).inner };
                assert!(w.bodies.iter().all(|(_, body)| {
                    let p = body.translation();
                    let v = body.linvel();
                    p.x.is_finite()
                        && p.y.is_finite()
                        && p.z.is_finite()
                        && v.x.is_finite()
                        && v.y.is_finite()
                        && v.z.is_finite()
                }));
            }
            mps_core::rapier::world::world_destroy(world);
        }));
    }
    for join in joins {
        join.join().expect("concurrent world worker panicked");
    }
}

#[test]
#[ignore = "long-running performance benchmark"]
fn million_body_single_world_step_benchmark() {
    const MILLION_BODIES: u32 = 1_000_000;
    let start = std::time::Instant::now();
    let world = make_world(MILLION_BODIES, true);
    let created = start.elapsed();
    let step_start = std::time::Instant::now();
    mps_core::rapier::world::world_step(world, DT);
    let stepped = step_start.elapsed();
    let w = unsafe { &(*world).inner };
    let validate_start = std::time::Instant::now();
    assert_eq!(w.bodies.len(), MILLION_BODIES as usize);
    assert!(
        w.bodies
            .iter()
            .all(|(_, body)| body.translation().x.is_finite())
    );
    let validated = validate_start.elapsed();
    let destroy_start = std::time::Instant::now();
    mps_core::rapier::world::world_destroy(world);
    eprintln!(
        "million body timings: create={created:?}, step={stepped:?}, validate={validated:?}, destroy={:?}, total={:?}",
        destroy_start.elapsed(),
        start.elapsed()
    );
}

#[test]
#[ignore = "long-running performance benchmark"]
fn tiered_body_scale_performance_benchmark() {
    for bodies in [100_000_u32, 500_000, 1_000_000] {
        let total_start = std::time::Instant::now();
        let create_start = std::time::Instant::now();
        let world = make_world(bodies, true);
        let create = create_start.elapsed();
        let step_start = std::time::Instant::now();
        mps_core::rapier::world::world_step(world, DT);
        let step = step_start.elapsed();
        let validate_start = std::time::Instant::now();
        let w = unsafe { &(*world).inner };
        assert_eq!(w.bodies.len(), bodies as usize);
        assert!(w.bodies.iter().all(|(_, body)| {
            let p = body.translation();
            p.x.is_finite() && p.y.is_finite() && p.z.is_finite()
        }));
        let validate = validate_start.elapsed();
        let destroy_start = std::time::Instant::now();
        mps_core::rapier::world::world_destroy(world);
        let destroy = destroy_start.elapsed();
        eprintln!(
            "scale={bodies}: create={create:?}, step={step:?}, validate={validate:?}, destroy={destroy:?}, total={:?}",
            total_start.elapsed()
        );
    }
}

#[test]
#[ignore = "long-running performance benchmark"]
fn collider_mode_performance_benchmark() {
    for mode in [
        ColliderMode::None,
        ColliderMode::Simple,
        ColliderMode::Compound,
    ] {
        let start = std::time::Instant::now();
        let world = make_world_with_colliders(100_000, true, mode);
        let created = start.elapsed();
        let step_start = std::time::Instant::now();
        mps_core::rapier::world::world_step(world, DT);
        let step = step_start.elapsed();
        mps_core::rapier::world::world_destroy(world);
        eprintln!("collider_mode={mode:?}: create={created:?}, step={step:?}");
    }
}

#[test]
#[ignore = "long-running performance benchmark"]
fn dense_contact_solver_performance_benchmark() {
    const BODIES: u32 = 10_000;
    let start = std::time::Instant::now();
    let world = mps_core::rapier::world::world_create(Vec3 {
        x: 0.0,
        y: -9.81,
        z: 0.0,
    });
    let w = unsafe { &mut (*world).inner };
    for i in 0..BODIES {
        let x = (i % 100) as f64 * 0.75;
        let y = (i / 100) as f64 * 0.75;
        let body = w.bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(x, y, 0.0))
                .build(),
        );
        w.colliders.insert_with_parent(
            ColliderBuilder::ball(0.5).density(1.0).build(),
            body,
            &mut w.bodies,
        );
    }
    let created = start.elapsed();
    let step_start = std::time::Instant::now();
    mps_core::rapier::world::world_step(world, DT);
    let step = step_start.elapsed();
    let mut timings = [0.0; 7];
    assert_eq!(
        mps_core::rapier::world::world_get_pipeline_timings(world, timings.as_mut_ptr(), 7),
        7
    );
    eprintln!(
        "dense Rapier pipeline counters (ms): update={:.3}, broad={:.3}, narrow={:.3}, island={:.3}, solver={:.3}, ccd={:.3}, total={:.3}",
        timings[0], timings[1], timings[2], timings[3], timings[4], timings[5], timings[6]
    );
    eprintln!("dense contacts: bodies={BODIES}, create={created:?}, step={step:?}");
    mps_core::rapier::world::world_destroy(world);
}

#[test]
#[ignore = "long-running performance benchmark"]
fn dense_contact_solver_iteration_sweep() {
    const BODIES: u32 = 5_000;
    for iterations in [1_u8, 2, 4, 8] {
        let world = mps_core::rapier::world::world_create(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        });
        let w = unsafe { &mut (*world).inner };
        for i in 0..BODIES {
            let x = (i % 100) as f64 * 0.75;
            let y = (i / 100) as f64 * 0.75;
            let body = w.bodies.insert(
                RigidBodyBuilder::dynamic()
                    .translation(Vector::new(x, y, 0.0))
                    .build(),
            );
            w.colliders.insert_with_parent(
                ColliderBuilder::ball(0.5).density(1.0).build(),
                body,
                &mut w.bodies,
            );
        }
        assert_eq!(
            mps_core::rapier::world::world_set_integration_parameters(
                world,
                DT,
                iterations as u32,
                1
            ),
            Bool::TRUE
        );
        let start = std::time::Instant::now();
        mps_core::rapier::world::world_step(world, DT);
        eprintln!(
            "dense solver iterations={iterations}: step={:?}",
            start.elapsed()
        );
        mps_core::rapier::world::world_destroy(world);
    }
}

#[test]
#[ignore = "long-running performance benchmark"]
fn dense_contact_ccd_substep_sweep() {
    const BODIES: u32 = 5_000;
    for ccd_substeps in [0_u32, 1, 2, 4] {
        let world = mps_core::rapier::world::world_create(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        });
        let w = unsafe { &mut (*world).inner };
        for i in 0..BODIES {
            let body = w.bodies.insert(
                RigidBodyBuilder::dynamic()
                    .translation(Vector::new(
                        (i % 100) as f64 * 0.75,
                        (i / 100) as f64 * 0.75,
                        0.0,
                    ))
                    .build(),
            );
            w.colliders.insert_with_parent(
                ColliderBuilder::ball(0.5).density(1.0).build(),
                body,
                &mut w.bodies,
            );
        }
        assert_eq!(
            mps_core::rapier::world::world_set_integration_parameters(world, DT, 2, ccd_substeps),
            Bool::TRUE
        );
        let start = std::time::Instant::now();
        mps_core::rapier::world::world_step(world, DT);
        eprintln!(
            "dense CCD substeps={ccd_substeps}: step={:?}",
            start.elapsed()
        );
        mps_core::rapier::world::world_destroy(world);
    }
}

/// Deterministic body layout: grid positions in x/y, spread z, and varied
/// velocities. Ball colliders (r = 0.5, density 1) give every dynamic body a
/// positive mass; spacing 2.0 keeps bodies out of contact so only the law
/// under test produces forces.
fn make_world(n_bodies: u32, with_velocity: bool) -> *mut WorldHandle {
    make_world_with_colliders(n_bodies, with_velocity, ColliderMode::Simple)
}

fn make_world_with_colliders(
    n_bodies: u32,
    with_velocity: bool,
    mode: ColliderMode,
) -> *mut WorldHandle {
    use mps_core::rapier::collider::{
        collider_builder_create, collider_builder_create_compound_boxes, collider_builder_destroy,
    };
    use mps_core::rapier::collision_mode::{
        world_create_with_collision_mode, world_insert_default_collider,
    };
    let world = world_create_with_collision_mode(Vec3::default(), mode as u32);
    assert!(!world.is_null());
    let simple = collider_builder_create(
        0,
        Vec3 {
            x: 0.5,
            y: 0.0,
            z: 0.0,
        },
    );
    let boxes = [
        -0.5, -0.25, -0.25, 0.0, 0.25, 0.25, 0.0, -0.25, -0.25, 0.5, 0.25, 0.25,
    ];
    let compound = collider_builder_create_compound_boxes(boxes.as_ptr(), 2);
    assert!(!simple.is_null() && !compound.is_null());
    for i in 0..n_bodies {
        let x = (i % 16) as f64 * 2.0 - 15.0;
        let y = ((i / 16) % 16) as f64 * 2.0 - 15.0;
        let z = i as f64 * 0.75;
        let mut rb = RigidBodyBuilder::dynamic().translation(Vector::new(x, y, z));
        if with_velocity {
            // Spans ~0..60 m/s so the air-drag test crosses its Reynolds
            // regime boundary; body 52 solves i≡3 (mod 7), i≡2 (mod 5),
            // i≡1 (mod 3) and stays exactly at rest (exercises the speed gate).
            let vx = ((i % 7) as f64 - 3.0) * 10.0;
            let vy = ((i % 5) as f64 - 2.0) * 10.0;
            let vz = ((i % 3) as f64 - 1.0) * 20.0;
            rb = rb.linvel(Vector::new(vx, vy, vz));
        }
        if mode == ColliderMode::None {
            rb = rb.additional_mass(1.0);
        }
        let handle = unsafe { (*world).inner.bodies.insert(rb.build()) };
        let collider = world_insert_default_collider(
            world,
            mps_core::rapier::ffi::pack_rigid_body_handle(handle),
            simple,
            compound,
        );
        assert_eq!(collider == 0, mode == ColliderMode::None);
    }
    collider_builder_destroy(simple);
    collider_builder_destroy(compound);
    world
}

/// (handle, pre-step velocity/position, mass) sampled before the step.
fn sample(
    world: *mut WorldHandle,
    pos: bool,
) -> Vec<(rapier3d::prelude::RigidBodyHandle, Vector, f64)> {
    let w = unsafe { &mut (*world).inner };
    w.bodies
        .iter()
        .filter(|(_, b)| b.is_dynamic())
        .map(|(h, b)| (h, if pos { b.translation() } else { b.linvel() }, b.mass()))
        .collect()
}

fn read_linvel(world: *mut WorldHandle, h: rapier3d::prelude::RigidBodyHandle) -> Vector {
    let w = unsafe { &*world };
    w.inner.bodies[h].linvel()
}

fn read_angvel(world: *mut WorldHandle, h: rapier3d::prelude::RigidBodyHandle) -> Vector {
    let w = unsafe { &*world };
    w.inner.bodies[h].angvel()
}

fn assert_close(actual: Vector, expected: Vector, context: &str) {
    let diff = (actual - expected).length();
    let scale = expected.length().max(1.0e-9);
    assert!(
        diff <= 1.0e-9 * scale + 1.0e-12,
        "{context}: got {actual:?}, expected {expected:?} (|Δ| = {diff})"
    );
}

// ---------------------------------------------------------------------------
// Air drag — per-body fill with a max-Reynolds reduction
// ---------------------------------------------------------------------------

fn air_drag_reference(v0: Vector, mass: f64) -> Vector {
    const FLUID_DENSITY: f64 = 1.225;
    const VISCOSITY: f64 = 1.8e-5;
    const CHAR_LEN: f64 = 0.5;
    const REF_AREA: f64 = 0.2;
    const CD: f64 = 0.47;
    const RE_LIMIT: f64 = 1.0e6;

    let speed = v0.length();
    if speed <= 1.0e-12 {
        return v0;
    }
    let reynolds = FLUID_DENSITY * speed * CHAR_LEN / VISCOSITY;
    let drag = if reynolds <= RE_LIMIT {
        3.0 * std::f64::consts::PI * VISCOSITY * CHAR_LEN * speed
    } else {
        0.5 * FLUID_DENSITY * speed * speed * CD * REF_AREA
    };
    let force = -v0 / speed * drag;
    v0 + force / mass * DT
}

fn check_air_drag(n_bodies: u32) {
    let world = make_world(n_bodies, true);
    mps_core::rapier::events::world_set_air_drag_law(
        world,
        AirDragLaw {
            fluid_velocity: Vec3::default(),
            density: 1.225,
            dynamic_viscosity: 1.8e-5,
            characteristic_length: 0.5,
            reference_area: 0.2,
            drag_coefficient: 0.47,
            reynolds_stokes_limit: 1.0e6,
            enabled: Bool::TRUE,
        },
    );
    let pre = sample(world, false);
    mps_core::rapier::world::world_step(world, DT);
    for (h, v0, mass) in &pre {
        let expected = air_drag_reference(*v0, *mass);
        let actual = read_linvel(world, *h);
        assert_close(
            actual,
            expected,
            &format!("air drag n={n_bodies} body {:?}", h.into_raw_parts()),
        );
    }
    mps_core::rapier::world::world_destroy(world);
}

#[test]
fn air_drag_parallel_fill_matches_serial_reference() {
    check_air_drag(PARALLEL_N);
}

#[test]
fn air_drag_serial_path_matches_reference() {
    check_air_drag(SERIAL_N);
}

// ---------------------------------------------------------------------------
// External forces — multi-source per-body fill (buoyancy)
// ---------------------------------------------------------------------------

#[test]
fn external_force_parallel_fill_matches_analytic_reference() {
    let world = make_world(PARALLEL_N, false);
    // Buoyancy only: F = -g·(ρ·V) = (0, +9.81, 0) N on every body.
    mps_core::rapier::events::world_set_external_force_law(
        world,
        ExternalForceLaw {
            buoyancy_enabled: Bool::TRUE,
            fluid_density: 1000.0,
            displaced_volume: 0.001,
            buoyancy_gravity: Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
            enabled: Bool::TRUE,
            ..Default::default()
        },
    );
    let pre = sample(world, false);
    mps_core::rapier::world::world_step(world, DT);
    let buoyancy = Vector::new(0.0, 9.81, 0.0);
    for (h, _v0, mass) in &pre {
        let expected = buoyancy / *mass * DT;
        let actual = read_linvel(world, *h);
        assert_close(
            actual,
            expected,
            &format!("buoyancy body {:?}", h.into_raw_parts()),
        );
    }
    mps_core::rapier::world::world_destroy(world);
}

// ---------------------------------------------------------------------------
// Terrain gravity — heavy per-body sampling (lunar mascon)
// ---------------------------------------------------------------------------

#[test]
fn terrain_gravity_parallel_matches_formula_reference() {
    let world = make_world(PARALLEL_N, false);
    assert!(
        mps_core::rapier::events::world_register_terrain_gravity_mascon(world).0 != 0,
        "mascon terrain law registration failed"
    );
    // Move the grid up to lunar-surface altitudes so the mascon model is in
    // its intended regime.
    let w = unsafe { &mut (*world).inner };
    let radius = 1.7374e6;
    for (k, (_, body)) in w.bodies.iter_mut().enumerate() {
        let theta = k as f64 * 0.13;
        let phi = k as f64 * 0.071;
        let r = radius + (k % 11) as f64 * 5.0e4;
        body.set_translation(
            Vector::new(
                r * theta.cos() * phi.cos(),
                r * theta.sin() * phi.cos(),
                r * phi.sin(),
            ),
            false,
        );
    }
    let positions = sample(world, true);
    mps_core::rapier::world::world_step(world, DT);
    for (h, pos, _mass) in &positions {
        let p = Vec3 {
            x: pos.x,
            y: pos.y,
            z: pos.z,
        };
        let a = lunar_mascon_gravity(p);
        let expected = Vector::new(a.x, a.y, a.z) * DT;
        let actual = read_linvel(world, *h);
        assert_close(
            actual,
            expected,
            &format!("mascon gravity body {:?}", h.into_raw_parts()),
        );
    }
    mps_core::rapier::world::world_destroy(world);
}

// ---------------------------------------------------------------------------
// Newtonian pairwise gravity — chunked parallel O(n²) decomposition
// ---------------------------------------------------------------------------

fn check_pairwise_gravity(n_bodies: u32) {
    let world = make_world(n_bodies, false);
    mps_core::rapier::events::world_set_newton_gravity_law(
        world,
        NewtonGravityLaw {
            gravitational_constant: 1000.0, // game-scale G so the drift is observable
            min_distance: 0.01,
            max_distance: 0.0,
            enabled: Bool::TRUE,
        },
    );
    let pre = sample(world, true);
    let dt = DT;

    // Serial upper-triangle reference (the legacy accumulation order).
    let n = pre.len();
    let mut net = vec![Vector::ZERO; n];
    let g = 1000.0;
    let min_dist = 0.01;
    for i in 0..n {
        for j in (i + 1)..n {
            let (_, pi, mi) = &pre[i];
            let (_, pj, mj) = &pre[j];
            let offset = *pj - *pi;
            let dist_sq = offset.length_squared();
            let dist = dist_sq.sqrt().max(min_dist);
            let f_ij = offset * (g * mi * mj / (dist_sq * dist));
            net[i] += f_ij;
            net[j] -= f_ij;
        }
    }

    mps_core::rapier::world::world_step(world, dt);
    for (i, (h, _p, mass)) in pre.iter().enumerate() {
        let expected = net[i] / *mass * dt;
        let actual = read_linvel(world, *h);
        assert_close(
            actual,
            expected,
            &format!(
                "pairwise gravity n={n_bodies} body {:?}",
                h.into_raw_parts()
            ),
        );
    }
    mps_core::rapier::world::world_destroy(world);
}

#[test]
fn newtonian_gravity_parallel_chunked_matches_serial_reference() {
    check_pairwise_gravity(PARALLEL_N);
}

#[test]
fn newtonian_gravity_serial_path_matches_reference() {
    check_pairwise_gravity(SERIAL_N);
}

#[test]
fn newtonian_gravity_parallel_is_deterministic_across_runs() {
    let world_a = make_world(PARALLEL_N, false);
    let world_b = make_world(PARALLEL_N, false);
    for world in [world_a, world_b] {
        mps_core::rapier::events::world_set_newton_gravity_law(
            world,
            NewtonGravityLaw {
                gravitational_constant: 1000.0,
                min_distance: 0.01,
                max_distance: 0.0,
                enabled: Bool::TRUE,
            },
        );
    }
    mps_core::rapier::world::world_step(world_a, DT);
    mps_core::rapier::world::world_step(world_b, DT);
    let velocities_a = sample(world_a, false);
    for (h, _v, _m) in &velocities_a {
        let va = read_linvel(world_a, *h);
        let vb = read_linvel(world_b, *h);
        assert_close(
            va,
            vb,
            &format!("determinism body {:?}", h.into_raw_parts()),
        );
    }
    mps_core::rapier::world::world_destroy(world_a);
    mps_core::rapier::world::world_destroy(world_b);
}

// ---------------------------------------------------------------------------
// Pulsar magnetic-dipole torque — dual-bucket fill (add_torque + fallback)
// ---------------------------------------------------------------------------

#[test]
fn pulsar_torque_parallel_matches_formula_reference() {
    let world = make_world(PARALLEL_N - 1, false);
    // One collider-less dynamic body exercises the angvel fallback bucket.
    {
        let w = unsafe { &mut (*world).inner };
        let handle = w.bodies.insert(RigidBodyBuilder::dynamic().build());
        let _ = handle; // mass() == 0 → fallback path
    }
    // Spread the bodies along the spin axis so each samples a different |B|.
    {
        let w = unsafe { &mut (*world).inner };
        for (k, (_, body)) in w.bodies.iter_mut().enumerate() {
            body.set_translation(Vector::new(0.0, 0.0, 2.0e4 + k as f64 * 1.0e3), false);
        }
    }
    let pulsar = PulsarMagneticDipoleLaw {
        moment_of_inertia: 1.0e38,
        ns_radius_m: 1.0e4,
        period_ms: 1000.0,
        // Ṗ chosen so the resulting torque yields dω ≈ 0.04 rad/s — rapier
        // clamps the per-step rotation angle (ω·dt ≤ π/4), which would mask a
        // larger torque and break the comparison against the analytic dω.
        period_derivative: 1.0e-38,
        pulsar_position: Vec3::default(),
        spin_axis: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 1.0,
        },
        body_dipole_moment: Vec3 {
            x: 0.0,
            y: 100.0,
            z: 0.0,
        },
        enabled: Bool::TRUE,
    };
    assert!(
        mps_core::rapier::events::world_set_pulsar_magnetic_dipole_law(world, pulsar).0 != 0,
        "pulsar law registration failed"
    );

    let b_surf =
        mps_formula::high_energy_astro::pulsar_surface_b_field(1.0e38, 1.0e4, 1000.0, 1.0e-38)
            .expect("B_surf should be defined");
    let pre = sample(world, true);
    let dt = DT;
    mps_core::rapier::world::world_step(world, dt);

    for (h, pos, mass) in &pre {
        let r = pos.length();
        assert!(r > 1.0e4, "body must be outside the NS surface");
        let b_mag = b_surf * (1.0e4 / r).powi(3);
        // τ = μ × B = (100 ŷ) × (b ẑ) = (100·b, 0, 0)
        let torque = Vector::new(100.0 * b_mag, 0.0, 0.0);
        if *mass > 0.0 {
            // Normal path: dω = τ / I · dt with the ball's inertia 2/5·m·R².
            let inertia = 0.4 * mass * 0.25;
            let expected = torque * (dt / inertia);
            assert_close(
                read_angvel(world, *h),
                expected,
                &format!("pulsar torque body {:?}", h.into_raw_parts()),
            );
        } else {
            // Fallback: unit rotational inertia, dω = τ · dt.
            let expected = torque * dt;
            assert_close(
                read_angvel(world, *h),
                expected,
                &format!("pulsar fallback body {:?}", h.into_raw_parts()),
            );
        }
    }
    mps_core::rapier::world::world_destroy(world);
}

// ---------------------------------------------------------------------------
// Snapshot export — parallel pose computation
// ---------------------------------------------------------------------------

#[test]
fn body_snapshot_parallel_matches_world_state() {
    let world = make_world(PARALLEL_N, true);
    // Add fixed + kinematic bodies; the dynamic snapshot must exclude them.
    {
        let w = unsafe { &mut (*world).inner };
        let fixed = w.bodies.insert(
            RigidBodyBuilder::fixed()
                .translation(Vector::new(500.0, 0.0, 0.0))
                .build(),
        );
        let collider = ColliderBuilder::ball(0.5).build();
        w.colliders
            .insert_with_parent(collider, fixed, &mut w.bodies);
        w.bodies.insert(
            RigidBodyBuilder::kinematic_position_based()
                .translation(Vector::new(600.0, 0.0, 0.0))
                .build(),
        );
    }

    let total = mps_core::rapier::world::world_body_snapshot_count(world) as usize;
    let dynamic = mps_core::rapier::world::world_dynamic_body_snapshot_count(world) as usize;
    assert_eq!(total, (PARALLEL_N + 2) as usize);
    assert_eq!(dynamic, PARALLEL_N as usize);

    let mut handles = vec![0u64; total];
    let mut values = vec![0.0f64; total * 13];
    let written = mps_core::rapier::world::world_body_snapshot(
        world,
        handles.as_mut_ptr(),
        values.as_mut_ptr(),
        total as u32,
    ) as usize;
    assert_eq!(written, total);

    let w = unsafe { &mut (*world).inner };
    for i in 0..written {
        let handle = mps_core::rapier::ffi::unpack_rigid_body_handle(handles[i]);
        let body = &w.bodies[handle];
        let t = body.translation();
        let lv = body.linvel();
        let v = &values[i * 13..i * 13 + 13];
        assert_eq!(v[0], t.x, "snapshot tx mismatch at {i}");
        assert_eq!(v[1], t.y, "snapshot ty mismatch at {i}");
        assert_eq!(v[2], t.z, "snapshot tz mismatch at {i}");
        assert_eq!(v[7], lv.x, "snapshot vx mismatch at {i}");
        assert_eq!(v[8], lv.y, "snapshot vy mismatch at {i}");
        assert_eq!(v[9], lv.z, "snapshot vz mismatch at {i}");
    }

    // Dynamic snapshot: every entry must unpack to a dynamic body, and the
    // velocity lanes must stay zero.
    let mut dyn_handles = vec![0u64; dynamic];
    let mut dyn_values = vec![0.0f64; dynamic * 7];
    let dyn_written = mps_core::rapier::world::world_dynamic_body_snapshot(
        world,
        dyn_handles.as_mut_ptr(),
        dyn_values.as_mut_ptr(),
        dynamic as u32,
    ) as usize;
    assert_eq!(dyn_written, dynamic);
    for i in 0..dyn_written {
        let handle = mps_core::rapier::ffi::unpack_rigid_body_handle(dyn_handles[i]);
        assert!(w.bodies[handle].is_dynamic(), "non-dynamic body at {i}");
        let t = w.bodies[handle].translation();
        assert_eq!(
            dyn_values[i * 7],
            t.x,
            "dynamic snapshot tx mismatch at {i}"
        );
        assert_eq!(
            dyn_values[i * 7 + 2],
            t.z,
            "dynamic snapshot tz mismatch at {i}"
        );
    }
    mps_core::rapier::world::world_destroy(world);
}

// ---------------------------------------------------------------------------
// Thread pool FFI
// ---------------------------------------------------------------------------

#[test]
fn thread_pool_ffi_reports_usable_count() {
    let count = mps_core::rapier::parallel::parallel_thread_count();
    assert!(count > 0, "thread count must be positive, got {count}");

    // Resizing only succeeds before the pool is first used; both outcomes are
    // acceptable here, but the getter must stay consistent and an explicit
    // zero must always be rejected.
    if mps_core::rapier::parallel::parallel_set_thread_count(count) == Bool::TRUE {
        assert_eq!(mps_core::rapier::parallel::parallel_thread_count(), count);
    } else {
        assert!(mps_core::rapier::parallel::parallel_thread_count() >= 1);
    }
    assert_eq!(
        mps_core::rapier::parallel::parallel_set_thread_count(0),
        Bool::FALSE,
        "zero thread count must be rejected"
    );
}
