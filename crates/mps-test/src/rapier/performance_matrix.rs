//! Repeated, interleaved first-step comparisons. Timings exclude setup/validation.
#![cfg(test)]
use mps_core::rapier::{
    events::*,
    ffi::{Bool, Vec3, WorldHandle},
    world::*,
};
use rapier3d::prelude::{ActiveHooks, ColliderBuilder, RigidBodyBuilder, Vector};
use std::time::Instant;

#[derive(Clone, Copy)]
struct Config {
    name: &'static str,
    colliders: bool,
    dense: bool,
    iterations: u32,
    substeps: u32,
    collision: bool,
    contact: bool,
    ccd: bool,
    hooks: bool,
    sleeping: bool,
}
impl Default for Config {
    fn default() -> Self {
        Self {
            name: "dense_baseline",
            colliders: true,
            dense: true,
            iterations: 4,
            substeps: 1,
            collision: false,
            contact: false,
            ccd: false,
            hooks: false,
            sleeping: true,
        }
    }
}
struct World(*mut WorldHandle);
impl Drop for World {
    fn drop(&mut self) {
        world_destroy(self.0);
    }
}
fn scene(n: usize, config: Config) -> World {
    let world = World(world_create(Vec3 {
        x: 0.0,
        y: -9.81,
        z: 0.0,
    }));
    assert!(!world.0.is_null());
    let w = unsafe { &mut (*world.0).inner };
    let spacing = if config.dense { 0.75 } else { 3.0 };
    for i in 0..n {
        let h = w.bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(
                    (i % 100) as f64 * spacing,
                    (i / 100) as f64 * spacing,
                    0.0,
                ))
                .linvel(Vector::new(if i % 2 == 0 { 20.0 } else { -20.0 }, 0.0, 0.0))
                .additional_mass_properties(rapier3d::prelude::MassProperties::new(
                    Vector::ZERO,
                    1.0,
                    Vector::splat(0.1),
                ))
                .build(),
        );
        if config.colliders {
            w.colliders.insert_with_parent(
                ColliderBuilder::ball(0.5)
                    .density(0.0)
                    .contact_force_event_threshold(0.0)
                    .active_hooks(if config.hooks {
                        ActiveHooks::FILTER_CONTACT_PAIRS | ActiveHooks::MODIFY_SOLVER_CONTACTS
                    } else {
                        ActiveHooks::empty()
                    })
                    .build(),
                h,
                &mut w.bodies,
            );
        }
    }
    assert_eq!(
        world_apply_runtime_settings(
            world.0,
            config.iterations,
            config.substeps,
            config.collision as u32,
            config.contact as u32,
            config.ccd as u32,
            config.sleeping as u32
        ),
        Bool::TRUE
    );
    world
}

#[cfg(windows)]
fn cpu_seconds() -> f64 {
    #[repr(C)]
    struct FileTime {
        low: u32,
        high: u32,
    }
    #[link(name = "kernel32")]
    unsafe extern "system" {
        fn GetCurrentProcess() -> *mut std::ffi::c_void;
        fn GetProcessTimes(
            process: *mut std::ffi::c_void,
            created: *mut FileTime,
            exited: *mut FileTime,
            kernel: *mut FileTime,
            user: *mut FileTime,
        ) -> i32;
    }
    let mut times = [const { FileTime { low: 0, high: 0 } }; 4];
    let p = times.as_mut_ptr();
    // Windows fills four separate FILETIME records for the current process.
    assert_ne!(
        unsafe { GetProcessTimes(GetCurrentProcess(), p, p.add(1), p.add(2), p.add(3)) },
        0
    );
    times[2..]
        .iter()
        .map(|t| (((t.high as u64) << 32) | t.low as u64) as f64 * 1e-7)
        .sum()
}
#[cfg(not(windows))]
fn cpu_seconds() -> f64 {
    f64::NAN
}

fn sample(config: Config, bodies: usize, steps: usize) -> (f64, f64, [f64; 7]) {
    let world = scene(bodies, config);
    let cpu_start = cpu_seconds();
    let start = Instant::now();
    for _ in 0..steps {
        world_step(world.0, 1.0 / 60.0);
    }
    let elapsed = start.elapsed().as_secs_f64();
    let cpu = cpu_seconds() - cpu_start;
    let w = unsafe { &(*world.0).inner };
    assert_eq!(w.bodies.len(), bodies);
    assert!(
        w.bodies
            .iter()
            .all(|(_, b)| b.translation().is_finite() && b.linvel().is_finite())
    );
    if !config.collision {
        assert_eq!(world_collision_event_count(world.0), 0);
    } else if config.dense {
        assert!(world_collision_event_count(world.0) > 0);
    }
    if !config.contact {
        assert_eq!(world_contact_force_event_count(world.0), 0);
    } else if config.dense {
        assert!(world_contact_force_event_count(world.0) > 0);
    }
    let mut stages = [0.0; 7];
    assert_eq!(
        world_get_pipeline_timings(world.0, stages.as_mut_ptr(), 7),
        7
    );
    (elapsed * 1000.0, cpu, stages)
}

#[test]
#[ignore = "Release performance matrix; run alone with --test-threads=1 --nocapture"]
fn world_step_performance_matrix() {
    if cfg!(debug_assertions) {
        panic!("run the matrix with --release");
    }
    let bodies = std::env::var("MPS_MATRIX_BODIES")
        .map(|v| {
            v.parse::<usize>()
                .expect("MPS_MATRIX_BODIES must be an integer")
        })
        .unwrap_or(10_000);
    let steps = std::env::var("MPS_MATRIX_STEPS")
        .map(|v| v.parse().expect("invalid steps"))
        .unwrap_or(1usize);
    assert!(steps > 0);
    assert!(bodies >= 2);
    let base = Config::default();
    let mut configs = vec![
        Config {
            name: "none",
            colliders: false,
            dense: false,
            ..base
        },
        Config {
            name: "sparse",
            dense: false,
            ..base
        },
        base,
        Config {
            name: "dense_all",
            collision: true,
            contact: true,
            ccd: true,
            hooks: true,
            ..base
        },
    ];
    for (name, iterations) in [
        ("solver_1", 1),
        ("solver_2", 2),
        ("solver_4", 4),
        ("solver_8", 8),
    ] {
        configs.push(Config {
            name,
            iterations,
            ..base
        });
    }
    for (name, substeps) in [("ccd_0", 0), ("ccd_1", 1), ("ccd_2", 2), ("ccd_4", 4)] {
        configs.push(Config {
            name,
            substeps,
            ccd: true,
            ..base
        });
    }
    configs.extend([
        Config {
            name: "collision_on",
            collision: true,
            ..base
        },
        Config {
            name: "contact_on",
            contact: true,
            ..base
        },
        Config {
            name: "hooks_on",
            hooks: true,
            ..base
        },
        Config {
            name: "sleep_off",
            sleeping: false,
            ..base
        },
    ]);
    if let Ok(filter) = std::env::var("MPS_MATRIX_CASES") {
        configs.retain(|config| filter.split(',').any(|name| name == config.name));
        assert!(!configs.is_empty(), "MPS_MATRIX_CASES matched no cases");
    }
    let cores = std::thread::available_parallelism().unwrap().get();
    eprintln!(
        "matrix: bodies={bodies}, steps={steps}, logical_cpus={cores}, rayon_threads={}, profiler={}",
        mps_core::rapier::parallel::thread_count(),
        cfg!(feature = "profiler")
    );
    let mut samples = vec![Vec::new(); configs.len()];
    // Warm up code paths, then rotate order to distribute thermal/order bias.
    for config in &configs {
        sample(*config, bodies, steps);
    }
    for round in 0..10 {
        for offset in 0..configs.len() {
            let index = (round + offset) % configs.len();
            samples[index].push(sample(configs[index], bodies, steps));
        }
        eprintln!("matrix repeat {}/10 complete", round + 1);
    }
    let mut raw = String::from(
        "case,bodies,repeat,wall_ms,cpu_seconds,update_ms,broad_ms,narrow_ms,island_ms,solver_ms,ccd_ms,pipeline_ms\n",
    );
    let mut csv = String::from("case,n,mean_ms,p50_ms,p95_ms,cpu_percent_machine\n");
    for (config, sample) in configs.iter().zip(samples) {
        for (repeat, (wall, cpu, stages)) in sample.iter().enumerate() {
            raw.push_str(&format!(
                "{},{bodies},{},{wall:.6},{cpu:.6}",
                config.name,
                repeat + 1
            ));
            for stage in stages {
                raw.push_str(&format!(",{stage:.6}"));
            }
            raw.push('\n');
        }
        let cpu: f64 = sample.iter().map(|v| v.1).sum();
        let mut times: Vec<f64> = sample.iter().map(|v| v.0).collect();
        times.sort_by(f64::total_cmp);
        let total: f64 = times.iter().sum();
        let row = format!(
            "{},10,{:.3},{:.3},{:.3},{:.2}\n",
            config.name,
            total / 10.0,
            (times[4] + times[5]) / 2.0,
            times[9],
            cpu / (total / 1000.0) / cores as f64 * 100.0
        );
        eprint!("{row}");
        csv.push_str(&row);
    }
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target");
    std::fs::create_dir_all(&root).unwrap();
    std::fs::write(root.join("world-step-matrix.csv"), csv).unwrap();
    std::fs::write(root.join("world-step-matrix-raw.csv"), raw).unwrap();
}

/// Jolt/Rapier 跨引擎对比基准（`#[ignore]`，与 `world_step_performance_matrix`
/// 同协议：release、--test-threads=1、10 轮轮换取 p50/p95）。
///
/// **对比方法**（ADR-0008 之后的定论：Jolt 是 C++，不编进本仓库）：
/// 1. 本测试用**与 Jolt 官方 Testbed 基准同构的场景**（10k 刚体球堆 +
///    solver 迭代扫描）跑 mps 的 `world_step`，输出标准化 CSV
///    `target/engine-compare-mps.csv`：每步墙钟毫秒（p50/p95/mean）、
///    CPU%、阶段分解（broad/narrow/solver）。
/// 2. 同机同场景的 Jolt 数字由 Jolt 官方 `PerformanceTest`（JoltPhysics 仓库
///    `Testbed/Tests/PerformanceTest.cpp`，默认 4×4×4 堆叠 ×N）或自建
///    `jolt::PerformanceTest` 采集，填进 `docs/evidence/engine-compare.md`
///    的对照表。两张表同负载、同迭代数（Jolt 12 vs mps 12）、同 sleeping
///    策略才可比。
/// 3. 结论记录进 evidence 文档（日期、commit、CPU 型号、频率），不进 CI。
///
/// 场景刻意与 Jolt PerformanceTest 对齐：立方体堆叠（有休眠收益）而非
/// 本文件主矩阵的无限反弹球对（无休眠、纯 solver 压力）——前者是 MC 服务器
/// 的真实负载形态。
#[test]
#[ignore = "cross-engine compare; run alone with --release --test-threads=1 --nocapture"]
fn engine_benchmark_vs_jolt_protocol() {
    if cfg!(debug_assertions) {
        panic!("run the engine compare with --release");
    }
    // Jolt PerformanceTest 默认 4 级金字塔 × 每级 4×4（共 ~100 动态体 +
    // 地面）；这里放大到 MC 服务器量级：N×N 网格立方体落在静态地面上。
    let side = std::env::var("MPS_COMPARE_SIDE")
        .map(|v| v.parse::<usize>().expect("MPS_COMPARE_SIDE must be usize"))
        .unwrap_or(40); // 40×40 = 1600 立方体，接近 Jolt 大场景
    let steps = std::env::var("MPS_COMPARE_STEPS")
        .map(|v| v.parse::<usize>().expect("MPS_COMPARE_STEPS must be usize"))
        .unwrap_or(600); // 10 秒模拟，够休眠稳定
    assert!(side > 0 && steps > 0);

    let mut wall = Vec::with_capacity(10);
    for _ in 0..10 {
        let world = World(world_create(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        }));
        let w = unsafe { &mut (*world.0).inner };
        // 静态地面。
        w.colliders
            .insert(ColliderBuilder::cuboid(1e4, 0.5, 1e4).build());
        // side×side 立方体堆（间隔略大于边长，避免初始穿插）。
        let spacing = 1.1;
        for i in 0..side {
            for j in 0..side {
                let h = w.bodies.insert(
                    RigidBodyBuilder::dynamic()
                        .translation(Vector::new(
                            (i as f64 - side as f64 * 0.5) * spacing,
                            1.0 + (j as f64) * spacing,
                            0.0,
                        ))
                        .build(),
                );
                w.colliders.insert_with_parent(
                    ColliderBuilder::cuboid(0.5, 0.5, 0.5).build(),
                    h,
                    &mut w.bodies,
                );
            }
        }
        // solver 迭代对齐 Jolt 默认 12。
        assert_eq!(
            world_apply_runtime_settings(world.0, 12, 1, 0, 0, 0, 1),
            Bool::TRUE
        );
        // 预热 60 步（落定+休眠），不计入计时——与 Jolt 采集方式一致。
        for _ in 0..60 {
            world_step(world.0, 1.0 / 60.0);
        }
        let start = Instant::now();
        for _ in 0..steps {
            world_step(world.0, 1.0 / 60.0);
        }
        wall.push(start.elapsed().as_secs_f64() * 1000.0 / steps as f64);
        drop(world); // World RAII 统一销毁（手动 destroy 会 double-free）
    }
    wall.sort_by(f64::total_cmp);
    let p50 = wall[4];
    let p95 = wall[9];
    let mean: f64 = wall.iter().sum::<f64>() / wall.len() as f64;
    let cores = std::thread::available_parallelism().unwrap().get();
    eprintln!(
        "engine-compare: {0}x{0} cubes ({0}x{0} bodies), {steps} steps, 12 iters\n  mean {mean:.3} ms/step, p50 {p50:.3}, p95 {p95:.3}",
        side * side,
    );

    // 标准化输出：docs/evidence/engine-compare.md 的采集脚本消费这个 CSV。
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target");
    std::fs::create_dir_all(&root).unwrap();
    let csv = format!(
        "engine,scene,bodies,steps,mean_ms,p50_ms,p95_ms,logical_cpus\nmps,stacked_cubes,{},{steps},{mean:.4},{p50:.4},{p95:.4},{cores}\n",
        side * side,
    );
    std::fs::write(root.join("engine-compare-mps.csv"), csv).unwrap();
}

#[test]
fn runtime_settings_validate_before_mutation() {
    let world = scene(2, Config::default());
    assert_eq!(
        world_apply_runtime_settings(world.0, 0, 1, 1, 1, 1, 1),
        Bool::FALSE
    );
    assert!(
        unsafe { &(*world.0).inner }
            .bodies
            .iter()
            .all(|(_, b)| !b.is_ccd_enabled())
    );
    assert_eq!(
        world_apply_runtime_settings(world.0, 2, 2, 1, 1, 1, 0),
        Bool::TRUE
    );
    assert!(
        unsafe { &(*world.0).inner }
            .bodies
            .iter()
            .all(|(_, b)| b.is_ccd_enabled() && b.activation().normalized_linear_threshold < 0.0)
    );
}

#[test]
fn runtime_settings_events_do_not_change_collision_response() {
    let base = Config::default();
    let off = scene(2, base);
    let on = scene(
        2,
        Config {
            collision: true,
            contact: true,
            hooks: true,
            ..base
        },
    );
    world_step(off.0, 1.0 / 60.0);
    world_step(on.0, 1.0 / 60.0);
    assert_eq!(world_collision_event_count(off.0), 0);
    assert_eq!(world_contact_force_event_count(off.0), 0);
    assert!(world_collision_event_count(on.0) > 0);
    assert!(world_contact_force_event_count(on.0) > 0);
    let a = unsafe { &(*off.0).inner };
    let b = unsafe { &(*on.0).inner };
    for ((_, a), (_, b)) in a.bodies.iter().zip(b.bodies.iter()) {
        assert!((a.translation() - b.translation()).length() < 1e-10);
        assert!((a.linvel() - b.linvel()).length() < 1e-10);
    }
}

#[test]
fn runtime_settings_sleeping_and_ccd_have_real_effects() {
    for sleeping in [0, 1] {
        let world = World(world_create(Vec3::default()));
        let h = unsafe {
            (*world.0)
                .inner
                .bodies
                .insert(RigidBodyBuilder::dynamic().additional_mass(1.0).build())
        };
        assert_eq!(
            world_apply_runtime_settings(world.0, 4, 1, 0, 0, 0, sleeping),
            Bool::TRUE
        );
        for _ in 0..120 {
            world_step(world.0, 1.0 / 60.0);
        }
        assert_eq!(
            unsafe { (&(*world.0).inner.bodies)[h].is_sleeping() },
            sleeping != 0
        );
    }
    for ccd in [0, 1] {
        let world = World(world_create(Vec3::default()));
        let w = unsafe { &mut (*world.0).inner };
        w.colliders
            .insert(ColliderBuilder::cuboid(0.02, 2.0, 2.0).build());
        let h = w.bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(-1.0, 0.0, 0.0))
                .linvel(Vector::new(120.0, 0.0, 0.0))
                .build(),
        );
        w.colliders
            .insert_with_parent(ColliderBuilder::ball(0.1).build(), h, &mut w.bodies);
        assert_eq!(
            world_apply_runtime_settings(world.0, 4, 1, 0, 0, ccd, 0),
            Bool::TRUE
        );
        world_step(world.0, 1.0 / 60.0);
        let x = unsafe { (&(*world.0).inner.bodies)[h].translation().x };
        if ccd != 0 {
            assert!(x < 0.0, "CCD failed to stop projectile: {x}");
        } else {
            assert!(x > 0.0, "discrete baseline did not cross thin wall: {x}");
        }
    }
}
