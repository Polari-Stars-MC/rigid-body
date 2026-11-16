//! 粒子云 → 体积散射的耦合测试（ADR-0019）。
//!
//! 关键断言：
//!
//! * Poly6 核归一化正确、紧支撑（`r >= h` 时为 0）
//! * 核加权数密度随粒子聚集而升高
//! * SPH 质量密度 → 数密度的换算自洽
//! * 沿视线穿过粒子云的积分随粒子增多而增大、随消光而饱和
//! * 方向必须归一化（否则积分长度失真）
//! * 包围盒正确

use mps_formula::tyndall_particles::{
    ParticleOptics, inscatter_through_particles, number_density_from_mass_density,
    particle_cloud_bounds, poly6_kernel, sample_number_density,
};

const EPS: f64 = 1e-12;

#[test]
fn poly6_is_compact_supported() {
    let h = 1.0;
    // r = 0 时取最大值。
    let w0 = poly6_kernel(0.0, h).unwrap();
    assert!(w0 > 0.0);
    // r 增大时单调递减。
    let w_half = poly6_kernel(0.5, h).unwrap();
    assert!(w0 > w_half, "kernel must decrease: {w0} vs {w_half}");
    // r >= h 时严格为 0（紧支撑）。
    assert_eq!(poly6_kernel(1.0, h).unwrap(), 0.0);
    assert_eq!(poly6_kernel(2.0, h).unwrap(), 0.0);
}

#[test]
fn poly6_matches_the_analytic_formula() {
    let h: f64 = 2.0;
    let r: f64 = 0.5;
    let expect = 315.0 / (64.0 * std::f64::consts::PI * h.powi(9)) * (h * h - r * r).powi(3);
    let got = poly6_kernel(r, h).unwrap();
    assert!(
        (got - expect).abs() / expect < EPS,
        "got {got} expected {expect}"
    );
}

#[test]
fn poly6_rejects_invalid_arguments() {
    assert!(poly6_kernel(-1.0, 1.0).is_none());
    assert!(poly6_kernel(0.5, 0.0).is_none());
    assert!(poly6_kernel(0.5, -1.0).is_none());
    assert!(poly6_kernel(f64::NAN, 1.0).is_none());
    assert!(poly6_kernel(0.5, f64::INFINITY).is_none());
}

/// 数密度随粒子聚集单调升高。
#[test]
fn density_grows_with_particle_count() {
    let h = 1.0;
    let query = [0.0, 0.0, 0.0];
    let sparse = vec![[0.0, 0.0, 0.0], [0.4, 0.0, 0.0]];
    let dense = vec![
        [0.0, 0.0, 0.0],
        [0.4, 0.0, 0.0],
        [0.0, 0.4, 0.0],
        [0.0, 0.0, 0.4],
        [0.25, 0.25, 0.25],
    ];
    let n_sparse = sample_number_density(&sparse, query, h).unwrap();
    let n_dense = sample_number_density(&dense, query, h).unwrap();
    assert!(
        n_dense > n_sparse,
        "more particles must give higher density: {n_dense} vs {n_sparse}"
    );
}

/// 超出核半径的粒子不贡献。
#[test]
fn density_ignores_distant_particles() {
    let h = 1.0;
    let query = [0.0, 0.0, 0.0];
    let near = vec![[0.1, 0.0, 0.0]];
    let far = vec![[10.0, 0.0, 0.0]];
    let n_near = sample_number_density(&near, query, h).unwrap();
    let n_far = sample_number_density(&far, query, h).unwrap();
    assert!(n_near > 0.0);
    assert_eq!(n_far, 0.0, "particles beyond h must not contribute");
}

#[test]
fn density_rejects_invalid_input() {
    let positions = vec![[0.0, 0.0, 0.0]];
    assert!(sample_number_density(&positions, [0.0, 0.0, 0.0], 0.0).is_none());
    assert!(sample_number_density(&positions, [0.0, 0.0, 0.0], -1.0).is_none());
    assert!(sample_number_density(&positions, [f64::NAN, 0.0, 0.0], 1.0).is_none());
    // 非有限的粒子被跳过而不是整体失败。
    let dirty = vec![[f64::NAN, 0.0, 0.0], [0.1, 0.0, 0.0]];
    assert!(sample_number_density(&dirty, [0.0, 0.0, 0.0], 1.0).is_some());
}

/// SPH 质量密度 → 数密度：`n = ρ/m`。
#[test]
fn mass_density_conversion() {
    let n = number_density_from_mass_density(1000.0, 0.001).unwrap();
    assert!((n - 1e6).abs() / 1e6 < EPS);
    // 质量翻倍 → 数密度减半（同样质量密度下粒子更少）。
    let n2 = number_density_from_mass_density(1000.0, 0.002).unwrap();
    assert!((n2 * 2.0 - n).abs() / n < EPS);
}

#[test]
fn mass_density_conversion_rejects_invalid() {
    assert!(number_density_from_mass_density(-1.0, 0.001).is_none());
    assert!(number_density_from_mass_density(1.0, 0.0).is_none());
    assert!(number_density_from_mass_density(1.0, -0.001).is_none());
    assert!(number_density_from_mass_density(f64::NAN, 0.001).is_none());
}

/// 光学参数校验：散射不能超过消光。
#[test]
fn particle_optics_validation() {
    let good = ParticleOptics {
        sigma_sca: 1e-12,
        sigma_ext: 1e-12,
        mass: 1e-6,
    };
    assert!(good.is_valid());

    let bad_albedo = ParticleOptics {
        sigma_sca: 2e-12,
        sigma_ext: 1e-12,
        mass: 1e-6,
    };
    assert!(!bad_albedo.is_valid(), "scattering > extinction");

    let bad_mass = ParticleOptics {
        sigma_sca: 1e-12,
        sigma_ext: 1e-12,
        mass: 0.0,
    };
    assert!(!bad_mass.is_valid());
}

/// 沿视线穿过粒子云：粒子越多，散射越强。
#[test]
fn inscatter_grows_with_particle_count() {
    let optics = ParticleOptics {
        sigma_sca: 1e-12,
        sigma_ext: 1e-12,
        mass: 1e-6,
    };
    let origin = [-2.0, 0.0, 0.0];
    let direction = [1.0, 0.0, 0.0];
    let h = 0.8;

    // 稀疏：沿 X 轴几个粒子。
    let sparse: Vec<[f64; 3]> = (0..5).map(|i| [i as f64 * 0.5 - 1.0, 0.0, 0.0]).collect();
    // 密集：同样的位置但每个点周围加一圈。
    let mut dense = sparse.clone();
    for i in 0..5 {
        let x = i as f64 * 0.5 - 1.0;
        dense.push([x, 0.2, 0.0]);
        dense.push([x, 0.0, 0.2]);
        dense.push([x, -0.2, 0.0]);
    }

    let l_sparse =
        inscatter_through_particles(&sparse, optics, origin, direction, 4.0, h, 1.0, 1.0, 64)
            .unwrap();
    let l_dense =
        inscatter_through_particles(&dense, optics, origin, direction, 4.0, h, 1.0, 1.0, 64)
            .unwrap();

    assert!(l_sparse > 0.0, "sparse cloud must scatter something");
    assert!(
        l_dense > l_sparse,
        "denser cloud must be brighter: {l_dense} vs {l_sparse}"
    );
}

/// 视线不穿过粒子云时亮度为 0。
#[test]
fn inscatter_is_zero_when_ray_misses_the_cloud() {
    let optics = ParticleOptics {
        sigma_sca: 1e-12,
        sigma_ext: 1e-12,
        mass: 1e-6,
    };
    // 粒子云在原点附近，视线在 y = 100 处平行走过。
    let cloud = vec![[0.0, 0.0, 0.0], [0.2, 0.0, 0.0]];
    let l = inscatter_through_particles(
        &cloud,
        optics,
        [-2.0, 100.0, 0.0],
        [1.0, 0.0, 0.0],
        4.0,
        0.5,
        1.0,
        1.0,
        32,
    )
    .unwrap();
    assert_eq!(l, 0.0, "a ray far from the cloud must see nothing");
}

/// 强消光下亮度饱和（远处段落被挡住）。
#[test]
fn inscatter_saturates_with_high_extinction() {
    let positions: Vec<[f64; 3]> = (0..40).map(|i| [i as f64 * 0.1, 0.0, 0.0]).collect();
    let origin = [-1.0, 0.0, 0.0];
    let direction = [1.0, 0.0, 0.0];

    let low = ParticleOptics {
        sigma_sca: 1e-13,
        sigma_ext: 1e-13,
        mass: 1e-6,
    };
    let high = ParticleOptics {
        sigma_sca: 1e-11,
        sigma_ext: 1e-11,
        mass: 1e-6,
    };

    let l_low =
        inscatter_through_particles(&positions, low, origin, direction, 6.0, 0.6, 1.0, 1.0, 128)
            .unwrap();
    let l_high =
        inscatter_through_particles(&positions, high, origin, direction, 6.0, 0.6, 1.0, 1.0, 128)
            .unwrap();
    // 高消光时靠前的段落就吃掉了全部预算，总亮度不会随消光系数线性增长。
    assert!(l_low > 0.0 && l_high > 0.0);
    assert!(
        l_high < l_low * 100.0,
        "high extinction must saturate, got {l_high} vs {l_low}"
    );
}

/// 方向必须归一化 —— 否则物理长度错了，直接拒绝。
#[test]
fn inscatter_rejects_unnormalised_direction() {
    let optics = ParticleOptics {
        sigma_sca: 1e-12,
        sigma_ext: 1e-12,
        mass: 1e-6,
    };
    let cloud = vec![[0.0, 0.0, 0.0]];
    let r = inscatter_through_particles(
        &cloud,
        optics,
        [0.0, 0.0, 0.0],
        [2.0, 0.0, 0.0], // 未归一化
        1.0,
        0.5,
        1.0,
        1.0,
        8,
    );
    assert!(r.is_none(), "unnormalised direction must be rejected");
}

#[test]
fn inscatter_rejects_invalid_arguments() {
    let optics = ParticleOptics {
        sigma_sca: 1e-12,
        sigma_ext: 1e-12,
        mass: 1e-6,
    };
    let cloud = vec![[0.0, 0.0, 0.0]];
    let o = [0.0, 0.0, 0.0];
    let d = [1.0, 0.0, 0.0];

    // steps = 0
    assert!(inscatter_through_particles(&cloud, optics, o, d, 1.0, 0.5, 1.0, 1.0, 0).is_none());
    // 负长度
    assert!(inscatter_through_particles(&cloud, optics, o, d, -1.0, 0.5, 1.0, 1.0, 8).is_none());
    // 非法 h
    assert!(inscatter_through_particles(&cloud, optics, o, d, 1.0, 0.0, 1.0, 1.0, 8).is_none());
    // 非法光学参数
    let bad = ParticleOptics {
        sigma_sca: 2e-12,
        sigma_ext: 1e-12,
        mass: 1e-6,
    };
    assert!(inscatter_through_particles(&cloud, bad, o, d, 1.0, 0.5, 1.0, 1.0, 8).is_none());
}

/// 空粒子云 → 亮度 0（不是错误）。
#[test]
fn inscatter_is_zero_for_empty_cloud() {
    let optics = ParticleOptics {
        sigma_sca: 1e-12,
        sigma_ext: 1e-12,
        mass: 1e-6,
    };
    let l = inscatter_through_particles(
        &[],
        optics,
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        1.0,
        0.5,
        1.0,
        1.0,
        8,
    )
    .unwrap();
    assert_eq!(l, 0.0);
}

#[test]
fn bounds_enclose_the_cloud() {
    let cloud = vec![[-1.0, -2.0, -3.0], [1.0, 2.0, 3.0], [0.0, 0.0, 0.0]];
    let (min, max) = particle_cloud_bounds(&cloud).unwrap();
    assert_eq!(min, [-1.0, -2.0, -3.0]);
    assert_eq!(max, [1.0, 2.0, 3.0]);
}

#[test]
fn bounds_reject_empty_or_all_invalid() {
    assert!(particle_cloud_bounds(&[]).is_none());
    assert!(particle_cloud_bounds(&[[f64::NAN, 0.0, 0.0]]).is_none());
    // 部分有效时用有效的那些。
    let mixed = vec![[f64::NAN, 0.0, 0.0], [1.0, 1.0, 1.0]];
    let (min, _) = particle_cloud_bounds(&mixed).unwrap();
    assert_eq!(min, [1.0, 1.0, 1.0]);
}

// ── 场景层：从 WorldHandle 读真实粒子（tyndall_scene.rs）────────────────────

use mps_core::rapier::error::{ERR_NOT_FOUND, last_error_code};
use mps_core::rapier::ffi::{Bool, Vec3, WorldHandle};
use mps_core::rapier::fluid_sph::{fluid_add_particle, fluid_create};
use mps_core::rapier::tyndall_scene::{
    TYNDALL_SOURCE_FLUID, TYNDALL_SOURCE_GRANULAR, tyndall_bounds_from_world,
    tyndall_collect_particles, tyndall_density_from_world, tyndall_inscatter_from_world,
    tyndall_source_abi_version,
};
use mps_core::rapier::world::{world_create, world_destroy};

fn make_world() -> *mut WorldHandle {
    world_create(Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    })
}

/// 建一个沿 X 轴排布的小烟柱。
fn make_smoke(world: *mut WorldHandle, count: u32) -> u32 {
    let id = fluid_create(world, 0.0, 0.0, 0.0, 0.8, 2000.0, 1000.0, 0.1, 0.0);
    assert_ne!(id, u32::MAX, "fluid_create failed");
    for i in 0..count {
        let x = i as f64 * 0.4;
        let n = fluid_add_particle(world, id, x, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-3);
        assert_ne!(n, u32::MAX, "fluid_add_particle failed at {i}");
    }
    id
}

#[test]
fn source_abi_version_matches_define() {
    assert_eq!(tyndall_source_abi_version(), 2);
    assert_eq!(TYNDALL_SOURCE_FLUID, 0);
    assert_eq!(TYNDALL_SOURCE_GRANULAR, 1);
}

/// 先探数量、再按容量读取 —— 与 `granular_read_particles` 同约定。
#[test]
fn collect_particles_probes_then_reads() {
    let world = make_world();
    let id = make_smoke(world, 5);

    // 探数量：out 为 null。
    let total = tyndall_collect_particles(world, TYNDALL_SOURCE_FLUID, id, std::ptr::null_mut(), 0);
    assert_eq!(total, 5);

    // 按容量读取。
    let mut buf = vec![Vec3::default(); 5];
    let n = tyndall_collect_particles(world, TYNDALL_SOURCE_FLUID, id, buf.as_mut_ptr(), 5);
    assert_eq!(n, 5);
    // 位置应该是 0, 0.4, 0.8, ...
    assert!((buf[0].x - 0.0).abs() < 1e-12);
    assert!((buf[1].x - 0.4).abs() < 1e-12);
    assert!((buf[4].x - 1.6).abs() < 1e-12);

    // 容量不足时只写前 capacity 个，但仍返回总数。
    let mut small = vec![Vec3::default(); 2];
    let n2 = tyndall_collect_particles(world, TYNDALL_SOURCE_FLUID, id, small.as_mut_ptr(), 2);
    assert_eq!(n2, 5, "must report the total, not the copied count");

    world_destroy(world);
}

#[test]
fn collect_particles_rejects_bad_source_and_id() {
    let world = make_world();
    let id = make_smoke(world, 2);

    let bogus_source = tyndall_collect_particles(world, 99, id, std::ptr::null_mut(), 0);
    assert_eq!(bogus_source, 0);

    let bad_id =
        tyndall_collect_particles(world, TYNDALL_SOURCE_FLUID, 9999, std::ptr::null_mut(), 0);
    assert_eq!(bad_id, 0);
    assert_eq!(last_error_code(), ERR_NOT_FOUND);

    let null_world = tyndall_collect_particles(
        std::ptr::null(),
        TYNDALL_SOURCE_FLUID,
        id,
        std::ptr::null_mut(),
        0,
    );
    assert_eq!(null_world, 0);

    world_destroy(world);
}

/// 端到端：从真实流体粒子算出的散射亮度 > 0，且比空场景大。
#[test]
fn inscatter_from_world_end_to_end() {
    let world = make_world();
    let id = make_smoke(world, 20);

    let mut lum = 0.0f64;
    let ok = tyndall_inscatter_from_world(
        world,
        TYNDALL_SOURCE_FLUID,
        id,
        Vec3 {
            x: -1.0,
            y: 0.0,
            z: 0.0,
        },
        Vec3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        },
        12.0,  // 长度
        0.8,   // 核半径
        1e-12, // sigma_sca
        1e-12, // sigma_ext
        1.0,   // phase
        1.0,   // light
        128,   // steps
        &mut lum,
    );
    assert_eq!(ok, Bool::TRUE, "inscatter should succeed");
    assert!(lum > 0.0, "a smoke column must scatter light, got {lum}");

    world_destroy(world);
}

/// 方向未归一化必须被拒绝（否则积分长度失真）。
#[test]
fn inscatter_from_world_rejects_unnormalised_direction() {
    let world = make_world();
    let id = make_smoke(world, 3);
    let mut lum = -1.0f64;

    let ok = tyndall_inscatter_from_world(
        world,
        TYNDALL_SOURCE_FLUID,
        id,
        Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        Vec3 {
            x: 3.0,
            y: 0.0,
            z: 0.0,
        }, // 未归一化
        5.0,
        0.8,
        1e-12,
        1e-12,
        1.0,
        1.0,
        32,
        &mut lum,
    );
    assert_eq!(ok, Bool::FALSE);
    assert_eq!(lum, -1.0, "failed call must not write out");

    world_destroy(world);
}

/// 数密度查询：粒子所在处的密度应高于远处。
#[test]
fn density_from_world_is_higher_near_particles() {
    let world = make_world();
    let id = make_smoke(world, 6);

    let mut near = 0.0f64;
    let mut far = 0.0f64;
    assert_eq!(
        tyndall_density_from_world(
            world,
            TYNDALL_SOURCE_FLUID,
            id,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            0.8,
            &mut near,
        ),
        Bool::TRUE
    );
    assert_eq!(
        tyndall_density_from_world(
            world,
            TYNDALL_SOURCE_FLUID,
            id,
            Vec3 {
                x: 0.0,
                y: 100.0,
                z: 0.0,
            },
            0.8,
            &mut far,
        ),
        Bool::TRUE
    );
    assert!(near > 0.0, "near a particle the density must be positive");
    assert_eq!(far, 0.0, "far away there are no particles");

    world_destroy(world);
}

/// 包围盒覆盖所有粒子。
#[test]
fn bounds_from_world_covers_the_cloud() {
    let world = make_world();
    let id = make_smoke(world, 5);

    let mut min = Vec3::default();
    let mut max = Vec3::default();
    assert_eq!(
        tyndall_bounds_from_world(world, TYNDALL_SOURCE_FLUID, id, &mut min, &mut max),
        Bool::TRUE
    );
    // 粒子 x = 0, 0.4, 0.8, 1.2, 1.6
    assert!((min.x - 0.0).abs() < 1e-12, "min.x = {}", min.x);
    assert!((max.x - 1.6).abs() < 1e-12, "max.x = {}", max.x);

    world_destroy(world);
}
