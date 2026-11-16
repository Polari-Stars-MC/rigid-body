//! `mps-core/src/rapier/tyndall_scene.rs` 的镜像测试。
//!
//! 这个模块的端到端行为（读真实流体粒子算散射）在
//! [`crate::rapier::tyndall_particles`] 里测过了 —— 那里有完整的粒子云/散射
//! 积分测试。这里只覆盖**模块自身的入口契约**：来源枚举、错误码、null 处理。
//!
//! 之所以不重复搬一遍物理测试：`tyndall_scene` 只是把
//! `mps_formula::tyndall_particles` 的结果搬进 C ABI，物理正确性属于公式层。

use mps_core::rapier::error::{
    ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER, last_error_code,
};
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

fn make_smoke(world: *mut WorldHandle, count: u32) -> u32 {
    let id = fluid_create(world, 0.0, 0.0, 0.0, 0.8, 2000.0, 1000.0, 0.1, 0.0);
    assert_ne!(id, u32::MAX, "fluid_create failed");
    for i in 0..count {
        let x = i as f64 * 0.4;
        fluid_add_particle(world, id, x, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-3);
    }
    id
}

#[test]
fn source_ids_are_stable() {
    // 这两个数字进了头文件的 `#define`，不能随便改。
    assert_eq!(TYNDALL_SOURCE_FLUID, 0);
    assert_eq!(TYNDALL_SOURCE_GRANULAR, 1);
    assert_eq!(tyndall_source_abi_version(), 2);
}

#[test]
fn collect_particles_reports_total_not_copied() {
    let world = make_world();
    let id = make_smoke(world, 4);

    let total = tyndall_collect_particles(world, TYNDALL_SOURCE_FLUID, id, std::ptr::null_mut(), 0);
    assert_eq!(total, 4);

    // 容量不足：仍返回总数。
    let mut buf = vec![Vec3::default(); 2];
    let n = tyndall_collect_particles(world, TYNDALL_SOURCE_FLUID, id, buf.as_mut_ptr(), 2);
    assert_eq!(n, 4);

    world_destroy(world);
}

#[test]
fn collect_particles_error_paths() {
    let world = make_world();
    let id = make_smoke(world, 2);

    // 未知来源。
    assert_eq!(
        tyndall_collect_particles(world, 42, id, std::ptr::null_mut(), 0),
        0
    );
    assert_eq!(last_error_code(), ERR_INVALID_ARGUMENT);

    // 未知 id。
    assert_eq!(
        tyndall_collect_particles(world, TYNDALL_SOURCE_FLUID, 999, std::ptr::null_mut(), 0),
        0
    );
    assert_eq!(last_error_code(), ERR_NOT_FOUND);

    // null world。
    assert_eq!(
        tyndall_collect_particles(
            std::ptr::null(),
            TYNDALL_SOURCE_FLUID,
            id,
            std::ptr::null_mut(),
            0
        ),
        0
    );
    assert_eq!(last_error_code(), ERR_NULL_POINTER);

    world_destroy(world);
}

#[test]
fn inscatter_rejects_null_out_and_world() {
    let world = make_world();
    let id = make_smoke(world, 2);

    // null out。
    assert_eq!(
        tyndall_inscatter_from_world(
            world,
            TYNDALL_SOURCE_FLUID,
            id,
            Vec3::default(),
            Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            1.0,
            0.8,
            1e-12,
            1e-12,
            1.0,
            1.0,
            8,
            std::ptr::null_mut(),
        ),
        Bool::FALSE
    );
    assert_eq!(last_error_code(), ERR_NULL_POINTER);

    // null world。
    let mut out = 0.0f64;
    assert_eq!(
        tyndall_inscatter_from_world(
            std::ptr::null(),
            TYNDALL_SOURCE_FLUID,
            id,
            Vec3::default(),
            Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            1.0,
            0.8,
            1e-12,
            1e-12,
            1.0,
            1.0,
            8,
            &mut out,
        ),
        Bool::FALSE
    );
    assert_eq!(last_error_code(), ERR_NULL_POINTER);

    world_destroy(world);
}

#[test]
fn density_and_bounds_reject_null_out() {
    let world = make_world();
    let id = make_smoke(world, 2);

    assert_eq!(
        tyndall_density_from_world(
            world,
            TYNDALL_SOURCE_FLUID,
            id,
            Vec3::default(),
            0.8,
            std::ptr::null_mut()
        ),
        Bool::FALSE
    );
    assert_eq!(last_error_code(), ERR_NULL_POINTER);

    assert_eq!(
        tyndall_bounds_from_world(
            world,
            TYNDALL_SOURCE_FLUID,
            id,
            std::ptr::null_mut(),
            std::ptr::null_mut()
        ),
        Bool::FALSE
    );
    assert_eq!(last_error_code(), ERR_NULL_POINTER);

    world_destroy(world);
}
