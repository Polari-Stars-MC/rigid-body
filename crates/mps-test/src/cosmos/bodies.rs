//! `mps_cosmos::bodies` 测试 —— 迁移自 `crates/mps-cosmos/src/bodies.rs`。

#[cfg(test)]
use mps_cosmos::bodies::{fixed_body_builder, satellite_builder};
#[cfg(test)]
use rapier3d::prelude::{ColliderSet, Vector};

#[test]
fn satellite_builder_sets_mass_and_kinematics() {
    let builder = satellite_builder(
        1000.0,
        Vector::new(7e6, 0.0, 0.0),
        Vector::new(0.0, 7800.0, 0.0),
        1.0,
    );
    let mut body = builder.build();
    // build() 只把 additional_mass_properties 暂存到 additional_local_mprops，
    // 并未并入 local_mprops；RigidBody::mass() 走的是 local_mprops.inv_mass。
    // 所以在第一次 step（或显式 recompute）之前，mass() 读不到 1000.0。
    // 这里显式重算一次以验证 builder 设置真的进入了质量属性。
    body.recompute_mass_properties_from_colliders(&ColliderSet::new());
    assert!((body.mass() - 1000.0).abs() < 1e-6);
    assert!((body.translation().x - 7e6).abs() < 1e-6);
    assert!((body.linvel().y - 7800.0).abs() < 1e-6);
    assert!(body.is_dynamic());
}

#[test]
fn fixed_builder_is_not_dynamic() {
    let body = fixed_body_builder(Vector::new(1.0, 2.0, 3.0)).build();
    assert!(body.is_fixed());
}

/// 复刻 Java `SpaceWorld.step` 的完整调用链：建世界 → 插恒星(固定)+行星(卫星) → 推进 → 回读。
/// 任何一步返回 0/null 都会在这里显形（尤其是 `satellite_builder` 内部 panic 被 ffi_guard 吞掉）。
#[test]
fn cosmos_gravity_source_insert_and_readback_end_to_end() {
    #[cfg(test)]
    use mps_cosmos::ffi::{
        cosmos_body_linvel_out, cosmos_body_translation_out, cosmos_fixed_body_builder,
        cosmos_satellite_builder, cosmos_world_create, cosmos_world_insert_body_as_gravity_source,
        cosmos_world_step,
    };
    use mps_formula::ffi::Vec3;

    let world = cosmos_world_create(0.01, 4, 1, 1, 1, 1.0e6);
    assert!(!world.is_null(), "cosmos_world_create 返回 null");

    let sun_builder = cosmos_fixed_body_builder(0.0, 0.0, 0.0);
    assert!(!sun_builder.is_null(), "fixed_body_builder 返回 null");
    let sun = cosmos_world_insert_body_as_gravity_source(world, sun_builder, 1.99e30);
    assert_ne!(sun, 0, "恒星插入失败（返回 0）");

    let earth_builder = cosmos_satellite_builder(
        5.97e24,
        1.53e10,
        1.0876018e7,
        -1.47e11,
        30185.0,
        1.90,
        2951.0,
        6.371e6,
    );
    assert!(
        !earth_builder.is_null(),
        "satellite_builder 返回 null（内部 panic）"
    );
    let earth = cosmos_world_insert_body_as_gravity_source(world, earth_builder, 5.97e24);
    assert_ne!(earth, 0, "行星插入失败（返回 0）");

    let mut out = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    let rc = cosmos_body_translation_out(world, earth, &mut out as *mut Vec3);
    assert_eq!(rc, 1, "插入后立刻回读失败");
    let start = (out.x, out.y, out.z);

    for _ in 0..2000 {
        cosmos_world_step(world, 0.01);
    }

    let rc = cosmos_body_translation_out(world, earth, &mut out as *mut Vec3);
    assert_eq!(rc, 1, "step 之后回读失败");
    let mut vel = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    let rc_vel = cosmos_body_linvel_out(world, earth, &mut vel as *mut Vec3);
    assert_eq!(rc_vel, 1, "step 之后线速度回读失败");

    println!(
        "start=({:.3e},{:.3e},{:.3e}) now=({:.3e},{:.3e},{:.3e}) v=({:.3e},{:.3e},{:.3e})",
        start.0, start.1, start.2, out.x, out.y, out.z, vel.x, vel.y, vel.z
    );
    assert!((out.x, out.y, out.z) != start, "位置没变：轨道没推进");

    // 20 秒：太阳引力 a = G·M☉/r² ≈ 6.08e-3 m/s² → |Δv| ≈ 0.12 m/s。
    // 恒星如果是"固定体但在 refresh 里被 gm=0 覆盖"，这里 Δv 就会是 0。
    let dv = ((vel.x - 30185.0).powi(2) + (vel.y - 1.90).powi(2) + (vel.z - 2951.0).powi(2)).sqrt();
    println!("Δv(20s) = {:.5} m/s", dv);
    assert!(
        dv > 0.05,
        "20 秒内几乎没有速度变化（Δv={dv}）：太阳引力没生效"
    );
}
