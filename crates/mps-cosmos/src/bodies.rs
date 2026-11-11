//! 太空刚体构造 helper。
//!
//! 对常用航天器参数（质量、初始位置/速度、可选姿态）做轻量包装，内部
//! 调用 `rapier3d::prelude::RigidBodyBuilder::dynamic()` + 设置附加质量
//! 与平移/线速度。返回 `RigidBodyBuilder` 仍可链式进一步配置后交给
//! [`crate::CosmosWorld::insert_body`]。

use rapier3d::prelude::{MassProperties, RigidBodyBuilder, Vector};

/// C ABI 不透明 builder 句柄（对应 `cosmos.h` 里的 `CosmosRigidBodyBuilder *`）。
///
/// cbindgen 以 `parse_deps = true`（include `mps-formula`）运行，只为**本 crate
/// 内定义**的函数生成声明：此前 `extern "C"` 签名直接用 rapier 的
/// `RigidBodyBuilder`，头文件里就会出现没有任何 typedef 的裸 `RigidBodyBuilder *`，
/// `cosmos.h` 无法独立编译。本类型仿照 mps-core 的 `RigidBodyBuilderHandle`
/// （无 `repr` 属性 + 私有 `inner` 字段 → cbindgen 只发
/// `typedef struct CosmosRigidBodyBuilder CosmosRigidBodyBuilder;`）。
///
/// 不加 `#[repr(transparent)]`（否则 cbindgen 会穿透单字段结构，把头文件里的
/// typedef 发成指向未定义 `RigidBodyBuilder` 的别名，`cosmos.h` 又不能独立
/// 编译）。单字段 repr(Rust) 结构与字段本身同大小/对齐（与 mps-core 的
/// `RigidBodyBuilderHandle` 同一先例）：JNI 侧（mps-jni）仍按
/// `Box<RigidBodyBuilder>` 分配 builder 并把裸指针当 jlong 传递，与 ffi 侧
/// `Box<CosmosRigidBodyBuilder>` 的裸指针布局一致、可安全互转——两条路径
/// 创建的 builder 句柄在 ABI 上仍然是同一份内存。
pub struct CosmosRigidBodyBuilder {
    inner: RigidBodyBuilder,
}

impl CosmosRigidBodyBuilder {
    pub fn new(inner: RigidBodyBuilder) -> Self {
        Self { inner }
    }

    pub fn as_inner(&self) -> &RigidBodyBuilder {
        &self.inner
    }

    pub fn as_inner_mut(&mut self) -> &mut RigidBodyBuilder {
        &mut self.inner
    }

    pub fn into_inner(self) -> RigidBodyBuilder {
        self.inner
    }
}

impl std::fmt::Debug for CosmosRigidBodyBuilder {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CosmosRigidBodyBuilder")
            .finish_non_exhaustive()
    }
}

/// 构造一个动态刚体 builder，给定质量（kg）、初始位置与速度（SI 单位）。
///
/// 惯性张量设为均匀球体 `I = 2/5 · m · r²`，其中 `r` 由 `radius` 给出
/// （仅用于估算惯性，不创建 collider）。设 `radius <= 0` 时惯量取小正值
/// 避免奇异。
pub fn satellite_builder(
    mass: f64,
    position: Vector,
    velocity: Vector,
    radius: f64,
) -> RigidBodyBuilder {
    let inertia = if radius > 0.0 && mass > 0.0 {
        (2.0 / 5.0) * mass * radius * radius
    } else {
        1.0e-3 // 退化：极小但非零惯量
    };
    RigidBodyBuilder::dynamic()
        // rapier 0.35 从\"线速度阈值\"改成\"姿态漂移\"判 sleep——空跑的卫星在
        // 大 `dt` 下会单步触发 sleep 并把 linvel 清零，与项目对\"永恒漂浮天体\"
        // 的语义冲突。对刚发射的卫星永远禁用 sleep，归属它们的速度完全由我们
        // 的积子或 Rapier 力律决定（fixed_body_builder 仍可 sleep，但 fixed 体不
        // 受 sleep 影响速度）。
        .can_sleep(false)
        .additional_mass_properties(MassProperties::new(
            Vector::ZERO,
            mass,
            Vector::splat(inertia),
        ))
        .translation(position)
        .linvel(velocity)
}

/// 构造一个固定（静态）刚体 builder，仅指定位置 —— 适合作为 n-body 引力源
/// 中心（如恒星/行星本体），不参与动态推进。
pub fn fixed_body_builder(position: Vector) -> RigidBodyBuilder {
    RigidBodyBuilder::fixed().translation(position)
}
