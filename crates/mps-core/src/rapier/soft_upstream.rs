//! 官方 `SoftBody` 后端：把 mps 的软体授权数据镜像到 rapier v0.36 的软体上求解。
//!
//! 迁移目标：软体的**求解**只有一个实现（官方 XPBD 元件 + 可变形碰撞体），mps 侧保留
//! **授权**（增量 `add_particle` / `add_spring` / `add_triangle` / `add_tetrahedron`）与
//! **查询**（粒子、能量、体积、应力）。89 个 `soft_body_*` C ABI 签名一个都不变。
//!
//! ## 每步的两次同步
//!
//! ```text
//! prepare():  拓扑变了就重建官方体 → 位姿/速度推过去 → 扩展项写进 fork_extras → 每体重力
//! pipeline.step(...)                      ← 官方求解（弹性/体积/接触/CCD）
//! collect():  官方体的解写回授权数据      ← C ABI 的读接口拿到的就是本步结果
//! ```
//!
//! ## 扩展项的落点
//!
//! fork 独有的本构（风、气压、体级阻尼、温度场、粘弹性、各向异性、主动应变、内聚、
//! 子步、撕裂三准则）已经在 rapier 里融合进官方求解路径（`dynamics::soft_body::
//! fork_extras`），这里只负责把 mps 的字段翻译成设置调用。

use std::collections::HashMap;

use crate::rapier::soft_data::{
    SoftBody as ForkSoftBody, SoftBodyId, SoftBodySet as ForkSoftBodySet, TearCriterion,
};
use rapier3d::prelude::SpringCoefficients;
use rapier3d::math::Real;
use rapier3d::prelude::{
    CohesionParams, ColliderSet, ForkTearCriterion, ImpulseJointSet, IslandManager,
    MultibodyJointSet, RigidBodySet, SoftBodyBuilder, SoftBodyHandle, SoftBodySet, ThermalParams,
    Vector, ViscoelasticParams, Wind,
};

/// 软体求解后端。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) enum SoftBodyBackend {
    /// fork 自研积分器（历史路径，逐步被官方后端取代）。
    Fork,
    /// rapier 官方 `SoftBody` 求解（默认：迁移目标）。
    #[default]
    Upstream,
}

impl SoftBodyBackend {
    /// C ABI 模式值：`0` = fork，`1` = upstream。
    pub(crate) fn from_ffi(mode: u32) -> Option<Self> {
        match mode {
            0 => Some(Self::Fork),
            1 => Some(Self::Upstream),
            _ => None,
        }
    }
}

/// 一个授权体在官方 `SoftBodySet` 里的镜像。
#[derive(Clone, Copy)]
pub(crate) struct UpstreamSlot {
    pub(crate) handle: SoftBodyHandle,
    /// 建体时的拓扑指纹：变了才重建（位置/速度每帧都变，不参与）。
    topology: u64,
}

fn mix(h: &mut u64, v: u64) {
    *h ^= v;
    *h = h.wrapping_mul(0x1000_0000_01b3);
}

/// 拓扑指纹：粒子数、边、约束、面、体的数量与索引。
fn topology_signature(body: &ForkSoftBody) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    mix(&mut h, body.particles.len() as u64);
    mix(&mut h, body.springs.len() as u64);
    mix(&mut h, body.distance_constraints.len() as u64);
    mix(&mut h, body.triangles.len() as u64);
    mix(&mut h, body.tetrahedra.len() as u64);
    for t in &body.triangles {
        mix(&mut h, t[0] as u64);
        mix(&mut h, t[1] as u64);
        mix(&mut h, t[2] as u64);
    }
    for t in &body.tetrahedra {
        for i in t {
            mix(&mut h, *i as u64);
        }
    }
    h
}

/// 从授权数据建一个官方软体。
///
/// 边 = 弹簧 ∪ 距离约束的端点对（两者描述同一批结构边，只是不同求解器用）；有三角面时
/// 同时注册表面，让官方可变形碰撞体生效；没有面拓扑的粒子云不建表面碰撞体。
fn build(
    body: &ForkSoftBody,
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    set: &mut SoftBodySet,
) -> SoftBodyHandle {
    let positions: Vec<Vector> = body.particles.iter().map(|p| p.pos).collect();

    let mut edges = Vec::with_capacity(body.springs.len() + body.distance_constraints.len());
    for s in &body.springs {
        edges.push([s.a as u32, s.b as u32]);
    }
    for c in &body.distance_constraints {
        edges.push([c.a as u32, c.b as u32]);
    }

    let pinned: Vec<u32> = body
        .particles
        .iter()
        .enumerate()
        .filter(|(_, p)| p.inv_mass == 0.0)
        .map(|(i, _)| i as u32)
        .collect();

    // Mirror the fork's per-particle masses. The official solver computes
    // acceleration as `force × official_inv_mass`, and `prepare` pushes
    // fork-mass-scaled forces (`gravity · m_fork`); if the official body kept
    // the builder default (mass 1.0), gravity would be under-applied by the
    // mass ratio (a 0.02-mass balloon would feel g/50). Pinned particles
    // report mass 0 in the fork data; their value is irrelevant (they are
    // pinned via `pinned_particles`), so fill them with the free-particle mean
    // to keep the builder's all-positive finite-masses validation happy.
    let mut masses: Vec<Real> = body.particles.iter().map(|p| p.mass()).collect();
    let mean_free_mass = {
        let free: Vec<Real> = masses
            .iter()
            .copied()
            .filter(|m| m.is_finite() && *m > 0.0)
            .collect();
        if free.is_empty() {
            1.0
        } else {
            free.iter().sum::<Real>() / free.len() as Real
        }
    };
    for m in &mut masses {
        if !m.is_finite() || *m <= 0.0 {
            *m = mean_free_mass;
        }
    }

    let mut builder = SoftBodyBuilder::new(positions)
        .edges(edges)
        .pinned_particles(pinned)
        .masses(masses)
        // 每个软体可以有自己的重力：官方只认世界重力 × gravity_scale，所以这里关掉，
        // 由 `prepare` 每步按 `body.gravity` 折算成粒子力。
        .gravity_scale(0.0)
        .can_sleep(true);

    if body.triangles.is_empty() {
        builder = builder.no_surface_collider();
    } else {
        builder = builder.surface(body.triangles.clone());
    }

    set.insert(builder, bodies, colliders)
}

/// 把授权数据里的扩展项翻译成 rapier 的 `fork_extras` 设置。
fn sync_extras(sim: &mut rapier3d::prelude::SoftBody, body: &ForkSoftBody) {
    sim.set_wind_field(body.wind.map(|w| Wind {
        accel: w.accel,
        drag: w.drag,
    }));
    // Phase 11 内压：官方求解器直接消费 `fork.pressure`（边界面压力），
    // 这里必须逐帧同步（`soft_body_set_pressure` 只写授权数据）。
    sim.set_internal_pressure(body.pressure);
    let _ = sim.set_fork_damping(body.damping.clamp(0.0, 0.999));
    let _ = sim.set_thermal_field(body.temperature.map(|t| ThermalParams {
        temp: t.temp,
        ambient: t.ambient,
        expansion: t.expansion,
        stiffness_temp_coeff: t.stiffness_temp_coeff,
    }));
    let _ = sim.set_viscoelastic_layer(body.viscoelastic.map(|v| ViscoelasticParams {
        rate_coefficient: v.rate_coefficient,
    }));
    let _ = sim.set_anisotropy_axes(body.anisotropy);
    let _ = sim.set_cohesion_glue(body.cohesion.map(|c| CohesionParams {
        radius: c.radius,
        stiffness: c.stiffness,
        break_distance: c.break_distance,
    }));
    let _ = sim.set_extra_substeps(body.substeps.max(1));
    sim.set_tear_criterion(body.tear.map(|t| match t {
        TearCriterion::Strain(threshold) => ForkTearCriterion::Strain(threshold),
        TearCriterion::Stress(threshold) => ForkTearCriterion::Stress(threshold),
        TearCriterion::Energy(threshold) => ForkTearCriterion::Energy(threshold),
    }));
}

/// 步进前：按需重建、推进位姿/速度与每体重力。
#[allow(clippy::too_many_arguments)]
pub(crate) fn prepare(
    fork: &mut ForkSoftBodySet,
    set: &mut SoftBodySet,
    slots: &mut HashMap<u32, UpstreamSlot>,
    islands: &mut IslandManager,
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    impulse_joints: &mut ImpulseJointSet,
    multibody_joints: &mut MultibodyJointSet,
) {
    // 先清掉已经不在授权集合里的镜像（`soft_body_destroy` 只动授权数据）。
    if !slots.is_empty() {
        let stale: Vec<u32> = slots
            .keys()
            .copied()
            .filter(|id| fork.get(SoftBodyId(*id)).is_none())
            .collect();
        for id in stale {
            if let Some(slot) = slots.remove(&id)
                && set.contains(slot.handle)
            {
                set.remove(
                    slot.handle,
                    islands,
                    bodies,
                    colliders,
                    impulse_joints,
                    multibody_joints,
                );
            }
        }
    }

    for id in 0..fork.len() {
        let Some(body) = fork.get_mut(SoftBodyId(id as u32)) else {
            continue;
        };
        let signature = topology_signature(body);

        let rebuild = match slots.get(&(id as u32)) {
            Some(slot) => slot.topology != signature || !set.contains(slot.handle),
            None => true,
        };
        if rebuild {
            if let Some(slot) = slots.get(&(id as u32))
                && set.contains(slot.handle)
            {
                set.remove(
                    slot.handle,
                    islands,
                    bodies,
                    colliders,
                    impulse_joints,
                    multibody_joints,
                );
            }
            let handle = build(body, bodies, colliders, set);
            slots.insert(
                id as u32,
                UpstreamSlot {
                    handle,
                    topology: signature,
                },
            );
        }

        let Some(handle) = slots.get(&(id as u32)).map(|s| s.handle) else {
            continue;
        };
        // 官方软体参与求解的前提（见 `collect_awake_bodies`）：根刚体必须在活动岛里、
        // 不睡眠、已启用。mps 的软体没有碰撞体，岛不会自然把它算进来，所以每步显式唤醒。
        if !body.sleeping
            && let Some(root) = set.get(handle).map(|s| s.root_body())
            && let Some(rb) = bodies.get_mut(root)
        {
            rb.set_enabled(true);
            rb.wake_up(true);
        }
        let Some(sim) = set.get_mut(handle) else {
            continue;
        };



        // 拓扑/外部参数：每次全量同步（都是廉价的字段赋值与 wake）。
        sync_extras(sim, body);

        // 位姿/速度：授权数据是真相源。只在重建时整批推：每步回推等于把官方求解结果
        // 当作“用户瞬移”覆盖掉，软体会被钉在原地。
        sim.reset_forces(false);
        let n = sim.num_particles().min(body.particles.len());
        if rebuild {
            for i in 0..n {
                let p = &body.particles[i];
                sim.set_particle_position(i, p.pos);
                sim.set_particle_velocity(i, p.vel);
            }
            // 绑定到刚体的粒子：官方有自己的 attachment（锚点按当前位姿算局部坐标），
            // 重建时注册一次即可。
            for i in 0..n {
                if let Some(anchor) = body.particles[i].bound_body {
                    sim.attach_particle(i, anchor, &*bodies);
                }
            }
        }

        // 每体重力：官方 g 已关，这里按 fork 的 `gravity` 逐粒子施加。
        let gravity = body.gravity;
        for i in 0..n {
            let inv_mass = body.particles[i].inv_mass;
            if inv_mass <= 0.0 {
                continue;
            }
            let mass = 1.0 / inv_mass;
            sim.add_particle_force(i, gravity * mass, false);
        }

        // 逐边静止长度：把每个弹簧/距离约束自己的 activation（主动应变）折算成官方边的
        // rest length（`rest·(1 − γ)`）。官方的边顺序与 `build` 里一致：先 `springs`
        // 后 `distance_constraints`，所以按下标直接对上。
        let base = body.springs.len();
        for (k, s) in body.springs.iter().enumerate() {
            sim.set_edge_rest_length(k, s.rest_length * (1.0 - s.activation.clamp(0.0, 1.0)));
        }
        for (k, c) in body.distance_constraints.iter().enumerate() {
            sim.set_edge_rest_length(base + k, c.rest * (1.0 - c.activation.clamp(0.0, 1.0)));
        }

        // 逐边刚度：fork 用的是 `Spring.stiffness`（N/m）与 `DistanceConstraint.compliance`
        // （XPBD α），官方边用的是 `SpringCoefficients`（固有频率 Hz + 阻尼比）。这里做单调
        // 换算：`f = √(k/m)/2π`；α 则取 `f = 1/(2π√(α·m))`，α→0 视为刚性（高频）。
        for (k, s) in body.springs.iter().enumerate() {
            let m = edge_mass(body, s.a, s.b);
            let f = (s.stiffness / m).sqrt() / (2.0 * std::f64::consts::PI);
            sim.set_edge_softness(k, Some(softness(f)));
        }
        for (k, c) in body.distance_constraints.iter().enumerate() {
            let m = edge_mass(body, c.a, c.b);
            let f = if c.compliance <= 1.0e-9 {
                RIGID_FREQUENCY
            } else {
                1.0 / (2.0 * std::f64::consts::PI * (c.compliance * m).sqrt())
            };
            sim.set_edge_softness(base + k, Some(softness(f)));
        }

        // 授权数据的 `sleeping` 直接映射到官方的 enabled 开关：官方有自己的睡眠
        // 体系（island + 根刚体），这里只保证“标记为睡着的软体不被推进”。
        sim.set_enabled(!body.sleeping);
        // 授权数据是真相源：每步显式唤醒，否则官方会按根刚体的睡眠状态跳过求解。
        if !body.sleeping {
            sim.wake_up();
        }
    }
}

/// 步进后：把官方体的解写回授权数据（C ABI 的读接口读授权数据）。
pub(crate) fn collect(
    fork: &mut ForkSoftBodySet,
    set: &SoftBodySet,
    slots: &HashMap<u32, UpstreamSlot>,
) {
    for (id, slot) in slots {
        let Some(sim) = set.get(slot.handle) else {
            continue;
        };
        let Some(body) = fork.get_mut(SoftBodyId(*id)) else {
            continue;
        };
        let n = sim.num_particles().min(body.particles.len());
        for i in 0..n {
            body.particles[i].pos = sim.particle_position(i);
            body.particles[i].vel = sim.particle_velocity(i);
        }
        // The authorized `sleeping` flag is the source of truth (set by
        // `soft_body_sleep`/`soft_body_wake`); never clear it from the mirror — a
        // disabled official body reports `is_sleeping() == false`, which would
        // otherwise clobber a forced sleep and re-enable the body next step.
        // Natural (energy-based) sleep only ever *sets* the flag here.
        body.sleeping = body.sleeping || sim.is_sleeping();
    }
}

/// 逐边换算用的阻尼比：接近临界阻尼，避免引入额外振荡。
const EDGE_DAMPING_RATIO: Real = 0.7;
/// 视为“刚性”的固有频率（Hz）。
const RIGID_FREQUENCY: Real = 120.0;

/// 一条边的等效质量：两端粒子质量的平均值。
fn edge_mass(body: &ForkSoftBody, a: usize, b: usize) -> Real {
    let ma = body.particles.get(a).map(|p| p.mass()).unwrap_or(1.0);
    let mb = body.particles.get(b).map(|p| p.mass()).unwrap_or(1.0);
    ((ma + mb) * 0.5).max(1.0e-9)
}

/// 把固有频率折成官方边的软度系数（频率夹在合理区间内）。
fn softness(frequency: Real) -> SpringCoefficients<Real> {
    SpringCoefficients {
        natural_frequency: frequency.clamp(0.1, RIGID_FREQUENCY),
        damping_ratio: EDGE_DAMPING_RATIO,
    }
}
