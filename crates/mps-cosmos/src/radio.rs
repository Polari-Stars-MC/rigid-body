//! 星际无线电传播（Kelvin 空间坐标系，f64）。
//!
//! 与 `CosmosWorld` 松耦合：传播所需的"反射天体"直接来自 cosmos 世界的
//! 刚体集合（位置 f64 原生，无需 Java 复制）；收发器（天线）状态由 Java
//! 经共享缓冲批量提交，传播结果同样批量回读。
//!
//! 模型（与 Java 侧旧实现等价，全部 double）：
//! - 直射 + 天体球面镜面反射，最多 [`MAX_BOUNCES`] 次；
//! - 跳过源天线所在天体与目标天线所在天体；
//! - 能量按路径长度 1/d² 衰减，反射乘 [`REFLECT_ATTENUATION`]；
//! - 多普勒频移按收发相对速度投影；
//! - 定向天线按波束锥角过滤。

use rapier3d::prelude::{RigidBodyHandle, Vector};

/// 最大反射次数
pub const MAX_BOUNCES: u32 = 3;
/// 每次反射能量保留比
pub const REFLECT_ATTENUATION: f64 = 0.8;
/// 命中判定最小距离（m）
pub const HIT_EPS: f64 = 1.0e-6;
/// 信号最长存活时间（ms）
pub const SIGNAL_TTL_MS: u64 = 5000;
/// 灵敏度兜底下限（W）
pub const MIN_SENSITIVITY: f64 = 1.0e-15;

/// 天线（收发器）槽位 —— Java 提交的完整状态。
#[derive(Clone, Debug)]
pub struct RadioNode {
    pub id: u64,
    pub pos: Vector,
    pub vel: Vector,
    pub dir: Vector,
    pub frequency: f64,
    pub power: f64,
    pub sensitivity: f64,
    pub rx_gain: f64,
    pub tx_gain: f64,
    pub beam_angle: f64,
    /// 天线是否属于某刚体（飞船），用于跳过来自飞船自身的反射干扰
    pub owner_body: Option<u64>,
}

impl RadioNode {
    pub fn directional(&self) -> bool {
        self.tx_gain > 1.0 && self.beam_angle < std::f64::consts::PI
    }
}

/// 活跃信号（发射端状态已在此快照，传播期间不再变化）。
#[derive(Clone, Debug)]
pub struct ActiveSignal {
    pub id: u64,
    pub tx_node_id: u64,
    pub birth_ms: u64,
    pub origin: Vector,
    pub origin_vel: Vector,
    pub origin_dir: Vector,
    pub frequency: f64,
    pub energy: f64,
    pub tx_gain: f64,
    pub beam_angle: f64,
    pub owner_body: Option<u64>,
}

/// 反射天体 —— 来自 CosmosWorld 刚体（位置每次 step 实时读）。
#[derive(Clone, Copy, Debug)]
pub struct Reflector {
    pub handle: RigidBodyHandle,
    pub radius: f64,
}

/// 传播用反射体（句柄 + 最新位置 + 半径）。
#[derive(Clone, Copy, Debug)]
pub struct ReflectorState {
    pub handle_raw: u64,
    pub pos: Vector,
    pub radius: f64,
}

/// 传播结果（写入 Java 回读缓冲）。
#[derive(Clone, Copy, Debug)]
pub struct RadioResult {
    pub signal_id: u64,
    pub rx_node_id: u64,
    pub received_power: f64,
    pub received_frequency: f64,
}

/// 无线电传播世界。挂接在 `CosmosWorld` 上（Java 每 SpaceWorld 一个）。
pub struct RadioWorld {
    nodes: Vec<RadioNode>,
    signals: Vec<ActiveSignal>,
    pub(crate) reflectors: Vec<Reflector>,
    results: Vec<RadioResult>,
    /// 上一步的时间戳（用于清理过期信号）
    last_step_ms: u64,
}

impl RadioWorld {
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            signals: Vec::new(),
            reflectors: Vec::new(),
            results: Vec::new(),
            last_step_ms: 0,
        }
    }

    // ==================== 注册/注销 ====================

    pub fn register_node(&mut self, node: RadioNode) {
        if let Some(slot) = self.nodes.iter_mut().find(|n| n.id == node.id) {
            *slot = node;
        } else {
            self.nodes.push(node);
        }
    }

    /// 整表覆盖：以本次提交为准（Java 每 tick 提交全部在线收发器）。
    pub fn set_nodes(&mut self, nodes: Vec<RadioNode>) {
        self.nodes = nodes;
    }

    pub fn unregister_node(&mut self, id: u64) {
        self.nodes.retain(|n| n.id != id);
    }

    pub fn set_reflectors(&mut self, reflectors: Vec<Reflector>) {
        self.reflectors = reflectors;
    }

    pub fn add_reflector(&mut self, handle: RigidBodyHandle, radius: f64) {
        if !self.reflectors.iter().any(|r| r.handle == handle) {
            self.reflectors.push(Reflector { handle, radius });
        }
    }

    pub fn remove_reflector(&mut self, handle: RigidBodyHandle) {
        self.reflectors.retain(|r| r.handle != handle);
    }

    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    pub fn signal_count(&self) -> usize {
        self.signals.len()
    }

    pub fn reflector_count(&self) -> usize {
        self.reflectors.len()
    }

    pub fn reflectors(&self) -> &[Reflector] {
        &self.reflectors
    }

    // ==================== 信号 ====================

    pub fn submit_signal(&mut self, signal: ActiveSignal) {
        // 同 id（同一发射事件）只保留最新一条
        if let Some(existing) = self.signals.iter_mut().find(|s| s.id == signal.id) {
            *existing = signal;
        } else {
            self.signals.push(signal);
        }
    }

    // ==================== 传播 ====================

    /// 执行一轮传播（在天体 step 之后调用，反射天体位置为最新）。
    /// `now_ms` 用于信号过期清理；由调用方（世界 step）提供。
    pub fn step(&mut self, bodies: &[ReflectorState], now_ms: u64) {
        self.results.clear();

        // 1. 过期信号清理
        self.signals
            .retain(|s| now_ms.saturating_sub(s.birth_ms) <= SIGNAL_TTL_MS);

        if self.signals.is_empty() || self.nodes.is_empty() {
            self.last_step_ms = now_ms;
            return;
        }

        // 2. 每个信号 × 每个接收节点；一旦信号成功投递给任一接收端，
        //    就从活跃列表移除（同 tick 会广播给所有可达端）。
        //    未投递（暂时无人可达）的信号保留到 TTL 由步骤 1 清理。
        let mut to_remove: Vec<u64> = Vec::new();
        for signal in &self.signals {
            let mut delivered = false;
            for rx in &self.nodes {
                if rx.id == signal.tx_node_id {
                    continue;
                }
                if let Some(p) = self.trace_path(signal, rx, bodies) {
                    self.results.push(p);
                    delivered = true;
                }
            }
            if delivered {
                to_remove.push(signal.id);
            }
        }
        self.signals.retain(|s| !to_remove.contains(&s.id));

        self.last_step_ms = now_ms;
    }

    pub fn node_ids(&self) -> impl Iterator<Item = u64> + '_ {
        self.nodes.iter().map(|n| n.id)
    }

    pub fn signal_ids(&self) -> impl Iterator<Item = u64> + '_ {
        self.signals.iter().map(|s| s.id)
    }

    pub fn take_results(&mut self) -> Vec<RadioResult> {
        std::mem::take(&mut self.results)
    }

    // ==================== 路径追踪 ====================

    fn trace_path(
        &self,
        signal: &ActiveSignal,
        rx: &RadioNode,
        bodies: &[ReflectorState],
    ) -> Option<RadioResult> {
        // 波束判定：发射端 → 接收端方向是否在波束内
        if signal.tx_gain > 1.0 {
            let to_rx = (rx.pos - signal.origin).try_normalize()?;
            let dot = to_rx.dot(signal.origin_dir).clamp(-1.0, 1.0);
            if dot.acos() > signal.beam_angle {
                return None;
            }
        }

        // 源/目标各自所在的天体（天线在行星表面时该行星不遮挡自身视线）。
        // 以 owner_body（刚体句柄）优先；退化时按“位置在球内”兜底。
        let source_body = signal
            .owner_body
            .and_then(|h| bodies.iter().position(|r| r.handle_raw == h))
            .or_else(|| {
                bodies
                    .iter()
                    .position(|r| signal.origin.distance(r.pos) <= r.radius * 1.02)
            });
        let target_body = rx
            .owner_body
            .and_then(|h| bodies.iter().position(|r| r.handle_raw == h))
            .or_else(|| {
                bodies
                    .iter()
                    .position(|r| rx.pos.distance(r.pos) <= r.radius * 1.02)
            });

        // 1) 尝试直射（跳过源/目标所在天体）；记录首个挡路天体
        let direct_blocker = first_blocker_idx(
            signal.origin,
            rx.pos,
            bodies,
            source_body,
            target_body,
        );
        if direct_blocker.is_none() {
            let path_len = signal.origin.distance(rx.pos);
            return self.finish(signal, rx, path_len, 1.0);
        }

        // 2) 直射被挡 → 经第三个天体 M（不是挡路天体本身）的球面镜面反射一次
        //    反射点 P ∈ 球面，满足 ∠(origin→P, P→M) == ∠(P→M, P→rx)
        //    在 A、B、M 三点所在平面内解圆上反射（2D 问题）。
        let mut best: Option<(f64, f64)> = None; // (path_len, att)
        for (i, r) in bodies.iter().enumerate() {
            if source_body == Some(i) || target_body == Some(i) || direct_blocker == Some(i) {
                continue; // 源/目标所在天体 + 挡路天体 不当作反射面
            }
            if let Some((p_len, att)) = sphere_reflect_path(
                signal.origin,
                rx.pos,
                r.pos,
                r.radius,
                bodies,
                i,
            ) {
                if best.map_or(true, |(bl, _)| p_len < bl) {
                    best = Some((p_len, att));
                }
            }
        }

        let (path_len, reflect_att) = best?;
        self.finish(signal, rx, path_len, reflect_att)
    }

    fn finish(
        &self,
        signal: &ActiveSignal,
        rx: &RadioNode,
        path_len: f64,
        reflect_att: f64,
    ) -> Option<RadioResult> {
        let received = signal.energy * dist_att(path_len) * reflect_att * rx.rx_gain;
        if received <= rx.sensitivity.max(MIN_SENSITIVITY) {
            return None;
        }
        let los = (rx.pos - signal.origin).try_normalize()?;
        let v_rel = rx.vel.dot(los) - signal.origin_vel.dot(los);
        let freq = signal.frequency * (1.0 + v_rel / 299_792_458.0);
        Some(RadioResult {
            signal_id: signal.id,
            rx_node_id: rx.id,
            received_power: received,
            received_frequency: freq.max(0.01),
        })
    }
}

impl Default for RadioWorld {
    fn default() -> Self {
        Self::new()
    }
}

/// 射线 A→B 是否被某个天体（跳过 skip 集合）截断；返回最近命中距离。
fn first_blocker(
    a: Vector,
    b: Vector,
    bodies: &[ReflectorState],
    skip_source: Option<usize>,
    skip_target: Option<usize>,
) -> Option<f64> {
    let dir = (b - a).try_normalize()?;
    let max_dist = a.distance(b);
    bodies
        .iter()
        .enumerate()
        .filter(|(i, _)| Some(*i) != skip_source && Some(*i) != skip_target)
        .filter_map(|(_, r)| ray_sphere_hit(a, dir, max_dist, r.pos, r.radius))
        .min_by(|x, y| x.partial_cmp(y).unwrap())
}

/// 返回首个截断 A→B 的天体索引（无截断返回 None）。
fn first_blocker_idx(
    a: Vector,
    b: Vector,
    bodies: &[ReflectorState],
    skip_source: Option<usize>,
    skip_target: Option<usize>,
) -> Option<usize> {
    let dir = (b - a).try_normalize()?;
    let max_dist = a.distance(b);
    bodies
        .iter()
        .enumerate()
        .filter(|(i, _)| Some(*i) != skip_source && Some(*i) != skip_target)
        .filter_map(|(i, r)| {
            ray_sphere_hit(a, dir, max_dist, r.pos, r.radius).map(|t| (i, t))
        })
        .min_by(|(_, x), (_, y)| x.partial_cmp(y).unwrap())
        .map(|(i, _)| i)
}

/// 射线与球面相交：返回 t（> HIT_EPS 且 < max_dist）。
fn ray_sphere_hit(
    a: Vector,
    dir: Vector,
    max_dist: f64,
    center: Vector,
    radius: f64,
) -> Option<f64> {
    let oc = a - center;
    let b = oc.dot(dir);
    let c = oc.length_squared() - radius * radius;
    let disc = b * b - c;
    if disc < 0.0 {
        return None;
    }
    let sqrt = disc.sqrt();
    let mut t = -b - sqrt;
    if t <= HIT_EPS {
        t = -b + sqrt;
    }
    if t > HIT_EPS && t < max_dist {
        Some(t)
    } else {
        None
    }
}

/// 经球面 M 的一次镜面反射路径（3D 球面镜像法近似）：
/// 反射点 P 在 A、B、M 平面内的球面上，满足反射角相等。
/// 在平面内用角度二分逼近反射点，然后验证 A→P 与 P→B 均不被其它天体截断。
fn sphere_reflect_path(
    a: Vector,
    b: Vector,
    center: Vector,
    radius: f64,
    bodies: &[ReflectorState],
    mirror_idx: usize,
) -> Option<(f64, f64)> {
    let va = a - center;
    let vb = b - center;
    let da = va.length();
    let db = vb.length();
    if da <= radius || db <= radius {
        return None;
    }

    // A、B、M 三点平面内求解球面反射点 P。
    // 反射定律（入射角=反射角）等价于：P 在球面上且
    //   (A−P)·(B−P) 与 P 的位置满足切线对称条件。
    // 平面内用角度二分：取参考方向 e1 = A 方向，法向 e3 = e1×B方向，
    // e2 = e3×e1。设 P(θ) = center + (e1·cosθ + e2·sinθ)·radius。
    // 反射角差 f(θ) = angle(in, normal) − angle(out, normal) 单调过零。
    // 用扫描+分段二分（数值稳健，成本可忽略：反射体数量很少）。

    let e1 = va / da;
    let e3 = va.cross(vb).try_normalize()?;
    let e2 = e3.cross(e1).try_normalize()?;

    // 反射点朝向约束：P 必须同时被 A 与 B “看见”的前半球
    // 前半球（对 A）：A·P 方向上 (P−A) 与 (P−M) 点积 < 0 无意义，直接算。
    // 反射角差函数 g(θ)：找到使入射角 == 出射角的 θ
    let g = |theta: f64| -> f64 {
        let p = center + (e1 * theta.cos() + e2 * theta.sin()) * radius;
        let normal = (p - center).try_normalize();
        let Some(normal) = normal else { return f64::MAX; };
        let in_dir = (a - p).try_normalize();
        let out_dir = (b - p).try_normalize();
        let (Some(in_dir), Some(out_dir)) = (in_dir, out_dir) else {
            return f64::MAX;
        };
        // 入射/出射必须从反射面外侧（可见半球内）
        if in_dir.dot(normal) <= 1e-9 || out_dir.dot(normal) <= 1e-9 {
            return f64::MAX;
        }
        in_dir.dot(normal).acos() - out_dir.dot(normal).acos()
    };

    // 粗扫定位变号区间，然后二分精化
    let samples = 360;
    let mut best_theta: Option<f64> = None;
    let mut prev_theta = -std::f64::consts::PI;
    let mut prev_g = g(prev_theta);
    for i in 1..=samples {
        let theta = -std::f64::consts::PI + 2.0 * std::f64::consts::PI * (i as f64) / (samples as f64);
        let cur_g = g(theta);
        if prev_g.is_finite() && cur_g.is_finite() && prev_g * cur_g <= 0.0 {
            // 区间 [prev_theta, theta] 内二分
            let mut lo = prev_theta;
            let mut hi = theta;
            let mut glo = prev_g;
            for _ in 0..60 {
                let mid = 0.5 * (lo + hi);
                let gmid = g(mid);
                if !gmid.is_finite() {
                    break;
                }
                if glo * gmid <= 0.0 {
                    hi = mid;
                } else {
                    lo = mid;
                    glo = gmid;
                }
            }
            let theta_opt = 0.5 * (lo + hi);
            if best_theta.map_or(true, |bt| g(theta_opt).abs() < g(bt).abs()) {
                best_theta = Some(theta_opt);
            }
        }
        prev_theta = theta;
        prev_g = cur_g;
    }

    let theta = best_theta?;
    let p = center + (e1 * theta.cos() + e2 * theta.sin()) * radius;
    let normal = (p - center).try_normalize()?;
    let in_dir = (a - p).try_normalize()?;
    let out_dir = (b - p).try_normalize()?;
    let err = (in_dir.dot(normal).acos() - out_dir.dot(normal).acos()).abs();
    if err > 1e-5 {
        return None;
    }
    let path_len = a.distance(p) + p.distance(b);

    // 验证两段均不被其它天体截断
    let skip_mirror = Some(mirror_idx);
    let src_block = first_blocker(a, p, bodies, None, skip_mirror);
    if src_block.is_some() {
        return None;
    }
    let dst_block = first_blocker(p, b, bodies, skip_mirror, None);
    if dst_block.is_some() {
        return None;
    }

    Some((path_len, REFLECT_ATTENUATION))
}

/// 距离衰减：1/d²（游戏链路模型，不做频段相关的弗里斯损耗）
fn dist_att(distance: f64) -> f64 {
    if distance <= 1.0 {
        1.0
    } else {
        1.0 / (distance * distance)
    }
}

// ---------------------------------------------------------------------------
// 单元测试
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn node(id: u64, pos: Vector) -> RadioNode {
        RadioNode {
            id,
            pos,
            vel: Vector::ZERO,
            dir: Vector::new(0.0, 0.0, 1.0),
            frequency: 2.4e9,
            power: 50.0,
            sensitivity: 1.0e-15,
            rx_gain: 1.0,
            tx_gain: 1.0,
            beam_angle: std::f64::consts::PI,
            owner_body: None,
        }
    }

    fn signal(tx_id: u64, tx_pos: Vector) -> ActiveSignal {
        ActiveSignal {
            id: tx_id * 100 + 1,
            tx_node_id: tx_id,
            birth_ms: 0,
            origin: tx_pos,
            origin_vel: Vector::ZERO,
            origin_dir: Vector::new(1.0, 0.0, 0.0),
            frequency: 2.4e9,
            energy: 50.0,
            tx_gain: 1.0,
            beam_angle: std::f64::consts::PI,
            owner_body: None,
        }
    }

    fn reflector(handle: u64, pos: Vector, radius: f64) -> ReflectorState {
        ReflectorState {
            handle_raw: handle,
            pos,
            radius,
        }
    }

    #[test]
    fn direct_line_of_sight_reaches() {
        let mut radio = RadioWorld::new();
        radio.register_node(node(1, Vector::new(0.0, 0.0, 0.0)));
        radio.register_node(node(2, Vector::new(1000.0, 0.0, 0.0)));
        radio.submit_signal(ActiveSignal {
            id: 1,
            tx_node_id: 1,
            birth_ms: 0,
            origin: Vector::new(0.0, 0.0, 0.0),
            origin_vel: Vector::ZERO,
            origin_dir: Vector::new(1.0, 0.0, 0.0),
            frequency: 2.4e9,
            energy: 50.0,
            tx_gain: 1.0,
            beam_angle: std::f64::consts::PI,
            owner_body: None,
        });

        // 无天体遮挡 → 直达
        radio.step(&[], 1000);
        let results = radio.take_results();
        assert_eq!(results.len(), 1);
        assert_eq!(results[0].rx_node_id, 2);
        assert!(results[0].received_power > 0.0);
    }

    #[test]
    fn celestial_body_blocks_direct_path_but_third_body_reflects() {
        // 场景：大行星挡在 A→B 直线上（直接不可达），
        // 但 A、B 都能"看到"旁边的小反射体 M，经 M 一次反射应可达。
        let mut radio = RadioWorld::new();
        // A 与 B 在大行星两侧（行星半径 4000，A/B 距原点 8000）→ 直线穿过行星被挡
        radio.register_node(node(1, Vector::new(-8000.0, 0.0, 0.0)));
        radio.register_node(node(2, Vector::new(8000.0, 0.0, 0.0)));
        radio.submit_signal(ActiveSignal {
            id: 1,
            tx_node_id: 1,
            birth_ms: 0,
            origin: Vector::new(-8000.0, 0.0, 0.0),
            origin_vel: Vector::ZERO,
            origin_dir: Vector::new(1.0, 0.0, 0.0),
            frequency: 2.4e9,
            energy: 50.0,
            tx_gain: 1.0,
            beam_angle: std::f64::consts::PI,
            owner_body: None,
        });

        // 挡路大行星在原点；反射体 M 在侧面高处，A/B 均可见
        let bodies = vec![
            ReflectorState {
                handle_raw: 1,
                pos: Vector::ZERO,
                radius: 4000.0,
            },
            ReflectorState {
                handle_raw: 2,
                pos: Vector::new(0.0, 30000.0, 0.0),
                radius: 1500.0,
            },
        ];
        radio.step(&bodies, 1000);
        let results = radio.take_results();
        // 反射体在 y=20000，A/B 视线均不穿过它 → 反射路径应可达
        assert!(
            !results.is_empty(),
            "expected a reflected path to reach receiver, got none"
        );
        assert!(results[0].received_power > 0.0);
    }

    #[test]
    fn planet_blocks_direct_path_no_reflector_unreachable() {
        // 挡路行星在中间，无反射体 → 必须不可达
        let mut radio = RadioWorld::new();
        radio.register_node(node(1, Vector::new(-8000.0, 0.0, 0.0)));
        radio.register_node(node(2, Vector::new(8000.0, 0.0, 0.0)));
        radio.submit_signal(signal(1, Vector::new(-8000.0, 0.0, 0.0)));

        let bodies = vec![reflector(1, Vector::ZERO, 4000.0)];
        radio.step(&bodies, 1000);
        let results = radio.take_results();
        assert!(results.is_empty(), "blocked path must not reach");
    }

    #[test]
    fn sensitivity_filters_weak_signal() {
        let mut radio = RadioWorld::new();
        // 1000 米、50W：1/d² = 1e-6 → 5e-5 W，超过默认灵敏度 1e-15
        // 但这里把灵敏度设成 1.0 W → 应被滤掉
        let mut rx = node(2, Vector::new(1000.0, 0.0, 0.0));
        rx.sensitivity = 1.0;
        radio.register_node(node(1, Vector::ZERO));
        radio.register_node(rx);
        radio.submit_signal(signal(1, Vector::ZERO));

        radio.step(&[], 1000);
        assert!(radio.take_results().is_empty(), "weak signal must be filtered");
    }

    #[test]
    fn directional_beam_excludes_out_of_cone() {
        let mut radio = RadioWorld::new();
        let mut tx = node(1, Vector::ZERO);
        tx.tx_gain = 10.0;
        tx.beam_angle = 0.1; // 约 5.7°，朝 +X
        radio.register_node(tx);
        // 接收端在 +Y 方向，明显在波束外
        radio.register_node(node(2, Vector::new(0.0, 1000.0, 0.0)));
        let mut s = signal(1, Vector::ZERO);
        s.tx_gain = 10.0;
        s.beam_angle = 0.1;
        radio.submit_signal(s);

        radio.step(&[], 1000);
        assert!(radio.take_results().is_empty(), "out-of-cone must not reach");
    }

    #[test]
    fn doppler_shift_moves_frequency() {
        let mut radio = RadioWorld::new();
        radio.register_node(node(1, Vector::ZERO));
        let mut rx = node(2, Vector::new(1.0e7, 0.0, 0.0));
        rx.vel = Vector::new(3000.0, 0.0, 0.0); // 朝发射端飞
        radio.register_node(rx);
        radio.submit_signal(signal(1, Vector::ZERO));

        radio.step(&[], 1000);
        let results = radio.take_results();
        assert_eq!(results.len(), 1);
        // v_rel = 3000（向源运动为正→频率升高？源静止，接收端向源：v_rel = -3000）
        // 公式：f * (1 + v_rel/c)。接收端沿 -X 运动 → v_rel = -3000 → 频率下降
        let f = results[0].received_frequency;
        assert!((f - 2.4e9).abs() > 1.0, "frequency should shift, got {f}");
    }

    #[test]
    fn signal_expires_after_ttl() {
        let mut radio = RadioWorld::new();
        radio.register_node(node(1, Vector::ZERO));
        radio.register_node(node(2, Vector::new(1000.0, 0.0, 0.0)));
        let mut s = signal(1, Vector::ZERO);
        s.birth_ms = 0;
        radio.submit_signal(s);

        // now = 6000ms > TTL 5000 → 过期移除
        radio.step(&[], 6000);
        assert!(radio.take_results().is_empty());
        assert_eq!(radio.signal_count(), 0);
    }

    #[test]
    fn signal_delivered_once_then_removed() {
        let mut radio = RadioWorld::new();
        radio.register_node(node(1, Vector::ZERO));
        radio.register_node(node(2, Vector::new(1000.0, 0.0, 0.0)));
        radio.submit_signal(signal(1, Vector::ZERO));

        // 第一步投递成功 → 信号移除
        radio.step(&[], 1000);
        assert_eq!(radio.take_results().len(), 1);
        assert_eq!(radio.signal_count(), 0);

        // 第二步不再投递
        radio.step(&[], 2000);
        assert!(radio.take_results().is_empty());
    }
}
