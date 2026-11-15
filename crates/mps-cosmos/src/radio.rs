//! 星际无线电传播（Kelvin 空间坐标系，f64）。
//!
//! 与 `CosmosWorld` 松耦合：传播所需的"反射天体"直接来自 cosmos 世界的
//! 刚体集合（位置 f64 原生，无需 Java 复制）；收发器（天线）状态由 Java
//! 经共享缓冲批量提交，传播结果同样批量回读。
//!
//! 模型（与 Java 侧旧实现等价，全部 double）：
//! - 发射出去是一条**球面波前**，按 [`WAVE_SPEED`] 往外扩；波前扫过哪个天线，
//!   就给哪个天线投递一次（同一条波能先后到达远近不同的端，不会因为"有人收到"就消失）；
//! - 波前强度按 1/d² 衰减，掉到 [`NOISE_FLOOR`]（背景噪声）以下才销毁；
//! - 每个天线判定时走直射 + 天体球面镜面反射，最多 [`MAX_BOUNCES`] 次；
//! - 跳过源天线所在天体与目标天线所在天体；
//! - 投递能量按路径长度 1/d² 衰减，反射乘 [`REFLECT_ATTENUATION`]；
//! - 多普勒频移按收发相对速度投影；
//! - 定向天线按波束锥角过滤；接收端按方向给增益（正对满系数、越偏越低、出锥 0）。

use rapier3d::prelude::{RigidBodyHandle, Vector};

/// 最大反射次数
pub const MAX_BOUNCES: u32 = 3;
/// 每次反射能量保留比
pub const REFLECT_ATTENUATION: f64 = 0.8;
/// 命中判定最小距离（m）
pub const HIT_EPS: f64 = 1.0e-6;
/// 天文单位（m），写波速用的。
pub const AU: f64 = 1.495_978_707e11;
/// 默认波速（m/s）：**100 AU/s**（比光速快得多，跨星系通讯不至于等十几分钟）。
/// 实际用的是 [`RadioWorld::set_wave_speed`] 设进去的值（Java 建完世界传），
/// 没设过才用这个默认值。决定波前什么时候到达某个天线。
pub const WAVE_SPEED: f64 = 100.0 * AU;
/// 背景噪声底（W）：波前强度掉到它以下就销毁。
/// 射程 = [`max_range`] = sqrt(能量 / 噪声底)：100 W 全向约 3.16e11 m（≈2.11 AU，
/// 按 100 AU/s 飞 21 ms）。
pub const NOISE_FLOOR: f64 = 1.0e-21;
/// 灵敏度兜底下限（W）
pub const MIN_SENSITIVITY: f64 = 1.0e-15;

/// 强度掉到背景噪声以下时，波前走到的距离（米）：`sqrt(能量 / 噪声底)`。
pub fn max_range(energy: f64) -> f64 {
    (energy.max(0.0) / NOISE_FLOOR).sqrt()
}

/// 从 Java 缓冲里读出来的波束半张角：非法值（没写、NaN、负数）一律当全向。
pub fn sanitize_beam_angle(value: f64) -> f64 {
    if value.is_finite() && value > 0.0 {
        value.min(std::f64::consts::PI)
    } else {
        std::f64::consts::PI
    }
}

/// 波前已经扩到多远（米）：波速 × 年龄。
fn wave_front(signal: &ActiveSignal, now_ms: u64, wave_speed: f64) -> f64 {
    (now_ms.saturating_sub(signal.birth_ms) as f64 / 1000.0) * wave_speed
}

/// 波前追上接收端要多久（秒）：解 `|d0 + v·t| = c·t`。
///
/// 展开成 `(v²−c²)t² + 2(v·d0)t + d0² = 0`，取最小的正根；
/// 波速永远大于接收端速度，所以正根一定存在（除非波速被设成 ≤ |v|）。
/// 静止（v=0）时就是 `|d0| / c`。
fn catch_up_time(origin: Vector, rx_pos: Vector, rx_vel: Vector, speed: f64) -> Option<f64> {
    let d0 = rx_pos - origin;
    let a = rx_vel.length_squared() - speed * speed;
    let b = 2.0 * rx_vel.dot(d0);
    let c0 = d0.length_squared();

    let roots = if a.abs() <= 1.0e-12 {
        // 退化：接收端速度刚好等于波速，退成一次方程
        if b.abs() <= 1.0e-12 {
            None
        } else {
            Some(-c0 / b)
        }
    } else {
        let disc = (b * b - 4.0 * a * c0).max(0.0);
        let root = disc.sqrt();
        let t1 = (-b - root) / (2.0 * a);
        let t2 = (-b + root) / (2.0 * a);
        match (t1 > 0.0, t2 > 0.0) {
            (true, true) => Some(t1.min(t2)),
            (true, false) => Some(t1),
            (false, true) => Some(t2),
            (false, false) => None,
        }
    };
    roots.filter(|t| t.is_finite() && *t > 0.0)
}

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
    /// 发射波束半张角（弧度，π = 全向）
    pub beam_angle: f64,
    /// 接收波束半张角（弧度，π = 全向）：来的方向不在这个锥里就不收
    pub rx_beam_angle: f64,
    /// 天线是否属于某刚体（飞船），用于跳过来自飞船自身的反射干扰
    pub owner_body: Option<u64>,
}

impl RadioNode {
    pub fn directional(&self) -> bool {
        self.tx_gain > 1.0 && self.beam_angle < std::f64::consts::PI
    }

    /// 接收方向增益：`系数 × cos(夹角 / 半张角 × π/2)`。
    ///
    /// - `系数` 就是 `rx_gain`（Java 按天线类型给，全向天线 1、方向性天线/dish 更大）；
    /// - 来的方向越偏离天线指向（`dir`），夹角越大、增益越低，正对时满增益；
    /// - 夹角超过 `rx_beam_angle` 直接返回 0：出了接收锥就不收。
    pub fn rx_gain_factor(&self, from: Vector) -> f64 {
        NodeView::of(self).gain_from(from)
    }
}

/// 一步之内不变的节点量：方向先归一化、"接收锥的 cos"先算好，
/// 这样每对判定就不用重复归一化、也不用为了"在不在锥里"再 acos 一次。
#[derive(Clone, Copy)]
struct NodeView {
    id: u64,
    pos: Vector,
    vel: Vector,
    /// 已归一化的天线指向；零向量表示没给方向（当全向）。
    axis: Vector,
    /// 峰值系数（Java 给的 rx_gain）。
    coeff: f64,
    /// 接收半张角（π = 全向）。
    beam: f64,
    /// cos(beam)，锥内判定直接比这个。
    cos_beam: f64,
    omni: bool,
    sensitivity: f64,
    owner_body: Option<u64>,
}

impl NodeView {
    fn of(node: &RadioNode) -> Self {
        let omni = node.rx_beam_angle >= std::f64::consts::PI;
        Self {
            id: node.id,
            pos: node.pos,
            vel: node.vel,
            axis: node.dir.try_normalize().unwrap_or(Vector::ZERO),
            coeff: node.rx_gain.max(0.0),
            beam: if omni {
                std::f64::consts::PI
            } else {
                node.rx_beam_angle
            },
            cos_beam: if omni { -1.0 } else { node.rx_beam_angle.cos() },
            omni,
            sensitivity: node.sensitivity,
            owner_body: node.owner_body,
        }
    }

    /// 信号从 `from` 过来，这个台的接收方向增益；0 = 出了接收锥不收。
    fn gain_from(&self, from: Vector) -> f64 {
        if self.omni || self.axis == Vector::ZERO {
            return self.coeff;
        }
        let Some(incoming) = (from - self.pos).try_normalize() else {
            return self.coeff;
        };
        let dot = incoming.dot(self.axis).clamp(-1.0, 1.0);
        if dot < self.cos_beam {
            return 0.0; // 角度外不接受（比 cos，省一次 acos）
        }
        let t = (dot.acos() / self.beam).clamp(0.0, 1.0);
        self.coeff * (t * std::f64::consts::FRAC_PI_2).cos()
    }
}

/// 活跃信号（发射端状态已在此快照，传播期间不再变化）。
///
/// 传播不用"每步把所有接收端扫一遍"，而是发射时就把**最快到达时刻**解析算出来
/// （直线距离 / 波速）排成时刻表，物理刻走到哪就判哪几条。
#[derive(Clone, Debug)]
pub struct ActiveSignal {
    pub id: u64,
    pub birth_ms: u64,
    pub origin: Vector,
    pub origin_vel: Vector,
    pub origin_dir: Vector,
    pub frequency: f64,
    pub energy: f64,
    pub tx_gain: f64,
    pub beam_angle: f64,
    pub owner_body: Option<u64>,
    /// 到达时刻表，按 `at_ms` 升序（发射时才建，见 [`RadioWorld::build_schedule`]）。
    pub schedule: Vec<Arrival>,
    /// 时刻表处理到哪一条了。
    pub cursor: usize,
}

/// 到达时刻表的一条：这条波最快什么时候能到某个接收端。
#[derive(Clone, Copy, Debug)]
pub struct Arrival {
    /// 最早可能到达的墙钟时刻（ms）= 出生时刻 + 直线距离 / 波速。
    pub at_ms: u64,
    pub node_id: u64,
    /// 建表时节点在 `nodes` 里的下标（到点时先用它，id 对不上再线性找）。
    pub index: usize,
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
    /// 波速（m/s）。Java 建完世界会通过 FFI 覆盖；没设过就用 [`WAVE_SPEED`]。
    wave_speed: f64,
}

impl RadioWorld {
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            signals: Vec::new(),
            reflectors: Vec::new(),
            results: Vec::new(),
            last_step_ms: 0,
            wave_speed: WAVE_SPEED,
        }
    }

    /// 设置波速（m/s）。非法值（NaN / ≤0）回落到默认 [`WAVE_SPEED`]。
    /// 已经飞出去、还没开始判定的波会按新波速重排时刻表；判定过的不动。
    pub fn set_wave_speed(&mut self, speed: f64) {
        self.wave_speed = if speed.is_finite() && speed > 0.0 {
            speed
        } else {
            WAVE_SPEED
        };
        for i in 0..self.signals.len() {
            if self.signals[i].cursor != 0 {
                continue; // 已经判过一些了，半路换波速不重排，免得重复投递
            }
            let schedule = self.build_schedule(&self.signals[i]);
            self.signals[i].schedule = schedule;
        }
    }

    pub fn wave_speed(&self) -> f64 {
        self.wave_speed
    }

    /// 解析出"这条波最快什么时候到每个接收端"。
    ///
    /// 不是拿发射那一刻的直线距离去除波速——接收端在动，波前是**追上**它的：
    /// 解 `|d0 + v·t| = c·t`（d0 = 发射时的相对位置，v = 接收端快照速度，c = 波速），
    /// 取最小的正根，就是波前追上的时刻。静止时退化成 `|d0| / c`。
    /// 追上的那一刻波前一共走了 `c·t`，超过噪声底射程就说明追到之前波已经烧完，不进表。
    fn build_schedule(&self, signal: &ActiveSignal) -> Vec<Arrival> {
        let speed = self.wave_speed.max(1.0);
        let reach = max_range(signal.energy);
        let mut schedule: Vec<Arrival> = Vec::with_capacity(self.nodes.len());
        for (index, node) in self.nodes.iter().enumerate() {
            let Some(catch_time) = catch_up_time(signal.origin, node.pos, node.vel, speed) else {
                continue;
            };
            if speed * catch_time > reach {
                continue; // 追上之前波就烧完了
            }
            let at_ms = signal.birth_ms + (catch_time * 1000.0).ceil() as u64;
            schedule.push(Arrival {
                at_ms,
                node_id: node.id,
                index,
            });
        }
        schedule.sort_unstable_by_key(|a| a.at_ms);
        schedule
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
        let mut signal = signal;
        // 发射时就把到达时刻表建好：之后每步只处理"到点了"的那几条
        signal.schedule = self.build_schedule(&signal);
        signal.cursor = 0;
        // 同 id（同一发射事件）只保留最新一条
        if let Some(existing) = self.signals.iter_mut().find(|s| s.id == signal.id) {
            *existing = signal;
        } else {
            self.signals.push(signal);
        }
    }

    // ==================== 传播 ====================

    /// 执行一轮波前推进（在天体 step 之后调用，反射天体位置为最新）。
    /// `now_ms` 由调用方（世界 step）提供，用来算波前扩到哪了。
    pub fn step(&mut self, bodies: &[ReflectorState], now_ms: u64) {
        self.results.clear();

        // 1. 波前强度掉到背景噪声以下就销毁（射程 = sqrt(能量 / 噪声底)）。
        //    没人听的时候也得走这一步，不然波会一直挂在表里。
        //    但时刻表里还有没判定的接收端就先留着：射程内的到点时刻一定早于烧毁时刻，
        //    只是整数毫秒量化后两者可能落在同一步，先烧掉就会把该收的全吞了。
        let wave_speed = self.wave_speed;
        self.signals.retain(|s| {
            s.cursor < s.schedule.len()
                || s.energy * dist_att(wave_front(s, now_ms, wave_speed)) >= NOISE_FLOOR
        });

        if self.signals.is_empty() || self.nodes.is_empty() {
            self.last_step_ms = now_ms;
            return;
        }

        // 2. 时刻表到点了才判定：每条波只处理 `at_ms <= now` 的那几条。
        //    同一条波先到近端后到远端，天然就是这么排的；每步不再扫全部接收端。
        for i in 0..self.signals.len() {
            // 发射端波束也提前算：定向才需要判，锥角 cos 只算一次
            let tx_directive =
                self.signals[i].tx_gain > 1.0 && self.signals[i].beam_angle < std::f64::consts::PI;
            let tx_cos_beam = self.signals[i]
                .beam_angle
                .clamp(0.0, std::f64::consts::PI)
                .cos();
            let mut arrived: Vec<RadioResult> = Vec::new();
            let mut cursor = self.signals[i].cursor;
            while cursor < self.signals[i].schedule.len()
                && self.signals[i].schedule[cursor].at_ms <= now_ms
            {
                let arrival = self.signals[i].schedule[cursor];
                cursor += 1;
                // 先按建表时的下标取（快路径），节点表换过位置再按 id 找
                let node = match self.nodes.get(arrival.index) {
                    Some(node) if node.id == arrival.node_id => node,
                    _ => match self.nodes.iter().find(|n| n.id == arrival.node_id) {
                        Some(node) => node,
                        None => continue, // 这个接收端已经没了
                    },
                };
                let rx = NodeView::of(node);
                if let Some(p) =
                    self.trace_path(&self.signals[i], &rx, bodies, tx_directive, tx_cos_beam)
                {
                    arrived.push(p);
                }
            }
            self.signals[i].cursor = cursor;
            self.results.append(&mut arrived);
        }

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
        rx: &NodeView,
        bodies: &[ReflectorState],
        tx_directive: bool,
        tx_cos_beam: f64,
    ) -> Option<RadioResult> {
        let floor = rx.sensitivity.max(MIN_SENSITIVITY);

        // 接收方向增益：正对来波满增益，越来越偏越来越低，出了接收锥就是 0（不收）
        let direct_gain = rx.gain_from(signal.origin);
        if direct_gain <= 0.0 {
            return None;
        }

        let to_rx = rx.pos - signal.origin;
        let distance = to_rx.length();
        if distance <= HIT_EPS {
            return None;
        }

        // 发射端波束判定：发射端 → 接收端方向是否在波束内（比 cos，省一次 acos）
        if tx_directive {
            let cos_axis = (to_rx.dot(signal.origin_dir) / distance).clamp(-1.0, 1.0);
            if cos_axis < tx_cos_beam {
                return None;
            }
        }

        // 直射的最强可能功率都收不到 → 连遮挡都不用算
        if signal.energy * dist_att(distance) * direct_gain <= floor {
            return None;
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
        let direct_blocker =
            first_blocker_idx(signal.origin, rx.pos, bodies, source_body, target_body);
        if direct_blocker.is_none() {
            return self.finish(signal, rx, distance, 1.0, direct_gain);
        }

        // 2) 直射被挡 → 经第三个天体 M（不是挡路天体本身）的球面镜面反射一次
        //    反射点 P ∈ 球面，满足 ∠(origin→P, P→M) == ∠(P→M, P→rx)
        //    在 A、B、M 三点所在平面内解圆上反射（2D 问题）。
        let mut best: Option<(f64, f64, f64)> = None; // (path_len, att, 接收增益)
        for (i, r) in bodies.iter().enumerate() {
            if source_body == Some(i) || target_body == Some(i) || direct_blocker == Some(i) {
                continue; // 源/目标所在天体 + 挡路天体 不当作反射面
            }
            // 预筛：方向角 / 强度都够不着的天体直接跳过，不做那 360 点粗扫
            if !planet_may_reflect(signal, tx_directive, rx, r, floor) {
                continue;
            }
            if let Some((p_len, att, point)) =
                sphere_reflect_path(signal.origin, rx.pos, r.pos, r.radius, bodies, i)
            {
                // 反射进来的方向按反射点算，同样过接收方向增益
                let gain = rx.gain_from(point);
                if gain <= 0.0 {
                    continue;
                }
                if best.is_none_or(|(bl, _, _)| p_len < bl) {
                    best = Some((p_len, att, gain));
                }
            }
        }

        let (path_len, reflect_att, rx_gain) = best?;
        self.finish(signal, rx, path_len, reflect_att, rx_gain)
    }

    fn finish(
        &self,
        signal: &ActiveSignal,
        rx: &NodeView,
        path_len: f64,
        reflect_att: f64,
        rx_gain: f64,
    ) -> Option<RadioResult> {
        let received = signal.energy * dist_att(path_len) * reflect_att * rx_gain;
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
        // `total_cmp` 保证 NaN 也有全序，恶意/坏输入不会在 FFI 边界 panic。
        .min_by(|x, y| x.total_cmp(y))
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
        .filter_map(|(i, r)| ray_sphere_hit(a, dir, max_dist, r.pos, r.radius).map(|t| (i, t)))
        .min_by(|(_, x), (_, y)| x.total_cmp(y))
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

/// 球面反射粗扫的 (cos, sin) 采样表：均匀采样，全局算一次，所有调用共用。
/// 原来每个采样点都现算 sin/cos，一次反射搜索就是几百次三角函数。
const SCAN_SAMPLES: usize = 128;
static SCAN_TRIG: std::sync::OnceLock<[(f64, f64); SCAN_SAMPLES]> = std::sync::OnceLock::new();

fn scan_trig() -> &'static [(f64, f64); SCAN_SAMPLES] {
    SCAN_TRIG.get_or_init(|| {
        let mut table = [(0.0f64, 0.0f64); SCAN_SAMPLES];
        for (i, slot) in table.iter_mut().enumerate() {
            let theta = -std::f64::consts::PI
                + 2.0 * std::f64::consts::PI * (i as f64) / (SCAN_SAMPLES as f64);
            *slot = (theta.cos(), theta.sin());
        }
        table
    })
}

/// 预筛：这颗天体有没有可能把这条波反射到这个接收端。
/// 三个判据都很便宜（方向角 / 强度 / 波够不够得着），不过就跳过整段反射搜索
/// （那是 360 点粗扫 + 每段二分，单次几微秒）。
fn planet_may_reflect(
    signal: &ActiveSignal,
    tx_directive: bool,
    rx: &NodeView,
    planet: &ReflectorState,
    floor: f64,
) -> bool {
    let to_planet = planet.pos - signal.origin;
    let d1 = to_planet.length();
    if d1 <= planet.radius {
        return false; // 波源就在球里，谈不上反射
    }
    // 1) 强度：波走到这颗天体时已经低于背景噪声 → 波到不了它
    if signal.energy * dist_att(d1) < NOISE_FLOOR {
        return false;
    }
    // 2) 强度：反射最短路径（三角不等式 d1+d2 是下界）都过不了接收门限 → 没戏
    let d2 = (rx.pos - planet.pos).length();
    if signal.energy * dist_att(d1 + d2) * REFLECT_ATTENUATION * rx.coeff <= floor {
        return false;
    }
    // 3) 方向角：定向发射时，整颗天体都在波束外 → 波前扫不到它
    if tx_directive {
        let cos_axis = (to_planet.dot(signal.origin_dir) / d1).clamp(-1.0, 1.0);
        let half = (planet.radius / d1).clamp(0.0, 1.0).asin();
        let limit = (signal.beam_angle + half)
            .clamp(0.0, std::f64::consts::PI)
            .cos();
        if cos_axis < limit {
            return false;
        }
    }
    true
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
) -> Option<(f64, f64, Vector)> {
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
    let g_at = |c: f64, s: f64| -> f64 {
        let p = center + (e1 * c + e2 * s) * radius;
        let normal = (p - center).try_normalize();
        let Some(normal) = normal else {
            return f64::MAX;
        };
        let in_dir = (a - p).try_normalize();
        let out_dir = (b - p).try_normalize();
        let (Some(in_dir), Some(out_dir)) = (in_dir, out_dir) else {
            return f64::MAX;
        };
        // 入射/出射必须从反射面外侧（可见半球内）
        if in_dir.dot(normal) <= 1e-9 || out_dir.dot(normal) <= 1e-9 {
            return f64::MAX;
        }
        // 反射角相等 ⟺ 两个夹角的 cos 相等（cos 在 [0,π] 单调），直接比 dot，省两次 acos
        in_dir.dot(normal) - out_dir.dot(normal)
    };
    let g = |theta: f64| -> f64 { g_at(theta.cos(), theta.sin()) };

    // 粗扫定位变号区间，然后二分精化。
    // 128 点（2.8°）足够框住根区间，二分 30 次已经把区间压到 1e-9 rad 以下，
    // 再密的粗扫只是白算 g()（每次还带两次归一化）。
    let table = scan_trig();
    let mut best_theta: Option<f64> = None;
    let mut prev_theta = -std::f64::consts::PI;
    let mut prev_g = g(prev_theta);
    for i in 1..=SCAN_SAMPLES {
        let (c, s) = table[i % SCAN_SAMPLES];
        let theta =
            -std::f64::consts::PI + 2.0 * std::f64::consts::PI * (i as f64) / (SCAN_SAMPLES as f64);
        let cur_g = g_at(c, s);
        if prev_g.is_finite() && cur_g.is_finite() && prev_g * cur_g <= 0.0 {
            // 区间 [prev_theta, theta] 内二分
            let mut lo = prev_theta;
            let mut hi = theta;
            let mut glo = prev_g;
            for _ in 0..30 {
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
            if best_theta.is_none_or(|bt| g(theta_opt).abs() < g(bt).abs()) {
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
    let err = (in_dir.dot(normal) - out_dir.dot(normal)).abs();
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

    Some((path_len, REFLECT_ATTENUATION, p))
}

/// 距离衰减：1/d²（游戏链路模型，不做频段相关的弗里斯损耗）
fn dist_att(distance: f64) -> f64 {
    if distance <= 1.0 {
        1.0
    } else {
        1.0 / (distance * distance)
    }
}

// 单元测试已迁到 mps-test 镜像文件 crates/mps-test/src/cosmos/radio.rs。
// （仓库约定：源 crate 不放 #[cfg(test)] 模块，见 AGENTS.md）。
