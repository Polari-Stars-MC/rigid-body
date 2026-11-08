//! 轨道动力学只读诊断量（平均运动共振 / 偏心率矢量 / Kozai–Lidov 周期）。
//!
//! 全部为**纯函数**，复用 `mps_formula` 的物理原语（GM、引力常数），对给定
//! 状态（位置 / 速度 / 半长轴 / 质量）做瞬时（osculating）诊断。**不改任何积分
//! 路径、不写回世界态**——因此不影响现有仿真输出（满足「原方法不变」铁律），
//! 仅作可观测性导出。

use mps_formula::celestial_data::G;
use rapier3d::prelude::Vector;

/// 平运动 `n = sqrt(GM / a³)`（rad/s）。`a>0` 且 `gm>0` 才有意义，否则返回 0。
#[inline]
pub fn mean_motion(gm: f64, a: f64) -> f64 {
    if gm <= 0.0 || a <= 0.0 || !gm.is_finite() || !a.is_finite() {
        return 0.0;
    }
    (gm / a.powi(3)).sqrt()
}

/// 两轨道平运动比 `n1/n2 = (gm1/a1³)^½ / (gm2/a2³)^½`。
///
/// 用于**平均运动共振（MMR）**判定：若比值接近小整数比 `p:q`（如 2:1、3:2、
/// 4:3），系统落入共振区，长周期摄动显著。任一参数非法返回 0。
#[inline]
pub fn mean_motion_ratio(gm1: f64, a1: f64, gm2: f64, a2: f64) -> f64 {
    let n1 = mean_motion(gm1, a1);
    let n2 = mean_motion(gm2, a2);
    if n2 <= 0.0 || !n2.is_finite() {
        return 0.0;
    }
    n1 / n2
}

/// Laplace–Runge–Lenz（偏心率）矢量 `e_vec = (v × h)/GM − r̂`，其中
/// `h = r × v` 为比角动量（specific angular momentum）。
///
/// 其模长即偏心率 `e`，方向指向近心点。只读诊断，不改积分。输入非法
/// （`r=0` / `h=0` / `gm≤0`）返回零向量。
#[inline]
pub fn eccentricity_vector(position: Vector, velocity: Vector, gm: f64) -> Vector {
    let r = position;
    let r_mag = (r.x * r.x + r.y * r.y + r.z * r.z).sqrt();
    if r_mag < 1e-12_f64 || gm <= 0.0 || !gm.is_finite() {
        return Vector::ZERO;
    }
    let h = r.cross(velocity); // 比角动量
    let h_mag = (h.x * h.x + h.y * h.y + h.z * h.z).sqrt();
    if h_mag < 1e-20_f64 {
        return Vector::ZERO;
    }
    let v_cross_h = velocity.cross(h);
    let r_hat = r / r_mag;
    v_cross_h / gm - r_hat
}

/// Kozai–Lidov 周期（离心率振荡特征时长，秒）。标准式：
///
/// `P_KL = (2/(3π)) · ((M1+M3)/M3) · (a_out/a_in)³ · (1−e_out²)^{3/2} · P_out`
///
/// 其中 `P_out = 2π·sqrt(a_out³ / (G·M3))` 为第三体绕内双星质心的周期
/// （vis-viva 周期）。输入：内体半长轴 `a_in`、外体半长轴 `a_out`、外体离心率
/// `e_out`（须 ∈ [0, 1)）、内主星质量 `M1`（须有限且为正）、第三体质量 `M3`。
/// 任一参数非法返回 0（文档化的失败值）。
/// 只读诊断，不改积分。
#[inline]
pub fn kozai_period(a_in: f64, a_out: f64, e_out: f64, m_primary: f64, m_tertiary: f64) -> f64 {
    if a_in <= 0.0
        || a_out <= 0.0
        || m_primary <= 0.0
        || m_tertiary <= 0.0
        || !a_in.is_finite()
        || !a_out.is_finite()
        || !m_primary.is_finite()
        || !m_tertiary.is_finite()
        || !e_out.is_finite()
        || !(0.0..1.0).contains(&e_out)
    {
        return 0.0;
    }
    let p_out = 2.0 * std::f64::consts::PI * (a_out.powi(3) / (G * m_tertiary)).sqrt();
    let ratio = (m_primary + m_tertiary) / m_tertiary;
    let a_term = (a_out / a_in).powi(3);
    let e_term = (1.0 - e_out * e_out).powf(1.5);
    (2.0 / (3.0 * std::f64::consts::PI)) * ratio * a_term * e_term * p_out
}
