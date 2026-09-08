//! Daniel Bernoulli —— 贡献目录与公式实现。
//!
//! 本文件收录该科学家的里程碑式贡献，并承载其名下的公式实现
//! （从原 `mps-formula` 域模块迁移而来；实现在此，域模块仅作
//! `pub use` 重导出以保持 FFI/ABI 不变）。不引入 Rapier / `WorldHandle`。

use super::ScientistRecord;

/// 本科学家的贡献记录。
#[allow(dead_code)]
pub const SCIENTIST: ScientistRecord = ScientistRecord {
    id: "daniel_bernoulli",
    name: "Daniel Bernoulli",
    birth_year: Some(1700),
    death_year: Some(1782),
    field_id: "fluid",
    nationality: "Swiss",
    contribution: "Bernoulli's principle of fluid dynamics",
    key_constants: "Bernoulli equation",
};

/// 该科学家名下的公式实现（从各域模块迁移而来）。
pub mod formulas {
    use crate::domain::{self, FormulaError};
    use crate::ffi::*;
    fn aero_surface_valid(surface: &AeroSurface) -> bool {
        vec3_finite(surface.point)
            && vec3_finite(surface.normal)
            && surface.area.is_finite()
            && surface.drag_coefficient.is_finite()
            && surface.lift_coefficient.is_finite()
            && surface.area > 0.0
            && surface.drag_coefficient >= 0.0
            && surface.lift_coefficient >= 0.0
    }

    /// Bernoulli static pressure.
    pub fn bernoulli_pressure(
        total_pressure: f64,
        density: f64,
        velocity: f64,
        gravity: f64,
        elevation: f64,
    ) -> f64 {
        bernoulli_pressure_checked(total_pressure, density, velocity, gravity, elevation)
            .unwrap_or(f64::NAN)
    }

    /// Checked static pressure. Density must be positive and speed nonnegative;
    /// signed pressure, gravity and elevation are allowed. All inputs and
    /// results must be finite. Zero gravity is valid for pressure calculation.
    pub fn bernoulli_pressure_checked(
        total_pressure: f64,
        density: f64,
        velocity: f64,
        gravity: f64,
        elevation: f64,
    ) -> Result<f64, FormulaError> {
        domain::finite(total_pressure, "total_pressure")?;
        domain::positive(density, "density")?;
        domain::nonnegative(velocity, "velocity")?;
        domain::finite(gravity, "gravity")?;
        domain::finite(elevation, "elevation")?;
        domain::finite_result(
            total_pressure - 0.5 * density * velocity * velocity - density * gravity * elevation,
            "static pressure",
        )
    }

    /// Bernoulli report.
    pub fn bernoulli_report(
        pressure: f64,
        density: f64,
        velocity: f64,
        gravity: f64,
        elevation: f64,
    ) -> Option<BernoulliReport> {
        bernoulli_report_checked(pressure, density, velocity, gravity, elevation).ok()
    }

    /// Checked Bernoulli report. Unlike static pressure, total head divides by
    /// gravity, so zero gravity (of either sign) is a domain error. No infinite
    /// head is returned. Density is positive, speed nonnegative, all values finite.
    pub fn bernoulli_report_checked(
        pressure: f64,
        density: f64,
        velocity: f64,
        gravity: f64,
        elevation: f64,
    ) -> Result<BernoulliReport, FormulaError> {
        domain::finite(pressure, "pressure")?;
        domain::positive(density, "density")?;
        domain::nonnegative(velocity, "velocity")?;
        domain::finite(gravity, "gravity")?;
        domain::finite(elevation, "elevation")?;
        let dynamic_pressure =
            domain::finite_result(0.5 * density * velocity * velocity, "dynamic pressure")?;
        let weight = domain::finite_result(density * gravity, "specific weight")?;
        let total_pressure = domain::finite_result(
            pressure + dynamic_pressure + weight * elevation,
            "total pressure",
        )?;
        let total_head = domain::divide(total_pressure, weight)?;
        Ok(BernoulliReport {
            pressure,
            velocity,
            elevation,
            total_head,
            dynamic_pressure,
        })
    }

    /// Reynolds number: Re = rho * v * L / mu
    pub fn re_n(density: f64, velocity: f64, char_length: f64, viscosity: f64) -> Option<f64> {
        if !density.is_finite()
            || density < 0.0
            || !velocity.is_finite()
            || velocity < 0.0
            || !char_length.is_finite()
            || char_length <= 0.0
            || !viscosity.is_finite()
            || viscosity <= 0.0
        {
            return None;
        }
        Some(density * velocity * char_length / viscosity)
    }

    /// Flow regime based on Reynolds number: 0=laminar, 1=transition, 2=turbulent
    pub fn flow_regime(reynolds: f64) -> u8 {
        if reynolds < 2000.0 {
            0
        } else if reynolds < 4000.0 {
            1
        } else {
            2
        }
    }

    /// Friction factor for pipe flow (Darcy-Weisbach).
    /// Laminar: f = 64/Re. Turbulent: Colebrook equation (iterative).
    pub fn darcy_friction_factor(reynolds: f64, relative_roughness: f64) -> Option<f64> {
        if !reynolds.is_finite()
            || reynolds <= 0.0
            || !relative_roughness.is_finite()
            || relative_roughness < 0.0
        {
            return None;
        }
        if reynolds < 2000.0 {
            return Some(64.0 / reynolds);
        }
        // Colebrook-White: 1/sqrt(f) = -2*log10(eps/(3.7*D) + 2.51/(Re*sqrt(f)))
        let eps = relative_roughness;
        let mut f: f64 = 0.02; // initial guess
        for _ in 0..30 {
            let f_sqrt = f.sqrt();
            let rhs = -2.0 * (eps / 3.7 + 2.51 / (reynolds * f_sqrt)).log10();
            let f_new = 1.0 / (rhs * rhs);
            if (f_new - f).abs() < 1.0e-10 {
                f = f_new;
                break;
            }
            f = f_new;
        }
        Some(f)
    }

    /// Compute the force and torque produced by a single aerodynamic surface.
    pub fn compute_surface_force(
        surface: AeroSurface,
        body_linvel: Vec3,
        body_angvel: Vec3,
        body_center: Vec3,
        wind_velocity: Vec3,
        air_density: f64,
    ) -> Option<(Vec3, Vec3)> {
        if !aero_surface_valid(&surface) || !air_density.is_finite() || air_density < 0.0 {
            return None;
        }

        let point = vec3_to_rapier(surface.point);
        let normal = vec3_to_rapier(surface.normal);
        let unit_normal = normal.try_normalize()?;

        let body_center = vec3_to_rapier(body_center);
        let body_linvel = vec3_to_rapier(body_linvel);
        let body_angvel = vec3_to_rapier(body_angvel);

        let arm = point - body_center;
        let point_velocity = body_linvel + body_angvel.cross(arm);
        let relative_air = vec3_to_rapier(wind_velocity) - point_velocity;
        let speed_squared = relative_air.length_squared();
        if speed_squared <= 1.0e-18 {
            return None;
        }

        let speed = speed_squared.sqrt();
        let flow_dir = relative_air / speed;
        let exposure = flow_dir.dot(unit_normal).max(0.0);
        if exposure <= 0.0 {
            return None;
        }

        let dynamic_pressure = 0.5 * air_density * speed_squared;
        let effective_area = surface.area * exposure;
        let drag = flow_dir * (dynamic_pressure * effective_area * surface.drag_coefficient);
        let lift_axis = flow_dir.cross(unit_normal);
        let lift = lift_axis
            .try_normalize()
            .map(|axis| {
                let lift_dir = axis.cross(flow_dir);
                lift_dir * (dynamic_pressure * effective_area * surface.lift_coefficient)
            })
            .unwrap_or(crate::math::Vector3f64::zeros());
        let force = drag + lift;

        Some((vec3_from_rapier(force), vec3_from_rapier(arm.cross(force))))
    }

    /// Estimate total surface force without modifying a body.
    pub fn estimate_surface_force(
        body_linvel: Vec3,
        body_angvel: Vec3,
        body_center: Vec3,
        wind_velocity: Vec3,
        air_density: f64,
        surface: AeroSurface,
    ) -> Option<AeroForceReport> {
        let (force, torque) = compute_surface_force(
            surface,
            body_linvel,
            body_angvel,
            body_center,
            wind_velocity,
            air_density,
        )?;
        Some(AeroForceReport {
            total_force: force,
            total_torque: torque,
            surface_count: 1,
            active_surface_count: 1,
        })
    }
}
