use std::slice;

use rapier3d::prelude::Vector;

use crate::rapier::error::{
    ERR_CAPACITY, ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, set_error,
};
use crate::rapier::ffi::{
    Bool, ElectromagneticField, FaradayInductionReport, FdtdYeeReport, LorentzForceReport,
    MagneticFluxReport, MaxwellPointReport, Vec3, vec3_finite, vec3_from_rapier, vec3_to_rapier,
};

use crate::rapier::math::{KahanSum, finite_non_negative, finite_positive};

const EPSILON: f64 = 1.0e-12;
const VACUUM_PERMITTIVITY: f64 = 8.854_187_812_8e-12;
const VACUUM_PERMEABILITY: f64 = 1.256_637_062_12e-6;
const MAX_FIELD_CELLS: u32 = 2_000_000;

fn field_valid(field: ElectromagneticField) -> bool {
    vec3_finite(field.electric) && vec3_finite(field.magnetic)
}

#[unsafe(no_mangle)]
pub extern "C" fn em_lorentz_force(
    charge: f64,
    velocity: Vec3,
    field: ElectromagneticField,
    mass: f64,
    out_report: *mut LorentzForceReport,
) -> Bool {
    if !charge.is_finite()
        || !vec3_finite(velocity)
        || !field_valid(field)
        || !finite_non_negative(mass)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid Lorentz force parameters");
        return Bool::FALSE;
    }
    let v = vec3_to_rapier(velocity);
    let electric = vec3_to_rapier(field.electric);
    let magnetic = vec3_to_rapier(field.magnetic);
    let force = (electric + v.cross(magnetic)) * charge;
    let acceleration = if mass > EPSILON {
        force / mass
    } else {
        Vector::ZERO
    };
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "Lorentz force output is null");
        return Bool::FALSE;
    };
    *out_report = LorentzForceReport {
        electric_force: vec3_from_rapier(electric * charge),
        magnetic_force: vec3_from_rapier(v.cross(magnetic) * charge),
        total_force: vec3_from_rapier(force),
        acceleration: vec3_from_rapier(acceleration),
    };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn em_magnetic_flux(
    magnetic_field: Vec3,
    area_normal: Vec3,
    area: f64,
    out_report: *mut MagneticFluxReport,
) -> Bool {
    if !vec3_finite(magnetic_field) || !vec3_finite(area_normal) || !finite_non_negative(area) {
        set_error(ERR_INVALID_ARGUMENT, "invalid magnetic flux parameters");
        return Bool::FALSE;
    }
    let b = vec3_to_rapier(magnetic_field);
    let n = vec3_to_rapier(area_normal);
    let normal_len = n.length();
    if normal_len <= EPSILON {
        set_error(ERR_INVALID_ARGUMENT, "magnetic flux normal is zero");
        return Bool::FALSE;
    }
    let unit_normal = n / normal_len;
    let normal_component = b.dot(unit_normal);
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "magnetic flux output is null");
        return Bool::FALSE;
    };
    *out_report = MagneticFluxReport {
        flux: normal_component * area,
        normal_component,
        area,
    };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn em_faraday_induction(
    previous_flux: f64,
    current_flux: f64,
    dt: f64,
    turns: f64,
    resistance: f64,
    out_report: *mut FaradayInductionReport,
) -> Bool {
    if !previous_flux.is_finite()
        || !current_flux.is_finite()
        || !finite_positive(dt)
        || !finite_non_negative(turns)
        || !finite_non_negative(resistance)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid Faraday induction parameters");
        return Bool::FALSE;
    }
    let flux_rate = (current_flux - previous_flux) / dt;
    let induced_emf = -turns * flux_rate;
    let induced_current = if resistance > EPSILON {
        induced_emf / resistance
    } else {
        0.0
    };
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "Faraday induction output is null");
        return Bool::FALSE;
    };
    *out_report = FaradayInductionReport {
        flux_rate,
        induced_emf,
        induced_current,
    };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn em_maxwell_point_update(
    field: ElectromagneticField,
    curl_electric: Vec3,
    curl_magnetic: Vec3,
    current_density: Vec3,
    charge_density: f64,
    divergence_electric: f64,
    divergence_magnetic: f64,
    permittivity: f64,
    permeability: f64,
    dt: f64,
    out_report: *mut MaxwellPointReport,
) -> Bool {
    if !field_valid(field)
        || !vec3_finite(curl_electric)
        || !vec3_finite(curl_magnetic)
        || !vec3_finite(current_density)
        || !charge_density.is_finite()
        || !divergence_electric.is_finite()
        || !divergence_magnetic.is_finite()
        || !finite_positive(permittivity)
        || !finite_positive(permeability)
        || !finite_non_negative(dt)
    {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid Maxwell point update parameters",
        );
        return Bool::FALSE;
    }

    let e = vec3_to_rapier(field.electric);
    let b = vec3_to_rapier(field.magnetic);
    let curl_e = vec3_to_rapier(curl_electric);
    let curl_b = vec3_to_rapier(curl_magnetic);
    let j = vec3_to_rapier(current_density);
    let electric_derivative = curl_b / (permittivity * permeability) - j / permittivity;
    let magnetic_derivative = -curl_e;
    let next_electric = e + electric_derivative * dt;
    let next_magnetic = b + magnetic_derivative * dt;

    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "Maxwell point output is null");
        return Bool::FALSE;
    };
    *out_report = MaxwellPointReport {
        next_field: ElectromagneticField {
            electric: vec3_from_rapier(next_electric),
            magnetic: vec3_from_rapier(next_magnetic),
        },
        electric_derivative: vec3_from_rapier(electric_derivative),
        magnetic_derivative: vec3_from_rapier(magnetic_derivative),
        gauss_electric_residual: divergence_electric - charge_density / permittivity,
        gauss_magnetic_residual: divergence_magnetic,
    };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn em_fdtd_yee_update(
    electric_fields: *const Vec3,
    magnetic_fields: *const Vec3,
    curl_electric: *const Vec3,
    curl_magnetic: *const Vec3,
    cell_count: u32,
    permittivity: f64,
    permeability: f64,
    conductivity: f64,
    dt: f64,
    out_electric_fields: *mut Vec3,
    out_magnetic_fields: *mut Vec3,
    capacity: u32,
    out_report: *mut FdtdYeeReport,
) -> Bool {
    if cell_count == 0 || cell_count > MAX_FIELD_CELLS || capacity < cell_count {
        set_error(ERR_CAPACITY, "invalid FDTD Yee grid capacity");
        return Bool::FALSE;
    }
    if electric_fields.is_null()
        || magnetic_fields.is_null()
        || curl_electric.is_null()
        || curl_magnetic.is_null()
        || out_electric_fields.is_null()
        || out_magnetic_fields.is_null()
    {
        set_error(ERR_NULL_POINTER, "FDTD Yee grid pointers are null");
        return Bool::FALSE;
    }
    if !finite_positive(permittivity)
        || !finite_positive(permeability)
        || !finite_non_negative(conductivity)
        || !finite_non_negative(dt)
    {
        set_error(ERR_INVALID_ARGUMENT, "invalid FDTD Yee grid parameters");
        return Bool::FALSE;
    }

    let electric_fields = unsafe { slice::from_raw_parts(electric_fields, cell_count as usize) };
    let magnetic_fields = unsafe { slice::from_raw_parts(magnetic_fields, cell_count as usize) };
    let curl_electric = unsafe { slice::from_raw_parts(curl_electric, cell_count as usize) };
    let curl_magnetic = unsafe { slice::from_raw_parts(curl_magnetic, cell_count as usize) };
    let out_electric = unsafe { slice::from_raw_parts_mut(out_electric_fields, capacity as usize) };
    let out_magnetic = unsafe { slice::from_raw_parts_mut(out_magnetic_fields, capacity as usize) };

    let mut max_electric_delta = 0.0;
    let mut max_magnetic_delta = 0.0;
    let mut total_energy_acc = KahanSum::default();
    for index in 0..cell_count as usize {
        if !vec3_finite(electric_fields[index])
            || !vec3_finite(magnetic_fields[index])
            || !vec3_finite(curl_electric[index])
            || !vec3_finite(curl_magnetic[index])
        {
            set_error(ERR_INVALID_ARGUMENT, "invalid FDTD Yee grid cell");
            return Bool::FALSE;
        }
        let e = vec3_to_rapier(electric_fields[index]);
        let b = vec3_to_rapier(magnetic_fields[index]);
        let e_delta = (vec3_to_rapier(curl_magnetic[index]) / (permittivity * permeability)
            - e * (conductivity / permittivity))
            * dt;
        let b_delta = -vec3_to_rapier(curl_electric[index]) * dt;
        let next_e = e + e_delta;
        let next_b = b + b_delta;
        out_electric[index] = vec3_from_rapier(next_e);
        out_magnetic[index] = vec3_from_rapier(next_b);
        max_electric_delta = f64::max(max_electric_delta, e_delta.length());
        max_magnetic_delta = f64::max(max_magnetic_delta, b_delta.length());
        total_energy_acc.add(
            0.5 * permittivity * next_e.length_squared()
                + 0.5 * next_b.length_squared() / permeability,
        );
    }

    if let Some(out_report) = unsafe { out_report.as_mut() } {
        *out_report = FdtdYeeReport {
            cell_count,
            max_electric_delta,
            max_magnetic_delta,
            total_energy_density: total_energy_acc.value(),
            courant_number: dt / (permittivity * permeability).sqrt(),
        };
    }
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn em_vacuum_permittivity() -> f64 {
    VACUUM_PERMITTIVITY
}

#[unsafe(no_mangle)]
pub extern "C" fn em_vacuum_permeability() -> f64 {
    VACUUM_PERMEABILITY
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v3(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3 { x, y, z }
    }

    #[test]
    fn electromagnetic_formulas_work() {
        let field = ElectromagneticField {
            electric: v3(1.0, 0.0, 0.0),
            magnetic: v3(0.0, 0.0, 2.0),
        };
        let mut lorentz = LorentzForceReport::default();
        assert_eq!(
            em_lorentz_force(3.0, v3(0.0, 4.0, 0.0), field, 2.0, &mut lorentz),
            Bool::TRUE
        );
        assert_eq!(lorentz.total_force.x, 27.0);

        let mut flux = MagneticFluxReport::default();
        assert_eq!(
            em_magnetic_flux(v3(0.0, 0.0, 2.0), v3(0.0, 0.0, 1.0), 3.0, &mut flux),
            Bool::TRUE
        );
        assert_eq!(flux.flux, 6.0);

        let mut induction = FaradayInductionReport::default();
        assert_eq!(
            em_faraday_induction(2.0, 5.0, 0.5, 10.0, 5.0, &mut induction),
            Bool::TRUE
        );
        assert_eq!(induction.induced_emf, -60.0);
        assert_eq!(induction.induced_current, -12.0);
    }

    #[test]
    fn maxwell_and_yee_updates_work() {
        let field = ElectromagneticField {
            electric: v3(1.0, 0.0, 0.0),
            magnetic: v3(0.0, 1.0, 0.0),
        };
        let mut maxwell = MaxwellPointReport::default();
        assert_eq!(
            em_maxwell_point_update(
                field,
                v3(0.0, 0.0, 1.0),
                v3(0.0, 0.0, 2.0),
                v3(0.0, 0.0, 0.0),
                0.0,
                0.0,
                0.0,
                2.0,
                4.0,
                0.5,
                &mut maxwell,
            ),
            Bool::TRUE
        );
        assert_eq!(maxwell.next_field.magnetic.z, -0.5);

        let electric = [v3(1.0, 0.0, 0.0)];
        let magnetic = [v3(0.0, 1.0, 0.0)];
        let curl_e = [v3(0.0, 0.0, 1.0)];
        let curl_b = [v3(0.0, 0.0, 2.0)];
        let mut out_e = [Vec3::default()];
        let mut out_b = [Vec3::default()];
        let mut report = FdtdYeeReport::default();
        assert_eq!(
            em_fdtd_yee_update(
                electric.as_ptr(),
                magnetic.as_ptr(),
                curl_e.as_ptr(),
                curl_b.as_ptr(),
                1,
                2.0,
                4.0,
                0.0,
                0.5,
                out_e.as_mut_ptr(),
                out_b.as_mut_ptr(),
                1,
                &mut report,
            ),
            Bool::TRUE
        );
        assert_eq!(out_b[0].z, -0.5);
        assert_eq!(report.cell_count, 1);
    }
}
