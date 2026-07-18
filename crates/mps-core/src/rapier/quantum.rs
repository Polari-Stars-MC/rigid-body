use crate::rapier::error::{ERR_INVALID_ARGUMENT, ERR_NULL_POINTER, clear_error, set_error};
use crate::rapier::ffi::{
    Bool, QuantumBarrier, QuantumOscillatorReport, QuantumTunnelingReport, QuantumWaveFunction,
};

use crate::rapier::math::{finite_non_negative, finite_positive};

const EPSILON: f64 = 1.0e-12;
const REDUCED_PLANCK: f64 = 1.054_571_817e-34;

fn effective_hbar(reduced_planck: f64) -> f64 {
    if reduced_planck == 0.0 {
        REDUCED_PLANCK
    } else {
        reduced_planck
    }
}

fn wave_function_valid(wave: QuantumWaveFunction) -> bool {
    wave.amplitude_real.is_finite() && wave.amplitude_imag.is_finite()
}

fn barrier_valid(barrier: QuantumBarrier) -> bool {
    let hbar = effective_hbar(barrier.reduced_planck);
    finite_non_negative(barrier.particle_energy)
        && finite_non_negative(barrier.barrier_potential)
        && finite_non_negative(barrier.barrier_width)
        && finite_positive(barrier.particle_mass)
        && finite_positive(hbar)
}

fn compute_tunneling(barrier: QuantumBarrier) -> Option<QuantumTunnelingReport> {
    if !barrier_valid(barrier) {
        return None;
    }
    let hbar = effective_hbar(barrier.reduced_planck);
    let mass = barrier.particle_mass;
    let energy = barrier.particle_energy;
    let potential = barrier.barrier_potential;

    if barrier.barrier_width <= EPSILON || energy >= potential {
        let wave_number = (2.0 * mass * energy.max(0.0)).sqrt() / hbar;
        return Some(QuantumTunnelingReport {
            wave_number,
            decay_constant: 0.0,
            exponent: 0.0,
            transmission_coefficient: 1.0,
            reflection_coefficient: 0.0,
        });
    }

    let delta = potential - energy;
    let decay_constant = (2.0 * mass * delta).sqrt() / hbar;
    let exponent = 2.0 * decay_constant * barrier.barrier_width;
    let transmission = (-exponent).exp().clamp(0.0, 1.0);
    Some(QuantumTunnelingReport {
        wave_number: (2.0 * mass * energy.max(0.0)).sqrt() / hbar,
        decay_constant,
        exponent,
        transmission_coefficient: transmission,
        reflection_coefficient: 1.0 - transmission,
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_reduced_planck_constant() -> f64 {
    REDUCED_PLANCK
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_wave_probability_density(wave: QuantumWaveFunction) -> f64 {
    if !wave_function_valid(wave) {
        return f64::NAN;
    }
    wave.amplitude_real * wave.amplitude_real + wave.amplitude_imag * wave.amplitude_imag
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_wave_normalize(
    wave: QuantumWaveFunction,
    out_wave: *mut QuantumWaveFunction,
) -> Bool {
    if !wave_function_valid(wave) {
        set_error(ERR_INVALID_ARGUMENT, "invalid quantum wave function");
        return Bool::FALSE;
    }
    let density = quantum_wave_probability_density(wave);
    if density <= EPSILON {
        set_error(ERR_INVALID_ARGUMENT, "quantum wave function has zero norm");
        return Bool::FALSE;
    }
    let Some(out_wave) = (unsafe { out_wave.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "quantum wave output is null");
        return Bool::FALSE;
    };
    let norm = density.sqrt();
    *out_wave = QuantumWaveFunction {
        amplitude_real: wave.amplitude_real / norm,
        amplitude_imag: wave.amplitude_imag / norm,
    };
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_wkb_transmission(action_integral: f64, reduced_planck: f64) -> f64 {
    let hbar = effective_hbar(reduced_planck);
    if !finite_non_negative(action_integral) || !finite_positive(hbar) {
        return f64::NAN;
    }
    (-2.0 * action_integral / hbar).exp().clamp(0.0, 1.0)
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_rectangular_barrier_tunneling(
    barrier: QuantumBarrier,
    out_report: *mut QuantumTunnelingReport,
) -> Bool {
    let Some(report) = compute_tunneling(barrier) else {
        set_error(ERR_INVALID_ARGUMENT, "invalid quantum tunneling barrier");
        return Bool::FALSE;
    };
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "quantum tunneling output is null");
        return Bool::FALSE;
    };
    *out_report = report;
    clear_error();
    Bool::TRUE
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_rectangular_barrier_probability(barrier: QuantumBarrier) -> f64 {
    compute_tunneling(barrier)
        .map(|report| report.transmission_coefficient)
        .unwrap_or(f64::NAN)
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_zero_point_energy(angular_frequency: f64, reduced_planck: f64) -> f64 {
    let hbar = effective_hbar(reduced_planck);
    if !finite_non_negative(angular_frequency) || !finite_positive(hbar) {
        return f64::NAN;
    }
    0.5 * hbar * angular_frequency
}

#[unsafe(no_mangle)]
pub extern "C" fn quantum_harmonic_oscillator_report(
    angular_frequency: f64,
    reduced_planck: f64,
    out_report: *mut QuantumOscillatorReport,
) -> Bool {
    let hbar = effective_hbar(reduced_planck);
    if !finite_non_negative(angular_frequency) || !finite_positive(hbar) {
        set_error(
            ERR_INVALID_ARGUMENT,
            "invalid quantum oscillator parameters",
        );
        return Bool::FALSE;
    }
    let Some(out_report) = (unsafe { out_report.as_mut() }) else {
        set_error(ERR_NULL_POINTER, "quantum oscillator output is null");
        return Bool::FALSE;
    };
    let level_spacing = hbar * angular_frequency;
    *out_report = QuantumOscillatorReport {
        angular_frequency,
        zero_point_energy: 0.5 * level_spacing,
        first_excited_energy: 1.5 * level_spacing,
        level_spacing,
    };
    clear_error();
    Bool::TRUE
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wave_probability_and_normalization_work() {
        let wave = QuantumWaveFunction {
            amplitude_real: 3.0,
            amplitude_imag: 4.0,
        };
        assert_eq!(quantum_wave_probability_density(wave), 25.0);

        let mut normalized = QuantumWaveFunction::default();
        assert_eq!(quantum_wave_normalize(wave, &mut normalized), Bool::TRUE);
        assert!((quantum_wave_probability_density(normalized) - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn tunneling_probability_uses_wkb_decay() {
        let barrier = QuantumBarrier {
            particle_energy: 1.0,
            barrier_potential: 5.0,
            barrier_width: 0.5,
            particle_mass: 1.0,
            reduced_planck: 1.0,
        };
        let mut report = QuantumTunnelingReport::default();
        assert_eq!(
            quantum_rectangular_barrier_tunneling(barrier, &mut report),
            Bool::TRUE
        );
        assert!(report.decay_constant > 0.0);
        assert!(report.transmission_coefficient > 0.0);
        assert!(report.transmission_coefficient < 1.0);
        assert!(
            (report.transmission_coefficient - quantum_wkb_transmission(2.0_f64.sqrt(), 1.0)).abs()
                < 1.0e-12
        );
    }

    #[test]
    fn zero_point_energy_is_half_hbar_omega() {
        assert_eq!(quantum_zero_point_energy(4.0, 2.0), 4.0);

        let mut report = QuantumOscillatorReport::default();
        assert_eq!(
            quantum_harmonic_oscillator_report(4.0, 2.0, &mut report),
            Bool::TRUE
        );
        assert_eq!(report.zero_point_energy, 4.0);
        assert_eq!(report.first_excited_energy, 12.0);
        assert_eq!(report.level_spacing, 8.0);
    }
}
