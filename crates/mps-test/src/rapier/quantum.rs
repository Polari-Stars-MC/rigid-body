#[cfg(test)]
mod tests {
    use mps_core::rapier::quantum::*;
    use mps_core::rapier::ffi::*;

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



