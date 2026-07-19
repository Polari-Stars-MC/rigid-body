#[cfg(test)]
mod tests {
    use mps_core::rapier::acoustics::*;
    use mps_core::rapier::ffi::*;

    #[test]
    fn modal_analysis_solves_generalized_eigenproblem() {
        let stiffness = [4.0, 0.0, 0.0, 9.0];
        let mass = [1.0, 0.0, 0.0, 1.0];
        let mut eigenvalues = [0.0; 2];
        let mut frequencies = [0.0; 2];
        let mut modes = [0.0; 4];
        let mut report = ModalAnalysisReport::default();
        assert_eq!(
            acoustic_generalized_modal_analysis(
                stiffness.as_ptr(),
                mass.as_ptr(),
                2,
                2,
                eigenvalues.as_mut_ptr(),
                frequencies.as_mut_ptr(),
                modes.as_mut_ptr(),
                2,
                4,
                &mut report,
            ),
            Bool::TRUE
        );
        assert!((eigenvalues[0] - 4.0).abs() < 1.0e-8);
        assert!((eigenvalues[1] - 9.0).abs() < 1.0e-8);
        assert_eq!(report.mode_count, 2);
    }

    #[test]
    fn wave_step_and_resonance_detection_work() {
        let previous = [0.0, 0.0, 0.0];
        let current = [0.0, 1.0, 0.0];
        let laplacian = [1.0, -2.0, 1.0];
        let mut next = [0.0; 3];
        let mut wave = AcousticWaveReport::default();
        assert_eq!(
            acoustic_wave_equation_step(
                previous.as_ptr(),
                current.as_ptr(),
                laplacian.as_ptr(),
                3,
                1.0,
                0.0,
                0.1,
                next.as_mut_ptr(),
                3,
                &mut wave,
            ),
            Bool::TRUE
        );
        assert!(next[1] < 2.0);
        assert!(wave.acoustic_energy > 0.0);

        let modal = [100.0, 250.0, 500.0];
        let damping = [0.02, 0.05, 0.1];
        let mut resonance = AcousticResonanceReport::default();
        assert_eq!(
            acoustic_detect_resonance(
                248.0,
                modal.as_ptr(),
                damping.as_ptr(),
                3,
                5.0,
                &mut resonance
            ),
            Bool::TRUE
        );
        assert_eq!(resonance.resonant, Bool::TRUE);
        assert_eq!(resonance.nearest_mode_index, 1);
    }

    #[test]
    fn contact_excitation_feeds_modal_synthesis_and_spatialization() {
        let metal = AcousticMaterial {
            density: 7_800.0,
            hardness: 2.0e11,
            damping: 0.03,
            roughness: 0.25,
            restitution: 0.55,
            sound_speed: 5_000.0,
        };
        let wood = AcousticMaterial {
            density: 700.0,
            hardness: 1.0e10,
            damping: 0.12,
            roughness: 0.55,
            restitution: 0.35,
            sound_speed: 3_300.0,
        };
        let contact = AcousticContactDesc {
            normal_force: 120.0,
            normal_velocity: -2.0,
            tangential_velocity: 0.4,
            contact_area: 0.002,
            dt: 1.0 / 60.0,
        };
        let mut excitation = AcousticExcitationReport::default();
        assert_eq!(
            acoustic_contact_material_excitation(metal, wood, contact, &mut excitation),
            Bool::TRUE
        );
        assert!(excitation.amplitude > 0.0);
        assert!(excitation.brightness >= 0.0 && excitation.brightness <= 1.0);

        let frequencies = [180.0, 520.0, 1_200.0];
        let damping = [0.02, 0.04, 0.08];
        let gains = [1.0, 0.45, 0.18];
        let mut displacements = [0.0; 3];
        let mut velocities = [0.0; 3];
        let mut modal = ModalSynthesisReport::default();
        assert_eq!(
            acoustic_modal_synthesis_step(
                frequencies.as_ptr(),
                damping.as_ptr(),
                gains.as_ptr(),
                displacements.as_mut_ptr(),
                velocities.as_mut_ptr(),
                3,
                excitation.amplitude,
                1.0 / 48_000.0,
                0.25,
                &mut modal,
            ),
            Bool::TRUE
        );
        assert_eq!(modal.mode_count, 3);
        assert!(modal.modal_energy > 0.0);

        let mut stereo = SpatializedSample::default();
        assert_eq!(
            acoustic_spatialize_mono_sample(
                modal.sample,
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                Vec3::default(),
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                1.0,
                1.0,
                &mut stereo,
            ),
            Bool::TRUE
        );
        assert!(stereo.right.abs() >= stereo.left.abs());
        assert!(stereo.attenuation > 0.0 && stereo.attenuation <= 1.0);
    }
}



