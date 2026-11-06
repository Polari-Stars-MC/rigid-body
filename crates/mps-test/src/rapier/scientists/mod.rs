//! 科学家目录模块镜像测试（每科学家一个子文件 + 聚合 API）。
#[cfg(test)]
mod tests {
    use mps_formula::disciplines::discipline_by_id;
    use mps_formula::scientists::*;

    #[test]
    fn each_scientist_file_registered_and_populated() {
        assert!(scientist_count() > 0);
        assert_eq!(SCIENTIST_ISAAC_NEWTON.id, "isaac_newton");
        assert_eq!(SCIENTIST_GALILEO_GALILEI.id, "galileo_galilei");
        assert_eq!(
            SCIENTIST_WILLIAM_ROWAN_HAMILTON.id,
            "william_rowan_hamilton"
        );
        assert_eq!(SCIENTIST_JOSEPH_LOUIS_LAGRANGE.id, "joseph_louis_lagrange");
        assert_eq!(SCIENTIST_LEONHARD_EULER.id, "leonhard_euler");
        assert_eq!(SCIENTIST_CLAUDE_LOUIS_NAVIER.id, "claude_louis_navier");
        assert_eq!(SCIENTIST_GEORGE_STOKES.id, "george_stokes");
        assert_eq!(SCIENTIST_DANIEL_BERNOULLI.id, "daniel_bernoulli");
        assert_eq!(SCIENTIST_OSBORNE_REYNOLDS.id, "osborne_reynolds");
        assert_eq!(SCIENTIST_ERNST_MACH.id, "ernst_mach");
        assert_eq!(SCIENTIST_ARCHIMEDES.id, "archimedes");
        assert_eq!(SCIENTIST_JAMES_CLERK_MAXWELL.id, "james_clerk_maxwell");
        assert_eq!(SCIENTIST_MICHAEL_FARADAY.id, "michael_faraday");
        assert_eq!(SCIENTIST_ANDRE_MARIE_AMPERE.id, "andre_marie_ampere");
        assert_eq!(
            SCIENTIST_CHARLES_AUGUSTIN_DE_COULOMB.id,
            "charles_augustin_de_coulomb"
        );
        assert_eq!(SCIENTIST_GEORG_OHM.id, "georg_ohm");
        assert_eq!(SCIENTIST_HEINRICH_HERTZ.id, "heinrich_hertz");
        assert_eq!(SCIENTIST_HANS_CHRISTIAN_ORSTED.id, "hans_christian_orsted");
        assert_eq!(SCIENTIST_CARL_FRIEDRICH_GAUSS.id, "carl_friedrich_gauss");
        assert_eq!(SCIENTIST_BERNHARD_RIEMANN.id, "bernhard_riemann");
        assert_eq!(SCIENTIST_JOSEPH_FOURIER.id, "joseph_fourier");
        assert_eq!(SCIENTIST_HENRI_POINCARE.id, "henri_poincare");
        assert_eq!(SCIENTIST_PIERRE_SIMON_LAPLACE.id, "pierre_simon_laplace");
        assert_eq!(SCIENTIST_JOHANNES_KEPLER.id, "johannes_kepler");
        assert_eq!(SCIENTIST_ALBERT_EINSTEIN.id, "albert_einstein");
        assert_eq!(SCIENTIST_MAX_PLANCK.id, "max_planck");
        assert_eq!(SCIENTIST_NIELS_BOHR.id, "niels_bohr");
        assert_eq!(SCIENTIST_WERNER_HEISENBERG.id, "werner_heisenberg");
        assert_eq!(SCIENTIST_ERWIN_SCHRODINGER.id, "erwin_schrodinger");
        assert_eq!(SCIENTIST_WOLFGANG_PAULI.id, "wolfgang_pauli");
        assert_eq!(SCIENTIST_SATYENDRA_NATH_BOSE.id, "satyendra_nath_bose");
        assert_eq!(SCIENTIST_ENRICO_FERMI.id, "enrico_fermi");
        assert_eq!(SCIENTIST_PAUL_DIRAC.id, "paul_dirac");
        assert_eq!(SCIENTIST_RICHARD_FEYNMAN.id, "richard_feynman");
        assert_eq!(SCIENTIST_LUDWIG_BOLTZMANN.id, "ludwig_boltzmann");
        assert_eq!(SCIENTIST_RUDOLF_CLAUSIUS.id, "rudolf_clausius");
        assert_eq!(SCIENTIST_SADI_CARNOT.id, "sadi_carnot");
        assert_eq!(SCIENTIST_LORD_KELVIN.id, "lord_kelvin");
        assert_eq!(SCIENTIST_JAMES_WATT.id, "james_watt");
        assert_eq!(SCIENTIST_LEV_LANDAU.id, "lev_landau");
        assert_eq!(SCIENTIST_ERNEST_RUTHERFORD.id, "ernest_rutherford");
        assert_eq!(SCIENTIST_MARIE_CURIE.id, "marie_curie");
        assert_eq!(SCIENTIST_AUGUSTIN_FRESNEL.id, "augustin_fresnel");
        assert_eq!(SCIENTIST_CHRISTIAAN_HUYGENS.id, "christiaan_huygens");
        assert_eq!(SCIENTIST_THOMAS_YOUNG.id, "thomas_young");
        assert_eq!(SCIENTIST_ALEKSANDR_LYAPUNOV.id, "aleksandr_lyapunov");
        assert_eq!(SCIENTIST_EDWARD_LORENZ.id, "edward_lorenz");
        assert_eq!(SCIENTIST_MITCHELL_FEIGENBAUM.id, "mitchell_feigenbaum");
        assert_eq!(SCIENTIST_ERNST_CHLADNI.id, "ernst_chladni");
        assert_eq!(SCIENTIST_LORD_RAYLEIGH.id, "lord_rayleigh");
        assert_eq!(SCIENTIST_LUDWIG_PRANDTL.id, "ludwig_prandtl");
        assert_eq!(SCIENTIST_THEODORE_VON_KARMAN.id, "theodore_von_karman");
        assert_eq!(
            SCIENTIST_JEAN_LE_ROND_DALEMBERT.id,
            "jean_le_rond_dalembert"
        );
        assert_eq!(SCIENTIST_HENDRIK_LORENTZ.id, "hendrik_lorentz");
        assert_eq!(SCIENTIST_GOTTFRIED_LEIBNIZ.id, "gottfried_leibniz");
        assert_eq!(SCIENTIST_LOUIS_DE_BROGLIE.id, "louis_de_broglie");
        assert_eq!(SCIENTIST_MAX_BORN.id, "max_born");
        assert_eq!(SCIENTIST_JAMES_CHADWICK.id, "james_chadwick");
        assert_eq!(SCIENTIST_HIDEKI_YUKAWA.id, "hideki_yukawa");
        assert_eq!(SCIENTIST_KARL_SCHWARZSCHILD.id, "karl_schwarzschild");
        assert_eq!(SCIENTIST_EDWIN_HUBBLE.id, "edwin_hubble");
        assert_eq!(SCIENTIST_ARTHUR_EDDINGTON.id, "arthur_eddington");
    }

    #[test]
    fn newton_has_gravitation_and_laws() {
        let n = scientist_by_id("isaac_newton").expect("Newton cataloged");
        assert_eq!(n.field_id, "mechanics");
        assert!(n.contribution.contains("universal gravitation"));
        use mps_formula::scientists::isaac_newton::formulas::*;
        // `G` is a `const`, so this is a compile-time property of the catalog
        // rather than something to check per test run (clippy:
        // `assertions_on_constants`). Value pinned to the CODATA-2018 figure.
        const {
            assert!(G > 0.0);
            assert!((G - 6.67430e-11).abs() < 1.0e-16);
        }
    }

    #[test]
    fn maxwell_lists_em_formulas() {
        let m = scientist_by_id("james_clerk_maxwell").expect("Maxwell cataloged");
        assert_eq!(m.field_id, "electromagnetism");
        use mps_formula::scientists::james_clerk_maxwell::formulas::*;
        let _ = (poynting_vector, biot_savart_element);
    }

    #[test]
    fn every_scientist_field_id_maps_to_a_discipline() {
        let orphan: Vec<&str> = SCIENTISTS
            .iter()
            .filter(|s| discipline_by_id(s.field_id).is_none())
            .map(|s| s.field_id)
            .collect();
        assert!(orphan.is_empty(), "orphan field_ids: {orphan:?}");
    }

    #[test]
    fn rejects_empty_and_unknown_id() {
        assert!(scientist_by_id("").is_none());
        assert!(scientist_by_id("nobody_xyz").is_none());
    }

    #[test]
    fn electromagnetism_field_has_faraday_and_maxwell() {
        let em = scientists_by_field("electromagnetism");
        let ids: Vec<&str> = em.iter().map(|s| s.id).collect();
        assert!(ids.contains(&"michael_faraday"));
        assert!(ids.contains(&"james_clerk_maxwell"));
    }
}
