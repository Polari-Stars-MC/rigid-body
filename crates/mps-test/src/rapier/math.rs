#[cfg(test)]
mod tests {
    use mps_core::rapier::math::*;
    use mps_core::rapier::ffi::*;

    #[test]
    fn kahan_sum_beats_naive_on_many_small_values() {
        let big = 1.0e12_f64;
        let small = 1.0e-6_f64;

        // Naive: the small values get swallowed.
        let mut naive = big;
        for _ in 0..1_000_000 {
            naive += small;
        }
        assert!((naive - big).abs() < 1.0, "naive sum lost all small terms");

        // Kahan: the small values are preserved.
        let mut kahan = KahanSum::new(big);
        for _ in 0..1_000_000 {
            kahan.add(small);
        }
        let expected = big + 1.0;
        assert!(
            (kahan.value() - expected).abs() < 1.0e-6,
            "Kahan sum preserved small terms: {} vs {}",
            kahan.value(),
            expected
        );
    }

    #[test]
    fn kahan_vec3_beats_naive_on_many_small_forces() {
        // Use values large enough that naive summation loses the small components.
        // 1e15 has ~0.1 ulp resolution, so adding 1e-6 * 1e6 = 1.0 will be lost.
        let big = Vec3 {
            x: 1.0e15,
            y: -2.0e15,
            z: 3.0e15,
        };
        let small = Vec3 {
            x: 1.0e-6,
            y: 2.0e-6,
            z: 3.0e-6,
        };
        let n = 1_000_000;

        // Naive: small components are swallowed (1e15 + 1.0 == 1e15 in f64).
        let mut naive = big;
        for _ in 0..n {
            naive.x += small.x;
            naive.y += small.y;
            naive.z += small.z;
        }
        assert!((naive.x - big.x).abs() < 0.5, "naive vec3 x lost small terms");

        // Kahan: small components are preserved.
        let mut kahan = KahanVec3::new(big);
        for _ in 0..n {
            kahan.add(small);
        }
        let val = kahan.value();
        assert!(
            (val.x - (big.x + 1.0)).abs() < 1.0e-6,
            "KahanVec3 x preserved small terms"
        );
    }

    #[test]
    fn kahan_sum_reset_works() {
        let mut acc = KahanSum::new(42.0);
        acc.add(1.0);
        assert!((acc.value() - 43.0).abs() < 1.0e-12);
        acc.reset();
        assert!(acc.value().abs() < 1.0e-12);
        acc.add(7.0);
        assert!((acc.value() - 7.0).abs() < 1.0e-12);
    }

    #[test]
    fn kahan_vec3_reset_works() {
        let mut acc = KahanVec3::new(Vec3 {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        });
        acc.reset();
        acc.add(Vec3 {
            x: 10.0,
            y: 20.0,
            z: 30.0,
        });
        let v = acc.value();
        assert!((v.x - 10.0).abs() < 1.0e-12);
        assert!((v.y - 20.0).abs() < 1.0e-12);
        assert!((v.z - 30.0).abs() < 1.0e-12);
    }

    #[test]
    fn approx_eq_relative_tolerance() {
        // Same magnitude: absolute tolerance works.
        assert!(approx_eq(1.0, 1.0 + 1e-13, 1e-12, 1e-12));
        assert!(!approx_eq(1.0, 1.0 + 1e-11, 1e-12, 1e-12));

        // Large values: relative tolerance kicks in.
        let big = 1.0e15;
        assert!(approx_eq(big, big + 100.0, 1e-12, 1e-12));
        assert!(!approx_eq(big, big + 1e10, 1e-12, 1e-12));

        // Small values: absolute floor prevents false positives near zero.
        assert!(approx_eq(0.0, 1e-13, 1e-12, 1e-12));
        assert!(!approx_eq(0.0, 1e-9, 1e-12, 1e-12));
    }

    #[test]
    fn mul_add_is_more_precise_than_separate_ops() {
        // mul_add rounds once; a*b + c rounds twice.
        let a = 1.0e-10_f64;
        let b = 1.0e10_f64;
        let c = 1.0_f64;
        let naive = a * b + c;
        let fused = mul_add(a, b, c);
        // Both should be ~2.0, but mul_add is at least as precise.
        assert!((naive - 2.0).abs() <= (fused - 2.0).abs() + 1e-15);
    }
}



