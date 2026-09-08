# Formula Numeric Domains

## Contract

Checked pure Rust formulas return `Result<T, mps_formula::FormulaError>`.
They do not access the thread-local FFI error slot. `domain` provides finite,
positive, nonnegative, square-root, logarithm, division and finite-result checks.

- NaN and either infinity are invalid inputs, including unused inputs in a zero limit.
- Physical constraints are explicit per parameter; signed quantities are not
  rejected merely because they are negative.
- Division by either signed zero and invalid real square-root/logarithm domains fail.
- Non-finite intermediate values or results fail. This conservative policy can
  reject extreme finite inputs whose exact mathematical result is representable.
- Subnormal values and finite underflow to zero follow IEEE 754. A denominator
  that underflows to zero is still a division-by-zero error.
- A formula may allow an infinite output only where its documentation names
  the specific field and physical limit. There is no blanket allow-infinity flag.

## FFI Mapping

`ffi::formula_result` maps domain errors to `ERR_INVALID_ARGUMENT` with a detailed
message and clears the previous error on success. Existing C signatures and
failure sentinels remain unchanged: scalar interfaces may return NaN on failure;
report interfaces return false and leave the caller's output untouched.
Pointer errors remain `ERR_NULL_POINTER` and are separate from numeric errors.

## Migration Coverage

Checked APIs currently exist for:

| Module | API | Domain or special limit |
| --- | --- | --- |
| `spaceflight` | `ballistic_coefficient_checked` | Positive mass, drag coefficient and area; no infinite coefficient |
| `spaceflight` | `kepler_period_checked` | Positive mu and semi-major axis |
| `spaceflight` | `tsiolkovsky_delta_v_checked` | Positive inputs, initial mass >= final mass; equal masses give zero |
| `spaceflight` | `semi_major_axis_decay_rate_checked` | Positive axis/mass/mu, nonnegative density/drag/area; vacuum gives zero |
| `fluid` | `bernoulli_pressure_checked` | Positive density, nonnegative speed; signed pressure/gravity/elevation |
| `fluid` | `bernoulli_report_checked` | Same domains; zero gravity is invalid for total head |
| `thermodynamics` | `fourier_conduction_checked` | Finite temperatures, nonnegative conductivity/area, positive thickness |

The existing Option/scalar Rust APIs delegate to these checked implementations
and preserve their signatures and failure sentinels. Fourier conduction now has
a pure checked API in addition to its existing C entry point. The corresponding
Kepler-period, rocket-equation, decay-rate, Bernoulli and conduction C wrappers
use the shared error mapping rather than duplicate the computation.

Other legacy formulas are not yet covered by this contract. Migration should
replace each formula and its FFI duplicate together, with tests for every input,
output overflow, physical bounds and intentional singularities.

## Intentional Infinite Output

`fourier_conduction_checked` and `thermal_fourier_conduction` return positive
infinity in `thermal_resistance` when conductivity or area is exactly zero.
This represents an insulator; heat rate is zero and all other fields remain
finite. Finite positive conductivity/area whose product underflows to zero is
not this physical limit and returns an error. Negative and infinite inputs are
invalid. Zero final rocket mass and zero Bernoulli head gravity are errors,
not permitted infinite results.
