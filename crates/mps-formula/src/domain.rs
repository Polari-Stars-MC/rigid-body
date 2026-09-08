//! Numeric-domain policy for checked formulas.
//!
//! Inputs must be finite. A formula additionally specifies positive,
//! nonnegative, or other physical constraints. Non-finite computed results
//! are errors unless the formula explicitly documents a singular limit.
//! Validation never reads or writes the thread-local FFI error slot.

use std::fmt;

/// A numerical or physical domain failure in a pure formula.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum FormulaError {
    NullPointer {
        parameter: &'static str,
    },
    InvalidLength {
        parameter: &'static str,
    },
    MisalignedPointer {
        parameter: &'static str,
    },
    CapacityOverflow {
        parameter: &'static str,
    },
    NonFiniteInput {
        parameter: &'static str,
    },
    OutOfDomain {
        parameter: &'static str,
        requirement: &'static str,
    },
    DivisionByZero,
    NonFiniteResult {
        quantity: &'static str,
    },
}

impl fmt::Display for FormulaError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NullPointer { parameter } => write!(f, "{parameter} pointer is null"),
            Self::InvalidLength { parameter } => write!(f, "{parameter} length is invalid"),
            Self::MisalignedPointer { parameter } => write!(f, "{parameter} pointer is misaligned"),
            Self::CapacityOverflow { parameter } => {
                write!(f, "{parameter} length overflows capacity")
            }
            Self::NonFiniteInput { parameter } => write!(f, "{parameter} must be finite"),
            Self::OutOfDomain {
                parameter,
                requirement,
            } => write!(f, "{parameter} must be {requirement}"),
            Self::DivisionByZero => f.write_str("division by zero"),
            Self::NonFiniteResult { quantity } => {
                write!(f, "{quantity} is not representable as a finite f64")
            }
        }
    }
}

impl std::error::Error for FormulaError {}

pub fn finite(value: f64, parameter: &'static str) -> Result<f64, FormulaError> {
    if value.is_finite() {
        Ok(value)
    } else {
        Err(FormulaError::NonFiniteInput { parameter })
    }
}

pub fn positive(value: f64, parameter: &'static str) -> Result<f64, FormulaError> {
    finite(value, parameter)?;
    if value > 0.0 {
        Ok(value)
    } else {
        Err(FormulaError::OutOfDomain {
            parameter,
            requirement: "positive",
        })
    }
}

pub fn nonnegative(value: f64, parameter: &'static str) -> Result<f64, FormulaError> {
    finite(value, parameter)?;
    if value >= 0.0 {
        Ok(value)
    } else {
        Err(FormulaError::OutOfDomain {
            parameter,
            requirement: "nonnegative",
        })
    }
}

pub fn finite_result(value: f64, quantity: &'static str) -> Result<f64, FormulaError> {
    if value.is_finite() {
        Ok(value)
    } else {
        Err(FormulaError::NonFiniteResult { quantity })
    }
}

/// Real square root; negative arguments (including rounding errors) are rejected.
pub fn sqrt(value: f64) -> Result<f64, FormulaError> {
    Ok(nonnegative(value, "square root argument")?.sqrt())
}

/// Natural logarithm. Zero is an error, not a successful negative infinity.
pub fn ln(value: f64) -> Result<f64, FormulaError> {
    Ok(positive(value, "logarithm argument")?.ln())
}

/// Finite division; both positive and negative zero denominators are errors.
pub fn divide(numerator: f64, denominator: f64) -> Result<f64, FormulaError> {
    finite(numerator, "numerator")?;
    finite(denominator, "denominator")?;
    if denominator == 0.0 {
        return Err(FormulaError::DivisionByZero);
    }
    finite_result(numerator / denominator, "quotient")
}
