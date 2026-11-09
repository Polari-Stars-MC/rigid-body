//! Mirror of `mps-cosmos/src/error.rs` — the crate-local `ERR_*` literals that
//! cbindgen turns into `#define`s in `cosmos.h`.

#[cfg(test)]
mod tests {
    use mps_cosmos::error::{
        ERR_CAPACITY, ERR_INTERNAL, ERR_INVALID_ARGUMENT, ERR_NOT_FOUND, ERR_NULL_POINTER,
        ERR_OK, ERR_UNSUPPORTED,
    };

    #[test]
    fn cosmos_err_literals_match_formula() {
        // The cosmos-side literals must equal the canonical mps-formula codes
        // (the compile-time `const _: () = { assert!(...) }` in
        // `mps_cosmos::error` already pins this; this test pins it at runtime
        // against the formula crate).
        assert_eq!(ERR_OK, mps_formula::error::ERR_OK);
        assert_eq!(ERR_NULL_POINTER, mps_formula::error::ERR_NULL_POINTER);
        assert_eq!(ERR_INVALID_ARGUMENT, mps_formula::error::ERR_INVALID_ARGUMENT);
        assert_eq!(ERR_NOT_FOUND, mps_formula::error::ERR_NOT_FOUND);
        assert_eq!(ERR_CAPACITY, mps_formula::error::ERR_CAPACITY);
        assert_eq!(ERR_UNSUPPORTED, mps_formula::error::ERR_UNSUPPORTED);
        assert_eq!(ERR_INTERNAL, mps_formula::error::ERR_INTERNAL);
    }

    #[test]
    fn cosmos_err_literals_are_0_through_6() {
        assert_eq!(ERR_OK, 0);
        assert_eq!(ERR_NULL_POINTER, 1);
        assert_eq!(ERR_INVALID_ARGUMENT, 2);
        assert_eq!(ERR_NOT_FOUND, 3);
        assert_eq!(ERR_CAPACITY, 4);
        assert_eq!(ERR_UNSUPPORTED, 5);
        assert_eq!(ERR_INTERNAL, 6);
    }
}
