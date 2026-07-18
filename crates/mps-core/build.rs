use std::env;
use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-changed=src");
    println!("cargo:rerun-if-changed=cbindgen.toml");

    let crate_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").expect("manifest dir"));
    let include_dir = crate_dir.join("include");
    let header_path = include_dir.join("rigid_body.h");

    std::fs::create_dir_all(&include_dir).expect("create include directory");

    let config = cbindgen::Config::from_file(crate_dir.join("cbindgen.toml")).unwrap_or_default();
    cbindgen::Builder::new()
        .with_config(config)
        .with_crate(crate_dir)
        .generate()
        .expect("generate C header")
        .write_to_file(header_path);
}
