use std::fs;

fn main() {
    println!("cargo:rerun-if-changed=VERSION");

    let version = fs::read_to_string("VERSION")
        .expect("VERSION file missing")
        .trim()
        .to_string();

    println!("cargo:rustc-env=DAEMON_VERSION={}", version);
}