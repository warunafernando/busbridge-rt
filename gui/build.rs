//! Link the BUSBRIDGE C library into this GUI.
//!
//! Statically, so the application ships as one executable: no
//! "busbridge.dll was not found" for anyone who moves the .exe. `setupapi` and
//! `advapi32` are what the WinUSB enumeration inside it needs.
//!
//! The library lives in the STM32H5 repository, which is where it is built and
//! tested, so this reaches across trees by default. Override with
//! BUSBRIDGE_LIB_DIR for a checkout somewhere else. That cross-tree reach is
//! the same shape acceptance.py already has for the analyser, and the same
//! justification: the host library and this firmware have to agree, and the
//! worst defect this project ever had was a disagreement between exactly those
//! two.
use std::env;
use std::path::PathBuf;

fn main() {
    let arch = match env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_default().as_str() {
        "x86" => "x86",
        _ => "x64",
    };

    let lib_dir = env::var("BUSBRIDGE_LIB_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| {
            PathBuf::from(r"E:\STMicro\busbridge\windows\build").join(arch)
        });

    if !lib_dir.join("busbridge_static.lib").exists() {
        panic!(
            "busbridge_static.lib not found in {}\n\
             Build the C library first:  E:\\STMicro\\busbridge\\windows\\build.ps1\n\
             Or point BUSBRIDGE_LIB_DIR at it.",
            lib_dir.display()
        );
    }

    println!("cargo:rustc-link-search=native={}", lib_dir.display());
    println!("cargo:rustc-link-lib=static=busbridge_static");
    println!("cargo:rustc-link-lib=dylib=setupapi");
    println!("cargo:rustc-link-lib=dylib=advapi32");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=BUSBRIDGE_LIB_DIR");
}
