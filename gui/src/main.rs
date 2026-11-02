//! Bench console for BUSBRIDGE-RT.
//!
//! A second window rather than a change to the bench console in the STM32H5
//! tree, and the reason is the instrument rather than the code. That console
//! drives the shared command surface well and already connects to this board -
//! what it cannot express is the capture engine: mode, tap binding, sample
//! rate, and where a record came from. Those are not extra controls on an
//! existing panel, they are the thing being operated.
//!
//! It still talks to both boards. An STM32H5 answers everything in the console
//! at the bottom; the capture tabs are shown disabled, with the reason, because
//! a window that silently looks different on two boards teaches nobody why.
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod app;
mod device;
mod ffi;
mod runner;
mod smoke;
mod style;
mod wave;
mod worker;

fn main() -> eframe::Result<()> {
    // The scripts live beside this crate, one level up. Overridable because a
    // checkout somewhere else is ordinary, and because the alternative is a
    // window that silently has no bench tab.
    let scripts = std::env::var("BUSBRIDGE_SCRIPTS").unwrap_or_else(|_| {
        std::env::current_exe()
            .ok()
            .and_then(|p| {
                // target/<profile>/exe -> crate -> repo root
                p.parent()?.parent()?.parent()?.parent()
                    .map(|r| r.join("scripts").display().to_string())
            })
            .unwrap_or_else(|| r"E:\BusBridge-RT\scripts".into())
    });

    // --connect <serial> opens that board on startup. It exists so the window
    // can be exercised without a person clicking, which is the only way the
    // connect path gets tested at all - and the bench console next door has
    // the same flag for the same reason.
    let mut connect = None;
    let mut smoke = false;
    let mut wave = false;
    let mut argv = std::env::args().skip(1);
    while let Some(a) = argv.next() {
        match a.as_str() {
            "--connect" => connect = argv.next(),
            "--wave" => wave = true,
            "--smoke" => {
                smoke = true;
                connect = argv.next();
            }
            "--help" | "-h" => {
                eprintln!("busbridge-rt-gui [--connect <serial>] [--smoke <serial>]");
                eprintln!("  --smoke  drive the capture path with no window, and exit");
                eprintln!("  --wave   open with the waveform panel already on");
                eprintln!("  BUSBRIDGE_SCRIPTS  where the bench scripts live");
                eprintln!("  SNIFFER_EXE        the waveform analyser, for Files");
                return Ok(());
            }
            other => eprintln!("ignoring unknown argument: {other}"),
        }
    }

    if smoke {
        // No window at all. The smoke test is for a terminal and for CI, and a
        // window that opens and closes is noise in both.
        std::process::exit(smoke::run(connect));
    }

    let opts = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1180.0, 820.0])
            .with_min_inner_size([900.0, 600.0])
            .with_title("BUSBRIDGE-RT bench console"),
        ..Default::default()
    };
    eframe::run_native(
        "busbridge-rt-gui",
        opts,
        Box::new(move |cc| {
            Ok(Box::new(app::App::new(
                scripts,
                connect,
                wave,
                cc.egui_ctx.clone(),
            )))
        }),
    )
}
