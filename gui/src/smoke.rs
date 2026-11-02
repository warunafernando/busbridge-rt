//! `--smoke`: drive the window's own code paths with no window.
//!
//! A GUI is the one part of this project that nothing else can check. The
//! firmware has acceptance.py, the host library has unit_test.c, the analyser
//! has its golden corpus - and a window has a person clicking, which is not a
//! test and does not run twice the same way.
//!
//! So this takes the real worker, the real configuration ordering and the real
//! reply parsing, and runs a capture through them end to end: configure, start,
//! drive known traffic, drain, stop. It asserts on what comes back rather than
//! on whether anything threw, because the failure this guards against is a
//! panel that looks right and sends the wrong commands in the wrong order.
//!
//! It is in the shipped binary rather than in a test harness on purpose. The
//! thing worth checking is the path the window actually takes, and a copy of
//! that path in a test file is a copy that can drift from it.

use std::sync::mpsc::RecvTimeoutError;
use std::time::{Duration, Instant};

use crate::device::Board;
use crate::worker::{self, CaptureCfg, Evt, Req};

const WRITE: &str = "S A12+ D00+ D40- P";
const READ: &str = "S A13+ D40+ D11+ D22+ D33+ D44+ D55+";

pub fn run(serial_prefix: Option<String>) -> i32 {
    let mut fails = 0usize;
    let mut step = |name: &str, ok: bool, detail: &str| {
        println!(
            "  {:<52} {}  {}",
            name,
            if ok { "PASS" } else { "FAIL" },
            &detail[..detail.len().min(44)]
        );
        if !ok {
            fails += 1;
        }
    };

    println!("== busbridge-rt-gui smoke: the window's own code paths ==");

    let boards = crate::device::list();
    let want = serial_prefix.unwrap_or_default();
    let Some(info) = boards.iter().find(|b| b.serial.starts_with(&want)) else {
        println!("  no attached board matches {want:?}; nothing to test.");
        return 1;
    };

    let (tx, rx) = worker::spawn();
    let _ = tx.send(Req::Connect(info.serial.clone()));

    // Connect
    let mut board = Board::Unknown;
    let mut connected = false;
    let deadline = Instant::now() + Duration::from_secs(15);
    while Instant::now() < deadline && !connected {
        match rx.recv_timeout(Duration::from_millis(500)) {
            Ok(Evt::Connected { board: b, ident, .. }) => {
                board = b;
                connected = true;
                step("the board opens and says what it is", true, &ident);
            }
            Ok(Evt::Error(e)) => {
                step("the board opens and says what it is", false, &e);
                return 1;
            }
            Ok(_) => {}
            Err(RecvTimeoutError::Timeout) => {}
            Err(_) => break,
        }
    }
    if !connected {
        step("the board opens and says what it is", false, "timed out");
        return 1;
    }
    if !board.has_capture_engine() {
        println!("  {} has no capture engine - the rest is RT1186-only.", board.label());
        let _ = tx.send(Req::Quit);
        return if fails > 0 { 1 } else { 0 };
    }

    // The same configuration the Capture tab pushes, through the same request.
    let cfg = CaptureCfg {
        proto: "i3c".into(),
        source: "wire".into(),
        mode: "oversample".into(),
        taps: "d8 d9".into(),
        rate_hz: 1_000_000,
        sample_hz: 0,
        dma: true,
        decode: true,
    };
    let _ = tx.send(Req::Configure(cfg));

    // The target has to be told to be an I3C target before anything will
    // answer ENTDAA, and that is the other board's business. The window only
    // ever holds one board, so this opens the second directly rather than
    // through the worker - which is also what acceptance.py does, and why it
    // takes two boards to run.
    //
    // Without this the smoke test reported targets=0 and four failed frame
    // checks, none of which were the window's fault. A test that fails for
    // something it did not set up teaches the wrong lesson.
    let target = boards
        .iter()
        .find(|b| b.serial != info.serial)
        .and_then(|b| crate::device::Device::open(&b.serial).ok());
    match &target {
        Some(t) => {
            for line in ["EE:I3C OFF", "EE:I3C ON", "EE:FILL PATTERN",
                         "EE:WR 0x40 40 11 22 33 44 55"] {
                let r = t.command(line);
                if !crate::device::Device::ok(&r) {
                    println!("  target setup: {line} -> {r}");
                }
            }
        }
        None => println!("  no second board: the target's I3C cannot be enabled here"),
    }

    // The controller side, driven exactly as a person would from the console.
    for line in [
        "I3C:FREE",
        "I3C:PORT 2",
        "I3C:ROOT 24000000",
        "I3C:FREQ 1000000 500000",
        "I3C:ENTDAA",
    ] {
        let _ = tx.send(Req::Raw(line.into()));
    }
    let _ = tx.send(Req::Clear);
    let _ = tx.send(Req::Start);
    let _ = tx.send(Req::Raw("I3C:W 0x09 00 40".into()));
    let _ = tx.send(Req::Raw("I3C:R 0x09 6".into()));
    let _ = tx.send(Req::Stop);
    let _ = tx.send(Req::Status);

    // Collect until the status reply arrives, which is the last thing queued.
    let mut events: Vec<String> = Vec::new();
    let mut cfg_refused = None;
    let mut daa = String::new();
    let mut status = String::new();
    let deadline = Instant::now() + Duration::from_secs(30);
    while Instant::now() < deadline && status.is_empty() {
        match rx.recv_timeout(Duration::from_millis(500)) {
            Ok(Evt::Events(mut v)) => events.append(&mut v),
            Ok(Evt::Status(s)) => status = s,
            Ok(Evt::Reply { cmd, reply }) => {
                if cmd == "I3C:ENTDAA" {
                    daa = reply.clone();
                }
                if reply.starts_with("ERR") && cmd.starts_with("SNIFF:") {
                    cfg_refused.get_or_insert(format!("{cmd} -> {reply}"));
                }
            }
            Ok(Evt::Error(e)) => {
                if cfg_refused.is_none() && e.contains("configuration") {
                    cfg_refused = Some(e);
                }
            }
            Ok(_) => {}
            Err(RecvTimeoutError::Timeout) => {}
            Err(_) => break,
        }
    }

    step(
        "the capture configuration is accepted in order",
        cfg_refused.is_none(),
        cfg_refused.as_deref().unwrap_or(""),
    );
    step(
        "a target answers dynamic address assignment",
        daa.contains("targets=1"),
        &daa,
    );

    let joined = events.join(" ");
    step(
        "the window's own parsing finds the write frame",
        joined.contains(WRITE),
        &joined,
    );
    step(
        "and the read frame",
        joined.contains(READ),
        &joined,
    );
    step(
        "no header field leaked into the event stream",
        !events.iter().any(|t| t.contains('=')),
        &joined,
    );
    step(
        "SNIFF:STAT? came back",
        status.starts_with("OK"),
        &status,
    );

    let _ = tx.send(Req::Quit);
    println!();
    println!("  {} check(s) failed", fails);
    if fails > 0 {
        1
    } else {
        0
    }
}
