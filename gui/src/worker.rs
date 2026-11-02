//! The thread that owns the board handle.
//!
//! Every USB round trip here is blocking and some of them are slow - an
//! acceptance run is minutes, a bench sweep is seconds - so none of it may
//! happen on the UI thread. The window sends `Req` and receives `Evt`, and
//! never touches the handle itself.
//!
//! One worker, one board, one handle: a board's USB interface belongs to one
//! process at a time, and inside this process to one thread. That is not a
//! style choice. Two threads sharing the handle would interleave a command
//! with another's reply, and the symptom - every answer belonging to the
//! previous question - looks exactly like a firmware fault.

use std::sync::mpsc::{Receiver, Sender, TryRecvError};
use std::time::{Duration, Instant};

use crate::device::{self, Board, Device, Mark};

pub enum Req {
    Connect(String),
    Disconnect,
    /// Anything the UI has no dedicated control for, typed by hand.
    Raw(String),
    /// Push the whole capture configuration, in the order the firmware needs.
    Configure(CaptureCfg),
    Start,
    Stop,
    Clear,
    /// Drain decoded events once.
    ReadEvents,
    /// Peek the controller trace, with timestamps.
    ReadTrace,
    /// Keep draining events until told otherwise.
    Follow(bool),
    /// Arm the edge-list view for the waveform, with a budget in edges.
    RawStart(u32),
    RawStop,
    Status,
    Quit,
}

#[derive(Clone, PartialEq)]
pub struct CaptureCfg {
    pub proto: String,
    pub source: String,
    pub mode: String,
    pub taps: String,
    pub rate_hz: u32,
    pub sample_hz: u32,
    pub dma: bool,
    pub decode: bool,
}

impl Default for CaptureCfg {
    fn default() -> Self {
        // The I3C defaults this bench actually runs: tap pair d8/d9, 1 MHz,
        // pin-clocked with the DMA intake. Chosen so that pressing Start on a
        // freshly opened window does something correct rather than something
        // that needs four changes first.
        CaptureCfg {
            proto: "i3c".into(),
            source: "wire".into(),
            mode: "pinclock".into(),
            taps: "d8 d9".into(),
            rate_hz: 1_000_000,
            sample_hz: 0,
            dma: true,
            decode: true,
        }
    }
}

pub enum Evt {
    Connected {
        serial: String,
        board: Board,
        ident: String,
    },
    Disconnected,
    /// A command and what it answered, for the log.
    Reply {
        cmd: String,
        reply: String,
    },
    Events(Vec<String>),
    /// One SNIFF:RAW:READ? reply, unparsed.
    ///
    /// Handed over as text rather than as edges because the 32-bit tick has to
    /// be unwrapped across replies, and that running state belongs with the
    /// buffer it accumulates into - not duplicated here, where a second copy
    /// could disagree with the first about where the capture started.
    RawData(String),
    RawArmed(u64),
    Marks(Vec<Mark>, u64),
    Status(String),
    Error(String),
}

pub fn spawn() -> (Sender<Req>, Receiver<Evt>) {
    let (tx_req, rx_req) = std::sync::mpsc::channel::<Req>();
    let (tx_evt, rx_evt) = std::sync::mpsc::channel::<Evt>();
    std::thread::Builder::new()
        .name("board".into())
        .spawn(move || run(rx_req, tx_evt))
        .expect("worker thread");
    (tx_req, rx_evt)
}

fn run(rx: Receiver<Req>, tx: Sender<Evt>) {
    let mut dev: Option<Device> = None;
    let mut following = false;
    let mut raw = false;
    // Whether the controller trace is one of the running engines. Set from the
    // configuration rather than asked for separately, so it cannot disagree
    // with what the board was actually told.
    let mut tracing = false;
    let mut next_poll = Instant::now();

    loop {
        // A blocking receive while idle, and a non-blocking one while
        // following, so a window that is watching a live bus does not wait on
        // the channel for its next drain.
        let req = if following {
            match rx.try_recv() {
                Ok(r) => Some(r),
                Err(TryRecvError::Empty) => None,
                Err(TryRecvError::Disconnected) => return,
            }
        } else {
            match rx.recv() {
                Ok(r) => Some(r),
                Err(_) => return,
            }
        };

        if let Some(req) = req {
            match req {
                Req::Quit => return,
                Req::Connect(serial) => {
                    dev = None; // close the old one first: one handle at a time
                    following = false;
                    match Device::open(&serial) {
                        Ok(d) => {
                            let _ = tx.send(Evt::Connected {
                                serial: d.serial.clone(),
                                board: d.board,
                                ident: d.ident.clone(),
                            });
                            dev = Some(d);
                        }
                        Err(e) => {
                            let _ = tx.send(Evt::Error(e));
                        }
                    }
                }
                Req::Disconnect => {
                    dev = None;
                    following = false;
                    let _ = tx.send(Evt::Disconnected);
                }
                Req::Follow(on) => {
                    following = on && dev.is_some();
                    next_poll = Instant::now();
                }
                Req::RawStart(budget) => {
                    let Some(d) = dev.as_ref() else {
                        let _ = tx.send(Evt::Error("not connected".into()));
                        continue;
                    };
                    let line = format!("SNIFF:RAW:START {budget}");
                    let r = d.command(&line);
                    if Device::ok(&r) {
                        // tick_hz is the sample clock the engine settled on,
                        // which is not the rate that was asked for - it is
                        // divided down from a root and the achievable value is
                        // the engine's to choose. Drawing a time axis from the
                        // requested rate would be a plausible wrong answer.
                        let hz = device::field_u64(&r, "tick_hz=").unwrap_or(10_000_000);
                        let _ = tx.send(Evt::RawArmed(hz));
                        raw = true;
                        following = true;
                        next_poll = Instant::now();
                    }
                    say(&tx, &line, r);
                }
                Req::RawStop => {
                    let was = raw;
                    raw = false;
                    if let Some(d) = dev.as_ref().filter(|_| was) {
                        let r = d.command("SNIFF:RAW:STOP");
                        say(&tx, "SNIFF:RAW:STOP", r);
                        drain_raw(d, &tx);
                    }
                }
                other => {
                    if let Req::Configure(ref c) = other {
                        tracing = c.source != "wire";
                    }
                    let Some(d) = dev.as_ref() else {
                        let _ = tx.send(Evt::Error("not connected".into()));
                        continue;
                    };
                    handle(d, other, &tx, &mut following, &mut raw);
                }
            }
        }

        if following {
            if let Some(d) = dev.as_ref() {
                if Instant::now() >= next_poll {
                    // Peek BEFORE draining, and only in a mode that has a
                    // trace to peek.
                    //
                    // In `self` and `both` the controller ring is what
                    // SNIFF:READ? drains, so the live event poll was eating
                    // the very events the trace view peeks at - press Read
                    // trace after a capture and it answered n=0 while the
                    // events sat on screen beside it. Peeking first means the
                    // marks are exactly what the drain is about to take.
                    if tracing {
                        peek_trace(d, &tx);
                    }
                    drain_events(d, &tx);
                    if raw {
                        drain_raw(d, &tx);
                    }
                    // 50 ms: fast enough that a running capture looks live,
                    // slow enough that the drain is not most of what the board
                    // is doing. The firmware's ring is the thing that bounds
                    // loss, not this interval.
                    next_poll = Instant::now() + Duration::from_millis(50);
                }
            } else {
                following = false;
            }
        } else {
            // Nothing to do and nothing to wait for: the next loop blocks on
            // recv(), so no sleep is needed here.
        }
    }
}

fn say(tx: &Sender<Evt>, cmd: &str, reply: String) {
    let _ = tx.send(Evt::Reply {
        cmd: cmd.to_string(),
        reply,
    });
}

fn handle(d: &Device, req: Req, tx: &Sender<Evt>, following: &mut bool, raw: &mut bool) {
    match req {
        Req::Raw(line) => {
            let r = d.command(&line);
            say(tx, &line, r);
        }
        Req::Configure(cfg) => {
            // Order matters and is not obvious.
            //
            // SNIFF:CH re-derives the tap set when the protocol changes, so
            // the channel binding has to follow the protocol, not precede it.
            // SNIFF:MODE refuses pinclock with fewer than two taps, so it has
            // to follow the binding. Rate last, because it is the only one
            // whose sensible value depends on the rest.
            let mut lines = vec![
                format!("SNIFF:SRC {}", cfg.source),
                format!("SNIFF:CH {} {}", cfg.proto, cfg.taps),
                format!("SNIFF:MODE {}", cfg.mode),
                format!("SNIFF:DMA {}", if cfg.dma { 1 } else { 0 }),
                format!("SNIFF:DECODE {}", if cfg.decode { 1 } else { 0 }),
            ];
            lines.push(if cfg.sample_hz > 0 {
                format!("SNIFF:RATE {} {}", cfg.rate_hz, cfg.sample_hz)
            } else {
                format!("SNIFF:RATE {}", cfg.rate_hz)
            });
            for l in lines {
                let r = d.command(&l);
                let bad = !Device::ok(&r);
                say(tx, &l, r);
                if bad {
                    // Stop at the first refusal rather than pushing the rest
                    // onto a configuration the board has already rejected.
                    let _ = tx.send(Evt::Error(format!("configuration stopped at: {l}")));
                    return;
                }
            }
        }
        Req::Start => {
            let r = d.command("SNIFF:START");
            let ok = Device::ok(&r);
            say(tx, "SNIFF:START", r);
            *following = ok;
        }
        Req::Stop => {
            *following = false;
            // One stop, not two.
            //
            // SNIFF:RAW:STOP and SNIFF:STOP both send BB_MSG_STOP to the
            // capture core - they are the same stop wearing two names, and the
            // only difference is that the raw one is where the edge cursor is
            // handled. Sending both always made the second answer
            // `capture-not-running`, which reads as a capture that died on its
            // own.
            let line = if *raw { "SNIFF:RAW:STOP" } else { "SNIFF:STOP" };
            let r = d.command(line);
            let ok = Device::ok(&r);
            say(tx, line, r);
            if *raw {
                if ok {
                    drain_raw(d, tx);
                }
                *raw = false;
            }
            // One last drain: events captured just before the stop are still
            // in the ring, and a view that loses them makes a short capture
            // look like a failed one.
            drain_events(d, tx);
        }
        Req::Clear => {
            let r = d.command("SNIFF:CLEAR");
            say(tx, "SNIFF:CLEAR", r);
        }
        Req::ReadEvents => drain_events(d, tx),
        Req::ReadTrace => {
            let r = d.command("SNIFF:TRACE? 60");
            if Device::ok(&r) {
                let hz = device::field_u64(&r, "cycle_hz=").unwrap_or(240_000_000);
                let _ = tx.send(Evt::Marks(device::marks(&r), hz));
            }
            say(tx, "SNIFF:TRACE? 60", r);
        }
        Req::Status => {
            let r = d.command("SNIFF:STAT?");
            let _ = tx.send(Evt::Status(r.clone()));
            say(tx, "SNIFF:STAT?", r);
        }
        _ => {}
    }
}

/// Take whatever decoded events the board has to hand.
///
/// Asks for 120 and takes what comes, rather than looping until `more=0`. The
/// capture core hands over what it has; a read that comes back short while
/// records are still queued is ordinary, and the caller comes back. Looping
/// here would hold the worker in a drain while the UI waits for the frame.
/// Take whatever edges the board has to hand, one reply's worth.
///
/// 79 records, because that is what a reply line holds - the firmware bounds
/// it at BB_REPLY_BYTES_MAX and asking for more gets a refusal rather than a
/// long line. Asking for exactly the maximum keeps the round trips down on a
/// busy bus, where the waveform is the thing most likely to fall behind.
fn drain_raw(d: &Device, tx: &Sender<Evt>) {
    for _ in 0..8 {
        let r = d.command("SNIFF:RAW:READ? 79");
        if !Device::ok(&r) {
            return;
        }
        let payload = r.strip_prefix("OK").unwrap_or("").trim();
        if payload.is_empty() {
            return;
        }
        let whole = payload.split_whitespace().count() == 79 * 5;
        let _ = tx.send(Evt::RawData(r));
        if !whole {
            // Short reply: the board has handed over what it had. Coming back
            // immediately would spin against an idle bus.
            return;
        }
    }
}

/// Peek the controller trace without consuming it.
///
/// Quiet - no entry in the transcript. This runs twenty times a second while a
/// capture is live, and a log that is nine tenths its own polling is a log
/// nobody reads.
fn peek_trace(d: &Device, tx: &Sender<Evt>) {
    let r = d.command("SNIFF:TRACE? 60");
    if !Device::ok(&r) {
        return;
    }
    let m = device::marks(&r);
    if !m.is_empty() {
        let hz = device::field_u64(&r, "cycle_hz=").unwrap_or(240_000_000);
        let _ = tx.send(Evt::Marks(m, hz));
    }
}

fn drain_events(d: &Device, tx: &Sender<Evt>) {
    let r = d.command("SNIFF:READ? 120");
    if !Device::ok(&r) {
        return;
    }
    let toks = device::tokens(&r);
    if !toks.is_empty() {
        let _ = tx.send(Evt::Events(toks));
    }
}
