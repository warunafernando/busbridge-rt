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
                other => {
                    let Some(d) = dev.as_ref() else {
                        let _ = tx.send(Evt::Error("not connected".into()));
                        continue;
                    };
                    handle(d, other, &tx, &mut following);
                }
            }
        }

        if following {
            if let Some(d) = dev.as_ref() {
                if Instant::now() >= next_poll {
                    drain_events(d, &tx);
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

fn handle(d: &Device, req: Req, tx: &Sender<Evt>, following: &mut bool) {
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
            let r = d.command("SNIFF:STOP");
            say(tx, "SNIFF:STOP", r);
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
