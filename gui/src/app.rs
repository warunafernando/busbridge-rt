//! The window.
//!
//! Five tabs, in the order the instrument is actually used: pick a board, set
//! the capture up and run it, look at where the records came from, write or
//! open a file, and run the bench measurements. A console underneath all of
//! them, because every control here is a command line and hiding that from
//! someone debugging their own firmware helps nobody.
//!
//! RT-only controls are shown and disabled on a board that lacks the capture
//! engine, with the reason on the hover. Hiding them would make the window
//! look different on two boards for no stated cause; greying them out says
//! "this exists, not here".

use std::sync::mpsc::{Receiver, Sender};

use eframe::egui;
use egui::{Color32, RichText};

use crate::device::{self, Board, Mark};
use crate::runner::Run;
use crate::style::{self, Palette};
use crate::wave::{self, Edges, View};
use crate::worker::{self, CaptureCfg, Evt, Req};

const MAX_EVENTS: usize = 20_000;
const MAX_LOG: usize = 2_000;

#[derive(PartialEq, Clone, Copy)]
enum Tab {
    Board,
    Capture,
    Trace,
    Files,
    Bench,
}

pub struct App {
    tx: Sender<Req>,
    rx: Receiver<Evt>,

    tab: Tab,
    boards: Vec<device::Info>,
    picked: Option<String>,
    connected: Option<(String, Board, String)>,
    err: Option<String>,

    cfg: CaptureCfg,
    pushed: Option<CaptureCfg>,
    running: bool,
    events: Vec<String>,
    status: String,

    marks: Vec<Mark>,
    marks_hz: u64,

    wave_on: bool,
    edges: Edges,
    view: View,
    wave_err: Option<String>,

    log: Vec<(String, String)>,
    raw_entry: String,
    /// Commands typed, newest last. A bench console without history is a
    /// console where every repeated command is retyped, and the commands here
    /// are long and exact - `SNIFF:CH i3c d8 d9` does not survive a typo.
    history: Vec<String>,
    /// Where the up-arrow has walked back to. None means "at the live entry".
    hist_pos: Option<usize>,
    log_errors_only: bool,

    runs: Vec<Run>,
    /// The board this window was on before it handed it to a script, so it can
    /// take it back when the script is done.
    resume: Option<String>,
    last_resume: std::time::Instant,
    scripts_dir: String,
    cap_path: String,
    sniffer_exe: String,
}

impl App {
    pub fn new(
        scripts_dir: String,
        connect: Option<String>,
        wave: bool,
        ctx: egui::Context,
    ) -> App {
        let (tx, rx) = worker::spawn(move || ctx.request_repaint());
        let mut a = App {
            tx,
            rx,
            tab: Tab::Board,
            boards: Vec::new(),
            picked: None,
            connected: None,
            err: None,
            cfg: CaptureCfg::default(),
            pushed: None,
            running: false,
            events: Vec::new(),
            status: String::new(),
            marks: Vec::new(),
            marks_hz: 240_000_000,
            wave_on: wave,
            edges: Edges::default(),
            view: View::default(),
            wave_err: None,
            log: Vec::new(),
            raw_entry: String::new(),
            history: Vec::new(),
            hist_pos: None,
            log_errors_only: false,
            runs: Vec::new(),
            resume: None,
            last_resume: std::time::Instant::now(),
            scripts_dir,
            cap_path: String::new(),
            sniffer_exe: std::env::var("SNIFFER_EXE").unwrap_or_else(|_| {
                r"E:\STMicro\sniffer\target\release\sniffer.exe".into()
            }),
        };
        a.rescan();
        if let Some(serial) = connect {
            // Accept a prefix: a full USB serial is 32 characters and nobody
            // types one correctly twice.
            let full = a
                .boards
                .iter()
                .find(|b| b.serial.starts_with(&serial))
                .map(|b| b.serial.clone());
            match full {
                Some(s) => {
                    a.picked = Some(s.clone());
                    a.send(Req::Connect(s));
                }
                None => a.err = Some(format!("no attached board starts with {serial}")),
            }
        }
        a
    }

    fn rescan(&mut self) {
        self.boards = device::list();
        if self.picked.is_none() {
            self.picked = self.boards.first().map(|b| b.serial.clone());
        }
    }

    fn send(&self, r: Req) {
        let _ = self.tx.send(r);
    }

    fn is_rt(&self) -> bool {
        matches!(self.connected, Some((_, b, _)) if b.has_capture_engine())
    }

    fn pump(&mut self) {
        while let Ok(e) = self.rx.try_recv() {
            match e {
                Evt::Connected {
                    serial,
                    board,
                    ident,
                } => {
                    self.connected = Some((serial, board, ident));
                    self.err = None;
                    self.pushed = None;
                    // Jump to Capture on a connect the person asked for, not
                    // on the one that follows a script.
                    //
                    // A bench run ends, the window takes the board back, and
                    // the tab changed under whoever was reading the result -
                    // which is the moment they least want to be moved. resume
                    // is still set here; it is cleared once the connection is
                    // seen.
                    if self.resume.is_none() {
                        self.tab = Tab::Capture;
                    }
                }
                Evt::Disconnected => {
                    self.connected = None;
                    self.running = false;
                }
                Evt::Reply { cmd, reply } => {
                    if reply.starts_with("ERR") {
                        self.err = Some(format!("{cmd} -> {reply}"));
                    }
                    self.log.push((cmd, reply));
                    if self.log.len() > MAX_LOG {
                        self.log.drain(..self.log.len() - MAX_LOG);
                    }
                }
                Evt::Events(mut v) => {
                    self.events.append(&mut v);
                    if self.events.len() > MAX_EVENTS {
                        let n = self.events.len() - MAX_EVENTS;
                        self.events.drain(..n);
                    }
                }
                Evt::RawArmed(hz) => {
                    self.edges.clear();
                    self.edges.tick_hz = hz;
                    self.view = View::default();
                    self.wave_err = None;
                }
                Evt::RawData(reply) => {
                    if let Err(e) = self.edges.push_reply(&reply) {
                        // Reported, not swallowed. A malformed edge record is
                        // a transport fault, and this project has paid once
                        // for a truncated reply parsed into plausible data.
                        self.wave_err = Some(e);
                    }
                }
                Evt::Marks(mut m, hz) => {
                    // Appended, not replaced. Each peek sees only what has not
                    // been drained yet, so a replace would show the last fifty
                    // milliseconds and call it the trace.
                    //
                    // The timestamps are deltas from the ring's t0, which does
                    // not move until the next arm, so marks from different
                    // polls share an origin and can simply be concatenated.
                    self.marks_hz = hz;
                    self.marks.append(&mut m);
                    if self.marks.len() > MAX_EVENTS {
                        let n = self.marks.len() - MAX_EVENTS;
                        self.marks.drain(..n);
                    }
                }
                Evt::Status(s) => self.status = s,
                Evt::Error(e) => self.err = Some(e),
            }
        }
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.pump();
        let was_busy = self.runs.iter().any(|r| !r.done);
        for r in self.runs.iter_mut() {
            r.pump();
        }
        // Take the board back when the last script lets go of it.
        //
        // The window disconnected itself to hand the board over, so leaving it
        // disconnected afterwards is a state nobody chose: every control is
        // dead and the only clue is a chip in the corner. Restoring what was
        // there before is the least surprising thing it can do.
        if was_busy && !self.runs.iter().any(|r| !r.done) {
            // Asked for, not taken. The script's process can still hold the
            // USB interface for a moment after it exits, so the first attempt
            // can be refused - and a `take()` here spent the only chance on
            // it. The request is repeated below until it lands.
            if let Some(serial) = self.resume.clone() {
                self.send(Req::Connect(serial));
            }
        }
        if self.resume.is_some() {
            if self.connected.is_some() {
                self.resume = None;
            } else if !self.runs.iter().any(|r| !r.done) {
                // Half a second between attempts, which is long enough for the
                // handle to be released and short enough not to be noticed.
                let now = std::time::Instant::now();
                if now.duration_since(self.last_resume)
                    > std::time::Duration::from_millis(500)
                {
                    self.last_resume = now;
                    if let Some(serial) = self.resume.clone() {
                        self.send(Req::Connect(serial));
                    }
                }
                ctx_repaint(ui);
            }
        }
        // A live capture and a running script both change the window without
        // any input, so the frame has to be asked for rather than waited on.
        if self.running || self.runs.iter().any(|r| !r.done) {
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(60));
        } else if self.connected.is_some() {
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(400));
        }

        style::install(ui.ctx());
        egui::Panel::top("top").show(ui, |ui| self.top_bar(ui));
        if let Some(e) = self.err.clone() {
            egui::Panel::top("err").show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label(RichText::new(e).color(Color32::from_rgb(220, 90, 90)));
                    if ui.small_button("dismiss").clicked() {
                        self.err = None;
                    }
                });
            });
        }
        egui::Panel::bottom("console")
            .resizable(true)
            .default_size(170.0)
            .min_size(70.0)
            .show(ui, |ui| self.console(ui));
        egui::CentralPanel::default().show(ui, |ui| match self.tab {
            Tab::Board => self.tab_board(ui),
            Tab::Capture => self.tab_capture(ui),
            Tab::Trace => self.tab_trace(ui),
            Tab::Files => self.tab_files(ui),
            Tab::Bench => self.tab_bench(ui),
        });
    }
}

impl App {
    fn top_bar(&mut self, ui: &mut egui::Ui) {
        ui.add_space(3.0);
        ui.horizontal(|ui| {
            ui.label(RichText::new("BUSBRIDGE").color(Palette::ACCENT).strong().size(16.0));
            ui.label(RichText::new("RT").color(Palette::DIM).size(16.0));
            ui.add_space(8.0);

            // Tabs first and at fixed positions. They are the only thing in
            // this bar whose place must not depend on how long the board name
            // happens to be.
            for (t, name) in [
                (Tab::Board, "Board"),
                (Tab::Capture, "Capture"),
                (Tab::Trace, "Source & trace"),
                (Tab::Files, "Files"),
                (Tab::Bench, "Bench"),
            ] {
                let on = self.tab == t;
                let text = if on {
                    RichText::new(name).color(Color32::BLACK).strong()
                } else {
                    RichText::new(name).color(Palette::DIM)
                };
                let fill = if on { Palette::ACCENT } else { Color32::TRANSPARENT };
                if ui.add(egui::Button::new(text).fill(fill)).clicked() {
                    self.tab = t;
                }
            }

            // Connection state at the right-hand end, where it can change
            // width without moving anything.
            style::right(ui, |ui| {
                match self.connected.clone() {
                    Some((serial, board, _)) => {
                        if ui.button("Disconnect").clicked() {
                            self.send(Req::Disconnect);
                        }
                        let c = if board.has_capture_engine() {
                            Palette::GOOD
                        } else {
                            Palette::WARN
                        };
                        style::chip(ui, board.label(), &serial[..8.min(serial.len())], c);
                    }
                    None => style::chip(ui, "board", "not connected", Palette::FAINT),
                }
            });
        });
        ui.add_space(3.0);
    }

    fn tab_board(&mut self, ui: &mut egui::Ui) {
        style::card(ui, "attached", |ui| {
            ui.horizontal(|ui| {
                if ui.button("Rescan").clicked() {
                    self.rescan();
                }
                style::right(ui, |ui| {
                    style::chip(ui, "found", &self.boards.len().to_string(), Palette::DATA);
                });
            });
            ui.add_space(6.0);

            if self.boards.is_empty() {
                style::note(ui, Palette::WARN, "No board is attached over USB.");
                return;
            }
            let boards: Vec<(String, String)> = self
                .boards
                .iter()
                .map(|b| (b.serial.clone(), b.port.clone()))
                .collect();
            let here = self.connected.as_ref().map(|(s, _, _)| s.clone());
            for (serial, port) in boards {
                let live = here.as_deref() == Some(serial.as_str());
                ui.horizontal(|ui| {
                    let dot = if live { Palette::GOOD } else { Palette::FAINT };
                    let (r, _) =
                        ui.allocate_exact_size(egui::vec2(10.0, 10.0), egui::Sense::hover());
                    ui.painter().circle_filled(r.center(), 4.0, dot);
                    ui.label(
                        RichText::new(&serial[..14.min(serial.len())])
                            .monospace()
                            .color(if live { Palette::TEXT } else { Palette::DIM }),
                    );
                    ui.label(RichText::new(port).color(Palette::FAINT).small());
                    style::right(ui, |ui| {
                        if live {
                            if ui.button("Disconnect").clicked() {
                                self.send(Req::Disconnect);
                            }
                            style::chip(ui, "", "connected", Palette::GOOD);
                        } else if ui.button("Connect").clicked() {
                            self.picked = Some(serial.clone());
                            self.send(Req::Connect(serial.clone()));
                        }
                    });
                });
            }
        });

        ui.add_space(8.0);

        if let Some((_, board, ident)) = self.connected.clone() {
            style::card(ui, board.label(), |ui| {
                ui.label(RichText::new(&ident).monospace().color(Palette::DIM).small());
                if !board.has_capture_engine() {
                    ui.add_space(6.0);
                    style::note(ui, Palette::WARN,
                        "No CM7 capture engine on this board. Capture, trace and files are RT1186-only and stay disabled; the console works as usual.");
                }
            });
        } else {
            style::card(ui, "", |ui| {
                style::note(ui, Palette::DIM,
                    "A board's USB interface belongs to one process at a time. If one will not open, the bench console or a script probably has it.");
            });
        }
    }

    fn tab_capture(&mut self, ui: &mut egui::Ui) {
        let rt = self.is_rt();
        if !rt {
            style::card(ui, "", |ui| {
                style::note(ui, Palette::WARN,
                    "This board has no CM7 capture engine. The controls below are the RT1186's; the console still works.");
            });
            ui.add_space(8.0);
        }

        ui.add_enabled_ui(rt, |ui| {
            style::card(ui, "capture", |ui| {
                style::row(ui, "Protocol", |ui| {
                    let mut p = self.cfg.proto.clone();
                    if style::segmented(ui, &mut p, &[
                        ("i2c", "Two wires, ack on the ninth bit."),
                        ("i3c", "Two wires, T-bit on the ninth - parity on a write, end-of-data on a read."),
                        ("spi", "Four wires."),
                        ("mdio", "Clause 22 and clause 45."),
                        ("uart", "Asynchronous, no clock to tap."),
                    ]) {
                        // The default tap pair differs per protocol, and a stale
                        // pair is worse than none: tap 0 must be the clock for a
                        // pin-clocked capture, and the wrong pair decodes to
                        // nothing with no error.
                        self.cfg.taps = if p == "i3c" { "d8 d9".into() } else { "d4 d5".into() };
                        self.cfg.proto = p;
                    }
                });
                style::row(ui, "Taps", |ui| {
                    ui.add(egui::TextEdit::singleline(&mut self.cfg.taps)
                        .desired_width(130.0)
                        .font(egui::TextStyle::Monospace))
                        .on_hover_text("Tap 0 is the clock. SNIFF:TAPS? lists what this board has spare.");
                });
                style::row(ui, "Mode", |ui| {
                    let mut m = self.cfg.mode.clone();
                    if style::segmented(ui, &mut m, &[
                        ("pinclock", "Shifted by the bus's own clock: one sample per bit, exact to 12.5 MHz. No framing, and the last frame of a window loses its tail."),
                        ("oversample", "Free-running sample clock: sees framing, idle and glitches, but needs about four samples a bit so it follows a bus only to ~3 MHz."),
                    ]) {
                        self.cfg.mode = m;
                    }
                });
                style::row(ui, "Bus rate", |ui| {
                    let mut mhz = self.cfg.rate_hz as f64 / 1e6;
                    if ui.add(egui::DragValue::new(&mut mhz)
                            .speed(0.05).range(0.01..=15.0).max_decimals(3).suffix(" MHz"))
                        .changed()
                    {
                        self.cfg.rate_hz = (mhz * 1e6) as u32;
                    }
                    ui.add_space(4.0);
                    for r in [1_000_000u32, 4_000_000, 8_000_000, 12_500_000] {
                        let on = self.cfg.rate_hz == r;
                        let label = if r == 12_500_000 { "12.5".to_string() }
                                    else { format!("{}", r / 1_000_000) };
                        let t = if on { RichText::new(label).color(Color32::BLACK).strong() }
                                else { RichText::new(label).color(Palette::DIM) };
                        let fill = if on { Palette::ACCENT } else { Color32::TRANSPARENT };
                        if ui.add(egui::Button::new(t).fill(fill)).clicked() {
                            self.cfg.rate_hz = r;
                        }
                    }
                });
                style::row(ui, "Intake", |ui| {
                    ui.checkbox(&mut self.cfg.dma, "eDMA")
                        .on_hover_text("Takes the pin-clocked intake off the CPU. Needed at 12.5 MHz; harmless below.");
                    ui.checkbox(&mut self.cfg.decode, "decode")
                        .on_hover_text("Off captures samples only - the measurement that says what capture costs without decode.");
                });
            });

            ui.add_space(8.0);

            style::card(ui, "run", |ui| {
                ui.horizontal(|ui| {
                    let dirty = self.pushed.as_ref() != Some(&self.cfg);
                    let start = egui::Button::new(
                        RichText::new("Start").color(Color32::BLACK).strong()
                    ).fill(Palette::GOOD);
                    if ui.add_enabled(!self.running, start).clicked() {
                        if dirty {
                            self.send(Req::Configure(self.cfg.clone()));
                            self.pushed = Some(self.cfg.clone());
                        }
                        self.events.clear();
                        self.marks.clear();
                        self.send(Req::Clear);
                        if self.wave_on {
                            // SNIFF:RAW:START restarts the capture core itself,
                            // so it stands in for SNIFF:START rather than
                            // following it. Sending both would start the capture
                            // twice and throw away the first.
                            self.edges.clear();
                            self.send(Req::RawStart(200_000));
                        } else {
                            self.send(Req::Start);
                        }
                        self.running = true;
                    }
                    if ui.add_enabled(self.running, egui::Button::new("Stop")).clicked() {
                        self.send(Req::Stop);
                        self.running = false;
                    }
                    if ui.button("Clear").clicked() {
                        self.events.clear();
                        self.marks.clear();
                        self.edges.clear();
                        self.send(Req::Clear);
                    }
                    if ui.button("Status").clicked() {
                        self.send(Req::Status);
                    }
                    ui.separator();
                    if ui.add_enabled(dirty, egui::Button::new("Apply"))
                        .on_hover_text("Push the configuration without starting. Start applies it anyway.")
                        .clicked()
                    {
                        self.send(Req::Configure(self.cfg.clone()));
                        self.pushed = Some(self.cfg.clone());
                    }

                    // Pin-clocked capture skips the raw sample store while it
                    // decodes - that is what bought 6 to 8 MHz - so asking for
                    // both gets an empty trace rather than an error. Refusing the
                    // combination is better than drawing nothing and letting it
                    // look like a dead bus.
                    let blocked = self.cfg.mode == "pinclock" && self.cfg.decode;
                    let mut want = self.wave_on;
                    let cb = ui.add_enabled(!blocked, egui::Checkbox::new(&mut want, "Waveform"));
                    if blocked {
                        cb.on_hover_text("Pin-clocked capture does not store raw samples while it decodes. Turn decode off for a waveform, or use oversampled mode to have both.");
                    } else if want != self.wave_on {
                        self.wave_on = want;
                        if !want {
                            self.send(Req::RawStop);
                        } else if self.running {
                            self.send(Req::RawStart(200_000));
                        }
                    }

                    // Everything that changes width lives at the right-hand end.
                    // A readout that grows must never move a button.
                    style::right(ui, |ui| {
                        if self.running {
                            style::chip(ui, "state", "running", Palette::GOOD);
                        } else {
                            style::chip(ui, "state", "stopped", Palette::FAINT);
                        }
                        if dirty {
                            style::chip(ui, "config", "not pushed", Palette::WARN);
                        }
                    });
                });
            });
        });

        ui.add_space(8.0);

        if self.wave_on {
            style::card(ui, "waveform", |ui| {
                let names: Vec<String> = self.cfg.taps.split_whitespace().enumerate()
                    // Tap 0 is the clock by the firmware's own rule, and the
                    // label says so: a two-line trace with no clock named is a
                    // guess the reader should not have to make.
                    .map(|(i, t)| if i == 0 { format!("{t} clk") } else { t.to_string() })
                    .collect();

                ui.horizontal(|ui| {
                    ui.checkbox(&mut self.view.follow, "follow")
                        .on_hover_text("Keep the right edge pinned to the newest sample.");
                    if ui.button("fit")
                        .on_hover_text("Fit the most recent burst. An idle bus is most of what an oversampled window holds, and the thing you want is usually what just happened.")
                        .clicked()
                    {
                        let (a, b) = self.edges.last_burst();
                        if b > a {
                            self.view.left = a as f64;
                            self.view.width = (b - a) as f64;
                            self.view.follow = false;
                        }
                    }
                    if ui.button("all").on_hover_text("The whole captured span.").clicked() {
                        let (a, b) = self.edges.span_ticks();
                        if b > a {
                            self.view.left = a as f64;
                            self.view.width = (b - a) as f64;
                            self.view.follow = false;
                        }
                    }

                    style::right(ui, |ui| {
                        let hz = self.edges.tick_hz.max(1) as f64;
                        let (f0, f1) = self.edges.span_ticks();
                        let (a0, a1) = self.edges.last_burst();
                        if let Some(e) = self.wave_err.clone() {
                            style::chip(ui, "error", &e, Palette::BAD);
                        }
                        style::chip(ui, "burst", &secs((a1 - a0) as f64 / hz), Palette::TEXT);
                        style::chip(ui, "captured", &secs((f1 - f0) as f64 / hz), Palette::DIM);
                        style::chip(ui, "edges", &self.edges.list.len().to_string(), Palette::DATA);
                    });
                });
                ui.add_space(4.0);
                wave::draw(ui, &self.edges, &mut self.view, &names,
                           self.cfg.mode == "pinclock");
            });
            ui.add_space(8.0);
        }

        style::card(ui, "events", |ui| {
            ui.horizontal(|ui| {
                if self.cfg.proto == "i3c" {
                    style::note(ui, Palette::DIM,
                        "I3C: the ninth bit is a T-bit driven by the transmitter, not an acknowledge.");
                }
                style::right(ui, |ui| {
                    style::chip(ui, "events", &self.events.len().to_string(), Palette::DATA);
                });
            });
            ui.add_space(4.0);
            events_view(ui, &self.events);
            if !self.status.is_empty() {
                ui.add_space(4.0);
                ui.label(RichText::new(&self.status).monospace().color(Palette::FAINT).small());
            }
        });
    }

    fn tab_trace(&mut self, ui: &mut egui::Ui) {
        let rt = self.is_rt();
        style::card(ui, "source", |ui| {
            style::note(ui, Palette::DIM,
                "Where a record came from, kept apart from what it says. `wire` is a measurement; `controller` is what our own peripheral accepted. They differ exactly when something is wrong.");
            ui.add_space(8.0);
            ui.add_enabled_ui(rt, |ui| {
                ui.horizontal(|ui| {
                    let mut src = self.cfg.source.clone();
                    if style::segmented(ui, &mut src, &[
                        ("wire", "FlexIO samples the tap pads. Sees everything on the bus, including other controllers. Needs the taps wired."),
                        ("self", "The controller's own transfers, recorded as it drives them. No taps at all - it works while the bus is in use because it IS the bus in use."),
                        ("both", "Both engines over one window, each keeping its own tag. Frames can be ordered against each other to about 1.5 us; events inside a frame cannot."),
                    ]) {
                        self.cfg.source = src;
                    }
                    ui.add_space(6.0);
                    if ui.button("Read trace").clicked() {
                        self.send(Req::ReadTrace);
                    }
                    style::right(ui, |ui| {
                        style::chip(ui, "marks", &self.marks.len().to_string(), Palette::DATA);
                    });
                });
            });
            if self.cfg.source != "wire" {
                ui.add_space(8.0);
                style::note(ui, Palette::WARN,
                    "A controller-side T-bit is computed from the byte it belongs to, so it agrees by construction and can never fail. Marks below say which flags were computed.");
            }
        });

        ui.add_space(8.0);

        style::card(ui, "controller trace", |ui| {
            if self.marks.is_empty() {
                style::note(ui, Palette::DIM,
                    "Nothing yet. Set the source to `self` or `both`, start a capture, and drive a transfer - marks arrive while it runs. `Read trace` asks for them once.");
                return;
            }
            let t0 = self.marks.first().map(|m| m.dt_cycles).unwrap_or(0);
            let hz = self.marks_hz.max(1) as f64;
            egui::ScrollArea::vertical().show(ui, |ui| {
                egui::Grid::new("marks")
                    .num_columns(3)
                    .striped(true)
                    .spacing([18.0, 3.0])
                    .show(ui, |ui| {
                        ui.label(RichText::new("TIME").color(Palette::FAINT).small().strong());
                        ui.label(RichText::new("TOKEN").color(Palette::FAINT).small().strong());
                        ui.label(RichText::new("NINTH BIT").color(Palette::FAINT).small().strong());
                        ui.end_row();
                        for m in &self.marks {
                            let us = (m.dt_cycles.saturating_sub(t0)) as f64 / hz * 1e6;
                            ui.label(
                                RichText::new(format!("{us:9.3} us"))
                                    .monospace()
                                    .color(Palette::DIM),
                            );
                            ui.label(RichText::new(&m.token).monospace().color(token_colour(&m.token)));
                            if m.derived {
                                ui.label(RichText::new("computed").color(Palette::WARN).small());
                            } else {
                                ui.label(RichText::new("observed").color(Palette::GOOD).small());
                            }
                            ui.end_row();
                        }
                    });
            });
        });
    }

    fn tab_files(&mut self, ui: &mut egui::Ui) {
        let busy = self.runs.iter().any(|r| !r.done);
        style::card(ui, "record", |ui| {
            style::note(ui, Palette::DIM,
                "Recording and decoding stay in the scripts and the analyser rather than being reimplemented here. A window that disagreed with the terminal would be worse than no window.");
            ui.add_space(8.0);
            ui.add_enabled_ui(self.is_rt() && !busy, |ui| {
                ui.horizontal(|ui| {
                    if ui.button("Oversampled")
                        .on_hover_text("scripts/capture.py - measured timing, exact below ~3 MHz")
                        .clicked()
                    {
                        self.start_script("capture.py", &[]);
                    }
                    if ui.button("Pin-clocked")
                        .on_hover_text("scripts/pinclock_cap.py - reconstructs a file for rates too fast to oversample. Its timing is INVENTED and the file says so.")
                        .clicked()
                    {
                        self.start_script("pinclock_cap.py", &[]);
                    }
                });
            });
        });

        ui.add_space(8.0);

        style::card(ui, "open", |ui| {
            ui.horizontal(|ui| {
                ui.add(
                    egui::TextEdit::singleline(&mut self.cap_path)
                        .desired_width(400.0)
                        .font(egui::TextStyle::Monospace)
                        .hint_text("path to a .cap file"),
                );
                if ui.button("Show in analyser").clicked() {
                    let p = self.cap_path.clone();
                    let exe = self.sniffer_exe.clone();
                    if p.is_empty() {
                        self.err = Some("no file named".into());
                    } else if !std::path::Path::new(&exe).exists() {
                        self.err = Some(format!(
                            "analyser not found at {exe} - build it in the sniffer tree, or set SNIFFER_EXE"
                        ));
                    } else {
                        let _ = std::process::Command::new(&exe).arg("show").arg(&p).spawn();
                    }
                }
            });
            ui.add_space(6.0);
            style::note(ui, Palette::WARN,
                "A file built from a pin-clocked capture carries synthetic timing and warns on open. That mode records no transition times, so the timing in such a file is reconstructed, not measured.");
        });

        ui.add_space(8.0);
        self.runs_view(ui);
    }

    fn tab_bench(&mut self, ui: &mut egui::Ui) {
        let busy = self.runs.iter().any(|r| !r.done);
        style::card(ui, "measurements", |ui| {
            style::note(ui, Palette::DIM,
                "These hold the board for as long as they run, so this window disconnects first. Each is the same script the documentation cites.");
            ui.add_space(8.0);
            ui.add_enabled_ui(!busy, |ui| {
                for (script, args, what) in [
                    ("acceptance.py", vec![], "The whole suite: 66 checks, both boards attached."),
                    ("self_trace.py", vec![], "Does the controller trace say what the wire says, and admit what it cannot see?"),
                    ("clock_offset.py", vec!["8".to_string(), "12".into(), "2000".into()],
                        "Where the capture core's tick 0 sits in CM33 time, and whether the clocks drift."),
                    ("trace_lag.py", vec!["4".to_string()],
                        "How far behind the wire the controller stamps its own events."),
                    ("merge_both.py", vec!["3".to_string()],
                        "A `both` capture merged into one ordered stream, with its error bar."),
                ] {
                    ui.horizontal(|ui| {
                        let (r, _) = ui.allocate_exact_size(
                            // Wide enough for the longest script name. 132 wrapped
                            // clock_offset.py onto two lines and made that one row
                            // taller than the rest, which reads as emphasis.
                            egui::vec2(152.0, 24.0), egui::Sense::hover());
                        let btn = ui.put(r, egui::Button::new(
                            RichText::new(script).monospace()));
                        if btn.clicked() {
                            self.start_script(script, &args);
                        }
                        ui.label(RichText::new(what).color(Palette::DIM).small());
                    });
                }
            });
        });

        ui.add_space(8.0);
        self.runs_view(ui);
    }

    fn start_script(&mut self, script: &str, args: &[String]) {
        // The script needs the board, and this window has it. Hand it over
        // rather than letting the script fail to open it.
        if let Some((serial, _, _)) = self.connected.clone() {
            self.resume = Some(serial);
            self.send(Req::Disconnect);
        }
        self.runs.push(Run::python(&self.scripts_dir, script, args));
        // Keep the last few runs only; the output of a dozen acceptance runs
        // is megabytes of text nobody scrolls back to.
        if self.runs.len() > 4 {
            self.runs.remove(0);
        }
    }

    fn runs_view(&mut self, ui: &mut egui::Ui) {
        if self.runs.is_empty() {
            return;
        }
        let mut stop_idx = None;
        for (i, r) in self.runs.iter().enumerate().rev() {
            style::card(ui, "", |ui| {
                ui.horizontal(|ui| {
                    ui.label(RichText::new(&r.title).monospace().strong());
                    if !r.done && ui.small_button("stop").clicked() {
                        stop_idx = Some(i);
                    }
                    style::right(ui, |ui| {
                        let (c, v) = match r.verdict() {
                            "passed" => (Palette::GOOD, "passed"),
                            "FAILED" => (Palette::BAD, "FAILED"),
                            "stopped" => (Palette::WARN, "stopped"),
                            _ => (Palette::ACCENT, "running"),
                        };
                        style::chip(ui, "", v, c);
                        style::chip(ui, "lines", &r.lines.len().to_string(), Palette::DIM);
                    });
                });
                ui.add_space(4.0);
                egui::Frame::new()
                    .fill(Palette::PLOT_BG)
                    .corner_radius(egui::CornerRadius::same(5))
                    .inner_margin(egui::Margin::same(7))
                    .show(ui, |ui| {
                        ui.set_width(ui.available_width());
                        egui::ScrollArea::vertical()
                            .id_salt(format!("run{i}"))
                            .max_height(250.0)
                            .stick_to_bottom(true)
                            .show(ui, |ui| {
                                for l in &r.lines {
                                    let c = if l.starts_with('!') || l.contains("FAIL") {
                                        Palette::BAD
                                    } else if l.contains("PASS") {
                                        Palette::GOOD
                                    } else if l.starts_with("==") {
                                        Palette::ACCENT
                                    } else {
                                        Palette::TEXT
                                    };
                                    ui.label(RichText::new(l).monospace().color(c));
                                }
                            });
                    });
            });
            ui.add_space(8.0);
        }
        if let Some(i) = stop_idx {
            self.runs[i].stop();
        }
    }

    fn console(&mut self, ui: &mut egui::Ui) {
        ui.add_space(5.0);
        ui.horizontal(|ui| {
            ui.label(
                RichText::new("\u{203a}")
                    .color(Palette::ACCENT)
                    .monospace()
                    .strong()
                    .size(15.0),
            );
            let r = ui.add(
                egui::TextEdit::singleline(&mut self.raw_entry)
                    // Room for Send, the errors toggle, Clear and the two
                    // counters. 250 was sized before the counters moved onto
                    // this row and clipped Clear to a single letter.
                    .desired_width((ui.available_width() - 380.0).max(160.0))
                    .font(egui::TextStyle::Monospace)
                    // Plain words, not an arrow glyph: the default font has no
                    // U+2191 and drew it as a replacement box, which reads
                    // as a missing character rather than as a hint.
                    .hint_text("SNIFF:STAT?    Up / Down for history"),
            );

            // History on the arrows, and only while the field has focus - the
            // same keys scroll the transcript otherwise.
            //
            // The keys are consumed explicitly. A singleline TextEdit ignores
            // up and down, but it does not swallow them, so without this the
            // scroll area underneath moved at the same time and the window
            // appeared to jump every time someone walked back a command.
            if r.has_focus() && !self.history.is_empty() {
                let (up, down) = ui.input_mut(|i| {
                    (
                        i.consume_key(egui::Modifiers::NONE, egui::Key::ArrowUp),
                        i.consume_key(egui::Modifiers::NONE, egui::Key::ArrowDown),
                    )
                });
                if up {
                    let next = match self.hist_pos {
                        None => self.history.len() - 1,
                        Some(0) => 0,
                        Some(n) => n - 1,
                    };
                    self.hist_pos = Some(next);
                    self.raw_entry = self.history[next].clone();
                }
                if down {
                    match self.hist_pos {
                        Some(n) if n + 1 < self.history.len() => {
                            self.hist_pos = Some(n + 1);
                            self.raw_entry = self.history[n + 1].clone();
                        }
                        Some(_) => {
                            // Past the newest entry is the empty line being
                            // typed, not the oldest one wrapped around.
                            self.hist_pos = None;
                            self.raw_entry.clear();
                        }
                        None => {}
                    }
                }
            }

            let send = (r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)))
                || ui.button("Send").clicked();
            if send {
                let line = self.raw_entry.trim().to_string();
                if !line.is_empty() {
                    // Not recorded twice in a row. Pressing enter on the same
                    // command four times should leave one entry to walk back
                    // to, not four.
                    if self.history.last().map(String::as_str) != Some(line.as_str()) {
                        self.history.push(line.clone());
                    }
                    self.hist_pos = None;
                    self.send(Req::Raw(line));
                    self.raw_entry.clear();
                }
                r.request_focus();
            }

            ui.checkbox(&mut self.log_errors_only, "errors")
                .on_hover_text("Show only the commands that were refused.");
            if ui.button("Clear").clicked() {
                self.log.clear();
            }
            // Counts on this row, not under the transcript. Anything placed
            // after a scroll area that does not shrink makes the panel ask for
            // more height than it has, and a resizable bottom panel answers
            // that by growing until it has eaten the window - which is exactly
            // what it did.
            let errs = self.log.iter().filter(|(_, r)| r.starts_with("ERR")).count();
            style::right(ui, |ui| {
                if errs > 0 {
                    style::chip(ui, "errors", &errs.to_string(), Palette::BAD);
                }
                style::chip(ui, "lines", &self.log.len().to_string(), Palette::DIM);
            });
        });
        ui.add_space(5.0);

        // Whatever the panel has left, and no more.
        let room = (ui.available_height() - 8.0).max(32.0);
        egui::Frame::new()
            .fill(Palette::PLOT_BG)
            .corner_radius(egui::CornerRadius::same(5))
            .inner_margin(egui::Margin::same(8))
            .show(ui, |ui| {
                ui.set_width(ui.available_width());
                egui::ScrollArea::vertical()
                    .id_salt("log")
                    .stick_to_bottom(true)
                    .max_height(room - 18.0)
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                        if self.log.is_empty() {
                            ui.label(
                                RichText::new("nothing sent yet")
                                    .color(Palette::FAINT)
                                    .small(),
                            );
                        }
                        for (cmd, reply) in &self.log {
                            let bad = reply.starts_with("ERR");
                            if self.log_errors_only && !bad {
                                continue;
                            }
                            ui.horizontal_wrapped(|ui| {
                                ui.spacing_mut().item_spacing.x = 6.0;
                                ui.label(
                                    RichText::new("\u{203a}")
                                        .color(Palette::FAINT)
                                        .monospace(),
                                );
                                ui.label(
                                    RichText::new(cmd).monospace().color(Palette::DIM),
                                );
                            });
                            // The reply indented under its command, so a long
                            // one does not read as the next command.
                            ui.horizontal_wrapped(|ui| {
                                ui.add_space(14.0);
                                ui.label(RichText::new(reply).monospace().color(
                                    if bad { Palette::BAD } else { Palette::TEXT },
                                ));
                            });
                            ui.add_space(2.0);
                        }
                    });
            });
        ui.add_space(3.0);
    }
}

/// Ask for another frame soon, for the cases that are waiting on a timer
/// rather than on the worker.
fn ctx_repaint(ui: &egui::Ui) {
    ui.ctx()
        .request_repaint_after(std::time::Duration::from_millis(200));
}

/// What colour a token is, in one place.
///
/// The event stream and the trace table show the same vocabulary and must
/// colour it the same way: a `D40+` that is green in one view and plain in the
/// other reads as two different things.
fn token_colour(t: &str) -> Color32 {
    match t {
        "S" | "Sr" => Palette::CLOCK,
        "P" => Palette::FAINT,
        "!" => Palette::BAD,
        _ if t.starts_with('A') => Palette::ADDR,
        _ if t.starts_with('C') => Palette::CCC,
        _ => Palette::DATA,
    }
}

/// A duration in seconds, printed at the scale it deserves.
///
/// One formatter rather than a hardcoded `ms` everywhere: a capture window
/// here runs from tens of nanoseconds to minutes, and "0.00008 ms" is a number
/// nobody reads correctly at a glance.
fn secs(t: f64) -> String {
    if t < 1e-6 {
        format!("{:.0} ns", t * 1e9)
    } else if t < 1e-3 {
        format!("{:.2} us", t * 1e6)
    } else if t < 1.0 {
        format!("{:.2} ms", t * 1e3)
    } else {
        format!("{t:.2} s")
    }
}

/// The decoded event stream, wrapped and coloured by what each token is.
///
/// Framing apart from data on purpose: a wall of `D40+ D11+ D22+` with the
/// STARTs the same colour reads as one long transfer, and the thing a person
/// is usually looking for is where the frames begin.
fn events_view(ui: &mut egui::Ui, events: &[String]) {
    egui::Frame::new()
        .fill(Palette::PLOT_BG)
        .corner_radius(egui::CornerRadius::same(5))
        .inner_margin(egui::Margin::same(8))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.set_min_height(54.0);
            egui::ScrollArea::vertical()
                .id_salt("events")
                .stick_to_bottom(true)
                .auto_shrink([false, false])
                .show(ui, |ui| {
                    ui.horizontal_wrapped(|ui| {
                        ui.spacing_mut().item_spacing.x = 5.0;
                        for t in events {
                            ui.label(
                                RichText::new(t).monospace().color(token_colour(t)),
                            );
                        }
                    });
                });
        });
}
