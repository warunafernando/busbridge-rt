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

    log: Vec<(String, String)>,
    raw_entry: String,

    runs: Vec<Run>,
    scripts_dir: String,
    cap_path: String,
    sniffer_exe: String,
}

impl App {
    pub fn new(scripts_dir: String, connect: Option<String>) -> App {
        let (tx, rx) = worker::spawn();
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
            log: Vec::new(),
            raw_entry: String::new(),
            runs: Vec::new(),
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
                    self.tab = Tab::Capture;
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
                Evt::Marks(m, hz) => {
                    self.marks = m;
                    self.marks_hz = hz;
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
        for r in self.runs.iter_mut() {
            r.pump();
        }
        // A live capture and a running script both change the window without
        // any input, so the frame has to be asked for rather than waited on.
        if self.running || self.runs.iter().any(|r| !r.done) {
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(60));
        } else if self.connected.is_some() {
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(400));
        }

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
        ui.horizontal(|ui| {
            ui.heading("BUSBRIDGE-RT");
            ui.separator();
            match &self.connected {
                Some((serial, board, _)) => {
                    let c = if board.has_capture_engine() {
                        Color32::from_rgb(120, 200, 120)
                    } else {
                        Color32::from_rgb(220, 180, 90)
                    };
                    ui.label(RichText::new(board.label()).color(c));
                    ui.label(RichText::new(&serial[..8.min(serial.len())]).weak());
                    if ui.button("Disconnect").clicked() {
                        self.send(Req::Disconnect);
                    }
                }
                None => {
                    ui.label(RichText::new("not connected").weak());
                }
            }
            ui.separator();
            for (t, name) in [
                (Tab::Board, "Board"),
                (Tab::Capture, "Capture"),
                (Tab::Trace, "Source & trace"),
                (Tab::Files, "Files"),
                (Tab::Bench, "Bench"),
            ] {
                if ui.selectable_label(self.tab == t, name).clicked() {
                    self.tab = t;
                }
            }
        });
    }

    fn tab_board(&mut self, ui: &mut egui::Ui) {
        ui.heading("Boards");
        ui.label(
            RichText::new(
                "A board's USB interface belongs to one process at a time. If one \
                 will not open, the bench console or a script probably has it.",
            )
            .weak(),
        );
        ui.add_space(6.0);
        if ui.button("Rescan").clicked() {
            self.rescan();
        }
        ui.add_space(6.0);

        if self.boards.is_empty() {
            ui.label("none attached.");
            return;
        }
        let boards: Vec<(String, String)> = self
            .boards
            .iter()
            .map(|b| (b.serial.clone(), b.port.clone()))
            .collect();
        for (serial, port) in boards {
            ui.horizontal(|ui| {
                let sel = self.picked.as_deref() == Some(serial.as_str());
                if ui
                    .selectable_label(sel, format!("{}  ({port})", &serial[..12.min(serial.len())]))
                    .clicked()
                {
                    self.picked = Some(serial.clone());
                }
                if ui.button("Connect").clicked() {
                    self.send(Req::Connect(serial.clone()));
                }
            });
        }

        if let Some((_, board, ident)) = &self.connected {
            ui.add_space(10.0);
            ui.separator();
            ui.label(RichText::new(board.label()).strong());
            ui.label(RichText::new(ident).monospace().weak());
            if !board.has_capture_engine() {
                ui.add_space(4.0);
                ui.label(
                    RichText::new(
                        "No CM7 capture engine on this board. The capture, trace and \
                         file tabs are RT1186-only and will stay disabled; everything \
                         in the console works as usual.",
                    )
                    .color(Color32::from_rgb(220, 180, 90)),
                );
            }
        }
    }

    fn tab_capture(&mut self, ui: &mut egui::Ui) {
        let rt = self.is_rt();
        ui.heading("Capture");
        if !rt {
            ui.label(
                RichText::new("Needs the RT1186's capture engine.")
                    .color(Color32::from_rgb(220, 180, 90)),
            );
        }
        ui.add_space(4.0);

        ui.add_enabled_ui(rt, |ui| {
            egui::Grid::new("cfg").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| {
                ui.label("Protocol");
                ui.horizontal(|ui| {
                    for p in ["i2c", "i3c", "spi", "mdio", "uart"] {
                        if ui.selectable_label(self.cfg.proto == p, p).clicked() {
                            self.cfg.proto = p.into();
                            // The default tap pair differs per protocol, and a
                            // stale pair is worse than none: tap 0 must be the
                            // clock for a pin-clocked capture, and the wrong
                            // pair decodes to nothing with no error.
                            self.cfg.taps = match p {
                                "i3c" => "d8 d9".into(),
                                _ => "d4 d5".into(),
                            };
                        }
                    }
                });
                ui.end_row();

                ui.label("Taps");
                ui.add(egui::TextEdit::singleline(&mut self.cfg.taps).desired_width(140.0))
                    .on_hover_text("Tap 0 is the clock. SNIFF:TAPS? lists what this board has spare.");
                ui.end_row();

                ui.label("Mode");
                ui.horizontal(|ui| {
                    for (m, hint) in [
                        ("pinclock", "Shifted by the bus's own clock: one sample per bit, exact to 12.5 MHz. No framing, and the last frame of a window loses its tail."),
                        ("oversample", "Free-running sample clock: sees framing, idle and glitches, but needs about four samples a bit so it follows a bus only to ~3 MHz."),
                    ] {
                        if ui.selectable_label(self.cfg.mode == m, m).on_hover_text(hint).clicked() {
                            self.cfg.mode = m.into();
                        }
                    }
                });
                ui.end_row();

                ui.label("Bus rate");
                ui.horizontal(|ui| {
                    let mut hz = self.cfg.rate_hz as f64 / 1e6;
                    if ui.add(egui::DragValue::new(&mut hz).speed(0.1).range(0.01..=15.0).suffix(" MHz")).changed() {
                        self.cfg.rate_hz = (hz * 1e6) as u32;
                    }
                    for r in [1_000_000u32, 4_000_000, 8_000_000, 12_500_000] {
                        if ui.small_button(format!("{}", r / 1_000_000)).clicked() {
                            self.cfg.rate_hz = r;
                        }
                    }
                });
                ui.end_row();

                ui.label("Intake");
                ui.horizontal(|ui| {
                    ui.checkbox(&mut self.cfg.dma, "eDMA")
                        .on_hover_text("Takes the pin-clocked intake off the CPU. Needed at 12.5 MHz; harmless below.");
                    ui.checkbox(&mut self.cfg.decode, "decode")
                        .on_hover_text("Off captures samples only - the measurement that says what capture costs without decode.");
                });
                ui.end_row();
            });

            ui.add_space(8.0);
            ui.horizontal(|ui| {
                let dirty = self.pushed.as_ref() != Some(&self.cfg);
                if ui.add_enabled(dirty, egui::Button::new("Apply")).clicked() {
                    self.send(Req::Configure(self.cfg.clone()));
                    self.pushed = Some(self.cfg.clone());
                }
                if dirty {
                    ui.label(RichText::new("not pushed").weak());
                }
                ui.separator();
                if ui.add_enabled(!self.running, egui::Button::new("Start")).clicked() {
                    if self.pushed.as_ref() != Some(&self.cfg) {
                        self.send(Req::Configure(self.cfg.clone()));
                        self.pushed = Some(self.cfg.clone());
                    }
                    self.events.clear();
                    self.send(Req::Clear);
                    self.send(Req::Start);
                    self.running = true;
                }
                if ui.add_enabled(self.running, egui::Button::new("Stop")).clicked() {
                    self.send(Req::Stop);
                    self.running = false;
                }
                if ui.button("Clear").clicked() {
                    self.events.clear();
                    self.send(Req::Clear);
                }
                if ui.button("Status").clicked() {
                    self.send(Req::Status);
                }
            });
        });

        ui.add_space(6.0);
        if !self.status.is_empty() {
            ui.label(RichText::new(&self.status).monospace().weak());
        }
        ui.separator();
        ui.horizontal(|ui| {
            ui.label(RichText::new(format!("{} events", self.events.len())).strong());
            if self.cfg.proto == "i3c" {
                ui.label(
                    RichText::new(
                        "I3C: the ninth bit is a T-bit driven by the transmitter, \
                         not an acknowledge.",
                    )
                    .weak(),
                );
            }
        });
        events_view(ui, &self.events);
    }

    fn tab_trace(&mut self, ui: &mut egui::Ui) {
        let rt = self.is_rt();
        ui.heading("Source & controller trace");
        ui.label(
            RichText::new(
                "Where a record came from, kept apart from what it says. `wire` is \
                 a measurement; `controller` is what our own peripheral accepted. \
                 They differ exactly when something is wrong.",
            )
            .weak(),
        );
        ui.add_space(6.0);

        ui.add_enabled_ui(rt, |ui| {
            ui.horizontal(|ui| {
                for (s, hint) in [
                    ("wire", "FlexIO samples the tap pads. Sees everything on the bus, including other controllers. Needs the taps wired."),
                    ("self", "The controller's own transfers, recorded as it drives them. No taps at all - it works while the bus is in use because it IS the bus in use."),
                    ("both", "Both engines over one window, each keeping its own tag. Frames can be ordered against each other to about 1.5 us; events inside a frame cannot."),
                ] {
                    if ui.selectable_label(self.cfg.source == s, s).on_hover_text(hint).clicked() {
                        self.cfg.source = s.into();
                    }
                }
                ui.separator();
                if ui.button("Read trace").clicked() {
                    self.send(Req::ReadTrace);
                }
            });
        });

        if self.cfg.source != "wire" {
            ui.add_space(4.0);
            ui.label(
                RichText::new(
                    "A controller-side T-bit is computed from the byte it belongs to, \
                     so it agrees by construction and can never fail. Marks below \
                     carry ~ where the flag was computed rather than observed.",
                )
                .color(Color32::from_rgb(220, 180, 90)),
            );
        }

        ui.add_space(6.0);
        ui.separator();
        if self.marks.is_empty() {
            ui.label(RichText::new("no trace read yet.").weak());
            return;
        }
        let t0 = self.marks.first().map(|m| m.dt_cycles).unwrap_or(0);
        let hz = self.marks_hz.max(1) as f64;
        egui::ScrollArea::vertical().show(ui, |ui| {
            egui::Grid::new("marks").num_columns(3).striped(true).show(ui, |ui| {
                ui.label(RichText::new("t (us)").strong());
                ui.label(RichText::new("token").strong());
                ui.label(RichText::new("flag").strong());
                ui.end_row();
                for m in &self.marks {
                    let us = (m.dt_cycles.saturating_sub(t0)) as f64 / hz * 1e6;
                    ui.label(RichText::new(format!("{us:9.3}")).monospace());
                    ui.label(RichText::new(&m.token).monospace());
                    if m.derived {
                        ui.label(
                            RichText::new("computed")
                                .color(Color32::from_rgb(220, 180, 90))
                                .small(),
                        );
                    } else {
                        ui.label(RichText::new("observed").weak().small());
                    }
                    ui.end_row();
                }
            });
        });
    }

    fn tab_files(&mut self, ui: &mut egui::Ui) {
        ui.heading("Capture files");
        ui.label(
            RichText::new(
                "Recording and decoding stay in the scripts and the analyser rather \
                 than being reimplemented here. A window that disagreed with the \
                 terminal would be worse than no window.",
            )
            .weak(),
        );
        ui.add_space(8.0);

        let busy = self.runs.iter().any(|r| !r.done);
        ui.add_enabled_ui(self.is_rt() && !busy, |ui| {
            ui.horizontal(|ui| {
                if ui
                    .button("Record (oversampled)")
                    .on_hover_text("scripts/capture.py - measured timing, exact below ~3 MHz")
                    .clicked()
                {
                    self.start_script("capture.py", &[]);
                }
                if ui
                    .button("Record (pin-clocked)")
                    .on_hover_text(
                        "scripts/pinclock_cap.py - reconstructs a file for rates too \
                         fast to oversample. Its timing is INVENTED and the file says so.",
                    )
                    .clicked()
                {
                    self.start_script("pinclock_cap.py", &[]);
                }
            });
        });

        ui.add_space(10.0);
        ui.label("Open a .cap in the analyser");
        ui.horizontal(|ui| {
            ui.add(egui::TextEdit::singleline(&mut self.cap_path).desired_width(360.0))
                .on_hover_text("Path to a .cap file");
            if ui.button("Show").clicked() {
                let p = self.cap_path.clone();
                let exe = self.sniffer_exe.clone();
                if p.is_empty() {
                    self.err = Some("no file named".into());
                } else if !std::path::Path::new(&exe).exists() {
                    self.err = Some(format!(
                        "analyser not found at {exe} - build it in the sniffer tree, \
                         or set SNIFFER_EXE"
                    ));
                } else {
                    let _ = std::process::Command::new(&exe).arg("show").arg(&p).spawn();
                }
            }
        });
        ui.label(
            RichText::new(
                "A file built from a pin-clocked capture carries synthetic timing and \
                 warns on open. That mode records no transition times, so the timing \
                 in such a file is reconstructed, not measured.",
            )
            .weak()
            .small(),
        );

        ui.add_space(10.0);
        ui.separator();
        self.runs_view(ui);
    }

    fn tab_bench(&mut self, ui: &mut egui::Ui) {
        ui.heading("Bench measurements");
        ui.label(
            RichText::new(
                "These hold the board for as long as they run, so connecting here is \
                 disconnected first. Each is the same script the documentation cites.",
            )
            .weak(),
        );
        ui.add_space(8.0);

        let busy = self.runs.iter().any(|r| !r.done);
        ui.add_enabled_ui(!busy, |ui| {
            egui::Grid::new("bench").num_columns(2).spacing([10.0, 6.0]).show(ui, |ui| {
                for (script, args, what) in [
                    ("acceptance.py", vec![], "The whole suite: 66 checks, both boards attached."),
                    ("self_trace.py", vec![], "Does the controller trace say what the wire says, and admit what it cannot see?"),
                    ("clock_offset.py", vec!["8".to_string(), "12".into(), "2000".into()], "Where the capture core's tick 0 sits in CM33 time, and whether the clocks drift."),
                    ("trace_lag.py", vec!["4".to_string()], "How far behind the wire the controller stamps its own events."),
                    ("merge_both.py", vec!["3".to_string()], "A `both` capture merged into one ordered stream, with its error bar."),
                ] {
                    if ui.button(script).clicked() {
                        self.start_script(script, &args);
                    }
                    ui.label(RichText::new(what).weak().small());
                    ui.end_row();
                }
            });
        });

        ui.add_space(10.0);
        ui.separator();
        self.runs_view(ui);
    }

    fn start_script(&mut self, script: &str, args: &[String]) {
        // The script needs the board, and this window has it. Hand it over
        // rather than letting the script fail to open it.
        if self.connected.is_some() {
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
        let mut stop_idx = None;
        for (i, r) in self.runs.iter().enumerate().rev() {
            ui.horizontal(|ui| {
                let c = match r.verdict() {
                    "passed" => Color32::from_rgb(120, 200, 120),
                    "FAILED" => Color32::from_rgb(220, 90, 90),
                    _ => Color32::GRAY,
                };
                ui.label(RichText::new(&r.title).strong());
                ui.label(RichText::new(r.verdict()).color(c));
                if !r.done && ui.small_button("stop").clicked() {
                    stop_idx = Some(i);
                }
            });
            egui::ScrollArea::vertical()
                .id_salt(format!("run{i}"))
                .max_height(240.0)
                .stick_to_bottom(true)
                .show(ui, |ui| {
                    for l in &r.lines {
                        let c = if l.starts_with('!') {
                            Color32::from_rgb(220, 90, 90)
                        } else if l.contains("FAIL") {
                            Color32::from_rgb(220, 90, 90)
                        } else if l.contains("PASS") {
                            Color32::from_rgb(120, 200, 120)
                        } else {
                            ui.visuals().text_color()
                        };
                        ui.label(RichText::new(l).monospace().color(c));
                    }
                });
            ui.separator();
        }
        if let Some(i) = stop_idx {
            self.runs[i].stop();
        }
    }

    fn console(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("Command");
            let r = ui.add(
                egui::TextEdit::singleline(&mut self.raw_entry)
                    .desired_width(ui.available_width() - 90.0)
                    .hint_text("SNIFF:STAT?"),
            );
            let send = (r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)))
                || ui.button("Send").clicked();
            if send && !self.raw_entry.trim().is_empty() {
                self.send(Req::Raw(self.raw_entry.trim().to_string()));
                self.raw_entry.clear();
                r.request_focus();
            }
        });
        egui::ScrollArea::vertical()
            .id_salt("log")
            .stick_to_bottom(true)
            .show(ui, |ui| {
                for (cmd, reply) in &self.log {
                    ui.horizontal_wrapped(|ui| {
                        ui.label(RichText::new(format!("> {cmd}")).monospace().weak());
                    });
                    let c = if reply.starts_with("ERR") {
                        Color32::from_rgb(220, 90, 90)
                    } else {
                        ui.visuals().text_color()
                    };
                    ui.horizontal_wrapped(|ui| {
                        ui.label(RichText::new(reply).monospace().color(c));
                    });
                }
            });
    }
}

/// The decoded event stream, wrapped and coloured by what each token is.
///
/// Framing apart from data on purpose: a wall of `D40+ D11+ D22+` with the
/// STARTs the same colour reads as one long transfer, and the thing a person
/// is usually looking for is where the frames begin.
fn events_view(ui: &mut egui::Ui, events: &[String]) {
    egui::ScrollArea::vertical()
        .id_salt("events")
        .stick_to_bottom(true)
        .auto_shrink([false, false])
        .show(ui, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.spacing_mut().item_spacing.x = 4.0;
                for t in events {
                    let c = match t.as_str() {
                        "S" | "Sr" => Color32::from_rgb(120, 190, 240),
                        "P" => Color32::from_rgb(150, 150, 170),
                        "!" => Color32::from_rgb(220, 90, 90),
                        _ if t.starts_with('A') => Color32::from_rgb(230, 200, 120),
                        _ if t.starts_with('C') => Color32::from_rgb(200, 140, 230),
                        _ => ui.visuals().text_color(),
                    };
                    ui.label(RichText::new(t).monospace().color(c));
                }
            });
        });
}
