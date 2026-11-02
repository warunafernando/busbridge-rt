//! The live waveform: the capture's raw edge list, drawn.
//!
//! A second view of the same capture rather than a second capture. The
//! firmware stores the container - records, headers, per-entry deltas - and
//! `SNIFF:RAW:READ?` converts that to what a waveform actually needs: a
//! free-running tick and a line mask, five bytes each. Neither presentation
//! subsumes the other, and the conversion lives in the one place that has to
//! know both.
//!
//! WHAT A WAVEFORM HERE CAN AND CANNOT SHOW
//!
//! In oversampled mode this is a measurement: a free-running sample clock, so
//! the time between two edges is real and the idle between transactions is
//! real. In pin-clocked mode it is not. That mode is shifted by the bus's own
//! clock, so a "tick" is a bus clock and the spacing between edges says what
//! the bus did, not when. The drawing is the same; the axis means something
//! different, and the panel says so rather than leaving it to be inferred.
//!
//! Pin-clocked mode also skips the raw sample store while decoding - that is
//! what bought 6 to 8 MHz - so a waveform there needs `SNIFF:DECODE 0`, and
//! asking for both at once gets an empty trace rather than an error. The
//! capture panel refuses that combination instead of drawing nothing.

use eframe::egui;
use egui::{Color32, Sense, Stroke, Vec2};

/// One transition: the tick it happened at, and every line's level after it.
#[derive(Clone, Copy)]
pub struct Edge {
    pub tick: u64,
    pub lines: u8,
}

pub struct Edges {
    pub list: Vec<Edge>,
    pub tick_hz: u64,
    /// Ticks are 32-bit on the wire and only ever go up, so they are unwrapped
    /// here rather than at the draw, where a wrap would read as the capture
    /// jumping backwards by seven minutes.
    last_raw: u32,
    wrap: u64,
    started: bool,
}

impl Default for Edges {
    fn default() -> Self {
        Edges {
            list: Vec::new(),
            tick_hz: 10_000_000,
            last_raw: 0,
            wrap: 0,
            started: false,
        }
    }
}

const MAX_EDGES: usize = 400_000;

impl Edges {
    pub fn clear(&mut self) {
        self.list.clear();
        self.last_raw = 0;
        self.wrap = 0;
        self.started = false;
    }

    /// Take a `SNIFF:RAW:READ?` reply: "OK F1 82 00 00 03 ..." as hex bytes.
    ///
    /// A reply that is not a whole number of records is refused rather than
    /// parsed as far as it goes. A partial record is a transport fault, and
    /// this project has already paid once for a truncated reply that was
    /// parsed into plausible, wrong data.
    pub fn push_reply(&mut self, reply: &str) -> Result<usize, String> {
        let body = reply.strip_prefix("OK").unwrap_or(reply).trim();
        if body.is_empty() {
            return Ok(0);
        }
        let mut bytes = Vec::new();
        for tok in body.split_whitespace() {
            match u8::from_str_radix(tok, 16) {
                Ok(b) => bytes.push(b),
                Err(_) => return Err(format!("not a hex record: {tok}")),
            }
        }
        if bytes.len() % 5 != 0 {
            return Err(format!(
                "{} bytes is not a whole number of 5-byte records",
                bytes.len()
            ));
        }
        let n = bytes.len() / 5;
        for i in 0..n {
            let o = i * 5;
            let raw = u32::from_le_bytes([bytes[o], bytes[o + 1], bytes[o + 2], bytes[o + 3]]);
            if self.started && raw < self.last_raw {
                self.wrap += 1u64 << 32;
            }
            self.started = true;
            self.last_raw = raw;
            self.list.push(Edge {
                tick: self.wrap + raw as u64,
                lines: bytes[o + 4],
            });
        }
        if self.list.len() > MAX_EDGES {
            let drop = self.list.len() - MAX_EDGES;
            self.list.drain(..drop);
        }
        Ok(n)
    }

    pub fn span_ticks(&self) -> (u64, u64) {
        match (self.list.first(), self.list.last()) {
            (Some(a), Some(b)) => (a.tick, b.tick),
            _ => (0, 0),
        }
    }

    /// The most recent burst of activity, which is what "fit" should show.
    ///
    /// Fitting the captured span was the obvious thing and it is the wrong
    /// thing. An oversampled capture of a real bus is mostly idle: driving two
    /// transfers by hand put 1.6 seconds between them, so the whole span was
    /// 1640 ms with every transition inside two slivers at the ends. Correct,
    /// and useless.
    ///
    /// Trimming leading and trailing silence does not help either, because
    /// there was none to trim - the idle was in the MIDDLE, between two real
    /// frames, and cutting that would hide one of them. What a person wants on
    /// a live capture is the thing that just happened, so this walks back from
    /// the newest edge while the gaps stay small and stops at the first real
    /// silence.
    ///
    /// The threshold is relative - twenty times the median gap - rather than
    /// an absolute time, so the same capture fits the same way at 1 MHz and at
    /// 12.5 MHz.
    pub fn last_burst(&self) -> (u64, u64) {
        if self.list.len() < 3 {
            return self.span_ticks();
        }
        let mut gaps: Vec<u64> = self
            .list
            .windows(2)
            .map(|w| w[1].tick.saturating_sub(w[0].tick))
            .collect();
        gaps.sort_unstable();
        let median = gaps[gaps.len() / 2].max(1);
        let idle = median.saturating_mul(20).max(4);

        let hi = self.list.len() - 1;
        let mut lo = hi;
        while lo > 0 && self.list[lo].tick.saturating_sub(self.list[lo - 1].tick) <= idle {
            lo -= 1;
        }
        if hi == lo {
            return self.span_ticks();
        }
        let width = self.list[hi].tick - self.list[lo].tick;
        // A little air either side so the first and last transition are not
        // drawn on the frame - and clamped, because padding that pushed the
        // fitted window past the captured one reported `active 1804 ms` for a
        // capture 1640 ms long, which is a number that cannot be true.
        let pad = (width / 20 + 1).min(width.max(1));
        let (f0, f1) = self.span_ticks();
        (
            self.list[lo].tick.saturating_sub(pad).max(f0),
            (self.list[hi].tick + pad).min(f1.max(self.list[hi].tick)),
        )
    }
}

/// What part of the capture is on screen.
pub struct View {
    /// Left edge, in ticks.
    pub left: f64,
    /// How many ticks the window is wide.
    pub width: f64,
    /// Keep the right edge pinned to the newest edge.
    pub follow: bool,
}

impl Default for View {
    fn default() -> Self {
        View {
            left: 0.0,
            width: 2000.0,
            follow: true,
        }
    }
}

const ROW_H: f32 = 46.0;
const PAD: f32 = 12.0;
const RULER_H: f32 = 20.0;

/// A tick step that lands on a round number.
///
/// 1, 2 or 5 times a power of ten - the spacings a person reads without having
/// to divide. Chosen from the span rather than fixed, so the same ruler works
/// across the six orders of magnitude between a 12.5 MHz bit and an idle
/// second.
fn nice_step(span: f64, want: f64) -> f64 {
    if span <= 0.0 {
        return 1.0;
    }
    let raw = span / want.max(1.0);
    let mag = 10f64.powf(raw.log10().floor());
    let n = raw / mag;
    mag * if n < 1.5 {
        1.0
    } else if n < 3.5 {
        2.0
    } else if n < 7.5 {
        5.0
    } else {
        10.0
    }
}

/// A time offset, at the scale the span deserves.
fn tick_label(t: f64, step: f64) -> String {
    let a = t.abs();
    if step >= 1e-3 {
        format!("{:.*} ms", if step >= 1e-2 { 0 } else { 1 }, t * 1e3)
    } else if step >= 1e-6 {
        format!("{:.*} us", if step >= 1e-5 { 0 } else { 1 }, t * 1e6)
    } else if a < 1.0 {
        format!("{:.0} ns", t * 1e9)
    } else {
        format!("{t:.0} s")
    }
}

/// Draw the lines, the ruler, and handle pan and zoom.
///
/// `names` is in tap order, which is the order the firmware was told to bind
/// them - so index 0 is the clock, and that is why it is drawn first and
/// labelled rather than coloured the same as the rest.
pub fn draw(ui: &mut egui::Ui, edges: &Edges, view: &mut View, names: &[String], pinclock: bool) {
    let n = names.len().max(1);
    let h = ROW_H * n as f32 + PAD * 2.0 + RULER_H;
    let (resp, painter) =
        ui.allocate_painter(Vec2::new(ui.available_width(), h), Sense::click_and_drag());
    let rect = resp.rect;
    painter.rect_filled(rect, 4.0, ui.visuals().extreme_bg_color);

    if edges.list.len() < 2 {
        painter.text(
            rect.center(),
            egui::Align2::CENTER_CENTER,
            "no edges captured",
            egui::FontId::proportional(13.0),
            ui.visuals().weak_text_color(),
        );
        return;
    }

    let (first, last) = edges.span_ticks();
    if view.follow {
        view.left = (last as f64 - view.width).max(first as f64);
    }

    // Zoom about the pointer rather than the centre: zooming about the centre
    // walks whatever you were looking at off the screen.
    if resp.hovered() {
        let scroll = ui.input(|i| i.smooth_scroll_delta.y);
        if scroll.abs() > 0.1 {
            let f = (1.0 - scroll as f64 * 0.002).clamp(0.2, 5.0);
            let anchor = ui
                .input(|i| i.pointer.hover_pos())
                .map(|p| ((p.x - rect.left()) / rect.width()).clamp(0.0, 1.0) as f64)
                .unwrap_or(0.5);
            let at = view.left + view.width * anchor;
            view.width = (view.width * f).clamp(8.0, (last - first) as f64 * 4.0 + 64.0);
            view.left = at - view.width * anchor;
            view.follow = false;
        }
    }
    if resp.dragged() {
        let dx = resp.drag_delta().x as f64;
        view.left -= dx * view.width / rect.width() as f64;
        view.follow = false;
    }

    let t0 = view.left;
    let tw = view.width.max(1.0);
    let x_of = |tick: f64| rect.left() + ((tick - t0) / tw) as f32 * rect.width();

    let plot_bottom = rect.bottom() - RULER_H;
    let grid_c = Color32::from_rgb(0x20, 0x29, 0x32);
    let clock_c = Color32::from_rgb(0x6F, 0xB6, 0xD8);
    let data_c = Color32::from_rgb(0x8F, 0xE3, 0xB0);
    let dim = Color32::from_rgb(0x5A, 0x68, 0x76);

    // ---- grid and ruler ---------------------------------------------------
    //
    // In pin-clocked mode a tick is a bus clock, not a unit of time, so the
    // ruler counts clocks. Printing microseconds there would be a plausible
    // wrong answer: the numbers would look like time and would not be.
    let hz = edges.tick_hz.max(1) as f64;
    let (span_u, unit_per_tick) = if pinclock {
        (tw, 1.0)
    } else {
        (tw / hz, 1.0 / hz)
    };
    let step_u = nice_step(span_u, 7.0);
    let step_ticks = step_u / unit_per_tick;
    // Labels relative to the left edge of the view, not to the capture.
    //
    // Absolute offsets from the first edge read 1620600 us, 1620620 us - seven
    // digits where two carry meaning, and the eye has to subtract two of them
    // to get the thing it wanted. A view-relative ruler is what a scope shows
    // and what a person is already reading when they zoom in on a frame.
    if step_ticks.is_finite() && step_ticks > 0.0 {
        let base = t0;
        let mut k = ((t0 - base) / step_ticks).floor();
        for _ in 0..40 {
            let tick = base + k * step_ticks;
            k += 1.0;
            if tick < t0 {
                continue;
            }
            if tick > t0 + tw {
                break;
            }
            let x = x_of(tick);
            painter.line_segment(
                [
                    egui::pos2(x, rect.top() + PAD * 0.5),
                    egui::pos2(x, plot_bottom),
                ],
                Stroke::new(1.0, grid_c),
            );
            let off = (tick - base) * unit_per_tick;
            let label = if pinclock {
                format!("{off:.0}")
            } else {
                tick_label(off, step_u)
            };
            painter.text(
                egui::pos2(x + 3.0, plot_bottom + 3.0),
                egui::Align2::LEFT_TOP,
                label,
                egui::FontId::monospace(10.0),
                dim,
            );
        }
    }
    painter.line_segment(
        [
            egui::pos2(rect.left(), plot_bottom),
            egui::pos2(rect.right(), plot_bottom),
        ],
        Stroke::new(1.0, grid_c),
    );

    // ---- the traces -------------------------------------------------------
    for (ch, name) in names.iter().enumerate() {
        let top = rect.top() + PAD + ROW_H * ch as f32;
        let lo = top + ROW_H - 18.0;
        let hi = top + 4.0;
        let colour = if ch == 0 { clock_c } else { data_c };
        let stroke = Stroke::new(1.5, colour);
        let bit = 1u8 << ch;

        painter.text(
            egui::pos2(rect.left() + 7.0, top - 2.0),
            egui::Align2::LEFT_TOP,
            name,
            egui::FontId::monospace(10.5),
            colour.gamma_multiply(0.85),
        );

        // Walk only the edges inside the view, plus the one before it - which
        // is what says where the line was when the window opens. Without it a
        // trace drawn from a window that starts mid-transaction begins at the
        // wrong level, and a level is not something an edge list states.
        let start = edges
            .list
            .partition_point(|e| (e.tick as f64) < t0)
            .saturating_sub(1);
        let mut prev_x = x_of(edges.list[start].tick as f64).max(rect.left());
        let mut prev_y = if edges.list[start].lines & bit != 0 { hi } else { lo };

        for e in edges.list[start..].iter() {
            let x = x_of(e.tick as f64);
            if x > rect.right() {
                break;
            }
            let y = if e.lines & bit != 0 { hi } else { lo };
            let xc = x.clamp(rect.left(), rect.right());
            painter.line_segment([egui::pos2(prev_x, prev_y), egui::pos2(xc, prev_y)], stroke);
            if (y - prev_y).abs() > 0.5 {
                painter.line_segment([egui::pos2(xc, prev_y), egui::pos2(xc, y)], stroke);
            }
            prev_x = xc;
            prev_y = y;
        }
        painter.line_segment(
            [egui::pos2(prev_x, prev_y), egui::pos2(rect.right(), prev_y)],
            stroke,
        );
    }

    // ---- cursor -----------------------------------------------------------
    //
    // Where the pointer is, in the same units the ruler uses. A waveform you
    // cannot take a reading off is a picture of a waveform.
    if let Some(p) = resp.hover_pos() {
        let x = p.x.clamp(rect.left(), rect.right());
        painter.line_segment(
            [egui::pos2(x, rect.top()), egui::pos2(x, plot_bottom)],
            Stroke::new(1.0, Color32::from_rgb(0xE0, 0xB1, 0x5C).gamma_multiply(0.7)),
        );
        let tick = t0 + (x - rect.left()) as f64 / rect.width() as f64 * tw;
        let off = (tick - t0) * unit_per_tick;
        let txt = if pinclock {
            format!("{off:.0} clk")
        } else {
            tick_label(off, step_u.min(off.abs().max(1e-9)))
        };
        painter.text(
            egui::pos2(x + 4.0, rect.top() + 2.0),
            egui::Align2::LEFT_TOP,
            txt,
            egui::FontId::monospace(10.5),
            Color32::from_rgb(0xE0, 0xB1, 0x5C),
        );
    }

    // No "N us across" footer. The ruler carries the scale now, and the
    // footer sat on top of the right-hand tick label - two numbers in the same
    // place, one of them wrong about what it measured.
}
