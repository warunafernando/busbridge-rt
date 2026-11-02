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
use egui::{Color32, Rect, Sense, Stroke, Vec2};

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

const ROW_H: f32 = 44.0;
const PAD: f32 = 10.0;

/// Draw the lines, and handle pan and zoom.
///
/// `names` is in tap order, which is the order the firmware was told to bind
/// them - so index 0 is the clock, and that is why it is drawn first and
/// labelled rather than coloured the same as the rest.
pub fn draw(ui: &mut egui::Ui, edges: &Edges, view: &mut View, names: &[String], pinclock: bool) {
    let n = names.len().max(1);
    let h = ROW_H * n as f32 + PAD * 2.0 + 18.0;
    let (resp, painter) =
        ui.allocate_painter(Vec2::new(ui.available_width(), h), Sense::click_and_drag());
    let rect = resp.rect;
    painter.rect_filled(rect, 2.0, ui.visuals().extreme_bg_color);

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

    let grid = Stroke::new(1.0, ui.visuals().weak_text_color().gamma_multiply(0.25));
    let clock_c = Color32::from_rgb(120, 190, 240);
    let data_c = Color32::from_rgb(140, 220, 160);

    for (ch, name) in names.iter().enumerate() {
        let top = rect.top() + PAD + ROW_H * ch as f32;
        let lo = top + ROW_H - 14.0;
        let hi = top + 6.0;
        painter.line_segment(
            [egui::pos2(rect.left(), lo), egui::pos2(rect.right(), lo)],
            grid,
        );
        painter.text(
            egui::pos2(rect.left() + 4.0, top),
            egui::Align2::LEFT_TOP,
            name,
            egui::FontId::monospace(11.0),
            if ch == 0 { clock_c } else { data_c },
        );

        let colour = if ch == 0 { clock_c } else { data_c };
        let stroke = Stroke::new(1.4, colour);
        let bit = 1u8 << ch;

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
            painter.line_segment(
                [egui::pos2(prev_x, prev_y), egui::pos2(xc, prev_y)],
                stroke,
            );
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

    // The axis, and what it means - which is not the same in both modes.
    let hz = edges.tick_hz.max(1) as f64;
    let label = if pinclock {
        format!("{:.0} bus clocks across", tw)
    } else {
        let us = tw / hz * 1e6;
        if us >= 1000.0 {
            format!("{:.2} ms across", us / 1000.0)
        } else {
            format!("{us:.2} us across")
        }
    };
    painter.text(
        egui::pos2(rect.right() - 6.0, rect.bottom() - 4.0),
        egui::Align2::RIGHT_BOTTOM,
        label,
        egui::FontId::monospace(11.0),
        ui.visuals().weak_text_color(),
    );
}
