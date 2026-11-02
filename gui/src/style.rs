//! The window's look, in one place.
//!
//! Family resemblance to the analyser next door is deliberate - they are two
//! views of the same captures and a person moves between them - but the
//! palette is set out here rather than shared, because a shared one would make
//! this crate depend on that one for a reason that is not functional.
//!
//! The rules the rest of the window follows:
//!
//!   - Controls come before readouts on a line, and readouts are pushed to the
//!     right. A number that changes width must never move a button: the span
//!     readout was added to the left of `follow` and `fit` and shifted both of
//!     them by sixty pixels, which is how a click aimed at `fit` landed on
//!     `follow`. Anything that grows gets `right_to_left` and lives at the end.
//!   - Monospace for anything read as data - bytes, tokens, register values,
//!     timings - and proportional for prose. A hex byte in a proportional font
//!     is harder to compare against the one above it than it looks.
//!   - One accent colour, used for the thing that is currently true. Two
//!     accents and neither means anything.

use eframe::egui;
use egui::{Color32, RichText, Ui};

pub struct Palette;

impl Palette {
    pub const TEXT: Color32 = Color32::from_rgb(0xD8, 0xE2, 0xEC);
    pub const DIM: Color32 = Color32::from_rgb(0x7E, 0x8E, 0x9F);
    pub const FAINT: Color32 = Color32::from_rgb(0x5A, 0x68, 0x76);
    pub const ACCENT: Color32 = Color32::from_rgb(0x4F, 0xB3, 0xE8);
    pub const GOOD: Color32 = Color32::from_rgb(0x7A, 0xC7, 0x8E);
    pub const WARN: Color32 = Color32::from_rgb(0xE0, 0xB1, 0x5C);
    pub const BAD: Color32 = Color32::from_rgb(0xE0, 0x7A, 0x5F);
    /// The clock trace, and the framing tokens that mark where frames begin.
    pub const CLOCK: Color32 = Color32::from_rgb(0x6F, 0xB6, 0xD8);
    /// Data traces and data bytes.
    pub const DATA: Color32 = Color32::from_rgb(0x8F, 0xE3, 0xB0);
    pub const ADDR: Color32 = Color32::from_rgb(0xE3, 0xC4, 0x7C);
    pub const CCC: Color32 = Color32::from_rgb(0xC9, 0x9B, 0xE8);
    pub const PLOT_BG: Color32 = Color32::from_rgb(0x0E, 0x12, 0x16);
    pub const CARD: Color32 = Color32::from_rgb(0x1C, 0x22, 0x29);
    pub const LINE: Color32 = Color32::from_rgb(0x2C, 0x35, 0x3E);
}

pub fn install(ctx: &egui::Context) {
    ctx.all_styles_mut(|style| {
        style.spacing.item_spacing = egui::vec2(9.0, 7.0);
        style.spacing.button_padding = egui::vec2(10.0, 5.0);
        style.spacing.interact_size.y = 24.0;
        style.spacing.menu_margin = egui::Margin::same(8);
        for w in [
            &mut style.visuals.widgets.noninteractive,
            &mut style.visuals.widgets.inactive,
            &mut style.visuals.widgets.hovered,
            &mut style.visuals.widgets.active,
            &mut style.visuals.widgets.open,
        ] {
            w.corner_radius = egui::CornerRadius::same(5);
            w.expansion = 0.0;
        }
        // A readable base size. The default is tuned for dense tooling and
        // this window is read across a bench, often at arm's length.
        for (st, id) in style.text_styles.iter_mut() {
            let sz = match st {
                egui::TextStyle::Heading => 19.0,
                egui::TextStyle::Small => 11.0,
                egui::TextStyle::Monospace => 12.5,
                _ => 13.5,
            };
            id.size = sz;
        }
    });

    let mut dark = (*ctx.style_of(egui::Theme::Dark)).clone();
    dark.visuals.panel_fill = Color32::from_rgb(0x15, 0x1A, 0x20);
    dark.visuals.window_fill = Color32::from_rgb(0x1A, 0x20, 0x27);
    dark.visuals.faint_bg_color = Color32::from_rgb(0x1F, 0x26, 0x2D);
    dark.visuals.extreme_bg_color = Palette::PLOT_BG;
    dark.visuals.override_text_color = Some(Palette::TEXT);
    dark.visuals.widgets.noninteractive.bg_stroke = egui::Stroke::new(1.0, Palette::LINE);
    dark.visuals.widgets.inactive.bg_fill = Color32::from_rgb(0x25, 0x2D, 0x36);
    dark.visuals.widgets.inactive.weak_bg_fill = Color32::from_rgb(0x22, 0x29, 0x31);
    dark.visuals.widgets.hovered.bg_fill = Color32::from_rgb(0x30, 0x3A, 0x45);
    dark.visuals.widgets.hovered.weak_bg_fill = Color32::from_rgb(0x2C, 0x35, 0x3F);
    dark.visuals.widgets.active.bg_fill = Color32::from_rgb(0x3A, 0x46, 0x53);
    dark.visuals.selection.bg_fill = Palette::ACCENT.gamma_multiply(0.35);
    dark.visuals.selection.stroke = egui::Stroke::new(1.0, Palette::ACCENT);
    ctx.set_style_of(egui::Theme::Dark, dark);
    ctx.set_theme(egui::ThemePreference::Dark);
}

/// A titled card. Everything in a panel lives in one of these, so a panel
/// reads as a few things rather than as forty widgets.
pub fn card<R>(ui: &mut Ui, title: &str, add: impl FnOnce(&mut Ui) -> R) -> R {
    egui::Frame::new()
        .fill(Palette::CARD)
        .stroke(egui::Stroke::new(1.0, Palette::LINE))
        .corner_radius(egui::CornerRadius::same(7))
        .inner_margin(egui::Margin::symmetric(13, 11))
        .show(ui, |ui| {
            // Fill the panel. A card that shrinks to its contents makes a
            // column of cards look ragged, and the eye reads the ragged edge
            // as meaning something.
            ui.set_width(ui.available_width());
            if !title.is_empty() {
                ui.label(
                    RichText::new(title.to_uppercase())
                        .color(Palette::FAINT)
                        .small()
                        .strong(),
                );
                ui.add_space(6.0);
            }
            add(ui)
        })
        .inner
}

/// A row with a fixed-width label, so the controls on every row line up.
///
/// Without this each row started wherever its own label ended, and a column of
/// controls that does not line up reads as unrelated controls.
pub fn row<R>(ui: &mut Ui, label: &str, add: impl FnOnce(&mut Ui) -> R) -> R {
    ui.horizontal(|ui| {
        let (rect, _) = ui.allocate_exact_size(egui::vec2(74.0, 20.0), egui::Sense::hover());
        ui.painter().text(
            egui::pos2(rect.left(), rect.center().y),
            egui::Align2::LEFT_CENTER,
            label,
            egui::FontId::proportional(13.0),
            Palette::DIM,
        );
        add(ui)
    })
    .inner
}

/// A segmented control: one of n, drawn as one object rather than n labels.
///
/// Returns true when the selection changed. The hover text belongs on the
/// option rather than the group, because what each option costs is the thing
/// worth saying and it differs per option.
pub fn segmented(ui: &mut Ui, current: &mut String, options: &[(&str, &str)]) -> bool {
    let mut changed = false;
    egui::Frame::new()
        .fill(Color32::from_rgb(0x1A, 0x20, 0x27))
        .corner_radius(egui::CornerRadius::same(6))
        .inner_margin(egui::Margin::same(2))
        .show(ui, |ui| {
            ui.spacing_mut().item_spacing.x = 2.0;
            ui.horizontal(|ui| {
                for (name, hint) in options {
                    let on = current == name;
                    let text = if on {
                        RichText::new(*name).color(Color32::BLACK).strong()
                    } else {
                        RichText::new(*name).color(Palette::DIM)
                    };
                    let btn = egui::Button::new(text)
                        .fill(if on {
                            Palette::ACCENT
                        } else {
                            Color32::TRANSPARENT
                        })
                        .corner_radius(egui::CornerRadius::same(5));
                    if ui.add(btn).on_hover_text(*hint).clicked() && !on {
                        *current = (*name).to_string();
                        changed = true;
                    }
                }
            });
        });
    changed
}

/// A labelled number, for the right-hand end of a control row.
///
/// One label with two colours rather than two labels in a row. The row version
/// needed a nested horizontal to keep "edges 12" from rendering as "12 edges"
/// inside a right-to-left group - and that nested layout took the full
/// available width, so every chip stretched across the window. A single
/// LayoutJob has no direction to inherit and sizes to exactly its text.
pub fn chip(ui: &mut Ui, label: &str, value: &str, colour: Color32) {
    let mut job = egui::text::LayoutJob::default();
    let small = egui::FontId::proportional(11.0);
    let mono = egui::FontId::monospace(12.0);
    job.append(
        label,
        0.0,
        egui::TextFormat {
            font_id: small,
            color: Palette::FAINT,
            ..Default::default()
        },
    );
    job.append(
        value,
        6.0,
        egui::TextFormat {
            font_id: mono,
            color: colour,
            ..Default::default()
        },
    );
    egui::Frame::new()
        .fill(Color32::from_rgb(0x1A, 0x20, 0x27))
        .corner_radius(egui::CornerRadius::same(5))
        .inner_margin(egui::Margin::symmetric(8, 3))
        .show(ui, |ui| ui.label(job));
}

/// Everything that changes width, pushed to the right-hand end of a row.
///
/// `allocate_ui_with_layout` with the remaining width, not a bare
/// `with_layout`: the bare form right-aligns inside whatever the cursor had
/// left, which inside a content-sized horizontal is nothing - so the chips sat
/// against the last button instead of against the window edge.
pub fn right<R>(ui: &mut Ui, add: impl FnOnce(&mut Ui) -> R) -> R {
    let w = ui.available_width();
    ui.allocate_ui_with_layout(
        egui::vec2(w, 24.0),
        egui::Layout::right_to_left(egui::Align::Center),
        add,
    )
    .inner
}

/// A note the reader should not skip, in the colour that says how much to care.
pub fn note(ui: &mut Ui, colour: Color32, text: &str) {
    ui.horizontal_wrapped(|ui| {
        ui.label(RichText::new(text).color(colour).small());
    });
}
