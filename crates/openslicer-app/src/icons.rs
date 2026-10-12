//! Small vector icons drawn with the egui painter, so no icon font is required.

use eframe::egui::{pos2, vec2, Color32, Painter, Pos2, Rect, Shape, Stroke};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Icon {
    Open,
    Export,
    Select,
    Move,
    Drop,
    Fit,
    Cube,
    Printer,
    Split,
    Connector,
    Assembly,
    Eye,
    EyeOff,
}

/// Draws `icon` inside `rect` (the glyph occupies the central ~60%).
pub fn paint(painter: &Painter, rect: Rect, icon: Icon, color: Color32) {
    let s = rect.width().min(rect.height());
    let c = rect.center();
    let u = s / 24.0; // design grid: 24 units
    let p = |x: f32, y: f32| pos2(c.x + x * u, c.y + y * u);
    let st = Stroke::new((1.6 * u).max(1.2), color);
    let line = |a: Pos2, b: Pos2| painter.line_segment([a, b], st);
    let poly = |pts: &[Pos2], closed: bool| {
        painter.add(Shape::Path(eframe::epaint::PathShape { points: pts.to_vec(), closed, fill: Color32::TRANSPARENT, stroke: st.into() }));
    };
    match icon {
        Icon::Open => {
            poly(&[p(-9., -5.), p(-9., 7.), p(9., 7.), p(9., -3.), p(0., -3.), p(-2., -6.), p(-9., -6.)], true);
        }
        Icon::Export => {
            poly(&[p(-8., 2.), p(-8., 8.), p(8., 8.), p(8., 2.)], false);
            line(p(0., -9.), p(0., 3.));
            poly(&[p(-4., -1.), p(0., 3.), p(4., -1.)], false);
        }
        Icon::Select => {
            poly(&[p(-6., -9.), p(-6., 7.), p(-2., 3.), p(1., 9.), p(4., 8.), p(1., 2.), p(7., 2.)], true);
        }
        Icon::Move => {
            line(p(-9., 0.), p(9., 0.));
            line(p(0., -9.), p(0., 9.));
            poly(&[p(-6., -3.), p(-9., 0.), p(-6., 3.)], false);
            poly(&[p(6., -3.), p(9., 0.), p(6., 3.)], false);
            poly(&[p(-3., -6.), p(0., -9.), p(3., -6.)], false);
            poly(&[p(-3., 6.), p(0., 9.), p(3., 6.)], false);
        }
        Icon::Drop => {
            line(p(-9., 8.), p(9., 8.));
            line(p(0., -9.), p(0., 4.));
            poly(&[p(-4., 0.), p(0., 4.), p(4., 0.)], false);
        }
        Icon::Fit => {
            for (sx, sy) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
                poly(&[p(sx * 9., sy * 4.), p(sx * 9., sy * 9.), p(sx * 4., sy * 9.)], false);
            }
            painter.rect_stroke(Rect::from_center_size(c, vec2(8. * u, 8. * u)), 1.0, st, eframe::egui::StrokeKind::Middle);
        }
        Icon::Cube => {
            poly(&[p(0., -9.), p(8., -4.5), p(8., 4.5), p(0., 9.), p(-8., 4.5), p(-8., -4.5)], true);
            poly(&[p(-8., -4.5), p(0., 0.), p(8., -4.5)], false);
            line(p(0., 0.), p(0., 9.));
        }
        Icon::Printer => {
            painter.rect_stroke(Rect::from_min_max(p(-9., -9.), p(9., 9.)), 1.5, st, eframe::egui::StrokeKind::Middle);
            for i in -1..=1 {
                let o = i as f32 * 6.0;
                line(p(o, -9.), p(o, 9.));
                line(p(-9., o), p(9., o));
            }
        }
        Icon::Split => {
            poly(&[p(-9., -7.), p(9., -7.), p(9., 7.), p(-9., 7.)], true);
            // dashed cut line
            for k in 0..5 {
                let y = -8.0 + k as f32 * 4.0;
                line(p(0., y), p(0., y + 2.0));
            }
        }
        Icon::Connector => {
            poly(&[p(-9., -3.), p(-2., -3.), p(-2., 3.), p(-9., 3.)], true);
            poly(&[p(2., -3.), p(9., -3.), p(9., 3.), p(2., 3.)], true);
            line(p(-2., 0.), p(2., 0.));
        }
        Icon::Assembly => {
            poly(&[p(-9., 0.), p(-3., -3.), p(3., 0.), p(-3., 3.)], true);
            poly(&[p(0., -8.), p(6., -5.), p(12., -8.), p(6., -11.)], true);
            line(p(0., 6.), p(0., 9.));
        }
        Icon::Eye | Icon::EyeOff => {
            poly(&[p(-10., 0.), p(-5., -5.), p(0., -6.), p(5., -5.), p(10., 0.), p(5., 5.), p(0., 6.), p(-5., 5.)], true);
            painter.circle_stroke(c, 2.5 * u, st);
            if icon == Icon::EyeOff {
                line(p(-9., 9.), p(9., -9.));
            }
        }
    }
}
