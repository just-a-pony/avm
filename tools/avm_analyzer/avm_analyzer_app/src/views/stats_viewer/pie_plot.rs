use std::f64::consts::PI;

use avm_stats::Sample;
use egui::RichText;
use egui_plot::{Plot, PlotPoint, PlotPoints, PlotResponse, Polygon, Text};

pub struct PiePlot {
    pub num_vertices: usize,
    pub decimal_precision: usize,
}

impl Default for PiePlot {
    fn default() -> Self {
        Self {
            num_vertices: 360,
            decimal_precision: 2,
        }
    }
}

impl PiePlot {
    pub fn show(&self, ui: &mut egui::Ui, data: &[Sample]) -> PlotResponse<()> {
        let total: f64 = data.iter().map(|sample| sample.value).sum();
        let mut cumulative_sum = 0.0;

        Plot::new("pie_plot")
            .show_background(false)
            .show_axes([false; 2])
            .clamp_grid(true)
            .show_grid(false)
            .allow_boxed_zoom(false)
            .allow_drag(false)
            .allow_zoom(false)
            .allow_scroll(false)
            .data_aspect(1.0)
            .show_x(false)
            .show_y(false)
            .include_x(-1.1)
            .include_x(1.1)
            .include_y(-1.1)
            .include_y(1.1)
            .show(ui, |plot_ui| {
                for Sample { name, value } in data.iter() {
                    let fraction = value / total;
                    let num_vertices = (self.num_vertices as f64 * fraction).ceil() as usize;
                    let start_angle = 2.0 * PI * cumulative_sum;
                    let end_angle = 2.0 * PI * (cumulative_sum + fraction);
                    cumulative_sum += fraction;
                    let mut points = vec![];

                    if data.len() > 1 {
                        points.push([0.0, 0.0]);
                    }

                    let angle_step = (end_angle - start_angle) / num_vertices as f64;
                    points.extend((0..=num_vertices).map(|i| {
                        let angle = start_angle + angle_step * i as f64;
                        [angle.sin(), angle.cos()]
                    }));

                    let center_angle = start_angle + (end_angle - start_angle) / 2.0;
                    let center_x = 0.75 * center_angle.sin();
                    let center_y = 0.75 * center_angle.cos();

                    let hovered = plot_ui
                        .pointer_coordinate()
                        .map(|pointer| {
                            let radius = pointer.y.hypot(pointer.x);
                            let mut theta = pointer.x.atan2(pointer.y);
                            if theta < 0.0 {
                                theta += 2.0 * PI;
                            }
                            radius < 1.0 && start_angle < theta && theta < end_angle
                        })
                        .unwrap_or_default();

                    plot_ui.polygon(Polygon::new(PlotPoints::new(points)).name(name).highlight(hovered));

                    let label = format!("{} - {:.2}%", name, 100.0 * value / total);
                    plot_ui.text(Text::new(PlotPoint::new(center_x, center_y), label));

                    if hovered {
                        let pointer = plot_ui.pointer_coordinate().unwrap();
                        let label = format!(
                            "{} - {:.2}% ({:.prec$}/{:.prec$})",
                            name,
                            100.0 * fraction,
                            value,
                            total,
                            prec = self.decimal_precision
                        );

                        plot_ui.text(Text::new(pointer, RichText::new(label).heading()).name(name));
                    }
                }
            })
    }
}
