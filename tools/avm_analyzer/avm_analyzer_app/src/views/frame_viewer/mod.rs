mod frame_overlay;
mod screen_bounds;
mod selected_object;

use avm_stats::{Frame, Plane, PlaneType, Spatial};
use egui::{pos2, vec2, Color32, Mesh, PointerButton, Pos2, Rect, RichText, Rounding, Shape, Stroke, Ui, Vec2};
use egui_plot::{Bar, BarChart, Plot};

use crate::image_manager::ImageType;
use crate::stream::CurrentFrame as _;
use crate::views::RenderView;
use crate::{app_state::AppState, image_manager::JET_COLORMAP};
pub use frame_overlay::{FrameOverlay, REF_FRAME_COLORS};
pub use screen_bounds::ScreenBounds;
pub use selected_object::{SelectedObject, SelectedObjectKind};

use super::ViewMode;

pub struct FrameViewer;

impl FrameViewer {
    fn check_hovered_object(
        &self,
        state: &AppState,
        frame: &Frame,
        hover_pos_world: Pos2,
    ) -> Option<SelectedObjectKind> {
        let view_settings = state.settings.sharable.view_mode.view_settings();
        if view_settings.allow_transform_unit_selection {
            if let Some(hovered_transform_unit) = frame
                .iter_transform_units(state.settings.sharable.current_plane.to_plane())
                .find(|ctx| ctx.transform_unit.rect().contains(hover_pos_world))
            {
                return Some(SelectedObjectKind::TransformUnit(hovered_transform_unit.locator));
            }
        }
        if view_settings.allow_coding_unit_selection {
            let coding_unit_kind = frame.coding_unit_kind(state.settings.sharable.current_plane);
            if let Some(hovered_coding_unit) = frame
                .iter_coding_units(coding_unit_kind)
                .find(|ctx| ctx.coding_unit.rect().contains(hover_pos_world))
            {
                return Some(SelectedObjectKind::CodingUnit(hovered_coding_unit.locator));
            }
        }
        None
    }
}
impl RenderView for FrameViewer {
    fn title(&self) -> String {
        "Frame View".into()
    }

    fn render(&self, ui: &mut Ui, state: &mut AppState) -> anyhow::Result<()> {
        egui::warn_if_debug_build(ui);

        let Some(frame) = state.stream.current_frame() else {
            ui.spinner();
            return Ok(());
        };

        // Okay to unwrap here since we already have a Frame, so we must also have a Stream.
        let stream = state.stream.as_ref().unwrap();

        let mut world_bounds = state.settings.sharable.world_bounds;
        let mut set_selected_object = false;
        let mut set_selected_object_parent = false;
        let mut hover_pos_world = None;

        if state.settings.sharable.view_mode == ViewMode::Transform
            && matches!(
                state.settings.sharable.current_plane,
                PlaneType::Planar(Plane::U | Plane::V)
            )
        {
            ui.label(
                RichText::new("WARNING: Chroma transform tree data extraction is currently buggy.").color(Color32::RED),
            );
            ui.end_row();
        }

        let size = ui.available_size_before_wrap();
        let (screen_bounds, response) = ui.allocate_exact_size(size, egui::Sense::drag());
        let scale = world_bounds.calc_scale(screen_bounds);

        if response.dragged_by(PointerButton::Primary) {
            let delta = response.drag_delta() / -scale;
            world_bounds = world_bounds.translate(delta);
        } else if response.hover_pos().is_some() && ui.input(|i| i.scroll_delta) != Vec2::ZERO {
            let delta = ui.input(|i| i.scroll_delta);

            if let Some(mouse_pos) = ui.input(|i| i.pointer.hover_pos()) {
                if screen_bounds.contains(mouse_pos) {
                    let zoom = if delta.y < 0.0 {
                        f32::powf(1.001, -delta.y)
                    } else {
                        1.0 / (f32::powf(1.001, delta.y))
                    };
                    let zoom_center = world_bounds.screen_pos_to_world(mouse_pos, screen_bounds);
                    world_bounds.zoom_point(zoom_center, zoom);
                }
            }
        } else if response.hover_pos().is_some() {
            if let Some(mouse_pos) = ui.input(|i| i.pointer.hover_pos()) {
                if response.double_clicked() && state.settings.selected_object_leaf.is_some() {
                    set_selected_object_parent = true;
                } else if response.clicked() {
                    set_selected_object = true;
                }
                hover_pos_world = Some(world_bounds.screen_pos_to_world(mouse_pos, screen_bounds));
            }
        }

        state.settings.sharable.world_bounds = world_bounds;
        let width = frame.width();
        let height = frame.height();
        let view_settings = state.settings.sharable.view_mode.view_settings();

        let image_type = ImageType::new(
            state.settings.sharable.current_plane,
            view_settings.pixel_type,
            view_settings.pixel_type.is_delta() && state.settings.sharable.show_relative_delta,
            view_settings.show_heatmap,
        );

        let Ok(texture_handle) = stream.images.get_or_create_image(
            ui.ctx(),
            &stream.pixel_data,
            frame,
            image_type,
            &state.settings.sharable.heatmap_settings,
        ) else {
            return Ok(());
        };

        if state.settings.sharable.show_yuv {
            let mut image_mesh = Mesh::with_texture(texture_handle.id());
            let image_world = Rect::from_min_size(pos2(0.0, 0.0), vec2(width as f32, height as f32));
            let image_screen = world_bounds.world_rect_to_screen(image_world, screen_bounds);
            let image_uv = Rect::from_min_size(pos2(0.0, 0.0), vec2(1.0, 1.0));
            image_mesh.add_rect_with_uv(image_screen, image_uv, Color32::WHITE);
            ui.painter().with_clip_rect(response.rect).add(image_mesh);
        }
        let mut painter = ui.painter().with_clip_rect(response.rect);
        let overlay = FrameOverlay::new(frame, &state.settings);
        overlay.draw(&mut painter);

        let style = &state.settings.persistent.style.overlay;
        if state.settings.sharable.show_overlay {
            if let Some(hover_pos_world) = hover_pos_world {
                if let Some(hovered_object) = self.check_hovered_object(state, frame, hover_pos_world) {
                    if let Some(world_rect) = hovered_object.rect(frame) {
                        let screen_rect = world_bounds.world_rect_to_screen(world_rect, screen_bounds);
                        let mut already_selected = false;
                        if let Some(selected_object_leaf) = &state.settings.selected_object_leaf {
                            if selected_object_leaf == &hovered_object {
                                already_selected = true;
                            }
                        }
                        if set_selected_object && !already_selected {
                            state.settings.sharable.pixel_viewer_bounds =
                                Rect::from_min_size(pos2(0.0, 0.0), world_rect.size());
                            if matches!(hovered_object, SelectedObjectKind::TransformUnit(_)) {
                                state.settings.sharable.coeffs_viewer_bounds =
                                    Rect::from_min_size(pos2(0.0, 0.0), world_rect.size());
                            }
                            state.settings.selected_object_leaf = Some(hovered_object.clone());
                            state.settings.selected_object = Some(SelectedObject::new(hovered_object));
                        }
                        if set_selected_object_parent && already_selected {
                            if let Some(SelectedObject {
                                kind: selected_object_kind,
                                ..
                            }) = &state.settings.selected_object
                            {
                                if let Some(parent) = selected_object_kind.get_parent(frame) {
                                    state.settings.selected_object = Some(SelectedObject::new(parent));
                                }
                            }
                        }
                        painter.add(Shape::rect_stroke(
                            screen_rect,
                            Rounding::ZERO,
                            style.highlighted_object_stroke,
                        ));
                    }
                } else if set_selected_object {
                    state.settings.selected_object = None;
                    state.settings.selected_object_leaf = None;
                }
            }
            if let Some(selected_object) = &state.settings.selected_object {
                if let Some(world_rect) = selected_object.rect(frame) {
                    let screen_rect = world_bounds.world_rect_to_screen(world_rect, screen_bounds);
                    painter.add(Shape::rect_stroke(
                        screen_rect,
                        Rounding::ZERO,
                        style.selected_object_stroke,
                    ));
                }
            }
        }
        // TODO(comc): Refactor into separate module.
        if view_settings.show_heatmap {
            let mut show_heatmap_legend = state.settings.sharable.show_heatmap_legend;
            let log_scale = state.settings.sharable.heatmap_histogram_log_scale;
            let ui_ctx = ui.ctx();
            egui::Window::new("Heatmap Legend")
                .id("Heatmap Legend".into())
                .open(&mut show_heatmap_legend)
                .default_width(400.0)
                .default_height(400.0)
                .resizable(true)
                .collapsible(false)
                .show(ui_ctx, |ui| {
                    Plot::new("heatmap_legend")
                        .show_background(false)
                        .show_axes([true, true])
                        .clamp_grid(true)
                        .show_grid(false)
                        .allow_boxed_zoom(false)
                        .allow_drag(false)
                        .allow_zoom(false)
                        .allow_scroll(false)
                        .show_x(false)
                        .show_y(false)
                        .x_axis_label("Bits / pixel")
                        .y_axis_formatter(move |value, _num_chars, _range| {
                            if log_scale {
                                (10_f64).powf(value).to_string()
                            } else {
                                value.to_string()
                            }
                        })
                        .show(ui, |plot_ui| {
                            if let Ok(heatmap) = stream.images.get_or_create_heatmap(
                                ui_ctx,
                                frame,
                                image_type,
                                &state.settings.sharable.heatmap_settings,
                            ) {
                                let histogram = heatmap.histogram;
                                plot_ui.bar_chart(BarChart::new(
                                    histogram
                                        .iter()
                                        .enumerate()
                                        .map(|(index, &value)| {
                                            let mut value = value as f64;
                                            if log_scale && value > 0.0 {
                                                value = value.log10();
                                            }
                                            let bar_width = heatmap.bucket_width as f64;
                                            let x = index as f64 * bar_width;
                                            let colormap_index = 255.0 * (index as f64 / histogram.len() as f64);
                                            let color = JET_COLORMAP[colormap_index as usize];
                                            let fill = Color32::from_rgb(color[0], color[1], color[2]);
                                            Bar {
                                                name: "bar".into(),
                                                orientation: egui_plot::Orientation::Vertical,
                                                argument: x,
                                                value,
                                                base_offset: None,
                                                bar_width: heatmap.bucket_width as f64,
                                                stroke: Stroke::new(1.0, Color32::BLACK),
                                                fill,
                                            }
                                        })
                                        .collect(),
                                ));
                            }
                        });
                });
            state.settings.sharable.show_heatmap_legend = show_heatmap_legend;
        }
        Ok(())
    }
}
