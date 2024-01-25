use avm_stats::{PixelType, PlaneType};
use convert_case::{Case, Casing};
use egui::{pos2, vec2, Align2, Color32, Mesh, Rect, Rounding, Shape, Stroke, TextStyle, Ui};

use crate::app_state::AppState;
use crate::image_manager::ImageType;
use crate::stream::{ChangeFrame, CurrentFrame, FrameStatus};

use crate::views::render_view::RenderView;

use super::frame_viewer::REF_FRAME_COLORS;
use super::{SelectedObjectKind, ViewMode};

// TODO(comc): There must be a built-in way to do horizontal scrolling with the mouse wheel in egui.
const SCROLL_FACTOR: f32 = 0.2;
pub struct FrameSelectViewer;
impl RenderView for FrameSelectViewer {
    fn title(&self) -> String {
        "Frame Select".into()
    }
    fn render(&self, ui: &mut Ui, state: &mut AppState) -> anyhow::Result<()> {
        let mut set_current_frame_index = None;
        let Some(stream) = state.stream.as_ref() else {
            return Ok(());
        };
        let scroll_to = state.settings.scroll_to_index.take();
        let mut motion_vector_arrows: Vec<(usize, i32)> = Vec::new();
        if state.settings.sharable.view_mode == ViewMode::Motion {
            if let Some(selected_object) = state.settings.selected_object.as_ref() {
                if let SelectedObjectKind::CodingUnit(cu) = selected_object.kind {
                    if let Some(frame) = stream.current_frame() {
                        if let Some(cu) = cu.try_resolve(frame) {
                            for mv in cu.coding_unit.prediction_mode.as_ref().unwrap().motion_vectors.iter() {
                                if mv.ref_frame_is_inter && !mv.ref_frame_is_tip {
                                    let order_hint = mv.ref_frame_order_hint;
                                    if let Some(frame_index) = stream.lookup_order_hint(order_hint) {
                                        if !motion_vector_arrows
                                            .iter()
                                            .any(|(other_frame_index, _)| *other_frame_index == frame_index)
                                        {
                                            motion_vector_arrows.push((frame_index, mv.ref_frame))
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        let sorted_frame_indices = stream.get_sorted_frames(state.settings.persistent.frame_sort_order);
        let mut frame_rects = vec![Rect::NOTHING; sorted_frame_indices.len()];
        egui::ScrollArea::horizontal()
            .max_height(200.0)
            .drag_to_scroll(true)
            .vscroll(false)
            .show(ui, |ui| {
                egui::Grid::new("img_grid").show(ui, |ui| {
                    for &frame_id in sorted_frame_indices.iter() {
                        let frame = stream.get_frame(frame_id);
                        let (id, rect) = ui.allocate_space(egui::vec2(100.0, 100.0));
                        frame_rects[frame_id] = rect;
                        if let Some(scroll_to) = scroll_to {
                            if scroll_to == frame_id {
                                ui.scroll_to_rect(rect, None);
                            }
                        }
                        let response = ui.interact(rect, id, egui::Sense::click());
                        let scroll_delta = ui.input(|i| i.scroll_delta);
                        if scroll_delta.length() > 0.0 {
                            let scroll_delta = SCROLL_FACTOR * vec2(scroll_delta.y, scroll_delta.x);
                            ui.scroll_with_delta(scroll_delta);
                        }
                        ui.painter()
                            .add(Shape::rect_filled(rect, Rounding::ZERO, Color32::GRAY));

                        let frame_id_text = if let FrameStatus::Loaded(frame) = frame {
                            let display_index = frame.frame_params.as_ref().unwrap().display_index;
                            format!("Frame {frame_id} ({display_index})")
                        } else {
                            format!("Frame {frame_id}")
                        };
                        let text = ui.fonts(|f| {
                            // TODO(comc): Use display or decode index instead of "frame_id":
                            Shape::text(
                                f,
                                rect.center_top(),
                                Align2::CENTER_TOP,
                                frame_id_text,
                                TextStyle::Body.resolve(ui.style()),
                                Color32::BLACK,
                            )
                        });
                        ui.painter().add(text);

                        match frame {
                            FrameStatus::OutOfBounds => {}
                            FrameStatus::Decoding => {
                                let text = ui.fonts(|f| {
                                    Shape::text(
                                        f,
                                        rect.center(),
                                        Align2::CENTER_CENTER,
                                        "Decoding...",
                                        TextStyle::Body.resolve(ui.style()),
                                        Color32::BLACK,
                                    )
                                });
                                ui.painter().add(text);
                            }
                            FrameStatus::Invalid => {
                                let text = ui.fonts(|f| {
                                    Shape::text(
                                        f,
                                        rect.center(),
                                        Align2::CENTER_CENTER,
                                        "ERROR",
                                        TextStyle::Body.resolve(ui.style()),
                                        Color32::BLACK,
                                    )
                                });
                                ui.painter().add(text);
                            }
                            FrameStatus::Pending => {
                                let text = ui.fonts(|f| {
                                    Shape::text(
                                        f,
                                        rect.center(),
                                        Align2::CENTER_CENTER,
                                        "Loading...",
                                        TextStyle::Body.resolve(ui.style()),
                                        Color32::BLACK,
                                    )
                                });
                                ui.painter().add(text);
                            }
                            FrameStatus::Loaded(frame) => {
                                // TODO(comc): Don't unwrap!
                                let image_type =
                                    ImageType::new(PlaneType::Rgb, PixelType::Reconstruction, false, false);
                                let Ok(texture_handle) = stream.images.get_or_create_image(
                                    ui.ctx(),
                                    &stream.pixel_data,
                                    frame,
                                    image_type,
                                    &state.settings.sharable.heatmap_settings,
                                ) else {
                                    return;
                                };
                                let mut image_mesh = Mesh::with_texture(texture_handle.id());
                                // TODO(comc): preserve aspect ratio.
                                let image_uv = Rect::from_min_size(pos2(0.0, 0.0), vec2(1.0, 1.0));
                                let mut image_rect = rect;
                                image_rect.set_top(rect.top() + 20.0);
                                image_rect.set_bottom(rect.bottom() - 20.0);
                                image_mesh.add_rect_with_uv(image_rect, image_uv, Color32::WHITE);
                                ui.painter().add(image_mesh);

                                if response.clicked() {
                                    set_current_frame_index = Some(frame_id);
                                }
                                let mut frame_type_name =
                                    frame.frame_type_name().trim_end_matches("_FRAME").to_case(Case::Pascal);
                                let tip_mode = frame.tip_mode_name();
                                if tip_mode == "TIP_FRAME_AS_OUTPUT" {
                                    frame_type_name = format!("{frame_type_name} (TIP)");
                                }

                                let text = ui.fonts(|f| {
                                    Shape::text(
                                        f,
                                        rect.center_bottom(),
                                        Align2::CENTER_BOTTOM,
                                        frame_type_name,
                                        TextStyle::Body.resolve(ui.style()),
                                        Color32::BLACK,
                                    )
                                });
                                ui.painter().add(text);
                            }
                            FrameStatus::Unloaded => {
                                let text = ui.fonts(|f| {
                                    Shape::text(
                                        f,
                                        rect.center(),
                                        Align2::CENTER_CENTER,
                                        "Click to load",
                                        TextStyle::Body.resolve(ui.style()),
                                        Color32::BLACK,
                                    )
                                });
                                ui.painter().add(text);

                                if response.clicked() {
                                    set_current_frame_index = Some(frame_id);
                                }
                            }
                        };

                        if let Some(current_frame) = state.stream.current_frame() {
                            if current_frame.decode_index() == frame_id {
                                ui.painter().add(Shape::rect_stroke(
                                    rect,
                                    Rounding::ZERO,
                                    Stroke::new(3.0, Color32::YELLOW),
                                ));
                            }
                        }
                    }
                });
                let painter = ui.painter();
                for (mv_index, &(frame_index, ref_frame)) in motion_vector_arrows.iter().enumerate() {
                    let start_rect = frame_rects[stream.current_frame_index];
                    let end_rect = frame_rects[frame_index];
                    let color_index = ref_frame as usize % 8;
                    let color = REF_FRAME_COLORS[color_index];
                    let pos_offset = vec2(0.0, mv_index as f32 * 5.0);
                    painter.add(Shape::line_segment(
                        [start_rect.center() + pos_offset, end_rect.center() + pos_offset],
                        Stroke::new(2.0, color),
                    ));
                    painter.add(Shape::circle_filled(
                        start_rect.center() + pos_offset,
                        2.0,
                        REF_FRAME_COLORS[color_index],
                    ));

                    let dir_sign = (start_rect.left() - end_rect.left()).signum();
                    let triangle_tip = end_rect.center() + pos_offset;
                    let triangle_vertices = vec![
                        triangle_tip,
                        triangle_tip + vec2(dir_sign * 3.0, 3.0),
                        triangle_tip + vec2(dir_sign * 3.0, -3.0),
                    ];
                    painter.add(Shape::convex_polygon(
                        triangle_vertices,
                        REF_FRAME_COLORS[color_index],
                        Stroke::NONE,
                    ));

                    painter.add(Shape::rect_stroke(end_rect, Rounding::ZERO, Stroke::new(3.0, color)));
                }
            });
        if let Some(current_frame_index) = set_current_frame_index {
            // Unwrapping is okay, since we already know stream exists at this point.
            let stream = state.stream.as_mut().unwrap();
            stream.change_frame(ChangeFrame::index(current_frame_index));
            state.settings.selected_object = None;
        }
        Ok(())
    }
}
