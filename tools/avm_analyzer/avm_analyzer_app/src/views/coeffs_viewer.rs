use avm_stats::MAX_COEFFS_SIZE;

use anyhow::anyhow;
use egui::{pos2, vec2, Align2, Color32, PointerButton, Rect, RichText, Rounding, Shape, Stroke, TextStyle, Ui, Vec2};
use itertools::{Itertools, MinMaxResult};

use crate::app_state::AppState;
use crate::settings::CoeffViewSelect;
use crate::views::RenderView;
use crate::views::{ScreenBounds, SelectedObjectKind};
use crate::views::{MIN_BLOCK_HEIGHT_FOR_TEXT, MIN_BLOCK_WIDTH_FOR_TEXT};

pub struct CoeffsViewer;

impl RenderView for CoeffsViewer {
    fn title(&self) -> String {
        "Coeffs View".into()
    }

    fn render(&self, ui: &mut Ui, state: &mut AppState) -> anyhow::Result<()> {
        let Some(stream) = &state.stream else {
            return Ok(());
        };

        let Some(frame) = stream.current_frame() else {
            return Ok(());
        };

        let Some(selected_object) = &state.settings.selected_object else {
            return Ok(());
        };

        let SelectedObjectKind::TransformUnit(transform_unit_index) = selected_object.kind.clone() else {
            return Ok(());
        };

        let transform_unit_ctx = transform_unit_index
            .try_resolve(frame)
            .ok_or(anyhow!("Invalid transform unit index"))?;
        let transform_unit = transform_unit_ctx.transform_unit;

        let coeff_view_options = [
            CoeffViewSelect::Dequantized,
            CoeffViewSelect::Quantized,
            CoeffViewSelect::DequantValue,
        ];

        ui.label(RichText::new("WARNING: Coeff data extraction is currently buggy.").color(Color32::RED));
        ui.end_row();

        egui::ComboBox::from_label("Coeff view")
            .selected_text(state.settings.sharable.coeff_view_select.name())
            .show_ui(ui, |ui| {
                for coeff_option in coeff_view_options.iter() {
                    ui.selectable_value(
                        &mut state.settings.sharable.coeff_view_select,
                        *coeff_option,
                        coeff_option.name(),
                    );
                }
            });
        ui.end_row();

        let Some(mut object_rect) = selected_object.rect(frame) else {
            return Err(anyhow!("Invalid selected object"));
        };
        if state.settings.sharable.current_plane.use_chroma() {
            object_rect = object_rect * 0.5;
        }
        let size = ui.available_size_before_wrap();
        let (screen_bounds, response) = ui.allocate_exact_size(size, egui::Sense::drag());
        let mut world_bounds = state.settings.sharable.coeffs_viewer_bounds;
        let scale = world_bounds.calc_scale(screen_bounds);
        let mut hover_pos_world = None;

        // TODO(comc): Factor out this common code with frame_viewer and detailed_pixel_viewer.
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
                hover_pos_world = Some(world_bounds.screen_pos_to_world(mouse_pos, screen_bounds));
            }
        }
        state.settings.sharable.coeffs_viewer_bounds = world_bounds;

        let painter = ui.painter().with_clip_rect(response.rect);
        let clip_rect = painter.clip_rect();
        let mut shapes = Vec::new();
        let ui_style = ui.ctx().style();
        let mut hover_text = None;
        let mut min_coeff = 0;
        let mut max_coeff = 255;
        let coeff_view = match state.settings.sharable.coeff_view_select {
            CoeffViewSelect::DequantValue => &transform_unit.dequantizer_values,
            CoeffViewSelect::Dequantized => &transform_unit.dequantized_coeffs,
            CoeffViewSelect::Quantized => &transform_unit.quantized_coeffs,
        };
        match coeff_view.iter().minmax() {
            MinMaxResult::NoElements | MinMaxResult::OneElement(_) => {}
            MinMaxResult::MinMax(&min_v, &max_v) => {
                min_coeff = min_v;
                max_coeff = max_v;
            }
        };

        // TODO(comc): Manually doing an inverse DCT does not yield the expected pixels for some blocks. Verify that the Residual = (PreFiltered - Prediction) logic makes sense.
        let coeffs_width = MAX_COEFFS_SIZE.min(object_rect.width() as usize);
        let coeffs_height = MAX_COEFFS_SIZE.min(object_rect.height() as usize);

        for y in 0..coeffs_height {
            for x in 0..coeffs_width {
                let stride = coeffs_width;
                let index = y * stride + x;

                let coeff = coeff_view.get(index).copied();
                let have_coeff = coeff.is_some();
                let coeff = coeff.unwrap_or_default();
                let color = (coeff - min_coeff) as f32 / (max_coeff - min_coeff) as f32;
                let world_rect = Rect::from_min_size(pos2(x as f32, y as f32), vec2(1.0, 1.0));
                let screen_rect = world_bounds.world_rect_to_screen(world_rect, clip_rect);
                if let Some(hover_pos_world) = hover_pos_world {
                    if world_rect.contains(hover_pos_world) && have_coeff {
                        hover_text = Some(format!("Coeff={} (x={}, y={})", coeff, x, y));
                    }
                }

                shapes.push(Shape::rect_filled(
                    screen_rect,
                    Rounding::ZERO,
                    Color32::from_gray((color * 255.0) as u8),
                ));
                shapes.push(Shape::rect_stroke(
                    screen_rect,
                    Rounding::ZERO,
                    Stroke::new(1.0, Color32::from_gray(20)),
                ));

                if have_coeff
                    && screen_rect.height() >= MIN_BLOCK_HEIGHT_FOR_TEXT
                    && screen_rect.width() >= MIN_BLOCK_WIDTH_FOR_TEXT
                {
                    let overlay_style = &state.settings.persistent.style.overlay;
                    shapes.extend(painter.fonts(|f| {
                        let colors = if overlay_style.enable_text_shadows {
                            vec![Color32::BLACK, overlay_style.pixel_viewer_text_color]
                        } else {
                            vec![overlay_style.pixel_viewer_text_color]
                        };
                        colors
                            .into_iter()
                            .enumerate()
                            .map(|(i, color)| {
                                let pos_offset = vec2(i as f32, i as f32);
                                Shape::text(
                                    f,
                                    screen_rect.center() + pos_offset,
                                    Align2::CENTER_CENTER,
                                    format!("{coeff}"),
                                    TextStyle::Body.resolve(&ui_style),
                                    color,
                                )
                            })
                            .collect::<Vec<_>>()
                    }));
                }
            }
        }
        painter.extend(shapes);
        if let Some(hover_text) = hover_text {
            response.on_hover_text_at_pointer(hover_text);
        }
        Ok(())
    }
}
