use super::ScreenBounds;
use crate::settings::{MotionFieldColoring, Settings};
use crate::views::{MIN_BLOCK_HEIGHT_FOR_TEXT, MIN_BLOCK_SIZE_TO_RENDER, MIN_BLOCK_WIDTH_FOR_TEXT};

use avm_stats::{CodingUnitKind, Frame, ProtoEnumMapping, Spatial, MOTION_VECTOR_PRECISION};
use egui::{pos2, vec2, Align2, Color32, Painter, Rect, Rounding, Shape, Stroke, TextStyle};
use itertools::Itertools;

fn is_rect_too_small(rect: Rect) -> bool {
    rect.width() < MIN_BLOCK_SIZE_TO_RENDER || rect.height() < MIN_BLOCK_SIZE_TO_RENDER
}

fn is_text_too_small(rect: Rect) -> bool {
    rect.width() < MIN_BLOCK_WIDTH_FOR_TEXT || rect.height() < MIN_BLOCK_HEIGHT_FOR_TEXT
}

// TODO(comc): Make configurable.
pub const REF_FRAME_COLORS: &[Color32] = &[
    Color32::LIGHT_RED,
    Color32::LIGHT_GREEN,
    Color32::LIGHT_BLUE,
    Color32::LIGHT_YELLOW,
    Color32::BROWN,
    Color32::KHAKI,
    Color32::GOLD,
    Color32::LIGHT_GRAY,
];

// TODO(comc): World space to screen space conversion could be done in a shader.
pub struct FrameOverlay<'a> {
    frame: &'a Frame,
    settings: &'a Settings,
}

impl<'a> FrameOverlay<'a> {
    pub fn new(frame: &'a Frame, settings: &'a Settings) -> Self {
        Self { frame, settings }
    }

    pub fn draw(&self, painter: &mut Painter) {
        let view_settings = self.settings.sharable.view_mode.view_settings();

        if self.settings.sharable.show_overlay {
            if view_settings.show_transform_units {
                self.draw_transform_units(painter);
            }

            if view_settings.show_coding_units {
                self.draw_coding_units(painter);
            }

            if view_settings.show_superblocks {
                self.draw_superblocks(painter);
            }

            if view_settings.show_prediction_modes {
                self.draw_prediction_modes(painter);
            }

            if view_settings.show_transform_types {
                self.draw_transform_modes(painter);
            }
        }

        if (self.settings.sharable.show_overlay || self.settings.sharable.motion_field.show)
            && view_settings.show_motion_vectors
        {
            self.draw_motion_vectors(painter);
        }
    }

    fn is_in_bounds(&self, world_rect: Rect, screen_bounds: Rect) -> bool {
        let world_bounds = self.settings.sharable.world_bounds;
        // Because the aspect ratio of the viewport is not necessarily the same as the current frame, we might see extra than the plain world_bounds.
        let extended_world_bounds = world_bounds.screen_rect_to_world(screen_bounds, screen_bounds);
        extended_world_bounds.intersects(world_rect)
    }

    fn draw_transform_units(&self, painter: &mut Painter) -> Option<()> {
        let style = &self.settings.persistent.style.overlay;
        let world_bounds = self.settings.sharable.world_bounds;
        let clip_rect = painter.clip_rect();
        let shapes = self
            .frame
            .iter_transform_rects(self.settings.sharable.current_plane.to_plane())
            .filter_map(|world_rect| {
                if !self.is_in_bounds(world_rect, clip_rect) {
                    return None;
                }
                let screen_rect = world_bounds.world_rect_to_screen(world_rect, clip_rect);
                if is_rect_too_small(screen_rect) {
                    return None;
                }
                let points = [
                    screen_rect.left_top(),
                    screen_rect.right_top(),
                    screen_rect.right_bottom(),
                    screen_rect.left_bottom(),
                ];
                Some(Shape::dashed_line(&points[..], style.transform_unit_stroke, 4.0, 4.0))
            })
            .flatten();
        painter.extend(shapes);

        None
    }
    fn draw_coding_units(&self, painter: &mut Painter) -> Option<()> {
        let style = &self.settings.persistent.style.overlay;
        let world_bounds = self.settings.sharable.world_bounds;
        let clip_rect = painter.clip_rect();
        let coding_unit_kind = self.frame.coding_unit_kind(self.settings.sharable.current_plane);
        let shapes = self
            .frame
            .iter_coding_unit_rects(coding_unit_kind)
            .filter_map(|world_rect| {
                if !self.is_in_bounds(world_rect, clip_rect) {
                    return None;
                }
                let screen_rect = world_bounds.world_rect_to_screen(world_rect, clip_rect);
                if is_rect_too_small(screen_rect) {
                    return None;
                }
                Some(Shape::rect_stroke(
                    screen_rect,
                    Rounding::ZERO,
                    style.coding_unit_stroke,
                ))
            });
        painter.extend(shapes);
        None
    }
    fn draw_motion_vectors(&self, painter: &mut Painter) -> Option<()> {
        let _style = &self.settings.persistent.style.overlay;
        let world_bounds = self.settings.sharable.world_bounds;
        let clip_rect = painter.clip_rect();
        let coding_unit_kind = self.frame.coding_unit_kind(self.settings.sharable.current_plane);
        // Length of largest motion vector, in screen space.
        let mut largest_mv = 1e-9;
        let granularity = self.settings.sharable.motion_field.granularity;
        let raw_shapes = self
            .frame
            .iter_coding_units(coding_unit_kind)
            .filter_map(|ctx| {
                let cu = ctx.coding_unit;
                let world_rect = cu.rect();
                let screen_rect = world_bounds.world_rect_to_screen(world_rect, clip_rect);
                let Ok(prediction_mode) = cu.get_prediction_mode() else {
                    return None;
                };
                let Ok(mode) = self
                    .frame
                    .enum_lookup(ProtoEnumMapping::PredictionMode, prediction_mode.mode)
                else {
                    return None;
                };
                // TODO(comc): Make this a method on PredictionParams or CodingUnit.
                let is_motion = !mode.contains("_PRED");
                if !is_motion {
                    return None;
                }
                let is_compound = mode.contains('_');
                let num_mvs = if is_compound { 2 } else { 1 };
                // let mv_center = world_rect.center();
                let mvs = (0..num_mvs)
                    .filter_map(|i| {
                        let mv = prediction_mode.motion_vectors.get(i as usize)?;
                        let dx = mv.dx as f32 / MOTION_VECTOR_PRECISION;
                        let dy = mv.dy as f32 / MOTION_VECTOR_PRECISION;
                        let ref_frame = mv.ref_frame;
                        if ref_frame == -1 {
                            return None;
                        }

                        let order_hint = mv.ref_frame_order_hint;
                        // let mv_tip = mv_center + vec2(dx, dy);
                        let mv_world = vec2(dx, dy);
                        let mv_screen = mv_world * world_bounds.calc_scale(clip_rect);
                        // let mv_tip_screen = world_bounds.world_pos_to_screen(mv_tip, clip_rect);
                        // let screen_pos = screen_rect.center();
                        // let mv_vector = mv_tip_screen - screen_pos;
                        let magnitude = mv_screen.length();
                        if magnitude > largest_mv {
                            largest_mv = magnitude;
                        }
                        let is_future = order_hint > self.frame.frame_params.as_ref().unwrap().raw_display_index;
                        let color = match self.settings.sharable.motion_field.coloring {
                            MotionFieldColoring::RefFrames => {
                                let color_index = ref_frame as usize % 8;
                                REF_FRAME_COLORS[color_index]
                            }
                            MotionFieldColoring::PastFuture => REF_FRAME_COLORS[is_future as usize],
                            MotionFieldColoring::Monochrome => REF_FRAME_COLORS[0],
                        };
                        Some((mv_screen, color))
                    })
                    .collect_vec();

                if !self.is_in_bounds(world_rect, clip_rect) {
                    return None;
                }
                if is_rect_too_small(screen_rect) {
                    return None;
                }
                if mvs.is_empty() {
                    return None;
                }
                let mv_rects = if self.settings.sharable.motion_field.show {
                    let mut rects = Vec::new();
                    let rows = world_rect.height() as i32;
                    let cols = world_rect.width() as i32;
                    let top = world_rect.top() as i32;
                    let left = world_rect.left() as i32;
                    let granularity_pixels = (granularity * 4) as i32;
                    let first_row = (granularity_pixels / 2 - top).rem_euclid(granularity_pixels);
                    let first_col = (granularity_pixels / 2 - left).rem_euclid(granularity_pixels);
                    for row in (first_row..rows).step_by(granularity_pixels as usize) {
                        let y = row + top;
                        for col in (first_col..cols).step_by(granularity_pixels as usize) {
                            let x = col + left;
                            let rect = Rect::from_center_size(
                                pos2(x as f32, y as f32),
                                vec2(granularity_pixels as f32, granularity_pixels as f32),
                            );
                            let screen_rect = world_bounds.world_rect_to_screen(rect, clip_rect);
                            rects.push(screen_rect);
                        }
                    }
                    rects
                } else {
                    vec![screen_rect]
                };

                Some((mv_rects, mvs))
            })
            .collect_vec();

        let longest_vector_world_space = granularity as f32 * 2.0;
        let longest_vector_screen_space = world_bounds.calc_scale(clip_rect) * longest_vector_world_space;
        let normalization_factor = longest_vector_screen_space / largest_mv;
        let shapes = raw_shapes.iter().flat_map(|(rects, mvs)| {
            let mut mv_shapes = Vec::new();
            for screen_rect in rects {
                for &(mut mv_vector, color) in mvs.iter() {
                    if self.settings.sharable.motion_field.normalize && self.settings.sharable.motion_field.show {
                        mv_vector *= normalization_factor;
                    }
                    mv_vector *= self.settings.sharable.motion_field.scale;
                    let screen_pos = screen_rect.center();
                    if self.settings.sharable.motion_field.show_origin {
                        mv_shapes.push(Shape::circle_filled(screen_pos, 1.0, color));
                    }
                    mv_shapes.push(Shape::line_segment(
                        [screen_pos, screen_pos + mv_vector],
                        Stroke::new(1.5, color),
                    ));
                }
            }
            mv_shapes
        });
        painter.extend(shapes);
        None
    }

    fn draw_superblocks(&self, painter: &mut Painter) -> Option<()> {
        let style = &self.settings.persistent.style.overlay;
        let world_bounds = self.settings.sharable.world_bounds;
        let clip_rect = painter.clip_rect();
        let shapes = self.frame.iter_superblock_rects().filter_map(|world_rect| {
            if !self.is_in_bounds(world_rect, clip_rect) {
                return None;
            }
            let screen_rect = world_bounds.world_rect_to_screen(world_rect, clip_rect);
            if is_rect_too_small(screen_rect) {
                return None;
            }
            Some(Shape::rect_stroke(screen_rect, Rounding::ZERO, style.superblock_stroke))
        });
        painter.extend(shapes);
        None
    }

    fn draw_prediction_modes(&self, painter: &mut Painter) -> Option<()> {
        let overlay_style = &self.settings.persistent.style.overlay;
        let world_bounds = self.settings.sharable.world_bounds;
        let clip_rect = painter.clip_rect();
        let coding_unit_kind = self.frame.coding_unit_kind(self.settings.sharable.current_plane);
        let ui_style = painter.ctx().style();
        let shapes = self
            .frame
            .iter_coding_units(coding_unit_kind)
            .filter_map(|ctx| {
                let cu = ctx.coding_unit;
                let world_rect = cu.rect();
                if !self.is_in_bounds(world_rect, clip_rect) {
                    return None;
                }
                let screen_rect = world_bounds.world_rect_to_screen(world_rect, clip_rect);
                if is_rect_too_small(screen_rect) || is_text_too_small(screen_rect) {
                    return None;
                }

                let Ok(prediction_mode) = cu.get_prediction_mode() else {
                    return None;
                };
                let mode_name = if coding_unit_kind == CodingUnitKind::ChromaOnly {
                    self.frame
                        .enum_lookup(ProtoEnumMapping::UvPredictionMode, prediction_mode.uv_mode)
                } else {
                    self.frame
                        .enum_lookup(ProtoEnumMapping::PredictionMode, prediction_mode.mode)
                };
                let Ok(mode_name) = mode_name else {
                    return None;
                };

                let screen_pos = screen_rect.center();
                let mode_name = mode_name.trim_end_matches("_PRED");
                let uses_palette = match coding_unit_kind {
                    CodingUnitKind::Shared | CodingUnitKind::LumaOnly => {
                        prediction_mode.palette_count > 0
                    }
                    CodingUnitKind::ChromaOnly => prediction_mode.uv_palette_count > 0,
                };
                let uses_intrabc = match coding_unit_kind {
                    CodingUnitKind::Shared | CodingUnitKind::LumaOnly => {
                        prediction_mode.use_intrabc
                    }
                    CodingUnitKind::ChromaOnly => false
                };
                let mode_name = if uses_palette {
                    "PALETTE"
                } else if uses_intrabc {
                    "INTRA_BC"
                } else {
                    mode_name
                };

                let colors = if overlay_style.enable_text_shadows {
                    vec![Color32::BLACK, overlay_style.mode_name_color]
                } else {
                    vec![overlay_style.mode_name_color]
                };
                painter.fonts(|f| {
                    Some(
                        colors
                            .into_iter()
                            .enumerate()
                            .map(|(i, color)| {
                                let pos_offset = vec2(i as f32, i as f32);
                                Shape::text(
                                    f,
                                    screen_pos + pos_offset,
                                    Align2::CENTER_CENTER,
                                    mode_name,
                                    TextStyle::Body.resolve(&ui_style),
                                    color,
                                )
                            })
                            .collect::<Vec<_>>(),
                    )
                })
            })
            .flatten()
            .collect::<Vec<_>>(); // Note: need to collect here since painter.fonts requires read-only access to painter, and we can't mutate in painter.extend at the same time.

        painter.extend(shapes);
        None
    }

    fn draw_transform_modes(&self, painter: &mut Painter) -> Option<()> {
        let overlay_style = &self.settings.persistent.style.overlay;
        let world_bounds = self.settings.sharable.world_bounds;
        let clip_rect = painter.clip_rect();
        let ui_style = painter.ctx().style();
        let shapes = self
            .frame
            .iter_transform_units(self.settings.sharable.current_plane.to_plane())
            .filter_map(|ctx| {
                let tu = ctx.transform_unit;
                let world_rect = tu.rect();
                if !self.is_in_bounds(world_rect, clip_rect) {
                    return None;
                }
                let screen_rect = world_bounds.world_rect_to_screen(world_rect, clip_rect);
                if is_rect_too_small(screen_rect) || is_text_too_small(screen_rect) {
                    return None;
                }

                let tx_type = tu.primary_tx_type_or_skip(self.frame);
                let screen_pos = screen_rect.center();
                let colors = if overlay_style.enable_text_shadows {
                    vec![Color32::BLACK, overlay_style.mode_name_color]
                } else {
                    vec![overlay_style.mode_name_color]
                };
                painter.fonts(|f| {
                    Some(
                        colors
                            .into_iter()
                            .enumerate()
                            .map(|(i, color)| {
                                let pos_offset = vec2(i as f32, i as f32);
                                Shape::text(
                                    f,
                                    screen_pos + pos_offset,
                                    Align2::CENTER_CENTER,
                                    tx_type.clone(),
                                    TextStyle::Body.resolve(&ui_style),
                                    color,
                                )
                            })
                            .collect::<Vec<_>>(),
                    )
                })
            })
            .flatten()
            .collect::<Vec<_>>(); // Note: need to collect here since painter.fonts requires read-only access to painter, and we can't mutate in painter.extend at the same time.

        painter.extend(shapes);
        None
    }
}
