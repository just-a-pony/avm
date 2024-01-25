use avm_stats::{Plane, PlaneType, Spatial};
use egui::{vec2, Button, Checkbox, RichText, Ui};

use crate::app_state::AppState;
use crate::settings::{DistortionView, FrameSortOrder, MotionFieldColoring};
use crate::stream::{ChangeFrame, CurrentFrame};
use crate::views::RenderView;

use super::ViewMode;

const PLAYBACK_BUTTON_HEIGHT: f32 = 30.0;
const PLAYBACK_BUTTON_WIDTH: f32 = 30.0;
const PLAYBACK_BUTTON_FONT_SIZE: f32 = 18.0;

fn create_playback_button(icon: &str) -> Button {
    Button::new(RichText::new(format!(" {icon} ")).size(PLAYBACK_BUTTON_FONT_SIZE))
        .min_size(vec2(PLAYBACK_BUTTON_WIDTH, PLAYBACK_BUTTON_HEIGHT))
}

pub struct ControlsViewer;

impl RenderView for ControlsViewer {
    fn title(&self) -> String {
        "Controls".into()
    }

    fn render(&self, ui: &mut Ui, state: &mut AppState) -> anyhow::Result<()> {
        ui.horizontal(|ui| {
            ui.vertical(|ui| {
                ui.label("View");
                let frame_sort_options = [FrameSortOrder::Decode, FrameSortOrder::Display];
                egui::ComboBox::from_label("Frame sort order")
                    .selected_text(format!("{:?}", state.settings.persistent.frame_sort_order))
                    .show_ui(ui, |ui| {
                        for &sort_option in frame_sort_options.iter() {
                            ui.selectable_value(
                                &mut state.settings.persistent.frame_sort_order,
                                sort_option,
                                format!("{sort_option:?}"),
                            );
                        }
                    });
                ui.end_row();

                let plane_options = [
                    PlaneType::Planar(Plane::Y),
                    PlaneType::Planar(Plane::U),
                    PlaneType::Planar(Plane::V),
                    PlaneType::Rgb,
                ];

                egui::ComboBox::from_label("Plane view")
                    .selected_text(format!("{}", state.settings.sharable.current_plane))
                    .show_ui(ui, |ui| {
                        for &plane_option in plane_options.iter() {
                            ui.selectable_value(
                                &mut state.settings.sharable.current_plane,
                                plane_option,
                                format!("{plane_option}"),
                            );
                        }
                    });
                ui.end_row();

                ui.add_enabled(
                    state.settings.sharable.view_mode.view_settings().pixel_type.is_delta(),
                    egui::Checkbox::new(&mut state.settings.sharable.show_relative_delta, "Show relative delta"),
                );

                ui.end_row();

                ui.checkbox(&mut state.settings.sharable.show_overlay, "Show overlay");

                ui.end_row();

                ui.checkbox(&mut state.settings.sharable.show_yuv, "Show YUV");

                ui.end_row();

                if ui.button("Reset Zoom").clicked() {
                    if let Some(frame) = state.stream.current_frame() {
                        state.settings.sharable.world_bounds = frame.rect();
                    }
                }
            });
            if let Some(stream) = &mut state.stream {
                ui.separator();
                ui.vertical(|ui| {
                    ui.label("Playback");
                    ui.horizontal(|ui| {
                        let frame_sort_order: FrameSortOrder = state.settings.persistent.frame_sort_order;
                        if ui
                            .add(create_playback_button("⏮"))
                            .on_hover_text("Go to first frame")
                            .clicked()
                        {
                            stream.change_frame(ChangeFrame::first().order(frame_sort_order));
                        }
                        if ui
                            .add(create_playback_button("⏪"))
                            .on_hover_text("Go to previous frame")
                            .clicked()
                        {
                            stream.change_frame(ChangeFrame::prev().order(frame_sort_order));
                        }
                        let play_pause = if state.settings.playback.playback_running {
                            "⏸"
                        } else {
                            "▶"
                        };
                        if ui
                            .add(create_playback_button(play_pause))
                            .on_hover_text("Start/stop playback")
                            .clicked()
                        {
                            state.settings.playback.playback_running = !state.settings.playback.playback_running;
                        }

                        if ui
                            .add(create_playback_button("⏩"))
                            .on_hover_text("Go to next frame")
                            .clicked()
                        {
                            stream.change_frame(ChangeFrame::next().order(frame_sort_order));
                        }
                        if ui
                            .add(create_playback_button("⏭"))
                            .on_hover_text("Go to last frame")
                            .clicked()
                        {
                            stream.change_frame(ChangeFrame::last().order(frame_sort_order));
                        }
                    });
                    // TODO(comc): Re-enable after adding pause logic for pending loads.
                    // ui.checkbox(
                    //     &mut state.settings.playback.show_loaded_frames_only,
                    //     "Playback loaded frames only",
                    // );
                    ui.checkbox(&mut state.settings.playback.playback_loop, "Loop playback");
                    ui.label("Playback FPS:");
                    ui.add(egui::Slider::new(&mut state.settings.playback.playback_fps, 1.0..=60.0).step_by(1.0));
                });
            }

            if state.settings.sharable.view_mode == ViewMode::Heatmap {
                ui.separator();
                ui.vertical(|ui| {
                    ui.label("Heatmap");
                    ui.checkbox(&mut state.settings.sharable.show_heatmap_legend, "Show Heatmap Legend");
                    ui.end_row();
                    ui.label("Histogram buckets");
                    ui.add(egui::Slider::new(
                        &mut state.settings.sharable.heatmap_settings.histogram_buckets,
                        4..=100,
                    ));
                    ui.end_row();
                    ui.checkbox(
                        &mut state.settings.sharable.heatmap_histogram_log_scale,
                        "Histogram log scale",
                    );
                    ui.end_row();
                    ui.label("Symbol filter");
                    ui.text_edit_singleline(&mut state.settings.sharable.heatmap_settings.symbol_filter);
                    if ui.button("Recalculate heatmap").clicked() {
                        if let Some(stream) = &state.stream {
                            stream.images.clear_heatmaps();
                        }
                    }
                });
            }

            if let ViewMode::Distortion(mut distortion_view) = state.settings.sharable.view_mode {
                ui.separator();
                ui.vertical(|ui| {
                    let distortion_view_options = [
                        DistortionView::Distortion,
                        DistortionView::Original,
                        DistortionView::Reconstruction,
                    ];
                    egui::ComboBox::from_label("Displayed pixels")
                        .selected_text(distortion_view.name())
                        .show_ui(ui, |ui| {
                            for &distortion_view_option in distortion_view_options.iter() {
                                ui.selectable_value(
                                    &mut distortion_view,
                                    distortion_view_option,
                                    distortion_view_option.name(),
                                );
                            }
                        });
                    ui.end_row();
                });
                state.settings.sharable.view_mode = ViewMode::Distortion(distortion_view);
            }

            if state.settings.sharable.view_mode == ViewMode::Motion {
                ui.separator();
                ui.vertical(|ui| {
                    ui.label("Motion");
                    ui.checkbox(&mut state.settings.sharable.motion_field.show, "Show motion field");
                    ui.end_row();
                    ui.checkbox(
                        &mut state.settings.sharable.motion_field.show_origin,
                        "Show motion vector origin",
                    );
                    ui.end_row();
                    ui.add_enabled(
                        state.settings.sharable.motion_field.show,
                        Checkbox::new(
                            &mut state.settings.sharable.motion_field.normalize,
                            "Normalize vector lengths",
                        ),
                    );
                    ui.end_row();
                    ui.label("Motion vector scale:");
                    ui.add(egui::Slider::new(&mut state.settings.sharable.motion_field.scale, 0.1..=10.0).step_by(0.1));

                    let coloring_options = [
                        MotionFieldColoring::RefFrames,
                        MotionFieldColoring::PastFuture,
                        MotionFieldColoring::Monochrome,
                    ];
                    egui::ComboBox::from_label("Coloring scheme")
                        .selected_text(state.settings.sharable.motion_field.coloring.name())
                        .show_ui(ui, |ui| {
                            for &coloring_option in coloring_options.iter() {
                                ui.selectable_value(
                                    &mut state.settings.sharable.motion_field.coloring,
                                    coloring_option,
                                    coloring_option.name(),
                                );
                            }
                        });
                    ui.end_row();

                    ui.label("Granularity (in 4x4 units):");
                    ui.checkbox(&mut state.settings.sharable.motion_field.auto_granularity, "Automatic");
                    ui.add_enabled(
                        !state.settings.sharable.motion_field.auto_granularity,
                        egui::Slider::new(&mut state.settings.sharable.motion_field.granularity, 1..=32),
                    );
                    if let Some(frame) = state.stream.current_frame() {
                        if state.settings.sharable.motion_field.auto_granularity {
                            state.settings.sharable.motion_field.granularity = frame.height() as usize / 64;
                        }
                    }
                    ui.end_row();
                });
            }
        });
        Ok(())
    }
}
