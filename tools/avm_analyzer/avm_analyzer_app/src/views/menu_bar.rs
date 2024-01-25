use egui::{RichText, Ui};

use crate::app_state::AppState;
use crate::settings::DistortionView;
use crate::views::RenderView;
use crate::views::ViewMode;

pub struct MenuBar;

impl RenderView for MenuBar {
    fn title(&self) -> String {
        "Menu".into()
    }
    fn render(&self, ui: &mut Ui, state: &mut AppState) -> anyhow::Result<()> {
        egui::menu::bar(ui, |ui| {
            ui.menu_button("File", |ui| {
                if ui.button("Open Local Stream (.zip)").clicked() {
                    state.local_stream_manager.prompt_local_stream();
                    ui.close_menu();
                }
                if state.settings.sharable.show_remote_streams {
                    if ui.button("Open Remote Stream").clicked() {
                        state.settings.show_stream_select = true;
                        ui.close_menu();
                    }
                    if ui.button("Decode Stream on Server").clicked() {
                        state.settings.show_decode_progress = true;
                        state.server_decode_manager.prompt_stream();
                        ui.close_menu();
                    }
                }
                if ui.button("Open Demo Stream").clicked() {
                    state.local_stream_manager.load_demo_stream();
                    ui.close_menu();
                }
            });
            ui.menu_button("View Mode", |ui| {
                let current_mode = state.settings.sharable.view_mode;
                let mut have_orig_yuv = false;
                if let Some(stream) = state.stream.as_ref() {
                    if stream.have_orig_yuv() {
                        have_orig_yuv = true;
                    }
                }

                for view_mode in [
                    ViewMode::CodingFlow,
                    ViewMode::Prediction,
                    ViewMode::Transform,
                    ViewMode::Filters,
                    ViewMode::Distortion(DistortionView::Distortion),
                    ViewMode::Motion,
                    ViewMode::Heatmap,
                ] {
                    let prefix = if current_mode == view_mode { "• " } else { "  " };
                    let mut text = RichText::new(format!("{}{}", prefix, view_mode));
                    if current_mode == view_mode {
                        text = text.strong();
                    }
                    let enabled = !matches!(view_mode, ViewMode::Distortion(_)) || have_orig_yuv;
                    if ui.add_enabled(enabled, egui::Button::new(text)).clicked() {
                        state.settings.sharable.view_mode = view_mode;
                        ui.close_menu();
                    }
                }
            });

            ui.menu_button("Settings", |ui| {
                if ui.button("Edit Settings").clicked() {
                    state.settings.show_settings_window = true;
                    ui.close_menu();
                }
            });

            ui.menu_button("Window", |ui| {
                if ui.button("Decode Progress").clicked() {
                    state.settings.show_decode_progress = true;
                    ui.close_menu();
                }

                if ui.button("Performance").clicked() {
                    state.settings.show_performance_window = true;
                    ui.close_menu();
                }
            });
        });
        Ok(())
    }
}
