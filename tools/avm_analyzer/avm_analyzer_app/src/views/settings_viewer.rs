use egui::{TextEdit, Ui};

use crate::app_state::AppState;
use crate::settings::PersistentSettings;
use crate::views::render_view::RenderView;

pub struct SettingsViewer;

impl RenderView for SettingsViewer {
    fn title(&self) -> String {
        "Settings".into()
    }
    fn render(&self, ui: &mut Ui, state: &mut AppState) -> anyhow::Result<()> {
        let settings = &mut state.settings;
        egui::ScrollArea::vertical().show(ui, |ui| {
            if ui.button("Reset to defaults").clicked() {
                settings.persistent = PersistentSettings::default();
            }
            ui.label("AVM source root URL:");
            let text_edit = TextEdit::singleline(&mut settings.persistent.avm_source_url).desired_width(800.0);
            text_edit.show(ui);

            ui.horizontal(|ui| {
                let mut apply_cache_strategy = settings.persistent.apply_cache_strategy;
                let mut cache_strategy_limit = settings.persistent.cache_strategy_limit;
                ui.checkbox(&mut apply_cache_strategy, "Limit number of frames kept in memory");
                ui.add_enabled(
                    apply_cache_strategy,
                    egui::Slider::new(&mut cache_strategy_limit, 1..=100),
                );
                settings.persistent.apply_cache_strategy = apply_cache_strategy;
                settings.persistent.cache_strategy_limit = cache_strategy_limit;
            });

            ui.checkbox(
                &mut settings.persistent.update_sharable_url,
                "Update URL with sharable state",
            );

            let style = &mut settings.persistent.style.overlay;
            ui.horizontal(|ui| {
                ui.label("Highlighted object color:");
                ui.color_edit_button_srgba(&mut style.highlighted_object_stroke.color);
            });

            ui.horizontal(|ui| {
                ui.label("Selected object color:");
                ui.color_edit_button_srgba(&mut style.selected_object_stroke.color);
            });

            ui.horizontal(|ui| {
                ui.label("Coding unit color:");
                ui.color_edit_button_srgba(&mut style.coding_unit_stroke.color);
            });

            ui.horizontal(|ui| {
                ui.label("Transform unit color:");
                ui.color_edit_button_srgba(&mut style.transform_unit_stroke.color);
            });

            ui.horizontal(|ui| {
                ui.label("Superblock color:");
                ui.color_edit_button_srgba(&mut style.superblock_stroke.color);
            });

            ui.horizontal(|ui| {
                ui.label("Mode name color:");
                ui.color_edit_button_srgba(&mut style.mode_name_color);
            });

            ui.horizontal(|ui| {
                ui.label("Pixel / coeffs viewer text color:");
                ui.color_edit_button_srgba(&mut style.pixel_viewer_text_color);
            });

            ui.checkbox(&mut style.enable_text_shadows, "Enable text shadows");
        });
        Ok(())
    }
}
