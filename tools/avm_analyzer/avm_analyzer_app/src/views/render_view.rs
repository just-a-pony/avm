use egui::Ui;

use crate::app_state::AppState;

pub trait RenderView {
    fn render(&self, ui: &mut Ui, state: &mut AppState) -> anyhow::Result<()>;
    fn title(&self) -> String;
}

impl PartialEq for dyn RenderView {
    fn eq(&self, other: &Self) -> bool {
        self.title() == other.title()
    }
}
