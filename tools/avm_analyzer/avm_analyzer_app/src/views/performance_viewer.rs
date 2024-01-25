use egui::util::History;
use egui::Ui;

use crate::app_state::AppState;
use crate::views::render_view::RenderView;

pub struct PerformanceViewer;

pub struct PerformanceHistory {
    frame_times: History<f32>,
}

impl Default for PerformanceHistory {
    fn default() -> Self {
        Self {
            frame_times: History::new(2..100, 1.0),
        }
    }
}

impl PerformanceHistory {
    pub fn on_new_frame(&mut self, now: f64, previous_frame_time: Option<f32>) {
        let previous_frame_time = previous_frame_time.unwrap_or_default();
        if let Some(latest) = self.frame_times.latest_mut() {
            *latest = previous_frame_time;
        }
        self.frame_times.add(now, previous_frame_time);
    }

    pub fn mean_frame_time(&self) -> f32 {
        self.frame_times.average().unwrap_or_default()
    }

    pub fn fps(&self) -> f32 {
        1.0 / self.frame_times.mean_time_interval().unwrap_or_default()
    }

    pub fn ui(&mut self, ui: &mut egui::Ui) {
        ui.ctx().request_repaint();
        ui.label(format!("FPS: {:.1}", self.fps()));
        ui.label(format!(
            "Mean CPU usage: {:.2} ms / frame",
            1e3 * self.mean_frame_time()
        ));

        let mem_use = re_memory::MemoryUse::capture();
        if let Some(counted) = mem_use.counted {
            ui.label(format!("Memory usage: {}MiB", counted / 1024 / 1024));
        }
    }
}

impl RenderView for PerformanceViewer {
    fn title(&self) -> String {
        "Performance".into()
    }
    fn render(&self, ui: &mut Ui, state: &mut AppState) -> anyhow::Result<()> {
        state.performance_history.ui(ui);
        Ok(())
    }
}
