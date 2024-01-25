use avm_analyzer_common::DecodeState;
use egui::{Color32, ProgressBar, Ui};
use egui_extras::{Column, TableBuilder};
use itertools::Itertools;

use crate::app_state::AppState;
use crate::stream::Stream;
use crate::views::render_view::RenderView;

pub struct DecodeProgressViewer;

impl RenderView for DecodeProgressViewer {
    fn title(&self) -> String {
        "Decode Progress".into()
    }
    fn render(&self, ui: &mut Ui, state: &mut AppState) -> anyhow::Result<()> {
        let mut pending_decodes = state.server_decode_manager.pending_decodes.lock().unwrap();
        if ui.button("Clear").clicked() {
            pending_decodes.retain(|_k, v| !matches!(v.state, DecodeState::Complete(_) | DecodeState::Failed));
        }
        TableBuilder::new(ui)
            .column(Column::initial(200.0).resizable(true))
            .column(Column::initial(200.0).resizable(true))
            .striped(true)
            .header(20.0, |mut header| {
                header.col(|ui| {
                    ui.heading("Stream");
                });
                header.col(|ui| {
                    ui.heading("Status");
                });
            })
            .body(|body| {
                let sorted_pending_decodes: Vec<_> = pending_decodes.values().sorted_by_key(|p| p.start_time).collect();
                let num_rows: usize = sorted_pending_decodes.len();
                body.rows(20.0, num_rows, |mut row| {
                    let stream_info = &sorted_pending_decodes[row.index()].stream_info;
                    let stream_name = &stream_info.stream_name;
                    let decode_state = &sorted_pending_decodes[row.index()].state;
                    let progress_bar = match decode_state {
                        DecodeState::Complete(_) => ProgressBar::new(1.0).fill(Color32::LIGHT_GREEN).text("Finished"),
                        DecodeState::Failed => ProgressBar::new(1.0).fill(Color32::RED).text("FAILED"),
                        DecodeState::Pending(progress) => {
                            let percent = progress.decoded_frames as f32 / progress.total_frames as f32;
                            let text = format!(
                                "{}/{} ({:.0}%)",
                                progress.decoded_frames,
                                progress.total_frames,
                                percent * 100.0
                            );
                            ProgressBar::new(percent).text(text).animate(true)
                        }
                        DecodeState::UploadComplete => {
                            let text = format!("0/{} (0%)", stream_info.num_frames);
                            ProgressBar::new(1.0).text(text).animate(true)
                        }
                        DecodeState::Uploading => ProgressBar::new(1.0).fill(Color32::GRAY).text("Uploading"),
                    };
                    row.col(|ui| {
                        ui.horizontal(|ui| {
                            // TODO(comc): We could load pending streams too, as stream_select_viewer allows.
                            let ready_to_load = matches!(decode_state, DecodeState::Complete(_));
                            if ui.add_enabled(ready_to_load, egui::Button::new("Load")).clicked() {
                                state.stream = Some(Stream::from_http(stream_info.clone(), false, 0, &state.settings.sharable.streams_url).unwrap());
                            }

                            ui.label(stream_name);
                        });
                    });
                    row.col(|ui| {
                        ui.add(progress_bar);
                    });
                });
            });
        Ok(())
    }
}
