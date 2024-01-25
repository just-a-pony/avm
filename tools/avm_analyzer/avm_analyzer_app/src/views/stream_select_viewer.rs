use egui::load::Bytes;
use egui::{Image, ImageSource, RichText, Sense, Ui};
use egui_extras::{Column, TableBuilder};
use itertools::Itertools;

use crate::app_state::AppState;
use crate::stream::Stream;
use crate::views::render_view::RenderView;

pub struct StreamSelectViewer;

impl RenderView for StreamSelectViewer {
    fn title(&self) -> String {
        "Stream Select".into()
    }
    fn render(&self, ui: &mut Ui, state: &mut AppState) -> anyhow::Result<()> {
        if ui.button("Refresh stream list").clicked() {
            state.http_stream_manager.load_stream_list();
        }
        let mut set_stream = None;
        let streams = state.http_stream_manager.streams.lock().unwrap();
        TableBuilder::new(ui)
            .column(Column::initial(600.0).resizable(true))
            .column(Column::remainder().resizable(false).at_least(100.0))
            .sense(Sense::click())
            .striped(true)
            .header(20.0, |mut header| {
                header.col(|ui| {
                    ui.heading("Name");
                });
                header.col(|ui| {
                    ui.heading("Preview");
                });
            })
            .body(|body| {
                let streams: Vec<_> = streams
                    .iter()
                    .sorted_by_key(|stream| stream.stream_name.as_str())
                    .collect();
                let num_rows: usize = streams.len();
                body.rows(100.0, num_rows, |mut row| {
                    let stream_info = streams[row.index()];
                    row.col(|ui| {
                        let text = RichText::new(stream_info.stream_name.as_str()).heading();
                        ui.label(text);
                    });
                    row.col(|ui| {
                        if let Some(thumbnail_png) = &stream_info.thumbnail_png {
                            // TODO(comc): Make stream_info.thumbnail_png an Arc to avoid copy.
                            let thumbnail_png = thumbnail_png.clone();
                            let uri = format!("bytes://{}", stream_info.stream_name);
                            let image = Image::new(ImageSource::Bytes {
                                uri: uri.into(),
                                bytes: Bytes::Shared(thumbnail_png.into()),
                            });
                            ui.add(image);
                        }
                    });
                    if row.response().clicked() {
                        set_stream = Some(stream_info.clone());
                    }
                });
            });
        if let Some(set_stream) = set_stream {
            state.stream = Some(Stream::from_http(set_stream, false, 0, &state.settings.sharable.streams_url)?);
            state.settings.show_stream_select = false;
        }
        Ok(())
    }
}
