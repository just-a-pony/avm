mod pie_plot;

use pie_plot::PiePlot;

use anyhow::{anyhow, Context};
use avm_stats::{FrameStatistic, StatSortMode};
use egui::Ui;
use log::warn;
use std::fmt::Write;
use wasm_bindgen::JsCast;
use web_sys::HtmlElement;

use crate::views::render_view::RenderView;
use crate::{app_state::AppState, stream::CurrentFrame};
pub struct StatsViewer;

// TODO(comc): Move to common location.
pub fn create_file_download(bytes: &[u8], file_name: &str) -> anyhow::Result<()> {
    let uint8arr = js_sys::Uint8Array::new(&unsafe { js_sys::Uint8Array::view(bytes) }.into());
    let array = js_sys::Array::new();
    array.push(&uint8arr.buffer());
    let blob = web_sys::Blob::new_with_u8_array_sequence_and_options(
        &array,
        web_sys::BlobPropertyBag::new().type_("application/octet-stream"),
    )
    .map_err(|_| anyhow!("Blob error"))?;
    let download_url = web_sys::Url::create_object_url_with_blob(&blob).map_err(|_| anyhow!("Blob error"))?;
    let window = web_sys::window().context("No window")?;
    let document = window.document().context("No document")?;
    let body = document.body().context("No body")?;
    let dummy = document.create_element("a").map_err(|_| anyhow!("Element error"))?;
    let dummy: HtmlElement = dummy
        .dyn_into::<web_sys::HtmlElement>()
        .map_err(|_| anyhow!("Element error"))?;
    body.append_child(&dummy).map_err(|_| anyhow!("Element error"))?;
    dummy
        .set_attribute("href", &download_url)
        .map_err(|_| anyhow!("Element error"))?;
    dummy
        .set_attribute("download", file_name)
        .map_err(|_| anyhow!("Element error"))?;
    dummy
        .set_attribute("style", "display: none")
        .map_err(|_| anyhow!("Element error"))?;
    dummy.click();
    web_sys::Url::revoke_object_url(&download_url).map_err(|_| anyhow!("Revoke URL error"))?;
    Ok(())
}

// TODO(comc): Scroll wheel zoom, export stats, bar chart mode, table mode.
impl RenderView for StatsViewer {
    fn title(&self) -> String {
        "Frame Stats".into()
    }

    fn render(&self, ui: &mut Ui, state: &mut AppState) -> anyhow::Result<()> {
        let Some(frame) = state.stream.current_frame() else {
            // No frame loaded yet.
            return Ok(());
        };
        let stats_settings = &mut state.settings.sharable.stats_settings;
        let prev_state = (state.settings.sharable.selected_stat, stats_settings.clone());

        let mut selected_stat = state.settings.sharable.selected_stat;
        egui::ComboBox::from_label("Statistic")
            .selected_text(selected_stat.name())
            .show_ui(ui, |ui| {
                for stat in &[
                    FrameStatistic::LumaModes,
                    FrameStatistic::ChromaModes,
                    FrameStatistic::BlockSizes,
                    FrameStatistic::Symbols,
                    FrameStatistic::PartitionSplit,
                ] {
                    ui.selectable_value(&mut selected_stat, *stat, stat.name());
                }
            });
        ui.end_row();
        state.settings.sharable.selected_stat = selected_stat;
        if matches!(selected_stat, FrameStatistic::PartitionSplit) {
            ui.horizontal(|ui| {
                ui.label("Block sizes:")
                    .on_hover_text("Comma separated list of block sizes, e.g. \"64x64,128x128\".");
                ui.text_edit_singleline(&mut stats_settings.partition_split_block_sizes);
                ui.end_row();
            });
        }

        let mut export_data = false;
        ui.horizontal(|ui| {
            // TODO(comc): Find a way to implement screenshot on web.
            // if ui.button("Save plot").clicked() {
            //     ui.ctx().send_viewport_cmd(egui::ViewportCommand::Screenshot);
            // }
            if ui.button("Export data").clicked() {
                export_data = true;
            }
        });
        let mut sort_by = stats_settings.sort_by;
        egui::ComboBox::from_label("Sort mode")
            .selected_text(sort_by.name())
            .show_ui(ui, |ui| {
                for sort_mode in &[StatSortMode::ByName, StatSortMode::ByValue] {
                    ui.selectable_value(&mut sort_by, *sort_mode, sort_mode.name());
                }
            });
        ui.end_row();
        stats_settings.sort_by = sort_by;

        let mut show_relative_total = stats_settings.show_relative_total;
        ui.checkbox(&mut show_relative_total, "Show relative total");
        ui.end_row();
        stats_settings.show_relative_total = show_relative_total;

        ui.horizontal(|ui| {
            let mut apply_limit_count = stats_settings.apply_limit_count;
            let mut limit_count = stats_settings.limit_count;
            ui.checkbox(&mut apply_limit_count, "Limit Top N");
            ui.add_enabled(apply_limit_count, egui::Slider::new(&mut limit_count, 1..=50));
            stats_settings.apply_limit_count = apply_limit_count;
            stats_settings.limit_count = limit_count;
        });
        ui.end_row();

        ui.horizontal(|ui| {
            let mut apply_limit_frac = stats_settings.apply_limit_frac;
            let mut limit_frac = stats_settings.limit_frac * 100.0;
            ui.checkbox(&mut apply_limit_frac, "Threshold Percent");
            ui.add_enabled(
                apply_limit_frac,
                egui::Slider::new(&mut limit_frac, 0.0..=50.0).step_by(0.1),
            );
            stats_settings.apply_limit_frac = apply_limit_frac;
            stats_settings.limit_frac = limit_frac / 100.0;
        });
        ui.end_row();

        ui.horizontal(|ui| {
            ui.label("Include:");
            let mut include_filter_exact_match = stats_settings.include_filter_exact_match;
            ui.checkbox(&mut include_filter_exact_match, "Exact match");
            ui.text_edit_singleline(&mut stats_settings.include_filter);
            stats_settings.include_filter_exact_match = include_filter_exact_match;
        });
        ui.end_row();

        ui.horizontal(|ui| {
            ui.label("Exclude:");
            let mut exclude_filter_exact_match = stats_settings.exclude_filter_exact_match;
            ui.checkbox(&mut exclude_filter_exact_match, "Exact match");
            ui.text_edit_singleline(&mut stats_settings.exclude_filter);
            stats_settings.exclude_filter_exact_match = exclude_filter_exact_match;
        });
        ui.end_row();

        let decimal_precision = match selected_stat {
            FrameStatistic::LumaModes => 0,
            FrameStatistic::ChromaModes => 0,
            FrameStatistic::BlockSizes => 0,
            FrameStatistic::Symbols => 2,
            FrameStatistic::PartitionSplit => 0,
        };

        let pie_plot = PiePlot {
            decimal_precision,
            ..Default::default()
        };

        let changed = state.settings.sharable.selected_stat != prev_state.0 || *stats_settings != prev_state.1;
        let calculate_data = changed || state.settings.cached_stat_data.is_none();

        if calculate_data {
            state.settings.cached_stat_data = Some(selected_stat.calculate(frame, stats_settings));
        }
        let _resp = pie_plot.show(ui, state.settings.cached_stat_data.as_ref().unwrap());
        if export_data {
            if let Some(stream) = &state.stream {
                let file_name = format!(
                    "{}_frame_{:04}_{}.csv",
                    stream.stream_info.stream_name,
                    stream.current_frame_index,
                    selected_stat.name().replace(' ', "_")
                );

                let data = state.settings.cached_stat_data.as_ref().unwrap();
                let total: f64 = data.iter().map(|sample| sample.value).sum();
                let header = format!("{},Count,Percent\n", selected_stat.name());
                let csv: String = data.iter().fold(String::new(), |mut output, sample| {
                    let _ = writeln!(output, "{},{},{}", sample.name, sample.value, sample.value / total);
                    output
                });
                let mut bytes: Vec<u8> = header.bytes().collect();
                bytes.extend(csv.bytes());

                if let Err(err) = create_file_download(&bytes, &file_name) {
                    warn!("Failed to create file download: {err:?}");
                }
            }
        }

        // TODO(comc): Find a way to implement screenshot on web.
        // let screenshot = ui.ctx().input(|i| {
        //     for event in &i.raw.events {
        //         if let egui::Event::Screenshot { image, .. } = event {
        //             return Some(image.clone());
        //         }
        //     }
        //     None
        // });

        // if let Some(screenshot) = screenshot {
        //     let mut bytes = Vec::new();
        //     let png_encoder = image::codecs::png::PngEncoder::new(Cursor::new(&mut bytes));
        //     let pixels_per_point = ui.ctx().pixels_per_point();
        //     let plot = screenshot.region(&resp.response.rect, Some(pixels_per_point));
        //     match png_encoder.write_image(
        //         plot.as_raw(),
        //         plot.width() as u32,
        //         plot.height() as u32,
        //         image::ColorType::Rgba8,
        //     ) {
        //         Ok(_) => {
        //             if let Err(err) = create_file_download(&bytes, "test.png") {
        //                 warn!("Failed to create file download: {err:?}");
        //             }
        //         }
        //         Err(err) => {
        //             warn!("Failed to encode PNG: {err:?}");
        //         }
        //     }
        // }

        Ok(())
    }
}
