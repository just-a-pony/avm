use egui::{Align2, Color32, Id, LayerId, Order, TextStyle};
use itertools::Itertools;
use log::warn;

use crate::app_state::AppState;

const KNOWN_AVM_STREAM_EXTENSIONS: &[&str] = &[".obu", ".bin", ".ivf"];
const KNOWN_ZIP_EXTENSIONS: &[&str] = &[".zip"];

const ZIP_MIME_TYPES: &[&str] = &["application/zip"];
const AVM_STREAM_MIME_TYPES: &[&str] = &["application/macbinary", "application/octet-stream"];

pub fn handle_drag_and_drop(ctx: &egui::Context, state: &mut AppState) {
    preview_files_being_dropped(ctx);
    ctx.input_mut(|i| {
        let sorted_dropped_files = i.raw.dropped_files.drain(..).sorted_by_key(|file| file.name.clone());
        for dropped_file in sorted_dropped_files {
            if KNOWN_AVM_STREAM_EXTENSIONS
                .iter()
                .any(|ext| dropped_file.name.ends_with(ext))
            {
                state.server_decode_manager.handle_dropped_file(dropped_file);
                state.settings.show_decode_progress = true;
            } else if KNOWN_ZIP_EXTENSIONS.iter().any(|ext| dropped_file.name.ends_with(ext)) {
                state.local_stream_manager.handle_dropped_file(dropped_file);
            } else {
                warn!("Unknown file type: {}", dropped_file.name);
            }
        }
    });
}

fn preview_files_being_dropped(ctx: &egui::Context) {
    if !ctx.input(|i| i.raw.hovered_files.is_empty()) {
        let text = ctx.input(|i| {
            if i.raw.hovered_files.len() > 1 {
                format!("Decode multiple streams: {}", i.raw.hovered_files.len())
            } else {
                let mime_type = i.raw.hovered_files[0].mime.as_str();
                if ZIP_MIME_TYPES.contains(&mime_type) {
                    "Load local stream (.zip)".to_string()
                } else if AVM_STREAM_MIME_TYPES.contains(&mime_type) {
                    "Decode stream on server".to_string()
                } else {
                    format!(
                        "Unsupported file type: {}",
                        if mime_type.is_empty() { "Unknown" } else { mime_type }
                    )
                }
            }
        });

        let painter = ctx.layer_painter(LayerId::new(Order::Foreground, Id::new("file_drop_target")));

        let screen_rect = ctx.screen_rect();
        painter.rect_filled(screen_rect, 0.0, Color32::from_black_alpha(192));
        painter.text(
            screen_rect.center(),
            Align2::CENTER_CENTER,
            text,
            TextStyle::Heading.resolve(&ctx.style()),
            Color32::WHITE,
        );
    }
}
