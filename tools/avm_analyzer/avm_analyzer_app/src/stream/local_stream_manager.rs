use std::{
    io::Cursor,
    sync::{Arc, Mutex},
};

use anyhow::anyhow;
use avm_analyzer_common::{
    AvmStreamInfo, DEFAULT_PROTO_PATH_FRAME_SUFFIX_FIRST, DEFAULT_PROTO_PATH_FRAME_SUFFIX_TEMPLATE,
};
use egui::DroppedFile;
use itertools::Itertools;
use log::info;
use rfd::AsyncFileDialog;
use wasm_bindgen_futures::spawn_local;
use zip::ZipArchive;

use crate::stream::Stream;

use super::stream_name_from_file_name;

pub struct LocalStreamInfo {
    pub file_name: String,
    pub zip_bytes: Arc<[u8]>,
}

impl LocalStreamInfo {
    pub fn get_stream_info(&self) -> anyhow::Result<AvmStreamInfo> {
        let archive = ZipArchive::new(Cursor::new(self.zip_bytes.clone()))?;
        let proto_file_names: Vec<_> = archive.file_names().filter(|n| n.ends_with(".pb")).sorted().collect();
        let first_proto = proto_file_names
            .first()
            .ok_or(anyhow!("No protobufs (.pb) found in .zip archive."))?;
        if !first_proto.ends_with(DEFAULT_PROTO_PATH_FRAME_SUFFIX_FIRST) {
            return Err(anyhow!("Unexpected protobuf naming scheme: expected suffix: {DEFAULT_PROTO_PATH_FRAME_SUFFIX_FIRST}, actual name: {first_proto}"));
        }
        let proto_path_template = first_proto.replace(
            DEFAULT_PROTO_PATH_FRAME_SUFFIX_FIRST,
            DEFAULT_PROTO_PATH_FRAME_SUFFIX_TEMPLATE,
        );

        info!("Found {} frames from .zip stream.", proto_file_names.len());
        let stream_info = AvmStreamInfo {
            stream_name: self.file_name.clone(),
            num_frames: proto_file_names.len(),
            proto_path_template,
            thumbnail_png: None,
        };
        for (i, proto_file_name) in proto_file_names.iter().enumerate() {
            let expected = stream_info.get_proto_path(i);
            if expected != **proto_file_name {
                return Err(anyhow!(
                    "Unexpected protobuf in stream: Expected name: {expected}, actual name: {proto_file_name}"
                ));
            }
        }

        Ok(stream_info)
    }
}

const DEMO_STREAM_BYTES: &[u8] = include_bytes!("../../assets/leo_qcif.zip");

pub struct LocalStreamManager {
    local_stream: Arc<Mutex<Option<LocalStreamInfo>>>,
}

impl LocalStreamManager {
    pub fn new() -> Self {
        Self {
            local_stream: Arc::new(Mutex::new(None)),
        }
    }

    pub fn load_demo_stream(&self) {
        let demo_stream = Some(LocalStreamInfo {
            file_name: "leo_qcif.ivf".into(),
            zip_bytes: DEMO_STREAM_BYTES.into(),
        });
        *self.local_stream.lock().unwrap() = demo_stream;
    }
    /// Currently only .zip files are supported.
    pub fn prompt_local_stream(&self) {
        let task = AsyncFileDialog::new().add_filter("ZIP", &["zip"]).pick_file();
        let local_stream = self.local_stream.clone();
        spawn_local(async move {
            let file = task.await;
            if let Some(file) = file {
                let bytes = file.read().await;
                let mut local_stream = local_stream.lock().unwrap();
                *local_stream = Some(LocalStreamInfo {
                    file_name: stream_name_from_file_name(&file.file_name()),
                    zip_bytes: bytes.into(),
                });
            }
        });
    }

    pub fn handle_dropped_file(&self, file: DroppedFile) {
        if let Some(bytes) = file.bytes {
            let mut local_stream = self.local_stream.lock().unwrap();
            *local_stream = Some(LocalStreamInfo {
                file_name: file.name,
                zip_bytes: bytes,
            });
        }
    }

    pub fn check_local_stream_ready(&self, stream: &mut Option<Stream>) -> anyhow::Result<()> {
        if let Some(local_stream) = self.local_stream.lock().unwrap().take() {
            info!(
                "Loaded local stream {}: {} bytes",
                local_stream.file_name,
                local_stream.zip_bytes.len()
            );
            *stream = Some(Stream::from_local_file(local_stream)?);
        }
        Ok(())
    }
}
