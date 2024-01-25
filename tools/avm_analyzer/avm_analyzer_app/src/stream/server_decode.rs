use avm_analyzer_common::{AvmStreamInfo, DecodeProgress, DecodeState, ProgressResponse, StartDecodeResponse};
use egui::DroppedFile;

use log::{info, warn};

use rand::distributions::Alphanumeric;
use rand::Rng;
use rfd::AsyncFileDialog;
use std::collections::HashMap;
use std::io::{self, Cursor, Write};
use std::sync::{Arc, Mutex};
use wasm_bindgen_futures::spawn_local;
use web_time::Instant;

use crate::stream::{stream_name_from_file_name, Stream};

use super::HttpStreamManager;
pub const HTTP_POLL_PERIOD_SECONDS: f32 = 1.0;
pub const MAX_RETRIES: i32 = 5;

#[derive(Clone)]
pub struct PendingServerDecode {
    pub stream_info: AvmStreamInfo,
    pub state: DecodeState,
    pub start_time: Instant,
    pub started_loading_first_frame: bool,
    // For retries:
    pub bytes: Arc<[u8]>,
    pub retry_count: i32,
}

impl PendingServerDecode {
    fn new(name: &str, state: DecodeState, bytes: Arc<[u8]>, retries_left: i32) -> Self {
        let stream_info = AvmStreamInfo {
            stream_name: name.into(),
            proto_path_template: "".into(),
            num_frames: 0,
            thumbnail_png: None,
        };
        Self {
            stream_info,
            state,
            start_time: Instant::now(),
            started_loading_first_frame: false,
            bytes,
            retry_count: retries_left,
        }
    }
}

pub struct ServerDecodeManager {
    pub pending_decodes: Arc<Mutex<HashMap<String, PendingServerDecode>>>,
    last_check_request: Instant,
}

impl ServerDecodeManager {
    pub fn new() -> Self {
        Self {
            pending_decodes: Arc::new(Mutex::new(HashMap::new())),
            last_check_request: Instant::now(),
        }
    }
    pub fn prompt_stream(&self) {
        let task = AsyncFileDialog::new()
            .add_filter("AVM Stream", &["bin", "ivf", "obu"])
            .pick_file();
        let pending_decodes = self.pending_decodes.clone();
        spawn_local(async move {
            let file = task.await;
            if let Some(file) = file {
                let bytes = file.read().await;
                upload(file.file_name().as_str(), bytes.into(), pending_decodes, MAX_RETRIES);
            }
        });
    }

    pub fn handle_dropped_file(&self, file: DroppedFile) {
        let pending_decodes = self.pending_decodes.clone();
        if let Some(bytes) = file.bytes {
            spawn_local(async move {
                upload(file.name.as_str(), bytes, pending_decodes, MAX_RETRIES);
            });
        }
    }

    pub fn check_newly_confirmed_uploads(&mut self, stream: &mut Option<Stream>, streams_url: &str) {
        let mut pending_decodes = self.pending_decodes.lock().unwrap();
        for (_name, pending_decode) in pending_decodes.iter_mut() {
            if let DecodeState::UploadComplete = pending_decode.state {
                *stream = Some(Stream::from_http(pending_decode.stream_info.clone(), true, 0, streams_url).unwrap());
                pending_decode.state = DecodeState::Pending(DecodeProgress {
                    decoded_frames: 0,
                    total_frames: pending_decode.stream_info.num_frames,
                });
            }
        }
    }
    fn check_for_retries(&mut self) {
        let mut retries = Vec::new();
        {
            let mut pending_decodes = self.pending_decodes.lock().unwrap();
            for (_name, pending_decode) in pending_decodes.iter_mut() {
                if matches!(pending_decode.state, DecodeState::Failed) {
                    retries.push(pending_decode.clone());
                }
            }
        }
        for retry in retries {
            if retry.retry_count > 0 {
                warn!("Retrying stream: {}. Retries remaining: {}", retry.stream_info.stream_name, retry.retry_count - 1);
                upload(
                    &retry.stream_info.stream_name,
                    retry.bytes,
                    self.pending_decodes.clone(),
                    retry.retry_count - 1
                );
            }
            else {
                warn!("Stream: {} failed after {} retries.", retry.stream_info.stream_name, MAX_RETRIES);
            }
        }
    }

    pub fn check_progress(&mut self, stream: &mut Option<Stream>, http_manager: &mut HttpStreamManager, streams_url: &str) {
        self.check_newly_confirmed_uploads(stream, streams_url);
        self.check_for_retries();
        let now = Instant::now();
        let elapsed = now - self.last_check_request;
        if elapsed.as_secs_f32() < HTTP_POLL_PERIOD_SECONDS {
            return;
        }
        self.last_check_request = now;
        let mut pending_decodes = self.pending_decodes.lock().unwrap();
        for (_name, pending_decode) in pending_decodes.iter() {
            if let DecodeState::Pending(_) = &pending_decode.state {
                let request = ehttp::Request::get(format!(
                    "/progress?stream_name={}",
                    pending_decode.stream_info.stream_name
                ));
                let inner_pending_decodes = self.pending_decodes.clone();
                ehttp::fetch(request, move |response| {
                    if let Ok(response) = response {
                        if let Some(json) = response.text() {
                            if let Ok(response) = serde_json::from_str::<ProgressResponse>(json) {
                                info!("Progress: {response:?}");
                                let mut pending_decodes = inner_pending_decodes.lock().unwrap();
                                if let Some(pending_decode) = pending_decodes.get_mut(&response.stream_name) {
                                    pending_decode.state = response.state;
                                } else {
                                    warn!(
                                        "Received decode progress update for unknown stream: {}.",
                                        response.stream_name
                                    );
                                }
                            }
                        }
                    }
                });
            }
        }
        if let Some(stream) = stream {
            // TODO(comc): Move this logic elsewhere.
            http_manager.update_pending_decodes(&pending_decodes);
            stream.update_pending_decodes(&mut pending_decodes);
        }
    }
}

const MULTIPART_FORM_BOUNDARY_PREFIX: &str = "--------";
const MULTIPART_FORM_BOUNDARY_LEN: usize = 16;

fn random_boundary() -> String {
    let alphanum: String = rand::thread_rng()
        .sample_iter(Alphanumeric)
        .take(MULTIPART_FORM_BOUNDARY_LEN)
        .map(char::from)
        .collect();
    format!("{MULTIPART_FORM_BOUNDARY_PREFIX}{alphanum}")
}

fn create_upload_request(file_name: &str, bytes: Arc<[u8]>) -> Result<ehttp::Request, io::Error> {
    let stream_name = stream_name_from_file_name(file_name);
    let mut request_body = Vec::new();
    let mut cursor = Cursor::new(&mut request_body);
    let boundary = random_boundary();
    write!(cursor, "--{}\r\n", boundary)?;
    write!(
        cursor,
        "Content-Disposition: form-data; name=\"{stream_name}\"; filename=\"{file_name}\""
    )?;
    write!(cursor, "\r\nContent-Type: {}", mime::APPLICATION_OCTET_STREAM)?;
    write!(cursor, "\r\n\r\n")?;
    cursor.write_all(&bytes)?;
    write!(cursor, "\r\n")?;
    write!(cursor, "--{}--\r\n", boundary)?;
    let content_type = format!("multipart/form-data; boundary={}", boundary);

    Ok(ehttp::Request {
        method: "POST".into(),
        url: "/upload".into(),
        body: request_body,
        headers: ehttp::headers(&[("Accept", "*/*"), ("Content-Type", &content_type)]),
    })
}

// TODO(comc): Handle duplicate stream names.
pub fn upload(file_name: &str, bytes: Arc<[u8]>, pending_decodes: Arc<Mutex<HashMap<String, PendingServerDecode>>>, retries_left: i32) {
    let stream_name = stream_name_from_file_name(file_name);
    {
        let mut pending_decodes = pending_decodes.lock().unwrap();
        if let Some(existing_decode) = pending_decodes.get(&stream_name) {
            match existing_decode.state {
                DecodeState::Pending(_) | DecodeState::UploadComplete | DecodeState::Uploading => {
                    warn!("Decode request for stream {stream_name} already pending.");
                    return;
                }
                _ => {}
            }
        }
        let new_decode = PendingServerDecode::new(&stream_name, DecodeState::Uploading, bytes.clone(), retries_left);
        pending_decodes.insert(stream_name.clone(), new_decode);
    }
    let req = create_upload_request(file_name, bytes).unwrap();
    // TODO(comc): Automatic retry on timeout.
    ehttp::fetch(req, move |response| {
        let mut decode_state = DecodeState::Failed;
        let mut updated_stream_info = None;

        if let Ok(response) = response {
            if let Some(json) = response.text() {
                info!("Got response: {}", json);
                if let Ok(response) = serde_json::from_str::<StartDecodeResponse>(json) {
                    updated_stream_info = Some(response.stream_info);
                    decode_state = DecodeState::UploadComplete;
                }
            }
        }
        let mut pending_decodes = pending_decodes.lock().unwrap();
        if let Some(pending_decode) = pending_decodes.get_mut(&stream_name) {
            pending_decode.state = decode_state;
            if let Some(updated_stream_info) = updated_stream_info {
                pending_decode.stream_info = updated_stream_info;
            }
        } else {
            warn!("Received status response for {stream_name}, but it no longer exists.");
        }
    });
}
