use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
};

use avm_analyzer_common::{AvmStreamInfo, AvmStreamList, DecodeState};
use log::info;

use super::server_decode::PendingServerDecode;

pub struct HttpStreamManager {
    pub streams: Arc<Mutex<Vec<AvmStreamInfo>>>,
}

impl HttpStreamManager {
    pub fn new() -> Self {
        Self {
            streams: Arc::new(Mutex::new(Vec::new())),
        }
    }

    pub fn load_stream_list(&self) {
        let request = ehttp::Request::get("/stream_list");
        let streams = self.streams.clone();
        ehttp::fetch(request, move |response| {
            if let Ok(response) = response {
                if let Some(json) = response.text() {
                    if let Ok(response) = serde_json::from_str::<AvmStreamList>(json) {
                        info!("Found: {} existing streams on server.", response.streams.len());
                        let mut streams = streams.lock().unwrap();
                        *streams.as_mut() = response.streams;
                    }
                }
            }
        });
    }
    pub fn update_pending_decodes(&mut self, pending_decodes: &HashMap<String, PendingServerDecode>) {
        for pending_decode in pending_decodes.values() {
            if let DecodeState::Complete(_) = pending_decode.state {
                let streams = self.streams.lock().unwrap();
                // TODO(comc): Make streams a hashmap instead.
                let already_exists = streams
                    .iter()
                    .any(|stream| stream.stream_name == pending_decode.stream_info.stream_name);
                if !already_exists {
                    self.load_stream_list();
                }
            }
        }
    }
}
