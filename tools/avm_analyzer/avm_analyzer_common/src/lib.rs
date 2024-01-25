use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct ProgressRequest {
    pub stream_name: String,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct DecodeProgress {
    pub decoded_frames: usize,
    pub total_frames: usize,
}

// TODO(comc): Add timeout state?
#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum DecodeState {
    /// Upload request sent by client, not yet acknowledged by server.
    Uploading,
    /// Upload was successful and the server sent confirmation.
    UploadComplete,
    /// Decode (extract_proto) is is progress.
    Pending(DecodeProgress),
    /// Decoding succeeded.
    Complete(usize),
    /// Decode failed for any reason.
    Failed,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct ProgressResponse {
    pub stream_name: String,
    pub state: DecodeState,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct StartDecodeResponse {
    pub stream_info: AvmStreamInfo,
}

pub const PROTO_PATH_FRAME_PLACEHOLDER: &str = "{FRAME}";
pub const DEFAULT_PROTO_PATH_FRAME_SUFFIX_FIRST: &str = "_frame_0000.pb";
pub const DEFAULT_PROTO_PATH_FRAME_SUFFIX_TEMPLATE: &str = "_frame_{FRAME}.pb";

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct AvmStreamInfo {
    pub stream_name: String,
    pub proto_path_template: String,
    pub num_frames: usize,
    pub thumbnail_png: Option<Vec<u8>>,
}

impl AvmStreamInfo {
    pub fn get_proto_path(&self, index: usize) -> String {
        self.proto_path_template
            .replace(PROTO_PATH_FRAME_PLACEHOLDER, &format!("{index:04}"))
    }
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct AvmStreamList {
    pub streams: Vec<AvmStreamInfo>,
}
