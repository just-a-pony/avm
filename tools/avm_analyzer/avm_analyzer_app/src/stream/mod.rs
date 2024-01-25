mod http_stream_manager;
mod local_stream_manager;
mod server_decode;

pub use http_stream_manager::*;
use itertools::Itertools;
pub use local_stream_manager::*;
pub use server_decode::*;

use std::collections::{HashMap, HashSet};
use std::io::Cursor;
use std::path::Path;
use std::sync::Arc;
use std::{io::Read, sync::Mutex};

use anyhow::anyhow;
use avm_analyzer_common::{AvmStreamInfo, DecodeState};
use avm_stats::{Frame, Spatial};

use log::{info, warn};
use poll_promise::Promise;
use prost::Message;

use web_time::Instant;
use zip::ZipArchive;

use crate::image_manager::{ImageManager, PixelDataManager};
use crate::settings::FrameSortOrder;
use local_stream_manager::LocalStreamInfo;
use server_decode::PendingServerDecode;

fn frame_from_bytes(bytes: &[u8]) -> anyhow::Result<Box<Frame>> {
    let start = Instant::now();
    let frame = Frame::decode(bytes)?;
    let duration = Instant::now() - start;
    info!(
        "Decoded frame {} in {:.2}ms:  {}x{}, {} superblocks.",
        frame.decode_index(),
        duration.as_secs_f32() * 1000.0,
        frame.width(),
        frame.height(),
        frame.superblocks.len(),
    );
    Ok(Box::new(frame))
}

pub struct LocalFileZipSource {
    zip_bytes: Arc<[u8]>,
}

impl LocalFileZipSource {
    fn new(zip_bytes: Arc<[u8]>) -> Self {
        Self { zip_bytes }
    }

    fn load_frame(&self, index: usize, stream_info: &AvmStreamInfo) -> anyhow::Result<FrameStatus> {
        let proto_path = stream_info.get_proto_path(index);
        let mut archive = ZipArchive::new(Cursor::new(self.zip_bytes.clone()))?;
        let proto_file = archive.by_name(&proto_path)?;
        info!(
            "Uncompressing {} bytes from {}.",
            proto_file.compressed_size(),
            proto_path
        );
        let start = Instant::now();
        let proto_bytes = proto_file.bytes().collect::<Result<Vec<_>, _>>()?;
        let duration = Instant::now() - start;
        info!(
            "Decompressed proto frame in {:.2}ms: {} bytes",
            duration.as_secs_f32() * 1000.0,
            proto_bytes.len()
        );

        let frame = frame_from_bytes(proto_bytes.as_slice())?;
        Ok(FrameStatus::Loaded(frame))
    }
}

pub struct HttpSource {
    url: String,
    decode_in_progress: bool,
}

impl HttpSource {
    fn new(url: impl AsRef<str>, decode_in_progress: bool) -> Self {
        Self {
            url: url.as_ref().into(),
            decode_in_progress,
        }
    }

    fn load_frame(
        &self,
        index: usize,
        stream_info: &AvmStreamInfo,
        promises: &mut Vec<Promise<FramePromiseResult>>,
    ) -> anyhow::Result<FrameStatus> {
        let url = format!("{}/{}", self.url, stream_info.get_proto_path(index));
        info!("Loading frame proto over HTTP: {url}");
        let (sender, promise) = Promise::new();
        let request = ehttp::Request::get(url);
        ehttp::fetch(request, move |response| {
            let frame = parse_proto_response(response);
            sender.send(FramePromiseResult { frame, index });
        });
        promises.push(promise);
        Ok(FrameStatus::Pending)
    }
}

pub enum StreamSource {
    LocalZipFile(LocalFileZipSource),
    Http(HttpSource),
}

#[derive(Clone)]
pub enum FrameStatus {
    Unloaded,
    Decoding,
    Pending,
    Invalid,
    OutOfBounds,
    Loaded(Box<Frame>),
}

impl FrameStatus {
    fn is_selectable(&self) -> bool {
        match self {
            FrameStatus::Decoding | FrameStatus::Invalid | FrameStatus::OutOfBounds => false,
            FrameStatus::Unloaded | FrameStatus::Pending | FrameStatus::Loaded(_) => true,
        }
    }
    fn is_loaded(&self) -> bool {
        matches!(self, FrameStatus::Loaded(_))
    }
}

struct FramePromiseResult {
    frame: Result<Option<Box<Frame>>, anyhow::Error>,
    index: usize,
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum CacheStrategy {
    Unlimited,
    Limited(usize),
}

impl CacheStrategy {
    pub fn from_limit(limit: Option<usize>) -> Self {
        match limit {
            Some(limit) => Self::Limited(limit),
            None => Self::Unlimited,
        }
    }
    fn keep_frame(&self, frame_index: usize, frame_visit_history: &[usize]) -> bool {
        match self {
            Self::Unlimited => true,
            Self::Limited(limit) => {
                let mut prev_frames = HashSet::new();
                for prev_frame in frame_visit_history.iter().rev() {
                    prev_frames.insert(prev_frame);
                    if prev_frames.len() == *limit {
                        break;
                    }
                }
                prev_frames.contains(&frame_index)
            }
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum StreamEventType {
    NewStream,
    // Note: The first frame loaded is not necessarily the first frame of the stream.
    FirstFrameLoaded(usize),
    FrameChanged(usize),
}

#[derive(Clone, Copy, Debug)]
pub struct StreamEvent {
    pub event: StreamEventType,
    lifetime: i32,
}

impl StreamEvent {
    pub fn new(event: StreamEventType) -> Self {
        Self { event, lifetime: 2 }
    }
}

#[derive(Clone, Copy, Debug)]
enum ChangeFrameKind {
    Next,
    Prev,
    First,
    Last,
    Index(usize),
}

#[derive(Clone, Copy, Debug)]
pub struct ChangeFrame {
    order: FrameSortOrder,
    kind: ChangeFrameKind,
    loaded_only: bool,
    allow_loop: bool,
}

impl Default for ChangeFrame {
    fn default() -> Self {
        Self {
            order: FrameSortOrder::Decode,
            kind: ChangeFrameKind::Index(0),
            loaded_only: false,
            allow_loop: false,
        }
    }
}

impl ChangeFrame {
    pub fn next() -> Self {
        Self {
            kind: ChangeFrameKind::Next,
            ..Default::default()
        }
    }

    pub fn prev() -> Self {
        Self {
            kind: ChangeFrameKind::Prev,
            ..Default::default()
        }
    }

    pub fn first() -> Self {
        Self {
            kind: ChangeFrameKind::First,
            ..Default::default()
        }
    }

    pub fn last() -> Self {
        Self {
            kind: ChangeFrameKind::Last,
            ..Default::default()
        }
    }
    pub fn index(index: usize) -> Self {
        Self {
            kind: ChangeFrameKind::Index(index),
            ..Default::default()
        }
    }
    pub fn loaded_only(mut self, loaded_only: bool) -> Self {
        self.loaded_only = loaded_only;
        self
    }

    pub fn allow_loop(mut self, allow_loop: bool) -> Self {
        self.allow_loop = allow_loop;
        self
    }

    pub fn order(mut self, order: FrameSortOrder) -> Self {
        self.order = order;
        self
    }
}

pub struct Stream {
    pub stream_info: AvmStreamInfo,
    pub source: StreamSource,
    pub frames: Vec<FrameStatus>,
    pub images: ImageManager,
    pub pixel_data: PixelDataManager,
    promises: Mutex<Vec<Promise<FramePromiseResult>>>,
    // TODO(comc): Handle decode_index to proto frame index mapping.
    pub current_frame_index: usize,
    events: Vec<StreamEvent>,
    have_first_frame: bool,
    pub frame_visit_history: Vec<usize>,
}

impl Stream {
    fn new(stream_info: AvmStreamInfo, source: StreamSource) -> Self {
        let mut default_frame_status = FrameStatus::Unloaded;
        if let StreamSource::Http(ref http_source) = source {
            if http_source.decode_in_progress {
                default_frame_status = FrameStatus::Decoding;
            }
        }
        let num_frames = stream_info.num_frames;
        Self {
            stream_info,
            source,
            frames: vec![default_frame_status; num_frames],
            promises: Mutex::new(Vec::new()),
            images: ImageManager::default(),
            pixel_data: PixelDataManager::default(),
            current_frame_index: usize::MAX,
            events: vec![StreamEvent::new(StreamEventType::NewStream)],
            have_first_frame: false,
            frame_visit_history: vec![0],
        }
    }

    pub fn from_http(
        stream_info: AvmStreamInfo,
        decode_in_progress: bool,
        first_first_to_load: usize,
        streams_url: &str
    ) -> anyhow::Result<Self> {
        let source = StreamSource::Http(HttpSource::new(streams_url, decode_in_progress));
        let mut stream = Stream::new(stream_info, source);
        stream.set_current_frame(first_first_to_load, false);
        Ok(stream)
    }

    pub fn from_local_file(local_stream: LocalStreamInfo) -> anyhow::Result<Self> {
        let stream_info = local_stream.get_stream_info()?;
        let source = StreamSource::LocalZipFile(LocalFileZipSource::new(local_stream.zip_bytes));
        let mut stream = Stream::new(stream_info, source);
        stream.set_current_frame(0, false);
        Ok(stream)
    }

    // Add a method on Frame for this.
    pub fn have_orig_yuv(&self) -> bool {
        if let FrameStatus::Loaded(frame) = self.get_frame(0) {
            if let Some(superblock) = &frame.superblocks.first() {
                if let Some(pixel_data) = &superblock.pixel_data.first() {
                    if pixel_data.original.is_some() {
                        return true;
                    }
                }
            }
        }
        false
    }

    /// Looks up an order hint from a motion vector and converts it into a decode index.
    pub fn lookup_order_hint(&self, order_hint: i32) -> Option<usize> {
        for index in (0..self.current_frame_index).rev() {
            if let FrameStatus::Loaded(frame) = self.get_frame(index) {
                if frame.frame_params.as_ref().unwrap().raw_display_index == order_hint {
                    return Some(index);
                }
            }
        }
        None
    }

    pub fn update_pending_decodes(&mut self, pending_decodes: &mut HashMap<String, PendingServerDecode>) {
        for pending_decode in pending_decodes.values_mut() {
            let pending_decode_matches = {
                if let StreamSource::Http(ref http_source) = self.source {
                    http_source.decode_in_progress
                        && pending_decode.stream_info.stream_name == self.stream_info.stream_name
                } else {
                    false
                }
            };
            let mut decode_finished = false;
            if pending_decode_matches {
                match pending_decode.state {
                    DecodeState::Complete(num_frames) => {
                        for i in 0..num_frames {
                            let frame = &mut self.frames[i];
                            if let FrameStatus::Decoding = frame {
                                *frame = FrameStatus::Unloaded
                            }
                        }
                        if num_frames > 0 && !pending_decode.started_loading_first_frame {
                            self.set_current_frame(0, false);
                            pending_decode.started_loading_first_frame = true;
                        }
                        decode_finished = true;
                    }
                    DecodeState::Pending(ref pending) => {
                        for i in 0..pending.decoded_frames {
                            let frame = &mut self.frames[i];
                            if let FrameStatus::Decoding = frame {
                                *frame = FrameStatus::Unloaded
                            }
                        }
                        if pending.decoded_frames > 0 && !pending_decode.started_loading_first_frame {
                            self.set_current_frame(0, false);
                            pending_decode.started_loading_first_frame = true;
                        }
                    }
                    DecodeState::Failed => {
                        for frame in self.frames.iter_mut() {
                            if let FrameStatus::Decoding = frame {
                                *frame = FrameStatus::Invalid
                            }
                        }
                    }
                    DecodeState::Uploading => {}
                    DecodeState::UploadComplete => {}
                }
            }
            if decode_finished {
                if let StreamSource::Http(http_source) = &mut self.source {
                    http_source.decode_in_progress = false;
                }
            }
        }
    }

    pub fn current_frame(&self) -> Option<&Frame> {
        if let FrameStatus::Loaded(ref frame) = self.get_frame(self.current_frame_index) {
            Some(frame)
        } else {
            None
        }
    }

    /// Sets the current frame to `index`.
    /// If `loaded_only` is true, only changed the frame if it is already loaded.
    /// Returns true if the frame was changed.
    fn set_current_frame(&mut self, index: usize, loaded_only: bool) -> bool {
        if index < self.num_frames()
            && index != self.current_frame_index
            && self.frames[index].is_selectable()
            && (self.frames[index].is_loaded() || !loaded_only)
        {
            self.current_frame_index = index;
            if let Err(err) = self.load_frame(self.current_frame_index) {
                warn!("Loading frame {} failed: {err:?}.", self.current_frame_index);
            }
            self.events.push(StreamEvent::new(StreamEventType::FrameChanged(index)));
            self.frame_visit_history.push(index);
            true
        } else {
            false
        }
    }

    pub fn apply_cache_strategy(&mut self, cache_strategy: CacheStrategy) {
        for frame_index in 0..self.num_frames() {
            if !cache_strategy.keep_frame(frame_index, &self.frame_visit_history) {
                self.unload_frame(frame_index);
            }
        }
    }

    pub fn unload_frame(&mut self, index: usize) {
        if matches!(self.frames[index], FrameStatus::Loaded(_)) {
            self.frames[index] = FrameStatus::Unloaded;
        }
    }

    pub fn change_frame(&mut self, change: ChangeFrame) -> bool {
        let sorted_frames = self.get_sorted_frames(change.order);
        let Some(current_index) = sorted_frames.iter().position(|&i| i == self.current_frame_index) else {
            return false;
        };

        match change.kind {
            ChangeFrameKind::First => self.set_current_frame(sorted_frames[0], change.loaded_only),
            ChangeFrameKind::Last => self.set_current_frame(sorted_frames[sorted_frames.len() - 1], change.loaded_only),
            ChangeFrameKind::Index(index) => {
                if let Some(sorted_index) = sorted_frames.get(index) {
                    self.set_current_frame(*sorted_index, change.loaded_only)
                } else {
                    false
                }
            }
            ChangeFrameKind::Prev => {
                if current_index > 0 {
                    self.set_current_frame(sorted_frames[current_index - 1], change.loaded_only)
                } else {
                    false
                }
            }
            ChangeFrameKind::Next => {
                let next_index = if change.allow_loop {
                    (current_index + 1) % sorted_frames.len()
                } else {
                    current_index + 1
                };
                if next_index < sorted_frames.len() {
                    // Try to change to the next frame. If it fails (e.g. because that frame is not yet loaded) and looping is on, try loading the first frame instead.
                    let frame_changed = self.set_current_frame(sorted_frames[next_index], change.loaded_only);
                    if !frame_changed && change.allow_loop {
                        self.set_current_frame(sorted_frames[0], change.loaded_only)
                    } else {
                        frame_changed
                    }
                } else {
                    false
                }
            }
        }
    }

    pub fn get_sorted_frames(&self, frame_sort_order: FrameSortOrder) -> Vec<usize> {
        let sorted_indices = match frame_sort_order {
            FrameSortOrder::Decode => (0..self.num_frames()).collect_vec(),
            // TODO(comc): There's not really a good way to sort frames by display order
            // ahead-of-time unless they're already loaded. For now, the decode index is used
            // as the sort key if we don't have the display index yet.
            FrameSortOrder::Display => self
                .frames
                .iter()
                .enumerate()
                .sorted_by_key(|(index, frame)| match frame {
                    FrameStatus::Loaded(frame) => (frame.display_index(), *index, true),
                    _ => (*index, *index, false),
                })
                .map(|(index, _)| index)
                .collect_vec(),
        };
        sorted_indices
    }

    pub fn get_frame(&self, index: usize) -> &FrameStatus {
        self.frames.get(index).unwrap_or(&FrameStatus::OutOfBounds)
    }

    pub fn load_frame(&mut self, index: usize) -> anyhow::Result<()> {
        let do_load = matches!(self.frames.get(index), Some(FrameStatus::Unloaded));
        if do_load {
            self.frames[index] = match &self.source {
                StreamSource::LocalZipFile(local) => {
                    let frame = local.load_frame(index, &self.stream_info)?;
                    if !self.have_first_frame {
                        self.have_first_frame = true;
                        self.events
                            .push(StreamEvent::new(StreamEventType::FirstFrameLoaded(index)));
                    }
                    frame
                }
                StreamSource::Http(http) => {
                    http.load_frame(index, &self.stream_info, &mut self.promises.lock().unwrap())?
                }
            };
        }
        Ok(())
    }

    pub fn num_frames(&self) -> usize {
        self.stream_info.num_frames
    }

    pub fn check_promises(&mut self) {
        let mut promises = self.promises.lock().unwrap();
        promises.retain_mut(|promise| {
            if let Some(result) = promise.ready_mut() {
                let frame = result.frame.as_mut();
                match frame {
                    Ok(frame) => {
                        self.frames[result.index] = FrameStatus::Loaded(frame.take().unwrap());
                        if !self.have_first_frame {
                            self.have_first_frame = true;
                            self.events
                                .push(StreamEvent::new(StreamEventType::FirstFrameLoaded(result.index)));
                        }
                    }
                    Err(_err) => {
                        self.frames[result.index] = FrameStatus::Invalid;
                    }
                }
                false
            } else {
                true
            }
        });
    }

    pub fn check_events(&mut self) -> Vec<StreamEvent> {
        let events = self.events.clone();
        for event in self.events.iter_mut() {
            event.lifetime -= 1;
        }
        self.events.retain(|ev| ev.lifetime > 0);
        events
    }
}

/// Helper to check if we have a current frame on an Option<Stream>.
pub trait CurrentFrame {
    fn current_frame(&self) -> Option<&Frame>;
    fn current_frame_is_loaded(&self) -> bool;
}

impl CurrentFrame for Option<Stream> {
    fn current_frame(&self) -> Option<&Frame> {
        let stream = self.as_ref()?;
        stream.current_frame()
    }
    fn current_frame_is_loaded(&self) -> bool {
        self.current_frame().is_some()
    }
}

fn parse_proto_response(response: Result<ehttp::Response, String>) -> anyhow::Result<Option<Box<Frame>>> {
    let response = response.map_err(|err| anyhow!("HTTP error: {err}"))?;
    let content_type = response.content_type().unwrap_or_default();
    let frame = match content_type {
        "application/octet-stream" => frame_from_bytes(response.bytes.as_slice())?,
        "application/zip" => {
            let mut archive = ZipArchive::new(Cursor::new(response.bytes.as_slice()))?;
            let proto_file_name = archive
                .file_names()
                .filter(|n| n.ends_with(".pb"))
                .sorted()
                .next()
                .ok_or(anyhow!("No protobufs (.pb) found in .zip archive."))?
                .to_string();
            let proto_file = archive.by_name(&proto_file_name)?;
            let proto_bytes = proto_file.bytes().collect::<Result<Vec<_>, _>>()?;
            frame_from_bytes(proto_bytes.as_slice())?
        }
        _ => {
            return Err(anyhow!("Unexpected content type: {content_type}"));
        }
    };
    Ok(Some(frame))
}

pub fn stream_name_from_file_name(file_name: &str) -> String {
    Path::new(file_name).file_stem().unwrap().to_string_lossy().to_string()
}
