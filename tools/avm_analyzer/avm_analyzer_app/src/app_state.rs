use crate::views::{PerformanceHistory, RenderView, SelectedObject, SelectedObjectKind};

use avm_analyzer_common::{AvmStreamInfo, DEFAULT_PROTO_PATH_FRAME_SUFFIX_TEMPLATE};
use avm_stats::Spatial;
use egui::{Ui, WidgetText};
use egui_dock::TabViewer;

use log::{info, warn};

use crate::settings::{Settings, SharableSettings};
use crate::stream::{
    CacheStrategy, FrameStatus, HttpStreamManager, LocalStreamManager, ServerDecodeManager, Stream, StreamEventType,
};

pub struct AppState {
    pub stream: Option<Stream>,
    pub settings: Settings,
    pub local_stream_manager: LocalStreamManager,
    pub server_decode_manager: ServerDecodeManager,
    pub http_stream_manager: HttpStreamManager,
    pub performance_history: PerformanceHistory,
    pub previous_shared_state: SharableSettings,
}

pub const STATE_URL_QUERY_PARAM_NAME: &str = "state";
pub const LOAD_STREAM_URL_PARAM_NAME: &str = "load_stream";
pub const NUM_FRAMES_URL_PARAM_NAME: &str = "num_frames";

impl AppState {
    pub fn new(saved_settings: Option<String>) -> Self {
        let http_stream_manager = HttpStreamManager::new();
        let local_stream_manager = LocalStreamManager::new();
        let mut stream = None;
        let mut shared_settings = None;
        let mut load_stream_info = None;
        let window = web_sys::window().unwrap();
        if let Ok(search) = window.location().search() {
            if let Ok(params) = web_sys::UrlSearchParams::new_with_str(&search) {
                if let Some(state) = params.get(STATE_URL_QUERY_PARAM_NAME) {
                    shared_settings = Some(state);
                }
                if let Some(load_stream) = params.get(LOAD_STREAM_URL_PARAM_NAME) {
                    let num_frames = params.get(NUM_FRAMES_URL_PARAM_NAME).unwrap_or("1".into());
                    let num_frames = num_frames.parse::<usize>().unwrap_or(1);
                    if let Some(stream_name_end) = load_stream.find(DEFAULT_PROTO_PATH_FRAME_SUFFIX_TEMPLATE) {
                        let stream_name = load_stream[..stream_name_end].to_string();
                        load_stream_info = Some(AvmStreamInfo {
                            stream_name,
                            num_frames,
                            proto_path_template: load_stream,
                            thumbnail_png: None
                        });
                    }
                }
            }
        }

        let mut settings = Settings::from_shared_settings_string(shared_settings.as_deref());
        settings.apply_saved_settings_string(saved_settings);
        if let Some(stream_info) = load_stream_info {
            settings.sharable.selected_stream = Some(stream_info);
            settings.sharable.show_remote_streams = false;
            settings.sharable.streams_url = "".into();
        }

        if settings.sharable.show_remote_streams {
            http_stream_manager.load_stream_list();
        }

        if let Some(stream_info) = &settings.sharable.selected_stream {
            // Unwrap is okay here, since nothing can fail immediately when creating an HTTP stream.
            stream = Some(Stream::from_http(stream_info.clone(), false, settings.sharable.selected_frame, &settings.sharable.streams_url).unwrap());
        } else {
            local_stream_manager.load_demo_stream();
            local_stream_manager
                .check_local_stream_ready(&mut stream)
                .expect("Failed to load demo stream");
        }
        Self {
            stream,
            settings,
            local_stream_manager,
            server_decode_manager: ServerDecodeManager::new(),
            http_stream_manager,
            performance_history: PerformanceHistory::default(),
            previous_shared_state: SharableSettings::default(),
        }
    }
    /// Called whenever the current frame changed. Should reset anything specific to the current frame, e.g. selected objects.
    pub fn check_stream_events(&mut self) {
        if let Some(stream) = self.stream.as_mut() {
            let events = stream.check_events();
            for event in events {
                let event = event.event;
                info!("Event: {event:?}");
                match event {
                    StreamEventType::NewStream => {
                        self.settings.selected_object = None;
                        self.settings.selected_object_leaf = None;
                        self.settings.cached_stat_data = None;
                        self.settings.scroll_to_index = Some(0);
                    }
                    StreamEventType::FrameChanged(index) => {
                        self.settings.selected_object = None;
                        self.settings.selected_object_leaf = None;
                        self.settings.cached_stat_data = None;
                        self.settings.scroll_to_index = Some(index);
                        let limit = if self.settings.persistent.apply_cache_strategy {
                            Some(self.settings.persistent.cache_strategy_limit)
                        } else {
                            None
                        };
                        stream.apply_cache_strategy(CacheStrategy::from_limit(limit));
                    }
                    StreamEventType::FirstFrameLoaded(index) => {
                        info!("First frame loaded: {index}");
                        if let FrameStatus::Loaded(frame) = stream.get_frame(index) {
                            if !self.settings.have_world_bounds_from_shared_settings {
                                self.settings.sharable.world_bounds = frame.rect();
                            }
                            if let Some(shared_selected_object) = self.settings.sharable.selected_object_kind.take() {
                                if let FrameStatus::Loaded(frame) = stream.get_frame(index) {
                                    let is_valid = match &shared_selected_object {
                                        SelectedObjectKind::CodingUnit(obj) => obj.try_resolve(frame).is_some(),
                                        SelectedObjectKind::TransformUnit(obj) => obj.try_resolve(frame).is_some(),
                                        SelectedObjectKind::Superblock(obj) => obj.try_resolve(frame).is_some(),
                                        SelectedObjectKind::Partition(obj) => obj.try_resolve(frame).is_some(),
                                    };
                                    if is_valid {
                                        info!("Restored selected object: {shared_selected_object:?}");
                                        self.settings.selected_object =
                                            Some(SelectedObject::new(shared_selected_object));
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

impl TabViewer for AppState {
    type Tab = Box<dyn RenderView>;

    fn ui(&mut self, ui: &mut Ui, tab: &mut Self::Tab) {
        if let Err(err) = tab.render(ui, self) {
            warn!("{}", err);
        }
    }

    fn title(&mut self, tab: &mut Self::Tab) -> WidgetText {
        tab.title().into()
    }
}
