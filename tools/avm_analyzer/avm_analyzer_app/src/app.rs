use eframe::Storage;
use egui::{CentralPanel, TopBottomPanel};
use egui_dock::NodeIndex;
use egui_dock::{DockArea, DockState};

use log::{info, warn};
use wasm_bindgen::JsValue;
use web_time::{Duration, Instant};

use crate::app_state::{AppState, STATE_URL_QUERY_PARAM_NAME};
use crate::stream::{ChangeFrame, CurrentFrame, HTTP_POLL_PERIOD_SECONDS};
use crate::views::{
    handle_drag_and_drop, BlockInfoViewer, CoeffsViewer, ControlsViewer, DecodeProgressViewer, DetailedPixelViewer,
    FrameInfoViewer, FrameSelectViewer, FrameViewer, MenuBar, PerformanceViewer, RenderView, SelectedObjectKind,
    SettingsViewer, StatsViewer, StreamSelectViewer, SymbolInfoViewer,
};

type TabType = Box<dyn RenderView>;

pub struct AvmAnalyzerApp {
    state: AppState,
    dock_state: DockState<TabType>,
}

const SAVED_SETTINGS_KEY: &str = "settings";
impl AvmAnalyzerApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let mut saved_settings = None;
        if let Some(storage) = cc.storage {
            if let Some(settings) = storage.get_string(SAVED_SETTINGS_KEY) {
                saved_settings = Some(settings);
            }
        }
        let mut dock_state: DockState<TabType> = DockState::new(vec![Box::new(FrameViewer)]);
        let surface = dock_state.main_surface_mut();

        let [others_view, _frames_view] =
            surface.split_above(NodeIndex::root(), 0.15, vec![Box::new(FrameSelectViewer)]);
        let [yuv_view, _info_view] = surface.split_left(
            others_view,
            0.2,
            vec![
                Box::new(FrameInfoViewer),
                Box::new(BlockInfoViewer),
                Box::new(SymbolInfoViewer),
                Box::new(StatsViewer),
            ],
        );

        let [_, _] = surface.split_below(yuv_view, 0.8, vec![Box::new(ControlsViewer)]);
        let state = AppState::new(saved_settings);

        Self { state, dock_state }
    }

    pub fn check_keyboard_input(&mut self, ctx: &egui::Context) {
        if let Some(stream) = self.state.stream.as_mut() {
            if ctx.input(|i| i.key_pressed(egui::Key::ArrowLeft)) {
                stream.change_frame(ChangeFrame::prev().order(self.state.settings.persistent.frame_sort_order));
            } else if ctx.input(|i| i.key_pressed(egui::Key::ArrowRight)) {
                stream.change_frame(ChangeFrame::next().order(self.state.settings.persistent.frame_sort_order));
            }
        }
    }

    pub fn check_playback(&mut self) {
        let playback = &mut self.state.settings.playback;
        if playback.playback_running {
            if let Some(stream) = self.state.stream.as_mut() {
                let current_time = Instant::now();
                let elapsed = current_time - playback.current_frame_display_instant;
                let time_between_frames = Duration::from_secs_f32(1.0 / playback.playback_fps);
                if elapsed > time_between_frames {
                    playback.current_frame_display_instant = current_time;
                    let changed = stream.change_frame(
                        ChangeFrame::next()
                            .order(self.state.settings.persistent.frame_sort_order)
                            .allow_loop(playback.playback_loop)
                            .loaded_only(playback.show_loaded_frames_only),
                    );
                    if !changed {
                        playback.playback_running = false;
                    }
                }
            }
        }
    }
}

impl eframe::App for AvmAnalyzerApp {
    fn update(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        self.state.check_stream_events();
        self.state
            .server_decode_manager
            .check_progress(&mut self.state.stream, &mut self.state.http_stream_manager, &self.state.settings.sharable.streams_url);
        self.check_keyboard_input(ctx);
        self.check_playback();
        handle_drag_and_drop(ctx, &mut self.state);

        if let Some(stream) = self.state.stream.as_mut() {
            stream.check_promises();
        }

        ctx.input(|input| {
            self.state
                .performance_history
                .on_new_frame(input.time, frame.info().cpu_usage)
        });

        ctx.request_repaint_after(Duration::from_secs_f32(HTTP_POLL_PERIOD_SECONDS));
        if let Err(err) = self
            .state
            .local_stream_manager
            .check_local_stream_ready(&mut self.state.stream)
        {
            warn!("{}", err);
        };

        let menu_bar = MenuBar {};
        TopBottomPanel::top(menu_bar.title()).show(ctx, |ui| {
            if let Err(err) = menu_bar.render(ui, &mut self.state) {
                warn!("{}", err);
            }
        });
        CentralPanel::default()
            .frame(egui::Frame::central_panel(&ctx.style()).inner_margin(0.))
            .show(ctx, |ui| {
                DockArea::new(&mut self.dock_state)
                    .show_close_buttons(false)
                    .show_add_buttons(false)
                    .draggable_tabs(false)
                    .show_tab_name_on_hover(false)
                    .show_inside(ui, &mut self.state);
            });

        let close_window = false;
        let mut show_stream_select = self.state.settings.show_stream_select;
        egui::Window::new("Load Stream")
            .default_width(800.0)
            .open(&mut show_stream_select)
            .collapsible(false)
            .show(ctx, |ui| {
                if let Err(err) = StreamSelectViewer.render(ui, &mut self.state) {
                    warn!("{}", err);
                }
            });
        if !show_stream_select || close_window {
            self.state.settings.show_stream_select = false;
        }

        let mut show_decode_progress: bool = self.state.settings.show_decode_progress;
        egui::Window::new("Decode Progress")
            .open(&mut show_decode_progress)
            .default_width(800.0)
            .default_height(600.0)
            .collapsible(false)
            .show(ctx, |ui| {
                if let Err(err) = DecodeProgressViewer.render(ui, &mut self.state) {
                    warn!("{}", err);
                }
            });
        self.state.settings.show_decode_progress = show_decode_progress;

        let mut show_performance_window: bool = self.state.settings.show_performance_window;
        egui::Window::new("Performance")
            .open(&mut show_performance_window)
            .collapsible(false)
            .show(ctx, |ui| {
                if let Err(err) = PerformanceViewer.render(ui, &mut self.state) {
                    warn!("{}", err);
                }
            });
        self.state.settings.show_performance_window = show_performance_window;

        let mut show_settings_window: bool = self.state.settings.show_settings_window;
        egui::Window::new("Settings")
            .open(&mut show_settings_window)
            .default_width(800.0)
            .default_height(600.0)
            .resizable(true)
            .collapsible(false)
            .show(ctx, |ui| {
                if let Err(err) = SettingsViewer.render(ui, &mut self.state) {
                    warn!("{}", err);
                }
            });
        self.state.settings.show_settings_window = show_settings_window;

        let mut show_pixel_viewer: bool = self.state.settings.sharable.show_pixel_viewer;
        let pixel_viewer_title = match &self.state.settings.selected_object {
            None => {
                show_pixel_viewer = false;
                "".to_string()
            }
            Some(selected_object) => {
                if let Some(frame) = self.state.stream.current_frame() {
                    selected_object
                        .rect(frame)
                        .map(|rect| {
                            format!(
                                "Pixels: {}x{} block at (x={}, y={})",
                                rect.width() as i32,
                                rect.height() as i32,
                                rect.left_top().x as i32,
                                rect.left_top().y as i32
                            )
                        })
                        .unwrap_or_default()
                } else {
                    "".to_string()
                }
            }
        };
        egui::Window::new(pixel_viewer_title)
            .id("Pixels".into())
            .open(&mut show_pixel_viewer)
            .default_width(400.0)
            .default_height(400.0)
            .resizable(true)
            .collapsible(false)
            .show(ctx, |ui| {
                if let Err(err) = DetailedPixelViewer.render(ui, &mut self.state) {
                    warn!("{}", err);
                }
            });
        self.state.settings.sharable.show_pixel_viewer = show_pixel_viewer;

        let mut show_coeffs_viewer: bool = self.state.settings.sharable.show_coeffs_viewer;
        let mut coeffs_viewer_title = "".to_string();
        match &self.state.settings.selected_object {
            None => {
                show_coeffs_viewer = false;
            }
            Some(selected_object) => {
                if let Some(frame) = self.state.stream.current_frame() {
                    if matches!(selected_object.kind, SelectedObjectKind::TransformUnit(_)) {
                        coeffs_viewer_title = selected_object
                            .rect(frame)
                            .map(|rect| {
                                format!(
                                    "Coeffs: {}x{} block at (x={}, y={})",
                                    rect.width() as i32,
                                    rect.height() as i32,
                                    rect.left_top().x as i32,
                                    rect.left_top().y as i32
                                )
                            })
                            .unwrap_or_default();
                    }
                }
            }
        };
        egui::Window::new(coeffs_viewer_title)
            .id("Coeffs".into())
            .open(&mut show_coeffs_viewer)
            .default_width(400.0)
            .default_height(400.0)
            .resizable(true)
            .collapsible(false)
            .show(ctx, |ui| {
                if let Err(err) = CoeffsViewer.render(ui, &mut self.state) {
                    warn!("{}", err);
                }
            });
        self.state.settings.sharable.show_coeffs_viewer = show_coeffs_viewer;

        if self.state.settings.persistent.update_sharable_url {
            let shared_state = self.state.settings.create_shared_settings(&self.state.stream);
            if shared_state != self.state.previous_shared_state {
                self.state.previous_shared_state = shared_state.clone();
                let window = web_sys::window().unwrap();
                let shared_settings_str = shared_state.encode();
                if let Ok(history) = window.history() {
                    if let Err(err) = history.replace_state_with_url(
                        &JsValue::NULL,
                        "AVM Analyzer",
                        Some(&format!("?{STATE_URL_QUERY_PARAM_NAME}={shared_settings_str}")),
                    ) {
                        warn!("Unable to set URL state: {err:?}");
                    }
                }
            }
        }
        // TODO(comc): Add selected tab to shared settings.
        // let find_me: Box<dyn RenderView> = Box::new(StatsViewer);
        // let idx = self.dock_state.find_tab(&find_me);
    }

    fn save(&mut self, storage: &mut dyn Storage) {
        info!("Saved settings to local storage.");
        if let Ok(saved_settings) = serde_json::to_string(&self.state.settings.persistent) {
            storage.set_string(SAVED_SETTINGS_KEY, saved_settings);
        } else {
            warn!("Error serialize saved settings.")
        }
    }

    fn auto_save_interval(&self) -> std::time::Duration {
        Duration::from_secs_f32(5.0)
    }
}
