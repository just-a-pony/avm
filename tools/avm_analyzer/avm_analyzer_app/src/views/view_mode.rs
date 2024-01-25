use avm_stats::PixelType;
use serde::{Deserialize, Serialize};

use crate::settings::DistortionView;

#[derive(Default, Debug, Copy, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ViewMode {
    #[default]
    CodingFlow,
    Prediction,
    Transform,
    Filters,
    Distortion(DistortionView),
    Motion,
    Heatmap,
}

pub struct ViewSettings {
    pub show_superblocks: bool,
    pub show_coding_units: bool,
    pub show_transform_units: bool,
    pub show_prediction_modes: bool,
    pub show_transform_types: bool,
    pub show_motion_vectors: bool,
    pub pixel_type: PixelType,
    pub show_heatmap: bool,
    pub allow_coding_unit_selection: bool,
    pub allow_transform_unit_selection: bool,
}

impl std::fmt::Display for ViewMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            ViewMode::CodingFlow => "Coding Flow",
            ViewMode::Prediction => "Prediction",
            ViewMode::Transform => "Transform",
            ViewMode::Filters => "Filters",
            ViewMode::Distortion(_) => "Distortion",
            ViewMode::Motion => "Motion",
            ViewMode::Heatmap => "Heatmap",
        };
        write!(f, "{s}")
    }
}

impl ViewMode {
    pub fn view_settings(&self) -> ViewSettings {
        let show_superblocks = true;
        let show_coding_units = true;
        let mut show_transform_units = false;
        let mut show_prediction_modes: bool = false;
        let mut show_transform_types: bool = false;
        let mut show_motion_vectors: bool = false;
        let mut pixel_type = PixelType::Reconstruction;
        let mut show_heatmap = false;
        let mut allow_coding_unit_selection = true;
        let mut allow_transform_unit_selection = false;
        match self {
            ViewMode::CodingFlow => {
                pixel_type = PixelType::Reconstruction;
            }
            ViewMode::Prediction => {
                pixel_type = PixelType::Prediction;
                show_transform_units = true;
                show_prediction_modes = true;
            }
            ViewMode::Transform => {
                pixel_type = PixelType::Residual;
                show_transform_units = true;
                show_transform_types = true;
                allow_coding_unit_selection = false;
                allow_transform_unit_selection = true;
            }
            ViewMode::Filters => {
                pixel_type = PixelType::FilterDelta;
            }
            ViewMode::Distortion(distortion_view) => {
                pixel_type = match distortion_view {
                    DistortionView::Distortion => PixelType::Distortion,
                    DistortionView::Original => PixelType::Original,
                    DistortionView::Reconstruction => PixelType::Reconstruction,
                }
            }
            ViewMode::Motion => {
                show_motion_vectors = true;
                pixel_type = PixelType::Prediction;
            }
            ViewMode::Heatmap => {
                show_heatmap = true;
            }
        }

        ViewSettings {
            show_superblocks,
            show_coding_units,
            show_transform_units,
            show_prediction_modes,
            show_transform_types,
            show_motion_vectors,
            pixel_type,
            show_heatmap,
            allow_coding_unit_selection,
            allow_transform_unit_selection,
        }
    }
}
