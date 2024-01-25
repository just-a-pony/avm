use itertools::{Itertools, MinMaxResult};
use serde::{Deserialize, Serialize};

use crate::{CodingUnitKind, Frame, FrameError, Spatial};
// TODO(comc): Allow filtering by symbol type.
// TODO(comc): Consider some way of handling this for TIP frames, e.g. weighted average of the two reference frames?
pub const DEFAULT_HISTROGRAM_BUCKETS: usize = 32;

#[derive(Debug, Serialize, Deserialize, Clone, PartialEq)]
pub struct HeatmapSettings {
    pub symbol_filter: String,
    pub histogram_buckets: usize,
    pub coding_unit_kind: CodingUnitKind,
}

impl Default for HeatmapSettings {
    fn default() -> Self {
        Self {
            symbol_filter: "".to_string(),
            histogram_buckets: DEFAULT_HISTROGRAM_BUCKETS,
            coding_unit_kind: CodingUnitKind::Shared,
        }
    }
}

#[derive(Clone)]
pub struct Heatmap {
    pub width: usize,
    pub height: usize,
    pub data: Vec<u8>,
    pub min_value: f32,
    pub max_value: f32,
    pub bucket_width: f32,
    pub histogram: Vec<f32>,
}

pub fn calculate_heatmap(frame: &Frame, settings: &HeatmapSettings) -> Result<Heatmap, FrameError> {
    let width = frame.width() as usize;
    let height = frame.height() as usize;
    let mut heatmap = vec![0.0; width * height];
    // TODO(comc): Option to iterate over both luma and chroma symbols and add them up (for SDP frames).
    let bit_rects = frame.iter_coding_units(settings.coding_unit_kind).map(|ctx| {
        let cu = ctx.coding_unit;
        let bits = ctx.iter_symbols().filter_map(|sym| {
            if settings.symbol_filter.is_empty() || sym.info.unwrap().source_function.contains(&settings.symbol_filter)
            {
                Some(sym.symbol.bits)
            } else {
                None
            }
        });
        let sum: f32 = bits.sum();
        let y0 = cu.y() as usize;
        let y1 = y0 + cu.height() as usize;
        let x0 = cu.x() as usize;
        let x1 = x0 + cu.width() as usize;

        Ok::<_, FrameError>((y0.min(height), y1.min(height), x0.min(width), x1.min(width), sum))
    });
    for bit_rect in bit_rects.flatten() {
        let (y0, y1, x0, x1, bits) = bit_rect;
        let area = ((y1 - y0) * (x1 - x0)) as f32;
        for y in y0..y1 {
            for x in x0..x1 {
                let index = y * width + x;
                heatmap[index] = bits / area;
            }
        }
    }
    let mut min = 0.0;
    let mut max = 255.0;
    match heatmap.iter().minmax() {
        MinMaxResult::NoElements | MinMaxResult::OneElement(_) => {}
        MinMaxResult::MinMax(&min_v, &max_v) => {
            min = min_v;
            max = max_v;
        }
    };
    let mut histogram = vec![0.0; settings.histogram_buckets];
    heatmap.iter().for_each(|&x| {
        let frac = (x - min) / (max - min);
        let bucket = (frac * settings.histogram_buckets as f32) as usize;
        let bucket = bucket.min(settings.histogram_buckets - 1);
        histogram[bucket] += 1.0;
    });
    let heatmap: Vec<u8> = heatmap
        .iter()
        .map(|&x| (255.0 * (x - min) / (max - min)) as u8)
        .collect();
    Ok(Heatmap {
        width,
        height,
        data: heatmap,
        min_value: min,
        max_value: max,
        bucket_width: (max - min) / settings.histogram_buckets as f32,
        histogram,
    })
}
