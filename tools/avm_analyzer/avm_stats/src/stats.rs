use std::collections::{HashMap, HashSet};

use itertools::Itertools;

use ordered_float::OrderedFloat;
use serde::{Deserialize, Serialize};

use crate::{CodingUnitKind, Frame, Plane, PlaneType, ProtoEnumMapping, Spatial};

#[derive(Copy, Clone, PartialEq, Eq, Debug, Deserialize, Serialize)]
pub enum FrameStatistic {
    LumaModes,
    ChromaModes,
    BlockSizes,
    Symbols,
    PartitionSplit,
}

#[derive(Default, Debug, Deserialize, Serialize, PartialEq)]
pub struct StatsFilter {
    pub include: Vec<String>,
    pub exclude: Vec<String>,
}

impl StatsFilter {
    fn from_comma_separated(include: &str, exclude: &str) -> Self {
        Self {
            include: include
                .split(',')
                .map(|s| s.trim().to_string())
                .filter(|s| !s.is_empty())
                .collect(),
            exclude: exclude
                .split(',')
                .map(|s| s.trim().to_string())
                .filter(|s| !s.is_empty())
                .collect(),
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug, Deserialize, Serialize)]
pub enum StatSortMode {
    Unsorted,
    ByName,
    ByValue,
}
impl StatSortMode {
    pub fn name(&self) -> &'static str {
        match self {
            Self::Unsorted => "Unsorted",
            Self::ByName => "By name",
            Self::ByValue => "By value",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Deserialize, Serialize)]
pub struct StatsSettings {
    pub sort_by: StatSortMode,
    // Using separate bool + value fields rather than an Option to make the UI design a bit more intuitive (e.g. checkbox + disabled number input).
    pub apply_limit_count: bool,
    pub limit_count: usize,
    pub apply_limit_frac: bool,
    pub limit_frac: f32,
    pub include_filter: String,
    pub exclude_filter: String,
    pub include_filter_exact_match: bool,
    pub exclude_filter_exact_match: bool,
    pub show_relative_total: bool,
    // Comma separated list of block sizes to include for partition split stats.
    pub partition_split_block_sizes: String,
}

impl Default for StatsSettings {
    fn default() -> Self {
        Self {
            sort_by: StatSortMode::ByValue,
            apply_limit_count: false,
            limit_count: 20,
            apply_limit_frac: false,
            limit_frac: 0.01,
            include_filter: "".into(),
            exclude_filter: "".into(),
            include_filter_exact_match: false,
            exclude_filter_exact_match: false,
            show_relative_total: false,
            partition_split_block_sizes: "".into(),
        }
    }
}

#[derive(Debug)]
pub struct Sample {
    pub name: String,
    pub value: f64,
}
impl Sample {
    pub fn new(name: String, value: f64) -> Self {
        Self { name, value }
    }
}

impl FrameStatistic {
    fn luma_modes(&self, frame: &Frame) -> HashMap<String, f64> {
        let modes = frame.iter_coding_units(CodingUnitKind::Shared).map(|ctx| {
            let cu = ctx.coding_unit;
            let prediction_mode = cu.prediction_mode.as_ref().unwrap();
            frame
                .enum_lookup(ProtoEnumMapping::PredictionMode, prediction_mode.mode)
                .unwrap_or("UNKNOWN".into())
        });
        let mut modes_map: HashMap<String, f64> = HashMap::new();

        for mode in modes {
            *modes_map.entry(mode).or_default() += 1.0;
        }

        modes_map
    }

    fn chroma_modes(&self, frame: &Frame) -> HashMap<String, f64> {
        let kind = frame.coding_unit_kind(PlaneType::Planar(Plane::U));
        let modes = frame.iter_coding_units(kind).map(|ctx| {
            let cu = ctx.coding_unit;
            let prediction_mode = cu.prediction_mode.as_ref().unwrap();
            frame
                .enum_lookup(ProtoEnumMapping::UvPredictionMode, prediction_mode.uv_mode)
                .unwrap_or("UNKNOWN".into())
        });
        let mut modes_map: HashMap<String, f64> = HashMap::new();

        for mode in modes {
            *modes_map.entry(mode).or_default() += 1.0;
        }

        modes_map
    }

    fn block_sizes(&self, frame: &Frame) -> HashMap<String, f64> {
        let sizes = frame.iter_coding_units(CodingUnitKind::Shared).map(|ctx| {
            let cu = ctx.coding_unit;
            let w = cu.width();
            let h = cu.height();
            format!("{w}x{h}")
        });
        let mut sizes_map: HashMap<String, f64> = HashMap::new();

        for size in sizes {
            *sizes_map.entry(size).or_default() += 1.0;
        }
        sizes_map
    }

    fn partition_split(&self, frame: &Frame, settings: &StatsSettings) -> HashMap<String, f64> {
        // TODO(comc): Add settings option for partition kind.
        let filter = StatsFilter::from_comma_separated(&settings.partition_split_block_sizes, "");
        let splits = frame.iter_partitions(CodingUnitKind::Shared).filter_map(|ctx| {
            let partition = ctx.partition;
            let size = partition.size_name();
            if !filter.include.is_empty() && !filter.include.iter().any(|incl| &size == incl) {
                return None;
            }
            let partition_type = frame
                .enum_lookup(ProtoEnumMapping::PartitionType, partition.partition_type)
                .unwrap_or("UNKNOWN".into());
            Some(partition_type)
        });
        let mut splits_map: HashMap<String, f64> = HashMap::new();

        for split in splits {
            *splits_map.entry(split).or_default() += 1.0;
        }
        splits_map
    }

    fn symbols(&self, frame: &Frame) -> HashMap<String, f64> {
        let mut symbols: HashMap<String, f64> = HashMap::new();
        // TODO(comc): Use iter_symbols. Add iter_symbols method for partition blocks as well.
        let sbs = frame.iter_superblocks().map(|sb_ctx| {
            let sb = sb_ctx.superblock;
            let mut symbols_sb: HashMap<String, f64> = HashMap::new();
            for symbol in sb.symbols.iter() {
                let info = symbol.info_id;
                let info = &frame.symbol_info[&info];
                let name = info.source_function.clone();
                let bits = symbol.bits;
                *symbols_sb.entry(name.clone()).or_default() += bits as f64;
            }
            symbols_sb
        });
        for symbols_sb in sbs {
            for (name, bits) in symbols_sb {
                *symbols.entry(name.clone()).or_default() += bits;
            }
        }
        symbols
    }

    fn apply_settings(&self, mapping: HashMap<String, f64>, settings: &StatsSettings) -> Vec<Sample> {
        let mut samples: Vec<_> = mapping
            .into_iter()
            .map(|(name, value)| Sample::new(name, value))
            .collect();
        let filter: StatsFilter = StatsFilter::from_comma_separated(&settings.include_filter, &settings.exclude_filter);
        let total: f64 = samples.iter().map(|sample| sample.value).sum();
        let mut other = 0.0;
        samples.retain(|Sample { name, value }| {
            let mut keep = true;
            // TODO(comc): Make this a method of StatsFilter.
            if !filter.include.is_empty() {
                if settings.include_filter_exact_match {
                    if !filter.include.contains(name) {
                        keep = false;
                    }
                } else if !filter.include.iter().any(|incl| name.contains(incl)) {
                    keep = false;
                }
            }
            if settings.exclude_filter_exact_match {
                if filter.exclude.contains(name) {
                    keep = false
                }
            } else if filter.exclude.iter().any(|excl| name.contains(excl)) {
                keep = false;
            }
            if !keep {
                other += value;
            }
            keep
        });

        let filtered_total: f64 = samples.iter().map(|sample| sample.value).sum();

        let top_n = if settings.apply_limit_count {
            let top_n: HashSet<_> = samples
                .iter()
                .sorted_by_key(|sample| (OrderedFloat(sample.value), &sample.name)) // name used as a tie-breaker.
                .rev()
                .map(|sample| sample.name.clone())
                .take(settings.limit_count)
                .collect();
            Some(top_n)
        } else {
            None
        };

        samples.retain(|Sample { name, value }| {
            let mut keep = true;
            if settings.apply_limit_frac {
                let frac = if settings.show_relative_total {
                    *value / filtered_total
                } else {
                    *value / total
                };
                if frac < settings.limit_frac as f64 {
                    keep = false;
                }
            }

            if let Some(top_n) = &top_n {
                if !top_n.contains(name) {
                    keep = false;
                }
            }

            if !keep {
                other += value;
            }
            keep
        });

        if !settings.show_relative_total && other > 0.0 {
            samples.push(Sample::new("Other".into(), other));
        }
        match settings.sort_by {
            StatSortMode::ByName => samples
                .into_iter()
                .sorted_by_key(|sample| sample.name.clone())
                .collect(),
            StatSortMode::ByValue => samples
                .into_iter()
                .sorted_by_key(|sample| (OrderedFloat(sample.value), sample.name.clone())) // name used as a tie-breaker.
                .collect(),
            StatSortMode::Unsorted => samples,
        }
    }

    pub fn calculate(&self, frame: &Frame, settings: &StatsSettings) -> Vec<Sample> {
        let mapping = match self {
            FrameStatistic::LumaModes => self.luma_modes(frame),
            FrameStatistic::ChromaModes => self.chroma_modes(frame),
            FrameStatistic::BlockSizes => self.block_sizes(frame),
            FrameStatistic::Symbols => self.symbols(frame),
            FrameStatistic::PartitionSplit => self.partition_split(frame, settings),
        };
        self.apply_settings(mapping, settings)
    }

    pub fn name(&self) -> &'static str {
        match self {
            FrameStatistic::LumaModes => "Luma modes",
            FrameStatistic::ChromaModes => "Chroma modes",
            FrameStatistic::BlockSizes => "Block sizes",
            FrameStatistic::Symbols => "Symbols",
            FrameStatistic::PartitionSplit => "Partition split",
        }
    }
}

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn test_exclude_filter() {
        let mapping: HashMap<String, f64> = [("ABC", 1.0), ("AAA", 2.0)]
            .iter()
            .map(|(k, v)| (k.to_string(), *v))
            .collect();
        let settings = StatsSettings {
            exclude_filter: "A".into(),
            exclude_filter_exact_match: true,
            show_relative_total: true,
            ..Default::default()
        };
        let samples = FrameStatistic::Symbols.apply_settings(mapping, &settings);
        assert_eq!(samples.len(), 2);
    }
}
