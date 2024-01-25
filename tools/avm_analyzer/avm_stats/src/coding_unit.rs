use crate::{
    CodingUnit, Frame, FrameError, PartitionContext, Plane, PredictionParams, ProtoEnumMapping, SuperblockContext,
    SuperblockLocator, SymbolContext, SymbolRange, TransformUnitContext, TransformUnitLocator,
};

use serde::{Deserialize, Serialize};
use FrameError::BadCodingUnit;
impl CodingUnit {
    pub fn plane_index(&self, plane: Plane) -> Result<usize, FrameError> {
        match plane {
            Plane::Y => Ok(0),
            Plane::U | Plane::V => {
                match self.transform_planes.len() {
                    // Split luma and chroma partition trees
                    2 => Ok(plane.to_usize() - 1),
                    // Unified luma and chroma partition tree
                    3 => Ok(plane.to_usize()),
                    _ => Err(BadCodingUnit(format!(
                        "Unexpected number of transform planes: got {}, expected 2 or 3 for plane {plane:?}",
                        self.transform_planes.len()
                    ))),
                }
            }
        }
    }

    pub fn has_chroma(&self) -> Result<bool, FrameError> {
        let num_transform_planes = self.transform_planes.len();
        match num_transform_planes {
            2 | 3 => Ok(true),
            1 => Ok(false),
            _ => Err(BadCodingUnit(format!(
                "Unexpected number of transform planes: {num_transform_planes}"
            ))),
        }
    }

    pub fn has_luma(&self) -> Result<bool, FrameError> {
        let num_transform_planes = self.transform_planes.len();
        match num_transform_planes {
            1 | 3 => Ok(true),
            2 => Ok(false),
            _ => Err(BadCodingUnit(format!(
                "Unexpected number of transform planes: {num_transform_planes}"
            ))),
        }
    }

    pub fn get_prediction_mode(&self) -> Result<&PredictionParams, FrameError> {
        self.prediction_mode
            .as_ref()
            .ok_or(BadCodingUnit("Missing prediction mode.".into()))
    }

    pub fn get_symbol_range(&self) -> Result<&SymbolRange, FrameError> {
        self.symbol_range
            .as_ref()
            .ok_or(BadCodingUnit("Missing symbol range.".into()))
    }

    pub fn lookup_mode_name(&self, frame: &Frame) -> Result<String, FrameError> {
        let mode = self.get_prediction_mode()?;
        frame.enum_lookup(ProtoEnumMapping::PredictionMode, mode.mode)
    }

    pub fn luma_mode_angle_delta(&self, frame: &Frame) -> Option<i32> {
        if let Ok(mode) = self.lookup_mode_name(frame) {
            if mode.ends_with("_PRED") {
                return self.prediction_mode.as_ref().map(|mode| mode.angle_delta);
            }
        }
        None
    }

    pub fn lookup_uv_mode_name(&self, frame: &Frame) -> Result<String, FrameError> {
        let mode = self.get_prediction_mode()?;
        frame.enum_lookup(ProtoEnumMapping::UvPredictionMode, mode.uv_mode)
    }

    pub fn chroma_mode_angle_delta(&self, frame: &Frame) -> Option<i32> {
        if let Ok(mode) = self.lookup_uv_mode_name(frame) {
            if mode.ends_with("_PRED") {
                return self.prediction_mode.as_ref().map(|mode| mode.uv_angle_delta);
            }
        }
        None
    }

    pub fn lookup_motion_vector_precision_name(&self, frame: &Frame) -> Result<String, FrameError> {
        let mode = self.get_prediction_mode()?;
        frame.enum_lookup(ProtoEnumMapping::MotionVectorPrecision, mode.motion_vector_precision)
    }
}

/// Which planes this coding unit contains.
#[derive(Copy, Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub enum CodingUnitKind {
    /// Coding unit contains all three planes. Equivalent to the shared partition tree type.
    Shared,
    /// Coding unit contains only luma.
    LumaOnly,
    /// Coding unit contains only chroma.
    ChromaOnly,
}

/// Index of a coding unit within its parent superblock.
#[derive(Copy, Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct CodingUnitLocator {
    /// Index of parent superblock within the frame.
    pub superblock: SuperblockLocator,
    /// Either Shared or ChromaOnly. Note that LumaOnly refers to the same underlying buffer as Shared.
    pub kind: CodingUnitKind,
    /// Index of coding unit with its parent superblock.
    pub index: usize,
}

impl CodingUnitLocator {
    pub fn new(superblock: SuperblockLocator, kind: CodingUnitKind, index: usize) -> Self {
        Self {
            superblock,
            kind,
            index,
        }
    }
    /// Convert this index into a `CodingUnitContext`.
    pub fn try_resolve<'a>(&self, frame: &'a Frame) -> Option<CodingUnitContext<'a>> {
        let superblock_context = self.superblock.try_resolve(frame)?;
        let coding_unit = match self.kind {
            CodingUnitKind::Shared => superblock_context.superblock.coding_units_shared.get(self.index),
            CodingUnitKind::LumaOnly => superblock_context.superblock.coding_units_shared.get(self.index),
            CodingUnitKind::ChromaOnly => superblock_context.superblock.coding_units_chroma.get(self.index),
        };

        coding_unit.map(|coding_unit| CodingUnitContext {
            coding_unit,
            superblock_context,
            locator: *self,
        })
    }

    pub fn resolve<'a>(&self, frame: &'a Frame) -> CodingUnitContext<'a> {
        self.try_resolve(frame).unwrap()
    }
}

/// Context about a coding unit during iteration.
#[derive(Copy, Clone)]
pub struct CodingUnitContext<'a> {
    /// Coding unit being iterated over.
    pub coding_unit: &'a CodingUnit,
    /// Superblock that owns this coding unit.
    pub superblock_context: SuperblockContext<'a>,
    /// The index of this coding unit within its parent superblock.
    pub locator: CodingUnitLocator,
}

impl<'a> CodingUnitContext<'a> {
    pub fn iter_symbols(&self) -> impl Iterator<Item = SymbolContext<'a>> {
        let symbol_range = self.coding_unit.symbol_range.clone().unwrap_or_default();
        self.superblock_context.iter_symbols(Some(symbol_range))
    }

    pub fn total_bits(&self) -> f32 {
        self.iter_symbols().map(|sym| sym.symbol.bits).sum()
    }

    pub fn iter_transform_units(self, plane: Plane) -> impl Iterator<Item = TransformUnitContext<'a>> {
        let plane_index = self.coding_unit.plane_index(plane);
        self.coding_unit.transform_planes[plane_index.unwrap()]
            .transform_units
            .iter()
            .enumerate()
            .map(move |(index, transform_unit)| TransformUnitContext {
                transform_unit,
                coding_unit_context: self,
                locator: TransformUnitLocator::new(self.locator, plane, index),
            })
    }

    pub fn find_parent_partition(&self) -> Option<PartitionContext<'a>> {
        self.superblock_context
            .root_partition(self.locator.kind)
            .and_then(|root| root.find_coding_unit_parent(self.coding_unit))
    }
}
