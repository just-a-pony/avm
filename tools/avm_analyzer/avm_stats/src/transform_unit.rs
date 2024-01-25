use serde::{Deserialize, Serialize};

use crate::{CodingUnitContext, CodingUnitLocator, Frame, Plane, ProtoEnumMapping, TransformUnit};

// TX blocks larger than 32x32 have all coefficients other than the top-left 32x32 set to 0.
pub const MAX_COEFFS_SIZE: usize = 32;

impl TransformUnit {
    pub fn primary_tx_type_or_skip(&self, frame: &Frame) -> String {
        let tx_type = self.tx_type;
        // Only lower 4-bits used for primary transform. Upper bits are IST.
        let tx_type = tx_type & 0xF;
        if self.skip == 1 {
            "SKIP".to_owned()
        } else {
            frame
                .enum_lookup(ProtoEnumMapping::TransformType, tx_type)
                .unwrap_or(format!("UNKNOWN ({tx_type})"))
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct TransformUnitLocator {
    pub coding_unit: CodingUnitLocator,
    pub plane: Plane,
    /// Index of this tranform unit with its parent.
    pub index: usize,
}

// Note: Converting plane to a usize does not automatically get the correct index into a coding unit's transform planes.
// e.g., in SDP mode, the chroma coding units will have two planes, but with plane IDs (1, 2), not (0, 1).

impl TransformUnitLocator {
    pub fn new(coding_unit: CodingUnitLocator, plane: Plane, index: usize) -> Self {
        Self {
            coding_unit,
            plane,
            index,
        }
    }

    pub fn try_resolve<'a>(&self, frame: &'a Frame) -> Option<TransformUnitContext<'a>> {
        let coding_unit_context = self.coding_unit.try_resolve(frame)?;
        let plane_index = coding_unit_context.coding_unit.plane_index(self.plane).ok()?;
        let transform_unit = coding_unit_context
            .coding_unit
            .transform_planes
            .get(plane_index)?
            .transform_units
            .get(self.index)?;
        Some(TransformUnitContext {
            transform_unit,
            coding_unit_context,
            locator: *self,
        })
    }

    pub fn resolve<'a>(&self, frame: &'a Frame) -> TransformUnitContext<'a> {
        self.try_resolve(frame).unwrap()
    }
}

#[derive(Copy, Clone)]
pub struct TransformUnitContext<'a> {
    pub transform_unit: &'a TransformUnit,
    pub coding_unit_context: CodingUnitContext<'a>,
    pub locator: TransformUnitLocator,
}
