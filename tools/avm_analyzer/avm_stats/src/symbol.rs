use crate::{Superblock, Symbol, SymbolInfo};
use once_cell::sync::Lazy;

pub static MISSING_SYMBOL_INFO: Lazy<SymbolInfo> = Lazy::new(|| SymbolInfo {
    id: -1,
    source_file: "UNKNOWN".into(),
    source_line: -1,
    source_function: "UNKNOWN".into(),
    tags: Vec::new(),
});

#[derive(Copy, Clone)]
pub struct SymbolContext<'a> {
    pub symbol: &'a Symbol,
    pub info: Option<&'a SymbolInfo>,
    pub superblock: &'a Superblock,
}

impl<'a> SymbolContext<'a> {
    // pub fn from_coding_unit_context(
    //     transform_unit: &'a TransformUnit,
    //     plane: Plane,
    //     transform_unit_index: usize,
    //     coding_unit_context: CodingUnitContext<'a>,
    // ) -> Self {
    //     let index = TransformUnitIndex::new(coding_unit_context.index, plane, transform_unit_index);
    //     Self {
    //         transform_unit,
    //         coding_unit: coding_unit_context.coding_unit,
    //         superblock: coding_unit_context.superblock,
    //         index,
    //     }
    // }
}
