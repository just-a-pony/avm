use serde::{Deserialize, Serialize};

use crate::{
    CodingUnitContext, CodingUnitKind, CodingUnitLocator, Frame, PartitionContext, PartitionIterator, PartitionLocator,
    Superblock, SymbolContext, SymbolRange,
};

#[derive(Copy, Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct SuperblockLocator {
    pub index: usize,
}

impl SuperblockLocator {
    pub fn new(index: usize) -> Self {
        Self { index }
    }
    pub fn try_resolve<'a>(&self, frame: &'a Frame) -> Option<SuperblockContext<'a>> {
        frame.superblocks.get(self.index).map(|superblock| SuperblockContext {
            superblock,
            frame,
            locator: *self,
        })
    }
    pub fn resolve<'a>(&self, frame: &'a Frame) -> SuperblockContext<'a> {
        self.try_resolve(frame).unwrap()
    }
}

#[derive(Copy, Clone, Debug)]
pub struct SuperblockContext<'a> {
    pub superblock: &'a Superblock,
    pub frame: &'a Frame,
    pub locator: SuperblockLocator,
}

impl<'a> SuperblockContext<'a> {
    pub fn iter_partitions(&self, kind: CodingUnitKind) -> impl Iterator<Item = PartitionContext<'a>> {
        let root = match kind {
            CodingUnitKind::Shared | CodingUnitKind::LumaOnly => self.superblock.luma_partition_tree.as_ref().unwrap(),
            CodingUnitKind::ChromaOnly => self.superblock.chroma_partition_tree.as_ref().unwrap(),
        };

        let root_locator = PartitionLocator::new(Vec::new(), kind, self.locator);
        let root_context = PartitionContext {
            partition: root,
            superblock_context: *self,
            locator: root_locator,
        };
        PartitionIterator {
            stack: vec![(root_context, 0)],
            max_depth: None,
        }
    }

    pub fn root_partition(&self, kind: CodingUnitKind) -> Option<PartitionContext<'a>> {
        self.iter_partitions(kind).next()
    }

    // Consuming self simplifies lifetime management in caller.
    pub fn iter_coding_units(self, kind: CodingUnitKind) -> impl Iterator<Item = CodingUnitContext<'a>> {
        let coding_units = match kind {
            CodingUnitKind::Shared | CodingUnitKind::LumaOnly => self.superblock.coding_units_shared.iter(),
            CodingUnitKind::ChromaOnly => self.superblock.coding_units_chroma.iter(),
        };
        coding_units
            .enumerate()
            .map(move |(index, coding_unit)| CodingUnitContext {
                coding_unit,
                superblock_context: self,
                locator: CodingUnitLocator::new(self.locator, kind, index),
            })
    }

    pub fn iter_symbols(&self, range: Option<SymbolRange>) -> impl Iterator<Item = SymbolContext<'a>> {
        let range = range.unwrap_or(SymbolRange {
            start: 0,
            end: self.superblock.symbols.len() as u32,
        });
        self.superblock.symbols[range.start as usize..range.end as usize]
            .iter()
            .map(|sym| SymbolContext {
                symbol: sym,
                info: self.frame.symbol_info.get(&sym.info_id),
                superblock: self.superblock,
            })
    }
}
