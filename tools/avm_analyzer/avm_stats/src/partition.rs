use serde::{Deserialize, Serialize};

use crate::{CodingUnit, Partition, Spatial, SuperblockContext};

use crate::{CodingUnitKind, Frame, SuperblockLocator, SymbolContext};

pub struct PartitionIterator<'a> {
    pub stack: Vec<(PartitionContext<'a>, usize)>,
    pub max_depth: Option<usize>,
}

impl<'a> PartitionIterator<'a> {
    fn new(root: PartitionContext<'a>) -> Self {
        Self {
            stack: vec![(root, 0)],
            max_depth: None,
        }
    }

    fn with_max_depth(root: PartitionContext<'a>, max_depth: usize) -> Self {
        Self {
            stack: vec![(root, 0)],
            max_depth: Some(max_depth),
        }
    }
}

impl<'a> Iterator for PartitionIterator<'a> {
    type Item = PartitionContext<'a>;
    fn next(&mut self) -> Option<Self::Item> {
        let (current, depth) = self.stack.pop()?;
        let max_depth = self.max_depth.unwrap_or(usize::MAX);
        let child_depth = depth + 1;
        if child_depth <= max_depth {
            self.stack
                .extend(current.partition.children.iter().enumerate().rev().map(|(i, child)| {
                    let mut child_context = current.clone();
                    child_context.partition = child;
                    child_context.locator.path_indices.push(i);
                    (child_context, child_depth)
                }));
        }
        Some(current)
    }
}

// TODO(comc): Handle shared vs luma differently at the shared level of the partition tree.
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct PartitionLocator {
    pub path_indices: Vec<usize>,
    pub kind: CodingUnitKind,
    pub superblock: SuperblockLocator,
}

impl PartitionLocator {
    pub fn new(path_indices: Vec<usize>, kind: CodingUnitKind, superblock: SuperblockLocator) -> Self {
        Self {
            path_indices,
            kind,
            superblock,
        }
    }

    pub fn try_resolve<'a>(&self, frame: &'a Frame) -> Option<PartitionContext<'a>> {
        let superblock_context = self.superblock.try_resolve(frame)?;
        let mut current = match self.kind {
            CodingUnitKind::Shared | CodingUnitKind::LumaOnly => {
                superblock_context.superblock.luma_partition_tree.as_ref()?
            }
            CodingUnitKind::ChromaOnly => superblock_context.superblock.chroma_partition_tree.as_ref()?,
        };
        for index in self.path_indices.iter() {
            if let Some(child) = current.children.get(*index) {
                current = child;
            } else {
                return None;
            }
        }
        Some(PartitionContext {
            partition: current,
            superblock_context,
            locator: self.clone(),
        })
    }

    pub fn resolve(self, frame: &Frame) -> PartitionContext {
        self.try_resolve(frame).unwrap()
    }

    pub fn is_root(&self) -> bool {
        self.path_indices.is_empty()
    }

    pub fn parent(&self) -> Option<PartitionLocator> {
        if self.is_root() {
            None
        } else {
            let mut parent = self.clone();
            parent.path_indices.pop();
            Some(parent)
        }
    }
}

/// Context about a partition block during iteration.
#[derive(Clone)]
pub struct PartitionContext<'a> {
    /// Partition block being iterated over.
    pub partition: &'a Partition,
    /// Superblock that owns this partition block.
    pub superblock_context: SuperblockContext<'a>,
    /// The index of this partition block within its parent superblock.
    pub locator: PartitionLocator,
}

impl<'a> PartitionContext<'a> {
    // Note: Also yields self.
    pub fn iter(&self) -> impl Iterator<Item = PartitionContext<'a>> {
        PartitionIterator::new(self.clone())
    }

    // Note: Also yields self.
    pub fn iter_with_max_depth(&self, max_depth: usize) -> impl Iterator<Item = PartitionContext<'a>> {
        PartitionIterator::with_max_depth(self.clone(), max_depth)
    }

    pub fn iter_direct_children(&self) -> impl Iterator<Item = PartitionContext<'a>> {
        self.iter_with_max_depth(1).skip(1)
    }

    pub fn iter_symbols(&self) -> impl Iterator<Item = SymbolContext<'a>> {
        let symbol_range = self.partition.symbol_range.clone().unwrap_or_default();
        self.superblock_context.iter_symbols(Some(symbol_range))
    }

    pub fn find_coding_unit_parent(&self, coding_unit: &'a CodingUnit) -> Option<PartitionContext<'a>> {
        if self.partition.rect() == coding_unit.rect() {
            return Some(self.clone());
        }
        for child in self.iter_direct_children() {
            if child.partition.rect().contains_rect(coding_unit.rect()) {
                return child.find_coding_unit_parent(coding_unit);
            }
        }
        None
    }

    pub fn is_root(&self) -> bool {
        self.locator.is_root()
    }
}
