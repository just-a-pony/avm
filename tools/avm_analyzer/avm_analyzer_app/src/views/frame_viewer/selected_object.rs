use avm_stats::{
    CodingUnitKind, CodingUnitLocator, Frame, FrameError, PartitionLocator, Plane, ProtoEnumMapping, Spatial,
    SuperblockLocator, SymbolContext, TransformUnitLocator, MISSING_SYMBOL_INFO, MOTION_VECTOR_PRECISION,
};
use egui::emath::Rect;
use serde::{Deserialize, Serialize};

#[derive(Debug)]
pub enum CachedInfo<T> {
    Missing,
    Calculated(T),
}

#[derive(Clone, Debug)]
pub struct SymbolEntry {
    pub func: String,
    pub bits: f32,
    pub tags: Vec<String>,
    pub file: String,
    pub line: i32,
    pub value: i32,
}
impl SymbolEntry {
    pub fn new(ctx: SymbolContext) -> Self {
        let bits = ctx.symbol.bits;
        let value = ctx.symbol.value;
        let sym_info = ctx.info.unwrap_or(&MISSING_SYMBOL_INFO);
        let func: String = sym_info.source_function.clone();
        let file = sym_info.source_file.clone();
        let line = sym_info.source_line;
        let tags = sym_info.tags.clone();

        Self {
            func,
            bits,
            tags,
            file,
            line,
            value,
        }
    }
}

#[derive(Debug)]
pub struct FieldEntry {
    pub name: String,
    pub value: String,
}

impl FieldEntry {
    fn new(name: String, value: String) -> Self {
        Self { name, value }
    }
}

#[derive(Debug)]
pub struct SelectedObjectInfo {
    pub fields: Vec<FieldEntry>,
    pub symbols: Vec<SymbolEntry>,
}

impl SelectedObjectInfo {
    fn get_symbols_coding_unit(locator: &CodingUnitLocator, frame: &Frame) -> Vec<SymbolEntry> {
        let Some(ctx) = locator.try_resolve(frame) else {
            return Vec::new();
        };
        let symbols: Vec<SymbolEntry> = ctx.iter_symbols().map(|sym| SymbolEntry::new(sym)).collect();
        symbols
    }

    fn get_symbols_partition(locator: &PartitionLocator, frame: &Frame) -> Vec<SymbolEntry> {
        let Some(ctx) = locator.try_resolve(frame) else {
            return Vec::new();
        };
        let symbols: Vec<SymbolEntry> = ctx.iter_symbols().map(|sym| SymbolEntry::new(sym)).collect();
        symbols
    }

    fn get_symbols_superblock(locator: &SuperblockLocator, frame: &Frame) -> Vec<SymbolEntry> {
        let Some(ctx) = locator.try_resolve(frame) else {
            return Vec::new();
        };
        let symbols: Vec<SymbolEntry> = ctx.iter_symbols(None).map(|sym| SymbolEntry::new(sym)).collect();
        symbols
    }

    fn get_symbols_transform_unit(_locator: &TransformUnitLocator, _frame: &Frame) -> Vec<SymbolEntry> {
        // TODO(comc): Transform units don't currently have symbol info tracked.
        Vec::new()
    }

    fn calculate_coding_unit(locator: &CodingUnitLocator, frame: &Frame) -> Result<Vec<FieldEntry>, FrameError> {
        let ctx = locator
            .try_resolve(frame)
            .ok_or(FrameError::Internal("Invalid coding unit locator".into()))?;
        let coding_unit = ctx.coding_unit;

        let mut fields = Vec::new();

        fields.push(FieldEntry::new("Type".into(), "Coding block".into()));
        let width = coding_unit.width();
        let height = coding_unit.height();
        fields.push(FieldEntry::new("Width".into(), width.to_string()));
        fields.push(FieldEntry::new("Height".into(), height.to_string()));
        let x = coding_unit.x();
        let y = coding_unit.y();
        fields.push(FieldEntry::new("Position".into(), format!("(x={}, y={})", x, y)));
        if coding_unit.has_luma()? {
            let mode_name = coding_unit.lookup_mode_name(frame)?;
            let mode_is_directional = mode_name.ends_with("_PRED");
            fields.push(FieldEntry::new("Prediction Mode".into(), mode_name));
            // TODO(comc): Make this a method.
            if mode_is_directional {
                if let Some(delta) = coding_unit.luma_mode_angle_delta(frame) {
                    fields.push(FieldEntry::new("Angle delta".into(), delta.to_string()));
                }
            } else {
                let motion_mode = coding_unit.get_prediction_mode()?.motion_mode;
                if let Ok(motion_mode_name) = frame.enum_lookup(ProtoEnumMapping::MotionMode, motion_mode) {
                    // TODO(comc): Make this a method.
                    let is_compound = motion_mode_name.contains('_');
                    fields.push(FieldEntry::new("Motion mode".into(), motion_mode_name));
                    if let Ok(mv_prec_name) = coding_unit.lookup_motion_vector_precision_name(frame) {
                        fields.push(FieldEntry::new("MV precision".into(), mv_prec_name));
                    }
                    let num_mvs = if is_compound { 2 } else { 1 };
                    for i in 0..num_mvs {
                        if let Some(mv) = coding_unit.get_prediction_mode()?.motion_vectors.get(i as usize) {
                            let ref_frame = mv.ref_frame;
                            if ref_frame == -1 {
                                continue;
                            }
                            let dx = mv.dx as f32 / MOTION_VECTOR_PRECISION;
                            let dy = mv.dy as f32 / MOTION_VECTOR_PRECISION;
                            let mut order_hint = mv.ref_frame_order_hint.to_string();
                            if mv.ref_frame_is_tip {
                                order_hint = "TIP".to_string();
                            }
                            fields.push(FieldEntry::new(
                                "Motion vector".into(),
                                format!("{} ({}): dx={}, dy={}", ref_frame, order_hint, dx, dy),
                            ));
                        }
                    }
                }
            }
        }
        if coding_unit.has_chroma()? {
            let uv_mode = coding_unit.lookup_uv_mode_name(frame)?;
            if uv_mode != "UV_MODE_INVALID" {
                fields.push(FieldEntry::new("UV Prediction Mode".into(), uv_mode));

                if let Some(delta) = coding_unit.chroma_mode_angle_delta(frame) {
                    fields.push(FieldEntry::new("UV angle delta".into(), delta.to_string()));
                }
            }
        }

        // TODO(comc): Disambiguate skip mode vs skip txfm.
        fields.push(FieldEntry::new("Skip Mode".into(), coding_unit.skip.to_string()));
        fields.push(FieldEntry::new(
            "Use Intra BC".into(),
            coding_unit.get_prediction_mode()?.use_intrabc.to_string(),
        ));
        fields.push(FieldEntry::new("QIndex".into(), coding_unit.qindex.to_string()));
        let bits = ctx.total_bits();
        fields.push(FieldEntry::new("Bits".into(), bits.to_string()));
        let num_transform_units = coding_unit.transform_planes[0].transform_units.len();
        fields.push(FieldEntry::new(
            "Transform units".into(),
            num_transform_units.to_string(),
        ));
        Ok(fields)
    }

    fn calculate_transform_unit(locator: &TransformUnitLocator, frame: &Frame) -> Result<Vec<FieldEntry>, FrameError> {
        let ctx = locator
            .try_resolve(frame)
            .ok_or(FrameError::Internal("Invalid transform unit locator".into()))?;
        let transform_unit = ctx.transform_unit;
        let mut fields = Vec::new();
        let width = transform_unit.width();
        let height = transform_unit.height();
        fields.push(FieldEntry::new("Type".into(), "Transform block".into()));
        fields.push(FieldEntry::new("Width".into(), width.to_string()));
        fields.push(FieldEntry::new("Height".into(), height.to_string()));
        let x = transform_unit.x();
        let y = transform_unit.y();
        fields.push(FieldEntry::new("Position".into(), format!("(x={}, y={})", x, y)));

        let tx_type = transform_unit.primary_tx_type_or_skip(frame);
        fields.push(FieldEntry::new("TX type".into(), tx_type));

        Ok(fields)
    }

    fn calculate_partition(locator: &PartitionLocator, frame: &Frame) -> Result<Vec<FieldEntry>, FrameError> {
        let ctx = locator
            .try_resolve(frame)
            .ok_or(FrameError::Internal("Invalid partition locator".into()))?;
        let partition = ctx.partition;
        let mut fields = Vec::new();
        let width = partition.width();
        let height = partition.height();
        fields.push(FieldEntry::new("Type".into(), "Partition block".into()));
        fields.push(FieldEntry::new("Width".into(), width.to_string()));
        fields.push(FieldEntry::new("Height".into(), height.to_string()));
        let x = partition.x();
        let y = partition.y();
        fields.push(FieldEntry::new("Position".into(), format!("(x={}, y={})", x, y)));

        let partition_type = partition.partition_type;
        let partition_type_name = frame
            .enum_lookup(ProtoEnumMapping::PartitionType, partition_type)
            .unwrap_or("UNKNOWN".into());
        fields.push(FieldEntry::new("Partition type".into(), partition_type_name));
        Ok(fields)
    }

    fn calculate_superblock(locator: &SuperblockLocator, frame: &Frame) -> Result<Vec<FieldEntry>, FrameError> {
        let ctx = locator
            .try_resolve(frame)
            .ok_or(FrameError::Internal("Invalid superblock locator".into()))?;
        let superblock = ctx.superblock;
        let mut fields = Vec::new();
        let width = superblock.width();
        let height = superblock.height();
        fields.push(FieldEntry::new("Type".into(), "Superblock".into()));
        fields.push(FieldEntry::new("Width".into(), width.to_string()));
        fields.push(FieldEntry::new("Height".into(), height.to_string()));
        let x = superblock.x();
        let y = superblock.y();
        fields.push(FieldEntry::new("Position".into(), format!("(x={}, y={})", x, y)));
        // TODO(comc): Switch between luma and chroma partition trees here.
        if let Some(partition_root) = &superblock.luma_partition_tree {
            let partition_type = partition_root.partition_type;
            let partition_type_name = frame
                .enum_lookup(ProtoEnumMapping::PartitionType, partition_type)
                .unwrap_or("UNKNOWN".into());
            fields.push(FieldEntry::new("Partition type".into(), partition_type_name));
        }
        Ok(fields)
    }

    fn calculate(kind: &SelectedObjectKind, frame: &Frame) -> Result<Self, FrameError> {
        let fields = match kind {
            SelectedObjectKind::TransformUnit(obj) => Self::calculate_transform_unit(obj, frame)?,
            SelectedObjectKind::CodingUnit(obj) => Self::calculate_coding_unit(obj, frame)?,
            SelectedObjectKind::Partition(obj) => Self::calculate_partition(obj, frame)?,
            SelectedObjectKind::Superblock(obj) => Self::calculate_superblock(obj, frame)?,
        };
        // TODO(comc): Make children selectable.
        // for (i, child) in kind.get_children(frame).iter().enumerate() {
        //     if let Some(rect) = child.rect(frame) {
        //         let width = rect.width();
        //         let height = rect.height();
        //         let x = rect.left();
        //         let y = rect.top();
        //         fields.push(FieldEntry::new(
        //             format!("Child {i}"),
        //             format!("{width}x{height} at ({x},{y})"),
        //         ));
        //     }
        // }

        let symbols = match kind {
            SelectedObjectKind::TransformUnit(obj) => Self::get_symbols_transform_unit(obj, frame),
            SelectedObjectKind::CodingUnit(obj) => Self::get_symbols_coding_unit(obj, frame),
            SelectedObjectKind::Partition(obj) => Self::get_symbols_partition(obj, frame),
            SelectedObjectKind::Superblock(obj) => Self::get_symbols_superblock(obj, frame),
        };
        Ok(Self { fields, symbols })
    }
}

// TODO(comc): Option to show partition blocks at user-selected depth.
#[derive(Deserialize, Serialize, Clone, Debug, PartialEq)]
pub enum SelectedObjectKind {
    TransformUnit(TransformUnitLocator),
    CodingUnit(CodingUnitLocator),
    Partition(PartitionLocator),
    Superblock(SuperblockLocator),
}
impl SelectedObjectKind {
    pub fn rect(&self, frame: &Frame) -> Option<Rect> {
        match self {
            SelectedObjectKind::TransformUnit(obj) => obj.try_resolve(frame).map(|ctx| ctx.transform_unit.rect()),
            SelectedObjectKind::CodingUnit(obj) => obj.try_resolve(frame).map(|ctx| ctx.coding_unit.rect()),
            SelectedObjectKind::Partition(obj) => obj.try_resolve(frame).map(|ctx| ctx.partition.rect()),
            SelectedObjectKind::Superblock(obj) => obj.try_resolve(frame).map(|ctx| ctx.superblock.rect()),
        }
    }

    pub fn get_parent(&self, frame: &Frame) -> Option<SelectedObjectKind> {
        match self {
            SelectedObjectKind::TransformUnit(obj) => {
                let ctx = obj.try_resolve(frame)?;
                Some(SelectedObjectKind::CodingUnit(ctx.coding_unit_context.locator))
            }

            SelectedObjectKind::CodingUnit(obj) => {
                let ctx = obj.try_resolve(frame)?;
                let parent = ctx.find_parent_partition()?;
                if parent.is_root() {
                    Some(SelectedObjectKind::Superblock(ctx.superblock_context.locator))
                } else {
                    // Partition tree leaf nodes map directly to coding units. To get the actual parent, we need to go one more level up the hierarchy.
                    let actual_parent = parent.locator.parent().unwrap();
                    if actual_parent.is_root() {
                        Some(SelectedObjectKind::Superblock(ctx.superblock_context.locator))
                    } else {
                        Some(SelectedObjectKind::Partition(actual_parent))
                    }
                }
            }

            SelectedObjectKind::Partition(obj) => {
                let ctx = obj.try_resolve(frame)?;
                if ctx.is_root() {
                    Some(SelectedObjectKind::Superblock(ctx.superblock_context.locator))
                } else {
                    let parent = ctx.locator.parent().unwrap();
                    if parent.is_root() {
                        Some(SelectedObjectKind::Superblock(ctx.superblock_context.locator))
                    } else {
                        Some(SelectedObjectKind::Partition(ctx.locator.parent().unwrap()))
                    }
                }
            }
            _ => None,
        }
    }

    #[allow(dead_code)]
    pub fn get_children(&self, frame: &Frame) -> Vec<SelectedObjectKind> {
        let mut children = Vec::new();
        match self {
            SelectedObjectKind::TransformUnit(_obj) => {}
            SelectedObjectKind::CodingUnit(obj) => {
                if let Some(ctx) = obj.try_resolve(frame) {
                    let kind = ctx.locator.kind;
                    let plane = match kind {
                        CodingUnitKind::LumaOnly | CodingUnitKind::Shared => Plane::Y,
                        CodingUnitKind::ChromaOnly => Plane::U,
                    };
                    // TODO(comc): Iterate over all three planes depending on plane view settings.
                    children.extend(
                        ctx.iter_transform_units(plane)
                            .map(|transform_unit| SelectedObjectKind::TransformUnit(transform_unit.locator)),
                    )
                }
            }

            SelectedObjectKind::Partition(obj) => {
                if let Some(ctx) = obj.try_resolve(frame) {
                    for (i, child) in ctx.iter_direct_children().enumerate() {
                        if child.partition.is_leaf_node {
                            if let Some(coding_unit_range) = &child.partition.coding_unit_range {
                                let coding_unit_index = coding_unit_range.start as usize;
                                let locator = CodingUnitLocator::new(
                                    ctx.superblock_context.locator,
                                    ctx.locator.kind,
                                    coding_unit_index,
                                );
                                children.push(SelectedObjectKind::CodingUnit(locator));
                            }
                        } else {
                            let mut locator = ctx.locator.clone();
                            locator.path_indices.push(i);
                            children.push(SelectedObjectKind::Partition(locator));
                        }
                    }
                }
            }
            _ => {}
        }
        children
    }
}

pub struct SelectedObject {
    pub kind: SelectedObjectKind,
    pub info: CachedInfo<Result<SelectedObjectInfo, FrameError>>,
}
impl SelectedObject {
    pub fn rect(&self, frame: &Frame) -> Option<Rect> {
        self.kind.rect(frame)
    }

    // TODO(comc): Refactor.
    pub fn get_or_calculate_info(&mut self, frame: &Frame) -> &Result<SelectedObjectInfo, FrameError> {
        if matches!(self.info, CachedInfo::Missing) {
            self.info = CachedInfo::Calculated(SelectedObjectInfo::calculate(&self.kind, frame));
        }
        let CachedInfo::Calculated(info) = &self.info else {
            panic!("Info is not calculated.");
        };
        info
    }
    pub fn new(kind: SelectedObjectKind) -> Self {
        Self {
            kind,
            info: CachedInfo::Missing,
        }
    }
}
