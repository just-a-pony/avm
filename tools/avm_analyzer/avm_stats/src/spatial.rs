use crate::{CodingUnit, Frame, Partition, Superblock, TransformUnit};

pub trait Spatial {
    fn width(&self) -> i32;
    fn height(&self) -> i32;
    fn x(&self) -> i32;
    fn y(&self) -> i32;
    fn rect(&self) -> emath::Rect {
        emath::Rect::from_min_size(
            emath::pos2(self.x() as f32, self.y() as f32),
            emath::vec2(self.width() as f32, self.height() as f32),
        )
    }
    fn size_name(&self) -> String {
        format!("{}x{}", self.width(), self.height())
    }
}

// TODO(comc): Could use a macro to implement each of these.
impl Spatial for TransformUnit {
    fn width(&self) -> i32 {
        // TODO(comc): This is very messy. Add derive Default to prost build script.
        self.size.as_ref().map_or(0, |size| size.width)
    }

    fn height(&self) -> i32 {
        self.size.as_ref().map_or(0, |size| size.height)
    }

    fn x(&self) -> i32 {
        self.position.as_ref().map_or(0, |position| position.x)
    }

    fn y(&self) -> i32 {
        self.position.as_ref().map_or(0, |position| position.y)
    }
}

impl Spatial for CodingUnit {
    fn width(&self) -> i32 {
        self.size.as_ref().map_or(0, |size| size.width)
    }

    fn height(&self) -> i32 {
        self.size.as_ref().map_or(0, |size| size.height)
    }

    fn x(&self) -> i32 {
        self.position.as_ref().map_or(0, |position| position.x)
    }

    fn y(&self) -> i32 {
        self.position.as_ref().map_or(0, |position| position.y)
    }
}

impl Spatial for Partition {
    fn width(&self) -> i32 {
        self.size.as_ref().map_or(0, |size| size.width)
    }

    fn height(&self) -> i32 {
        self.size.as_ref().map_or(0, |size| size.height)
    }

    fn x(&self) -> i32 {
        self.position.as_ref().map_or(0, |position| position.x)
    }

    fn y(&self) -> i32 {
        self.position.as_ref().map_or(0, |position| position.y)
    }
}

impl Spatial for Superblock {
    fn width(&self) -> i32 {
        self.size.as_ref().map_or(0, |size| size.width)
    }

    fn height(&self) -> i32 {
        self.size.as_ref().map_or(0, |size| size.height)
    }

    fn x(&self) -> i32 {
        self.position.as_ref().map_or(0, |position| position.x)
    }

    fn y(&self) -> i32 {
        self.position.as_ref().map_or(0, |position| position.y)
    }
}

impl Spatial for Frame {
    fn width(&self) -> i32 {
        self.frame_params.as_ref().map_or(0, |frame_params| frame_params.width)
    }

    fn height(&self) -> i32 {
        self.frame_params.as_ref().map_or(0, |frame_params| frame_params.height)
    }

    fn x(&self) -> i32 {
        0
    }

    fn y(&self) -> i32 {
        0
    }
}
