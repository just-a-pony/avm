use egui::{pos2, Pos2, Rect, Vec2};

pub trait ScreenBounds {
    fn zoom_point(&mut self, center: Pos2, zoom_factor: f32);
    fn calc_scale(&self, screen_bounds: Rect) -> f32;
    fn translate(&mut self, delta: Vec2);
    fn screen_pos_to_world(&self, screen_point: Pos2, screen_bounds: Rect) -> Pos2;
    fn world_pos_to_screen(&self, world_point: Pos2, screen_bounds: Rect) -> Pos2;

    fn screen_rect_to_world(&self, screen_rect: Rect, screen_bounds: Rect) -> Rect;
    fn world_rect_to_screen(&self, world_rect: Rect, screen_bounds: Rect) -> Rect;
}

impl ScreenBounds for Rect {
    fn zoom_point(&mut self, center: Pos2, zoom_factor: f32) {
        let mut left = center.x - self.left();
        let mut right = self.right() - center.x;
        left *= zoom_factor;
        right *= zoom_factor;
        let mut top = center.y - self.top();
        let mut bottom = self.bottom() - center.y;
        top *= zoom_factor;
        bottom *= zoom_factor;

        self.set_left(center.x - left);
        self.set_right(center.x + right);
        self.set_top(center.y - top);
        self.set_bottom(center.y + bottom);
    }

    fn calc_scale(&self, screen_bounds: Rect) -> f32 {
        let x_scale = screen_bounds.width() / (self.right() - self.left());
        let y_scale = screen_bounds.height() / (self.bottom() - self.top());
        x_scale.min(y_scale)
    }

    fn translate(&mut self, delta: Vec2) {
        self.set_left(self.left() - delta.x);
        self.set_right(self.right() - delta.x);
        self.set_top(self.top() - delta.y);
        self.set_bottom(self.bottom() - delta.y);
    }

    fn screen_pos_to_world(&self, screen_point: Pos2, screen_bounds: Rect) -> Pos2 {
        let scale = self.calc_scale(screen_bounds);
        let world_x = (screen_point.x - screen_bounds.left()) / scale + self.left();
        let world_y = (screen_point.y - screen_bounds.top()) / scale + self.top();
        pos2(world_x, world_y)
    }

    fn world_pos_to_screen(&self, world_point: Pos2, screen_bounds: Rect) -> Pos2 {
        let scale = self.calc_scale(screen_bounds);
        let screen_x = (world_point.x - self.left()) * scale + screen_bounds.left();
        let screen_y = (world_point.y - self.top()) * scale + screen_bounds.top();
        pos2(screen_x, screen_y)
    }

    fn screen_rect_to_world(&self, screen_rect: Rect, screen_bounds: Rect) -> Rect {
        let left_top = self.screen_pos_to_world(screen_rect.left_top(), screen_bounds);
        let right_bottom = self.screen_pos_to_world(screen_rect.right_bottom(), screen_bounds);
        Rect::from_min_max(left_top, right_bottom)
    }

    fn world_rect_to_screen(&self, world_rect: Rect, screen_bounds: Rect) -> Rect {
        let left_top = self.world_pos_to_screen(world_rect.left_top(), screen_bounds);
        let right_bottom = self.world_pos_to_screen(world_rect.right_bottom(), screen_bounds);
        Rect::from_min_max(left_top, right_bottom)
    }
}
