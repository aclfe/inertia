use nalgebra::Vector3;

#[derive(Clone, Copy)]
pub struct Projection {
    pub focal: f64,
    pub near: f64,
}

impl Projection {
    pub fn new(fov_y_degrees: f64) -> Self {
        let fov_y = fov_y_degrees.to_radians();
        Self {
            focal: 1.0 / (fov_y / 2.0).tan(),
            near: 0.1,
        }
    }

    pub fn project(&self, view: Vector3<f64>) -> Option<(f64, f64)> {
        if view.z <= self.near {
            return None;
        }
        let ndc_x = self.focal * view.x / view.z;
        let ndc_y = self.focal * view.y / view.z;
        Some((ndc_x, ndc_y))
    }
}
