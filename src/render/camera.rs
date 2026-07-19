use nalgebra::Vector3;

const WORLD_UP: Vector3<f64> = Vector3::new(0.0, 1.0, 0.0);
const MIN_PHI: f64 = 0.05;
const MAX_PHI: f64 = std::f64::consts::PI - 0.05;
const MIN_DISTANCE: f64 = 2.0;
const MAX_DISTANCE: f64 = 60.0;

pub struct Camera {
    pub theta: f64,
    pub phi: f64,
    pub distance: f64,
    pub target: Vector3<f64>,
}

impl Default for Camera {
    fn default() -> Self {
        Self::new()
    }
}

impl Camera {
    pub fn new() -> Self {
        Self {
            theta: std::f64::consts::FRAC_PI_4,
            phi: std::f64::consts::FRAC_PI_3,
            distance: 20.0,
            target: Vector3::new(0.0, 1.0, 0.0),
        }
    }

    pub fn eye(&self) -> Vector3<f64> {
        self.target
            + self.distance
                * Vector3::new(
                    self.phi.sin() * self.theta.cos(),
                    self.phi.cos(),
                    self.phi.sin() * self.theta.sin(),
                )
    }

    pub fn orbit(&mut self, dtheta: f64, dphi: f64) {
        self.theta += dtheta;
        self.phi = (self.phi + dphi).clamp(MIN_PHI, MAX_PHI);
    }

    pub fn zoom(&mut self, factor: f64) {
        self.distance = (self.distance * factor).clamp(MIN_DISTANCE, MAX_DISTANCE);
    }

    pub fn pan(&mut self, dright: f64, dup: f64) {
        let (right, up, _) = self.basis();
        self.target += right * dright + up * dup;
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    pub fn basis(&self) -> (Vector3<f64>, Vector3<f64>, Vector3<f64>) {
        let forward = (self.target - self.eye()).normalize();
        let right = forward.cross(&WORLD_UP).normalize();
        let up = right.cross(&forward);
        (right, up, forward)
    }

    pub fn view(&self) -> View {
        let (right, up, forward) = self.basis();
        View {
            eye: self.eye(),
            right,
            up,
            forward,
        }
    }
}

pub struct View {
    eye: Vector3<f64>,
    right: Vector3<f64>,
    up: Vector3<f64>,
    forward: Vector3<f64>,
}

impl View {
    pub fn transform(&self, world: Vector3<f64>) -> Vector3<f64> {
        let rel = world - self.eye;
        Vector3::new(
            rel.dot(&self.right),
            rel.dot(&self.up),
            rel.dot(&self.forward),
        )
    }

    pub fn right(&self) -> Vector3<f64> {
        self.right
    }
}
