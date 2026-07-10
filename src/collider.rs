use nalgebra::Vector3;

pub struct Reaction {
    pub id: usize,
    pub impulse: Vector3<f64>,
    pub point: Vector3<f64>,
}

pub enum Collider {
    Sphere {
        id: usize,
        center: Vector3<f64>,
        radius: f64,
        inv_mass: f64,
        linvel: Vector3<f64>,
        angvel: Vector3<f64>,
    },
    Cuboid {
        id: usize,
        center: Vector3<f64>,
        basis: [Vector3<f64>; 3],
        half: Vector3<f64>,
        inv_mass: f64,
        linvel: Vector3<f64>,
        angvel: Vector3<f64>,
    },
}

impl Collider {
    pub fn id(&self) -> usize {
        match *self {
            Collider::Sphere { id, .. } | Collider::Cuboid { id, .. } => id,
        }
    }

    pub fn inv_mass(&self) -> f64 {
        match *self {
            Collider::Sphere { inv_mass, .. } | Collider::Cuboid { inv_mass, .. } => inv_mass,
        }
    }

    pub fn center(&self) -> Vector3<f64> {
        match *self {
            Collider::Sphere { center, .. } | Collider::Cuboid { center, .. } => center,
        }
    }

    pub fn linvel(&self) -> Vector3<f64> {
        match *self {
            Collider::Sphere { linvel, .. } | Collider::Cuboid { linvel, .. } => linvel,
        }
    }

    pub fn angvel(&self) -> Vector3<f64> {
        match *self {
            Collider::Sphere { angvel, .. } | Collider::Cuboid { angvel, .. } => angvel,
        }
    }

    pub fn horizontal_half(&self) -> (f64, f64) {
        match *self {
            Collider::Sphere { radius, .. } => (radius, radius),
            Collider::Cuboid { basis, half, .. } => {
                let extent = |axis: usize| {
                    basis[0][axis].abs() * half.x
                        + basis[1][axis].abs() * half.y
                        + basis[2][axis].abs() * half.z
                };
                (extent(0), extent(2))
            }
        }
    }

    pub fn top_y(&self) -> f64 {
        match *self {
            Collider::Sphere { center, radius, .. } => center.y + radius,
            Collider::Cuboid {
                center,
                basis,
                half,
                ..
            } => {
                let vy = basis[0].y.abs() * half.x
                    + basis[1].y.abs() * half.y
                    + basis[2].y.abs() * half.z;
                center.y + vy
            }
        }
    }

    pub fn submerged_volume(&self, surface: f64) -> f64 {
        match *self {
            Collider::Sphere { center, radius, .. } => {
                let h = (surface - (center.y - radius)).clamp(0.0, 2.0 * radius);
                std::f64::consts::PI * h * h * (3.0 * radius - h) / 3.0
            }
            Collider::Cuboid {
                center,
                basis,
                half,
                ..
            } => {
                let vy = basis[0].y.abs() * half.x
                    + basis[1].y.abs() * half.y
                    + basis[2].y.abs() * half.z;
                let d = (surface - (center.y - vy)).clamp(0.0, 2.0 * vy);
                let (hx, hz) = self.horizontal_half();
                (2.0 * hx) * (2.0 * hz) * d
            }
        }
    }

    pub fn penetration(
        &self,
        p: Vector3<f64>,
        margin: f64,
    ) -> Option<(Vector3<f64>, f64, Vector3<f64>)> {
        match *self {
            Collider::Sphere { center, radius, .. } => {
                let d = p - center;
                let dist = d.norm();
                let surface = radius + margin;
                if dist >= surface {
                    return None;
                }
                let n = if dist > 1e-9 { d / dist } else { Vector3::y() };
                Some((n, surface - dist, center + n * radius))
            }
            Collider::Cuboid {
                center,
                basis,
                half,
                ..
            } => {
                let rel = p - center;
                let local =
                    Vector3::new(rel.dot(&basis[0]), rel.dot(&basis[1]), rel.dot(&basis[2]));
                let ext = half.add_scalar(margin);
                if local.x.abs() >= ext.x || local.y.abs() >= ext.y || local.z.abs() >= ext.z {
                    return None;
                }
                let pens = [
                    ext.x - local.x.abs(),
                    ext.y - local.y.abs(),
                    ext.z - local.z.abs(),
                ];
                let axis = if pens[0] <= pens[1] && pens[0] <= pens[2] {
                    0
                } else if pens[1] <= pens[2] {
                    1
                } else {
                    2
                };
                let sign = if local[axis] >= 0.0 { 1.0 } else { -1.0 };
                let n = basis[axis] * sign;
                let mut surface_local = local;
                surface_local[axis] = half[axis] * sign;
                let contact = center
                    + basis[0] * surface_local.x
                    + basis[1] * surface_local.y
                    + basis[2] * surface_local.z;
                Some((n, pens[axis], contact))
            }
        }
    }
}
