use nalgebra::Vector3;

use crate::collider::Collider;

const SPACING: f64 = 0.32;
const SUBSTEPS: usize = 4;
const ITERS: usize = 12;
const DAMPING: f64 = 0.99;
const PARTICLE_MASS: f64 = 0.03;
pub const DEFAULT_STIFFNESS: f64 = 1.0;
pub const MIN_STIFFNESS: f64 = 0.05;
pub const MAX_STIFFNESS: f64 = 1.0;
const GROUND_EPS: f64 = 0.01;
const GROUND_FRICTION: f64 = 0.5;
const BODY_FRICTION: f64 = 0.4;
const BODY_SURFACE_FRICTION: f64 = 0.35;
const COLLISION_MARGIN: f64 = 0.03;
const MAX_SUPPORT_DV: f64 = 25.0;
const PENETRATION_BIAS: f64 = 0.2;
const CLOTH_COLLIDE_DIST: f64 = SPACING;
const CLOTH_CLOTH_FRICTION: f64 = 0.3;
const SELF_COLLIDE_DIST: f64 = SPACING;
const SELF_COLLIDE_SKIP: usize = 2;
const SELF_COLLIDE_FRICTION: f64 = 0.3;
pub const WIND_ACCEL: Vector3<f64> = Vector3::new(6.0, 0.0, 2.5);
const MAX_FLUID_COUPLE_DV: f64 = 20.0;

pub struct Reaction {
    pub id: usize,
    pub impulse: Vector3<f64>,
    pub point: Vector3<f64>,
}

#[derive(Clone, Copy, Default)]
struct BodyContact {
    sum_n: Vector3<f64>,
    sum_c: Vector3<f64>,
    sum_vsurf: Vector3<f64>,
    count: f64,
    max_pen: f64,
}

pub struct ClothView {
    pub cols: usize,
    pub rows: usize,
    pub pos: Vec<Vector3<f64>>,
    pub pinned: Vec<bool>,
}

pub struct Cloth {
    cols: usize,
    rows: usize,
    pos: Vec<Vector3<f64>>,
    prev: Vec<Vector3<f64>>,
    pinned: Vec<bool>,
    inv_mass: Vec<f64>,
    constraints: Vec<(usize, usize, f64)>,
    grabbed: Option<usize>,
    grab_target: Vector3<f64>,
    stiffness: f64,
}

struct PairCorrection {
    pos_a: Vector3<f64>,
    prev_a: Vector3<f64>,
    pos_b: Vector3<f64>,
    prev_b: Vector3<f64>,
}

#[allow(clippy::too_many_arguments)]
fn resolve_particle_pair(
    pos_a: Vector3<f64>,
    prev_a: Vector3<f64>,
    inv_mass_a: f64,
    pos_b: Vector3<f64>,
    prev_b: Vector3<f64>,
    inv_mass_b: f64,
    contact: f64,
    friction: f64,
) -> Option<PairCorrection> {
    let w = inv_mass_a + inv_mass_b;
    if w == 0.0 {
        return None;
    }
    let delta = pos_b - pos_a;
    let dist2 = delta.norm_squared();
    if dist2 >= contact * contact || dist2 < 1e-12 {
        return None;
    }
    let dist = dist2.sqrt();
    let n = delta / dist;
    let pen = contact - dist;

    let rel = (pos_b - prev_b) - (pos_a - prev_a);
    let vn = rel.dot(&n);

    let sa = n * (pen * inv_mass_a / w);
    let sb = n * (pen * inv_mass_b / w);
    let new_pos_a = pos_a - sa;
    let mut new_prev_a = prev_a - sa;
    let new_pos_b = pos_b + sb;
    let mut new_prev_b = prev_b + sb;

    if vn < 0.0 {
        new_prev_a -= n * (vn * inv_mass_a / w);
        new_prev_b += n * (vn * inv_mass_b / w);
    }

    let vt = rel - n * vn;
    new_prev_a -= vt * (inv_mass_a / w * friction);
    new_prev_b += vt * (inv_mass_b / w * friction);

    Some(PairCorrection {
        pos_a: new_pos_a,
        prev_a: new_prev_a,
        pos_b: new_pos_b,
        prev_b: new_prev_b,
    })
}

impl Cloth {
    pub fn new(cols: usize, rows: usize, origin: Vector3<f64>) -> Self {
        let w = (cols - 1) as f64 * SPACING;
        let h = (rows - 1) as f64 * SPACING;
        let mut pos = Vec::with_capacity(cols * rows);
        for r in 0..rows {
            for c in 0..cols {
                let (u, v) = (c as f64 * SPACING - w / 2.0, r as f64 * SPACING);
                pos.push(origin + Vector3::new(u, 0.0, v - h / 2.0));
            }
        }

        let pinned = vec![false; cols * rows];
        let free_inv = 1.0 / PARTICLE_MASS;
        let inv_mass = pinned
            .iter()
            .map(|&p| if p { 0.0 } else { free_inv })
            .collect();

        let mut cloth = Self {
            cols,
            rows,
            prev: pos.clone(),
            pos,
            pinned,
            inv_mass,
            constraints: Vec::new(),
            grabbed: None,
            grab_target: Vector3::zeros(),
            stiffness: DEFAULT_STIFFNESS,
        };
        cloth.build_constraints();
        cloth
    }

    fn idx(&self, r: usize, c: usize) -> usize {
        r * self.cols + c
    }

    fn build_constraints(&mut self) {
        let add = |a: usize, b: usize, pos: &[Vector3<f64>], out: &mut Vec<(usize, usize, f64)>| {
            out.push((a, b, (pos[a] - pos[b]).norm()));
        };
        let mut cons = Vec::new();
        for r in 0..self.rows {
            for c in 0..self.cols {
                let i = self.idx(r, c);
                if c + 1 < self.cols {
                    add(i, self.idx(r, c + 1), &self.pos, &mut cons);
                }
                if r + 1 < self.rows {
                    add(i, self.idx(r + 1, c), &self.pos, &mut cons);
                }
                if r + 1 < self.rows && c + 1 < self.cols {
                    add(i, self.idx(r + 1, c + 1), &self.pos, &mut cons);
                    add(self.idx(r + 1, c), self.idx(r, c + 1), &self.pos, &mut cons);
                }
                if c + 2 < self.cols {
                    add(i, self.idx(r, c + 2), &self.pos, &mut cons);
                }
                if r + 2 < self.rows {
                    add(i, self.idx(r + 2, c), &self.pos, &mut cons);
                }
            }
        }
        self.constraints = cons;
    }

    pub fn positions(&self) -> &[Vector3<f64>] {
        &self.pos
    }

    pub fn toggle_pin(&mut self, i: usize) {
        self.pinned[i] = !self.pinned[i];
        self.inv_mass[i] = if self.pinned[i] {
            0.0
        } else {
            1.0 / PARTICLE_MASS
        };
    }

    pub fn pin_at(&mut self, i: usize, pos: Vector3<f64>) {
        self.pos[i] = pos;
        self.prev[i] = pos;
        self.pinned[i] = true;
        self.inv_mass[i] = 0.0;
    }

    pub fn stiffness(&self) -> f64 {
        self.stiffness
    }

    pub fn set_stiffness(&mut self, s: f64) {
        self.stiffness = s.clamp(MIN_STIFFNESS, MAX_STIFFNESS);
    }

    pub fn grab(&mut self, i: usize) {
        self.grabbed = Some(i);
        self.grab_target = self.pos[i];
    }

    pub fn drag_to(&mut self, target: Vector3<f64>) {
        self.grab_target = target;
    }

    pub fn is_grabbed(&self) -> bool {
        self.grabbed.is_some()
    }

    pub fn release(&mut self) {
        self.grabbed = None;
    }

    pub fn particle_pos(&self, i: usize) -> Vector3<f64> {
        self.pos[i]
    }

    pub fn fluid_coupling_state(
        &self,
        dt: f64,
    ) -> (Vec<Vector3<f64>>, Vec<Vector3<f64>>, Vec<f64>) {
        let h = dt / SUBSTEPS as f64;
        let vel = self
            .pos
            .iter()
            .zip(&self.prev)
            .map(|(p, q)| (p - q) / h)
            .collect();
        (self.pos.clone(), vel, self.inv_mass.clone())
    }

    pub fn apply_fluid_impulses(&mut self, imp: &[Vector3<f64>], dt: f64) {
        let h = dt / SUBSTEPS as f64;
        for (k, &j) in imp.iter().enumerate() {
            let wv = self.inv_mass[k];
            if wv == 0.0 {
                continue;
            }
            let mut dv = j * wv;
            let s = dv.norm();
            if s > MAX_FLUID_COUPLE_DV {
                dv *= MAX_FLUID_COUPLE_DV / s;
            }
            self.prev[k] -= dv * h;
        }
    }

    pub fn view(&self) -> ClothView {
        ClothView {
            cols: self.cols,
            rows: self.rows,
            pos: self.pos.clone(),
            pinned: self.pinned.clone(),
        }
    }

    pub fn step(
        &mut self,
        dt: f64,
        gravity: Vector3<f64>,
        wind: Vector3<f64>,
        ground_y: f64,
        bodies: &[Collider],
    ) -> Vec<Reaction> {
        let h = dt / SUBSTEPS as f64;
        let accel = gravity + wind;
        let mut acc: Vec<BodyContact> = vec![BodyContact::default(); bodies.len()];

        let held = self.grabbed.map(|g| (g, self.inv_mass[g]));
        if let Some((g, _)) = held {
            self.inv_mass[g] = 0.0;
        }

        for _ in 0..SUBSTEPS {
            self.integrate(h, accel);
            if let Some((g, _)) = held {
                self.pos[g] = self.grab_target;
                self.prev[g] = self.grab_target;
            }
            for _ in 0..ITERS {
                self.solve_constraints();
            }
            self.self_collide();
            self.collide_ground(ground_y);
            self.collide_bodies(bodies, h, &mut acc);
        }

        if let Some((g, inv_mass)) = held {
            self.inv_mass[g] = inv_mass;
        }

        bodies
            .iter()
            .zip(acc)
            .filter_map(|(body, bc)| {
                if bc.count <= 0.0 {
                    return None;
                }
                let len = bc.sum_n.norm();
                let inv_m = body.inv_mass();
                if len < 1e-6 || inv_m <= 0.0 {
                    return None;
                }
                let normal = bc.sum_n / len;
                let mass = 1.0 / inv_m;
                let approach = body.linvel().dot(&normal).max(0.0);
                let bias = bc.max_pen / dt * PENETRATION_BIAS;
                let dv = (approach + bias).min(MAX_SUPPORT_DV);
                let support = -normal * (mass * dv);

                let v_surf = bc.sum_vsurf / bc.count;
                let v_rel = body.linvel() - v_surf;
                let v_t = v_rel - normal * v_rel.dot(&normal);
                let vt_speed = v_t.norm();
                let friction = if vt_speed > 1e-9 {
                    let fdv = (BODY_SURFACE_FRICTION * vt_speed).min(MAX_SUPPORT_DV);
                    -v_t * (mass * fdv / vt_speed)
                } else {
                    Vector3::zeros()
                };

                let impulse = support + friction;
                if impulse.norm() < 1e-9 {
                    return None;
                }
                Some(Reaction {
                    id: body.id(),
                    impulse,
                    point: body.center(),
                })
            })
            .collect()
    }

    fn integrate(&mut self, h: f64, accel: Vector3<f64>) {
        let h2 = h * h;
        let mut mean = Vector3::zeros();
        let mut n = 0.0;
        for i in 0..self.pos.len() {
            if self.inv_mass[i] != 0.0 {
                mean += self.pos[i] - self.prev[i];
                n += 1.0;
            }
        }
        if n > 0.0 {
            mean /= n;
        }
        for i in 0..self.pos.len() {
            if self.inv_mass[i] == 0.0 {
                continue;
            }
            let p = self.pos[i];
            let vel = mean + (p - self.prev[i] - mean) * DAMPING;
            self.prev[i] = p;
            self.pos[i] = p + vel + accel * h2;
        }
    }

    fn solve_constraints(&mut self) {
        for &(i, j, rest) in &self.constraints {
            let (wi, wj) = (self.inv_mass[i], self.inv_mass[j]);
            let w = wi + wj;
            if w == 0.0 {
                continue;
            }
            let delta = self.pos[j] - self.pos[i];
            let len = delta.norm();
            if len < 1e-9 {
                continue;
            }
            let corr = delta * ((len - rest) / len) * self.stiffness;
            self.pos[i] += corr * (wi / w);
            self.pos[j] -= corr * (wj / w);
        }
    }

    fn collide_ground(&mut self, ground_y: f64) {
        let floor = ground_y + GROUND_EPS;
        for i in 0..self.pos.len() {
            if self.inv_mass[i] == 0.0 || self.pos[i].y >= floor {
                continue;
            }
            self.pos[i].y = floor;
            self.prev[i].x = self.pos[i].x + (self.prev[i].x - self.pos[i].x) * GROUND_FRICTION;
            self.prev[i].z = self.pos[i].z + (self.prev[i].z - self.pos[i].z) * GROUND_FRICTION;
            self.prev[i].y = self.pos[i].y;
        }
    }

    fn collide_bodies(&mut self, bodies: &[Collider], h: f64, acc: &mut [BodyContact]) {
        for (bi, body) in bodies.iter().enumerate() {
            for i in 0..self.pos.len() {
                let Some((n, pen, contact)) = body.penetration(self.pos[i], COLLISION_MARGIN)
                else {
                    continue;
                };
                let wp = self.inv_mass[i];
                let wb = body.inv_mass();
                let denom = wp + wb;
                if denom == 0.0 {
                    continue;
                }
                let vp = (self.pos[i] - self.prev[i]) / h;
                let r = contact - body.center();
                let vb = body.linvel() + body.angvel().cross(&r);
                let rel = vp - vb;
                let vn = rel.dot(&n);

                let shift = n * (pen * wp / denom);
                self.pos[i] += shift;
                self.prev[i] += shift;

                let rel_t = rel - n * vn;
                self.prev[i] += rel_t * ((1.0 - BODY_FRICTION) * h);

                if vn < 0.0 {
                    let jp = (-vn) / denom;
                    self.prev[i] -= n * (jp * wp * h);
                }

                if wb > 0.0 {
                    acc[bi].sum_n += n;
                    acc[bi].sum_c += contact;
                    acc[bi].sum_vsurf += vp;
                    acc[bi].count += 1.0;
                    acc[bi].max_pen = acc[bi].max_pen.max(pen);
                }
            }
        }
    }

    fn self_collide(&mut self) {
        let contact = SELF_COLLIDE_DIST;
        for a in 0..self.pos.len() {
            let (ra, ca) = (a / self.cols, a % self.cols);
            for b in (a + 1)..self.pos.len() {
                if ra.abs_diff(b / self.cols) <= SELF_COLLIDE_SKIP
                    && ca.abs_diff(b % self.cols) <= SELF_COLLIDE_SKIP
                {
                    continue;
                }
                let Some(corr) = resolve_particle_pair(
                    self.pos[a],
                    self.prev[a],
                    self.inv_mass[a],
                    self.pos[b],
                    self.prev[b],
                    self.inv_mass[b],
                    contact,
                    SELF_COLLIDE_FRICTION,
                ) else {
                    continue;
                };
                self.pos[a] = corr.pos_a;
                self.prev[a] = corr.prev_a;
                self.pos[b] = corr.pos_b;
                self.prev[b] = corr.prev_b;
            }
        }
    }

    pub fn collide_with(&mut self, other: &mut Cloth) {
        let contact = CLOTH_COLLIDE_DIST;
        for i in 0..self.pos.len() {
            for j in 0..other.pos.len() {
                let Some(corr) = resolve_particle_pair(
                    self.pos[i],
                    self.prev[i],
                    self.inv_mass[i],
                    other.pos[j],
                    other.prev[j],
                    other.inv_mass[j],
                    contact,
                    CLOTH_CLOTH_FRICTION,
                ) else {
                    continue;
                };
                self.pos[i] = corr.pos_a;
                self.prev[i] = corr.prev_a;
                other.pos[j] = corr.pos_b;
                other.prev[j] = corr.prev_b;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const GRAVITY: Vector3<f64> = Vector3::new(0.0, -9.81, 0.0);
    const DT: f64 = 1.0 / 60.0;

    fn step_n(cloth: &mut Cloth, bodies: &[Collider], ground: f64, n: usize) {
        for _ in 0..n {
            cloth.step(DT, GRAVITY, Vector3::zeros(), ground, bodies);
        }
    }

    fn all_finite(cloth: &Cloth) -> bool {
        cloth
            .pos
            .iter()
            .all(|p| p.x.is_finite() && p.y.is_finite() && p.z.is_finite())
    }

    #[test]
    fn hangs_stably_from_pinned_corners() {
        let mut cloth = Cloth::new(15, 15, Vector3::new(0.0, 5.0, 0.0));
        cloth.toggle_pin(0);
        cloth.toggle_pin(14);
        let corner_a = cloth.pos[0];
        let corner_b = cloth.pos[14];
        step_n(&mut cloth, &[], f64::NEG_INFINITY, 600);

        assert!(all_finite(&cloth), "cloth diverged");
        assert!(
            (cloth.pos[0] - corner_a).norm() < 1e-9,
            "pinned corner moved"
        );
        assert!(
            (cloth.pos[14] - corner_b).norm() < 1e-9,
            "pinned corner moved"
        );
        let center = cloth.pos[7 * 15 + 7];
        assert!(
            center.y < corner_a.y,
            "sheet didn't hang down from its pinned corners"
        );
        assert!(
            cloth
                .pos
                .iter()
                .all(|p| p.x.abs() < 20.0 && p.y.abs() < 20.0 && p.z.abs() < 20.0),
            "sheet exploded"
        );
    }

    #[test]
    fn drapes_over_sphere_without_tunnelling() {
        let center = Vector3::new(0.0, 0.0, 0.0);
        let radius = 1.0;
        let sphere = Collider::Sphere {
            id: 0,
            center,
            radius,
            inv_mass: 0.0,
            linvel: Vector3::zeros(),
            angvel: Vector3::zeros(),
        };
        let mut cloth = Cloth::new(15, 15, Vector3::new(0.0, 2.5, 0.0));
        step_n(
            &mut cloth,
            std::slice::from_ref(&sphere),
            f64::NEG_INFINITY,
            400,
        );

        assert!(all_finite(&cloth), "cloth diverged");
        assert!(
            cloth
                .pos
                .iter()
                .all(|p| (p - center).norm() >= radius - 1e-6),
            "a particle tunnelled into the sphere",
        );
    }

    #[test]
    fn drapes_over_box_and_grips() {
        let center = Vector3::new(0.0, 0.0, 0.0);
        let half = Vector3::new(1.5, 0.5, 1.5);
        let cube = Collider::Cuboid {
            id: 0,
            center,
            basis: [Vector3::x(), Vector3::y(), Vector3::z()],
            half,
            inv_mass: 0.0,
            linvel: Vector3::zeros(),
            angvel: Vector3::zeros(),
        };
        let mut cloth = Cloth::new(15, 15, Vector3::new(0.0, 1.5, 0.0));
        step_n(
            &mut cloth,
            std::slice::from_ref(&cube),
            f64::NEG_INFINITY,
            400,
        );

        assert!(all_finite(&cloth), "cloth diverged");
        let inside = |p: &Vector3<f64>| {
            (p.x - center.x).abs() < half.x
                && (p.y - center.y).abs() < half.y
                && (p.z - center.z).abs() < half.z
        };
        assert!(
            cloth.pos.iter().all(|p| !inside(p)),
            "a particle tunnelled into the box"
        );
        let top = half.y + COLLISION_MARGIN;
        let resting = cloth
            .pos
            .iter()
            .filter(|p| p.x.abs() < half.x && p.z.abs() < half.z && (p.y - top).abs() < 0.1)
            .count();
        assert!(
            resting > 10,
            "sheet slid off the box instead of gripping it (only {resting} resting)"
        );
    }

    #[test]
    fn body_contact_applies_friction() {
        let mut cloth = Cloth::new(2, 2, Vector3::new(0.0, 5.0, 0.0));
        let cube = Collider::Cuboid {
            id: 0,
            center: Vector3::zeros(),
            basis: [Vector3::x(), Vector3::y(), Vector3::z()],
            half: Vector3::new(2.0, 0.5, 2.0),
            inv_mass: 0.0,
            linvel: Vector3::zeros(),
            angvel: Vector3::zeros(),
        };
        cloth.pos[0] = Vector3::new(0.0, 0.525, 0.0);
        cloth.prev[0] = cloth.pos[0] - Vector3::new(0.1, 0.0, 0.0);

        let mut acc = vec![BodyContact::default()];
        cloth.collide_bodies(std::slice::from_ref(&cube), 1.0 / 240.0, &mut acc);

        let vt = cloth.pos[0].x - cloth.prev[0].x;
        assert!(vt > 0.0, "friction reversed the slide: {vt}");
        assert!(vt < 0.1, "tangential slide was not damped at all: {vt}");
    }

    #[test]
    fn heavy_box_rests_on_cloth_instead_of_sinking_through() {
        let mass = 50.0;
        let inv_m = 1.0 / mass;
        let half = Vector3::new(0.5, 0.5, 0.5);
        let basis = [Vector3::x(), Vector3::y(), Vector3::z()];
        let mut center = Vector3::new(0.0, 2.0, 0.0);
        let mut vel = Vector3::zeros();
        let mut cloth = Cloth::new(21, 21, Vector3::new(0.0, 0.3, 0.0));

        for _ in 0..600 {
            vel += GRAVITY * DT;
            center += vel * DT;
            let body = Collider::Cuboid {
                id: 0,
                center,
                basis,
                half,
                inv_mass: inv_m,
                linvel: vel,
                angvel: Vector3::zeros(),
            };
            for r in cloth.step(
                DT,
                GRAVITY,
                Vector3::zeros(),
                0.0,
                std::slice::from_ref(&body),
            ) {
                vel += r.impulse * inv_m;
            }
        }

        assert!(all_finite(&cloth) && center.y.is_finite(), "sim diverged");
        assert!(
            center.y > 0.35,
            "box sank through the cloth: y = {}",
            center.y
        );
        let under_box = cloth
            .pos
            .iter()
            .filter(|p| p.x.abs() < half.x && p.z.abs() < half.z)
            .all(|p| p.y < center.y);
        assert!(under_box, "box ended up below the cloth it was placed on");
    }

    #[test]
    fn contact_friction_is_relative_to_body_motion() {
        let h = 1.0 / 240.0;
        let mut cloth = Cloth::new(2, 2, Vector3::new(0.0, 5.0, 0.0));
        let platform_v = Vector3::new(3.0, 0.0, 0.0);
        let cube = Collider::Cuboid {
            id: 0,
            center: Vector3::zeros(),
            basis: [Vector3::x(), Vector3::y(), Vector3::z()],
            half: Vector3::new(2.0, 0.5, 2.0),
            inv_mass: 0.0,
            linvel: platform_v,
            angvel: Vector3::zeros(),
        };
        cloth.pos[0] = Vector3::new(0.0, 0.525, 0.0);
        cloth.prev[0] = cloth.pos[0] - platform_v * h;
        let mut acc = vec![BodyContact::default()];
        cloth.collide_bodies(std::slice::from_ref(&cube), h, &mut acc);
        let vx = (cloth.pos[0].x - cloth.prev[0].x) / h;
        assert!(
            (vx - platform_v.x).abs() < 1e-9,
            "contact dragged a co-moving particle: {vx}"
        );
    }

    #[test]
    fn cloth_rests_on_cloth_without_tunnelling() {
        let mut bottom = Cloth::new(15, 15, Vector3::new(0.0, 1.0, 0.0));
        for i in 0..bottom.pos.len() {
            bottom.toggle_pin(i);
        }
        let mut top = Cloth::new(15, 15, Vector3::new(0.0, 2.0, 0.0));

        for _ in 0..400 {
            top.step(DT, GRAVITY, Vector3::zeros(), f64::NEG_INFINITY, &[]);
            bottom.step(DT, GRAVITY, Vector3::zeros(), f64::NEG_INFINITY, &[]);
            for _ in 0..2 {
                bottom.collide_with(&mut top);
            }
        }

        assert!(
            top.pos.iter().all(|p| p.x.is_finite() && p.y.is_finite()),
            "top cloth diverged"
        );
        assert!(
            top.pos.iter().all(|p| p.y > 1.0 - CLOTH_COLLIDE_DIST),
            "top cloth tunnelled through the bottom cloth: min y = {}",
            top.pos.iter().map(|p| p.y).fold(f64::INFINITY, f64::min),
        );
        let avg_y = top.pos.iter().map(|p| p.y).sum::<f64>() / top.pos.len() as f64;
        assert!(
            avg_y < 1.0 + 2.0 * CLOTH_COLLIDE_DIST,
            "top cloth never settled: avg y = {avg_y}"
        );
    }

    #[test]
    fn body_set_on_cloth_grips_and_settles() {
        let mass = 5.0;
        let inv_m = 1.0 / mass;
        let radius = 0.5;
        let (cols, rows) = (21, 21);
        let mut cloth = Cloth::new(cols, rows, Vector3::new(0.0, 1.5, 0.0));
        for i in [0usize, cols - 1, cols * (rows - 1), cols * rows - 1] {
            cloth.toggle_pin(i);
        }
        let mut center = Vector3::new(0.10, 2.4, 0.05);
        let mut vel = Vector3::zeros();
        let step_body = |center: Vector3<f64>, vel: Vector3<f64>| Collider::Sphere {
            id: 0,
            center,
            radius,
            inv_mass: inv_m,
            linvel: vel,
            angvel: Vector3::zeros(),
        };
        for _ in 0..600 {
            vel += GRAVITY * DT;
            center += vel * DT;
            let body = step_body(center, vel);
            for r in cloth.step(
                DT,
                GRAVITY,
                Vector3::zeros(),
                f64::NEG_INFINITY,
                std::slice::from_ref(&body),
            ) {
                vel += r.impulse * inv_m;
            }
        }
        let start = center;
        let mut travel = 0.0;
        let mut prev = center;
        for _ in 0..600 {
            vel += GRAVITY * DT;
            center += vel * DT;
            let body = step_body(center, vel);
            for r in cloth.step(
                DT,
                GRAVITY,
                Vector3::zeros(),
                f64::NEG_INFINITY,
                std::slice::from_ref(&body),
            ) {
                vel += r.impulse * inv_m;
            }
            let d = center - prev;
            travel += (d.x * d.x + d.z * d.z).sqrt();
            prev = center;
        }
        assert!(center.y.is_finite(), "sim diverged");
        let hspeed = (vel.x * vel.x + vel.z * vel.z).sqrt();
        assert!(
            hspeed < 0.05,
            "ball never stopped sliding on the cloth: hspeed = {hspeed}"
        );
        assert!(
            travel < 0.1,
            "ball kept wandering the pocket instead of settling: horizontal travel = {travel:.3}",
        );
        let drift = ((center.x - start.x).powi(2) + (center.z - start.z).powi(2)).sqrt();
        assert!(
            drift < 0.1,
            "settled ball drifted across the sheet: {drift:.3}"
        );
    }

    #[test]
    fn resting_body_is_pushed_up() {
        let mut cloth = Cloth::new(15, 15, Vector3::new(0.0, 0.0, 0.0));
        for i in 0..cloth.pos.len() {
            cloth.toggle_pin(i);
        }
        let sphere = Collider::Sphere {
            id: 7,
            center: Vector3::new(0.0, 0.8, 0.0),
            radius: 1.0,
            inv_mass: 1.0,
            linvel: Vector3::new(0.0, -1.0, 0.0),
            angvel: Vector3::zeros(),
        };
        let reactions = cloth.step(
            DT,
            GRAVITY,
            Vector3::zeros(),
            f64::NEG_INFINITY,
            std::slice::from_ref(&sphere),
        );
        assert!(!reactions.is_empty(), "no contact produced");
        let total: Vector3<f64> = reactions.iter().map(|r| r.impulse).sum();
        assert!(
            total.y > 0.0,
            "reaction should support the body upward, got {total:?}"
        );
        assert!(
            reactions.iter().all(|r| r.id == 7),
            "reaction routed to wrong body"
        );
    }

    #[test]
    fn self_collision_separates_a_fold_but_leaves_a_flat_sheet_alone() {
        let mut cloth = Cloth::new(15, 15, Vector3::new(0.0, 5.0, 0.0));
        let before = cloth.pos.clone();
        cloth.self_collide();
        assert!(
            cloth
                .pos
                .iter()
                .zip(&before)
                .all(|(p, q)| (p - q).norm() < 1e-12),
            "flat sheet self-collided",
        );

        let far = cloth.pos[cloth.idx(14, 14)];
        cloth.pos[0] = far + Vector3::new(0.01, 0.0, 0.0);
        cloth.prev[0] = cloth.pos[0];
        cloth.self_collide();
        let d = (cloth.pos[0] - cloth.pos[cloth.idx(14, 14)]).norm();
        assert!(
            d >= SELF_COLLIDE_DIST - 1e-6,
            "folded particles were not separated: d = {d}"
        );
    }
}
