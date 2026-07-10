use nalgebra::Vector3;

use crate::collider::{Collider, Reaction};

const H: f64 = 0.6;
const H2: f64 = H * H;
const SPACING: f64 = 0.32;
const PARTICLE_MASS: f64 = 1.0;
const GAS_CONST: f64 = 140.0;
const SUBSTEPS: usize = 4;
const MAX_SPEED: f64 = 14.0;

pub const DEFAULT_VISCOSITY: f64 = 6.0;
pub const MIN_VISCOSITY: f64 = 0.0;
pub const MAX_VISCOSITY: f64 = 40.0;

const WALL_RESTITUTION: f64 = 0.3;
const WALL_FRICTION: f64 = 0.9;
const BODY_FRICTION: f64 = 0.7;
const BUOYANCY_DENSITY: f64 = 55.0;
const BUOYANCY_DRAG: f64 = 160.0;
const BUOYANCY_REACH: f64 = 2.0 * H;
const BUOYANCY_SKIN: f64 = 0.5 * H;
const MIN_BUOY_PARTICLES: usize = 4;
const MAX_COUPLING_DV: f64 = 8.0;
const CONTACT_DRAG_PER_PARTICLE: f64 = 0.02;
const MAX_CONTACT_DAMP: f64 = 0.4;
const CLOTH_CONTACT: f64 = SPACING;
const CLOTH_COUPLE_FRICTION: f64 = 0.5;
const COLLISION_MARGIN: f64 = 0.02;
pub const MAX_PARTICLES: usize = 12000;

pub const EMIT_SPEED: f64 = 3.0;
pub const EMIT_RADIUS: f64 = 0.4;
const MIN_SPAWN_SPACING: f64 = 0.75;

const PAR_MIN: usize = 1024;

const NBR_SKIN: f64 = 0.25;
const NBR_CUTOFF2: f64 = (H + NBR_SKIN) * (H + NBR_SKIN);
const MAX_NBR: usize = 160;
const MAX_DIM: i32 = 256;

#[derive(Default)]
struct Grid {
    origin: [i32; 3],
    dim: [i32; 3],
    cell_start: Vec<u32>,
    sorted: Vec<u32>,
    cursor: Vec<u32>,
}

impl Grid {
    fn build(&mut self, pts: &[Vector3<f64>]) {
        let n = pts.len();
        if n == 0 {
            self.dim = [0, 0, 0];
            return;
        }
        let (mut lo, mut hi) = (pts[0], pts[0]);
        for p in &pts[1..] {
            lo = lo.inf(p);
            hi = hi.sup(p);
        }
        for a in 0..3 {
            self.origin[a] = (lo[a] / H).floor() as i32;
            self.dim[a] = (((hi[a] / H).floor() as i32) - self.origin[a] + 1).min(MAX_DIM);
        }
        let ncells = self.dim[0] as usize * self.dim[1] as usize * self.dim[2] as usize;
        self.cell_start.clear();
        self.cell_start.resize(ncells + 1, 0);
        for &p in pts {
            let c = self.flat(p);
            self.cell_start[c + 1] += 1;
        }
        for c in 1..=ncells {
            self.cell_start[c] += self.cell_start[c - 1];
        }
        self.cursor.clear();
        self.cursor.extend_from_slice(&self.cell_start[..ncells]);
        self.sorted.resize(n, 0);
        for (i, &p) in pts.iter().enumerate() {
            let c = self.flat(p);
            self.sorted[self.cursor[c] as usize] = i as u32;
            self.cursor[c] += 1;
        }
    }

    #[inline]
    fn coord(&self, p: Vector3<f64>, a: usize) -> i32 {
        (((p[a] / H).floor() as i32) - self.origin[a]).clamp(0, self.dim[a] - 1)
    }

    #[inline]
    fn flat(&self, p: Vector3<f64>) -> usize {
        let x = self.coord(p, 0);
        let y = self.coord(p, 1);
        let z = self.coord(p, 2);
        ((z * self.dim[1] + y) * self.dim[0] + x) as usize
    }

    #[inline]
    fn for_neighbors(&self, p: Vector3<f64>, mut f: impl FnMut(usize)) {
        let (cx, cy, cz) = (self.coord(p, 0), self.coord(p, 1), self.coord(p, 2));
        for dz in -1..=1 {
            let z = cz + dz;
            if z < 0 || z >= self.dim[2] {
                continue;
            }
            for dy in -1..=1 {
                let y = cy + dy;
                if y < 0 || y >= self.dim[1] {
                    continue;
                }
                for dx in -1..=1 {
                    let x = cx + dx;
                    if x < 0 || x >= self.dim[0] {
                        continue;
                    }
                    let c = ((z * self.dim[1] + y) * self.dim[0] + x) as usize;
                    for &j in
                        &self.sorted[self.cell_start[c] as usize..self.cell_start[c + 1] as usize]
                    {
                        f(j as usize);
                    }
                }
            }
        }
    }
}

#[derive(Clone, Copy)]
struct Emitter {
    pos: Vector3<f64>,
    vel: Vector3<f64>,
    radius: f64,
}

pub struct FluidView {
    pub positions: Vec<Vector3<f64>>,
    pub speeds: Vec<f64>,
    pub density_ratio: Vec<f64>,
    pub emitters: Vec<Vector3<f64>>,
}

pub struct Fluid {
    pos: Vec<Vector3<f64>>,
    vel: Vec<Vector3<f64>>,
    density: Vec<f64>,
    pressure: Vec<f64>,
    viscosity: f64,
    emitters: Vec<Emitter>,
    grid: Grid,
    nbr: Vec<u32>,
    nbr_count: Vec<u32>,
    forces: Vec<Vector3<f64>>,
    threads: usize,
    poly6: f64,
    spiky_grad: f64,
    visc_lap: f64,
    rest_density: f64,
}

impl Default for Fluid {
    fn default() -> Self {
        Self::new()
    }
}

impl Fluid {
    pub fn new() -> Self {
        use std::f64::consts::PI;
        let poly6 = 315.0 / (64.0 * PI * H.powi(9));
        let spiky_grad = -45.0 / (PI * H.powi(6));
        let visc_lap = 45.0 / (PI * H.powi(6));

        let reach = (H / SPACING).ceil() as i32;
        let mut rest_density = 0.0;
        for i in -reach..=reach {
            for j in -reach..=reach {
                for k in -reach..=reach {
                    let r2 = (SPACING * SPACING) * ((i * i + j * j + k * k) as f64);
                    if r2 < H2 {
                        rest_density += PARTICLE_MASS * poly6 * (H2 - r2).powi(3);
                    }
                }
            }
        }

        Self {
            pos: Vec::new(),
            vel: Vec::new(),
            density: Vec::new(),
            pressure: Vec::new(),
            viscosity: DEFAULT_VISCOSITY,
            emitters: Vec::new(),
            grid: Grid::default(),
            nbr: Vec::new(),
            nbr_count: Vec::new(),
            forces: Vec::new(),
            threads: std::thread::available_parallelism().map_or(1, |n| n.get()),
            poly6,
            spiky_grad,
            visc_lap,
            rest_density,
        }
    }

    pub fn spawn_block(&mut self, center: Vector3<f64>, half: Vector3<f64>) {
        let count = |half: f64| (half / SPACING) as i32;
        let (nx, ny, nz) = (count(half.x), count(half.y), count(half.z));
        let jitter = |a: i32, b: i32, c: i32| {
            let h = ((a.wrapping_mul(73856093))
                ^ (b.wrapping_mul(19349663))
                ^ (c.wrapping_mul(83492791))) as u32;
            (h as f64 / u32::MAX as f64 - 0.5) * SPACING * 0.2
        };
        for i in -nx..=nx {
            for j in -ny..=ny {
                for k in -nz..=nz {
                    if self.pos.len() >= MAX_PARTICLES {
                        return;
                    }
                    let p = center
                        + Vector3::new(
                            i as f64 * SPACING + jitter(i, j, k),
                            j as f64 * SPACING + jitter(j, k, i),
                            k as f64 * SPACING + jitter(k, i, j),
                        );
                    self.push_particle(p, Vector3::zeros());
                }
            }
        }
    }

    pub fn spawn_blob(&mut self, center: Vector3<f64>, radius: f64) {
        let n = (radius / SPACING).ceil() as i32;
        for i in -n..=n {
            for j in -n..=n {
                for k in -n..=n {
                    if self.pos.len() >= MAX_PARTICLES {
                        return;
                    }
                    let off = Vector3::new(i as f64, j as f64, k as f64) * SPACING;
                    if off.norm() > radius {
                        continue;
                    }
                    let p = center + off;
                    if !self.too_crowded(p) {
                        self.push_particle(p, Vector3::new(0.0, -1.0, 0.0));
                    }
                }
            }
        }
    }

    pub fn add_emitter(&mut self, pos: Vector3<f64>, vel: Vector3<f64>, radius: f64) {
        self.emitters.push(Emitter { pos, vel, radius });
    }

    pub fn remove_emitter_near(&mut self, point: Vector3<f64>, threshold: f64) -> bool {
        if let Some((i, _)) = self
            .emitters
            .iter()
            .enumerate()
            .map(|(i, e)| (i, (e.pos - point).norm()))
            .filter(|&(_, d)| d < threshold)
            .min_by(|a, b| a.1.total_cmp(&b.1))
        {
            self.emitters.remove(i);
            true
        } else {
            false
        }
    }

    pub fn emitter_count(&self) -> usize {
        self.emitters.len()
    }

    fn emit(&mut self) {
        if self.emitters.is_empty() {
            return;
        }
        for e in self.emitters.clone() {
            let n = (e.radius / SPACING).ceil() as i32;
            for i in -n..=n {
                for k in -n..=n {
                    if self.pos.len() >= MAX_PARTICLES {
                        return;
                    }
                    let off = Vector3::new(i as f64 * SPACING, 0.0, k as f64 * SPACING);
                    if off.norm() > e.radius {
                        continue;
                    }
                    let p = e.pos + off;
                    if !self.too_crowded(p) {
                        self.push_particle(p, e.vel);
                    }
                }
            }
        }
    }

    fn push_particle(&mut self, p: Vector3<f64>, vel: Vector3<f64>) {
        self.pos.push(p);
        self.vel.push(vel);
        self.density.push(self.rest_density);
        self.pressure.push(0.0);
    }

    fn too_crowded(&self, p: Vector3<f64>) -> bool {
        let min2 = (MIN_SPAWN_SPACING * SPACING).powi(2);
        self.pos.iter().any(|q| (q - p).norm_squared() < min2)
    }

    pub fn clear(&mut self) {
        self.pos.clear();
        self.vel.clear();
        self.density.clear();
        self.pressure.clear();
        self.emitters.clear();
    }

    pub fn cull_below(&mut self, y_limit: f64) {
        let mut i = 0;
        while i < self.pos.len() {
            if self.pos[i].y < y_limit {
                self.pos.swap_remove(i);
                self.vel.swap_remove(i);
                self.density.swap_remove(i);
                self.pressure.swap_remove(i);
            } else {
                i += 1;
            }
        }
    }

    pub fn len(&self) -> usize {
        self.pos.len()
    }

    pub fn is_empty(&self) -> bool {
        self.pos.is_empty()
    }

    pub fn viscosity(&self) -> f64 {
        self.viscosity
    }

    pub fn set_viscosity(&mut self, v: f64) {
        self.viscosity = v.clamp(MIN_VISCOSITY, MAX_VISCOSITY);
    }

    pub fn is_active(&self) -> bool {
        !self.is_empty() || !self.emitters.is_empty()
    }

    pub fn view(&self) -> FluidView {
        FluidView {
            positions: self.pos.clone(),
            speeds: self.vel.iter().map(|v| v.norm()).collect(),
            density_ratio: self
                .density
                .iter()
                .map(|&d| d / self.rest_density)
                .collect(),
            emitters: self.emitters.iter().map(|e| e.pos).collect(),
        }
    }

    pub fn step(
        &mut self,
        dt: f64,
        gravity: Vector3<f64>,
        ground_y: f64,
        ground_half: f64,
        bodies: &[Collider],
        container: Option<(Vector3<f64>, Vector3<f64>)>,
    ) -> Vec<Reaction> {
        self.emit();
        if self.pos.is_empty() {
            return Vec::new();
        }
        let h = dt / SUBSTEPS as f64;

        self.grid.build(&self.pos);
        self.build_neighbors();
        let mut contact: Vec<(Vector3<f64>, f64)> = vec![(Vector3::zeros(), 0.0); bodies.len()];
        for _ in 0..SUBSTEPS {
            self.compute_density_pressure();
            self.compute_forces();
            self.integrate(h, gravity);
            self.collide_floor(ground_y, ground_half);
            if let Some((min, max)) = container {
                self.collide_container(min, max);
            }
            self.collide_bodies(bodies, &mut contact);
        }

        let mut reactions = self.buoyancy_reactions(bodies, dt, -gravity.y);
        for (bi, body) in bodies.iter().enumerate() {
            let (mut impulse, count) = contact[bi];
            let inv_m = body.inv_mass();
            if count == 0.0 || inv_m <= 0.0 {
                continue;
            }
            let dv = impulse.norm() * inv_m;
            if dv > MAX_COUPLING_DV {
                impulse *= MAX_COUPLING_DV / dv;
            }
            let mass = 1.0 / inv_m;
            let vy = body.linvel().y;
            if impulse.y > 0.0 {
                impulse.y = impulse.y.min((-vy).max(0.0) * mass);
            }
            let damp = (count / SUBSTEPS as f64 * CONTACT_DRAG_PER_PARTICLE).min(MAX_CONTACT_DAMP);
            impulse -= body.linvel() * (mass * damp);
            reactions.push(Reaction {
                id: body.id(),
                impulse,
                point: body.center(),
            });
        }
        reactions
    }

    /// Archimedes buoyancy, resolved per body from the water actually touching it.
    fn buoyancy_reactions(&self, bodies: &[Collider], dt: f64, g: f64) -> Vec<Reaction> {
        if g <= 0.0 || self.pos.len() < 8 {
            return Vec::new();
        }
        bodies
            .iter()
            .filter_map(|body| {
                if body.inv_mass() <= 0.0 {
                    return None;
                }
                let c = body.center();
                let (hx, hz) = body.horizontal_half();
                let top = body.top_y();
                let bottom = 2.0 * c.y - top;
                let mut ys: Vec<f64> = self
                    .pos
                    .iter()
                    .filter(|p| {
                        (p.x - c.x).abs() <= hx + BUOYANCY_SKIN
                            && (p.z - c.z).abs() <= hz + BUOYANCY_SKIN
                            && p.y <= top + COLLISION_MARGIN
                            && p.y >= bottom - BUOYANCY_REACH
                    })
                    .map(|p| p.y)
                    .collect();
                if ys.len() < MIN_BUOY_PARTICLES {
                    return None;
                }
                ys.sort_by(f64::total_cmp);
                let waterline = ys[((ys.len() as f64 * 0.9) as usize).min(ys.len() - 1)];
                let v_sub = body.submerged_volume(waterline);
                if v_sub <= 1e-6 {
                    return None;
                }
                let buoy = BUOYANCY_DENSITY * v_sub * g;
                let drag = -body.linvel() * (BUOYANCY_DRAG * v_sub);
                let impulse = Vector3::new(drag.x, buoy + drag.y, drag.z) * dt;
                Some(Reaction {
                    id: body.id(),
                    impulse,
                    point: body.center(),
                })
            })
            .collect()
    }

    fn build_neighbors(&mut self) {
        let n = self.pos.len();
        self.nbr.resize(n * MAX_NBR, 0);
        self.nbr_count.resize(n, 0);
        let (pos, grid, threads) = (&self.pos, &self.grid, self.threads);
        let one = |i: usize, row: &mut [u32]| -> u32 {
            let pi = pos[i];
            let mut c = 0usize;
            grid.for_neighbors(pi, |j| {
                if j == i || c >= MAX_NBR {
                    return;
                }
                if (pos[j] - pi).norm_squared() < NBR_CUTOFF2 {
                    row[c] = j as u32;
                    c += 1;
                }
            });
            debug_assert!(
                c < MAX_NBR,
                "particle hit the MAX_NBR cap; density/pressure/viscosity for it are being \
                 computed from a truncated neighborhood, raise MAX_NBR if this fires in practice"
            );
            c as u32
        };
        let nbr = &mut self.nbr;
        let count = &mut self.nbr_count;
        if n < PAR_MIN {
            for i in 0..n {
                count[i] = one(i, &mut nbr[i * MAX_NBR..(i + 1) * MAX_NBR]);
            }
            return;
        }
        let chunk = n.div_ceil(threads);
        std::thread::scope(|s| {
            for (c, (rows, cnts)) in nbr
                .chunks_mut(chunk * MAX_NBR)
                .zip(count.chunks_mut(chunk))
                .enumerate()
            {
                let (base, one) = (c * chunk, &one);
                s.spawn(move || {
                    for k in 0..cnts.len() {
                        cnts[k] = one(base + k, &mut rows[k * MAX_NBR..(k + 1) * MAX_NBR]);
                    }
                });
            }
        });
    }

    fn compute_density_pressure(&mut self) {
        let n = self.pos.len();
        self.density.resize(n, 0.0);
        self.pressure.resize(n, 0.0);
        let (poly6, rest, threads) = (self.poly6, self.rest_density, self.threads);
        let (pos, nbr, count) = (&self.pos, &self.nbr, &self.nbr_count);
        let self_term = PARTICLE_MASS * poly6 * H2.powi(3);
        let one = |i: usize| -> (f64, f64) {
            let pi = pos[i];
            let mut density = self_term;
            for &j in &nbr[i * MAX_NBR..i * MAX_NBR + count[i] as usize] {
                let r2 = (pos[j as usize] - pi).norm_squared();
                if r2 < H2 {
                    density += PARTICLE_MASS * poly6 * (H2 - r2).powi(3);
                }
            }
            let density = density.max(rest * 0.2);
            (density, (GAS_CONST * (density - rest)).max(0.0))
        };
        let density = &mut self.density;
        let pressure = &mut self.pressure;
        if n < PAR_MIN {
            for i in 0..n {
                (density[i], pressure[i]) = one(i);
            }
            return;
        }
        let chunk = n.div_ceil(threads);
        std::thread::scope(|s| {
            for (c, (dch, pch)) in density
                .chunks_mut(chunk)
                .zip(pressure.chunks_mut(chunk))
                .enumerate()
            {
                let (base, one) = (c * chunk, &one);
                s.spawn(move || {
                    for k in 0..dch.len() {
                        (dch[k], pch[k]) = one(base + k);
                    }
                });
            }
        });
    }

    fn compute_forces(&mut self) {
        let n = self.pos.len();
        self.forces.resize(n, Vector3::zeros());
        let (visc, spiky, visc_lap, threads) =
            (self.viscosity, self.spiky_grad, self.visc_lap, self.threads);
        let (pos, vel, density, pressure) = (&self.pos, &self.vel, &self.density, &self.pressure);
        let (nbr, count) = (&self.nbr, &self.nbr_count);
        let one = |i: usize| -> Vector3<f64> {
            let pi = pos[i];
            let vi = vel[i];
            let pressure_i = pressure[i];
            let mut f_pressure = Vector3::zeros();
            let mut f_viscosity = Vector3::zeros();
            for &jj in &nbr[i * MAX_NBR..i * MAX_NBR + count[i] as usize] {
                let j = jj as usize;
                let rij = pi - pos[j];
                let r = rij.norm();
                if !(1e-9..H).contains(&r) {
                    continue;
                }
                let dir = rij / r;
                f_pressure += -dir * PARTICLE_MASS * (pressure_i + pressure[j])
                    / (2.0 * density[j])
                    * spiky
                    * (H - r).powi(2);
                f_viscosity +=
                    visc * PARTICLE_MASS * (vel[j] - vi) / density[j] * visc_lap * (H - r);
            }
            f_pressure + f_viscosity
        };
        let forces = &mut self.forces;
        if n < PAR_MIN {
            for (i, f) in forces.iter_mut().enumerate() {
                *f = one(i);
            }
            return;
        }
        let chunk = n.div_ceil(threads);
        std::thread::scope(|s| {
            for (c, fch) in forces.chunks_mut(chunk).enumerate() {
                let (base, one) = (c * chunk, &one);
                s.spawn(move || {
                    for (k, f) in fch.iter_mut().enumerate() {
                        *f = one(base + k);
                    }
                });
            }
        });
    }

    fn integrate(&mut self, h: f64, gravity: Vector3<f64>) {
        for i in 0..self.pos.len() {
            let accel = self.forces[i] / self.density[i] + gravity;
            let mut v = self.vel[i] + accel * h;
            let speed = v.norm();
            if speed > MAX_SPEED {
                v *= MAX_SPEED / speed;
            }
            self.vel[i] = v;
            self.pos[i] += v * h;
        }
    }

    fn bounce(v: &mut Vector3<f64>, axis: usize) {
        v[axis] = -v[axis] * WALL_RESTITUTION;
        for a in 0..3 {
            if a != axis {
                v[a] *= WALL_FRICTION;
            }
        }
    }

    fn collide_floor(&mut self, ground_y: f64, ground_half: f64) {
        if !ground_y.is_finite() {
            return;
        }
        for i in 0..self.pos.len() {
            if self.pos[i].x.abs() > ground_half || self.pos[i].z.abs() > ground_half {
                continue;
            }
            if self.pos[i].y < ground_y {
                self.pos[i].y = ground_y;
                if self.vel[i].y < 0.0 {
                    Self::bounce(&mut self.vel[i], 1);
                }
            }
        }
    }

    fn collide_container(&mut self, min: Vector3<f64>, max: Vector3<f64>) {
        for i in 0..self.pos.len() {
            for axis in [0, 2] {
                if self.pos[i][axis] < min[axis] {
                    self.pos[i][axis] = min[axis];
                    if self.vel[i][axis] < 0.0 {
                        Self::bounce(&mut self.vel[i], axis);
                    }
                } else if self.pos[i][axis] > max[axis] {
                    self.pos[i][axis] = max[axis];
                    if self.vel[i][axis] > 0.0 {
                        Self::bounce(&mut self.vel[i], axis);
                    }
                }
            }
        }
    }

    fn collide_bodies(&mut self, bodies: &[Collider], accum: &mut [(Vector3<f64>, f64)]) {
        for (bi, body) in bodies.iter().enumerate() {
            let dynamic = body.inv_mass() > 0.0;
            for i in 0..self.pos.len() {
                let Some((n, pen, contact)) = body.penetration(self.pos[i], COLLISION_MARGIN)
                else {
                    continue;
                };
                self.pos[i] += n * pen;

                let r = contact - body.center();
                let vb = body.linvel() + body.angvel().cross(&r);
                let rel = self.vel[i] - vb;
                let vn = rel.dot(&n);
                if vn >= 0.0 {
                    continue;
                }
                let rel_t = rel - n * vn;
                let new_vel = vb + rel_t * BODY_FRICTION;
                if dynamic {
                    accum[bi].0 += (self.vel[i] - new_vel) * PARTICLE_MASS;
                    accum[bi].1 += 1.0;
                }
                self.vel[i] = new_vel;
            }
        }
    }

    pub fn couple_cloth(
        &mut self,
        verts: &[Vector3<f64>],
        vels: &[Vector3<f64>],
        inv_mass: &[f64],
    ) -> Vec<Vector3<f64>> {
        let mut imp = vec![Vector3::zeros(); verts.len()];
        if self.pos.is_empty() || verts.is_empty() {
            return imp;
        }
        let mut vgrid = Grid::default();
        vgrid.build(verts);
        let r2 = CLOTH_CONTACT * CLOTH_CONTACT;
        let wf = 1.0 / PARTICLE_MASS;
        for i in 0..self.pos.len() {
            let pi = self.pos[i];
            vgrid.for_neighbors(pi, |k| {
                let wv = inv_mass[k];
                let w = wf + wv;
                if w == 0.0 {
                    return;
                }
                let d = self.pos[i] - verts[k];
                let dist2 = d.norm_squared();
                if dist2 >= r2 || dist2 < 1e-12 {
                    return;
                }
                let dist = dist2.sqrt();
                let n = d / dist;
                let pen = CLOTH_CONTACT - dist;
                self.pos[i] += n * (pen * wf / w);

                let rel = self.vel[i] - vels[k];
                let vn = rel.dot(&n);
                if vn >= 0.0 {
                    return;
                }
                let jn = -vn / w;
                self.vel[i] += n * (jn * wf);
                imp[k] -= n * jn;
                let vt = rel - n * vn;
                let jt = vt * ((1.0 - CLOTH_COUPLE_FRICTION) / w);
                self.vel[i] -= jt * wf;
                imp[k] += jt;
            });
        }
        imp
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const GRAVITY: Vector3<f64> = Vector3::new(0.0, -9.81, 0.0);
    const GROUND: f64 = 0.0;
    const DT: f64 = 1.0 / 60.0;

    fn all_finite(f: &Fluid) -> bool {
        f.pos
            .iter()
            .all(|p| p.x.is_finite() && p.y.is_finite() && p.z.is_finite())
    }

    #[test]
    fn block_settles_on_floor_without_exploding() {
        let mut fluid = Fluid::new();
        fluid.spawn_block(Vector3::new(0.0, 2.0, 0.0), Vector3::new(1.5, 1.5, 1.5));
        assert!(
            fluid.len() > 100,
            "block spawned too few particles: {}",
            fluid.len()
        );
        for _ in 0..400 {
            fluid.step(DT, GRAVITY, GROUND, f64::INFINITY, &[], None);
        }
        assert!(all_finite(&fluid), "fluid diverged");
        for p in &fluid.pos {
            assert!(p.y >= -0.1, "particle sank through the floor: {}", p.y);
        }
        let max_speed = fluid.vel.iter().map(|v| v.norm()).fold(0.0f64, f64::max);
        assert!(max_speed < 2.5, "pool never settled, max speed {max_speed}");
    }

    #[test]
    fn container_holds_the_pool_without_leaking() {
        let mut fluid = Fluid::new();
        let (min, max) = (Vector3::new(-1.5, 0.0, -1.5), Vector3::new(1.5, 3.0, 1.5));
        fluid.spawn_block(Vector3::new(0.0, 1.2, 0.0), Vector3::new(1.4, 1.2, 1.4));
        for _ in 0..400 {
            fluid.step(DT, GRAVITY, GROUND, f64::INFINITY, &[], Some((min, max)));
        }
        assert!(all_finite(&fluid), "fluid diverged");
        for p in &fluid.pos {
            assert!(
                p.x >= min.x - 1e-6
                    && p.x <= max.x + 1e-6
                    && p.z >= min.z - 1e-6
                    && p.z <= max.z + 1e-6,
                "particle leaked out of the container at {p:?}"
            );
        }
    }

    #[test]
    fn no_floor_lets_fluid_fall_away() {
        let mut fluid = Fluid::new();
        fluid.spawn_block(Vector3::new(0.0, 2.0, 0.0), Vector3::new(1.0, 1.0, 1.0));
        for _ in 0..120 {
            fluid.step(DT, GRAVITY, f64::NEG_INFINITY, f64::INFINITY, &[], None);
        }
        assert!(all_finite(&fluid), "fluid diverged");
        let max_y = fluid
            .pos
            .iter()
            .map(|p| p.y)
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(
            max_y < 0.0,
            "fluid did not fall away without a floor: max y = {max_y}"
        );
    }

    #[test]
    fn water_rests_on_a_cloth_and_weighs_it_down() {
        let mut fluid = Fluid::new();
        let mut verts = Vec::new();
        for i in -4..=4 {
            for j in -4..=4 {
                verts.push(Vector3::new(i as f64 * SPACING, 0.0, j as f64 * SPACING));
            }
        }
        let vels = vec![Vector3::zeros(); verts.len()];
        let inv_mass = vec![0.0; verts.len()];

        for i in -2..=2 {
            for j in -2..=2 {
                fluid.push_particle(
                    Vector3::new(i as f64 * SPACING, 0.15, j as f64 * SPACING),
                    Vector3::new(0.0, -2.0, 0.0),
                );
            }
        }

        let imp = fluid.couple_cloth(&verts, &vels, &inv_mass);

        for (p, v) in fluid.pos.iter().zip(&fluid.vel) {
            assert!(p.y > -1e-9, "fluid tunnelled below the sheet: y = {}", p.y);
            assert!(
                v.y > -0.5,
                "fluid kept plunging through the sheet: vy = {}",
                v.y
            );
        }
        let total: Vector3<f64> = imp.iter().sum();
        assert!(
            total.y < 0.0,
            "water did not weigh the sheet down: {total:?}"
        );
    }

    #[test]
    fn submerged_body_feels_upward_buoyancy() {
        let mut fluid = Fluid::new();
        fluid.spawn_block(Vector3::new(0.0, 1.5, 0.0), Vector3::new(2.0, 1.5, 2.0));
        for _ in 0..90 {
            fluid.step(DT, GRAVITY, GROUND, f64::INFINITY, &[], None);
        }
        let body = Collider::Cuboid {
            id: 0,
            center: Vector3::new(0.0, 0.6, 0.0),
            basis: [Vector3::x(), Vector3::y(), Vector3::z()],
            half: Vector3::new(0.6, 0.6, 0.6),
            inv_mass: 1.0 / 10.0,
            linvel: Vector3::zeros(),
            angvel: Vector3::zeros(),
        };
        let reactions = fluid.step(
            DT,
            GRAVITY,
            GROUND,
            f64::INFINITY,
            std::slice::from_ref(&body),
            None,
        );
        let total: Vector3<f64> = reactions.iter().map(|r| r.impulse).sum();
        assert!(
            total.y > 0.0,
            "buoyancy should push a submerged body up, got {total:?}"
        );
    }

    #[test]
    fn body_outside_the_pool_is_not_buoyed() {
        let mut fluid = Fluid::new();
        fluid.spawn_block(Vector3::new(0.0, 1.5, 0.0), Vector3::new(2.0, 1.5, 2.0));
        for _ in 0..120 {
            fluid.step(DT, GRAVITY, GROUND, f64::INFINITY, &[], None);
        }
        let surface = {
            let mut ys: Vec<f64> = fluid.pos.iter().map(|p| p.y).collect();
            ys.sort_by(f64::total_cmp);
            ys[((ys.len() as f64 * 0.9) as usize).min(ys.len() - 1)]
        };
        let body = Collider::Cuboid {
            id: 0,
            center: Vector3::new(50.0, surface - 0.5, 0.0),
            basis: [Vector3::x(), Vector3::y(), Vector3::z()],
            half: Vector3::new(0.6, 0.6, 0.6),
            inv_mass: 1.0 / 10.0,
            linvel: Vector3::zeros(),
            angvel: Vector3::zeros(),
        };
        let reactions = fluid.step(
            DT,
            GRAVITY,
            GROUND,
            f64::INFINITY,
            std::slice::from_ref(&body),
            None,
        );
        let up: f64 = reactions.iter().map(|r| r.impulse.y).sum();
        assert_eq!(
            up, 0.0,
            "a body far from the pool must not be buoyed, got {up}"
        );
    }

    #[test]
    fn body_lifted_out_of_the_pool_stops_being_buoyed() {
        let mut fluid = Fluid::new();
        fluid.spawn_block(Vector3::new(0.0, 1.5, 0.0), Vector3::new(2.0, 1.5, 2.0));
        for _ in 0..120 {
            fluid.step(DT, GRAVITY, GROUND, f64::INFINITY, &[], None);
        }
        let surface = {
            let mut ys: Vec<f64> = fluid.pos.iter().map(|p| p.y).collect();
            ys.sort_by(f64::total_cmp);
            ys[((ys.len() as f64 * 0.9) as usize).min(ys.len() - 1)]
        };
        let body = Collider::Cuboid {
            id: 0,
            center: Vector3::new(0.0, surface + 2.0, 0.0),
            basis: [Vector3::x(), Vector3::y(), Vector3::z()],
            half: Vector3::new(0.5, 0.5, 0.5),
            inv_mass: 1.0 / 10.0,
            linvel: Vector3::zeros(),
            angvel: Vector3::zeros(),
        };
        let reactions = fluid.step(
            DT,
            GRAVITY,
            GROUND,
            f64::INFINITY,
            std::slice::from_ref(&body),
            None,
        );
        let up: f64 = reactions.iter().map(|r| r.impulse.y).sum();
        assert_eq!(
            up, 0.0,
            "a body lifted clear of the pool must not be buoyed, got {up}"
        );
    }

    #[test]
    fn body_over_a_thin_film_beside_a_deep_pool_is_not_buoyed() {
        let mut fluid = Fluid::new();
        fluid.spawn_block(Vector3::new(0.0, 1.5, 0.0), Vector3::new(2.0, 1.5, 2.0));
        fluid.spawn_block(Vector3::new(8.0, 0.3, 0.0), Vector3::new(2.0, 0.2, 2.0));
        for _ in 0..120 {
            fluid.step(DT, GRAVITY, GROUND, f64::INFINITY, &[], None);
        }
        let body = Collider::Cuboid {
            id: 0,
            center: Vector3::new(8.0, 1.5, 0.0),
            basis: [Vector3::x(), Vector3::y(), Vector3::z()],
            half: Vector3::new(0.6, 0.6, 0.6),
            inv_mass: 1.0 / 10.0,
            linvel: Vector3::zeros(),
            angvel: Vector3::zeros(),
        };
        let reactions = fluid.step(
            DT,
            GRAVITY,
            GROUND,
            f64::INFINITY,
            std::slice::from_ref(&body),
            None,
        );
        let up: f64 = reactions.iter().map(|r| r.impulse.y).sum();
        assert_eq!(
            up, 0.0,
            "a body over a thin film must be weighed against that film, not a deep pool elsewhere; got {up}"
        );
    }

    #[test]
    fn body_outside_a_wall_is_not_buoyed_by_the_pool_inside() {
        let container = Some((Vector3::new(-2.4, 0.0, -2.4), Vector3::new(2.4, 3.0, 2.4)));
        let mut fluid = Fluid::new();
        fluid.spawn_block(Vector3::new(0.0, 1.4, 0.0), Vector3::new(2.3, 1.4, 2.3));
        for _ in 0..200 {
            fluid.step(DT, GRAVITY, GROUND, f64::INFINITY, &[], container);
        }
        let body = Collider::Sphere {
            id: 0,
            center: Vector3::new(0.0, 1.5, -3.6),
            radius: 0.5,
            inv_mass: 1.0 / 3.0,
            linvel: Vector3::zeros(),
            angvel: Vector3::zeros(),
        };
        let reactions = fluid.step(
            DT,
            GRAVITY,
            GROUND,
            f64::INFINITY,
            std::slice::from_ref(&body),
            container,
        );
        let up: f64 = reactions.iter().map(|r| r.impulse.y).sum();
        assert_eq!(
            up, 0.0,
            "the pool inside the tank must not buoy a body outside the wall; got {up}"
        );
    }
}
