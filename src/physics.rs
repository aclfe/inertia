use std::collections::HashMap;
use std::collections::VecDeque;

use nalgebra::Vector3;
use rapier3d::prelude::{
    BroadPhaseBvh, CCDSolver, ColliderBuilder, ColliderSet, ImpulseJointSet, IntegrationParameters,
    IslandManager, MultibodyJointSet, NarrowPhase, PhysicsPipeline, RigidBodyBuilder,
    RigidBodyHandle, RigidBodySet, RigidBodyType, SharedShape, Vector,
};

use crate::cloth::{self, Cloth, ClothView};
use crate::collider::{Collider, Reaction};
use crate::fluid::{self, Fluid, FluidView};
use crate::nbody;

const BOX_HALF_EXTENT: f32 = 0.5;
const SPHERE_RADIUS: f32 = 0.5;
const STAR_RADIUS: f32 = 0.5;
const GROUND_HALF_HEIGHT: f32 = 0.5;

pub const DEFAULT_GRAVITY: f64 = 9.81;
pub const DEFAULT_FRICTION: f64 = 0.7;
pub const DEFAULT_RESTITUTION: f64 = 0.5;
pub const DEFAULT_DAMPING: f64 = 0.0;
pub const DEFAULT_WIND_STRENGTH: f64 = 1.0;
pub const MAX_WIND_STRENGTH: f64 = 4.0;
pub const MAX_GRAVITY: f64 = 40.0;
pub const MAX_FRICTION: f64 = 2.0;
pub const MAX_RESTITUTION: f64 = 1.0;
pub const MAX_DAMPING: f64 = 5.0;
const WALL_HALF_HEIGHT: f32 = 3.0;
const WALL_HALF_THICKNESS: f32 = 0.25;

pub const G: f64 = 1.0;
const SOFTENING2: f64 = 0.25;
const THETA: f64 = 0.7;
const TRAIL_MAX: usize = 160;
const TRAIL_EVERY: u32 = 2;

pub const GRID_HALF_EXTENT: f32 = 10.0;

const VOID_FALL_LIMIT: f32 = -15.0;

const CLOTH_COLLISION_PASSES: usize = 2;

const CONTAINER_WALL_THICKNESS: f32 = 0.15;

const FLUID_EMITTER_TOGGLE_DIST: f64 = 0.8;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SpawnKind {
    Box,
    Sphere,
    Star,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum GravityMode {
    Uniform,
    NBody,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum NBodyAlgo {
    BruteForce,
    BarnesHut,
}

impl NBodyAlgo {
    pub fn next(self) -> Self {
        match self {
            NBodyAlgo::BruteForce => NBodyAlgo::BarnesHut,
            NBodyAlgo::BarnesHut => NBodyAlgo::BruteForce,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum FloorMode {
    Normal,
    Infinite,
    Void,
}

impl FloorMode {
    pub fn next(self) -> Self {
        match self {
            FloorMode::Normal => FloorMode::Infinite,
            FloorMode::Infinite => FloorMode::Void,
            FloorMode::Void => FloorMode::Normal,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum BodyKind {
    Box { half_extents: Vector3<f64> },
    Sphere { radius: f64 },
    Star { radius: f64 },
}

pub struct BodyView {
    pub handle: RigidBodyHandle,
    pub kind: BodyKind,
    pub position: Vector3<f64>,
    pub basis: [Vector3<f64>; 3],
    pub linvel: Vector3<f64>,
    pub speed: f64,
    pub mass: f64,
    pub trail: Vec<Vector3<f64>>,
}

#[derive(Clone, Copy)]
struct NBodyState {
    pos: Vector3<f64>,
    vel: Vector3<f64>,
    mass: f64,
}

pub struct PhysicsWorld {
    pipeline: PhysicsPipeline,
    integration_parameters: IntegrationParameters,
    islands: IslandManager,
    broad_phase: BroadPhaseBvh,
    narrow_phase: NarrowPhase,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
    ground: RigidBodyHandle,
    floor_mode: FloorMode,
    order: Vec<(RigidBodyHandle, BodyKind)>,
    gravity_mode: GravityMode,
    nbody_algo: NBodyAlgo,
    gravity: f64,
    friction: f64,
    restitution: f64,
    damping: f64,
    show_trails: bool,
    trails: HashMap<RigidBodyHandle, VecDeque<Vector3<f64>>>,
    trail_tick: u32,
    cloths: Vec<Cloth>,
    wind: bool,
    wind_strength: f64,
    fluid: Fluid,
    nbody_state: HashMap<RigidBodyHandle, NBodyState>,
    dragging: Option<RigidBodyHandle>,
    container: Option<Container>,
}

struct Container {
    handle: RigidBodyHandle,
    walls: [(Vector3<f64>, Vector3<f64>); 4],
    interior: (Vector3<f64>, Vector3<f64>),
}

impl PhysicsWorld {
    pub fn new() -> Self {
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();
        let floor_mode = FloorMode::Infinite;
        let ground = Self::build_ground(&mut bodies, &mut colliders, floor_mode);

        let mut world = Self {
            pipeline: PhysicsPipeline::new(),
            integration_parameters: IntegrationParameters::default(),
            islands: IslandManager::new(),
            broad_phase: BroadPhaseBvh::new(),
            narrow_phase: NarrowPhase::new(),
            bodies,
            colliders,
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            ground,
            floor_mode,
            order: Vec::new(),
            gravity_mode: GravityMode::Uniform,
            nbody_algo: NBodyAlgo::BarnesHut,
            gravity: DEFAULT_GRAVITY,
            friction: DEFAULT_FRICTION,
            restitution: DEFAULT_RESTITUTION,
            damping: DEFAULT_DAMPING,
            show_trails: false,
            trails: HashMap::new(),
            trail_tick: 0,
            cloths: Vec::new(),
            wind: false,
            wind_strength: DEFAULT_WIND_STRENGTH,
            fluid: Fluid::new(),
            nbody_state: HashMap::new(),
            dragging: None,
            container: None,
        };
        world.apply_materials();
        world
    }

    fn build_ground(
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        mode: FloorMode,
    ) -> RigidBodyHandle {
        let ground = bodies.insert(RigidBodyBuilder::fixed());
        match mode {
            FloorMode::Infinite => {
                let floor = ColliderBuilder::new(SharedShape::halfspace(Vector::Y));
                colliders.insert_with_parent(floor, ground, bodies);
            }
            FloorMode::Normal => {
                let floor =
                    ColliderBuilder::cuboid(GRID_HALF_EXTENT, GROUND_HALF_HEIGHT, GRID_HALF_EXTENT)
                        .translation(Vector::new(0.0, -GROUND_HALF_HEIGHT, 0.0));
                colliders.insert_with_parent(floor, ground, bodies);
                let walls = [
                    (
                        GRID_HALF_EXTENT + WALL_HALF_THICKNESS,
                        0.0,
                        WALL_HALF_THICKNESS,
                        GRID_HALF_EXTENT,
                    ),
                    (
                        -GRID_HALF_EXTENT - WALL_HALF_THICKNESS,
                        0.0,
                        WALL_HALF_THICKNESS,
                        GRID_HALF_EXTENT,
                    ),
                    (
                        0.0,
                        GRID_HALF_EXTENT + WALL_HALF_THICKNESS,
                        GRID_HALF_EXTENT,
                        WALL_HALF_THICKNESS,
                    ),
                    (
                        0.0,
                        -GRID_HALF_EXTENT - WALL_HALF_THICKNESS,
                        GRID_HALF_EXTENT,
                        WALL_HALF_THICKNESS,
                    ),
                ];
                for (x, z, hx, hz) in walls {
                    let wall = ColliderBuilder::cuboid(hx, WALL_HALF_HEIGHT, hz)
                        .translation(Vector::new(x, WALL_HALF_HEIGHT, z));
                    colliders.insert_with_parent(wall, ground, bodies);
                }
            }
            FloorMode::Void => {}
        }
        ground
    }

    pub fn floor_mode(&self) -> FloorMode {
        self.floor_mode
    }

    pub fn set_floor_mode(&mut self, mode: FloorMode) {
        if mode == self.floor_mode {
            return;
        }
        self.bodies.remove(
            self.ground,
            &mut self.islands,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            true,
        );
        self.ground = Self::build_ground(&mut self.bodies, &mut self.colliders, mode);
        self.floor_mode = mode;
        self.apply_materials();
    }

    pub fn spawn_container(&mut self, center: Vector3<f64>, half_extent: f64, wall_height: f64) {
        self.remove_container();
        let t = CONTAINER_WALL_THICKNESS as f64;
        let outer = half_extent + t / 2.0;
        let wh = wall_height / 2.0;
        let walls_spec = [
            (
                center + Vector3::new(half_extent + t / 2.0, wh, 0.0),
                Vector3::new(t / 2.0, wh, outer),
            ),
            (
                center + Vector3::new(-half_extent - t / 2.0, wh, 0.0),
                Vector3::new(t / 2.0, wh, outer),
            ),
            (
                center + Vector3::new(0.0, wh, half_extent + t / 2.0),
                Vector3::new(outer, wh, t / 2.0),
            ),
            (
                center + Vector3::new(0.0, wh, -half_extent - t / 2.0),
                Vector3::new(outer, wh, t / 2.0),
            ),
        ];
        let handle = self.bodies.insert(RigidBodyBuilder::fixed());
        let mut walls = [(Vector3::zeros(), Vector3::zeros()); 4];
        for (i, (wcenter, half)) in walls_spec.into_iter().enumerate() {
            let collider =
                ColliderBuilder::cuboid(half.x as f32, half.y as f32, half.z as f32).translation(
                    Vector::new(wcenter.x as f32, wcenter.y as f32, wcenter.z as f32),
                );
            self.colliders
                .insert_with_parent(collider, handle, &mut self.bodies);
            walls[i] = (wcenter, half);
        }
        let interior = (
            center + Vector3::new(-half_extent, 0.0, -half_extent),
            center + Vector3::new(half_extent, wall_height, half_extent),
        );
        self.container = Some(Container {
            handle,
            walls,
            interior,
        });
        self.apply_materials();
    }

    pub fn remove_container(&mut self) {
        if let Some(c) = self.container.take() {
            self.bodies.remove(
                c.handle,
                &mut self.islands,
                &mut self.colliders,
                &mut self.impulse_joints,
                &mut self.multibody_joints,
                true,
            );
        }
    }

    pub fn container_walls(&self) -> &[(Vector3<f64>, Vector3<f64>)] {
        self.container.as_ref().map_or(&[], |c| &c.walls)
    }

    fn container_interior(&self) -> Option<(Vector3<f64>, Vector3<f64>)> {
        self.container.as_ref().map(|c| c.interior)
    }

    pub fn gravity_mode(&self) -> GravityMode {
        self.gravity_mode
    }

    pub fn set_gravity_mode(&mut self, mode: GravityMode) {
        if mode == self.gravity_mode {
            return;
        }
        self.gravity_mode = mode;
        let handles: Vec<RigidBodyHandle> = self.order.iter().map(|&(h, _)| h).collect();
        match mode {
            GravityMode::NBody => {
                for handle in handles {
                    if let Some(body) = self.bodies.get_mut(handle) {
                        let t = body.translation();
                        let lv = body.linvel();
                        let pos = Vector3::new(t.x as f64, t.y as f64, t.z as f64);
                        let vel = Vector3::new(lv.x as f64, lv.y as f64, lv.z as f64);
                        let mass = body.mass() as f64;
                        body.reset_forces(true);
                        body.set_body_type(RigidBodyType::KinematicPositionBased, true);
                        self.nbody_state
                            .insert(handle, NBodyState { pos, vel, mass });
                    }
                }
            }
            GravityMode::Uniform => {
                for handle in handles {
                    if Some(handle) == self.dragging {
                        continue;
                    }
                    let vel = self
                        .nbody_state
                        .get(&handle)
                        .map_or(Vector3::zeros(), |s| s.vel);
                    if let Some(body) = self.bodies.get_mut(handle) {
                        body.set_body_type(RigidBodyType::Dynamic, true);
                        body.set_linvel(
                            Vector::new(vel.x as f32, vel.y as f32, vel.z as f32),
                            true,
                        );
                        body.set_angvel(Vector::ZERO, true);
                        body.set_linear_damping(self.damping as f32);
                    }
                }
                self.nbody_state.retain(|h, _| Some(*h) == self.dragging);
            }
        }
    }

    pub fn nbody_algo(&self) -> NBodyAlgo {
        self.nbody_algo
    }

    pub fn set_nbody_algo(&mut self, algo: NBodyAlgo) {
        self.nbody_algo = algo;
    }

    pub fn show_trails(&self) -> bool {
        self.show_trails
    }

    pub fn set_show_trails(&mut self, on: bool) {
        self.show_trails = on;
        if !on {
            self.trails.clear();
        }
    }

    pub fn gravity(&self) -> f64 {
        self.gravity
    }

    pub fn set_gravity(&mut self, g: f64) {
        self.gravity = g.clamp(0.0, MAX_GRAVITY);
    }

    pub fn friction(&self) -> f64 {
        self.friction
    }

    pub fn set_friction(&mut self, f: f64) {
        self.friction = f.clamp(0.0, MAX_FRICTION);
        self.apply_materials();
    }

    pub fn restitution(&self) -> f64 {
        self.restitution
    }

    pub fn set_restitution(&mut self, e: f64) {
        self.restitution = e.clamp(0.0, MAX_RESTITUTION);
        self.apply_materials();
    }

    pub fn damping(&self) -> f64 {
        self.damping
    }

    pub fn set_damping(&mut self, d: f64) {
        self.damping = d.clamp(0.0, MAX_DAMPING);
        for &(handle, _) in &self.order {
            if let Some(body) = self.bodies.get_mut(handle) {
                body.set_linear_damping(self.damping as f32);
            }
        }
    }

    fn apply_materials(&mut self) {
        let (friction, restitution) = (self.friction as f32, self.restitution as f32);
        for (_, collider) in self.colliders.iter_mut() {
            collider.set_friction(friction);
            collider.set_restitution(restitution);
        }
    }

    pub fn step(&mut self, dt: f64) {
        self.integration_parameters.dt = dt as f32;
        let gravity = match self.gravity_mode {
            GravityMode::Uniform => Vector::new(0.0, -self.gravity as f32, 0.0),
            GravityMode::NBody => {
                self.step_nbody(dt);
                Vector::new(0.0, 0.0, 0.0)
            }
        };
        self.pipeline.step(
            gravity,
            &self.integration_parameters,
            &mut self.islands,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd_solver,
            &(),
            &(),
        );
        if self.floor_mode == FloorMode::Void {
            self.cull_fallen();
            self.fluid.cull_below(VOID_FALL_LIMIT as f64);
        }
        self.step_cloth(dt);
        self.step_fluid(dt);
        self.couple_fluid_cloth(dt);
        self.record_trails();
    }

    fn couple_fluid_cloth(&mut self, dt: f64) {
        if !self.fluid.is_active() || self.cloths.is_empty() {
            return;
        }
        for cloth in &mut self.cloths {
            let (pos, vel, inv_mass) = cloth.fluid_coupling_state(dt);
            let imp = self.fluid.couple_cloth(&pos, &vel, &inv_mass);
            cloth.apply_fluid_impulses(&imp, dt);
        }
    }

    fn collect_colliders(&self) -> (Vec<RigidBodyHandle>, Vec<Collider>) {
        let mut handles = Vec::new();
        let mut colliders = Vec::new();
        for &(handle, kind) in &self.order {
            let Some(body) = self.bodies.get(handle) else {
                continue;
            };
            let t = body.translation();
            let center = Vector3::new(t.x as f64, t.y as f64, t.z as f64);
            let inv_mass = if body.is_dynamic() {
                (1.0 / body.mass()) as f64
            } else {
                0.0
            };
            let lv = body.linvel();
            let av = body.angvel();
            let linvel = Vector3::new(lv.x as f64, lv.y as f64, lv.z as f64);
            let angvel = Vector3::new(av.x as f64, av.y as f64, av.z as f64);
            let id = handles.len();
            let collider = match kind {
                BodyKind::Sphere { radius } | BodyKind::Star { radius } => Collider::Sphere {
                    id,
                    center,
                    radius,
                    inv_mass,
                    linvel,
                    angvel,
                },
                BodyKind::Box { half_extents } => {
                    let rot = *body.rotation();
                    let to_v3 = |v: Vector| Vector3::new(v.x as f64, v.y as f64, v.z as f64);
                    Collider::Cuboid {
                        id,
                        center,
                        basis: [
                            to_v3(rot * Vector::X),
                            to_v3(rot * Vector::Y),
                            to_v3(rot * Vector::Z),
                        ],
                        half: half_extents,
                        inv_mass,
                        linvel,
                        angvel,
                    }
                }
            };
            handles.push(handle);
            colliders.push(collider);
        }
        (handles, colliders)
    }

    fn step_fluid(&mut self, dt: f64) {
        if !self.fluid.is_active() {
            return;
        }
        let (handles, colliders) = self.collect_colliders();
        let container = self.container_interior();

        let gravity = Vector3::new(0.0, -self.gravity, 0.0);
        let ground_y = if self.floor_mode == FloorMode::Void {
            f64::NEG_INFINITY
        } else {
            0.0
        };
        let ground_half = match self.floor_mode {
            FloorMode::Normal => GRID_HALF_EXTENT as f64,
            _ => f64::INFINITY,
        };
        let reactions = self
            .fluid
            .step(dt, gravity, ground_y, ground_half, &colliders, container);
        self.apply_reactions(&handles, reactions);
    }

    fn apply_reactions(
        &mut self,
        handles: &[RigidBodyHandle],
        reactions: impl IntoIterator<Item = Reaction>,
    ) {
        for r in reactions {
            if let Some(body) = self.bodies.get_mut(handles[r.id]) {
                let impulse =
                    Vector::new(r.impulse.x as f32, r.impulse.y as f32, r.impulse.z as f32);
                let point = Vector::new(r.point.x as f32, r.point.y as f32, r.point.z as f32);
                body.apply_impulse_at_point(impulse, point, true);
            }
        }
    }

    fn step_cloth(&mut self, dt: f64) {
        if self.cloths.is_empty() {
            return;
        }
        let (handles, mut colliders) = self.collect_colliders();
        for &(wcenter, half) in self.container_walls() {
            let id = colliders.len();
            colliders.push(Collider::Cuboid {
                id,
                center: wcenter,
                basis: [Vector3::x(), Vector3::y(), Vector3::z()],
                half,
                inv_mass: 0.0,
                linvel: Vector3::zeros(),
                angvel: Vector3::zeros(),
            });
        }

        let gravity = Vector3::new(0.0, -self.gravity, 0.0);
        let wind = if self.wind {
            cloth::WIND_ACCEL * self.wind_strength
        } else {
            Vector3::zeros()
        };
        let ground_y = if self.floor_mode == FloorMode::Void {
            f64::NEG_INFINITY
        } else {
            0.0
        };

        let mut reactions = Vec::new();
        for cloth in &mut self.cloths {
            reactions.extend(cloth.step(dt, gravity, wind, ground_y, &colliders));
        }
        self.apply_reactions(&handles, reactions);

        for _ in 0..CLOTH_COLLISION_PASSES {
            for a in 0..self.cloths.len() {
                for b in (a + 1)..self.cloths.len() {
                    let (left, right) = self.cloths.split_at_mut(b);
                    left[a].collide_with(&mut right[0]);
                }
            }
        }
    }

    pub fn spawn_cloth(&mut self, origin: Vector3<f64>) {
        self.cloths.push(Cloth::new(15, 15, origin));
    }

    pub fn spawn_cloth_hammock(&mut self, origin: Vector3<f64>, span_frac: f64) {
        let (cols, rows) = (15, 15);
        let mut cloth = Cloth::new(cols, rows, origin);
        for i in [0, cols - 1, cols * (rows - 1), cols * rows - 1] {
            let anchor = origin + (cloth.particle_pos(i) - origin) * span_frac;
            cloth.pin_at(i, anchor);
        }
        self.cloths.push(cloth);
    }

    pub fn remove_cloth(&mut self) {
        self.cloths.pop();
    }

    pub fn has_cloth(&self) -> bool {
        !self.cloths.is_empty()
    }

    pub fn cloth_count(&self) -> usize {
        self.cloths.len()
    }

    pub fn cloth_views(&self) -> Vec<ClothView> {
        self.cloths.iter().map(|c| c.view()).collect()
    }

    pub fn spawn_fluid_block(&mut self, center: Vector3<f64>, half: Vector3<f64>) {
        self.fluid.spawn_block(center, half);
    }

    pub fn spawn_fluid_blob(&mut self, center: Vector3<f64>) {
        self.fluid.spawn_blob(center, fluid::EMIT_RADIUS);
    }

    pub fn toggle_fluid_emitter(&mut self, pos: Vector3<f64>) {
        if !self
            .fluid
            .remove_emitter_near(pos, FLUID_EMITTER_TOGGLE_DIST)
        {
            self.fluid.add_emitter(
                pos,
                Vector3::new(0.0, -fluid::EMIT_SPEED, 0.0),
                fluid::EMIT_RADIUS,
            );
        }
    }

    pub fn fluid_emitter_count(&self) -> usize {
        self.fluid.emitter_count()
    }

    pub fn has_fluid(&self) -> bool {
        self.fluid.is_active()
    }

    pub fn fluid_len(&self) -> usize {
        self.fluid.len()
    }

    pub fn fluid_view(&self) -> Option<FluidView> {
        self.fluid.is_active().then(|| self.fluid.view())
    }

    pub fn fluid_viscosity(&self) -> f64 {
        self.fluid.viscosity()
    }

    pub fn set_fluid_viscosity(&mut self, v: f64) {
        self.fluid.set_viscosity(v);
    }

    pub fn clear_fluid(&mut self) {
        self.fluid.clear();
    }

    pub fn wind(&self) -> bool {
        self.wind
    }

    pub fn toggle_wind(&mut self) {
        self.wind = !self.wind;
    }

    pub fn wind_strength(&self) -> f64 {
        self.wind_strength
    }

    pub fn set_wind_strength(&mut self, s: f64) {
        self.wind_strength = s.clamp(0.0, MAX_WIND_STRENGTH);
    }

    pub fn toggle_cloth_pin_near(
        &mut self,
        target: (f64, f64),
        threshold: f64,
        project: impl Fn(Vector3<f64>) -> Option<(f64, f64)>,
    ) -> bool {
        match self.nearest_vertex(target, threshold, project) {
            Some((cloth, i)) => {
                self.cloths[cloth].toggle_pin(i);
                true
            }
            None => false,
        }
    }

    pub fn cloth_stiffness(&self) -> Option<f64> {
        self.cloths.first().map(|c| c.stiffness())
    }

    pub fn set_cloth_stiffness(&mut self, s: f64) {
        for cloth in &mut self.cloths {
            cloth.set_stiffness(s);
        }
    }

    pub fn grab_cloth_near(
        &mut self,
        target: (f64, f64),
        threshold: f64,
        project: impl Fn(Vector3<f64>) -> Option<(f64, f64)>,
    ) -> Option<Vector3<f64>> {
        let (cloth, i) = self.nearest_vertex(target, threshold, project)?;
        self.cloths[cloth].grab(i);
        Some(self.cloths[cloth].particle_pos(i))
    }

    fn nearest_vertex(
        &self,
        target: (f64, f64),
        threshold: f64,
        project: impl Fn(Vector3<f64>) -> Option<(f64, f64)>,
    ) -> Option<(usize, usize)> {
        let mut best: Option<(usize, usize, f64)> = None;
        for (ci, cloth) in self.cloths.iter().enumerate() {
            for (i, &p) in cloth.positions().iter().enumerate() {
                if let Some((x, y)) = project(p) {
                    let d = (x - target.0).powi(2) + (y - target.1).powi(2);
                    if best.is_none_or(|(_, _, bd)| d < bd) {
                        best = Some((ci, i, d));
                    }
                }
            }
        }
        best.filter(|&(_, _, d)| d < threshold * threshold)
            .map(|(ci, i, _)| (ci, i))
    }

    pub fn drag_cloth_to(&mut self, target: Vector3<f64>) {
        for cloth in &mut self.cloths {
            if cloth.is_grabbed() {
                cloth.drag_to(target);
            }
        }
    }

    pub fn release_cloth(&mut self) {
        for cloth in &mut self.cloths {
            cloth.release();
        }
    }

    pub fn any_cloth_grabbed(&self) -> bool {
        self.cloths.iter().any(|c| c.is_grabbed())
    }

    fn step_nbody(&mut self, dt: f64) {
        let handles: Vec<RigidBodyHandle> = self.order.iter().map(|&(h, _)| h).collect();
        if handles.is_empty() {
            return;
        }
        let algo = self.nbody_algo;
        let dragging = self.dragging;

        let mut pos = Vec::with_capacity(handles.len());
        let mut vel = Vec::with_capacity(handles.len());
        let mut mass = Vec::with_capacity(handles.len());
        let mut integrated = Vec::with_capacity(handles.len());
        for &h in &handles {
            if let (false, Some(body)) = (self.nbody_state.contains_key(&h), self.bodies.get(h)) {
                let t = body.translation();
                let lv = body.linvel();
                self.nbody_state.insert(
                    h,
                    NBodyState {
                        pos: Vector3::new(t.x as f64, t.y as f64, t.z as f64),
                        vel: Vector3::new(lv.x as f64, lv.y as f64, lv.z as f64),
                        mass: body.mass() as f64,
                    },
                );
            }
            let st = self.nbody_state[&h];
            let dragged = dragging == Some(h);
            let p = if dragged {
                self.bodies
                    .get(h)
                    .map(|b| {
                        let t = b.translation();
                        Vector3::new(t.x as f64, t.y as f64, t.z as f64)
                    })
                    .unwrap_or(st.pos)
            } else {
                st.pos
            };
            pos.push(p);
            vel.push(st.vel);
            mass.push(st.mass);
            integrated.push(!dragged);
        }

        let accel = |positions: &[Vector3<f64>]| -> Vec<Vector3<f64>> {
            let pm: Vec<(Vector3<f64>, f64)> = positions
                .iter()
                .copied()
                .zip(mass.iter().copied())
                .collect();
            let forces = match algo {
                NBodyAlgo::BruteForce => nbody::brute_force(&pm, G, SOFTENING2),
                NBodyAlgo::BarnesHut => nbody::barnes_hut(&pm, G, SOFTENING2, THETA),
            };
            forces.iter().zip(&mass).map(|(f, &m)| f / m).collect()
        };

        let half = dt * 0.5;
        let a0 = accel(&pos);
        for i in 0..pos.len() {
            if integrated[i] {
                vel[i] += a0[i] * half;
                pos[i] += vel[i] * dt;
            }
        }
        let a1 = accel(&pos);
        for i in 0..pos.len() {
            if integrated[i] {
                vel[i] += a1[i] * half;
            }
        }

        for (i, &h) in handles.iter().enumerate() {
            if !integrated[i] {
                continue;
            }
            if let Some(st) = self.nbody_state.get_mut(&h) {
                st.pos = pos[i];
                st.vel = vel[i];
            }
            if let Some(body) = self.bodies.get_mut(h) {
                body.set_next_kinematic_translation(Vector::new(
                    pos[i].x as f32,
                    pos[i].y as f32,
                    pos[i].z as f32,
                ));
            }
        }
    }

    fn record_trails(&mut self) {
        if !self.show_trails {
            return;
        }
        self.trail_tick = self.trail_tick.wrapping_add(1);
        if !self.trail_tick.is_multiple_of(TRAIL_EVERY) {
            return;
        }
        let samples: Vec<(RigidBodyHandle, Vector3<f64>)> = self
            .order
            .iter()
            .filter_map(|&(h, _)| {
                let body = self.bodies.get(h)?;
                let t = body.translation();
                Some((h, Vector3::new(t.x as f64, t.y as f64, t.z as f64)))
            })
            .collect();
        for (handle, pos) in samples {
            let trail = self.trails.entry(handle).or_default();
            trail.push_back(pos);
            if trail.len() > TRAIL_MAX {
                trail.pop_front();
            }
        }
    }

    fn cull_fallen(&mut self) {
        let fallen: Vec<RigidBodyHandle> = self
            .order
            .iter()
            .filter_map(|&(handle, _)| {
                let body = self.bodies.get(handle)?;
                (body.translation().y < VOID_FALL_LIMIT).then_some(handle)
            })
            .collect();
        for handle in fallen {
            self.remove(handle);
        }
    }

    pub fn spawn(&mut self, kind: SpawnKind, pos: Vector3<f64>, mass: f64) -> RigidBodyHandle {
        self.spawn_with_velocity(kind, pos, Vector3::zeros(), mass)
    }

    pub fn spawn_with_velocity(
        &mut self,
        kind: SpawnKind,
        pos: Vector3<f64>,
        linvel: Vector3<f64>,
        mass: f64,
    ) -> RigidBodyHandle {
        let handle = self.bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(pos.x as f32, pos.y as f32, pos.z as f32))
                .linvel(Vector::new(
                    linvel.x as f32,
                    linvel.y as f32,
                    linvel.z as f32,
                ))
                .ccd_enabled(true)
                .linear_damping(self.damping as f32),
        );

        let (collider, body_kind) = match kind {
            SpawnKind::Box => (
                ColliderBuilder::cuboid(BOX_HALF_EXTENT, BOX_HALF_EXTENT, BOX_HALF_EXTENT),
                BodyKind::Box {
                    half_extents: Vector3::new(
                        BOX_HALF_EXTENT as f64,
                        BOX_HALF_EXTENT as f64,
                        BOX_HALF_EXTENT as f64,
                    ),
                },
            ),
            SpawnKind::Sphere => (
                ColliderBuilder::ball(SPHERE_RADIUS),
                BodyKind::Sphere {
                    radius: SPHERE_RADIUS as f64,
                },
            ),
            SpawnKind::Star => (
                ColliderBuilder::ball(STAR_RADIUS),
                BodyKind::Star {
                    radius: STAR_RADIUS as f64,
                },
            ),
        };
        let collider = collider
            .mass(mass as f32)
            .friction(self.friction as f32)
            .restitution(self.restitution as f32);
        self.colliders
            .insert_with_parent(collider, handle, &mut self.bodies);
        self.order.push((handle, body_kind));
        if self.gravity_mode == GravityMode::NBody {
            if let Some(body) = self.bodies.get_mut(handle) {
                body.set_body_type(RigidBodyType::KinematicPositionBased, true);
            }
            self.nbody_state.insert(
                handle,
                NBodyState {
                    pos,
                    vel: linvel,
                    mass,
                },
            );
        }
        handle
    }

    pub fn spawn_star(
        &mut self,
        pos: Vector3<f64>,
        linvel: Vector3<f64>,
        mass: f64,
    ) -> RigidBodyHandle {
        self.spawn_with_velocity(SpawnKind::Star, pos, linvel, mass)
    }

    pub fn body_translation(&self, handle: RigidBodyHandle) -> Option<Vector3<f64>> {
        let t = self.bodies.get(handle)?.translation();
        Some(Vector3::new(t.x as f64, t.y as f64, t.z as f64))
    }

    pub fn start_drag(&mut self, handle: RigidBodyHandle) {
        self.dragging = Some(handle);
        if let Some(body) = self.bodies.get_mut(handle) {
            body.set_body_type(RigidBodyType::KinematicPositionBased, true);
        }
    }

    pub fn drag_to(&mut self, handle: RigidBodyHandle, mut pos: Vector3<f64>) {
        if self.floor_mode != FloorMode::Void {
            pos.y = pos.y.max(self.vertical_half(handle));
        }
        if let Some(st) = self.nbody_state.get_mut(&handle) {
            st.pos = pos;
        }
        if let Some(body) = self.bodies.get_mut(handle) {
            body.set_next_kinematic_translation(Vector::new(
                pos.x as f32,
                pos.y as f32,
                pos.z as f32,
            ));
        }
    }

    fn vertical_half(&self, handle: RigidBodyHandle) -> f64 {
        self.order
            .iter()
            .find(|(h, _)| *h == handle)
            .map_or(0.0, |(_, kind)| match kind {
                BodyKind::Box { half_extents } => half_extents.y,
                BodyKind::Sphere { radius } | BodyKind::Star { radius } => *radius,
            })
    }

    pub fn end_drag(&mut self, handle: RigidBodyHandle, linvel: Vector3<f64>) {
        self.dragging = None;
        self.hand_back(handle, linvel);
    }

    pub fn release_drag(&mut self) {
        if let Some(handle) = self.dragging.take() {
            self.hand_back(handle, Vector3::zeros());
        }
    }

    pub fn dragging_handle(&self) -> Option<RigidBodyHandle> {
        self.dragging
    }

    pub fn contains(&self, handle: RigidBodyHandle) -> bool {
        self.bodies.get(handle).is_some()
    }

    fn hand_back(&mut self, handle: RigidBodyHandle, linvel: Vector3<f64>) {
        if self.gravity_mode == GravityMode::NBody {
            if let Some(st) = self.nbody_state.get_mut(&handle) {
                st.vel = linvel;
            }
            return;
        }
        self.nbody_state.remove(&handle);
        if let Some(body) = self.bodies.get_mut(handle) {
            body.set_body_type(RigidBodyType::Dynamic, true);
            body.set_linvel(
                Vector::new(linvel.x as f32, linvel.y as f32, linvel.z as f32),
                true,
            );
            body.set_angvel(Vector::ZERO, true);
        }
    }

    pub fn remove(&mut self, handle: RigidBodyHandle) {
        self.trails.remove(&handle);
        self.nbody_state.remove(&handle);
        if self.dragging == Some(handle) {
            self.dragging = None;
        }
        self.bodies.remove(
            handle,
            &mut self.islands,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            true,
        );
        self.order.retain(|(h, _)| *h != handle);
    }

    pub fn clear(&mut self) {
        for handle in self.handles().collect::<Vec<_>>() {
            self.remove(handle);
        }
        self.cloths.clear();
        self.fluid.clear();
        self.nbody_state.clear();
        self.dragging = None;
    }

    pub fn len(&self) -> usize {
        self.order.len()
    }

    pub fn handles(&self) -> impl Iterator<Item = RigidBodyHandle> + '_ {
        self.order.iter().map(|(h, _)| *h)
    }

    pub fn render_bodies(&self) -> Vec<BodyView> {
        self.order
            .iter()
            .filter_map(|&(handle, kind)| {
                let body = self.bodies.get(handle)?;
                let pos = body.translation();
                let rot = *body.rotation();
                let bx = rot * Vector::X;
                let by = rot * Vector::Y;
                let bz = rot * Vector::Z;
                let (linvel, mass) = match self.nbody_state.get(&handle) {
                    Some(st) => (st.vel, st.mass),
                    None => {
                        let lv = body.linvel();
                        (
                            Vector3::new(lv.x as f64, lv.y as f64, lv.z as f64),
                            body.mass() as f64,
                        )
                    }
                };
                Some(BodyView {
                    handle,
                    kind,
                    position: Vector3::new(pos.x as f64, pos.y as f64, pos.z as f64),
                    basis: [
                        Vector3::new(bx.x as f64, bx.y as f64, bx.z as f64),
                        Vector3::new(by.x as f64, by.y as f64, by.z as f64),
                        Vector3::new(bz.x as f64, bz.y as f64, bz.z as f64),
                    ],
                    linvel,
                    speed: linvel.norm(),
                    mass,
                    trail: self
                        .trails
                        .get(&handle)
                        .map(|t| t.iter().copied().collect())
                        .unwrap_or_default(),
                })
            })
            .collect()
    }
}

#[cfg(test)]
mod fluid_integration {
    use super::*;

    #[test]
    fn dropped_box_splashes_and_settles_on_floor() {
        let mut w = PhysicsWorld::new();
        w.spawn_fluid_block(Vector3::new(0.0, 2.0, 0.0), Vector3::new(1.6, 1.2, 1.6));
        let boxh = w.spawn(SpawnKind::Box, Vector3::new(0.0, 4.0, 0.0), 30.0);

        for _ in 0..720 {
            w.step(1.0 / 60.0);
        }

        let by = w.body_translation(boxh).unwrap().y;
        let view = w.fluid_view().unwrap();
        let miny = view
            .positions
            .iter()
            .map(|p| p.y)
            .fold(f64::INFINITY, f64::min);

        assert!(by.is_finite() && by.abs() < 10.0, "box diverged: y={by}");
        assert!(
            (0.3..3.0).contains(&by),
            "box ended up somewhere unphysical: y={by}"
        );
        assert!(
            view.positions
                .iter()
                .all(|p| p.x.is_finite() && p.y.is_finite() && p.z.is_finite()),
            "fluid diverged",
        );
        assert!(miny > -0.2, "fluid sank through the floor: {miny}");
    }

    #[test]
    fn infinite_source_pours_and_spreads_on_floor() {
        let mut w = PhysicsWorld::new();
        let nozzle = Vector3::new(0.0, 3.0, 0.0);
        w.toggle_fluid_emitter(nozzle);
        assert_eq!(w.fluid_emitter_count(), 1, "source not placed");

        for _ in 0..900 {
            w.step(1.0 / 60.0);
        }

        let view = w.fluid_view().unwrap();
        assert!(
            view.positions.len() > 300,
            "source barely poured: {}",
            view.positions.len()
        );
        let miny = view
            .positions
            .iter()
            .map(|p| p.y)
            .fold(f64::INFINITY, f64::min);
        assert!(
            view.positions
                .iter()
                .all(|p| p.x.is_finite() && p.y.is_finite() && p.z.is_finite()),
            "poured fluid diverged",
        );
        assert!(miny > -0.2, "poured fluid sank through the floor: {miny}");

        w.toggle_fluid_emitter(nozzle);
        assert_eq!(
            w.fluid_emitter_count(),
            0,
            "source not removed by re-toggle"
        );
    }

    #[test]
    fn fluid_and_cloth_together_stay_stable() {
        let mut w = PhysicsWorld::new();
        w.spawn_cloth(Vector3::new(0.0, 1.2, 0.0));
        w.spawn_fluid_block(Vector3::new(0.0, 2.5, 0.0), Vector3::new(1.0, 0.6, 1.0));

        for _ in 0..600 {
            w.step(1.0 / 60.0);
        }

        let view = w.fluid_view().unwrap();
        assert!(
            view.positions
                .iter()
                .all(|p| p.x.is_finite() && p.y.is_finite() && p.z.is_finite()),
            "fluid diverged",
        );
        assert!(
            w.cloth_views().iter().all(|c| c
                .pos
                .iter()
                .all(|p| p.x.is_finite() && p.y.is_finite() && p.z.is_finite())),
            "cloth diverged",
        );
        assert!(
            view.positions.iter().all(|p| p.y > -0.5 && p.y < 20.0),
            "fluid ended up somewhere unphysical",
        );
    }

    #[test]
    fn water_poured_on_a_body_does_not_launch_it() {
        let mut w = PhysicsWorld::new();
        let boxh = w.spawn(
            SpawnKind::Box,
            Vector3::new(0.0, BOX_HALF_EXTENT as f64, 0.0),
            50.0,
        );
        let start_y = w.body_translation(boxh).unwrap().y;
        w.toggle_fluid_emitter(Vector3::new(0.0, 3.0, 0.0));

        let mut max_y = start_y;
        for _ in 0..900 {
            w.step(1.0 / 60.0);
            let y = w.body_translation(boxh).unwrap().y;
            max_y = max_y.max(y);
        }

        let end_y = w.body_translation(boxh).unwrap().y;
        assert!(end_y.is_finite(), "box diverged: y={end_y}");
        assert!(
            max_y < start_y + 0.3,
            "box was launched by water poured on top: peaked at y={max_y} (started {start_y})",
        );
    }

    #[test]
    fn dropped_box_does_not_bounce_on_thin_film() {
        let mut w = PhysicsWorld::new();
        w.spawn_fluid_block(Vector3::new(0.0, 2.0, 0.0), Vector3::new(1.6, 1.2, 1.6));
        let boxh = w.spawn(SpawnKind::Box, Vector3::new(0.0, 4.0, 0.0), 30.0);

        for _ in 0..300 {
            w.step(1.0 / 60.0);
        }
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for _ in 0..600 {
            w.step(1.0 / 60.0);
            let y = w.body_translation(boxh).unwrap().y;
            lo = lo.min(y);
            hi = hi.max(y);
        }
        assert!(hi.is_finite(), "box diverged: y in [{lo}, {hi}]");
        assert!(
            hi - lo < 0.05,
            "box is still bouncing on the film: y swung through [{lo:.3}, {hi:.3}]",
        );
    }
}

#[cfg(test)]
mod nbody_integration {
    use super::*;

    #[test]
    fn figure_eight_conserves_energy_over_long_run() {
        const SCALE: f64 = 4.0;
        const MASS: f64 = 8.0;
        let v_scale = (MASS / SCALE).sqrt();
        let mut w = PhysicsWorld::new();
        w.set_gravity_mode(GravityMode::NBody);
        w.set_nbody_algo(NBodyAlgo::BruteForce);
        w.set_floor_mode(FloorMode::Void);
        let bodies = [
            ((-0.970_004_36, 0.243_087_53), (0.466_203_685, 0.432_365_73)),
            ((0.970_004_36, -0.243_087_53), (0.466_203_685, 0.432_365_73)),
            ((0.0, 0.0), (-0.932_407_37, -0.864_731_46)),
        ];
        for ((px, pz), (vx, vz)) in bodies {
            w.spawn_star(
                Vector3::new(px * SCALE, 0.0, pz * SCALE),
                Vector3::new(vx * v_scale, 0.0, vz * v_scale),
                MASS,
            );
        }

        let energy = |w: &PhysicsWorld| -> f64 {
            let bs = w.render_bodies();
            let ke: f64 = bs
                .iter()
                .map(|b| 0.5 * b.mass * b.linvel.norm_squared())
                .sum();
            let mut pe = 0.0;
            for i in 0..bs.len() {
                for j in (i + 1)..bs.len() {
                    let r2 = (bs[i].position - bs[j].position).norm_squared();
                    pe -= G * bs[i].mass * bs[j].mass / (r2 + SOFTENING2).sqrt();
                }
            }
            ke + pe
        };

        let e0 = energy(&w);
        for _ in 0..3000 {
            w.step(1.0 / 60.0);
        }
        let e1 = energy(&w);

        let bs = w.render_bodies();
        assert!(
            bs.iter().all(|b| b.position.iter().all(|x| x.is_finite())),
            "figure-eight diverged",
        );
        assert!(
            bs.iter().all(|b| b.position.norm() < 20.0),
            "a body escaped the choreography"
        );
        assert!(
            ((e1 - e0) / e0).abs() < 0.02,
            "energy drifted: {e0} -> {e1}"
        );
    }
}
