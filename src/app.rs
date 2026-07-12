use std::collections::VecDeque;
use std::io;
use std::time::{Duration, Instant};

use nalgebra::Vector3;
use rapier3d::prelude::RigidBodyHandle;
use ratatui::DefaultTerminal;
use ratatui::crossterm::event::{self, Event};

use crate::input;
use crate::physics::{FloorMode, GravityMode, NBodyAlgo, PhysicsWorld, SpawnKind};
use crate::render::{self, ColorMode, Layout3D, camera::Camera, projection::Projection};

const PHYSICS_DT: f64 = 1.0 / 60.0;
const RENDER_INTERVAL: Duration = Duration::from_millis(33);
const MAX_CATCHUP_STEPS: u32 = 5;
const PHYSICS_BUDGET: Duration = Duration::from_millis(20);
pub const SPAWN_DROP_HEIGHT: f64 = 2.0;
pub const DEFAULT_SPAWN_MASS: f64 = 50.0;
pub const MIN_SPAWN_MASS: f64 = 1.0;
pub const MAX_SPAWN_MASS: f64 = 100_000.0;
pub const DEFAULT_SPAWN_SCALE: f64 = 1.0;
pub const MIN_SPAWN_SCALE: f64 = 0.4;
pub const MAX_SPAWN_SCALE: f64 = 3.0;
pub const MAX_SANDBOXES: usize = 9;

pub const DEFAULT_LAUNCH_SPEED: f64 = 20.0;
pub const MIN_LAUNCH_SPEED: f64 = 2.0;
pub const MAX_LAUNCH_SPEED: f64 = 80.0;
pub const MIN_TIME_SCALE: f64 = 0.1;
pub const MAX_TIME_SCALE: f64 = 4.0;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum Tunable {
    Gravity,
    Friction,
    Restitution,
    Damping,
    SpawnSize,
    LaunchSpeed,
    TimeScale,
    ClothStiffness,
    WindStrength,
    Viscosity,
    Ambient,
    Conductivity,
    Cooling,
}

impl Tunable {
    pub const ALL: [Tunable; 13] = [
        Tunable::Gravity,
        Tunable::Friction,
        Tunable::Restitution,
        Tunable::Damping,
        Tunable::SpawnSize,
        Tunable::LaunchSpeed,
        Tunable::TimeScale,
        Tunable::ClothStiffness,
        Tunable::WindStrength,
        Tunable::Viscosity,
        Tunable::Ambient,
        Tunable::Conductivity,
        Tunable::Cooling,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Tunable::Gravity => "Gravity",
            Tunable::Friction => "Friction",
            Tunable::Restitution => "Bounce",
            Tunable::Damping => "Drag",
            Tunable::SpawnSize => "Size",
            Tunable::LaunchSpeed => "Launch",
            Tunable::TimeScale => "Time",
            Tunable::ClothStiffness => "Stiffness",
            Tunable::WindStrength => "Wind",
            Tunable::Viscosity => "Viscosity",
            Tunable::Ambient => "Ambient",
            Tunable::Conductivity => "Conduct",
            Tunable::Cooling => "Cooling",
        }
    }

    fn next(self) -> Self {
        match self {
            Tunable::Gravity => Tunable::Friction,
            Tunable::Friction => Tunable::Restitution,
            Tunable::Restitution => Tunable::Damping,
            Tunable::Damping => Tunable::SpawnSize,
            Tunable::SpawnSize => Tunable::LaunchSpeed,
            Tunable::LaunchSpeed => Tunable::TimeScale,
            Tunable::TimeScale => Tunable::ClothStiffness,
            Tunable::ClothStiffness => Tunable::WindStrength,
            Tunable::WindStrength => Tunable::Viscosity,
            Tunable::Viscosity => Tunable::Ambient,
            Tunable::Ambient => Tunable::Conductivity,
            Tunable::Conductivity => Tunable::Cooling,
            Tunable::Cooling => Tunable::Gravity,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Tool {
    Place,
    Grab,
    Fluid,
    Heat,
    Build,
}

impl Tool {
    pub fn next(self) -> Self {
        match self {
            Tool::Place => Tool::Grab,
            Tool::Grab => Tool::Fluid,
            Tool::Fluid => Tool::Heat,
            Tool::Heat => Tool::Build,
            Tool::Build => Tool::Place,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum StructureKind {
    Platform,
    Ramp,
    Wall,
}

impl StructureKind {
    pub fn next(self) -> Self {
        match self {
            StructureKind::Platform => StructureKind::Ramp,
            StructureKind::Ramp => StructureKind::Wall,
            StructureKind::Wall => StructureKind::Platform,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            StructureKind::Platform => "platform",
            StructureKind::Ramp => "ramp",
            StructureKind::Wall => "wall",
        }
    }
}

pub struct GrabState {
    pub handle: RigidBodyHandle,
    pub plane_point: Vector3<f64>,
    pub plane_normal: Vector3<f64>,
    pub offset: Vector3<f64>,
    pub samples: VecDeque<(Instant, Vector3<f64>)>,
}

const MAX_THROW_SPEED: f64 = 25.0;
const THROW_SCALE: f64 = 0.35;
const THROW_WINDOW: Duration = Duration::from_millis(90);

impl GrabState {
    pub fn record(&mut self, hit: Vector3<f64>) {
        let now = Instant::now();
        self.samples.push_back((now, hit));
        while self.samples.len() > 2 && now - self.samples[0].0 > THROW_WINDOW {
            self.samples.pop_front();
        }
    }

    fn throw_velocity(&self) -> Vector3<f64> {
        let now = Instant::now();
        let mut recent = self
            .samples
            .iter()
            .filter(|(t, _)| now - *t <= THROW_WINDOW);
        let Some(&(t0, p0)) = recent.next() else {
            return Vector3::zeros();
        };
        let Some((t1, p1)) = recent.next_back().map(|&(t, p)| (t, p)) else {
            return Vector3::zeros();
        };
        let dt = (t1 - t0).as_secs_f64();
        if dt < 1e-3 {
            return Vector3::zeros();
        }
        let v = (p1 - p0) / dt * THROW_SCALE;
        let speed = v.norm();
        if speed > MAX_THROW_SPEED {
            v * (MAX_THROW_SPEED / speed)
        } else {
            v
        }
    }
}

pub struct ClothGrab {
    pub plane_point: Vector3<f64>,
    pub plane_normal: Vector3<f64>,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Scene {
    Blank,
    RigidStack,
    ThreeBody,
    Cloth,
    Thermal,
}

struct SandboxState {
    physics: PhysicsWorld,
    spawn_kind: SpawnKind,
    spawn_mass: f64,
    color_mode: ColorMode,
    selected: Option<RigidBodyHandle>,
}

impl SandboxState {
    fn from_scene(scene: Scene) -> Self {
        match scene {
            Scene::Blank => Self::blank(),
            Scene::RigidStack => Self::rigid_stack(),
            Scene::ThreeBody => Self::three_body(),
            Scene::Cloth => Self::cloth_demo(),
            Scene::Thermal => Self::thermal_demo(),
        }
    }

    fn thermal_demo() -> Self {
        let mut physics = PhysicsWorld::new();
        physics.set_ambient(20.0);
        physics.set_thermo(true);
        physics.spawn_container(Vector3::new(0.0, 0.0, 0.0), 2.4, 3.0);
        physics.spawn_fluid_block(Vector3::new(0.0, 1.1, 0.0), Vector3::new(2.25, 1.0, 2.25));
        physics.set_fluid_floor_temp(Some(240.0));
        Self {
            physics,
            spawn_kind: SpawnKind::Sphere,
            spawn_mass: 30.0,
            color_mode: ColorMode::Temperature,
            selected: None,
        }
    }

    fn cloth_demo() -> Self {
        let mut physics = PhysicsWorld::new();
        physics.set_floor_mode(FloorMode::Void);
        physics.spawn_cloth_hammock(Vector3::new(0.0, 3.2, 0.0), 0.6);
        physics.set_cloth_stiffness(0.5);
        let ball = physics.spawn(SpawnKind::Sphere, Vector3::new(-0.8, 4.6, 0.1), 5.0);
        physics.spawn(SpawnKind::Sphere, Vector3::new(0.9, 8.5, -0.2), 3.0);
        Self {
            physics,
            spawn_kind: SpawnKind::Sphere,
            spawn_mass: DEFAULT_SPAWN_MASS,
            color_mode: ColorMode::Velocity,
            selected: Some(ball),
        }
    }

    fn blank() -> Self {
        Self {
            physics: PhysicsWorld::new(),
            spawn_kind: SpawnKind::Sphere,
            spawn_mass: DEFAULT_SPAWN_MASS,
            color_mode: ColorMode::Velocity,
            selected: None,
        }
    }

    fn rigid_stack() -> Self {
        let mut physics = PhysicsWorld::new();
        let box_mass = 3.0;
        let rows: [(f64, &[f64]); 4] = [
            (0.5, &[-1.5, -0.5, 0.5, 1.5]),
            (1.5, &[-1.0, 0.0, 1.0]),
            (2.5, &[-0.5, 0.5]),
            (3.5, &[0.0]),
        ];
        for (y, xs) in rows {
            for &x in xs {
                physics.spawn(SpawnKind::Box, Vector3::new(x, y, 0.0), box_mass);
            }
        }
        let ball = physics.spawn_with_velocity(
            SpawnKind::Sphere,
            Vector3::new(-8.0, 1.0, 0.0),
            Vector3::new(13.0, 3.0, 0.0),
            80.0,
        );
        Self {
            physics,
            spawn_kind: SpawnKind::Box,
            spawn_mass: DEFAULT_SPAWN_MASS,
            color_mode: ColorMode::Velocity,
            selected: Some(ball),
        }
    }

    fn three_body() -> Self {
        const SCALE: f64 = 4.0;
        const MASS: f64 = 8.0;
        let v_scale = (MASS / SCALE).sqrt();

        let mut physics = PhysicsWorld::new();
        physics.set_gravity_mode(GravityMode::NBody);
        physics.set_nbody_algo(NBodyAlgo::BruteForce);
        physics.set_floor_mode(FloorMode::Void);
        physics.set_show_trails(true);

        let bodies = [
            ((-0.970_004_36, 0.243_087_53), (0.466_203_685, 0.432_365_73)),
            ((0.970_004_36, -0.243_087_53), (0.466_203_685, 0.432_365_73)),
            ((0.0, 0.0), (-0.932_407_37, -0.864_731_46)),
        ];
        let mut first = None;
        for ((px, pz), (vx, vz)) in bodies {
            let handle = physics.spawn_with_velocity(
                SpawnKind::Star,
                Vector3::new(px * SCALE, 0.0, pz * SCALE),
                Vector3::new(vx * v_scale, 0.0, vz * v_scale),
                MASS,
            );
            first.get_or_insert(handle);
        }
        Self {
            physics,
            spawn_kind: SpawnKind::Star,
            spawn_mass: DEFAULT_SPAWN_MASS,
            color_mode: ColorMode::Temperature,
            selected: first,
        }
    }
}

pub struct App {
    pub camera: Camera,
    pub projection: Projection,
    pub physics: PhysicsWorld,
    pub spawn_kind: SpawnKind,
    pub spawn_mass: f64,
    pub spawn_scale: f64,
    pub build_kind: StructureKind,
    pub color_mode: ColorMode,
    pub selected: Option<RigidBodyHandle>,
    saved: Vec<Option<SandboxState>>,
    active: usize,
    pub paused: bool,
    step_once: bool,
    pub time_scale: f64,
    pub launch_speed: f64,
    pub tunable: Tunable,
    pub tool: Tool,
    pub grab: Option<GrabState>,
    pub cloth_grab: Option<ClothGrab>,
    pub layout: Layout3D,
    pub show_help: bool,
    pub help_page: usize,
    pub running: bool,
}

fn scene_camera(scene: Scene) -> Camera {
    let mut camera = Camera::new();
    match scene {
        Scene::Cloth => {
            camera.target = Vector3::new(0.0, 2.4, 0.0);
            camera.distance = 8.5;
        }
        Scene::Thermal => {
            camera.target = Vector3::new(0.0, 1.0, 0.0);
            camera.distance = 12.0;
        }
        _ => {}
    }
    camera
}

impl App {
    pub fn new(scene: Scene) -> Self {
        let first = SandboxState::from_scene(scene);
        Self {
            camera: scene_camera(scene),
            projection: Projection::new(60.0),
            physics: first.physics,
            spawn_kind: first.spawn_kind,
            spawn_mass: first.spawn_mass,
            spawn_scale: DEFAULT_SPAWN_SCALE,
            build_kind: StructureKind::Platform,
            color_mode: first.color_mode,
            selected: first.selected,
            saved: vec![None],
            active: 0,
            paused: false,
            step_once: false,
            time_scale: 1.0,
            launch_speed: DEFAULT_LAUNCH_SPEED,
            tunable: Tunable::Gravity,
            tool: Tool::Place,
            grab: None,
            cloth_grab: None,
            layout: Layout3D::default(),
            show_help: false,
            help_page: 0,
            running: true,
        }
    }

    pub fn toggle_pause(&mut self) {
        self.paused = !self.paused;
    }

    pub const HELP_PAGES: usize = 2;

    pub fn toggle_help(&mut self) {
        self.show_help = !self.show_help;
        if self.show_help {
            self.help_page = 0;
        }
    }

    pub fn flip_help_page(&mut self) {
        self.help_page = (self.help_page + 1) % Self::HELP_PAGES;
    }

    pub fn cycle_tool(&mut self) {
        self.tool = self.tool.next();
    }

    pub fn request_step(&mut self) {
        self.step_once = true;
    }

    pub fn tunable_applies(&self, t: Tunable) -> bool {
        match t {
            Tunable::Gravity => self.physics.gravity_mode() != GravityMode::NBody,
            Tunable::ClothStiffness => self.physics.has_cloth(),
            Tunable::WindStrength => self.physics.has_cloth(),
            Tunable::Viscosity => self.physics.has_fluid(),
            Tunable::Ambient => self.physics.thermo_enabled(),
            Tunable::Conductivity => self.physics.thermo_enabled(),
            Tunable::Cooling => self.physics.thermo_enabled(),
            _ => true,
        }
    }

    pub fn cycle_tunable(&mut self) {
        let mut next = self.tunable.next();
        while !self.tunable_applies(next) {
            next = next.next();
        }
        self.tunable = next;
    }

    pub fn ensure_tunable_applicable(&mut self) {
        if !self.tunable_applies(self.tunable) {
            self.cycle_tunable();
        }
    }

    pub fn adjust_tunable(&mut self, increase: bool) {
        if !self.tunable_applies(self.tunable) {
            return;
        }
        let sign = if increase { 1.0 } else { -1.0 };
        match self.tunable {
            Tunable::Gravity => self
                .physics
                .set_gravity(self.physics.gravity() + sign * 0.5),
            Tunable::Friction => self
                .physics
                .set_friction(self.physics.friction() + sign * 0.05),
            Tunable::Restitution => self
                .physics
                .set_restitution(self.physics.restitution() + sign * 0.05),
            Tunable::Damping => self
                .physics
                .set_damping(self.physics.damping() + sign * 0.05),
            Tunable::LaunchSpeed => {
                self.launch_speed =
                    (self.launch_speed + sign * 2.5).clamp(MIN_LAUNCH_SPEED, MAX_LAUNCH_SPEED);
            }
            Tunable::SpawnSize => {
                let factor = if increase { 1.15 } else { 1.0 / 1.15 };
                self.spawn_scale =
                    (self.spawn_scale * factor).clamp(MIN_SPAWN_SCALE, MAX_SPAWN_SCALE);
            }
            Tunable::TimeScale => {
                let factor = if increase { 1.25 } else { 1.0 / 1.25 };
                self.time_scale = (self.time_scale * factor).clamp(MIN_TIME_SCALE, MAX_TIME_SCALE);
            }
            Tunable::ClothStiffness => {
                if let Some(s) = self.physics.cloth_stiffness() {
                    self.physics.set_cloth_stiffness(s + sign * 0.05);
                }
            }
            Tunable::WindStrength => {
                self.physics
                    .set_wind_strength(self.physics.wind_strength() + sign * 0.25);
            }
            Tunable::Viscosity => {
                self.physics
                    .set_fluid_viscosity(self.physics.fluid_viscosity() + sign * 1.0);
            }
            Tunable::Ambient => self
                .physics
                .set_ambient(self.physics.ambient() + sign * 5.0),
            Tunable::Conductivity => self
                .physics
                .set_conductivity(self.physics.conductivity() + sign * 0.25),
            Tunable::Cooling => self
                .physics
                .set_cooling(self.physics.cooling() + sign * 0.1),
        }
    }

    pub fn mode_label(&self) -> &'static str {
        if self.physics.has_fluid() {
            return "Fluid";
        }
        match self.physics.gravity_mode() {
            GravityMode::Uniform => "Rigid Body",
            GravityMode::NBody => "Space",
        }
    }

    pub fn tunable_value(&self, t: Tunable) -> String {
        match t {
            Tunable::Gravity => format!("{:.1} m/s2", self.physics.gravity()),
            Tunable::Friction => format!("{:.2}", self.physics.friction()),
            Tunable::Restitution => format!("{:.2}", self.physics.restitution()),
            Tunable::Damping => format!("{:.2}", self.physics.damping()),
            Tunable::SpawnSize => format!("{:.2}x", self.spawn_scale),
            Tunable::LaunchSpeed => format!("{:.1}", self.launch_speed),
            Tunable::TimeScale => format!("{:.2}x", self.time_scale),
            Tunable::ClothStiffness => self
                .physics
                .cloth_stiffness()
                .map_or_else(|| "-".to_string(), |s| format!("{s:.2}")),
            Tunable::WindStrength => format!("{:.2}x", self.physics.wind_strength()),
            Tunable::Viscosity => format!("{:.1}", self.physics.fluid_viscosity()),
            Tunable::Ambient => format!("{:.0} C", self.physics.ambient()),
            Tunable::Conductivity => format!("{:.2}", self.physics.conductivity()),
            Tunable::Cooling => format!("{:.2}x", self.physics.cooling()),
        }
    }

    pub fn clear(&mut self) {
        self.release_grab();
        self.physics.clear();
        self.selected = None;
    }

    pub fn begin_body_grab(
        &mut self,
        handle: RigidBodyHandle,
        plane_point: Vector3<f64>,
        plane_normal: Vector3<f64>,
        hit: Vector3<f64>,
    ) {
        self.physics.start_drag(handle);
        self.selected = Some(handle);
        let mut samples = VecDeque::new();
        samples.push_back((Instant::now(), hit));
        self.grab = Some(GrabState {
            handle,
            plane_point,
            plane_normal,
            offset: plane_point - hit,
            samples,
        });
    }

    pub fn begin_cloth_grab(&mut self, plane_point: Vector3<f64>, plane_normal: Vector3<f64>) {
        self.cloth_grab = Some(ClothGrab {
            plane_point,
            plane_normal,
        });
    }

    pub fn release_grab(&mut self) {
        if let Some(grab) = self.grab.take() {
            self.physics.end_drag(grab.handle, grab.throw_velocity());
        } else {
            self.physics.release_drag();
        }
        if self.cloth_grab.take().is_some() || self.physics.any_cloth_grabbed() {
            self.physics.release_cloth();
        }
    }

    fn reconcile_grab(&mut self) {
        let ui = self.grab.as_ref().map(|g| g.handle);
        let stale = ui.is_some_and(|h| !self.physics.contains(h));
        if ui != self.physics.dragging_handle() || stale {
            self.release_grab();
        }
        if self.cloth_grab.is_none() && self.physics.any_cloth_grabbed() {
            self.physics.release_cloth();
        }
    }

    pub fn active_sandbox(&self) -> usize {
        self.active
    }

    pub fn sandbox_count(&self) -> usize {
        self.saved.len()
    }

    pub fn switch_sandbox(&mut self, target: usize) {
        if target == self.active {
            return;
        }
        self.release_grab();
        if target == self.saved.len() && self.saved.len() < MAX_SANDBOXES {
            self.saved.push(Some(SandboxState::blank()));
        }
        let Some(Some(_)) = self.saved.get(target) else {
            return;
        };
        let incoming = self.saved[target].take().unwrap();
        let parked = SandboxState {
            physics: std::mem::replace(&mut self.physics, incoming.physics),
            spawn_kind: self.spawn_kind,
            spawn_mass: self.spawn_mass,
            color_mode: self.color_mode,
            selected: self.selected,
        };
        self.spawn_kind = incoming.spawn_kind;
        self.spawn_mass = incoming.spawn_mass;
        self.color_mode = incoming.color_mode;
        self.selected = incoming.selected;
        self.saved[self.active] = Some(parked);
        self.active = target;
    }

    pub fn run(mut self, terminal: &mut DefaultTerminal) -> io::Result<()> {
        let physics_step = Duration::from_secs_f64(PHYSICS_DT);
        let mut accumulator = Duration::ZERO;
        let mut last = Instant::now();

        while self.running {
            let frame_start = Instant::now();
            let elapsed = frame_start - last;
            last = frame_start;

            if self.paused {
                accumulator = Duration::ZERO;
                if self.step_once {
                    self.physics.step(PHYSICS_DT);
                }
            } else {
                accumulator += elapsed.mul_f64(self.time_scale);
                accumulator = accumulator.min(physics_step * MAX_CATCHUP_STEPS);
                let sim_start = Instant::now();
                while accumulator >= physics_step {
                    self.physics.step(PHYSICS_DT);
                    accumulator -= physics_step;
                    if sim_start.elapsed() >= PHYSICS_BUDGET {
                        accumulator = Duration::ZERO;
                        break;
                    }
                }
            }
            self.step_once = false;

            terminal.draw(|frame| render::draw(frame, &mut self))?;

            let timeout = RENDER_INTERVAL.saturating_sub(frame_start.elapsed());
            if event::poll(timeout)? {
                loop {
                    match event::read()? {
                        Event::Key(key) => input::handle_key(&mut self, key),
                        Event::Mouse(mouse) => input::handle_mouse(&mut self, mouse),
                        _ => {}
                    }
                    if !event::poll(Duration::ZERO)? {
                        break;
                    }
                }
            }

            self.reconcile_grab();
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tunable_next_cycles_through_all_variants_exactly_once() {
        let mut seen = std::collections::HashSet::new();
        let mut t = Tunable::Gravity;
        for _ in 0..Tunable::ALL.len() {
            assert!(seen.insert(t), "next() repeated a variant early: {t:?}");
            t = t.next();
        }
        assert_eq!(
            t,
            Tunable::Gravity,
            "next() cycle didn't return to its start"
        );
        for &v in &Tunable::ALL {
            assert!(
                seen.contains(&v),
                "ALL contains a variant next() never visits: {v:?}"
            );
        }
    }
}
