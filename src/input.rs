use nalgebra::Vector3;
use rapier3d::prelude::RigidBodyHandle;

use ratatui::crossterm::event::{
    KeyCode, KeyEvent, KeyModifiers, MouseButton, MouseEvent, MouseEventKind,
};

use crate::app::{App, MAX_SPAWN_MASS, MIN_SPAWN_MASS, SPAWN_DROP_HEIGHT, StructureKind, Tool};
use crate::physics::{G, GravityMode, HEAT_TOOL_DELTA, SpawnKind};

const ORBIT_STEP: f64 = 0.08;
const PAN_STEP: f64 = 0.3;
const ZOOM_FACTOR: f64 = 1.1;
const SELECT_THRESHOLD: f64 = 0.08;
const MASS_FACTOR: f64 = 2.0;

pub fn handle_key(app: &mut App, key: KeyEvent) {
    let shift = key.modifiers.contains(KeyModifiers::SHIFT);

    if app.show_help {
        match key.code {
            KeyCode::Char('?') | KeyCode::Char('h') | KeyCode::Esc | KeyCode::Char('q') => {
                app.show_help = false;
            }
            KeyCode::Tab
            | KeyCode::Char(' ')
            | KeyCode::Left
            | KeyCode::Right
            | KeyCode::Char('n')
            | KeyCode::Char('p') => app.flip_help_page(),
            _ => {}
        }
        return;
    }

    match key.code {
        KeyCode::Char('?') | KeyCode::Char('h') => app.toggle_help(),
        KeyCode::Char('q') | KeyCode::Esc => app.running = false,
        KeyCode::Char('r') => app.camera.reset(),
        KeyCode::Char('+') | KeyCode::Char('=') => app.camera.zoom(1.0 / ZOOM_FACTOR),
        KeyCode::Char('-') => app.camera.zoom(ZOOM_FACTOR),
        KeyCode::Left if shift => app.camera.pan(-PAN_STEP, 0.0),
        KeyCode::Right if shift => app.camera.pan(PAN_STEP, 0.0),
        KeyCode::Up if shift => app.camera.pan(0.0, PAN_STEP),
        KeyCode::Down if shift => app.camera.pan(0.0, -PAN_STEP),
        KeyCode::Left => app.camera.orbit(-ORBIT_STEP, 0.0),
        KeyCode::Right => app.camera.orbit(ORBIT_STEP, 0.0),
        KeyCode::Up => app.camera.orbit(0.0, -ORBIT_STEP),
        KeyCode::Down => app.camera.orbit(0.0, ORBIT_STEP),
        KeyCode::Char('b') => select_spawn(app, SpawnKind::Box),
        KeyCode::Char('s') => select_spawn(app, SpawnKind::Sphere),
        KeyCode::Char('n') => select_spawn(app, SpawnKind::Star),
        KeyCode::Char('x') => {
            if let Some(handle) = app.selected.take() {
                app.physics.remove(handle);
            }
        }
        KeyCode::Char('X') => app.clear(),
        KeyCode::Char(' ') => app.toggle_pause(),
        KeyCode::Char('m') => app.request_step(),
        KeyCode::Char(';') => app.cycle_tunable(),
        KeyCode::Char(',') => app.adjust_tunable(false),
        KeyCode::Char('.') => app.adjust_tunable(true),
        KeyCode::Tab => cycle_selection(app, !shift),
        KeyCode::Char('f') => {
            let next = app.physics.floor_mode().next();
            app.physics.set_floor_mode(next);
        }
        KeyCode::Char('g') => toggle_gravity_mode(app),
        KeyCode::Char('a') => {
            let next = app.physics.nbody_algo().next();
            app.physics.set_nbody_algo(next);
        }
        KeyCode::Char('d') => app.cycle_tool(),
        KeyCode::Char('c') => app.color_mode = app.color_mode.next(),
        KeyCode::Char('t') => {
            let on = !app.physics.show_trails();
            app.physics.set_show_trails(on);
        }
        KeyCode::Char('k') => {
            let y = 3.5 + app.physics.cloth_count() as f64 * 1.0;
            app.physics.spawn_cloth(Vector3::new(0.0, y, 0.0));
        }
        KeyCode::Char('l') => {
            let y = 3.5 + app.physics.cloth_count() as f64 * 1.0;
            app.physics
                .spawn_cloth_hammock(Vector3::new(0.0, y, 0.0), 0.9);
        }
        KeyCode::Char('K') => {
            app.physics.remove_cloth();
            app.cloth_grab = None;
        }
        KeyCode::Char('j') => {
            app.physics
                .spawn_fluid_block(Vector3::new(0.0, 3.0, 0.0), Vector3::new(1.3, 0.7, 1.3));
        }
        KeyCode::Char('J') => app.physics.clear_fluid(),
        KeyCode::Char('o') => {
            app.physics.toggle_container(
                Vector3::new(0.0, 0.0, 0.0),
                TANK_HALF_EXTENT,
                TANK_WALL_HEIGHT,
            );
        }
        KeyCode::Char('w') => app.physics.toggle_wind(),
        KeyCode::Char('e') => {
            use crate::render::ColorMode;
            let on = !app.physics.thermo_enabled();
            app.physics.set_thermo(on);
            if on {
                app.color_mode = ColorMode::Temperature;
            }
            app.ensure_tunable_applicable();
        }
        KeyCode::Char(c @ '1'..='9') => {
            app.switch_sandbox(c as usize - '1' as usize);
        }
        KeyCode::Char('[') => {
            app.spawn_mass = (app.spawn_mass / MASS_FACTOR).clamp(MIN_SPAWN_MASS, MAX_SPAWN_MASS);
        }
        KeyCode::Char(']') => {
            app.spawn_mass = (app.spawn_mass * MASS_FACTOR).clamp(MIN_SPAWN_MASS, MAX_SPAWN_MASS);
        }
        _ => {}
    }
}

fn select_spawn(app: &mut App, kind: SpawnKind) {
    app.spawn_kind = kind;
    app.tool = Tool::Place;
}

fn toggle_gravity_mode(app: &mut App) {
    use crate::physics::FloorMode;
    use crate::render::ColorMode;
    match app.physics.gravity_mode() {
        GravityMode::Uniform => {
            app.physics.set_gravity_mode(GravityMode::NBody);
            app.physics.set_floor_mode(FloorMode::Void);
            app.physics.set_show_trails(true);
            app.spawn_kind = SpawnKind::Star;
            app.color_mode = ColorMode::Temperature;
            app.ensure_tunable_applicable();
        }
        GravityMode::NBody => {
            app.physics.set_gravity_mode(GravityMode::Uniform);
            app.physics.set_floor_mode(FloorMode::Infinite);
            app.physics.set_show_trails(false);
            app.spawn_kind = SpawnKind::Sphere;
            app.color_mode = ColorMode::Velocity;
        }
    }
}

pub fn handle_mouse(app: &mut App, mouse: MouseEvent) {
    let (col, row) = (mouse.column, mouse.row);
    match mouse.kind {
        MouseEventKind::Down(MouseButton::Left) => match app.tool {
            Tool::Place => spawn_at_cursor(app, col, row),
            Tool::Grab => begin_grab(app, col, row),
            Tool::Fluid => pour_fluid(app, col, row),
            Tool::Heat => heat_at_cursor(app, col, row, true),
            Tool::Build => build_at_cursor(app, col, row),
        },
        MouseEventKind::Drag(MouseButton::Left) => match app.tool {
            Tool::Fluid => pour_fluid(app, col, row),
            Tool::Heat => heat_at_cursor(app, col, row, true),
            Tool::Build => {}
            _ => update_grab(app, col, row),
        },
        MouseEventKind::Up(MouseButton::Left) => app.release_grab(),
        MouseEventKind::Moved => app.release_grab(),
        MouseEventKind::Down(MouseButton::Right) => {
            if app.tool == Tool::Fluid {
                toggle_fluid_source(app, col, row);
            } else if app.tool == Tool::Heat {
                heat_at_cursor(app, col, row, false);
            } else if app.tool == Tool::Build {
                app.build_kind = app.build_kind.next();
            } else if !try_pin_cloth(app, col, row) {
                select_at_cursor(app, col, row);
            }
        }
        MouseEventKind::Down(MouseButton::Middle) => launch_at_cursor(app, col, row),
        _ => {}
    }
}

const FLUID_BRUSH_DROP: f64 = 0.8;
const FLUID_SOURCE_HEIGHT: f64 = 3.0;

fn pour_fluid(app: &mut App, column: u16, row: u16) {
    if let Some(hit) = ground_hit(app, column, row) {
        app.physics
            .spawn_fluid_blob(hit + Vector3::new(0.0, FLUID_BRUSH_DROP, 0.0));
    }
}

const FLUID_HEAT_RADIUS: f64 = 1.2;

fn heat_at_cursor(app: &mut App, column: u16, row: u16, warm: bool) {
    let delta = if warm {
        HEAT_TOOL_DELTA
    } else {
        -HEAT_TOOL_DELTA
    };
    if let Some(handle) = body_at_cursor(app, column, row) {
        app.physics.add_heat(handle, delta);
        app.selected = Some(handle);
    } else if app.physics.has_fluid()
        && let Some(hit) = ground_hit(app, column, row)
    {
        app.physics.heat_fluid_near(hit, FLUID_HEAT_RADIUS, delta);
    }
}

const PLATFORM_HALF: Vector3<f64> = Vector3::new(1.1, 0.15, 1.1);
const RAMP_HALF: Vector3<f64> = Vector3::new(1.2, 0.12, 1.1);
const RAMP_TILT: f64 = 0.5;
const WALL_HALF: Vector3<f64> = Vector3::new(0.15, 1.0, 1.1);
const TANK_HALF_EXTENT: f64 = 2.2;
const TANK_WALL_HEIGHT: f64 = 2.5;

fn build_at_cursor(app: &mut App, column: u16, row: u16) {
    let Some(hit) = ground_hit(app, column, row) else {
        return;
    };
    let handle = match app.build_kind {
        StructureKind::Platform => {
            let center = hit + Vector3::new(0.0, PLATFORM_HALF.y, 0.0);
            app.physics.spawn_obstacle(center, PLATFORM_HALF)
        }
        StructureKind::Wall => {
            let center = hit + Vector3::new(0.0, WALL_HALF.y, 0.0);
            app.physics.spawn_obstacle(center, WALL_HALF)
        }
        StructureKind::Ramp => {
            // Lift the tilted slab so its lower corner rests near the ground.
            let lift = RAMP_HALF.x * RAMP_TILT.sin() + RAMP_HALF.y * RAMP_TILT.cos();
            let center = hit + Vector3::new(0.0, lift, 0.0);
            app.physics.spawn_ramp(center, RAMP_HALF, RAMP_TILT)
        }
    };
    app.selected = Some(handle);
}

fn toggle_fluid_source(app: &mut App, column: u16, row: u16) {
    if let Some(hit) = ground_hit(app, column, row) {
        app.physics
            .toggle_fluid_emitter(hit + Vector3::new(0.0, FLUID_SOURCE_HEIGHT, 0.0));
    }
}

fn begin_grab(app: &mut App, column: u16, row: u16) {
    if grab_cloth(app, column, row) {
        return;
    }
    let Some(handle) = body_at_cursor(app, column, row) else {
        return;
    };
    let Some(pos) = app.physics.body_translation(handle) else {
        return;
    };
    let (_, _, forward) = app.camera.basis();
    let Some((eye, dir)) = cursor_ray(app, column, row) else {
        return;
    };
    let hit = plane_hit(eye, dir, pos, forward).unwrap_or(pos);
    app.begin_body_grab(handle, pos, forward, hit);
}

fn grab_cloth(app: &mut App, column: u16, row: u16) -> bool {
    if !app.physics.has_cloth() {
        return false;
    }
    let Some(target) = screen_to_ndc(app, column, row) else {
        return false;
    };
    let (_, _, forward) = app.camera.basis();
    let view = app.camera.view();
    let projection = app.projection;
    let Some(pos) = app.physics.grab_cloth_near(target, SELECT_THRESHOLD, |p| {
        projection.project(view.transform(p))
    }) else {
        return false;
    };
    app.begin_cloth_grab(pos, forward);
    true
}

fn update_grab(app: &mut App, column: u16, row: u16) {
    if let Some(cloth_grab) = app.cloth_grab.as_ref() {
        let Some((eye, dir)) = cursor_ray(app, column, row) else {
            return;
        };
        if let Some(hit) = plane_hit(eye, dir, cloth_grab.plane_point, cloth_grab.plane_normal) {
            app.physics.drag_cloth_to(hit);
        }
        return;
    }
    let Some((eye, dir)) = cursor_ray(app, column, row) else {
        return;
    };
    let Some(grab) = app.grab.as_mut() else {
        return;
    };
    let Some(hit) = plane_hit(eye, dir, grab.plane_point, grab.plane_normal) else {
        return;
    };

    grab.record(hit);
    let target = hit + grab.offset;
    let handle = grab.handle;
    app.physics.drag_to(handle, target);
}

fn plane_hit(
    eye: Vector3<f64>,
    dir: Vector3<f64>,
    point: Vector3<f64>,
    normal: Vector3<f64>,
) -> Option<Vector3<f64>> {
    let denom = dir.dot(&normal);
    if denom.abs() < 1e-6 {
        return None;
    }
    let t = (point - eye).dot(&normal) / denom;
    (t > 0.0).then(|| eye + dir * t)
}

fn launch_at_cursor(app: &mut App, column: u16, row: u16) {
    let Some((eye, dir)) = cursor_ray(app, column, row) else {
        return;
    };
    let origin = eye + dir * 1.0;
    let vel = dir * app.launch_speed;
    let handle =
        app.physics
            .spawn_scaled(app.spawn_kind, origin, vel, app.spawn_mass, app.spawn_scale);
    app.selected = Some(handle);
}

fn spawn_at_cursor(app: &mut App, column: u16, row: u16) {
    let Some(hit) = ground_hit(app, column, row) else {
        return;
    };
    let handle = match app.spawn_kind {
        SpawnKind::Star => {
            let vel = orbital_velocity(app, hit);
            app.physics
                .spawn_scaled(SpawnKind::Star, hit, vel, app.spawn_mass, app.spawn_scale)
        }
        kind => {
            let pos = hit + Vector3::new(0.0, SPAWN_DROP_HEIGHT, 0.0);
            app.physics
                .spawn_scaled(kind, pos, Vector3::zeros(), app.spawn_mass, app.spawn_scale)
        }
    };
    app.selected = Some(handle);
}

fn orbital_velocity(app: &App, pos: Vector3<f64>) -> Vector3<f64> {
    let bodies = app.physics.render_bodies();
    let Some(attractor) = bodies.iter().max_by(|a, b| a.mass.total_cmp(&b.mass)) else {
        return Vector3::zeros();
    };
    let r = pos - attractor.position;
    let dist = r.norm();
    if dist < 1e-3 {
        return Vector3::zeros();
    }
    let speed = (G * attractor.mass / dist).sqrt();
    let dir = Vector3::new(0.0, 1.0, 0.0).cross(&r).normalize();
    dir * speed + attractor.linvel
}

fn select_at_cursor(app: &mut App, column: u16, row: u16) {
    if let Some(handle) = body_at_cursor(app, column, row) {
        app.selected = Some(handle);
    }
}

fn try_pin_cloth(app: &mut App, column: u16, row: u16) -> bool {
    if !app.physics.has_cloth() {
        return false;
    }
    let Some(target) = screen_to_ndc(app, column, row) else {
        return false;
    };
    let view = app.camera.view();
    let projection = app.projection;
    app.physics
        .toggle_cloth_pin_near(target, SELECT_THRESHOLD, |p| {
            projection.project(view.transform(p))
        })
}

fn body_at_cursor(app: &App, column: u16, row: u16) -> Option<RigidBodyHandle> {
    let (ndc_x, ndc_y) = screen_to_ndc(app, column, row)?;
    let view = app.camera.view();
    let mut best: Option<(RigidBodyHandle, f64)> = None;
    for body in app.physics.render_bodies() {
        if let Some((x, y)) = app.projection.project(view.transform(body.position)) {
            let dist_sq = (x - ndc_x).powi(2) + (y - ndc_y).powi(2);
            if best.is_none_or(|(_, best_dist)| dist_sq < best_dist) {
                best = Some((body.handle, dist_sq));
            }
        }
    }
    best.filter(|&(_, d)| d < SELECT_THRESHOLD.powi(2))
        .map(|(h, _)| h)
}

fn cycle_selection(app: &mut App, forward: bool) {
    let handles: Vec<_> = app.physics.handles().collect();
    if handles.is_empty() {
        app.selected = None;
        return;
    }
    let next_index = match app
        .selected
        .and_then(|h| handles.iter().position(|&x| x == h))
    {
        Some(i) if forward => (i + 1) % handles.len(),
        Some(i) => (i + handles.len() - 1) % handles.len(),
        None => 0,
    };
    app.selected = Some(handles[next_index]);
}

fn screen_to_ndc(app: &App, column: u16, row: u16) -> Option<(f64, f64)> {
    let rect = app.layout.canvas;
    if rect.width < 2 || rect.height < 2 {
        return None;
    }
    if column < rect.x
        || column >= rect.x + rect.width
        || row < rect.y
        || row >= rect.y + rect.height
    {
        return None;
    }
    let width = (rect.width - 1) as f64;
    let height = (rect.height - 1) as f64;
    let px = (column - rect.x) as f64;
    let py = (row - rect.y) as f64;
    let cell_aspect = app.layout.cell_aspect;
    let ndc_x = -1.0 + (px / width) * 2.0;
    let ndc_y = cell_aspect - (py / height) * (2.0 * cell_aspect);
    Some((ndc_x, ndc_y))
}

fn cursor_ray(app: &App, column: u16, row: u16) -> Option<(Vector3<f64>, Vector3<f64>)> {
    let (ndc_x, ndc_y) = screen_to_ndc(app, column, row)?;
    let f = app.projection.focal;
    let view_dir = Vector3::new(ndc_x / f, ndc_y / f, 1.0);
    let (right, up, forward) = app.camera.basis();
    let dir = (right * view_dir.x + up * view_dir.y + forward * view_dir.z).normalize();
    Some((app.camera.eye(), dir))
}

fn ground_hit(app: &App, column: u16, row: u16) -> Option<Vector3<f64>> {
    plane_y_hit(app, column, row, 0.0)
}

fn plane_y_hit(app: &App, column: u16, row: u16, plane_y: f64) -> Option<Vector3<f64>> {
    let (eye, dir) = cursor_ray(app, column, row)?;
    if dir.y.abs() < 1e-6 {
        return None;
    }
    let t = (plane_y - eye.y) / dir.y;
    if t <= 0.0 {
        return None;
    }
    Some(eye + dir * t)
}
