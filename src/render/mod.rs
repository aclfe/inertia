pub mod camera;
pub mod projection;
pub mod shapes;

use nalgebra::Vector3;
use ratatui::Frame;
use ratatui::layout::{Alignment, Constraint, Layout, Rect};
use ratatui::style::{Color, Stylize};
use ratatui::symbols::Marker;
use ratatui::text::Line;
use ratatui::widgets::canvas::Canvas;
use ratatui::widgets::{Block, Borders, Clear, Paragraph};

use crate::app::{App, Tool, Tunable};
use crate::physics::{self, BodyKind, BodyView, GravityMode};

const GRID_STEP: i32 = 2;
const SIDEBAR_WIDTH: u16 = 32;
const SELECTED_COLOR: Color = Color::LightYellow;
const MAX_SPEED_FOR_COLOR: f64 = 8.0;
const CLOTH_COLOR: Color = Color::Rgb(120, 170, 210);
const CLOTH_PIN_COLOR: Color = Color::LightRed;
const FLUID_EMITTER_COLOR: Color = Color::LightCyan;
const CONTAINER_COLOR: Color = Color::Rgb(150, 150, 160);

#[derive(Clone, Copy, Default)]
pub struct Layout3D {
    pub canvas: Rect,
    pub cell_aspect: f64,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ColorMode {
    Velocity,
    Mass,
    Temperature,
    Density,
}

impl ColorMode {
    pub fn next(self) -> Self {
        match self {
            ColorMode::Velocity => ColorMode::Mass,
            ColorMode::Mass => ColorMode::Temperature,
            ColorMode::Temperature => ColorMode::Density,
            ColorMode::Density => ColorMode::Velocity,
        }
    }
}

fn velocity_color(speed: f64) -> Color {
    let t = (speed / MAX_SPEED_FOR_COLOR).clamp(0.0, 1.0);
    Color::Rgb(
        (t * 255.0) as u8,
        ((1.0 - t) * 180.0) as u8,
        ((1.0 - t) * 255.0) as u8,
    )
}

fn mass_t(mass: f64) -> f64 {
    (mass.max(1.0).log10() / 3.0).clamp(0.0, 1.0)
}

fn mass_color(mass: f64) -> Color {
    let v = (60.0 + mass_t(mass) * 195.0) as u8;
    Color::Rgb(v, v, v)
}

const TEMP_COLD_C: f64 = -20.0;
const TEMP_HOT_C: f64 = 250.0;

fn heat_color(temp: f64) -> Color {
    let u = ((temp - TEMP_COLD_C) / (TEMP_HOT_C - TEMP_COLD_C)).clamp(0.0, 1.0);
    let (r, g, b) = if u < 0.5 {
        let v = u * 2.0;
        (v, 0.0, 1.0 - v)
    } else {
        let v = (u - 0.5) * 2.0;
        (1.0, v, (v - 0.5).max(0.0) * 2.0)
    };
    Color::Rgb((r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8)
}

fn body_color(mode: ColorMode, body: &BodyView) -> Color {
    match mode {
        ColorMode::Velocity => velocity_color(body.speed),
        ColorMode::Mass => mass_color(body.mass),
        ColorMode::Temperature => heat_color(body.temp),
        ColorMode::Density => velocity_color(body.speed),
    }
}

fn density_color(ratio: f64) -> Color {
    let t = ((ratio - 0.6) / 0.8).clamp(0.0, 1.0);
    Color::Rgb(
        (t * 255.0) as u8,
        (80.0 + (1.0 - (t - 0.5).abs() * 2.0) * 120.0) as u8,
        ((1.0 - t) * 255.0) as u8,
    )
}

fn fluid_color(mode: ColorMode, speed: f64, density_ratio: f64, temp: f64) -> Color {
    match mode {
        ColorMode::Density => density_color(density_ratio),
        ColorMode::Temperature => heat_color(temp),
        _ => velocity_color(speed),
    }
}

pub fn draw(frame: &mut Frame, app: &mut App) {
    let [main_area, hint_area] =
        Layout::vertical([Constraint::Min(0), Constraint::Length(1)]).areas(frame.area());
    let [canvas_area, sidebar_area] =
        Layout::horizontal([Constraint::Min(0), Constraint::Length(SIDEBAR_WIDTH)])
            .areas(main_area);

    let cell_aspect = 2.0 * canvas_area.height.max(1) as f64 / canvas_area.width.max(1) as f64;
    app.layout = Layout3D {
        canvas: canvas_area,
        cell_aspect,
    };

    let view = app.camera.view();
    let projection = &app.projection;
    let bodies = app.physics.render_bodies();
    let cloth_views = app.physics.cloth_views();
    let fluid_view = app.physics.fluid_view();
    let container_walls = app.physics.container_walls().to_vec();
    let selected = app.selected;
    let color_mode = app.color_mode;
    let fluid_colors: Vec<Color> = fluid_view
        .as_ref()
        .map(|f| {
            f.speeds
                .iter()
                .zip(&f.density_ratio)
                .zip(&f.temps)
                .map(|((&s, &d), &t)| fluid_color(color_mode, s, d, t))
                .collect()
        })
        .unwrap_or_default();
    let draw_grid = app.physics.floor_mode() != physics::FloorMode::Void;

    let canvas = Canvas::default()
        .marker(Marker::Braille)
        .x_bounds([-1.0, 1.0])
        .y_bounds([-cell_aspect, cell_aspect])
        .paint(|ctx| {
            if draw_grid {
                shapes::draw_grid(
                    ctx,
                    &view,
                    projection,
                    physics::GRID_HALF_EXTENT as i32,
                    GRID_STEP,
                    Color::DarkGray,
                );
            }
            for &(center, half) in &container_walls {
                let basis = [Vector3::x(), Vector3::y(), Vector3::z()];
                shapes::draw_cuboid(ctx, &view, projection, center, basis, half, CONTAINER_COLOR);
            }
            if let Some(fluid) = &fluid_view {
                shapes::draw_fluid(
                    ctx,
                    &view,
                    projection,
                    fluid,
                    &fluid_colors,
                    FLUID_EMITTER_COLOR,
                );
            }
            for body in &bodies {
                if !body.trail.is_empty() {
                    shapes::draw_trail(
                        ctx,
                        &view,
                        projection,
                        &body.trail,
                        body_color(color_mode, body),
                    );
                }
            }
            for body in &bodies {
                let color = if Some(body.handle) == selected {
                    SELECTED_COLOR
                } else {
                    body_color(color_mode, body)
                };
                match body.kind {
                    BodyKind::Sphere { radius } | BodyKind::Star { radius } => {
                        shapes::draw_sphere(ctx, &view, projection, body.position, radius, color);
                    }
                    BodyKind::Box { half_extents } => {
                        shapes::draw_cuboid(
                            ctx,
                            &view,
                            projection,
                            body.position,
                            body.basis,
                            half_extents,
                            color,
                        );
                    }
                }
            }
            for cloth in &cloth_views {
                shapes::draw_cloth(ctx, &view, projection, cloth, CLOTH_COLOR, CLOTH_PIN_COLOR);
            }
        });
    frame.render_widget(canvas, canvas_area);

    draw_sidebar(frame, app, &bodies, sidebar_area);

    let hint = Line::from(
        " ?: help   d: place/grab   L-drag: throw   space: pause   ;/,/.: tune   1-9: sandbox   q: quit ",
    );
    frame.render_widget(Paragraph::new(hint).dim(), hint_area);

    if app.show_help {
        draw_help(frame, app.help_page);
    }
}

fn panel(title: &str) -> Block<'static> {
    Block::new()
        .borders(Borders::ALL)
        .title(format!(" {title} ").bold())
}

fn draw_sidebar(frame: &mut Frame, app: &App, bodies: &[BodyView], area: Rect) {
    let [mode_area, params_area, spawn_area, inspector_area] = Layout::vertical([
        Constraint::Length(4),
        Constraint::Length(Tunable::ALL.len() as u16 + 2),
        Constraint::Length(8),
        Constraint::Min(3),
    ])
    .areas(area);

    draw_mode_panel(frame, app, mode_area);
    draw_params_panel(frame, app, params_area);
    draw_spawn_panel(frame, app, spawn_area);
    draw_inspector_panel(frame, app, bodies, inspector_area);
}

fn draw_mode_panel(frame: &mut Frame, app: &App, area: Rect) {
    let mut header = vec![app.mode_label().bold()];
    if app.physics.thermo_enabled() {
        header.push("  Thermal".magenta().bold());
    }
    if app.paused {
        header.push("  [PAUSED]".red().bold());
    }
    let lines = vec![
        Line::from(header),
        Line::from(format!(
            "Sandbox {}/{}    Bodies {}",
            app.active_sandbox() + 1,
            app.sandbox_count(),
            app.physics.len(),
        )),
    ];
    frame.render_widget(Paragraph::new(lines).block(panel("Mode")), area);
}

fn draw_params_panel(frame: &mut Frame, app: &App, area: Rect) {
    let lines: Vec<Line> = Tunable::ALL
        .iter()
        .map(|&t| {
            let selected = t == app.tunable;
            let marker = if selected { "\u{25b8}" } else { " " };
            let text = format!(" {marker} {:<8}{}", t.label(), app.tunable_value(t));
            if !app.tunable_applies(t) {
                Line::from(text.dark_gray())
            } else if selected {
                Line::from(text.yellow().bold())
            } else {
                Line::from(text)
            }
        })
        .collect();
    frame.render_widget(
        Paragraph::new(lines).block(panel("Parameters  ; , .")),
        area,
    );
}

fn draw_spawn_panel(frame: &mut Frame, app: &App, area: Rect) {
    let mouse = match app.tool {
        Tool::Place => "Place (L-click drops)".to_string(),
        Tool::Grab => "Grab (L-drag throws)".to_string(),
        Tool::Fluid => "Fluid (L-pour R-source)".to_string(),
        Tool::Heat => "Heat (L-warm R-cool)".to_string(),
        Tool::Build => format!("Build {} (R cycles)", app.build_kind.label()),
    };
    let mut lines = vec![
        Line::from(vec!["Mouse  ".into(), mouse.bold()]),
        Line::from(format!("Kind   {:?}", app.spawn_kind)),
        Line::from(format!(
            "Mass   {:.0}   x{:.2}",
            app.spawn_mass, app.spawn_scale
        )),
        Line::from(format!("Color  {:?}", app.color_mode)),
    ];
    if app.physics.gravity_mode() == GravityMode::NBody {
        lines.push(Line::from(format!(
            "Solver {:?}   Trails {}",
            app.physics.nbody_algo(),
            if app.physics.show_trails() {
                "on"
            } else {
                "off"
            },
        )));
    } else {
        lines.push(Line::from(format!("Floor  {:?}", app.physics.floor_mode())));
    }
    if app.physics.has_cloth() {
        lines.push(Line::from(format!(
            "Cloth  on    Wind {}",
            if app.physics.wind() { "on" } else { "off" },
        )));
    }
    if app.physics.has_container() {
        lines.push(Line::from("Tank   on"));
    }
    if app.physics.has_fluid() {
        let sources = app.physics.fluid_emitter_count();
        let mut text = format!("Fluid  {} particles", app.physics.fluid_len());
        if sources > 0 {
            text.push_str(&format!("  {sources} src"));
        }
        lines.push(Line::from(text));
    }
    if app.physics.thermo_enabled() {
        lines.push(Line::from(format!(
            "Thermo on    Amb {:.0} C",
            app.physics.ambient()
        )));
    }
    frame.render_widget(Paragraph::new(lines).block(panel("Spawn")), area);
}

fn draw_inspector_panel(frame: &mut Frame, app: &App, bodies: &[BodyView], area: Rect) {
    let lines = match bodies.iter().find(|b| Some(b.handle) == app.selected) {
        Some(body) => {
            let kind_name = match body.kind {
                BodyKind::Box { .. } => "Box",
                BodyKind::Sphere { .. } => "Sphere",
                BodyKind::Star { .. } => "Star",
            };
            let mut lines = vec![
                Line::from(kind_name.bold()),
                Line::from(format!(
                    "Pos  {:.2}, {:.2}, {:.2}",
                    body.position.x, body.position.y, body.position.z
                )),
                Line::from(format!(
                    "Vel  {:.2}, {:.2}, {:.2}",
                    body.linvel.x, body.linvel.y, body.linvel.z
                )),
                Line::from(format!("Speed {:.2} m/s", body.speed)),
                Line::from(format!("Mass  {:.2} kg", body.mass)),
            ];
            if app.physics.thermo_enabled() {
                lines.push(Line::from(format!("Temp  {:.0} C", body.temp)));
            }
            lines
        }
        None => vec![Line::from("nothing selected".dim())],
    };
    frame.render_widget(Paragraph::new(lines).block(panel("Inspector")), area);
}

type HelpGroup = (&'static str, &'static [(&'static str, &'static str)]);

const HELP_PAGE_1: &[HelpGroup] = &[
    (
        "Camera",
        &[
            ("arrows", "orbit"),
            ("shift+arrows", "pan"),
            ("+ / -", "zoom"),
            ("r", "reset view"),
        ],
    ),
    (
        "Mouse tools",
        &[
            ("d", "cycle the active tool"),
            ("Place", "L drop object   (b/s/n picks kind)"),
            ("Grab", "L drag or throw a body / cloth vertex"),
            ("Fluid", "L pour water    R toggle source"),
            ("Heat", "L warm    R cool    (body or water)"),
            ("Build", "L place    R cycle platform/ramp/wall"),
            ("M-click", "launch object from camera"),
            ("R-click", "select body / pin cloth vertex"),
        ],
    ),
    (
        "Objects",
        &[
            ("b / s / n", "box / sphere / star  (picks Place)"),
            ("[ / ]", "spawn mass down / up"),
            ("k / l", "cloth sheet / pinned hammock"),
            ("j", "fluid block"),
            ("o", "water tank (toggle)"),
            ("K / J", "clear cloth / fluid"),
            ("x / X", "delete selected / clear all"),
        ],
    ),
];

const HELP_PAGE_2: &[HelpGroup] = &[
    (
        "Space",
        &[
            ("g", "enter / leave space (mutual gravity)"),
            ("a", "n-body solver"),
            ("t", "orbit trails"),
        ],
    ),
    (
        "Simulation",
        &[
            ("space", "pause"),
            ("m", "single step"),
            ("f", "floor mode"),
            ("w", "wind (cloth)"),
            ("e", "thermodynamics"),
            ("c", "color mode"),
        ],
    ),
    (
        "Parameters",
        &[
            (";", "select parameter"),
            (", / .", "decrease / increase"),
            ("Size", "a parameter; scales new bodies"),
        ],
    ),
    ("Sandboxes", &[("1-9", "switch / create slot")]),
    (
        "General",
        &[
            ("? / h", "close this help"),
            ("tab / space", "flip page"),
            ("q / esc", "quit"),
        ],
    ),
];

fn draw_help(frame: &mut Frame, page: usize) {
    let (groups, subtitle) = if page == 0 {
        (HELP_PAGE_1, "1/2  interaction")
    } else {
        (HELP_PAGE_2, "2/2  world & sim")
    };

    let mut lines = vec![
        Line::from(vec!["Controls   ".bold(), subtitle.dim()]),
        Line::from(""),
    ];
    for &(title, binds) in groups {
        lines.push(Line::from(title.bold().cyan()));
        for &(key, desc) in binds {
            lines.push(Line::from(format!("  {key:<13}{desc}")));
        }
        lines.push(Line::from(""));
    }
    lines.push(Line::from("tab/space: flip page    ? or esc: close".dim()));

    let width = 58u16;
    let height = lines.len() as u16 + 2;
    let area = frame.area();
    let [_, mid, _] = Layout::vertical([
        Constraint::Fill(1),
        Constraint::Length(height.min(area.height)),
        Constraint::Fill(1),
    ])
    .areas(area);
    let [_, center, _] = Layout::horizontal([
        Constraint::Fill(1),
        Constraint::Length(width.min(area.width)),
        Constraint::Fill(1),
    ])
    .areas(mid);

    frame.render_widget(Clear, center);
    let help = Paragraph::new(lines)
        .alignment(Alignment::Left)
        .block(panel("Help"));
    frame.render_widget(help, center);
}
