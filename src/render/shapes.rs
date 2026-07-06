use std::collections::HashMap;

use nalgebra::Vector3;
use ratatui::style::Color;
use ratatui::widgets::canvas::{Circle, Context, Line, Points};

use super::camera::View;
use super::projection::Projection;
use crate::cloth::ClothView;
use crate::fluid::FluidView;

fn project_point(view: &View, projection: &Projection, world: Vector3<f64>) -> Option<(f64, f64)> {
    projection.project(view.transform(world))
}

const TRAIL_FADE_STEPS: usize = 6;

pub fn draw_trail(
    ctx: &mut Context,
    view: &View,
    projection: &Projection,
    points: &[Vector3<f64>],
    color: Color,
) {
    let n = points.len();
    if n == 0 {
        return;
    }
    let mut buckets: [Vec<(f64, f64)>; TRAIL_FADE_STEPS] = Default::default();
    for (i, &p) in points.iter().enumerate() {
        let Some(xy) = project_point(view, projection, p) else {
            continue;
        };
        let age = (i + 1) as f64 / n as f64;
        let bucket = ((age * TRAIL_FADE_STEPS as f64) as usize).min(TRAIL_FADE_STEPS - 1);
        buckets[bucket].push(xy);
    }
    for (bucket, coords) in buckets.iter().enumerate() {
        if coords.is_empty() {
            continue;
        }
        let factor = (bucket + 1) as f64 / TRAIL_FADE_STEPS as f64;
        ctx.draw(&Points {
            coords,
            color: fade(color, factor),
        });
    }
}

fn fade(color: Color, factor: f64) -> Color {
    match color {
        Color::Rgb(r, g, b) => Color::Rgb(
            (r as f64 * factor) as u8,
            (g as f64 * factor) as u8,
            (b as f64 * factor) as u8,
        ),
        other => other,
    }
}

fn draw_world_line(
    ctx: &mut Context,
    view: &View,
    projection: &Projection,
    a: Vector3<f64>,
    b: Vector3<f64>,
    color: Color,
) {
    if let (Some((x1, y1)), Some((x2, y2))) = (
        project_point(view, projection, a),
        project_point(view, projection, b),
    ) {
        ctx.draw(&Line {
            x1,
            y1,
            x2,
            y2,
            color,
        });
    }
}

pub fn draw_grid(
    ctx: &mut Context,
    view: &View,
    projection: &Projection,
    half_extent: i32,
    step: i32,
    color: Color,
) {
    let half = half_extent as f64;
    for i in (-half_extent..=half_extent).step_by(step as usize) {
        let i = i as f64;
        draw_world_line(
            ctx,
            view,
            projection,
            Vector3::new(-half, 0.0, i),
            Vector3::new(half, 0.0, i),
            color,
        );
        draw_world_line(
            ctx,
            view,
            projection,
            Vector3::new(i, 0.0, -half),
            Vector3::new(i, 0.0, half),
            color,
        );
    }
}

pub fn draw_sphere(
    ctx: &mut Context,
    view: &View,
    projection: &Projection,
    center: Vector3<f64>,
    radius: f64,
    color: Color,
) {
    let Some((x, y)) = project_point(view, projection, center) else {
        return;
    };
    let Some((ex, ey)) = project_point(view, projection, center + view.right() * radius) else {
        return;
    };
    let screen_radius = ((ex - x).powi(2) + (ey - y).powi(2)).sqrt();
    ctx.draw(&Circle {
        x,
        y,
        radius: screen_radius,
        color,
    });
}

pub fn draw_cuboid(
    ctx: &mut Context,
    view: &View,
    projection: &Projection,
    center: Vector3<f64>,
    basis: [Vector3<f64>; 3],
    half_extents: Vector3<f64>,
    color: Color,
) {
    let corner = |bits: usize| {
        let sx = if bits & 1 == 0 { -1.0 } else { 1.0 };
        let sy = if bits & 2 == 0 { -1.0 } else { 1.0 };
        let sz = if bits & 4 == 0 { -1.0 } else { 1.0 };
        center
            + basis[0] * sx * half_extents.x
            + basis[1] * sy * half_extents.y
            + basis[2] * sz * half_extents.z
    };
    let corners: [Vector3<f64>; 8] = std::array::from_fn(corner);

    const EDGES: [(usize, usize); 12] = [
        (0, 1),
        (2, 3),
        (4, 5),
        (6, 7),
        (0, 2),
        (1, 3),
        (4, 6),
        (5, 7),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ];
    for (a, b) in EDGES {
        draw_world_line(ctx, view, projection, corners[a], corners[b], color);
    }
}

pub fn draw_fluid(
    ctx: &mut Context,
    view: &View,
    projection: &Projection,
    fluid: &FluidView,
    colors: &[Color],
    emitter_color: Color,
) {
    let mut buckets: HashMap<(u8, u8, u8), Vec<(f64, f64)>> = HashMap::new();
    for (i, &p) in fluid.positions.iter().enumerate() {
        let Some(xy) = project_point(view, projection, p) else {
            continue;
        };
        let key = match colors[i] {
            Color::Rgb(r, g, b) => (r, g, b),
            _ => (255, 255, 255),
        };
        buckets.entry(key).or_default().push(xy);
    }
    for ((r, g, b), coords) in &buckets {
        ctx.draw(&Points {
            coords,
            color: Color::Rgb(*r, *g, *b),
        });
    }

    for &e in &fluid.emitters {
        if let Some((x, y)) = project_point(view, projection, e) {
            ctx.draw(&Circle {
                x,
                y,
                radius: 0.04,
                color: emitter_color,
            });
            ctx.draw(&Points {
                coords: &[(x, y)],
                color: emitter_color,
            });
        }
    }
}

pub fn draw_cloth(
    ctx: &mut Context,
    view: &View,
    projection: &Projection,
    cloth: &ClothView,
    color: Color,
    pin_color: Color,
) {
    let (cols, rows) = (cloth.cols, cloth.rows);
    let idx = |r: usize, c: usize| r * cols + c;
    for r in 0..rows {
        for c in 0..cols {
            let i = idx(r, c);
            if c + 1 < cols {
                draw_world_line(
                    ctx,
                    view,
                    projection,
                    cloth.pos[i],
                    cloth.pos[idx(r, c + 1)],
                    color,
                );
            }
            if r + 1 < rows {
                draw_world_line(
                    ctx,
                    view,
                    projection,
                    cloth.pos[i],
                    cloth.pos[idx(r + 1, c)],
                    color,
                );
            }
        }
    }

    let pins: Vec<(f64, f64)> = cloth
        .pos
        .iter()
        .enumerate()
        .filter(|(i, _)| cloth.pinned[*i])
        .filter_map(|(_, &p)| project_point(view, projection, p))
        .collect();
    if !pins.is_empty() {
        ctx.draw(&Points {
            coords: &pins,
            color: pin_color,
        });
    }
}
