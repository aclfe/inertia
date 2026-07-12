use std::io;

use ratatui::DefaultTerminal;
use ratatui::Frame;
use ratatui::crossterm::event::{self, Event, KeyCode};
use ratatui::layout::{Alignment, Constraint, Layout};
use ratatui::style::Stylize;
use ratatui::text::Line;
use ratatui::widgets::{Block, Borders, Paragraph};

use crate::app::Scene;

const OPTIONS: [(&str, &str, Scene); 5] = [
    (
        "Blank sandbox",
        "start empty and spawn objects yourself",
        Scene::Blank,
    ),
    (
        "Rigid demo",
        "box pyramid with a ball launched into it",
        Scene::RigidStack,
    ),
    (
        "N-body demo",
        "three-body figure-eight in mutual gravity",
        Scene::ThreeBody,
    ),
    (
        "Cloth demo",
        "a hammock catches two balls dropped in sequence",
        Scene::Cloth,
    ),
    (
        "Thermal demo",
        "a hot floor drives convection in a tank of water",
        Scene::Thermal,
    ),
];

pub fn choose_scene(terminal: &mut DefaultTerminal) -> io::Result<Option<Scene>> {
    let mut cursor = 0usize;
    loop {
        terminal.draw(|frame| draw(frame, cursor))?;
        if let Event::Key(key) = event::read()? {
            match key.code {
                KeyCode::Char('q') | KeyCode::Esc => return Ok(None),
                KeyCode::Up => cursor = (cursor + OPTIONS.len() - 1) % OPTIONS.len(),
                KeyCode::Down => cursor = (cursor + 1) % OPTIONS.len(),
                KeyCode::Enter => return Ok(Some(OPTIONS[cursor].2)),
                KeyCode::Char(c @ '1'..='9') => {
                    let i = c as usize - '1' as usize;
                    if i < OPTIONS.len() {
                        return Ok(Some(OPTIONS[i].2));
                    }
                }
                _ => {}
            }
        }
    }
}

fn draw(frame: &mut Frame, cursor: usize) {
    let mut lines = vec![
        Line::from("INERTIA".bold()),
        Line::from("a terminal physics sandbox".dim()),
        Line::from(""),
    ];
    for (i, (name, desc, _)) in OPTIONS.iter().enumerate() {
        let text = format!(
            " {} {}. {name} - {desc}",
            if i == cursor { ">" } else { " " },
            i + 1
        );
        lines.push(if i == cursor {
            Line::from(text.bold().yellow())
        } else {
            Line::from(text)
        });
    }
    lines.push(Line::from(""));
    lines.push(Line::from(
        "up/down: move   1-5: pick   enter: start   q: quit".dim(),
    ));

    let height = lines.len() as u16 + 2;
    let width = 60;
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

    let menu = Paragraph::new(lines)
        .alignment(Alignment::Left)
        .block(Block::new().borders(Borders::ALL).title(" start ".bold()));
    frame.render_widget(menu, center);
}
