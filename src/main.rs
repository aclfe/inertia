mod app;
mod cloth;
mod collider;
mod fluid;
mod input;
mod menu;
mod nbody;
mod physics;
mod render;

use std::io;

use ratatui::crossterm::event::{DisableMouseCapture, EnableMouseCapture};
use ratatui::crossterm::execute;

fn main() {
    let mut terminal = ratatui::init();
    let result = (|| -> io::Result<()> {
        let Some(scene) = menu::choose_scene(&mut terminal)? else {
            return Ok(());
        };
        execute!(io::stdout(), EnableMouseCapture)?;
        let outcome = app::App::new(scene).run(&mut terminal);
        let _ = execute!(io::stdout(), DisableMouseCapture);
        outcome
    })();
    ratatui::restore();
    if let Err(err) = result {
        eprintln!("error: {err}");
        std::process::exit(1);
    }
}
