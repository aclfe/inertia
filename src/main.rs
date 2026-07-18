#[cfg(not(target_arch = "wasm32"))]
use std::io;

#[cfg(not(target_arch = "wasm32"))]
use ratatui::crossterm::event::{DisableMouseCapture, EnableMouseCapture};
#[cfg(not(target_arch = "wasm32"))]
use ratatui::crossterm::execute;

#[cfg(not(target_arch = "wasm32"))]
use inertia_tui::{app, menu};

#[cfg(not(target_arch = "wasm32"))]
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

// On wasm the same binary is the browser entry: trunk builds it and calls `main`,
// which boots the ratzilla sandbox (see `src/web.rs`).
#[cfg(target_arch = "wasm32")]
fn main() {
    inertia_tui::web::run();
}
