//! Browser entry point. Runs the exact same `App`, physics, and renderer as the
//! desktop TUI, but drives them from ratzilla's canvas backend instead of a
//! terminal: a requestAnimationFrame render loop plus DOM key/mouse events.

use std::cell::RefCell;
use std::rc::Rc;

use ratatui::Terminal;
use ratzilla::backend::canvas::CanvasBackendOptions;
use ratzilla::event::{
    KeyCode as RKeyCode, KeyEvent as RKeyEvent, MouseButton as RMouseButton,
    MouseEvent as RMouseEvent, MouseEventKind as RMouseKind,
};
use ratzilla::{CanvasBackend, WebRenderer};
use web_time::Instant;

use crate::app::{App, Scene};
use crate::event::{Button, Key, KeyInput, MouseInput, MouseKind};
use crate::{input, menu, render};

/// Id of the DOM element the canvas mounts into. The page must contain
/// `<div id="inertia-screen"></div>` sized to the desired render area.
const SCREEN_ID: &str = "inertia-screen";

/// The web build starts on the same scene picker the desktop program shows, then
/// runs the chosen sandbox. Quitting the sandbox returns here.
enum Screen {
    Menu(usize),
    Running(App),
}

/// Boots the browser sandbox. Called from the wasm `main`; `draw_web` keeps its
/// requestAnimationFrame loop alive internally after this returns.
pub fn run() {
    console_error_panic_hook::set_once();

    let screen = Rc::new(RefCell::new(Screen::Menu(0)));

    let backend =
        CanvasBackend::new_with_options(CanvasBackendOptions::new().grid_id(SCREEN_ID))
            .expect("create canvas backend");
    let mut terminal = Terminal::new(backend).expect("create terminal");

    // ratzilla has no Drag event; it emits Moved while a button is held. Track the
    // held button so we can synthesize the drags the input layer expects.
    let held: Rc<RefCell<Option<Button>>> = Rc::new(RefCell::new(None));

    terminal
        .on_key_event({
            let screen = screen.clone();
            move |ev| {
                let mut screen = screen.borrow_mut();
                match &mut *screen {
                    Screen::Menu(cursor) => {
                        if let Some(scene) = menu_key(cursor, to_key(ev)) {
                            *screen = Screen::Running(App::new(scene));
                        }
                    }
                    Screen::Running(app) => {
                        input::handle_key(app, to_key(ev));
                        if !app.running {
                            *screen = Screen::Menu(0);
                        }
                    }
                }
            }
        })
        .expect("attach key handler");

    terminal
        .on_mouse_event({
            let screen = screen.clone();
            let held = held.clone();
            move |ev| {
                if let Screen::Running(app) = &mut *screen.borrow_mut() {
                    if let Some(mouse) = to_mouse(ev, &held) {
                        input::handle_mouse(app, mouse);
                    }
                }
            }
        })
        .expect("attach mouse handler");

    let mut last = Instant::now();
    terminal.draw_web(move |frame| {
        let now = Instant::now();
        let elapsed = now.duration_since(last);
        last = now;
        match &mut *screen.borrow_mut() {
            Screen::Menu(cursor) => menu::draw(frame, *cursor),
            Screen::Running(app) => {
                app.tick(elapsed);
                app.reconcile_grab();
                render::draw(frame, app);
            }
        }
    });
}

/// Menu navigation, mirroring the desktop `choose_scene` loop. Returns the picked
/// scene on enter or a number key.
fn menu_key(cursor: &mut usize, key: KeyInput) -> Option<Scene> {
    let n = menu::OPTIONS.len();
    match key.key {
        Key::Up => {
            *cursor = (*cursor + n - 1) % n;
            None
        }
        Key::Down => {
            *cursor = (*cursor + 1) % n;
            None
        }
        Key::Enter => Some(menu::OPTIONS[*cursor].2),
        Key::Char(c @ '1'..='9') => {
            let i = c as usize - '1' as usize;
            (i < n).then(|| menu::OPTIONS[i].2)
        }
        _ => None,
    }
}

fn to_key(ev: RKeyEvent) -> KeyInput {
    let key = match ev.code {
        RKeyCode::Char(c) => Key::Char(c),
        RKeyCode::Left => Key::Left,
        RKeyCode::Right => Key::Right,
        RKeyCode::Up => Key::Up,
        RKeyCode::Down => Key::Down,
        RKeyCode::Enter => Key::Enter,
        RKeyCode::Esc => Key::Esc,
        RKeyCode::Tab => Key::Tab,
        _ => Key::Other,
    };
    KeyInput { key, shift: ev.shift }
}

fn to_button(b: RMouseButton) -> Option<Button> {
    match b {
        RMouseButton::Left => Some(Button::Left),
        RMouseButton::Right => Some(Button::Right),
        RMouseButton::Middle => Some(Button::Middle),
        _ => None,
    }
}

fn to_mouse(ev: RMouseEvent, held: &Rc<RefCell<Option<Button>>>) -> Option<MouseInput> {
    let kind = match ev.kind {
        RMouseKind::ButtonDown(b) => {
            let b = to_button(b)?;
            *held.borrow_mut() = Some(b);
            MouseKind::Down(b)
        }
        RMouseKind::ButtonUp(b) => {
            let b = to_button(b)?;
            *held.borrow_mut() = None;
            MouseKind::Up(b)
        }
        RMouseKind::Moved => match *held.borrow() {
            Some(Button::Left) => MouseKind::Drag(Button::Left),
            _ => MouseKind::Moved,
        },
        _ => return None,
    };
    Some(MouseInput {
        kind,
        col: ev.col,
        row: ev.row,
    })
}
