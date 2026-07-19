//! Platform-neutral input events. The desktop build maps crossterm events into
//! these; the browser build maps ratzilla events into these. This keeps `input.rs`
//! free of any backend-specific types so it compiles for both native and wasm.

pub enum Key {
    Char(char),
    Left,
    Right,
    Up,
    Down,
    Enter,
    Esc,
    Tab,
    Other,
}

pub struct KeyInput {
    pub key: Key,
    pub shift: bool,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Button {
    Left,
    Right,
    Middle,
}

pub enum MouseKind {
    Down(Button),
    Up(Button),
    Drag(Button),
    Moved,
    Other,
}

pub struct MouseInput {
    pub kind: MouseKind,
    pub col: u16,
    pub row: u16,
}

#[cfg(not(target_arch = "wasm32"))]
mod native {
    use super::{Button, Key, KeyInput, MouseInput, MouseKind};
    use ratatui::crossterm::event::{
        KeyCode, KeyEvent, KeyModifiers, MouseButton, MouseEvent, MouseEventKind,
    };

    impl From<KeyEvent> for KeyInput {
        fn from(ev: KeyEvent) -> Self {
            let key = match ev.code {
                KeyCode::Char(c) => Key::Char(c),
                KeyCode::Left => Key::Left,
                KeyCode::Right => Key::Right,
                KeyCode::Up => Key::Up,
                KeyCode::Down => Key::Down,
                KeyCode::Enter => Key::Enter,
                KeyCode::Esc => Key::Esc,
                KeyCode::Tab => Key::Tab,
                _ => Key::Other,
            };
            KeyInput {
                key,
                shift: ev.modifiers.contains(KeyModifiers::SHIFT),
            }
        }
    }

    fn button(b: MouseButton) -> Button {
        match b {
            MouseButton::Left => Button::Left,
            MouseButton::Right => Button::Right,
            MouseButton::Middle => Button::Middle,
        }
    }

    impl From<MouseEvent> for MouseInput {
        fn from(ev: MouseEvent) -> Self {
            let kind = match ev.kind {
                MouseEventKind::Down(b) => MouseKind::Down(button(b)),
                MouseEventKind::Up(b) => MouseKind::Up(button(b)),
                MouseEventKind::Drag(b) => MouseKind::Drag(button(b)),
                MouseEventKind::Moved => MouseKind::Moved,
                _ => MouseKind::Other,
            };
            MouseInput {
                kind,
                col: ev.column,
                row: ev.row,
            }
        }
    }
}
