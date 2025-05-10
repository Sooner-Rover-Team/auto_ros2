//! # `drive_tui`
//!
//! This is a [TUI] ("terminal user interface") to assist in managing the Rover
//! as it performs Autonomous tasks.
//!
//! It provides various functionality, but most importantly, it's the primary
//! method of asking the Rover to do something.
//!
//! [TUI]: https://en.wikipedia.org/wiki/Text-based_user_interface

use std::time::{Duration, Instant};

use ratatui::{
    crossterm::{
        event::{self, DisableMouseCapture, EnableMouseCapture, Event, KeyEventKind},
        execute,
        terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
    },
    prelude::{Backend, CrosstermBackend},
    Frame, Terminal,
};

/// State for the app.
#[derive(Debug)]
struct App {
    should_quit: bool,
}

#[allow(clippy::derivable_impls)]
impl Default for App {
    fn default() -> Self {
        Self { should_quit: false }
    }
}

impl App {
    /// How fast the app's screen updates.
    const TICK_RATE: Duration = Duration::from_millis(8); // 120 hz

    /// Runs the application's main loop until the user quits...
    fn run(&mut self) -> anyhow::Result<()> {
        // setup terminal
        enable_raw_mode()?;
        let mut stdout = std::io::stdout();
        execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;
        let backend = CrosstermBackend::new(stdout);
        let mut terminal = Terminal::new(backend)?;

        // create app and run it
        let app = App::default();
        Self::run_app(&mut terminal, app, Self::TICK_RATE).expect("app completion result");

        // restore terminal
        disable_raw_mode()?;
        execute!(
            terminal.backend_mut(),
            LeaveAlternateScreen,
            DisableMouseCapture
        )?;
        terminal.show_cursor()?;

        Ok(())
    }

    fn run_app<B: Backend>(
        terminal: &mut Terminal<B>,
        mut app: App,
        tick_rate: Duration,
    ) -> anyhow::Result<()> {
        let mut last_tick = Instant::now();
        loop {
            terminal.draw(|frame| app.draw(frame))?;

            let timeout = tick_rate.saturating_sub(last_tick.elapsed());
            if event::poll(timeout)? {
                if let Event::Key(key) = event::read()? {
                    if key.kind == KeyEventKind::Press {
                        {} // TODO(bray): handle various inputs for navigation?
                    }
                }
            }
            if last_tick.elapsed() >= tick_rate {
                app.on_tick()?;
                last_tick = Instant::now();
            }
            if app.should_quit {
                return Ok(());
            }
        }
    }

    fn draw(&self, _frame: &mut Frame) {
        todo!()
    }

    fn on_tick(&mut self) -> anyhow::Result<()> {
        todo!()
    }
}

fn main() -> anyhow::Result<()> {
    let app_result = App::default().run();
    ratatui::restore();
    app_result
}
