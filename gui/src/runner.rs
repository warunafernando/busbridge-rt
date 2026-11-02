//! Running a bench script and watching its output arrive.
//!
//! The measurement tools stay as scripts and are launched rather than
//! reimplemented. That is deliberate: `acceptance.py`, `clock_offset.py` and
//! the rest are the things that get run from a terminal when something is
//! wrong, they are what the documentation cites, and a second implementation
//! inside a window would be a second answer to the same question. A window
//! that disagrees with the terminal is worse than no window.
//!
//! A script holds the board's USB interface for as long as it runs, and that
//! interface belongs to one process at a time - so this disconnects first and
//! says so, rather than letting the script fail with "would not open" and
//! leaving the reason off screen.

use std::io::{BufRead, BufReader};
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{channel, Receiver};
use std::sync::Arc;

pub struct Run {
    pub title: String,
    pub lines: Vec<String>,
    pub done: bool,
    pub exit: Option<i32>,
    rx: Receiver<Line>,
    kill: Arc<AtomicBool>,
}

enum Line {
    Out(String),
    Done(Option<i32>),
}

impl Run {
    /// Start `python <script> <args...>` in the scripts directory.
    pub fn python(scripts_dir: &str, script: &str, args: &[String]) -> Run {
        let title = if args.is_empty() {
            script.to_string()
        } else {
            format!("{script} {}", args.join(" "))
        };
        let (tx, rx) = channel();
        let kill = Arc::new(AtomicBool::new(false));

        let dir = scripts_dir.to_string();
        let script = script.to_string();
        let args = args.to_vec();
        let kill_thread = kill.clone();

        std::thread::Builder::new()
            .name("script".into())
            .spawn(move || {
                let mut cmd = Command::new("python");
                cmd.current_dir(&dir)
                    .arg(&script)
                    .args(&args)
                    // Unbuffered, or a script that prints a line per check
                    // shows nothing for minutes and then everything at once -
                    // which makes a long run look like a hung one.

                    .env("PYTHONUNBUFFERED", "1")
                    .stdout(Stdio::piped())
                    .stderr(Stdio::piped());

                let mut child = match cmd.spawn() {
                    Ok(c) => c,
                    Err(e) => {
                        let _ = tx.send(Line::Out(format!("could not start python: {e}")));
                        let _ = tx.send(Line::Done(None));
                        return;
                    }
                };

                if let Some(out) = child.stdout.take() {
                    let tx = tx.clone();
                    let kill = kill_thread.clone();
                    std::thread::spawn(move || {
                        for l in BufReader::new(out).lines().map_while(Result::ok) {
                            if kill.load(Ordering::Relaxed) {
                                break;
                            }
                            let _ = tx.send(Line::Out(l));
                        }
                    });
                }
                if let Some(err) = child.stderr.take() {
                    let tx = tx.clone();
                    std::thread::spawn(move || {
                        for l in BufReader::new(err).lines().map_while(Result::ok) {
                            // Marked, not merged silently: a traceback mixed
                            // into a pass/fail list reads as a result.
                            let _ = tx.send(Line::Out(format!("! {l}")));
                        }
                    });
                }

                loop {
                    if kill_thread.load(Ordering::Relaxed) {
                        let _ = child.kill();
                        let _ = tx.send(Line::Out("(stopped)".into()));
                        let _ = tx.send(Line::Done(None));
                        return;
                    }
                    match child.try_wait() {
                        Ok(Some(st)) => {
                            // A moment for the pipe readers to finish the last
                            // lines; exiting immediately truncates the result
                            // the run exists to produce.
                            std::thread::sleep(std::time::Duration::from_millis(120));
                            let _ = tx.send(Line::Done(st.code()));
                            return;
                        }
                        Ok(None) => std::thread::sleep(std::time::Duration::from_millis(60)),
                        Err(e) => {
                            let _ = tx.send(Line::Out(format!("wait failed: {e}")));
                            let _ = tx.send(Line::Done(None));
                            return;
                        }
                    }
                }
            })
            .expect("script thread");

        Run {
            title,
            lines: Vec::new(),
            done: false,
            exit: None,
            rx,
            kill,
        }
    }

    /// Take whatever has arrived. Call once a frame.
    pub fn pump(&mut self) {
        while let Ok(l) = self.rx.try_recv() {
            match l {
                Line::Out(s) => {
                    self.lines.push(s);
                    // A long run can print thousands of lines; keep the tail.
                    if self.lines.len() > 4000 {
                        self.lines.drain(..self.lines.len() - 4000);
                    }
                }
                Line::Done(code) => {
                    self.done = true;
                    self.exit = code;
                }
            }
        }
    }

    pub fn stop(&self) {
        self.kill.store(true, Ordering::Relaxed);
    }

    /// Pass, fail, or still going - from the exit code, not from the text.
    pub fn verdict(&self) -> &'static str {
        if !self.done {
            return "running";
        }
        match self.exit {
            Some(0) => "passed",
            Some(_) => "FAILED",
            None => "stopped",
        }
    }
}
