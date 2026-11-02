//! A board, and the replies it gives.
//!
//! One safe wrapper over `ffi`, plus the parsing for the RT capture surface.
//! Everything here blocks on USB, so nothing here runs on the UI thread -
//! `worker` owns the handle and this is what it calls.
//!
//! Replies keep their `OK `/`ERR ` prefix. `bb_command()` strips it and signals
//! failure through its return code instead, which is the opposite of what every
//! script in this project was written against, and getting it wrong is not
//! loud: a binary reply parsed one byte off still looks like data.

use std::ffi::{CStr, CString};
use std::os::raw::c_char;

use crate::ffi;

/// Which board is on the other end, and therefore what it can be asked.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Board {
    /// FRDM-IMXRT1186: CM33 controller plus the CM7 capture engine.
    Rt1186,
    /// A NUCLEO running the STM32H5 build. Shares the command surface; has
    /// none of the capture engine.
    Stm32h5,
    /// Answered SYS:ID? with something this build does not recognise.
    Unknown,
}

impl Board {
    pub fn label(self) -> &'static str {
        match self {
            Board::Rt1186 => "FRDM-IMXRT1186",
            Board::Stm32h5 => "STM32H5 (NUCLEO)",
            Board::Unknown => "unrecognised board",
        }
    }

    /// Does this board have the CM7 capture engine behind SNIFF:*?
    ///
    /// The question every RT-only panel asks before offering itself. An
    /// unrecognised board is treated as not having it: offering a control that
    /// silently does nothing is worse than greying it out with a reason.
    pub fn has_capture_engine(self) -> bool {
        self == Board::Rt1186
    }
}

pub struct Info {
    pub serial: String,
    pub port: String,
}

/// Every attached board, in enumeration order.
pub fn list() -> Vec<Info> {
    let mut out = Vec::new();
    unsafe {
        let mut dev = ffi::bb_find_devices(std::ptr::null_mut());
        while !dev.is_null() {
            out.push(Info {
                serial: cstr(ffi::bb_device_serial(dev)),
                port: cstr(ffi::bb_device_port(dev)),
            });
            dev = ffi::bb_find_devices(dev);
        }
    }
    out
}

fn cstr(p: *const c_char) -> String {
    if p.is_null() {
        return String::new();
    }
    unsafe { CStr::from_ptr(p) }.to_string_lossy().into_owned()
}

pub struct Device {
    handle: ffi::bb_handle,
    pub serial: String,
    pub board: Board,
    pub ident: String,
}

impl Device {
    /// Open the board with this serial. A board's USB interface belongs to one
    /// process at a time, so this fails plainly if something else holds it -
    /// including the bench console next door, which is the usual culprit.
    pub fn open(serial: &str) -> Result<Device, String> {
        unsafe {
            let mut dev = ffi::bb_find_devices(std::ptr::null_mut());
            while !dev.is_null() {
                if cstr(ffi::bb_device_serial(dev)) == serial {
                    let h = ffi::bb_open(dev);
                    if h.is_null() {
                        return Err(format!(
                            "{serial} is attached but would not open. \
                             One process at a time - the bench console, or a \
                             script, may be holding it."
                        ));
                    }
                    ffi::bb_set_timeout(h, 20_000);
                    let mut d = Device {
                        handle: h,
                        serial: serial.to_string(),
                        board: Board::Unknown,
                        ident: String::new(),
                    };
                    d.identify();
                    return Ok(d);
                }
                dev = ffi::bb_find_devices(dev);
            }
        }
        Err(format!("no board with serial {serial} is attached"))
    }

    /// Ask what this is, once, at open.
    ///
    /// Matched on the board name rather than on a USB product id. The id is
    /// the same family across both builds and the name is what the firmware
    /// chose to call itself, which is the thing that changes when the firmware
    /// does.
    fn identify(&mut self) {
        let reply = self.command("SYS:ID?");
        self.ident = reply.clone();
        let up = reply.to_ascii_uppercase();
        self.board = if up.contains("IMXRT1186") || up.contains("FRDM-IMXRT") {
            Board::Rt1186
        } else if up.contains("NUCLEO") || up.contains("STM32H5") {
            Board::Stm32h5
        } else {
            Board::Unknown
        };
    }

    /// Send one line, get the whole reply back with its prefix restored.
    ///
    /// The prefix has to be put back, and the way it is put back is not a
    /// detail. `bb_command()` hands over the payload only and signals failure
    /// through its return code; `bb_last_reply()` holds the firmware's own
    /// error *text*, without the "ERR <n> " the firmware sent, and only when
    /// something failed. So success is "rc is zero", never "the text looks
    /// right".
    ///
    /// Getting that wrong is quiet. The first version here trusted
    /// bb_last_reply() whenever it was non-empty and treated anything not
    /// starting with "OK" as a failure - so every command that answers with a
    /// payload, `SNIFF:DMA` replying `dma=1` among them, read as an error
    /// while the commands that answer with a bare OK read as success. A
    /// configuration push stopped at the fourth line with nothing wrong.
    ///
    /// This mirrors bb.py's cmd(), deliberately: the scripts and this window
    /// have to agree about what a reply says, and two reconstructions of one
    /// protocol is one too many.
    pub fn command(&self, line: &str) -> String {
        let Ok(c) = CString::new(line) else {
            return "ERR 0 command-contains-a-nul".into();
        };
        // 4096 rather than the protocol's 1200, so a reply that overruns the
        // documented maximum is still seen whole here instead of being cut a
        // second time on the way to the screen. A line that long is a firmware
        // bug, and one that is visible is worth more than one that is tidy.
        let mut buf = vec![0u8; 4096];
        let rc = unsafe {
            ffi::bb_command(
                self.handle,
                c.as_ptr(),
                buf.as_mut_ptr() as *mut c_char,
                buf.len() as i32,
            )
        };
        let n = buf.iter().position(|&b| b == 0).unwrap_or(0);
        let text = String::from_utf8_lossy(&buf[..n]).into_owned();

        if rc == 0 {
            return if text.is_empty() {
                "OK".into()
            } else {
                format!("OK {text}")
            };
        }

        // The reason is the whole value of an error, so it is the part kept.
        // The firmware's own number is not reconstructed: inventing one that
        // looks like it would be worse than leaving it out.
        let last = unsafe { cstr(ffi::bb_last_reply()) };
        if last.starts_with("ERR") {
            return last;
        }
        if text.starts_with("ERR") {
            return text;
        }
        let why = unsafe { cstr(ffi::bb_strerror(rc)) };
        let detail = if !last.is_empty() {
            last
        } else if !text.is_empty() {
            text
        } else {
            format!("rc={rc} {why}")
        };
        format!("ERR {detail}")
    }

    /// True when the reply is a success.
    pub fn ok(reply: &str) -> bool {
        reply.starts_with("OK")
    }
}

impl Drop for Device {
    fn drop(&mut self) {
        unsafe {
            ffi::bb_close(self.handle);
        }
    }
}

// ---- parsing the capture surface -----------------------------------------

/// Pull `key=value` out of a reply. Returns None rather than a default,
/// because "the field was absent" and "the field was zero" are different
/// things and this project has been bitten by conflating them.
pub fn field<'a>(reply: &'a str, key: &str) -> Option<&'a str> {
    reply
        .split_whitespace()
        .find_map(|t| t.strip_prefix(key))
        .filter(|v| !v.is_empty())
}

pub fn field_u64(reply: &str, key: &str) -> Option<u64> {
    field(reply, key).and_then(|v| v.parse().ok())
}

/// The event tokens from a SNIFF:READ? reply.
///
/// Everything up to and including `more=` is header; the tokens start at the
/// first space after it. That split is the host-side contract the firmware is
/// written against - which is why the firmware emits `more=` last and always
/// follows it with a space, even when no tokens follow.
pub fn tokens(reply: &str) -> Vec<String> {
    let body = match reply.split_once("more=") {
        Some((_, rest)) => match rest.find(' ') {
            Some(i) => &rest[i..],
            None => "",
        },
        None => return Vec::new(),
    };
    body.split_whitespace()
        .filter(|t| {
            // The notes the firmware adds after more= are named here rather
            // than guessed at by shape: a token filter that worked by shape
            // would drop a real event the day one is added that looks like a
            // note.
            !matches!(
                *t,
                "framing=i3c-t-bit-not-ack"
                    | "framing=i2c-ack"
                    | "note=bits-exact-tail-truncates-under-8-bits"
            )
        })
        .map(str::to_string)
        .collect()
}

/// One entry of a SNIFF:TRACE? reply: a counter delta and its token.
///
/// `~` after the token marks a flag the controller computed rather than
/// observed. It is kept as a field rather than folded into the token, because
/// the whole point of the mark is that it must not be mistaken for a check
/// that passed.
pub struct Mark {
    pub dt_cycles: u64,
    pub token: String,
    pub derived: bool,
}

pub fn marks(reply: &str) -> Vec<Mark> {
    reply
        .split_whitespace()
        .filter_map(|t| {
            let (d, tok) = t.split_once('=')?;
            let dt: u64 = d.parse().ok()?;
            let derived = tok.ends_with('~');
            Some(Mark {
                dt_cycles: dt,
                token: tok.trim_end_matches('~').to_string(),
                derived,
            })
        })
        .collect()
}
