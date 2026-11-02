//! Raw declarations for busbridge.h.
//!
//! Thin on purpose: every function mirrors its C prototype and nothing here
//! tries to be safe or idiomatic. The safe surface is in `device`.
//!
//! Only the general entry points are declared, not the library's typed helpers
//! for I2C, I3C or its sniffer. That is a deliberate choice rather than
//! laziness: the typed sniffer API - bb_sniff_start and friends - is the
//! STM32-era surface, and it cannot express capture mode, tap binding, sample
//! rate or source. Driving the RT engine through it would mean asking a
//! 12.5 MHz instrument questions from a 1 MHz vocabulary. bb_command() carries
//! the whole command surface and always will, because that surface is the
//! contract the firmware documents.
#![allow(non_camel_case_types)]
#![allow(dead_code)]

use std::os::raw::{c_char, c_int};

pub type bb_device = *mut BbDeviceOpaque;
pub type bb_handle = *mut BbHandleOpaque;

#[repr(C)]
pub struct BbDeviceOpaque {
    _private: [u8; 0],
}

#[repr(C)]
pub struct BbHandleOpaque {
    _private: [u8; 0],
}

extern "C" {
    /// Walk the attached boards. Pass null for the first, then each result.
    pub fn bb_find_devices(first: bb_device) -> bb_device;
    pub fn bb_device_serial(dev: bb_device) -> *const c_char;
    pub fn bb_device_port(dev: bb_device) -> *const c_char;

    pub fn bb_open(dev: bb_device) -> bb_handle;
    pub fn bb_close(hndl: bb_handle) -> c_int;
    pub fn bb_set_timeout(hndl: bb_handle, ms: c_int) -> c_int;

    /// Send one command line, receive the reply with its "OK "/"ERR " prefix
    /// already stripped. Negative return is a BB_ERR_* code.
    pub fn bb_command(hndl: bb_handle, cmd: *const c_char, reply: *mut c_char, sz: c_int)
        -> c_int;

    /// The last reply in full, prefix included. bb_command() hands back only
    /// the payload, and every script in this project was written against the
    /// prefixed form - "OK 40 11 22", "ERR 5 ..." - so this is what the UI
    /// shows.
    pub fn bb_last_reply() -> *const c_char;
    pub fn bb_strerror(code: c_int) -> *const c_char;
}
