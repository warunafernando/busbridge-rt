"""Talk to the two boards on this bench.

Built over the BUSBRIDGE C library rather than over raw WinUSB. The library
already owns the transport and is tested against these boards, and - the part
that matters here - it can pick a device by serial, which is the only thing
that tells two attached boards apart: every one of them reports its port as
"USB".

`rt()` and `nu()` return cached handles, because a board's USB interface
belongs to one process at a time and opening it twice from one script would
fail the second time for a reason that looks like a wiring fault.

A note on replies: bb_command() hands back the *payload*, with the leading
"OK " already removed, and signals failure through its return code instead.
`cmd()` puts the prefix back, so callers see exactly what the firmware sent -
"OK 40 11 22 33 44 55", "ERR 5 ...". That is the shape every script here was
written against, and the shape the firmware documents.

Getting this wrong is not loud. Strip the prefix and a test comparing against
"OK ..." fails everywhere at once, which is at least obvious; but a binary
reply parsed with the wrong offset shifts every record by a byte and still
looks like data. That presented as the top byte of each timestamp toggling by
2^24 - a line mask read one place too high - and cost an afternoon.
"""
import ctypes
import os

# The library is built beside the firmware it talks to. Overridable so a
# checkout somewhere else, or a freshly built DLL, can be pointed at.
DLL = os.environ.get(
    "BUSBRIDGE_DLL",
    r"E:\STMicro\busbridge\windows\build\x64\busbridge.dll")

_lib = ctypes.CDLL(DLL)
_lib.bb_find_devices.restype = ctypes.c_void_p
_lib.bb_find_devices.argtypes = [ctypes.c_void_p]
_lib.bb_device_port.restype = ctypes.c_char_p
_lib.bb_device_port.argtypes = [ctypes.c_void_p]
_lib.bb_device_serial.restype = ctypes.c_char_p
_lib.bb_device_serial.argtypes = [ctypes.c_void_p]
_lib.bb_open.restype = ctypes.c_void_p
_lib.bb_open.argtypes = [ctypes.c_void_p]
_lib.bb_close.argtypes = [ctypes.c_void_p]
_lib.bb_set_timeout.argtypes = [ctypes.c_void_p, ctypes.c_int]
_lib.bb_command.restype = ctypes.c_int
_lib.bb_command.argtypes = [ctypes.c_void_p, ctypes.c_char_p,
                            ctypes.c_char_p, ctypes.c_int]
_lib.bb_last_reply.restype = ctypes.c_char_p

# Board serials. The RT1186 is the controller and the sniffer; the NUCLEO is
# the device under test. A prefix is enough and survives a re-enumeration.
RT_SERIAL = "975a6193"
NU_SERIAL = "36393335"

_cache = {}


def devices():
    """Every attached board, as (serial, port, handle)."""
    out, d = [], _lib.bb_find_devices(None)
    while d:
        out.append((_lib.bb_device_serial(d).decode(errors="replace"),
                    _lib.bb_device_port(d).decode(errors="replace"), d))
        d = _lib.bb_find_devices(ctypes.c_void_p(d))
    return out


class Board:
    def __init__(self, serial_prefix, timeout_ms=15000):
        found = [t for t in devices()
                 if t[0].lower().startswith(serial_prefix.lower())]
        if not found:
            attached = [t[0] for t in devices()]
            raise OSError(
                "no board with serial starting %r. Attached: %s. A board whose "
                "own USB connector has no cable to this machine will not appear "
                "here - the debug probe carries debug only."
                % (serial_prefix, attached or "none"))
        self.serial = found[0][0]
        self.h = _lib.bb_open(ctypes.c_void_p(found[0][2]))
        if not self.h:
            raise OSError("board %s is attached but would not open. Another "
                          "process may be holding it - one at a time."
                          % self.serial)
        self.timeout(timeout_ms)

    def timeout(self, ms):
        _lib.bb_set_timeout(ctypes.c_void_p(self.h), int(ms))

    def cmd(self, line):
        """Send a command and return the firmware's reply, prefix and all."""
        buf = ctypes.create_string_buffer(4096)
        rc = _lib.bb_command(ctypes.c_void_p(self.h), line.encode(),
                             buf, len(buf))
        text = buf.value.decode(errors="replace")
        if rc == 0:
            return ("OK " + text) if text else "OK"
        # A failure keeps its reason in the library's last reply - as the
        # firmware's detail only, without the "ERR <n> " the firmware sent, and
        # with `rc` being the *library's* error enum rather than the firmware's
        # number. The reason is the whole value of an error here, so it is the
        # part preserved; the number is not reconstructed, because inventing
        # one that looks like the firmware's would be worse than omitting it.
        last = _lib.bb_last_reply()
        last = last.decode(errors="replace") if last else ""
        if last.startswith("ERR"):
            return last
        if text.startswith("ERR"):
            return text
        return "ERR " + (last or text or ("rc=%d" % rc))

    def close(self):
        if getattr(self, "h", None):
            _lib.bb_close(ctypes.c_void_p(self.h))
            self.h = None


def board(serial_prefix):
    if serial_prefix not in _cache:
        _cache[serial_prefix] = Board(serial_prefix)
    return _cache[serial_prefix]


def rt():
    """The FRDM-IMXRT1186: bus controller and sniffer."""
    return board(RT_SERIAL)


def nu():
    """The NUCLEO-H533RE: the target."""
    return board(NU_SERIAL)


def reopen():
    """Drop the cached handles.

    Needed after a board resets - SYS:RESET, a reflash - because the old
    handle refers to a USB device that has gone away and come back as a new
    one. Without this the next command fails in a way that reads like the
    firmware is broken.
    """
    for b in _cache.values():
        b.close()
    _cache.clear()
