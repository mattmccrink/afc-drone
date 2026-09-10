"""
framing.py -- Pi-side of the RP2350 valve-node USB-CDC link.

Byte-for-byte compatible with the tiny's framing.ino:

    wire format:  [magic:4 LE][type:1][len:1][payload:len][crc8:1]
    crc8:         poly 0x07, init 0x00, over  type | len | payload   (NOT magic)
    reader:       magic-resync state machine -- never blocks, never flushes,
                  re-locks on the 4-byte magic after any garble (the teensyshot
                  host-link idiom, generalised from fixed 64 B to variable len).

Nothing here imports rclpy; the bridge node wires these into subscriptions and a
serial port. Everything is bounds-checked: len is a u8 (<=255) and the payload
buffer is sized to match, so no inbound frame can overrun.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterator, Optional

# ---- protocol constants (mirror config.h) -----------------------------------
PI_MAGIC = 0x52503233           # 'RP23'; on the wire LE -> 33 32 50 52
PI_MAGIC_BYTES = struct.pack("<I", PI_MAGIC)

FT_CMD = 0x01                   # Pi -> node : fast command (6x int16)
FT_ARM = 0x02                   # Pi -> node : slow arm token {state:u8, counter:u32}
FT_CTRL_TLM = 0x81              # node -> Pi : control telemetry
FT_SENSOR_TLM = 0x82            # node -> Pi : sensor telemetry

# ---- CMD field scaling (mirror on_frame() in framing.ino) -------------------
# payload = 6x int16 LE at offsets 0,2,4,6,8,10:
#   roll, pitch, yaw, throttle  (x1000)   |  [8] unused  |  mdot_target (x10)
_SCALE_RPY = 1000.0
_SCALE_THR = 1000.0
_SCALE_MDOT = 10.0
_I16_MIN, _I16_MAX = -32768, 32767


def crc8(data: bytes) -> int:
    """CRC-8 poly 0x07, init 0x00, MSB-first. Identical to the tiny's crc8()."""
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if (c & 0x80) else (c << 1) & 0xFF
    return c


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def _i16(x: float) -> int:
    v = int(round(x))
    return _I16_MIN if v < _I16_MIN else _I16_MAX if v > _I16_MAX else v


# ---- outbound frame writer --------------------------------------------------
def build_frame(ftype: int, payload: bytes = b"") -> bytes:
    """Frame a payload exactly as send_frame() expects to read it back."""
    if not 0 <= len(payload) <= 255:
        raise ValueError(f"payload len {len(payload)} exceeds u8 frame length")
    hdr = PI_MAGIC_BYTES + bytes((ftype & 0xFF, len(payload)))
    cc = crc8(hdr[4:] + payload)          # crc over type|len|payload
    return hdr + payload + bytes((cc,))


def build_cmd(roll: float, pitch: float, yaw: float,
              thrust: float, mdot_target: float,
              *, thrust_lo: float = 0.0, thrust_hi: float = 1.0) -> bytes:
    """Pack a FT_CMD frame from normalised, pre-mix control demand.

    roll/pitch/yaw are clamped to [-1, 1]; thrust to [thrust_lo, thrust_hi]
    (default [0, 1] -- widen to [-1, 1] for a reversible-thrust airframe);
    mdot_target in g/s. Offset [8] (the dead 'altitude' slot the tiny ignores)
    is sent as zero.
    """
    r = _i16(_clamp(roll,  -1.0, 1.0) * _SCALE_RPY)
    p = _i16(_clamp(pitch, -1.0, 1.0) * _SCALE_RPY)
    y = _i16(_clamp(yaw,   -1.0, 1.0) * _SCALE_RPY)
    t = _i16(_clamp(thrust, thrust_lo, thrust_hi) * _SCALE_THR)
    m = _i16(_clamp(mdot_target, 0.0, 400.0) * _SCALE_MDOT)
    payload = struct.pack("<6h", r, p, y, t, 0, m)
    return build_frame(FT_CMD, payload)


def build_arm(armed: bool, counter: int) -> bytes:
    """Pack a FT_ARM frame: state:u8, counter:u32 LE."""
    payload = struct.pack("<BI", 1 if armed else 0, counter & 0xFFFFFFFF)
    return build_frame(FT_ARM, payload)


# ---- inbound magic-resync reader --------------------------------------------
@dataclass
class Frame:
    type: int
    payload: bytes


class FrameReader:
    """Feed raw serial bytes in; get whole, CRC-checked frames out.

    Mirrors the tiny's parse_byte() state machine. A bad CRC or a garbled
    header costs at most one frame: the reader falls back into magic search and
    re-locks -- it never flushes the buffer and never blocks.
    """

    _MAGIC = 0                 # 0..3 = matching magic byte N
    _TYPE = 4
    _LEN = 5
    _PAYLOAD = 6
    _CRC = 7

    def __init__(self) -> None:
        self._state = self._MAGIC
        self._magic_pos = 0
        self._type = 0
        self._len = 0
        self._buf = bytearray()
        self.crc_errors = 0     # observability: count silently-dropped frames

    def feed(self, chunk: bytes) -> Iterator[Frame]:
        for byte in chunk:
            frame = self._step(byte)
            if frame is not None:
                yield frame

    def _step(self, c: int) -> Optional[Frame]:
        st = self._state
        if st == self._MAGIC:
            if c == PI_MAGIC_BYTES[self._magic_pos]:
                self._magic_pos += 1
                if self._magic_pos == 4:
                    self._magic_pos = 0
                    self._state = self._TYPE
            else:
                # resync: a fresh magic[0] restarts the match, else back to 0
                self._magic_pos = 1 if c == PI_MAGIC_BYTES[0] else 0
            return None
        if st == self._TYPE:
            self._type = c
            self._state = self._LEN
            return None
        if st == self._LEN:
            self._len = c
            self._buf.clear()
            self._state = self._PAYLOAD if c else self._CRC
            return None
        if st == self._PAYLOAD:
            self._buf.append(c)
            if len(self._buf) >= self._len:
                self._state = self._CRC
            return None
        # st == self._CRC
        self._state = self._MAGIC
        want = crc8(bytes((self._type, self._len)) + self._buf)
        if want == c:
            return Frame(self._type, bytes(self._buf))
        self.crc_errors += 1
        return None


# ---- node counts (mirror config.h) ------------------------------------------
VALVE_COUNT = 6
SERVO_COUNT = 12

# ---- inbound telemetry decoders (CTRL_TLM 0x81, SENSOR_TLM 0x82) ------------
# Mirror tlm_service() in framing.ino. Payload layouts:
#   CTRL_TLM   (43 B): B source, B armed, B comp_mode, B flow_fallback,
#                      H rpm_target, B n_valid, 6h valve[], 12H servo_us[]
#   SENSOR_TLM (56 B): 6h p_up*10, 6h p_lo*10, 6h t_die*100, 6h mdot*10,
#                      1h mdot_total*10, H valid_mask, I node_stamp_ms
CTRL_TLM_LEN = 43
SENSOR_TLM_LEN = 56


def decode_ctrl_tlm(payload: bytes) -> dict:
    if len(payload) < CTRL_TLM_LEN:
        raise ValueError(f"CTRL_TLM short: {len(payload)} < {CTRL_TLM_LEN}")
    source, armed, comp_mode, flow_fb, rpm_target, n_valid = \
        struct.unpack_from("<BBBBHB", payload, 0)
    valve = list(struct.unpack_from("<6h", payload, 7))
    servo_us = list(struct.unpack_from("<12H", payload, 19))
    return dict(source=source, armed=bool(armed), comp_mode=comp_mode,
                flow_fallback=bool(flow_fb), rpm_target=rpm_target,
                n_valid=n_valid, valve=valve, servo_us=servo_us)


def decode_sensor_tlm(payload: bytes) -> dict:
    if len(payload) < SENSOR_TLM_LEN:
        raise ValueError(f"SENSOR_TLM short: {len(payload)} < {SENSOR_TLM_LEN}")
    p_up = [x / 10.0 for x in struct.unpack_from("<6h", payload, 0)]
    p_lo = [x / 10.0 for x in struct.unpack_from("<6h", payload, 12)]
    t_die = [x / 100.0 for x in struct.unpack_from("<6h", payload, 24)]
    mdot = [x / 10.0 for x in struct.unpack_from("<6h", payload, 36)]
    (mdot_total,) = struct.unpack_from("<h", payload, 48)
    (mask,) = struct.unpack_from("<H", payload, 50)
    (node_stamp_ms,) = struct.unpack_from("<I", payload, 52)
    valid = [bool((mask >> v) & 1) for v in range(VALVE_COUNT)]
    return dict(p_up=p_up, p_lo=p_lo, t_die=t_die, mdot=mdot,
                mdot_total=mdot_total / 10.0, valid=valid,
                node_stamp_ms=node_stamp_ms)
