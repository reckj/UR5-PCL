"""Toolhead RS485 link over the UR Tool Communication Interface (TCI).

STATUS: experimental / internal. NOT validated for student use.

REAL-ARM ONLY (live source)
    The live source connects to the UR controller's TCI forwarder at
    TCP <robot_ip>:54321. Requires:
      - the "rs485" ToolComm Forwarder URCap installed on the controller,
      - Tool Communication enabled in the Installation tab (baud/parity must
        match the toolhead firmware), and
      - the toolhead Arduino streaming frames (see toolhead/examples/).
    URSim has NO tool RS485, so there is no simulation path for this link.
    Use `MockToolSource` to develop the host side (parser, viz, motion logic)
    fully offline.

WIRE FRAME (from the toolhead Arduino), one ASCII line per sample:
    R,G,B,C,BTN\\n
parsed into ToolFrame(r, g, b, c, button).

This module deliberately imports NOTHING from the rest of the framework (no
ur_rtde), so the parsing / mapping / mock logic is testable without hardware.
"""

from __future__ import annotations

import math
import socket
import time
from dataclasses import dataclass
from typing import Iterator, Optional

TCI_PORT = 54321  # UR ToolComm Forwarder URCap default


# --------------------------------------------------------------------------- #
# Data model + pure helpers (no I/O - unit-testable)
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class ToolFrame:
    """One sample from the toolhead. Raw APDS9960 channels are 16-bit."""

    r: int
    g: int
    b: int
    c: int
    button: bool

    @property
    def rgb8(self) -> tuple[int, int, int]:
        """R,G,B scaled to 0-255 against the clear channel, for display.

        Falls back to the max raw channel when clear is zero/absent. Always
        returns values clamped to 0-255.
        """
        scale = self.c if self.c > 0 else max(self.r, self.g, self.b, 1)
        out = []
        for v in (self.r, self.g, self.b):
            out.append(max(0, min(255, round(255 * v / scale))))
        return out[0], out[1], out[2]


def parse_line(line: str) -> Optional[ToolFrame]:
    """Parse one ``R,G,B,C,BTN`` line into a ToolFrame.

    Returns None for malformed / partial lines (so a stream reader can simply
    skip them and resync on the next newline).
    """
    parts = line.strip().split(",")
    if len(parts) != 5:
        return None
    try:
        r, g, b, c, btn = (int(p) for p in parts)
    except ValueError:
        return None
    return ToolFrame(r=r, g=g, b=b, c=c, button=bool(btn))


def map_rgb_to_offset(
    frame: ToolFrame,
    *,
    min_mm: float = 5.0,
    max_mm: float = 30.0,
    full_scale: Optional[int] = None,
    signed: bool = False,
) -> tuple[float, float, float]:
    """Map R,G,B -> (dx, dy, dz) Cartesian offset in METERS.

    r -> x, g -> y, b -> z (base frame).

    Each channel is normalised to 0..1 against ``full_scale`` (defaults to the
    frame's clear channel C, falling back to the max raw channel). Then:

      - signed=False (default, literal to the brief): per-axis magnitude is
        ``min_mm + norm * (max_mm - min_mm)``  ->  always within [min_mm, max_mm].
      - signed=True: per-axis offset is ``(norm*2 - 1) * max_mm``  -> range
        [-max_mm, +max_mm], so dim colors pull negative and bright push
        positive (handy for an interactive cumulative demo that can move both
        ways). min_mm is ignored in this mode.

    Returned in meters (offset_mm / 1000).
    """
    scale = full_scale if full_scale else (frame.c if frame.c > 0 else max(frame.r, frame.g, frame.b, 1))

    def axis_mm(v: int) -> float:
        norm = max(0.0, min(1.0, v / scale))
        if signed:
            return (norm * 2.0 - 1.0) * max_mm
        return min_mm + norm * (max_mm - min_mm)

    dx = axis_mm(frame.r) / 1000.0
    dy = axis_mm(frame.g) / 1000.0
    dz = axis_mm(frame.b) / 1000.0
    return dx, dy, dz


class ButtonEdge:
    """Detects a rising edge (released -> pressed) across successive frames."""

    def __init__(self) -> None:
        self._prev = False

    def rising(self, pressed: bool) -> bool:
        edge = pressed and not self._prev
        self._prev = pressed
        return edge

    def reset(self) -> None:
        self._prev = False


# --------------------------------------------------------------------------- #
# Live source - real arm only (TCP :54321)
# --------------------------------------------------------------------------- #
class TciStream:
    """Reads toolhead frames from the UR TCI forwarder (TCP <host>:54321).

    Use as a context manager or call connect()/close() yourself:

        with TciStream("192.168.1.10") as stream:
            for frame in stream.frames():
                ...

    `frames()` is a blocking generator that yields ToolFrame objects until the
    socket closes or close() is called.
    """

    def __init__(self, host: str, port: int = TCI_PORT, timeout: float = 5.0) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout
        self._sock: Optional[socket.socket] = None
        self._closed = False

    def connect(self) -> "TciStream":
        s = socket.create_connection((self.host, self.port), timeout=self.timeout)
        s.settimeout(self.timeout)
        self._sock = s
        self._closed = False
        return self

    def close(self) -> None:
        self._closed = True
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None

    def __enter__(self) -> "TciStream":
        return self.connect()

    def __exit__(self, *exc) -> bool:
        self.close()
        return False

    def frames(self) -> Iterator[ToolFrame]:
        if self._sock is None:
            raise RuntimeError("TciStream not connected; call connect() first")
        buf = b""
        while not self._closed:
            try:
                data = self._sock.recv(4096)
            except socket.timeout:
                continue
            if not data:
                break  # peer closed
            buf += data
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                frame = parse_line(raw.decode("ascii", "ignore"))
                if frame is not None:
                    yield frame


# --------------------------------------------------------------------------- #
# Offline source - synthetic frames so the host side runs without the arm
# --------------------------------------------------------------------------- #
class MockToolSource:
    """Generates synthetic ToolFrame samples for offline development.

    Colors sweep smoothly; the button pulses once every `press_every` seconds
    (a short HIGH window), so button-edge + motion logic and the viz can be
    exercised with no hardware.

        for frame in MockToolSource(max_frames=200).frames():
            ...
    """

    def __init__(
        self,
        period: float = 0.05,
        press_every: float = 3.0,
        press_hold: float = 0.15,
        full_scale: int = 30000,
        max_frames: Optional[int] = None,
        sleep: bool = True,
    ) -> None:
        self.period = period
        self.press_every = press_every
        self.press_hold = press_hold
        self.full_scale = full_scale
        self.max_frames = max_frames
        self.sleep = sleep
        self._stop = False

    def close(self) -> None:
        self._stop = True

    def __enter__(self) -> "MockToolSource":
        return self

    def __exit__(self, *exc) -> bool:
        self.close()
        return False

    def frames(self) -> Iterator[ToolFrame]:
        t = 0.0
        n = 0
        fs = self.full_scale
        while not self._stop:
            if self.max_frames is not None and n >= self.max_frames:
                return
            # three out-of-phase sweeps for r/g/b
            r = int(fs * (0.5 + 0.5 * math.sin(t * 0.7)))
            g = int(fs * (0.5 + 0.5 * math.sin(t * 0.7 + 2.094)))  # +120deg
            b = int(fs * (0.5 + 0.5 * math.sin(t * 0.7 + 4.188)))  # +240deg
            c = max(r, g, b, 1)
            phase = t % self.press_every
            pressed = phase < self.press_hold
            yield ToolFrame(r=r, g=g, b=b, c=c, button=pressed)
            n += 1
            t += self.period
            if self.sleep:
                time.sleep(self.period)
