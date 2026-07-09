"""Offline self-test for the toolhead RS485 host side (no hardware, no ur_rtde).

Loads ur5e.toolio.tci_rs485 directly by file path so it runs even where the
full ur5e package (ur_rtde) isn't installed. Exercises the pure pieces -
parsing, RGB->offset mapping, button-edge detection, the mock source - and a
numeric simulation of the cumulative+clamp motion decision.

Run:  python scripts/toolhead_rgb_selftest.py
"""

from __future__ import annotations

import importlib.util
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
MOD = HERE.parent / "src" / "ur5e" / "toolio" / "tci_rs485.py"

spec = importlib.util.spec_from_file_location("tci_rs485", MOD)
m = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = m  # dataclass needs the module registered before exec
spec.loader.exec_module(m)

failures = []


def check(name, cond):
    print(f"  [{'ok' if cond else 'FAIL'}] {name}")
    if not cond:
        failures.append(name)


print("parse_line:")
f = m.parse_line("100,200,300,600,1")
check("valid line parses", f is not None and (f.r, f.g, f.b, f.c, f.button) == (100, 200, 300, 600, True))
check("button 0 -> False", m.parse_line("1,2,3,4,0").button is False)
check("partial line -> None", m.parse_line("100,200,300") is None)
check("garbage -> None", m.parse_line("x,y,z,w,q") is None)
check("trailing CR tolerated", m.parse_line("1,2,3,4,1\r") is not None)

print("ToolFrame.rgb8:")
check("rgb8 clamps to 0-255", all(0 <= v <= 255 for v in m.ToolFrame(99999, 0, 0, 100, False).rgb8))
check("rgb8 c=0 fallback", m.ToolFrame(10, 0, 0, 0, False).rgb8 == (255, 0, 0))

print("map_rgb_to_offset (default, magnitude 5-30 mm):")
fr = m.ToolFrame(r=30000, g=15000, b=0, c=30000, button=True)
dx, dy, dz = m.map_rgb_to_offset(fr, min_mm=5, max_mm=30)
check("r=full -> 30 mm", abs(dx * 1000 - 30) < 1e-6)
check("g=half -> 17.5 mm", abs(dy * 1000 - 17.5) < 1e-6)
check("b=0 -> 5 mm (min)", abs(dz * 1000 - 5) < 1e-6)
check("all axes within [5,30] mm", all(5 - 1e-6 <= v * 1000 <= 30 + 1e-6 for v in (dx, dy, dz)))

print("map_rgb_to_offset (signed, -30..30 mm):")
dxs, dys, dzs = m.map_rgb_to_offset(fr, max_mm=30, signed=True)
check("r=full -> +30 mm", abs(dxs * 1000 - 30) < 1e-6)
check("b=0 -> -30 mm", abs(dzs * 1000 + 30) < 1e-6)

print("ButtonEdge:")
e = m.ButtonEdge()
seq = [False, True, True, False, True]
edges = [e.rising(s) for s in seq]
check("rising edges only on 0->1", edges == [False, True, False, False, True])

print("MockToolSource (bounded, no sleep):")
frames = list(m.MockToolSource(max_frames=120, press_every=3.0, press_hold=0.15,
                               period=0.05, sleep=False).frames())
check("yields requested count", len(frames) == 120)
check("channels in range", all(0 <= fr.r <= 30000 for fr in frames))
check("at least one button press", any(fr.button for fr in frames))

print("cumulative + clamp motion decision (numeric):")


class FakeRobot:
    def __init__(self, pose):
        self._p = list(pose)

    def pose(self):
        return list(self._p)

    def move_to(self, target, **_):
        self._p = list(target)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


base = [0.30, 0.00, 0.20, 0.0, 3.14, 0.0]
rob = FakeRobot(base)
span = 0.040  # 40 mm box
# 60 bright-red presses: dx ~ +30 mm each, cumulative, must clamp at base+span
for _ in range(60):
    dx, dy, dz = m.map_rgb_to_offset(m.ToolFrame(30000, 0, 0, 30000, True),
                                     min_mm=5, max_mm=30)
    cur = rob.pose()
    tx = clamp(cur[0] + dx, base[0] - span, base[0] + span)
    ty = clamp(cur[1] + dy, base[1] - span, base[1] + span)
    tz = clamp(cur[2] + dz, base[2] - span, base[2] + span)
    rob.move_to([tx, ty, tz, cur[3], cur[4], cur[5]])
final = rob.pose()
check("cumulative X clamps at base+span", abs(final[0] - (base[0] + span)) < 1e-9)
check("stays inside box on every axis",
      all(base[i] - span - 1e-9 <= final[i] <= base[i] + span + 1e-9 for i in range(3)))

print()
if failures:
    print(f"FAILED: {len(failures)} check(s): {failures}")
    sys.exit(1)
print("ALL CHECKS PASSED")
