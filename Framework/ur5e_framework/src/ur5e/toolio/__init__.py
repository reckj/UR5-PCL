"""ur5e.toolio - toolhead I/O over channels that are NOT plain RTDE.

STATUS: experimental / internal. NOT validated for student use.

Currently: the toolhead RS485 link over the UR Tool Communication Interface
(TCI), which the controller forwards to TCP <robot_ip>:54321. This path exists
ONLY on the real arm (URSim has no tool RS485); use MockToolSource for offline
development. See Documentation/10-toolhead-rgb-rs485-interface.md.
"""

from __future__ import annotations

from .tci_rs485 import (
    ToolFrame,
    parse_line,
    map_rgb_to_offset,
    ButtonEdge,
    TciStream,
    MockToolSource,
    TCI_PORT,
)

__all__ = [
    "ToolFrame",
    "parse_line",
    "map_rgb_to_offset",
    "ButtonEdge",
    "TciStream",
    "MockToolSource",
    "TCI_PORT",
]
