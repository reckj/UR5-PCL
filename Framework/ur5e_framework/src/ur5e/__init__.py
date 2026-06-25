"""ur5e — UR5e research/teaching framework (experimental / internal).

NOT validated for student use. Upstream toolbuilding only.
"""

from __future__ import annotations

from .connection import RobotConnection
from .config import resolve_host
from .robot import Robot, grid_poses, line_poses

__all__ = ["Robot", "grid_poses", "line_poses",
           "RobotConnection", "resolve_host", "connect"]


def connect(target_or_host: str, **kwargs) -> RobotConnection:
    """Connect by named target ('sim'/'real') or a raw host/IP.

    Receive-only unless control=True / io=True are passed through.
    Returns an already-connected RobotConnection.
    """
    host = resolve_host(target_or_host) if target_or_host in ("sim", "real") else target_or_host
    return RobotConnection(host, **kwargs).connect()
