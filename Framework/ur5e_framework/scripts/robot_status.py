"""Why can't I connect? — robot status in a few seconds. Read-only, moves nothing.

STATUS: validated for student use (HS26) — plain sockets, no ur_rtde, cannot hang:
every step has a 3 s timeout.

Asks the robot's own dashboard server (port 29999) what state it is in and
tells you what is blocking a motion script:

    python scripts/robot_status.py --target real      (UR_HOST must be set)
    python scripts/robot_status.py --host 192.168.1.11
    python scripts/robot_status.py --target sim

A motion script (anything using Robot / --target real) can only connect when
ALL of these hold — a receive-only connection or a ping works without them:

    network   this container reaches the robot (not only the laptop itself)
    Remote    the pendant is in Remote Control (header icon)
    RUNNING   the arm is powered ON and STARTED (brakes released)
    NORMAL    no protective stop / emergency stop / safety fault
    nobody else is driving the arm from another laptop at the same time
"""

from __future__ import annotations

import argparse
import os
import socket

PORTS = [(29999, "dashboard"), (30004, "RTDE (state + control)"), (30002, "script upload"), (30003, "realtime")]
TIMEOUT = 3.0


def port_open(host: str, port: int) -> str:
    """'' if a TCP connection opens, else the reason."""
    try:
        with socket.create_connection((host, port), timeout=TIMEOUT):
            return ""
    except socket.timeout:
        return "no answer (timeout)"
    except OSError as e:
        return str(e)


class Dashboard:
    def __init__(self, host: str):
        self.sock = socket.create_connection((host, 29999), timeout=TIMEOUT)
        self.sock.settimeout(TIMEOUT)
        self.greeting = self._read()

    def _read(self) -> str:
        data = b""
        try:
            while not data.endswith(b"\n"):
                chunk = self.sock.recv(1024)
                if not chunk:
                    break
                data += chunk
        except socket.timeout:
            pass
        return data.decode(errors="replace").strip()

    def ask(self, command: str) -> str:
        self.sock.sendall((command + "\n").encode())
        return self._read()

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


ROBOTMODE_FIX = {
    "POWER_OFF": "the arm is powered off — pendant: red button bottom-left -> ON, then START",
    "IDLE": "the arm is on but the brakes are engaged — pendant: bottom-left -> START",
    "POWER_ON": "the arm is still powering up — wait, then START",
    "BOOTING": "the controller is still booting — wait",
    "BACKDRIVE": "the arm is in backdrive — release the freedrive button, then START",
    "NO_CONTROLLER": "the controller is not running — power-cycle the robot",
    "DISCONNECTED": "the arm is not connected to the control box",
    "CONFIRM_SAFETY": "the pendant is waiting for 'Confirm Safety Configuration' — confirm it on the pendant",
}
SAFETY_FIX = {
    "REDUCED": "",        # fine, just slower
    "PROTECTIVE_STOP": "protective stop — acknowledge it on the pendant (Enable robot), check what the arm touched",
    "RECOVERY": "recovery mode — the arm is outside a safety limit; move it back by hand (freedrive), then restart",
    "SAFEGUARD_STOP": "safeguard stop is active — check the safeguard input",
    "SYSTEM_EMERGENCY_STOP": "emergency stop (system) — release it, then ON -> START",
    "ROBOT_EMERGENCY_STOP": "emergency stop pressed — twist the red button to release it, then ON -> START",
    "VIOLATION": "safety violation — see the pendant's log; power the arm off and on",
    "FAULT": "safety fault — see the pendant's log; restart the robot",
}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", choices=["sim", "real"], default=None)
    ap.add_argument("--host", default=None)
    args = ap.parse_args()
    if args.host:
        host = args.host
    elif args.target == "sim":
        host = os.environ.get("URSIM_HOST", "127.0.0.1")
    else:
        host = os.environ.get("UR_HOST", "")
        if not host:
            raise SystemExit("no robot address: start the shell with  -e UR_HOST=192.168.1.11  or pass --host 192.168.1.11")

    blockers = []
    print(f"robot status for {host}\n")
    print("1. network (from inside this container)")
    reasons = {}
    for port, what in PORTS:
        reasons[port] = port_open(host, port)
        print(f"   port {port:<6} {what:<24} {'open' if not reasons[port] else 'NOT reachable — ' + reasons[port]}")
    if all(reasons.values()):
        print("\n=> NETWORK: this container cannot reach the robot at all.\n"
              "   - On the laptop itself (not in Docker): ping " + host + "\n"
              "     no reply -> cable / the laptop's Ethernet address (must be 192.168.1.x, mask 255.255.255.0)\n"
              "     reply    -> the laptop reaches the robot but Docker does not: on macOS check\n"
              "                 System Settings > Privacy & Security > Local Network > Docker (must be ON),\n"
              "                 then quit and restart Docker Desktop; a VPN can cause the same\n"
              "   - wrong address? the pendant shows it under the menu (top right) > About")
        raise SystemExit(1)
    if reasons[29999]:
        print("\n=> the dashboard port is closed but other ports answer — cannot read the robot state.")
        raise SystemExit(1)
    if reasons[30004]:
        blockers.append("RTDE port 30004 is not reachable although the dashboard is — restart the robot controller")

    print("\n2. robot state (dashboard server)")
    d = Dashboard(host)
    try:
        version = d.ask("PolyScopeVersion")
        mode = d.ask("robotmode")
        safety = d.ask("safetystatus")
        remote = d.ask("is in remote control")
        running = d.ask("running")
        program = d.ask("programState")
        loaded = d.ask("get loaded program")
    finally:
        d.close()
    for label, value in (("software", version), ("robot mode", mode), ("safety", safety), ("remote control", remote),
                         ("program", f"{running}   [{program}]   {loaded}")):
        print(f"   {label:<15} {value}")

    mode_word = mode.split(":")[-1].strip().upper()
    safety_word = safety.split(":")[-1].strip().upper()
    if remote.strip().lower() == "false":
        blockers.append("the pendant is in LOCAL mode — tap the Local/Remote icon in the header and choose Remote Control")
    elif remote.strip().lower() != "true":
        print(f"   (could not read the Remote Control state: {remote!r})")
    if mode_word != "RUNNING":
        blockers.append(ROBOTMODE_FIX.get(mode_word, f"robot mode is {mode_word}, not RUNNING — pendant: ON, then START"))
    if safety_word not in ("NORMAL", "REDUCED"):
        blockers.append(SAFETY_FIX.get(safety_word, f"safety status is {safety_word} — see the pendant"))
    if "true" in running.lower():
        print("   note: a program is running on the controller right now. If that is another laptop's script,\n"
              "         two scripts cannot drive the arm at once — agree who has the robot. (Connecting stops it.)")

    print()
    if blockers:
        print("=> NOT READY for a motion script:")
        for b in blockers:
            print("   - " + b)
        print("   Also look at the pendant itself: an open pop-up dialog blocks every script until it is closed.")
        raise SystemExit(1)
    print("=> READY: network, Remote Control, RUNNING, safety NORMAL. A motion script should connect.\n"
          "   If it still hangs: look for an open pop-up on the pendant, and make sure no second laptop\n"
          "   is connected with a motion script.")


if __name__ == "__main__":
    main()
