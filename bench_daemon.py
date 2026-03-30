#!/usr/bin/env python3
"""
bench_daemon.py — SPEC CPU2006 benchmark daemon for CoMeT/Sniper.

Runs a "zone" (1-4) slice of the SPEC benchmark list in parallel,
fully daemonised (detached from the terminal session).

Usage:
    ./bench_daemon.py start  <zone>   # launch benchmarks for that zone
    ./bench_daemon.py stop   [zone]   # graceful stop (SIGTERM), default: all
    ./bench_daemon.py kill   [zone]   # forceful stop (SIGKILL), default: all
    ./bench_daemon.py status [zone]   # show running state,     default: all
    ./bench_daemon.py list                 Show zone → benchmark mapping

Zones split the BENCHMARKS list into 4 roughly-equal slices.
Each benchmark runs in its own output directory under test/spec/<name>/
and logs to test/spec/<name>.log.
"""

import os
import sys
import signal
import subprocess
import math
import time
import json
from typing import Optional, Dict, Any

# ──────────────────────────────────────────────────────────────────────────────
# Config
# ──────────────────────────────────────────────────────────────────────────────
COMET_ROOT = os.path.dirname(os.path.abspath(__file__))
RUN_SNIPER = os.path.join(COMET_ROOT, "run-sniper")
PINBALL_BASE = os.path.join(
    COMET_ROOT, "cpu2006-wholeprogram-pinballs-pinplay-1.1"
)
SPEC_DIR = os.path.join(COMET_ROOT, "test", "spec")
PID_DIR = os.path.join(COMET_ROOT, ".bench_daemon")
NUM_CORES = 16

# ──────────────────────────────────────────────────────────────────────────────
# Full list of SPEC benchmarks (folder names inside the pinball directory).
# Each entry is the <name> portion of  cpu2006-<name>-ref-1/pinball
# and maps to --traceinput/benchmarks=spec-<name>-native-1
# ──────────────────────────────────────────────────────────────────────────────
BENCHMARKS = [
    "astar",
    "bwaves",
    "bzip2",
    "cactusADM",
    "calculix",
    "dealII",
    "gamess",
    "gcc",
    "GemsFDTD",
    "gobmk",
    "gromacs",
    "h264ref",
    "hmmer",
    "lbm",
    "leslie3d",
    "libquantum",
    "mcf",
    "milc",
    "namd",
    "omnetpp",
    "perlbench",
    "povray",
    "sjeng",
    "soplex",
    "sphinx3",
    "tonto",
    "wrf",
    "xalancbmk",
    "zeusmp",
]

# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────

def zone_slice(zone: int) -> list:
    """Return the benchmark sub-list for the given zone (1-4)."""
    n = len(BENCHMARKS)
    size = math.ceil(n / 4)
    start = (zone - 1) * size
    end = min(zone * size, n)
    return BENCHMARKS[start:end]


def pid_file(zone: int) -> str:
    return os.path.join(PID_DIR, f"zone{zone}.pid")


def info_file(zone: int) -> str:
    return os.path.join(PID_DIR, f"zone{zone}.json")


def ensure_dirs():
    os.makedirs(PID_DIR, exist_ok=True)


def pinball_path(name: str) -> str:
    return os.path.join(PINBALL_BASE, f"cpu2006-{name}-ref-1", "pinball")


def build_cmd(name: str) -> list:
    """Build the run-sniper command for a single benchmark."""
    pb = pinball_path(name)
    pinballs = ",".join([pb] * NUM_CORES)
    return [
        RUN_SNIPER,
        "-v",
        "-s", "memTherm_core",
        "-c", "gainestown_2_5D_16core",
        "-n", str(NUM_CORES),
        "-v",
        "--sim-end=last",
        "-g", f"--traceinput/benchmarks=spec-{name}-native-1",
        f"--pinballs={pinballs}",
    ]


def daemonise():
    """Classic double-fork to fully detach from the terminal."""
    # First fork
    pid = os.fork()
    if pid > 0:
        # Parent returns — the caller will exit cleanly
        return False  # signal "I am the parent"

    # First child — new session
    os.setsid()

    # Second fork — prevent re-acquiring a controlling terminal
    pid = os.fork()
    if pid > 0:
        os._exit(0)

    # Grandchild — the actual daemon
    # Redirect std streams to /dev/null
    devnull = os.open(os.devnull, os.O_RDWR)
    os.dup2(devnull, 0)
    os.dup2(devnull, 1)
    os.dup2(devnull, 2)
    os.close(devnull)

    return True  # signal "I am the daemon"


# ──────────────────────────────────────────────────────────────────────────────
# Commands
# ──────────────────────────────────────────────────────────────────────────────

def cmd_start(zone: int):
    benches = zone_slice(zone)
    if not benches:
        print(f"[daemon] Zone {zone} is empty (no benchmarks to run).")
        sys.exit(1)

    pf = pid_file(zone)
    if os.path.exists(pf):
        with open(pf) as f:
            old_pid = int(f.read().strip())
        try:
            os.kill(old_pid, 0)  # probe — does it still live?
            print(f"[daemon] Zone {zone} is already running (PID {old_pid}).")
            print("         Use 'stop' or 'kill' first.")
            sys.exit(1)
        except OSError:
            pass  # stale pid file, clean up below

    ensure_dirs()

    print(f"[daemon] Zone {zone} — {len(benches)} benchmarks:")
    for b in benches:
        print(f"           • {b}")
    print(f"[daemon] Daemonising …")

    is_daemon = daemonise()
    if not is_daemon:
        # Parent — give the daemon a moment, then exit
        time.sleep(0.3)
        print(f"[daemon] Daemon launched. Check status with: ./bench_daemon.py status {zone}")
        return

    # ── We are now the daemon process ──
    daemon_pid = os.getpid()
    with open(pf, "w") as f:
        f.write(str(daemon_pid))

    # Launch all benchmarks in parallel
    children = {}  # bench_name -> Popen
    for name in benches:
        out_dir = os.path.join(SPEC_DIR, name)
        os.makedirs(out_dir, exist_ok=True)
        log_path = os.path.join(SPEC_DIR, f"{name}.log")
        log_fd = open(log_path, "w")

        cmd = build_cmd(name)
        proc = subprocess.Popen(
            cmd,
            cwd=out_dir,
            stdout=log_fd,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setpgrp,  # own process group per benchmark
        )
        children[name] = {"proc": proc, "log_fd": log_fd}

    # Write info file (PIDs of each benchmark sub-process)
    info = {
        "daemon_pid": daemon_pid,
        "zone": zone,
        "benchmarks": {
            name: {
                "pid": c["proc"].pid,
                "log": os.path.join(SPEC_DIR, f"{name}.log"),
            }
            for name, c in children.items()
        },
    }
    with open(info_file(zone), "w") as f:
        json.dump(info, f, indent=2)

    # Wait for all children and collect exit codes, updating the info file
    # incrementally so that `status` can report per-benchmark results as
    # they finish (not only after every benchmark is done).
    results = {}
    remaining = dict(children)  # shallow copy: name -> {"proc", "log_fd"}

    while remaining:
        for name in list(remaining.keys()):
            rc = remaining[name]["proc"].poll()
            if rc is not None:
                results[name] = rc
                remaining[name]["log_fd"].close()
                del remaining[name]

                # Incrementally write results so `status` picks them up
                info["results"] = results
                with open(info_file(zone), "w") as f:
                    json.dump(info, f, indent=2)

        if remaining:
            time.sleep(2)  # avoid busy-wait

    # Mark the zone as finished
    info["results"] = results
    info["finished"] = True
    with open(info_file(zone), "w") as f:
        json.dump(info, f, indent=2)

    # Clean up PID file
    try:
        os.remove(pf)
    except OSError:
        pass

    os._exit(0)


def _read_zone_info(zone: int) -> Optional[Dict[str, Any]]:
    ifile = info_file(zone)
    if not os.path.exists(ifile):
        return None
    with open(ifile) as f:
        return json.load(f)


def _pid_alive(pid: int) -> bool:
    """Check whether a process is alive, not a zombie, and is actually ours.

    Guards against PID reuse on busy servers: after a benchmark is killed,
    its PID may be reassigned to an unrelated process.
    """
    try:
        os.kill(pid, 0)
    except OSError:
        return False

    # On Linux, verify the process is actually sniper-related and not a
    # recycled PID belonging to some other user/program.
    try:
        with open(f"/proc/{pid}/status") as f:
            for line in f:
                if line.startswith("State:"):
                    if "Z" in line.split(":")[1]:
                        return False  # zombie
                    break
    except (FileNotFoundError, PermissionError):
        pass

    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            cmdline = f.read().decode("utf-8", errors="replace")
        # /proc cmdline uses \x00 as separator
        cmdline_lower = cmdline.lower()
        # Check for any sniper/CoMeT/benchmark-related keyword
        markers = ("sniper", "pin", "sift", "run-sniper", "comet", "bench_daemon", "spec")
        if not any(m in cmdline_lower for m in markers):
            return False  # PID reused by an unrelated process
    except (FileNotFoundError, PermissionError):
        pass  # /proc not available or process vanished

    return True

def _check_sim_state(log_path: str) -> Optional[str]:
    """Read the tail of a benchmark log to detect simulation completion.

    run-sniper prints '[SNIPER] End' after the simulation finishes, then
    continues with post-processing (McPAT, gen_simout, …).  By looking
    for this marker we can tell whether the *simulation* has finished
    even though the run-sniper PID is still alive.

    Returns:
        'ended'    — simulation finished normally
        'error'    — simulation hit an error / crash
        None       — no marker found (still simulating or log too short)
    """
    try:
        with open(log_path, "rb") as f:
            # Read last 8 KB — markers are near the end
            f.seek(0, 2)
            size = f.tell()
            f.seek(max(0, size - 8192))
            tail = f.read().decode("utf-8", errors="ignore")
    except (FileNotFoundError, PermissionError):
        return None

    # Only check for markers that run-sniper itself prints
    if "[SNIPER] End" in tail:
        return "ended"
    if "[SNIPER] Error" in tail:
        return "error"
    return None


def cmd_status(zone: Optional[int]):
    zones = [zone] if zone else [1, 2, 3, 4]
    for z in zones:
        benches = zone_slice(z)
        pf = pid_file(z)
        info = _read_zone_info(z)

        # Determine daemon state
        daemon_alive = False
        if os.path.exists(pf):
            with open(pf) as f:
                dpid = int(f.read().strip())
            daemon_alive = _pid_alive(dpid)

        header = f"Zone {z}  ({len(benches)} benchmarks)"
        if daemon_alive:
            header += f"  [RUNNING — daemon PID {dpid}]"
        elif info and info.get("finished"):
            header += "  [FINISHED]"
        else:
            header += "  [STOPPED]"

        print(header)

        if info and "benchmarks" in info:
            for name, binfo in info["benchmarks"].items():
                rc = info.get("results", {}).get(name)
                alive = _pid_alive(binfo["pid"])
                log_path = binfo.get("log", os.path.join(SPEC_DIR, f"{name}.log"))
                sim_state = _check_sim_state(log_path)

                if rc is not None:
                    state = f"exit {rc}" + ("  ✔" if rc == 0 else "  ✘")
                elif alive and sim_state == "ended":
                    state = "sim done, post-processing"
                elif alive and sim_state == "error":
                    state = "sim error, post-processing"
                elif alive:
                    state = "running"
                elif sim_state == "ended":
                    state = "sim done (process exited)"
                elif sim_state == "error":
                    state = "sim error (process exited)"
                else:
                    state = "exited (no code recorded)"
                print(f"    {name:20s}  PID {binfo['pid']:>8}  {state}")
        else:
            for b in benches:
                print(f"    {b:20s}  (not started)")
        print()


def _signal_zone(zone: int, sig: int, sig_name: str):
    pf = pid_file(zone)
    if not os.path.exists(pf):
        print(f"[daemon] Zone {zone}: no PID file found (not running?).")
        return

    with open(pf) as f:
        dpid = int(f.read().strip())

    if not _pid_alive(dpid):
        print(f"[daemon] Zone {zone}: daemon PID {dpid} is not running (stale).")
        try:
            os.remove(pf)
        except OSError:
            pass
        return

    # Kill the entire daemon process group — this takes out all children too
    info = _read_zone_info(zone)
    killed = []

    # Kill individual benchmark processes first
    if info and "benchmarks" in info:
        for name, binfo in info["benchmarks"].items():
            bpid = binfo["pid"]
            if _pid_alive(bpid):
                try:
                    os.killpg(os.getpgid(bpid), sig)
                    killed.append(name)
                except (OSError, ProcessLookupError):
                    pass

    # Kill the daemon itself
    try:
        os.kill(dpid, sig)
    except OSError:
        pass

    try:
        os.remove(pf)
    except OSError:
        pass

    if killed:
        print(f"[daemon] Zone {zone}: sent {sig_name} to daemon (PID {dpid}) + {len(killed)} benchmark(s).")
    else:
        print(f"[daemon] Zone {zone}: sent {sig_name} to daemon (PID {dpid}).")


def cmd_stop(zone: Optional[int]):
    zones = [zone] if zone else [1, 2, 3, 4]
    for z in zones:
        _signal_zone(z, signal.SIGTERM, "SIGTERM")


def cmd_kill(zone: Optional[int]):
    zones = [zone] if zone else [1, 2, 3, 4]
    for z in zones:
        _signal_zone(z, signal.SIGKILL, "SIGKILL")


# ──────────────────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────────────────────

USAGE = """\
Usage:
    bench_daemon.py start  <zone>        Start benchmarks for zone (1-4)
    bench_daemon.py stop   [zone]        Graceful stop  (SIGTERM)
    bench_daemon.py kill   [zone]        Forceful stop  (SIGKILL)
    bench_daemon.py status [zone]        Show running state
    bench_daemon.py list                 Show zone → benchmark mapping
"""


def cmd_list():
    for z in range(1, 5):
        benches = zone_slice(z)
        print(f"Zone {z}  ({len(benches)} benchmarks):")
        for b in benches:
            print(f"    • {b}")
        print()


def main():
    if len(sys.argv) < 2:
        print(USAGE)
        sys.exit(1)

    action = sys.argv[1].lower()

    if action == "list":
        cmd_list()
        return

    if action == "start":
        if len(sys.argv) < 3:
            print("Error: 'start' requires a zone number (1-4).")
            print(USAGE)
            sys.exit(1)
        zone = int(sys.argv[2])
        if zone not in (1, 2, 3, 4):
            print("Error: zone must be 1, 2, 3 or 4.")
            sys.exit(1)
        cmd_start(zone)

    elif action == "status":
        zone = int(sys.argv[2]) if len(sys.argv) >= 3 else None
        cmd_status(zone)

    elif action == "stop":
        zone = int(sys.argv[2]) if len(sys.argv) >= 3 else None
        cmd_stop(zone)

    elif action == "kill":
        zone = int(sys.argv[2]) if len(sys.argv) >= 3 else None
        cmd_kill(zone)

    else:
        print(f"Unknown action: {action}")
        print(USAGE)
        sys.exit(1)


if __name__ == "__main__":
    main()
