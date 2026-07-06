#!/usr/bin/env python3
"""ci_check.py — Pass/fail CI gate for a drone-follower run.

Usage:
    python3 tools/ci_check.py [--telemetry logs/telemetry.jsonl]
                              [--events logs/events.log]
                              [--min-alt 1.0] [--window 30]

Gate rules:
  1. Altitude floor. FAIL if, during the airborne phase, drone altitude ever
     drops below ``--min-alt`` (default 1.0 m). The airborne phase begins at
     the first telemetry row whose altitude exceeds min-alt (the pre-takeoff
     climb is ignored), and includes every row after it.
  2. Late errors. FAIL if there is ANY ERROR-level line in events.log within
     the final ``--window`` seconds (default 30) of the run. The run's end is
     the max telemetry ``t`` (mapped to wall-clock via the last telemetry
     ``wall``) or, if telemetry is unavailable, the max events timestamp.

Exit code: 0 on PASS, 1 on FAIL.
"""

import sys
import os
import json
import argparse
from datetime import datetime, timedelta


def parse_timestamp(raw):
    """Parse an ISO-8601 timestamp, returning a datetime or None."""
    try:
        return datetime.fromisoformat(raw)
    except (ValueError, TypeError):
        return None


def load_telemetry(path):
    """Return list of telemetry dicts (blank/malformed lines skipped)."""
    rows = []
    if not os.path.isfile(path):
        return rows
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return rows


def load_events(path):
    """Return list of (datetime, level, component, message) tuples.

    Malformed lines are skipped.
    """
    events = []
    if not os.path.isfile(path):
        return events
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            parts = [p.strip() for p in line.split("|", 3)]
            if len(parts) != 4:
                continue
            ts = parse_timestamp(parts[0])
            if ts is None:
                continue
            events.append((ts, parts[1].upper(), parts[2], parts[3]))
    return events


def check_altitude_floor(rows, min_alt):
    """Return (ok, detail_dict) for the altitude-floor rule."""
    # Find takeoff index: first row where drone_z first exceeds min_alt.
    airborne_start = None
    for i, row in enumerate(rows):
        z = row.get("drone_z")
        if z is not None and z > min_alt:
            airborne_start = i
            break

    detail = {
        "min_alt": min_alt,
        "airborne_start_index": airborne_start,
        "airborne_start_t": None,
        "min_airborne_alt": None,
        "min_airborne_t": None,
        "violations": 0,
    }

    if airborne_start is None:
        # Never took off -> the run never demonstrated flight. The spec gate is
        # "drone stayed above 1 m", so a run that never climbed above min-alt (or
        # produced no telemetry at all) is a FAIL, not a vacuous pass.
        detail["note"] = "drone never climbed above min-alt; no airborne phase -> FAIL"
        return False, detail

    detail["airborne_start_t"] = rows[airborne_start].get("t")

    worst_z = None
    worst_t = None
    violations = 0
    for row in rows[airborne_start:]:
        z = row.get("drone_z")
        if z is None:
            continue
        if worst_z is None or z < worst_z:
            worst_z = z
            worst_t = row.get("t")
        if z < min_alt:
            violations += 1

    detail["min_airborne_alt"] = worst_z
    detail["min_airborne_t"] = worst_t
    detail["violations"] = violations
    return violations == 0, detail


def determine_run_end(rows, events):
    """Return (end_dt, source_str) — wall-clock end time of the run.

    Use the LATER of (a) the max-``t`` telemetry row's ``wall`` time and (b) the
    max events timestamp. Taking the max matters: an ERROR logged AFTER the last
    telemetry sample (e.g. at shutdown, once telemetry_logger has stopped writing)
    would otherwise be past ``run_end`` and slip through the late-error gate.
    """
    best_t = None
    best_wall = None
    for row in rows:
        t = row.get("t")
        wall = parse_timestamp(row.get("wall"))
        if t is None or wall is None:
            continue
        if best_t is None or t > best_t:
            best_t = t
            best_wall = wall
    events_end = max((e[0] for e in events), default=None)

    candidates = [w for w in (best_wall, events_end) if w is not None]
    if not candidates:
        return None, "none"
    end = max(candidates)
    src = []
    if best_wall is not None:
        src.append("telemetry t={:.2f}".format(best_t))
    if events_end is not None:
        src.append("events")
    return end, "max(" + ", ".join(src) + ")"


def check_late_errors(events, run_end, window):
    """Return (ok, detail_dict) for the late-error rule."""
    detail = {
        "window": window,
        "run_end": run_end.isoformat() if run_end else None,
        "cutoff": None,
        "late_errors": [],
    }
    if run_end is None:
        detail["note"] = "could not determine run end; skipping late-error check"
        return True, detail

    cutoff = run_end - timedelta(seconds=window)
    detail["cutoff"] = cutoff.isoformat()

    late = []
    for ts, level, component, message in events:
        if level == "ERROR" and cutoff <= ts <= run_end:
            late.append((ts, component, message))

    detail["late_errors"] = [
        {"ts": ts.isoformat(), "component": c, "message": m}
        for ts, c, m in late
    ]
    return len(late) == 0, detail


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--telemetry", default="logs/telemetry.jsonl")
    parser.add_argument("--events", default="logs/events.log")
    parser.add_argument("--min-alt", type=float, default=1.0)
    parser.add_argument("--window", type=float, default=30.0)
    args = parser.parse_args(argv[1:])

    rows = load_telemetry(args.telemetry)
    events = load_events(args.events)

    print("=" * 62)
    print("CI CHECK — drone-follower run gate")
    print("=" * 62)
    print("  telemetry : {}  ({} rows)".format(args.telemetry, len(rows)))
    print("  events    : {}  ({} lines)".format(args.events, len(events)))
    print("  min-alt   : {} m".format(args.min_alt))
    print("  window    : {} s".format(args.window))
    if not rows:
        print("  !! no telemetry rows — the run produced no flight data (FAIL)")
    print("-" * 62)

    # Rule 1: altitude floor.
    alt_ok, alt = check_altitude_floor(rows, args.min_alt)
    print("[Rule 1] Altitude floor during airborne phase")
    if alt["airborne_start_index"] is None:
        print("         {}".format(alt.get("note", "no airborne phase")))
    else:
        print("         airborne starts at t={:.2f}s (index {})".format(
            alt["airborne_start_t"], alt["airborne_start_index"]))
        print("         min airborne altitude = {:.3f} m at t={:.2f}s".format(
            alt["min_airborne_alt"], alt["min_airborne_t"]))
        print("         violations below {} m: {}".format(
            args.min_alt, alt["violations"]))
    print("         -> {}".format("PASS" if alt_ok else "FAIL"))
    print("-" * 62)

    # Rule 2: late errors.
    run_end, src = determine_run_end(rows, events)
    err_ok, err = check_late_errors(events, run_end, args.window)
    print("[Rule 2] No ERROR within final {}s".format(args.window))
    print("         run end : {} ({})".format(err["run_end"], src))
    print("         cutoff  : {}".format(err["cutoff"]))
    if err["late_errors"]:
        print("         late ERROR lines found: {}".format(
            len(err["late_errors"])))
        for e in err["late_errors"]:
            print("           {} | {} | {}".format(
                e["ts"], e["component"], e["message"]))
    else:
        print("         late ERROR lines found: 0")
    print("         -> {}".format("PASS" if err_ok else "FAIL"))
    print("-" * 62)

    overall = alt_ok and err_ok
    print("OVERALL: {}".format("PASS" if overall else "FAIL"))
    print("=" * 62)
    return 0 if overall else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
