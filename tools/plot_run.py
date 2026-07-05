#!/usr/bin/env python3
"""plot_run.py — Generate diagnostic plots from a telemetry.jsonl run.

Usage:
    python3 tools/plot_run.py [<path-to-telemetry.jsonl>] [--out <dir>]

Defaults: input ``sample_logs/telemetry.jsonl``, output dir ``run_logs/plots``.

telemetry.jsonl is JSON Lines, one object per control tick, e.g.:
    {"t": 12.34, "wall": "...", "car_x": 1.2, "car_y": 3.4,
     "drone_x": 0.9, "drone_y": 2.8, "drone_z": 19.7,
     "rtf": 0.96, "msg_dt_ms": 33.1}

Any field may be null; such points are skipped for the affected plot.

Four PNGs are written:
    1. path_xy.png   drone XY path vs car XY path (equal aspect, start markers)
    2. msg_rate.png  message arrival rate (Hz) vs time, from 1000/msg_dt_ms
    3. rtf.png       real-time factor vs time, with a 0.8 reference line
    4. altitude.png  drone altitude (drone_z) vs time, with a 1.0 reference line

The three time-series plots use time-since-run-start on the x-axis (the logged
``t`` is absolute clock seconds, so we zero it to the first sample).
"""

import sys
import os
import json
import argparse

import matplotlib

# Non-interactive backend: required for headless / CI environments.
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402  (must follow backend selection)


def load_rows(path):
    """Load telemetry rows from a JSONL file.

    Returns (rows, skipped) where rows is a list of dicts and skipped is the
    count of blank/malformed lines that could not be parsed.
    """
    rows = []
    skipped = 0
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                skipped += 1
    return rows, skipped


def paired(rows, *keys):
    """Yield tuples of the requested keys for rows where ALL keys are present
    and non-null. Used to skip missing/null data points cleanly.
    """
    xs = [[] for _ in keys]
    for row in rows:
        vals = [row.get(k) for k in keys]
        if any(v is None for v in vals):
            continue
        for i, v in enumerate(vals):
            xs[i].append(v)
    return xs


def plot_path_xy(rows, out_path, t0=0.0):
    """Drone XY path vs car XY path."""
    drone_x, drone_y = paired(rows, "drone_x", "drone_y")
    car_x, car_y = paired(rows, "car_x", "car_y")

    fig, ax = plt.subplots(figsize=(7, 7))
    if car_x:
        ax.plot(car_x, car_y, "-", color="tab:orange", label="car", lw=1.5)
        ax.plot(car_x[0], car_y[0], "o", color="tab:orange",
                markersize=10, label="car start")
    if drone_x:
        ax.plot(drone_x, drone_y, "-", color="tab:blue", label="drone", lw=1.5)
        ax.plot(drone_x[0], drone_y[0], "s", color="tab:blue",
                markersize=10, label="drone start")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Drone vs Car XY path")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def plot_msg_rate(rows, out_path, t0=0.0):
    """Message arrival rate (Hz) over time, derived from msg_dt_ms."""
    t, dt = paired(rows, "t", "msg_dt_ms")
    # rate = 1000 / msg_dt_ms; guard against zero/negative gaps.
    ts, rate = [], []
    for ti, dti in zip(t, dt):
        if dti and dti > 0:
            ts.append(ti - t0)
            rate.append(1000.0 / dti)

    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(ts, rate, "-", color="tab:green", lw=1.0)
    ax.set_xlabel("time since start (s)")
    ax.set_ylabel("message rate (Hz)")
    ax.set_title("/car/position arrival rate")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def plot_rtf(rows, out_path, t0=0.0):
    """Real-time factor over time with a 0.8 reference line."""
    t, rtf = paired(rows, "t", "rtf")
    t = [ti - t0 for ti in t]

    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(t, rtf, "-", color="tab:purple", lw=1.0, label="rtf")
    ax.axhline(0.8, color="red", linestyle="--", lw=1.0,
               label="0.8 threshold")
    ax.set_xlabel("time since start (s)")
    ax.set_ylabel("real-time factor")
    ax.set_title("Gazebo real-time factor")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def plot_altitude(rows, out_path, t0=0.0):
    """Drone altitude over time with a 1.0 reference line."""
    t, z = paired(rows, "t", "drone_z")
    t = [ti - t0 for ti in t]

    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(t, z, "-", color="tab:blue", lw=1.0, label="drone_z")
    ax.axhline(1.0, color="red", linestyle="--", lw=1.0,
               label="1.0 m floor")
    ax.set_xlabel("time since start (s)")
    ax.set_ylabel("altitude drone_z (m)")
    ax.set_title("Drone altitude")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("telemetry", nargs="?",
                        default="sample_logs/telemetry.jsonl",
                        help="path to telemetry.jsonl")
    parser.add_argument("--out", default="run_logs/plots",
                        help="output directory for PNGs")
    args = parser.parse_args(argv[1:])

    if not os.path.isfile(args.telemetry):
        print("ERROR: telemetry file not found: {}".format(args.telemetry))
        return 1

    rows, skipped = load_rows(args.telemetry)
    print("Loaded {} telemetry rows ({} malformed lines skipped)".format(
        len(rows), skipped))

    os.makedirs(args.out, exist_ok=True)

    # Zero the time axis to the run start so the time-series plots read in
    # seconds-since-start rather than absolute clock epoch seconds.
    t0 = min((r["t"] for r in rows if r.get("t") is not None), default=0.0)

    targets = [
        ("path_xy.png", plot_path_xy),
        ("msg_rate.png", plot_msg_rate),
        ("rtf.png", plot_rtf),
        ("altitude.png", plot_altitude),
    ]

    written = []
    for name, fn in targets:
        out_path = os.path.join(args.out, name)
        fn(rows, out_path, t0)
        written.append(out_path)

    print("Wrote {} plots:".format(len(written)))
    for p in written:
        print("  {}".format(p))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
