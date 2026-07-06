# drone_system — a PX4/Gazebo drone that follows a car (ROS2)

A ROS2 system where a simulated **PX4 SITL drone** takes off and **follows a
simulated car** at a fixed trailing offset. Built for the Invictron Robotics
System Engineer assessment.

Everything — the PX4↔ROS2 bridge, PX4 SITL, Gazebo, and all five ROS2 nodes —
comes up from **one command**.

```bash
ros2 launch drone_system full_stack.launch.py
```

> **Why Docker?** "It Runs" is the non-negotiable criterion, and it must run on
> *your* machine, not just mine. So the entire stack is pinned inside a Docker
> image (Ubuntu 22.04 + ROS2 Humble + PX4 v1.15 + Gazebo). Your host distro is
> irrelevant. Build the image once; the launch command above then runs inside it.

---

## Prerequisites

- **Docker Engine** on a Linux host. Either add your user to the `docker` group
  (`sudo usermod -aG docker $USER && newgrp docker`) or run the scripts with
  `sudo`.
- **First run pulls a pre-built image** from GHCR
  (`ghcr.io/bhaveshbakshi633/drone_system`) — no compile, just a ~6 GB download.
  If that pull is unavailable it falls back to **building from source** (compiles
  PX4, ~20–30 min). Either way: internet, ~10 GB free disk, ≥8 GB RAM.
- Nothing else is needed to *run* it — ROS2, PX4, and Gazebo all live in the
  image.
- To run the host-side tools (`tools/plot_run.py`) *outside* the container you
  also need Python 3 + matplotlib (`pip install -r requirements.txt`); inside
  the image they are already installed.

## Quick start

```bash
git clone <this-repo-url> invictron-drone-follower
cd invictron-drone-follower

# One command from a fresh clone: pulls the pre-built image (no compile; falls
# back to a source build only if the pull is unavailable), then runs the 60-second
# headless integration test. Works on ANY Linux host with Docker — no X server
# required — and prints a PASS/FAIL gate at the end.
./scripts/run.sh
```

That is the portable "proof it runs". Under the hood it just gets the image and
runs the gate — the two steps you can also run by hand:

```bash
# 1) Get the image — pull the pre-built one (no compile)…
docker pull ghcr.io/bhaveshbakshi633/drone_system:latest
#    …or build it from source instead (compiles PX4, ~20–30 min, once):
#    ./scripts/build_image.sh

# 2) Run the exact 60-second headless integration test + pass/fail gate.
docker run --rm -v "$(pwd)/run_logs:/root/run/logs" \
    ghcr.io/bhaveshbakshi633/drone_system:latest bash /root/scripts/run_ci.sh
```

### Optional: watch it fly (Gazebo GUI)

Needs a **Linux desktop with an X11 display** (the container draws to your X
server). On a headless/SSH box or a Wayland-only session, use the headless path
above instead.

```bash
GUI=1 ./scripts/run.sh        # or: ./scripts/run_local.sh
```

Inside the built image — or on a native ROS2 Humble + PX4 v1.15 + `px4_msgs`
host (see [Native setup](#native-setup)) — the single launch command is:

```bash
ros2 launch drone_system full_stack.launch.py                          # headless
ros2 launch drone_system full_stack.launch.py headless:=0 car_viz:=1   # GUI + visible car box
```

`car_viz:=1` spawns a visible box in the Gazebo GUI that tracks the car's
`/car/position` (pure visualization — the follower still reads only the topic).

---

## What it does

1. `car_sim` publishes a scripted **figure-8** trajectory on `/car/position` (the
   follower's only car input). `car_viz` *optionally* spawns a matching box in the
   Gazebo GUI — pure visualization; the follower never reads it.
2. `follower` places the waypoint **5 m** behind the car at **20 m** altitude,
   emitting `/drone/waypoint` at **50 Hz** (with an optional `lead_time_s`
   look-ahead, off by default). This is the core node.
3. `px4_interface` arms the drone (**retry 3×**), takes off to **20 m**, then
   streams **OFFBOARD** setpoints to PX4 — position **plus a velocity feed-forward**
   (finite-diff of the waypoint stream) so the controller leads the moving target
   instead of trailing it.
4. `health_monitor` watches the Gazebo real-time factor.
5. `telemetry_logger` records everything for the plots.

### Topics

| Topic | Type | Producer → Consumer |
|---|---|---|
| `/car/position` | `geometry_msgs/PoseStamped` | car_sim → follower, telemetry_logger |
| `/drone/waypoint` | `geometry_msgs/PoseStamped` | follower → px4_interface |
| `/fmu/in/*`, `/fmu/out/*` | `px4_msgs/*` | px4_interface ↔ PX4 (uXRCE-DDS) |
| `/health/rtf` | `std_msgs/Float32` | health_monitor → telemetry_logger |

The follower reads **only** `/car/position` — never Gazebo ground-truth or car
model parameters, per the spec.

Frames: ROS is **ENU**, PX4 is **NED**. The conversion lives entirely inside
`px4_interface` (`x_ned=y_enu, y_ned=x_enu, z_ned=-z_enu`).

---

## Failure handling

Every threshold lives in [`drone_system/config/params.yaml`](drone_system/config/params.yaml).
Nothing is hardcoded — edit, relaunch, done.

| Failure | Detection | Response |
|---|---|---|
| `/car/position` stops (> 200 ms gap) | `follower` timer vs last accepted stamp | Drone **hovers**; WARNING logged w/ ISO timestamp |
| PX4 fails to arm | `px4_interface` checks `arming_state` | **Retry 3×**, then ERROR + **clean shutdown** (disarm, tear down launch) |
| Position jumps > 5 m in one step | `follower` step-distance gate | **Discard** sample, hold last valid, WARNING logged |
| Gazebo RTF < 0.8 | `health_monitor` compares PX4 lockstep sim-time vs a wall clock | WARNING **every 5 s** until it recovers |

A nominal 60 s run trips none of these (no gap, no jump, arm succeeds first try, RTF
healthy) — so the recovery code never *fires* in a clean log. Each path is reproducible
on demand so you can watch the recovery happen against a live stack:

```bash
scripts/demo_failures.sh car_gap   # kills /car/position   -> follower hovers + WARN
scripts/demo_failures.sh jump      # injects a >5 m teleport -> discard + hold + WARN
scripts/demo_failures.sh rtf       # pegs all CPUs -> RTF < 0.8 -> warning every 5 s
scripts/demo_failures.sh arm       # (relaunch) force_arm_fail_n -> 3 arm retries -> clean shutdown
# then: python3 tools/log_summary.py run_logs/events.log
```

---

## Logging

Two artifacts under `run_logs/` (or `logs/` when run natively):

- **`events.log`** — one event per line, `ISO_TIMESTAMP | LEVEL | component | message`.
  Every failure event carries a timestamp, severity, component, and plain-English
  description.
- **`telemetry.jsonl`** — per-tick JSON (car/drone XY, altitude, RTF, message gap).

---

## Tools

```bash
# Summary of a run: warnings, errors, unique error types, first/last error.
python3 tools/log_summary.py run_logs/events.log

# Four plots -> run_logs/plots/  (drone-vs-car path, msg rate, RTF, altitude)
python3 tools/plot_run.py run_logs/telemetry.jsonl --out run_logs/plots

# Pass/fail gate used by CI: altitude > 1 m, no errors in the final 30 s.
python3 tools/ci_check.py --telemetry run_logs/telemetry.jsonl --events run_logs/events.log
```

`sample_logs/` contains a nominal run (for trying the tools without flying) and a
`failing/` run that demonstrates the CI gate rejecting a bad flight.

---

## Continuous integration

[`.github/workflows/integration_test.yml`](.github/workflows/integration_test.yml)
builds the Docker image, runs the stack headless for 60 s, asserts the drone
stayed above 1 m with no errors in the final 30 s (`ci_check.py`), and uploads
all logs + plots as artifacts.

> The workflow runs two lanes: a fast **`tools`** job (every push/PR) that
> unit-checks `log_summary.py`, `ci_check.py` (nominal passes, the failing sample
> is rejected) and `plot_run.py` against the committed `sample_logs/`; and the
> heavy **`sitl-integration`** job (run on demand via *Actions → Run workflow*)
> that builds the image and runs the real 60 s flight. It compiles PX4 (slow,
> layer-cached) and is best on a capable or self-hosted runner — it fails loudly
> rather than falsely green if the drone never actually flies.

---

## Configuration

All tunables — offset distance, altitude, timeouts, jump gate, arm retries, RTF
threshold — are in `drone_system/config/params.yaml`. Override the whole file:

```bash
ros2 launch drone_system full_stack.launch.py params:=/path/to/my_params.yaml
```

Launch arguments: `headless:=1|0`, `car_viz:=1` (visible car box in the GUI),
`car_viz_world:=<gz world>` (default `default`), `params:=<path>`, `px4_dir:=<path>`.

---

## Native setup
<a name="native-setup"></a>

Requires Ubuntu 22.04, ROS2 Humble, and — built and reachable — PX4-Autopilot
v1.15 (`make px4_sitl_default`), `px4_msgs` (branch `release/1.15`), the
`MicroXRCEAgent` binary on `PATH`, and `pip install -r requirements.txt` for the
host tools. Then:

```bash
# in a colcon workspace containing px4_msgs and this package:
colcon build && source install/setup.bash
export PX4_DIR=/path/to/PX4-Autopilot          # a *built* PX4 tree
ros2 launch drone_system full_stack.launch.py headless:=0
```

The [Dockerfile](docker/Dockerfile) is the exact, tested recipe for every one of
those steps if you'd rather mirror it by hand.

---

## Repository layout

```
drone_system/            ROS2 ament_python package
  drone_system/          car_sim, follower, follow_policy, px4_interface,
                         health_monitor, telemetry_logger, car_viz, logutil
  launch/full_stack.launch.py
  config/params.yaml     every threshold
  test/test_follow_policy.py
tools/                   log_summary.py, plot_run.py, ci_check.py
docker/                  Dockerfile, entrypoint.sh
scripts/                 run.sh, build_image.sh, run_local.sh, run_ci.sh, demo_failures.sh
sample_logs/             example nominal + failing runs
.github/workflows/       integration_test.yml, publish_image.yml
ANALYSIS.md              the four required design answers
```

See [`ANALYSIS.md`](ANALYSIS.md) for the four design questions.
