"""Shared logging + telemetry helpers.

Two artifacts are produced, and the tools/ scripts consume exactly these:

  events.log      one line per event, pipe-delimited, human + machine readable:
                    ISO8601_TIMESTAMP | LEVEL | component | message
                  e.g.
                    2026-07-04T14:37:12.345678+05:30 | WARNING | follower | car_gap_ms=213 threshold=200 action=hover

  telemetry.jsonl one JSON object per line (JSON Lines), one per control tick:
                    {"t": <sim_seconds>, "wall": <iso>, "car_x":.., "car_y":..,
                     "drone_x":.., "drone_y":.., "drone_z":.., "rtf":.., "msg_dt_ms":..}

Keeping events (for log_summary.py) and telemetry (for plot_run.py) in separate
files means each tool has a dead-simple, unambiguous parser and neither can be
confused by the other's lines.
"""

import json
import logging
import os
from datetime import datetime, timezone
from logging.handlers import RotatingFileHandler


# Matches: ISO | LEVEL | component | message   (tools/log_summary.py mirrors this)
EVENT_DELIM = " | "


def _iso_now() -> str:
    """Local-time ISO8601 with timezone offset and microseconds."""
    return datetime.now(timezone.utc).astimezone().isoformat()


class _PipeFormatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:
        component = getattr(record, "component", record.name)
        ts = _iso_now()
        msg = record.getMessage()
        return EVENT_DELIM.join([ts, record.levelname, component, msg])


def get_event_logger(component: str, log_dir: str) -> logging.Logger:
    """Return a logger that writes pipe-delimited events to <log_dir>/events.log
    and echoes them to the console. Safe to call once per node."""
    os.makedirs(log_dir, exist_ok=True)
    logger = logging.getLogger(f"drone_system.{component}")
    logger.setLevel(logging.INFO)
    logger.propagate = False
    if logger.handlers:  # already configured (e.g. re-init)
        return logger

    fmt = _PipeFormatter()

    # Rotating so a long-running session cannot grow events.log without bound.
    # 10 MB x 3 files is far more than a normal run needs (a 60 s run is a few
    # KB) but caps the pathological case. Nodes append; each runs in its own
    # process, so there is no truncation race on the shared file.
    fh = RotatingFileHandler(
        os.path.join(log_dir, "events.log"),
        maxBytes=10 * 1024 * 1024, backupCount=2)
    fh.setFormatter(fmt)
    logger.addHandler(fh)

    ch = logging.StreamHandler()
    ch.setFormatter(fmt)
    logger.addHandler(ch)

    # Bind the component name onto every record from this logger.
    logging.setLogRecordFactory(_component_factory(component))
    return logger


def _component_factory(component: str):
    old = logging.getLogRecordFactory()

    def factory(*args, **kwargs):
        record = old(*args, **kwargs)
        if not hasattr(record, "component"):
            record.component = component
        return record

    return factory


class TelemetryWriter:
    """Single-writer JSON Lines telemetry sink consumed by tools/plot_run.py.

    Opens fresh each run (mode 'w'), so telemetry never concatenates across
    runs, and enforces a hard size cap so a forgotten long-running session can't
    fill the disk. Only ONE process (telemetry_logger) ever writes this file, so
    the truncate-on-open is safe.
    """

    DEFAULT_MAX_MB = 128.0

    def __init__(self, log_dir: str, filename: str = "telemetry.jsonl",
                 max_mb: float = DEFAULT_MAX_MB):
        os.makedirs(log_dir, exist_ok=True)
        self._path = os.path.join(log_dir, filename)
        self._fh = open(self._path, "w", buffering=1)  # fresh per run, line-buffered
        self._max_bytes = int(max_mb * 1024 * 1024)
        self._bytes = 0
        self._capped = False

    @property
    def path(self) -> str:
        return self._path

    def write(self, **fields) -> None:
        if self._capped:
            return
        fields.setdefault("wall", _iso_now())
        line = json.dumps(fields) + "\n"
        self._fh.write(line)
        self._bytes += len(line)
        if self._bytes >= self._max_bytes:
            self._capped = True
            self._fh.write(
                json.dumps({"note": "telemetry size cap reached; logging stopped"})
                + "\n")

    def close(self) -> None:
        try:
            self._fh.close()
        except Exception:
            pass
