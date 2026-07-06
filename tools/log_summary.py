#!/usr/bin/env python3
"""log_summary.py — Summarize a drone-follower events.log file.

Usage:
    python3 tools/log_summary.py [<path-to-events.log>]

If no path is given, defaults to ``logs/events.log``.

events.log line contract (pipe-delimited):
    ISO8601_TIMESTAMP | LEVEL | component | message

LEVEL is one of INFO, WARNING, ERROR.

The script prints:
    * total warnings and total errors
    * unique error "types" (ERROR messages grouped after normalizing
      trailing digits/ids so "arm failed after 3 attempts" and
      "...after 2 attempts" collapse into one type) with a representative
      message and a count
    * first / last error timestamp and the wall-clock duration between them

It is robust to blank/malformed lines (they are skipped and counted) and
always exits 0.
"""

import sys
import re
from collections import OrderedDict
from datetime import datetime


def parse_timestamp(raw):
    """Parse an ISO-8601 timestamp string, returning a datetime or None.

    Python 3.7+ ``datetime.fromisoformat`` handles the "+05:30" style
    offset our logs use. We fall back to None on anything unexpected.
    """
    try:
        return datetime.fromisoformat(raw)
    except (ValueError, TypeError):
        return None


def normalize_message(message):
    """Collapse an ERROR message into a "type" key.

    We strip standalone numbers (attempt counts, ids, etc.) so that
    otherwise-identical messages group together. E.g.
        "arm failed after 3 attempts" -> "arm failed after <n> attempts"
    """
    # Replace any run of digits with a placeholder token.
    return re.sub(r"\d+", "<n>", message).strip()


def format_duration(seconds):
    """Render a duration in seconds as a compact human string."""
    if seconds < 60:
        return "{:.3f} s".format(seconds)
    minutes, secs = divmod(seconds, 60)
    if minutes < 60:
        return "{:d} m {:.1f} s".format(int(minutes), secs)
    hours, minutes = divmod(int(minutes), 60)
    return "{:d} h {:d} m {:.1f} s".format(hours, minutes, secs)


def summarize(path):
    """Parse the log at ``path`` and return a summary dict."""
    warnings = 0
    errors = 0
    skipped = 0
    # Preserve first-seen order for stable, readable output.
    error_types = OrderedDict()  # normalized_key -> {"count", "sample"}
    error_timestamps = []

    try:
        with open(path, "r", encoding="utf-8") as fh:
            lines = fh.readlines()
    except OSError as exc:
        print("ERROR: could not read {}: {}".format(path, exc))
        return None

    for line in lines:
        line = line.strip()
        if not line:
            skipped += 1
            continue

        # Expect exactly 4 pipe-delimited fields. The message itself may
        # contain pipes, so split with maxsplit=3.
        parts = [p.strip() for p in line.split("|", 3)]
        if len(parts) != 4:
            skipped += 1
            continue

        ts_raw, level, _component, message = parts
        level = level.upper()

        if level == "WARNING":
            warnings += 1
        elif level == "ERROR":
            errors += 1
            key = normalize_message(message)
            if key not in error_types:
                error_types[key] = {"count": 0, "sample": message}
            error_types[key]["count"] += 1

            ts = parse_timestamp(ts_raw)
            if ts is not None:
                error_timestamps.append((ts, ts_raw))
            # If the timestamp is unparseable the line is still a valid ERROR (already
            # counted); it just can't contribute to the first/last-error span. We do
            # NOT count it as skipped/malformed -- that would double-count the line.
        elif level == "INFO":
            pass
        else:
            # Unknown level -> malformed.
            skipped += 1

    return {
        "warnings": warnings,
        "errors": errors,
        "skipped": skipped,
        "error_types": error_types,
        "error_timestamps": error_timestamps,
    }


def print_report(path, summary):
    """Pretty-print the summary to stdout."""
    print("=" * 60)
    print("Event log summary: {}".format(path))
    print("=" * 60)
    print("  {:<26}{}".format("Total warnings:", summary["warnings"]))
    print("  {:<26}{}".format("Total errors:", summary["errors"]))
    print("  {:<26}{}".format("Skipped/malformed lines:", summary["skipped"]))
    print()

    error_types = summary["error_types"]
    print("Unique error types: {}".format(len(error_types)))
    if error_types:
        # Sort by count descending for readability.
        rows = sorted(
            error_types.items(), key=lambda kv: kv[1]["count"], reverse=True
        )
        print("  {:>5}  {}".format("count", "representative message"))
        print("  {:>5}  {}".format("-----", "-" * 40))
        for _key, info in rows:
            print("  {:>5}  {}".format(info["count"], info["sample"]))
    print()

    timestamps = summary["error_timestamps"]
    if timestamps:
        timestamps.sort(key=lambda t: t[0])
        first_dt, first_raw = timestamps[0]
        last_dt, last_raw = timestamps[-1]
        span = (last_dt - first_dt).total_seconds()
        print("First error: {}".format(first_raw))
        print("Last error:  {}".format(last_raw))
        print("Error span:  {}".format(format_duration(span)))
    else:
        print("First error: (none)")
        print("Last error:  (none)")
        print("Error span:  n/a")
    print("=" * 60)


def main(argv):
    path = argv[1] if len(argv) > 1 else "logs/events.log"
    summary = summarize(path)
    if summary is not None:
        print_report(path, summary)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
