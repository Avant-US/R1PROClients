#!/usr/bin/env python3
"""
Analyze monitor CSV to diagnose why the robot is not moving.

Usage:
  python3 scripts/analyze_monitor.py /tmp/monitor/monitor_YYYYMMDD_HHMMSS.csv
"""

import csv
import sys
from collections import defaultdict
from pathlib import Path
import numpy as np


def load_csv(path: Path):
    records = defaultdict(list)
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            tag = row["tag"]
            vals = [float(x) for x in row["values"].split(",") if x]
            t = float(row["elapsed_s"])
            records[tag].append((t, vals))
    return records


def analyze(records: dict):
    print("=" * 70)
    print("MONITOR ANALYSIS")
    print("=" * 70)

    all_tags = sorted(records.keys())
    cmd_tags = [t for t in all_tags if t.startswith("cmd_")]
    fb_tags = [t for t in all_tags if t.startswith("fb_")]

    # --- 1. Message counts ---
    print("\n[1] Message counts:")
    for tag in all_tags:
        entries = records[tag]
        if len(entries) >= 2:
            dt = entries[-1][0] - entries[0][0]
            hz = len(entries) / dt if dt > 0 else 0
        else:
            hz = 0
        print(f"  {tag:22s}  count={len(entries):5d}  ~{hz:.1f} Hz")

    # --- 2. Action command check ---
    print("\n[2] Action commands published?")
    for tag in cmd_tags:
        entries = records[tag]
        if not entries:
            print(f"  {tag:22s}  ❌ NEVER published")
            continue
        vals = np.array([e[1] for e in entries])
        val_range = vals.max(axis=0) - vals.min(axis=0)
        first = vals[0]
        last = vals[-1]
        print(f"  {tag:22s}  ✅ {len(entries)} msgs")
        print(f"    first = [{', '.join(f'{v:+.4f}' for v in first)}]")
        print(f"    last  = [{', '.join(f'{v:+.4f}' for v in last)}]")
        print(f"    range = [{', '.join(f'{v:.4f}' for v in val_range)}]")

    # --- 3. Feedback change check ---
    print("\n[3] Feedback changed?")
    for tag in fb_tags:
        entries = records[tag]
        if not entries:
            print(f"  {tag:22s}  ❌ No feedback data")
            continue
        vals = np.array([e[1] for e in entries])
        val_range = vals.max(axis=0) - vals.min(axis=0)
        max_change = val_range.max()
        first = vals[0]
        last = vals[-1]
        moved = "✅ MOVED" if max_change > 0.01 else "❌ STATIC"
        print(f"  {tag:22s}  {moved}  max_change={max_change:.6f}")
        print(f"    first = [{', '.join(f'{v:+.4f}' for v in first)}]")
        print(f"    last  = [{', '.join(f'{v:+.4f}' for v in last)}]")

    # --- 4. Command vs feedback comparison ---
    print("\n[4] Command vs Feedback (first & last):")
    pairs = [
        ("cmd_left_arm",     "fb_left_arm"),
        ("cmd_right_arm",    "fb_right_arm"),
        ("cmd_left_gripper", "fb_left_gripper"),
        ("cmd_right_gripper","fb_right_gripper"),
        ("cmd_torso",        "fb_torso"),
        ("cmd_chassis",      "fb_chassis"),
    ]
    for cmd_tag, fb_tag in pairs:
        cmd_entries = records.get(cmd_tag, [])
        fb_entries = records.get(fb_tag, [])
        if not cmd_entries or not fb_entries:
            print(f"  {cmd_tag} vs {fb_tag}: insufficient data")
            continue
        cmd_first = np.array(cmd_entries[0][1])
        cmd_last = np.array(cmd_entries[-1][1])
        fb_first = np.array(fb_entries[0][1])
        fb_last = np.array(fb_entries[-1][1])
        n = min(len(cmd_first), len(fb_first))
        diff_first = np.abs(cmd_first[:n] - fb_first[:n])
        diff_last = np.abs(cmd_last[:n] - fb_last[:n])
        print(f"  {cmd_tag} vs {fb_tag}:")
        print(f"    cmd_first = [{', '.join(f'{v:+.6f}' for v in cmd_first)}]")
        print(f"    fb_first  = [{', '.join(f'{v:+.6f}' for v in fb_first)}]")
        print(f"    |diff|    = [{', '.join(f'{v:.6f}' for v in diff_first)}]")
        print(f"    cmd_last  = [{', '.join(f'{v:+.6f}' for v in cmd_last)}]")
        print(f"    fb_last   = [{', '.join(f'{v:+.6f}' for v in fb_last)}]")
        print(f"    |diff|    = [{', '.join(f'{v:.6f}' for v in diff_last)}]")

    # --- 5. Timing gaps ---
    print("\n[5] Command timing (gaps between consecutive messages):")
    for tag in cmd_tags:
        entries = records[tag]
        if len(entries) < 2:
            print(f"  {tag:22s}  not enough data")
            continue
        times = [e[0] for e in entries]
        gaps = np.diff(times)
        print(f"  {tag:22s}  gap: mean={gaps.mean()*1000:.1f}ms  "
              f"min={gaps.min()*1000:.1f}ms  max={gaps.max()*1000:.1f}ms  "
              f"std={gaps.std()*1000:.1f}ms")


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <monitor_csv_file>")
        sys.exit(1)
    path = Path(sys.argv[1])
    if not path.exists():
        sys.exit(f"File not found: {path}")
    records = load_csv(path)
    if not records:
        sys.exit("No data in CSV")
    analyze(records)


if __name__ == "__main__":
    main()
