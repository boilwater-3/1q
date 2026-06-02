#!/usr/bin/env python3
"""Analyze takeoff-land CSV data with summary + optional plots.

Usage:
  python3 tools/analyze_takeoff.py /tmp/1q_csv/c172x.csv
  python3 tools/analyze_takeoff.py /tmp/1q_csv/*.csv
  python3 tools/analyze_takeoff.py --plot /tmp/1q_csv/c172x.csv
"""

import sys
import csv
import math
import os

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

try:
    import pandas as pd
    HAS_PD = True
except ImportError:
    HAS_PD = False


def load_csv(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                row = {}
                for k, v in r.items():
                    fv = float(v)
                    if not math.isfinite(fv):
                        return rows
                    row[k] = fv
                rows.append(row)
            except (ValueError, TypeError):
                return rows
    return rows


def plot(path, rows):
    if not HAS_MPL:
        print("  matplotlib not available, skipping plot")
        return
    if not HAS_PD:
        print("  pandas not available, skipping plot")
        return

    df = pd.DataFrame(rows)
    model = os.path.basename(path).replace(".csv", "")
    out = path.replace(".csv", ".png")

    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    t = df["time_sec"]

    # Altitude
    ax = axes[0]
    ax.plot(t, df["agl_m"], "b-", linewidth=0.5, alpha=0.8)
    ax.fill_between(t, 0, df["agl_m"], alpha=0.15, color="blue")
    ax.set_ylabel("AGL (m)")
    ax.set_title(f"{model} — Takeoff / Cruise / Landing")
    ax.axhline(y=0, color="black", linestyle="--", linewidth=0.5)
    ax.grid(True, alpha=0.3)

    # Speed
    ax = axes[1]
    ax.plot(t, df["vc_kts"], "r-", linewidth=0.5, alpha=0.8)
    ax.set_ylabel("Speed (kts)")
    ax.axhline(y=50, color="gray", linestyle=":", linewidth=0.5, label="Vr~50")
    ax.grid(True, alpha=0.3)

    # Attitude
    ax = axes[2]
    ax.plot(t, df["pitch_deg"], "g-", linewidth=0.5, alpha=0.7, label="pitch")
    ax.plot(t, df["roll_deg"], "orange", linewidth=0.5, alpha=0.5, label="roll")
    ax.set_ylabel("Attitude (°)")
    ax.legend(loc="upper right")
    ax.axhline(y=0, color="black", linestyle="--", linewidth=0.3)
    ax.grid(True, alpha=0.3)

    # Throttle + WOW
    ax = axes[3]
    ax.plot(t, df["throttle"], "purple", linewidth=0.5, alpha=0.7, label="throttle")
    ax.plot(t, df["wow"], "brown", linewidth=0.5, alpha=0.5, label="WOW")
    ax.set_ylabel("Throttle / WOW")
    ax.set_xlabel("Time (s)")
    ax.legend(loc="upper right")
    ax.set_ylim(-0.1, 1.1)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(out, dpi=100)
    plt.close()
    print(f"  plot saved: {out}")


def summarize(path, rows):
    model = os.path.basename(path).replace(".csv", "")
    n = len(rows)
    t0 = rows[0]["time_sec"]
    tn = rows[-1]["time_sec"]
    dur = tn - t0

    agl_vals = [r["agl_m"] for r in rows]
    kts_vals = [r["vc_kts"] for r in rows]
    pitch_vals = [r["pitch_deg"] for r in rows]
    roll_vals = [r["roll_deg"] for r in rows]
    throttle_vals = [r["throttle"] for r in rows]
    wow_vals = [r["wow"] for r in rows]

    max_agl = max(agl_vals)
    max_kts = max(kts_vals)
    min_agl = min(agl_vals)
    final_state = int(rows[-1]["fm_state"])
    state_name = {2: "executing", 3: "completed", 4: "aborted"}.get(final_state, "?")
    crashed = min_agl < -1

    # Detect phases from WOW transitions
    phases = []
    prev_wow = wow_vals[0]
    for i, r in enumerate(rows):
        w = r["wow"]
        if prev_wow > 0.5 and w < 0.5:
            phases.append(("liftoff", r["time_sec"], r["agl_m"], r["vc_kts"]))
        elif prev_wow < 0.5 and w > 0.5:
            phases.append(("touchdown", r["time_sec"], r["agl_m"], r["vc_kts"]))
        prev_wow = w

    flags = []
    if max_kts < 60:
        flags.append(f"Vmax={max_kts:.0f}kts (no rotation)")
    if max_agl < 50:
        flags.append(f"Hmax={max_agl:.0f}m (no climb)")
    if crashed:
        flags.append(f"crashed (Hmin={min_agl:.0f}m)")
    if max(roll_vals) > 60:
        flags.append(f"roll_max={max(roll_vals):.0f}°")
    if max(pitch_vals) > 60:
        flags.append(f"pitch_max={max(pitch_vals):.0f}°")

    print(f"  {model:<6} {dur:6.0f}s  H={max_agl:5.0f}m  V={max_kts:5.0f}kts"
          f"  θ={max(pitch_vals):4.0f}°  φ={max(roll_vals):4.0f}°"
          f"  {state_name:>9}", end="")
    if phases:
        print(f"  phases:{len(phases)}", end="")
    if flags:
        print(f"  {' | '.join(flags)}", end="")
    print()

    return {
        "model": model,
        "dur_s": dur,
        "max_agl_m": max_agl,
        "max_kts": max_kts,
        "max_pitch": max(pitch_vals),
        "max_roll": max(roll_vals),
        "min_agl": min_agl,
        "state": state_name,
        "crashed": crashed,
        "flags": flags,
        "n_phases": len(phases),
    }


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    do_plot = "--plot" in sys.argv

    if not args:
        print("Usage: python3 tools/analyze_takeoff.py [--plot] <csv_file> [...]")
        sys.exit(1)

    summaries = []
    for path in args:
        if not os.path.exists(path):
            print(f"  skip: {path} not found")
            continue
        rows = load_csv(path)
        if not rows:
            continue
        s = summarize(path, rows)
        summaries.append(s)
        if do_plot:
            plot(path, rows)

    if len(summaries) > 1:
        ok = sum(1 for s in summaries if s["state"] == "completed" and not s["crashed"])
        fail = len(summaries) - ok
        print(f"\n  {ok}/{len(summaries)} passed, {fail} failed")


if __name__ == "__main__":
    main()
