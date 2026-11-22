#!/usr/bin/env python3
"""Collect compare_controllers.py results into one CSV and draw the line charts.

Usage: python summarize_results.py results_folder
Reads every <policy>_<seed>.json in the folder (tuned runs are named
tuned-<policy>_<seed>.json) and writes controller_results.csv,
controller_stuck_seed42.png, controller_vehicle_hours.png and
anchored_vs_fixed.png next to this script.
"""
from pathlib import Path
import csv
import json
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
folder = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "results"
runs = []
for f in sorted(folder.glob("*.json")):
    if f.name.endswith(".partial.json"):
        continue
    d = json.loads(f.read_text())
    d["name"] = f.stem.rsplit("_", 1)[0]
    runs.append(d)

with open(HERE / "controller_results.csv", "w", newline="") as out:
    w = csv.writer(out)
    w.writerow(["policy", "seed", "tuning", "ramp_4:00", "ramp_4:30", "ramp_5:00", "ramp_5:30",
                "stuck_over_3min_at_6:30", "arrived", "not_yet_entered_at_6pm", "vehicle_hours_4pm_on",
                "mean_delay_finished_s", "collisions", "teleports", "gridlock_at_end", "controller_cpu_s"])
    for d in sorted(runs, key=lambda d: (d["seed"], d["name"])):
        w.writerow([d["name"], d["seed"], json.dumps(d.get("tuning", {})) if d.get("tuning") else "",
                    *d["ramp_per_30min_4to6pm"], d["stuck_over_3min"][-1][1], d["arrived"],
                    d["not_yet_entered_at_6pm"], d["vehicle_hours_in_system_4pm_on"],
                    d["mean_delay_finished_s"], d["collisions"], d["teleports"], d["gridlock_at_end"],
                    d.get("controller_cpu_seconds", 0)])

INK, MUTED, GRID = "#1f1f1e", "#6b6a64", "#e4e3de"
STYLE = {  # baseline in dark ink, controllers in fixed categorical order
    "webster": ("Fixed-time (Webster)", INK),
    "int-local": ("Local queues only", "#2a78d6"),
    "max_pressure": ("Max-pressure", "#eb6834"),
    "mpc": ("Network MPC", "#1baf7a"),
    "int-coordinated": ("Coordinated (neighbour info)", "#e87ba4"),
}


def axes(ax, title):
    ax.set_title(title, loc="left", fontsize=11, color=INK)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.grid(axis="y", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


# Chart 1: cars stuck more than 3 minutes over the evening, seed 42 (fixed-time clears)
fig, ax = plt.subplots(figsize=(8, 4.5), dpi=150)
axes(ax, "Cars stuck more than 3 minutes, seed 42 (controllers take over at 4:00 PM)")
for key, (label, color) in STYLE.items():
    d = next((r for r in runs if r["name"] == key and r["seed"] == 42), None)
    if not d:
        continue
    xs = [t for t, _ in d["stuck_over_3min"]]
    ys = [n for _, n in d["stuck_over_3min"]]
    ax.plot(range(len(xs)), ys, color=color, linewidth=2.6 if key == "webster" else 2, label=label,
            marker="o", markersize=4)
    ax.annotate(label, (len(xs) - 1, ys[-1]), xytext=(6, 0), textcoords="offset points",
                va="center", fontsize=8.5, color=INK)
ax.set_xticks(range(len(xs)))
ax.set_xticklabels(xs)
ax.set_xlim(0, len(xs) + 2.6)
ax.set_ylabel("cars waiting > 3 min", color=MUTED, fontsize=9)
ax.legend(frameon=False, fontsize=8.5, loc="upper left")
fig.tight_layout()
fig.savefig(HERE / "controller_stuck_seed42.png")

# Chart 2: vehicle-hours in the network per seed, fixed-time vs controllers (lower is better)
seeds = sorted({r["seed"] for r in runs})
fig, ax = plt.subplots(figsize=(8, 4.5), dpi=150)
axes(ax, "Total vehicle-hours in the network, 4:00-6:30 PM (lower is better)")
for key, (label, color) in STYLE.items():
    pts = [(i, r["vehicle_hours_in_system_4pm_on"]) for i, s in enumerate(seeds)
           for r in runs if r["name"] == key and r["seed"] == s]
    if not pts:
        continue
    ax.plot([p[0] for p in pts], [p[1] for p in pts], color=color, linewidth=2.6 if key == "webster" else 2,
            marker="o", markersize=6, label=label)
ax.set_xticks(range(len(seeds)))
ax.set_xticklabels([f"seed {s}" for s in seeds])
ax.set_ylabel("vehicle-hours", color=MUTED, fontsize=9)
ax.legend(frameon=False, fontsize=8.5, loc="upper left", ncol=2)
fig.tight_layout()
fig.savefig(HERE / "controller_vehicle_hours.png")
# Chart 3: the plan-anchored controller against fixed-time on every seed (lower is better)
pairs = sorted(s for s in seeds if any(r["name"] == "anchored" and r["seed"] == s for r in runs)
               and any(r["name"] == "webster" and r["seed"] == s for r in runs))
if pairs:
    fig, ax = plt.subplots(figsize=(8, 4.5), dpi=150)
    axes(ax, "Total vehicle-hours in the network, 4:00-6:30 PM (lower is better)")
    for key, label, color, width in (("webster", "Fixed-time (Webster)", INK, 2.6),
                                     ("anchored", "Plan-anchored controller", "#2a78d6", 2.2)):
        ys = [next(r["vehicle_hours_in_system_4pm_on"] for r in runs if r["name"] == key and r["seed"] == s) for s in pairs]
        ax.plot(range(len(pairs)), ys, color=color, linewidth=width, marker="o", markersize=6, label=label)
    for i, s in enumerate(pairs):
        w = next(r for r in runs if r["name"] == "webster" and r["seed"] == s)
        if w["gridlock_at_end"]:
            ax.annotate("fixed-time\nlocks up", (i, w["vehicle_hours_in_system_4pm_on"]), xytext=(0, 10),
                        textcoords="offset points", ha="center", fontsize=8, color=MUTED)
    ax.set_xticks(range(len(pairs)))
    ax.set_xticklabels([f"seed {s}" for s in pairs])
    ax.set_ylabel("vehicle-hours", color=MUTED, fontsize=9)
    ax.set_ylim(0, max(r["vehicle_hours_in_system_4pm_on"] for r in runs if r["name"] in ("webster", "anchored")) * 1.15)
    ax.legend(frameon=False, fontsize=8.5, loc="lower right")
    fig.tight_layout()
    fig.savefig(HERE / "anchored_vs_fixed.png")
print("wrote controller_results.csv and the charts from", len(runs), "runs")
