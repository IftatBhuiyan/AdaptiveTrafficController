#!/usr/bin/env python3
"""Turn one or more evaluation folders into the results tables in README.md.

Outcomes on baseline-v2 are bimodal: an evening either clears (about
7,600-8,300 system vehicle-hours) or gridlocks (13,000+). A mean over eight
seeds mostly counts gridlocks, so this report leads with per-seed values,
gridlock counts and medians, then paired differences with an exact sign test.

Usage: python3 make_report.py RESULTS_FOLDER [MORE_FOLDERS] --out results/
"""
import argparse
from math import comb
import json
from pathlib import Path
import statistics

ORDER = ("fixed_native", "fixed", "max_pressure", "max_pressure_guarded", "ml_local", "ml_neighborhood")
NAMES = {"fixed_native": "Fixed time (SUMO runs the plan)", "fixed": "Fixed time (same safety layer as ML)",
         "max_pressure": "Max pressure (no max-green/service-gap rules)",
         "max_pressure_guarded": "Max pressure + same service rules as ML",
         "ml_local": "Learned, local view", "ml_neighborhood": "Learned, neighbourhood view"}
GRIDLOCK = 100  # vehicles stopped > 3 min at 6:30 PM, as in baseline-v2/check_run.py


def sign_test(wins, losses):
    n = wins + losses
    if not n:
        return 1.0
    return min(1.0, 2 * sum(comb(n, k) for k in range(min(wins, losses) + 1)) / 2 ** n)


def load(folders):
    rows = {}
    for folder in folders:
        for path in sorted(Path(folder).glob("*/result.json")):
            row = json.loads(path.read_text())
            rows[row["label"], row["seed"]] = row
    return rows


def fmt(value, digits=0):
    if value is None:
        return "–"
    return f"{value:,.{digits}f}"


def report(rows):
    labels = [label for label in ORDER if any(key[0] == label for key in rows)]
    seeds = sorted({seed for _, seed in rows})
    complete = [seed for seed in seeds if all((label, seed) in rows for label in labels)]
    out = [f"Held-out seeds: {', '.join(map(str, seeds))}. Full evening 3:45–6:30 PM, no exploration, frozen models.",
           f"Seeds with every policy finished: {len(complete)} of {len(seeds)}.", ""]

    def per_seed(key, title, digits=0):
        out.extend([f"**{title}**", "", "| Seed | " + " | ".join(labels) + " |", "|---|" + "---|" * len(labels)])
        for seed in seeds:
            out.append(f"| {seed} | " + " | ".join(fmt(rows.get((label, seed), {}).get(key), digits) for label in labels) + " |")
        medians = []
        for label in labels:
            values = [rows[label, seed][key] for seed in complete if rows[label, seed].get(key) is not None]
            medians.append(fmt(statistics.median(values), digits) if values else "–")
        out.extend(["| **median** | " + " | ".join(medians) + " |", ""])

    per_seed("system_vehicle_hours", "System vehicle-hours incl. vehicles waiting to enter (lower is better)")
    per_seed("completed", "Completed trips (higher is better)")
    per_seed("stuck_over_3min_at_end", "Vehicles stopped > 3 min at 6:30 PM (over 100 = gridlock)")

    out.extend(["**Summary per policy** (medians over seeds where every policy finished)", "",
                "| Policy | Runs | Over 100 stuck at 6:30 PM | Stopped veh-h | Spillback lane-h | Off-ramp vehicles | "
                "Longest stop p95 (s) | Vehicles stopped > 3 min | Lane collisions | Junction overlap reports | "
                "Signal-rule violations | Decision tick p95 (ms) | Model p95 (ms) |",
                "|---|" + "---|" * 12])
    for label in labels:
        mine = [rows[label, seed] for seed in complete]
        if not mine:
            continue
        median = lambda f: statistics.median(f(row) for row in mine)
        audit = [row.get("signal_audit", {}).get("violations") for row in mine]
        out.append(" | ".join([
            f"| {NAMES.get(label, label)}", str(len(mine)),
            str(sum(row["stuck_over_3min_at_end"] > GRIDLOCK for row in mine)),
            fmt(median(lambda r: r["stopped_vehicle_hours"])), fmt(median(lambda r: r["spillback_lane_hours"]), 1),
            fmt(median(lambda r: r["off_ramp_total"])),
            fmt(median(lambda r: r["fairness"]["vehicle_longest_stop_seconds"]["p95"])),
            fmt(median(lambda r: r["fairness"]["vehicles_stopped_over_3min"])),
            str(sum(row["lane_collisions"] for row in mine)), str(sum(row["junction_overlap_reports"] for row in mine)),
            "–" if None in audit else str(sum(audit)),
            "–" if label == "fixed_native" else fmt(median(lambda r: r["latency_ms"]["decision_tick"]["p95"]), 1),
            fmt(median(lambda r: r["latency_ms"]["model_forward"]["p95"]), 2) if label.startswith("ml") else "–"]) + " |")
    out.append("")

    pairs = [("ml_neighborhood", "ml_local"), ("ml_neighborhood", "fixed"), ("ml_local", "fixed"),
             ("ml_neighborhood", "fixed_native"), ("ml_local", "fixed_native"),
             ("ml_neighborhood", "max_pressure_guarded"), ("ml_local", "max_pressure_guarded"),
             ("max_pressure_guarded", "fixed")]
    out.extend(["**Paired by seed** (system vehicle-hours; negative = candidate lower)", "",
                "| Candidate vs reference | Seeds | Candidate lower / higher | Median difference | Exact sign-test p |",
                "|---|---|---|---|---|"])
    for candidate, reference in pairs:
        if candidate not in labels or reference not in labels:
            continue
        differences = [rows[candidate, s]["system_vehicle_hours"] - rows[reference, s]["system_vehicle_hours"] for s in complete]
        better, worse = sum(d < 0 for d in differences), sum(d > 0 for d in differences)
        out.append(f"| {candidate} vs {reference} | {len(differences)} | {better} / {worse} | "
                   f"{fmt(statistics.median(differences))} | {sign_test(better, worse):.3f} |")
    out.extend(["", "With 8 seeds the two-sided sign test reaches p < 0.05 only at 8/0 (p = 0.008); "
                "7/1 gives p = 0.070.", ""])
    return "\n".join(out) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("folders", nargs="+", type=Path)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    rows = load(args.folders)
    text = report(rows)
    print(text)
    if args.out:
        args.out.mkdir(parents=True, exist_ok=True)
        (args.out / "test-report.md").write_text(text)
        keep = ("label", "seed", "policy", "valid", "invalid_reasons", "completed", "pending", "system_vehicle_hours",
                "stopped_vehicle_hours", "spillback_lane_hours", "off_ramp_per_30min", "off_ramp_total",
                "stuck_over_3min_at_end", "stuck_trace", "lane_collisions", "junction_overlap_reports", "fairness",
                "latency_ms", "signal_audit", "controller", "model_sha256", "inputs", "wall_seconds")
        compact = [{key: row.get(key) for key in keep} for _, row in sorted(rows.items())]
        for row in compact:
            if row["signal_audit"]:
                row["signal_audit"] = {k: row["signal_audit"][k] for k in ("violations", "checks")}
        (args.out / "test-runs.json").write_text(json.dumps(compact, indent=1) + "\n")


if __name__ == "__main__":
    main()
