#!/usr/bin/env python3
"""Matched comparison of signal policies on baseline-v2 evenings.

Every policy runs the same network, demand file and SUMO seed. Policies:

  fixed_native          SUMO runs the baseline-v2 fixed-time programmes; no controller
  fixed                 the same green order and durations through the shared safety layer
  max_pressure          Varaiya (2013) max pressure as already implemented in intelligent_coordinator.py
  max_pressure_guarded  max pressure every 5 s with the maximum-green and service-gap rules
  NAME=model.json       a frozen learned model (for example ml_local=..., ml_neighborhood=...)

Seeds a model was fitted or early-stopped on are refused, so reported cases are
episodes the model has never seen.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import hashlib
import itertools
import json
from math import comb
from pathlib import Path
import random
import statistics

import baseline_v2_scenario as scenario

BUILT_IN = ("fixed_native", "fixed", "max_pressure", "max_pressure_guarded")
# (key, label, True when higher is better)
METRICS = (("completed", "Completed trips", True),
           ("system_vehicle_hours", "System vehicle-hours incl. entry backlog", False),
           ("stopped_vehicle_hours", "Stopped vehicle-hours", False),
           ("pending", "Not yet entered at end", False),
           ("mean_stopped_vehicles", "Mean stopped vehicles", False),
           ("spillback_lane_hours", "Spillback lane-hours", False),
           ("off_ramp_total", "Off-ramp vehicles", True),
           ("junction_overlap_reports", "Junction overlap reports", False),
           ("stuck_over_3min_at_end", "Stopped > 3 min at end", False),
           ("longest_stop_p95", "Longest stop per vehicle, p95 (s)", False),
           ("stopped_over_3min", "Vehicles ever stopped > 3 min", False),
           ("decision_ms_p95", "Decision time per tick, p95 (ms)", False))


def run(spec):
    from ml_episode import run_episode
    return run_episode(spec)


def flat(result):
    row = dict(result)
    row["longest_stop_p95"] = result["fairness"]["vehicle_longest_stop_seconds"]["p95"]
    row["stopped_over_3min"] = result["fairness"]["vehicles_stopped_over_3min"]
    row["decision_ms_p95"] = result["latency_ms"]["decision_tick"]["p95"]
    return row


def paired(rows, candidate, reference, key, better_high, seeds):
    pairs = [(rows[candidate, seed][key], rows[reference, seed][key]) for seed in seeds
             if (candidate, seed) in rows and (reference, seed) in rows
             and rows[candidate, seed][key] is not None and rows[reference, seed][key] is not None]
    if not pairs:
        return None
    differences = [a - b for a, b in pairs]
    wins = sum((d > 0) == better_high and d != 0 for d in differences)
    losses = sum((d < 0) == better_high and d != 0 for d in differences)
    decided = wins + losses
    # Two-sided exact sign test on the seeds that differ.
    p_value = min(1.0, 2 * sum(comb(decided, k) for k in range(min(wins, losses) + 1)) / 2 ** decided) if decided else 1.0
    rng = random.Random(0)
    means = sorted(statistics.fmean(rng.choices(differences, k=len(differences))) for _ in range(10000))
    reference_mean = statistics.fmean(b for _, b in pairs)
    return {"n": len(pairs), "mean_difference": round(statistics.fmean(differences), 3),
            "percent_of_reference": round(100 * statistics.fmean(differences) / reference_mean, 2) if reference_mean else None,
            "bootstrap_95": [round(means[250], 3), round(means[9749], 3)],
            "seeds_better": wins, "seeds_worse": losses, "sign_test_p": round(p_value, 4)}


def summarise(results, labels, seeds):
    rows = {(r["label"], r["seed"]): flat(r) for r in results}
    summary = {"policies": {}, "comparisons": {}}
    for label in labels:
        mine = [rows[label, seed] for seed in seeds if (label, seed) in rows]
        entry = {"runs": len(mine), "valid_runs": sum(r["valid"] for r in mine),
                 "collisions": sum(r["lane_collisions"] for r in mine), "teleports": sum(r["teleports"] for r in mine),
                 "signal_rule_violations": sum(r.get("signal_audit", {}).get("violations", 0) for r in mine),
                 "gridlocked_runs": sum(bool(r["gridlock_at_end"]) for r in mine)}
        for key, _, _ in METRICS:
            values = [r[key] for r in mine if r[key] is not None]
            entry[key] = {"mean": round(statistics.fmean(values), 2) if values else None,
                          "sd": round(statistics.stdev(values), 2) if len(values) > 1 else None,
                          "min": min(values, default=None), "max": max(values, default=None)}
        summary["policies"][label] = entry
    for candidate, reference in itertools.permutations(labels, 2):
        if reference in BUILT_IN:
            # Adaptive policies against each simpler reference.
            if candidate in BUILT_IN and BUILT_IN.index(candidate) < max(2, BUILT_IN.index(reference) + 1):
                continue
        elif candidate in BUILT_IN or "neighborhood" not in candidate or "neighborhood" in reference:
            # Between learned models: only neighbourhood against local.
            continue
        summary["comparisons"][f"{candidate} vs {reference}"] = {
            key: paired(rows, candidate, reference, key, high, seeds) for key, _, high in METRICS[:-1]}
    return summary


def markdown(summary, labels, seeds, note):
    lines = [f"Seeds: {', '.join(map(str, seeds))}. {note}", "",
             "| Policy | Valid runs | Lane collisions | Signal-rule violations | Gridlocked at end | "
             + " | ".join(label for _, label, _ in METRICS) + " |",
             "|---|---|---|---|---|" + "---|" * len(METRICS)]
    for label in labels:
        entry = summary["policies"][label]
        cells = [f'{entry[key]["mean"]} ± {entry[key]["sd"]}' if entry[key]["mean"] is not None else "–" for key, _, _ in METRICS]
        lines.append(f'| {label} | {entry["valid_runs"]}/{entry["runs"]} | {entry["collisions"]} | '
                     f'{entry["signal_rule_violations"]} | {entry["gridlocked_runs"]} | ' + " | ".join(cells) + " |")
    lines += ["", "Cells are mean ± standard deviation across seeds.", ""]
    for name, comparison in summary["comparisons"].items():
        lines += [f"**{name}** (paired by seed; negative is lower)", "",
                  "| Metric | Mean difference | % of reference | Bootstrap 95% | Seeds better / worse | Sign-test p |",
                  "|---|---|---|---|---|---|"]
        for key, label, _ in METRICS[:-1]:
            row = comparison[key]
            if row:
                lines.append(f'| {label} | {row["mean_difference"]} | {row["percent_of_reference"]} | '
                             f'{row["bootstrap_95"][0]} to {row["bootstrap_95"][1]} | '
                             f'{row["seeds_better"]} / {row["seeds_worse"]} | {row["sign_test_p"]} |')
        lines.append("")
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--policies", nargs="+", required=True, help="built-in names and/or NAME=model.json")
    parser.add_argument("--seeds", type=int, nargs="+", required=True)
    parser.add_argument("--minutes", type=int, default=165, help="simulated minutes from 3:45 PM (165 = full evening)")
    parser.add_argument("--jobs", type=int, default=8)
    parser.add_argument("--purpose", choices=["validation", "test"], default="test")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--resume", action="store_true",
                        help="continue an interrupted batch in --out: finished runs are kept, the rest are run")
    args = parser.parse_args()
    policies, models = [], {}
    for item in args.policies:
        name, _, path = item.partition("=")
        if path:
            payload = json.loads(Path(path).read_text())
            used = set(payload["training"]["train_seeds"])
            if args.purpose == "test":
                used |= set(payload["training"]["validation_seeds"])
            if used & set(args.seeds):
                parser.error(f"{name} was fitted on seeds {sorted(used & set(args.seeds))}; choose unseen seeds")
            models[name] = {"path": str(Path(path).resolve()), "variant": payload["variant"],
                            "weights_sha256": payload["weights_sha256"],
                            "file_sha256": hashlib.sha256(Path(path).read_bytes()).hexdigest()}
            policies.append((name, "ml", models[name]["path"]))
        elif name in BUILT_IN:
            policies.append((name, name, None))
        else:
            parser.error(f"Unknown policy {name}")
    args.out.mkdir(parents=True, exist_ok=args.resume)
    network = scenario.merged_network(args.out / "network")
    specs = [{"policy": policy, "model": model, "seed": seed, "label": label, "network": str(network),
              "folder": str(args.out / f"{label}-seed{seed}"), "end": scenario.BEGIN + args.minutes * 60}
             for seed in args.seeds for label, policy, model in policies]
    labels = [label for label, _, _ in policies]
    manifest = {"purpose": args.purpose, "seeds": args.seeds, "minutes": args.minutes, "policies": labels,
                "models": models, "inputs": scenario.input_hashes(), "complete": False}
    (args.out / "run.json").write_text(json.dumps(manifest, indent=1) + "\n")
    from ml_episode import describe
    results = []
    if args.resume:
        # A finished run is reused only if it used the same inputs and model.
        for spec in list(specs):
            receipt = Path(spec["folder"]) / "result.json"
            if not receipt.is_file():
                continue
            previous = json.loads(receipt.read_text())
            model_hash = models[spec["label"]]["file_sha256"] if spec["model"] else None
            if previous.get("complete") and previous["inputs"] == manifest["inputs"] and previous["model_sha256"] == model_hash:
                results.append(previous)
                specs.remove(spec)
        manifest["resumed_with_finished_runs"] = len(results)
        print(f"Resuming: {len(results)} finished runs kept, {len(specs)} to run", flush=True)
    total = len(results) + len(specs)
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        for future in as_completed([pool.submit(run, spec) for spec in specs]):
            results.append(future.result())
            print(json.dumps(describe(results[-1])), flush=True)
            (args.out / "results.json").write_text(json.dumps(results, indent=1) + "\n")
    summary = summarise(results, labels, args.seeds)
    manifest.update(complete=len(results) == total, all_valid=all(r["valid"] for r in results))
    (args.out / "run.json").write_text(json.dumps(manifest, indent=1) + "\n")
    (args.out / "summary.json").write_text(json.dumps(summary, indent=1) + "\n")
    note = f"{args.minutes} simulated minutes from 3:45 PM, {args.purpose} episodes."
    (args.out / "summary.md").write_text(markdown(summary, labels, args.seeds, note))
    print(markdown(summary, labels, args.seeds, note))


if __name__ == "__main__":
    main()
