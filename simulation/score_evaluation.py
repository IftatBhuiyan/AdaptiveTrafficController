"""Score paired experiments: fixed timing is 80; improvement earns up to 100."""
import argparse
import json
from pathlib import Path

from evaluation_validity import SCHEMA_VERSION, validity, validity_reasons


METRICS = (
    ("completed_in_observation", .4, True),
    ("system_vehicle_seconds", .3, False),
    ("stopped_vehicle_seconds", .2, False),
    ("ramp_route_completions", .1, True),
)


def case_key(row):
    return tuple(row[key] for key in ("controller_family", "scenario", "hour", "seed", "observation_minutes", "warmup_minutes")) + (row.get("initialization", "fixed"), row.get("network_variant", "original"), row.get("driver_profile", "nominal"))


def paired_score(reference, candidate, historical=False):
    def inputs(row):
        return {key: value for key, value in row["inputs"].items() if not key.startswith("implementation/")}
    if (case_key(reference) != case_key(candidate) or inputs(reference) != inputs(candidate)
            or reference["demand_sha256"] != candidate["demand_sha256"]
            or reference["scope"] != candidate["scope"]
            or reference.get("network_sha256") != candidate.get("network_sha256")):
        raise ValueError("Scores require matched scenarios, demand, network, and observation windows")
    new_receipts = any(row.get("evaluation_schema", 1) >= SCHEMA_VERSION for row in (reference, candidate))
    if new_receipts and reference["inputs"] != candidate["inputs"]:
        raise ValueError("Scores require identical frozen implementation and input hashes for new evaluations")
    if not new_receipts and reference["inputs"] != candidate["inputs"] and not historical:
        raise ValueError("Historical implementation hashes differ; use --historical only for explicitly labeled inspection")
    reasons = {"reference": validity_reasons(reference), "candidate": validity_reasons(candidate)}
    valid = not any(reasons.values())
    components = {}
    # Invalid/non-finite metrics must never enter ratio arithmetic or JSON output.
    for key, weight, higher in METRICS:
        if valid:
            numerator, denominator = (candidate[key], reference[key]) if higher else (reference[key], candidate[key])
            ratio = numerator / denominator if denominator else (1.0 if numerator == 0 else float("inf"))
            components[key] = round(min(100.0, 80.0 * ratio), 3)
        else:
            components[key] = 0.0
    score = sum(weight * components[key] for key, weight, _ in METRICS) if valid else 0.0
    guardrails = valid and (candidate["completed_in_observation"] >= .95 * reference["completed_in_observation"]
                           and candidate["system_vehicle_seconds"] <= 1.05 * reference["system_vehicle_seconds"]
                           and candidate["ramp_route_completions"] >= .8 * reference["ramp_route_completions"])
    return {"case": list(case_key(candidate)), "policy": candidate["policy"], "score": round(score, 2),
            "valid": valid, "invalid_reasons": reasons, "regression_guardrails_pass": guardrails,
            "comparison_class": "frozen" if all(row.get("evaluation_schema", 1) >= SCHEMA_VERSION for row in (reference, candidate)) else "historical", "components": components}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path)
    parser.add_argument("--reference", type=Path)
    parser.add_argument("--policy", default="coordinated")
    parser.add_argument("--seeds", type=int, nargs="+")
    parser.add_argument("--historical", action="store_true", help="Explicitly inspect old receipts with different implementation hashes; cannot establish target acceptance")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    results = json.loads(args.results.read_text())
    total_results = len(results)
    references = json.loads(args.reference.read_text()) if args.reference else results
    if args.seeds:
        results = [row for row in results if row["seed"] in args.seeds]
        references = [row for row in references if row["seed"] in args.seeds]
    fixed = {case_key(row): row for row in references if row["policy"] == "fixed"}
    candidates = [row for row in results if row["policy"] == args.policy]
    if not candidates:
        parser.error("No matching policy results")
    missing = [case_key(row) for row in candidates if case_key(row) not in fixed]
    if missing:
        parser.error("Missing fixed reference for " + repr(missing))
    try:
        scores = [paired_score(fixed[case_key(row)], row, args.historical) for row in candidates]
    except ValueError as error:
        parser.error(str(error))
    mean = sum(row["score"] for row in scores) / len(scores)
    complete = {case_key(row) for row in candidates} == set(fixed)
    manifest_path = args.results.parent / "run.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        complete = complete and manifest["complete"] and total_results == manifest["case_count"]
    valid = complete and all(row["valid"] for row in scores)
    historical = args.historical or any(row["comparison_class"] == "historical" for row in scores)
    report = {"rubric": "v1: fixed=80; throughput 40%, system time 30%, stopped time 20%, ramp completions 10%; each component capped at 100",
              "score": round(mean, 2), "target": 88, "complete": complete, "valid": valid,
              "historical": historical,
              "target_met": valid and not historical and mean >= 88 and all(row["regression_guardrails_pass"] for row in scores),
              "case_count": len(scores), "cases": scores}
    output = json.dumps(report, indent=2, allow_nan=False) + "\n"
    if args.out:
        with args.out.open("x") as destination:
            destination.write(output)
    print(output, end="")
    if not valid:
        raise SystemExit("At least one paired run is invalid or missing; inspect invalid_reasons before reporting success.")


if __name__ == "__main__":
    main()
