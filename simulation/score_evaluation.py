"""Score paired experiments: fixed timing is 80; improvement earns up to 100."""
import argparse
import json
from pathlib import Path


METRICS = (
    ("completed_in_observation", .4, True),
    ("system_vehicle_seconds", .3, False),
    ("stopped_vehicle_seconds", .2, False),
    ("ramp_route_completions", .1, True),
)


def case_key(row):
    return tuple(row[key] for key in ("controller_family", "scenario", "hour", "seed", "observation_minutes", "warmup_minutes")) + (row.get("initialization", "fixed"),)


def validity(row):
    return (row["collision_vehicle_reports"] == 0 and row["teleports"] == 0
            and row["inserted"] == row["completed"] + row["active"]
            and abs(row["actual_end"] - (row["hour"] * 3600 + row["observation_minutes"] * 60)) < .01
            and all(row[key] >= 0 for key, _, _ in METRICS))


def paired_score(reference, candidate):
    inputs = lambda row: {key: value for key, value in row["inputs"].items() if not key.startswith("implementation/")}
    if (case_key(reference) != case_key(candidate) or inputs(reference) != inputs(candidate)
            or reference["demand_sha256"] != candidate["demand_sha256"]
            or reference["scope"] != candidate["scope"]):
        raise ValueError("Scores require matched scenarios, demand, network, and observation windows")
    components = {}
    for key, weight, higher in METRICS:
        numerator, denominator = (candidate[key], reference[key]) if higher else (reference[key], candidate[key])
        ratio = numerator / denominator if denominator else (1.0 if numerator == 0 else float("inf"))
        components[key] = round(min(100.0, 80.0 * ratio), 3)
    valid = validity(reference) and validity(candidate)
    score = sum(weight * components[key] for key, weight, _ in METRICS) if valid else 0.0
    guardrails = (candidate["completed_in_observation"] >= .95 * reference["completed_in_observation"]
                  and candidate["system_vehicle_seconds"] <= 1.05 * reference["system_vehicle_seconds"]
                  and candidate["ramp_route_completions"] >= .8 * reference["ramp_route_completions"])
    return {"case": list(case_key(candidate)), "policy": candidate["policy"], "score": round(score, 2),
            "valid": valid, "regression_guardrails_pass": guardrails, "components": components}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path)
    parser.add_argument("--reference", type=Path)
    parser.add_argument("--policy", default="coordinated")
    parser.add_argument("--seeds", type=int, nargs="+")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    results = json.loads(args.results.read_text())
    total_results = len(results)
    references = json.loads(args.reference.read_text()) if args.reference else results
    if args.seeds:
        results = [row for row in results if row["seed"] in args.seeds]
        references = [row for row in references if row["seed"] in args.seeds]
    fixed = {case_key(row): row for row in references if row["policy"] == "fixed"}
    scores = [paired_score(fixed[case_key(row)], row) for row in results if row["policy"] == args.policy]
    if not scores:
        parser.error("No matching policy results")
    mean = sum(row["score"] for row in scores) / len(scores)
    complete = {case_key(row) for row in results if row["policy"] == args.policy} == set(fixed)
    manifest_path = args.results.parent / "run.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        complete = complete and manifest["complete"] and total_results == manifest["case_count"]
    report = {"rubric": "v1: fixed=80; throughput 40%, system time 30%, stopped time 20%, ramp completions 10%; each component capped at 100",
                      "score": round(mean, 2), "target": 88,
                      "complete": complete,
                      "target_met": complete and mean >= 88 and all(row["valid"] and row["regression_guardrails_pass"] for row in scores),
                      "case_count": len(scores), "cases": scores}
    output = json.dumps(report, indent=2) + "\n"
    if args.out:
        with args.out.open("x") as destination:
            destination.write(output)
    print(output, end="")


if __name__ == "__main__":
    main()
