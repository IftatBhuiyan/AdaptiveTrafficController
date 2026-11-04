"""Shared, reasoned acceptance checks for evaluator receipts and paired scores."""
import math


SCHEMA_VERSION = 2
SCORE_METRICS = ("completed_in_observation", "system_vehicle_seconds", "stopped_vehicle_seconds", "ramp_route_completions")
REQUIRED_NONNEGATIVE = SCORE_METRICS + ("collision_vehicle_reports", "teleports", "inserted", "completed", "active", "actual_end")
OPTIONAL_NONNEGATIVE = (
    "due", "pending", "max_pending", "max_stopped", "inserted_in_observation", "control_evaluations_in_observation",
    "phase_switches_in_observation", "accrued_loss_with_insertion_seconds", "spillback_lane_seconds",
    "max_observed_service_gap_seconds", "ramp_entered_merge_section_new_cohort", "maximum_waiting_plus_insertion_seconds",
    "wall_seconds", "off_ramp_edge_exit_crossings", "on_ramp_edge_exit_crossings", "unexplained_removals", "solver_max_mass_residual",
)


def finite_nonnegative(value):
    try:
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and value >= 0
    except OverflowError:
        return False


def validity_reasons(row):
    """Accept old receipts for inspection, but never override an explicit failure."""
    reasons = []
    if row.get("valid") is False:
        reasons.append("receipt_marked_invalid")
    if row.get("safe") is False:
        reasons.append("receipt_marked_unsafe")
    if row.get("evaluation_error"):
        reasons.append("evaluation_error")
    if row.get("complete") is False:
        reasons.append("incomplete_run")
    if row.get("lane_change_deadlocks"):
        reasons.append("lane_change_deadlocks")
    if row.get("diagnostic_errors", {}).get("lane_change_deadlock_scan"):
        reasons.append("deadlock_observation_unavailable")
    for key in REQUIRED_NONNEGATIVE + tuple(key for key in OPTIONAL_NONNEGATIVE if key in row):
        if not finite_nonnegative(row.get(key)):
            reasons.append("invalid_metric:" + key)
    for key in ("collision_vehicle_reports", "teleports"):
        if finite_nonnegative(row.get(key)) and row[key] != 0:
            reasons.append(key)
    if all(finite_nonnegative(row.get(key)) for key in ("inserted", "completed", "active")):
        if row["inserted"] != row["completed"] + row["active"]:
            reasons.append("vehicle_conservation")
    if row.get("conservation_error") or row.get("unexplained_removals"):
        reasons.append("vehicle_conservation")
    hour, minutes = row.get("hour"), row.get("observation_minutes")
    if not finite_nonnegative(hour) or not finite_nonnegative(minutes) or not finite_nonnegative(row.get("actual_end")):
        reasons.append("invalid_observation_window")
    elif minutes <= 0 or abs(row["actual_end"] - (hour * 3600 + minutes * 60)) >= .01:
        reasons.append("incomplete_observation_window")
    if finite_nonnegative(row.get("solver_max_mass_residual", 0)) and row.get("solver_max_mass_residual", 0) > 1e-6:
        reasons.append("solver_mass_conservation")
    if row.get("input_changed"):
        reasons.append("inputs_changed_during_run")
    if row.get("evaluation_schema", 1) >= SCHEMA_VERSION:
        if not row.get("inputs") or not row.get("inputs_start") or not row.get("inputs_end") or row["inputs"] != row["inputs_start"] or row["inputs"] != row["inputs_end"]:
            reasons.append("inputs_changed_during_run")
        if row.get("case_inputs_start") != row.get("case_inputs_end"):
            reasons.append("generated_inputs_changed_during_run")
        if row.get("frozen_inputs_verified") is not True:
            reasons.append("unverified_frozen_inputs")
        if row.get("case_inputs_start") != row.get("case_inputs_end"):
            reasons.append("generated_inputs_changed_during_run")
    if any(item.get("vehicle_conservation_error") for item in row.get("peak_halfhour_bins", [])):
        reasons.append("boundary_vehicle_conservation")
    for row_bin in row.get("peak_halfhour_bins", []):
        if row_bin.get("vehicle_conservation_error"):
            reasons.append("bin_vehicle_conservation")
        for name in ("on_ramp", "off_ramp"):
            ramp = row_bin.get(name, {})
            if ramp.get("due_conservation_error"):
                reasons.append("bin_due_conservation:" + name)
            for key in ("edge_exit_crossings", "offered_due", "observed_offered_due", "observed_due_inserted", "observed_due_pending"):
                if key in ramp and not finite_nonnegative(ramp[key]):
                    reasons.append("invalid_metric:peak_halfhour_bins." + name + "." + key)
    # Optional summaries explicitly use None for an empty/undefined population.
    for section in ("fairness", "latency"):
        for population, summary in row.get(section, {}).items():
            if not isinstance(summary, dict):
                continue
            for key in ("count", "p50", "p95", "max"):
                value = summary.get(key)
                if value is not None and not finite_nonnegative(value):
                    reasons.append("invalid_metric:" + section + "." + population + "." + key)
    return list(dict.fromkeys(reasons))


def validity(row):
    return not validity_reasons(row)
