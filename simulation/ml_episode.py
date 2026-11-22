"""Run one baseline-v2 evening under one signal policy and measure it.

Shared by data collection (train_neighborhood_ml.py) and held-out evaluation
(evaluate_neighborhood_ml.py), so training and testing use the same simulator
launch, the same safety layer and the same metric definitions.

Metric definitions follow evaluate_controller.py: system vehicle-seconds
integrate active vehicles plus vehicles that are due but not yet inserted
(boundary backlog). The departure schedule is read only here, by the evaluator.
"""
import bisect
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np

from sumo_runtime import configure
import baseline_v2_scenario as scenario

SUMO_HOME, SUMO = configure()
import traci  # noqa: E402
import traci.constants as tc  # noqa: E402

from adaptive_controller import Settings, Topology  # noqa: E402
from neighborhood_ml import NeighborhoodController, POLICIES, QModel, reward_definition  # noqa: E402

NATIVE = "fixed_native"  # SUMO runs the baseline-v2 programmes itself; no controller
METRIC_STEP = 1.0        # seconds between metric samples (signal audit runs every step)
STUCK_SECONDS = 180      # check_run.py: "stuck" = stopped for more than three minutes


def distribution(values):
    values = sorted(values)
    if not values:
        return {"count": 0, "mean": None, "p50": None, "p95": None, "max": None}
    def percentile(fraction):
        index = (len(values) - 1) * fraction
        low, high = math.floor(index), math.ceil(index)
        return round(values[low] + (values[high] - values[low]) * (index - low), 3)
    return {"count": len(values), "mean": round(sum(values) / len(values), 3), "p50": percentile(.5),
            "p95": percentile(.95), "max": round(values[-1], 3)}


class SignalAudit:
    """Independent check of what the signals displayed, read back from SUMO."""

    def __init__(self, controller, begin):
        self.signals, self.settings, self.begin = controller.signals, controller.settings, begin
        self.state = {key: None for key in self.signals}
        self.kind = {key: None for key in self.signals}
        self.since = {key: begin for key in self.signals}
        self.started_at_begin = {key: True for key in self.signals}
        self.red_since = {key: None for key in self.signals}
        self.clearance = {key: False for key in self.signals}
        self.violations, self.green_runs = [], []
        self.checks = defaultdict(int)

    @staticmethod
    def classify(state):
        lower = state.lower()
        return "yellow" if "y" in lower else "green" if "g" in lower else "red"

    def legal(self, key, *states):
        """Every green shown belongs to one phase of this signal's phase list."""
        return any(all(shown not in "gG" or allowed in "gG" for state in states for shown, allowed in zip(state, phase))
                   for phase in self.signals[key].greens)

    def observe(self, now, states):
        for key in self.signals:
            state = states.get(key)
            if state is None or state == self.state[key]:
                continue
            kind, previous_kind, previous = self.classify(state), self.kind[key], self.state[key]
            signal = self.signals[key]
            if previous is not None and signal.stage != "native" and not self.started_at_begin[key]:
                duration = now - self.since[key]
                if kind == "green":
                    self.checks["green_states"] += 1
                    if not self.legal(key, state):
                        self.violations.append({"time": now, "signal": key, "rule": "green_outside_phase_list", "state": state})
                    if previous_kind == "green" and not self.legal(key, previous, state):
                        self.violations.append({"time": now, "signal": key, "rule": "phase_change_without_clearance"})
                    if previous_kind == "yellow":
                        self.violations.append({"time": now, "signal": key, "rule": "green_directly_after_yellow"})
                    if previous_kind == "red" and self.clearance[key] and duration < self.settings.all_red - .21:
                        self.violations.append({"time": now, "signal": key, "rule": "all_red_too_short", "seconds": duration})
                if previous_kind == "green" and kind == "yellow":
                    self.checks["greens_ended"] += 1
                    self.green_runs.append(duration)
                    # The safety layer can hold every link of a new green red while a
                    # car is still inside the box, so the green timer is audited from
                    # the end of the previous yellow: all-red plus minimum green.
                    phase_time = now - self.red_since[key] if self.red_since[key] is not None else None
                    if phase_time is None and duration < self.settings.minimum_green - .21:
                        self.violations.append({"time": now, "signal": key, "rule": "minimum_green", "seconds": duration})
                    elif phase_time is not None and phase_time < self.settings.all_red + self.settings.minimum_green - .21:
                        self.violations.append({"time": now, "signal": key, "rule": "minimum_green", "seconds": phase_time})
                    elif duration < self.settings.minimum_green - .21:
                        self.checks["displayed_green_shortened_by_box_hold"] += 1
                if previous_kind == "yellow" and kind == "red":
                    self.checks["yellows_ended"] += 1
                    if duration < signal.yellow - .21:
                        self.violations.append({"time": now, "signal": key, "rule": "yellow_too_short", "seconds": duration})
            if kind != previous_kind:
                self.since[key] = now
                if kind == "red":
                    self.clearance[key] = previous_kind == "yellow"
                    if previous_kind == "yellow":
                        self.red_since[key] = now
                if previous is not None and signal.stage != "native":
                    self.started_at_begin[key] = False
            self.state[key], self.kind[key] = state, kind

    def summary(self):
        return {"violations": len(self.violations), "examples": self.violations[:20], "checks": dict(self.checks),
                "green_seconds": distribution(self.green_runs)}


def run_episode(spec):
    """spec: policy, seed, folder, network, [model], [epsilon], [record], [begin], [end], [explore_seed]."""
    policy, seed = spec["policy"], spec["seed"]
    folder = Path(spec["folder"])
    folder.mkdir(parents=True, exist_ok=True)
    network = Path(spec["network"])
    begin, end = spec.get("begin", scenario.BEGIN), spec.get("end", scenario.END)
    record, model_path = spec.get("record", False), spec.get("model")
    started = time.perf_counter()
    planned = scenario.departures(begin=begin)
    topology = Topology(network, Settings(imported_actions=True, entire_network=True))
    lengths = {lane: meta.length for lane, meta in topology.lanes.items()}
    result = {"schema": 1, "scenario": scenario.SCENARIO, "policy": policy, "seed": seed, "begin": begin, "end": end,
              "label": spec.get("label", policy), "epsilon": spec.get("epsilon", 0.0),
              "inputs": scenario.input_hashes(),
              "model_sha256": hashlib.sha256(Path(model_path).read_bytes()).hexdigest() if model_path else None}
    error, controller, audit = None, None, None
    inserted = arrived = collisions = teleports = 0
    system_seconds = stopped_seconds = spillback_seconds = queue_lane_seconds = 0.0
    max_stopped = max_pending = max_lane_queue = 0
    stopped_since, stopped_peaks, departure_times = {}, {}, {}
    ramp_seen, ramp_bins = set(), defaultdict(int)
    stuck_trace, decision_ms, model_ms = [], [], []
    collision_events = []
    junction_reports = lane_collisions = 0
    last_metric, last_stuck, now = begin, begin, begin
    try:
        traci.start([str(SUMO)] + scenario.sumo_args(network, seed, begin))
        result["sumo_version"] = traci.getVersion()[1]
        if policy != NATIVE:
            controller = NeighborhoodController(traci, network, policy, model=model_path,
                                                epsilon=spec.get("epsilon", 0.0),
                                                entry_edges=scenario.entry_edges() if record else (),
                                                action_space=spec.get("action_space", "phase"),
                                                seed=spec.get("explore_seed", seed), record=record)
            audit = SignalAudit(controller, begin)
            for key in controller.signals:
                traci.trafficlight.subscribe(key, (tc.TL_RED_YELLOW_GREEN_STATE,))
        traci.simulation.subscribe((tc.VAR_TIME, tc.VAR_DEPARTED_VEHICLES_IDS, tc.VAR_ARRIVED_VEHICLES_NUMBER,
                                    tc.VAR_COLLIDING_VEHICLES_NUMBER, tc.VAR_TELEPORT_STARTING_VEHICLES_NUMBER))
        cursor = 0
        while now < end - 1e-6:
            traci.simulationStep()
            sim = traci.simulation.getSubscriptionResults()
            now = sim[tc.VAR_TIME]
            for vehicle in sim[tc.VAR_DEPARTED_VEHICLES_IDS]:
                inserted += 1
                departure_times[vehicle] = now
                traci.vehicle.subscribe(vehicle, (tc.VAR_LANE_ID, tc.VAR_LANEPOSITION, tc.VAR_SPEED, tc.VAR_LENGTH))
            arrived += sim[tc.VAR_ARRIVED_VEHICLES_NUMBER]
            if sim[tc.VAR_COLLIDING_VEHICLES_NUMBER]:
                collisions += sim[tc.VAR_COLLIDING_VEHICLES_NUMBER]
                for event in traci.simulation.getCollisions():
                    # "junction" reports come from --collision.check-junctions,
                    # which check_run.py does not enable; they also occur under
                    # the unmodified fixed-time reference. Counted separately.
                    if event.type == "junction":
                        junction_reports += 1
                    else:
                        lane_collisions += 1
                    if len(collision_events) < 200:
                        collision_events.append({"time": now, "type": event.type, "lane": event.lane, "position": round(event.pos, 1),
                                                 "collider": event.collider, "victim": event.victim,
                                                 "collider_speed": round(event.colliderSpeed, 2), "victim_speed": round(event.victimSpeed, 2)})
            teleports += sim[tc.VAR_TELEPORT_STARTING_VEHICLES_NUMBER]
            observations = {vehicle: (row[tc.VAR_LANE_ID], row[tc.VAR_LANEPOSITION], row[tc.VAR_SPEED], row.get(tc.VAR_LENGTH, 5))
                            for vehicle, row in traci.vehicle.getAllSubscriptionResults().items()
                            if tc.VAR_LANE_ID in row}
            if controller is not None:
                before = controller.last_tick
                tick_started = time.perf_counter()
                controller.step(now, observations)
                if controller.last_tick != before:
                    decision_ms.append((time.perf_counter() - tick_started) * 1000)
                    if policy == "ml":
                        model_ms.append(controller.model_ms)
                audit.observe(now, {key: row[tc.TL_RED_YELLOW_GREEN_STATE]
                                    for key, row in traci.trafficlight.getAllSubscriptionResults().items()
                                    if tc.TL_RED_YELLOW_GREEN_STATE in row})
            if now < last_metric + METRIC_STEP - 1e-6 and now < end - 1e-6:
                continue
            elapsed, last_metric = now - last_metric, now
            cursor = bisect.bisect_right(planned, now, cursor)
            pending = max(0, cursor - inserted)
            stopped = 0
            lane_count, lane_halt, lane_gap = defaultdict(int), defaultdict(int), {}
            for vehicle, (lane, position, speed, length) in observations.items():
                if speed < .1:
                    stopped += 1
                    since = stopped_since.setdefault(vehicle, now)
                    if now - since > stopped_peaks.get(vehicle, 0):
                        stopped_peaks[vehicle] = now - since
                    lane_halt[lane] += 1
                else:
                    stopped_since.pop(vehicle, None)
                lane_count[lane] += 1
                if position < lane_gap.get(lane, 1e9):
                    lane_gap[lane] = position
                if vehicle not in ramp_seen and lane.rsplit("_", 1)[0] == scenario.OFF_RAMP_EXIT:
                    ramp_seen.add(vehicle)
                    ramp_bins[int((now - begin) // 1800)] += 1
            for vehicle in [v for v in stopped_since if v not in observations]:
                del stopped_since[vehicle]
            system_seconds += (len(observations) + pending) * elapsed
            stopped_seconds += stopped * elapsed
            max_stopped, max_pending = max(max_stopped, stopped), max(max_pending, pending)
            for lane, halting in lane_halt.items():
                length = lengths.get(lane)
                if length is None:
                    continue
                max_lane_queue = max(max_lane_queue, halting)
                # Spillback as in evaluate_controller.py: a queued lane that is
                # over 80% full with a vehicle within 10 m of its entrance.
                if lane_gap[lane] < 10 and lane_count[lane] * 7.5 > .8 * length:
                    spillback_seconds += elapsed
                if halting * 7.5 > .5 * length:
                    queue_lane_seconds += elapsed
            if now >= last_stuck + 900 - 1e-6:
                last_stuck = now
                stuck_trace.append({"time": now, "stuck": sum(now - since > STUCK_SECONDS for since in stopped_since.values()),
                                    "active": len(observations), "pending": pending, "arrived": arrived})
        active = traci.vehicle.getIDCount()
    except Exception as failure:  # keep a receipt for a failed run
        error = type(failure).__name__ + ": " + str(failure)
        active = len(stopped_since)
    finally:
        try:
            traci.close()
        except Exception:
            pass
    duration = max(1e-9, now - begin)
    due = bisect.bisect_right(planned, now)
    stuck_end = sum(now - since > STUCK_SECONDS for since in stopped_since.values())
    result.update({
        "complete": error is None and abs(now - end) < .01, "error": error, "actual_end": now,
        "collisions": collisions, "teleports": teleports, "collision_events": collision_events,
        "lane_collisions": lane_collisions, "junction_overlap_reports": junction_reports,
        "due": due, "inserted": inserted, "completed": arrived, "active": active,
        "pending": max(0, due - inserted), "max_pending": max_pending,
        "conservation_error": inserted - arrived - active,
        "system_vehicle_hours": round(system_seconds / 3600, 3),
        "stopped_vehicle_hours": round(stopped_seconds / 3600, 3),
        "mean_stopped_vehicles": round(stopped_seconds / duration, 2), "max_stopped_vehicles": max_stopped,
        "max_lane_queue_vehicles": max_lane_queue,
        "spillback_lane_hours": round(spillback_seconds / 3600, 3),
        "half_full_queue_lane_hours": round(queue_lane_seconds / 3600, 3),
        "off_ramp_per_30min": [ramp_bins.get(index, 0) for index in range(int(math.ceil((end - begin) / 1800)))],
        "off_ramp_total": len(ramp_seen),
        "stuck_over_3min_at_end": stuck_end, "gridlock_at_end": stuck_end > 100, "stuck_trace": stuck_trace,
        "fairness": {"vehicle_longest_stop_seconds": distribution(stopped_peaks.values()),
                     "vehicles_stopped_over_3min": sum(peak > STUCK_SECONDS for peak in stopped_peaks.values())},
        "latency_ms": {"decision_tick": distribution(decision_ms), "model_forward": distribution(model_ms)},
        "wall_seconds": round(time.perf_counter() - started, 1),
    })
    if controller is not None:
        status = controller.status()
        result["signal_audit"] = audit.summary()
        result["controller"] = {key: status.get(key) for key in (
            "controller", "policy", "action_space", "model_variant", "model_id", "signal_count", "decisions", "phase_switches",
            "model_decisions", "forced_by_service_rules", "random_actions", "control_evaluations")}
        result["settings"] = {key: getattr(controller.settings, key) for key in (
            "interval", "minimum_green", "maximum_green", "maximum_service_gap", "all_red", "clearance_extension")}
        if record and error is None:
            arrays = controller.training_arrays()
            if arrays is not None:
                np.savez_compressed(folder / "samples.npz", **arrays)
                result["samples"] = int(len(arrays["returns"]))
                result["reward"] = reward_definition()
    reasons = []
    if error or not result["complete"]:
        reasons.append("incomplete_run")
    if lane_collisions or (collisions and not junction_reports):
        reasons.append("collisions")
    if teleports:
        reasons.append("teleports")
    if result["conservation_error"]:
        reasons.append("vehicle_conservation")
    if controller is not None and result["signal_audit"]["violations"]:
        reasons.append("signal_rule_violations")
    result["invalid_reasons"], result["valid"] = reasons, not reasons
    (folder / "result.json").write_text(json.dumps(result, indent=1) + "\n")
    return result


def describe(result):
    return {key: result.get(key) for key in ("label", "seed", "valid", "invalid_reasons", "completed", "pending",
                                             "system_vehicle_hours", "stopped_vehicle_hours", "stuck_over_3min_at_end",
                                             "wall_seconds", "samples")}
