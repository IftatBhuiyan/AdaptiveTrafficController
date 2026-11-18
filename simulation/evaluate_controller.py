#!/usr/bin/env python3
"""Paired, reproducible controller experiments including boundary backlog."""
import argparse
import bisect
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import shutil
import time
import xml.etree.ElementTree as ET

from adaptive_controller import AdaptiveController, Settings, MODES, CENTRES
from evaluation_validity import SCHEMA_VERSION, validity, validity_reasons
from intelligent_coordinator import IntelligentCoordinator
from scenario_editor import ScenarioEditor, SUMO, DATA
from traffic_behavior import MODES as TRAFFIC_MODES
import traci
import traci.constants as tc


MPC_MODES = {"mpc", "mpc_learned"}
SOURCE = Path(__file__).resolve().parent


def roster(path, end):
    root = ET.parse(path).getroot()
    result = {v.get("id"): float(v.get("depart")) for v in root.findall("vehicle") if float(v.get("depart")) <= end}
    for flow in root.findall("flow"):
        number = int(flow.get("number"))
        begin, finish = float(flow.get("begin")), float(flow.get("end"))
        for i in range(number):
            departure = begin + i * (finish - begin) / number
            if departure <= end:
                result[f'{flow.get("id")}.{i}'] = departure
    return result


def route_cohorts(path, scheduled, edge_sets):
    """Evaluator-only offered demand; never passed to a controller."""
    root = ET.parse(path).getroot()
    routes = {r.get("id"): set(r.get("edges").split()) for r in root.findall("route")}
    cohorts = {name: {} for name in edge_sets}
    for name, edges in edge_sets.items():
        matching = {route for route, path in routes.items() if edges & path}
        for row in root.findall("vehicle") + root.findall("flow"):
            inline = row.find("route")
            belongs = row.get("route") in matching or (inline is not None and bool(edges & set(inline.get("edges").split())))
            if belongs:
                ids = [row.get("id")] if row.tag == "vehicle" else [f'{row.get("id")}.{i}' for i in range(int(row.get("number")))]
                cohorts[name].update((vid, scheduled[vid]) for vid in ids if vid in scheduled)
    return cohorts


def input_paths(model_path=None):
    # Freeze all Python sources, not just the controller entry points, and every
    # local data/config source the scenario builder can read. Exclude run outputs.
    paths = {"implementation/" + path.name: path for path in SOURCE.glob("*.py")}
    paths.update({"implementation/" + path.name: path for path in SOURCE.glob("*.json")})
    for path in DATA.iterdir():
        if path.is_file() and path.suffix.lower() in {".json", ".xml", ".csv", ".osm", ".sumocfg", ".py"}:
            paths[path.name] = path
    if model_path:
        paths["artifacts/forecast-model"] = Path(model_path).resolve()
    return paths


def input_hashes(model_path=None):
    return {name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in sorted(input_paths(model_path).items())}


def semantic_network_hash(path):
    # netconvert embeds temp paths/timestamps in XML comments. Default parsing
    # drops those comments; keep a separate raw hash for exact artifact receipts.
    return hashlib.sha256(ET.tostring(ET.parse(path).getroot(), encoding="utf-8")).hexdigest()


def verify_snapshot(output, hashes):
    root = Path(output)
    return all((root / name if name.startswith("implementation/") else root / "inputs" / name).is_file()
               and hashlib.sha256((root / name if name.startswith("implementation/") else root / "inputs" / name).read_bytes()).hexdigest() == digest
               for name, digest in hashes.items())


def distribution(values):
    values = sorted(values)
    if not values:
        return {"defined": False, "count": 0, "p50": None, "p95": None, "max": None}
    def percentile(fraction):
        index = (len(values) - 1) * fraction
        low, high = math.floor(index), math.ceil(index)
        return round(values[low] + (values[high] - values[low]) * (index - low), 4)
    return {"defined": True, "count": len(values), "p50": percentile(.5), "p95": percentile(.95), "max": round(values[-1], 4)}


def peak_bins(target, actual_end, planned_end, events, cohorts, departure_times, system_boundaries=None):
    """Half-open 30-minute bins; partial coverage cannot imply a ramp collapse."""
    bins = []
    for start in range(16 * 3600, 18 * 3600 + 1, 1800):
        end = start + 1800
        observed_start, observed_end = max(target, start), min(actual_end, planned_end, end)
        coverage = max(0.0, observed_end - observed_start)
        row = {"begin": start, "end": end, "coverage_seconds": round(coverage, 3),
               "coverage_complete": target <= start and min(actual_end, planned_end) >= end}
        if system_boundaries:
            row["system_boundary_receipts"] = [receipt for receipt in system_boundaries if start <= receipt["time"] <= end]
            row["vehicle_conservation_error"] = max((abs(receipt["conservation_error"]) for receipt in row["system_boundary_receipts"]), default=0)
        for name, due in cohorts.items():
            crossings = sum(start <= event["time"] < end and event["ramp"] == name for event in events)
            offered = sum(start <= when < end for when in due.values())
            def backlog(at):
                return sum(when <= at and departure_times.get(vid, float("inf")) > at for vid, when in due.items())
            backlog_start = backlog(max(target, start)) if coverage else None
            backlog_end = backlog(observed_end) if coverage else None
            observed_due = {vid: when for vid, when in due.items() if start <= when < min(end, actual_end, planned_end)} if coverage else {}
            observed_inserted = sum(departure_times.get(vid, float("inf")) <= observed_end for vid in observed_due)
            observed_pending = sum(departure_times.get(vid, float("inf")) > observed_end for vid in observed_due)
            row[name] = {"edge_exit_crossings": crossings, "offered_due": offered,
                         "observed_offered_due": len(observed_due), "observed_due_inserted": observed_inserted,
                         "observed_due_pending": observed_pending, "due_conservation_error": len(observed_due) - observed_inserted - observed_pending,
                         "boundary_backlog_start": backlog_start, "boundary_backlog_end": backlog_end,
                         "zero_crossings_with_demand": bool(row["coverage_complete"] and crossings == 0 and (offered or backlog_start))}
        bins.append(row)
    return bins


def batch_valid(results):
    return bool(results) and all(validity(result) for result in results)


# Spawned workers re-import this module; tuning travels through the environment.
TUNING = json.loads(os.environ.get("TC_TUNING", "{}"))


def run_case(case):
    policy, scenario, hour, seed, minutes, output, hashes, edits, predictive, family, initialization, *options = case
    model_path = options[0] if options else None
    collect_forecast = options[1] if len(options) > 1 else False
    collect_diagnostic_prefix = options[2] if len(options) > 2 else False
    network_variant = options[3] if len(options) > 3 else "original"
    driver_profile = options[4] if len(options) > 4 else "nominal"
    name = f"{scenario}-{hour}-{seed}-{policy}" + ("-no-forecast" if not predictive else "")
    folder = Path(output) / name
    folder.mkdir(parents=True, exist_ok=True)
    target, end = hour * 3600, hour * 3600 + minutes * 60
    now, previous_time = target - 900, target - 900
    started = time.perf_counter()
    settings = Settings(predictive=predictive, **TUNING)
    build, scheduled, planned_times, cohorts = {}, {}, [], {"off_ramp": {}, "on_ramp": {}}
    edge_sets = {"off_ramp": set(), "on_ramp": {"25345887"}, "off_ramp_downstream": {"25345883#2"}}
    trip_path = folder / "trips.xml"
    controller, connected, sumo_version, evaluation_error = None, False, None, None
    inserted, departed_ids, departure_times, arrived = 0, set(), {}, 0
    all_arrived_ids = set()
    system_seconds = stopped_seconds = spillback_lane_seconds = 0.0
    ramp_passed, ramp_seen, lane_usage = set(), set(), set()
    collisions, collision_events, teleports, collision_details = 0, [], 0, []
    lane_change_deadlocks, deadlock_candidates, diagnostic_errors = {}, {}, {}
    trace, max_pending, max_stopped, max_service_age = [], 0, 0, 0.0
    final_vehicles, previous_edges, crossing_events = {}, {}, []
    stopped_since, stopped_peaks, movement_peaks = {}, {}, {}
    decision_events, solver_events = [], []
    last_trace, last_deadlock_scan, solver_count = target - 30, now - 1, 0
    last_checkpoint, checkpoint_number, checkpoints = now, 0, []
    activated = False
    completed_at_activation = inserted_at_activation = 0
    evaluations_at_activation = switches_at_activation = 0
    status = {}
    case_inputs_start, case_inputs_end, system_boundaries = {}, {}, []
    hashes_start, hashes_end, snapshot_verified = {}, {}, False
    try:
        hashes_start = input_hashes(model_path)
        snapshot_verified = verify_snapshot(output, hashes)
        if hashes_start != hashes or not snapshot_verified:
            raise RuntimeError("Simulation source/input snapshot changed before evaluation")
        editor = ScenarioEditor(network_variant=network_variant)
        build = editor.build(edits, folder, scenario, target - 900, seed, driver_profile=driver_profile)
        case_inputs_start = {key: hashlib.sha256(Path(build[key]).read_bytes()).hexdigest() for key in ("network", "routes")}
        shutil.copy2(build["network"], folder / "frozen-network.net.xml")
        scheduled = roster(build["routes"], end)
        planned_times = sorted(scheduled.values())
        plan = json.loads((DATA / "full-area-demand.json").read_text())
        edge_sets["off_ramp"] = set(plan["station_edges"]["053169"])
        cohorts = route_cohorts(build["routes"], scheduled, edge_sets)
        args = build["args"] + ["--end", str(end), "--tripinfo-output", str(trip_path),
                               "--tripinfo-output.write-unfinished", "true", "--tripinfo-output.write-undeparted", "true",
                               "--collision-output", str(folder / "collisions.xml"), "--log", str(folder / "sumo.log"),
                               "--save-state.rng", "true"]
        traci.start([str(SUMO)] + args)
        connected = True
        sumo_version = traci.getVersion()[1]
        initial_mode = policy if initialization == "policy" else "fixed"
        if policy in MPC_MODES or (collect_forecast and policy == "fixed"):
            from network_mpc import NetworkMPC
            controller = NetworkMPC(traci, build["network"], mode=policy, settings=settings, model_path=model_path, boundary_exits=build.get("boundary_exits"))
            controller.mode = initial_mode
        else:
            controller_class = IntelligentCoordinator if family == "intelligent" else AdaptiveController
            controller = controller_class(traci, build["network"], initial_mode, settings)
        settings = controller.settings
        traci.simulation.subscribe((tc.VAR_TIME, tc.VAR_DEPARTED_VEHICLES_IDS, tc.VAR_ARRIVED_VEHICLES_IDS,
                                    tc.VAR_COLLIDING_VEHICLES_NUMBER, tc.VAR_TELEPORT_STARTING_VEHICLES_NUMBER))
        while now < end - 1e-6:
            traci.simulationStep()
            sim = traci.simulation.getSubscriptionResults()
            now = sim[tc.VAR_TIME]
            for vid in sim[tc.VAR_DEPARTED_VEHICLES_IDS]:
                departed_ids.add(vid)
                departure_times[vid] = now
                inserted += 1
                traci.vehicle.subscribe(vid, (tc.VAR_LANE_ID, tc.VAR_LANEPOSITION, tc.VAR_SPEED, tc.VAR_LENGTH, tc.VAR_TIMELOSS))
            arrived_ids = sim[tc.VAR_ARRIVED_VEHICLES_IDS]
            arrived += len(arrived_ids)
            all_arrived_ids.update(arrived_ids)
            collisions += sim[tc.VAR_COLLIDING_VEHICLES_NUMBER]
            teleports += sim[tc.VAR_TELEPORT_STARTING_VEHICLES_NUMBER]
            if collisions or teleports:
                try:
                    events = traci.simulation.getCollisions()
                    collision_events.extend(str(event) for event in events)
                    for event in events:
                        detail = {"time": now, "collision": str(event), "vehicles": {}}
                        for vehicle in (event.collider, event.victim):
                            try:
                                detail["vehicles"][vehicle] = {"lane": traci.vehicle.getLaneID(vehicle),
                                    "previous_lane": controller.last_external.get(vehicle),
                                    "position": traci.vehicle.getLanePosition(vehicle), "angle": traci.vehicle.getAngle(vehicle)}
                            except Exception as error:
                                detail["vehicles"][vehicle] = {"unavailable": type(error).__name__ + ": " + str(error)}
                        collision_details.append(detail)
                except Exception as error:
                    diagnostic_errors["collision_details"] = type(error).__name__ + ": " + str(error)
                final_vehicles = controller.vehicle_observations()
                break
            if now >= target and not activated:
                completed_at_activation, inserted_at_activation = arrived, inserted
                evaluations_at_activation = getattr(controller, "control_evaluations", 0)
                switches_at_activation = sum(signal.switches for signal in controller.signals.values())
                controller.mode = policy
                controller.forecast_errors.clear()
                activated = True
            before_evaluations = getattr(controller, "control_evaluations", 0)
            before_tick = controller.last_tick
            decision_started = time.perf_counter()
            controller.step(now)
            step_ms = (time.perf_counter() - decision_started) * 1000
            evaluated = getattr(controller, "control_evaluations", 0) - before_evaluations
            if evaluated or controller.last_tick != before_tick:
                decision_events.append({"time": now, "elapsed_ms": step_ms, "control_evaluations": evaluated,
                                        "period": "observation" if now >= target else "warmup", "mode": controller.mode})
            current_solver_events = getattr(controller, "solver_events", [])
            solver_events.extend(dict(event) for event in current_solver_events[solver_count:])
            solver_count = len(current_solver_events)
            observations = controller.vehicle_observations()
            final_vehicles = observations
            if now >= target and (not system_boundaries or now >= system_boundaries[-1]["time"] + 1800 - 1e-6 or now >= end - 1e-6):
                system_boundaries.append({"time": now, "inserted": inserted, "completed": arrived, "active": len(observations),
                                          "conservation_error": inserted - arrived - len(observations)})
            for vid, (lane, position, speed, length) in observations.items():
                if speed < .1:
                    stopped_since.setdefault(vid, now)
                    if now >= target:
                        stopped_peaks[vid] = max(stopped_peaks.get(vid, 0), now - stopped_since[vid])
                else:
                    stopped_since.pop(vid, None)
                edge = lane.rsplit("_", 1)[0]
                prior = previous_edges.get(vid)
                if prior != edge:
                    for ramp, edges in edge_sets.items():
                        if prior in edges and now >= target:
                            crossing_events.append({"time": now, "vehicle": vid, "edge": prior, "ramp": ramp})
                previous_edges[vid] = edge
                if lane.startswith("25345887_"):
                    ramp_seen.add(vid)
                    if now >= target:
                        lane_usage.add(lane)
                if now >= target and vid in ramp_seen and lane.startswith("1134690192_"):
                    ramp_passed.add(vid)
            for vid in arrived_ids:
                for ramp, edges in edge_sets.items():
                    if previous_edges.get(vid) in edges and now >= target:
                        crossing_events.append({"time": now, "vehicle": vid, "edge": previous_edges[vid], "ramp": ramp})
                previous_edges.pop(vid, None)
                stopped_since.pop(vid, None)
            # Integrate every SUMO step, including boundary backlog. Sampling at
            # 30 seconds is for the compact trace only, never for metrics/timing.
            elapsed = max(0.0, min(now, end) - max(previous_time, target))
            if elapsed:
                pending = max(0, bisect.bisect_right(planned_times, now) - inserted)
                active = len(observations)
                stopped = sum(v[2] < .1 for v in observations.values())
                system_seconds += (active + pending) * elapsed
                stopped_seconds += stopped * elapsed
                max_pending, max_stopped = max(max_pending, pending), max(max_stopped, stopped)
                for lane, row in controller.rows.items():
                    meta = controller.topology.lanes[lane]
                    if row["queue"] > 0 and row["entry_gap"] < 10 and row["stock"] * settings.storage_per_vehicle > .8 * meta.length:
                        spillback_lane_seconds += elapsed
                for lane, count in controller.queued.items():
                    if count:
                        age = max(0.0, now - controller.last_service.get(lane, now))
                        movement_peaks[lane] = max(movement_peaks.get(lane, 0), age)
                        max_service_age = max(max_service_age, age)
                if now >= last_trace + 30 - 1e-6:
                    status = controller.status()
                    trace.append({"time": round(now, 1), "active": active, "stopped": stopped, "pending": pending,
                                  "ramp_entered_merge_section_new_cohort": len(ramp_passed),
                                  "controller": {k: v for k, v in status.items() if k != "signals"},
                                  "signals": {t: status["signals"][t] for t in CENTRES if t in status["signals"]}})
                    last_trace = now
            # Scan warmup as well as observation. A red/yellow stop is recorded
            # as a candidate diagnostic, not silently called a lane-change jam.
            if now >= last_deadlock_scan + 5 - 1e-6:
                last_deadlock_scan = now
                for vid, (lane, position, speed, length) in observations.items():
                    if speed >= .1 or lane.startswith(":") or now - stopped_since.get(vid, now) <= 300:
                        continue
                    try:
                        if (controller.topology.lanes[lane].length - position >= length + 1
                                or traci.vehicle.getWaitingTime(vid) <= 300 or traci.vehicle.getLeader(vid, 10) is not None):
                            continue
                        state = traci.vehicle.getLaneChangeState(vid, 1)[0] | traci.vehicle.getLaneChangeState(vid, -1)[0]
                        if not (state & tc.LCA_STRATEGIC and state & tc.LCA_URGENT):
                            continue
                        lights = traci.vehicle.getNextTLS(vid)
                        blocking = next((light for light in lights if light[2] < length + 2 and light[3] not in "gG"), None)
                        diagnostic = {"lane": lane, "time": now, "stopped_seconds": now - stopped_since[vid],
                                      "lane_change_state": state, "period": "observation" if now >= target else "warmup"}
                        if blocking:
                            deadlock_candidates[vid] = dict(diagnostic, reason="signal_currently_blocks_head", next_tls=list(blocking))
                        else:
                            lane_change_deadlocks.setdefault(vid, dict(diagnostic, reason="urgent_strategic_head_no_leader_no_red_signal"))
                    except Exception as error:
                        diagnostic_errors["lane_change_deadlock_scan"] = type(error).__name__ + ": " + str(error)
            if now >= last_checkpoint + 60 - 1e-6:
                checkpoint = folder / f"checkpoint-{checkpoint_number % 2}.xml.gz"
                traci.simulation.saveState(str(checkpoint))
                checkpoints = [row for row in checkpoints if row["file"] != checkpoint.name]
                checkpoints.append({"time": now, "file": checkpoint.name})
                checkpoint_number += 1
                last_checkpoint = now
            previous_time = now
    except Exception as error:
        evaluation_error = type(error).__name__ + ": " + str(error)
    finally:
        if connected:
            try:
                now = traci.simulation.getTime()
                # IDList is the independent conservation count. A vehicle can
                # disappear without an arrival after a collision/removal.
                active_ids = set(traci.vehicle.getIDList())
                final_vehicles = {vid: final_vehicles.get(vid) for vid in active_ids}
            except Exception as error:
                evaluation_error = evaluation_error or ("final_state: " + type(error).__name__ + ": " + str(error))
            if controller is not None:
                try:
                    status = controller.status()
                except Exception as error:
                    diagnostic_errors["controller_status"] = type(error).__name__ + ": " + str(error)
                current_solver_events = getattr(controller, "solver_events", [])
                solver_events.extend(dict(event) for event in current_solver_events[solver_count:])
            try:
                traci.close()
            except Exception as error:
                evaluation_error = evaluation_error or ("close: " + type(error).__name__ + ": " + str(error))
        try:
            hashes_end = input_hashes(model_path)
            case_inputs_end = {key: hashlib.sha256(Path(build[key]).read_bytes()).hexdigest() for key in ("network", "routes") if key in build}
            snapshot_verified = snapshot_verified and verify_snapshot(output, hashes)
        except Exception as error:
            evaluation_error = evaluation_error or ("input_receipt: " + type(error).__name__ + ": " + str(error))
    due = {v: t for v, t in scheduled.items() if t <= now}
    pending_delay = sum(now - t for v, t in due.items() if v not in departed_ids)
    time_loss, insertion_delay, maximum_driver_delay = 0.0, 0.0, 0.0
    ramp_boundary_completed = 0
    if trip_path.exists():
        try:
            for row in ET.parse(trip_path).getroot().findall("tripinfo"):
                if float(row.get("depart", "-1")) >= 0:
                    time_loss += float(row.get("timeLoss", "0"))
                    insertion_delay += max(0, float(row.get("departDelay", "0")))
                    maximum_driver_delay = max(maximum_driver_delay, float(row.get("waitingTime", "0")) + max(0, float(row.get("departDelay", "0"))))
                    if row.get("id") in cohorts["on_ramp"] and target <= float(row.get("arrival", "-1")) <= now:
                        ramp_boundary_completed += 1
        except Exception as error:
            evaluation_error = evaluation_error or ("trip_receipt: " + type(error).__name__ + ": " + str(error))
    maximum_driver_delay = max(maximum_driver_delay, max((now - t for v, t in due.items() if v not in departed_ids), default=0))
    insertion_ages = [max(0, min(now, departure_times.get(vid, now)) - when) for vid, when in due.items()
                      if departure_times.get(vid, now) >= target]
    solver_costs = [event.get("elapsed_ms", event.get("duration_ms")) for event in solver_events]
    solver_costs = [value for value in solver_costs if isinstance(value, (int, float))]
    scope = controller.topology.metadata() if controller is not None else {}
    if family == "intelligent" and controller is not None:
        scope["native_green_timing_signals"] = sorted(controller.native_ids)
        scope["reference_phase_counts"] = controller.reference_counts
        scope["joint_signal_groups"] = {key: group.follower for key, group in controller.groups.items()}
    result = {"evaluation_schema": SCHEMA_VERSION, "policy": policy, "controller_family": family,
              "scenario": scenario, "hour": hour, "seed": seed, "predictive": predictive, "initialization": initialization, "network_variant": network_variant, "driver_profile": driver_profile,
              "driver_variation": build.get("driver_variation", {}), "model_repairs": build.get("model_repairs", {}),
              "observation_minutes": minutes, "warmup_minutes": 15, "actual_end": now, "complete": abs(now - end) < .01,
              "safe": collisions == 0 and teleports == 0, "lane_change_deadlocks": lane_change_deadlocks,
              "lane_change_deadlock_candidates": deadlock_candidates, "diagnostic_errors": diagnostic_errors,
              "collision_vehicle_reports": collisions, "collision_events": collision_events, "teleports": teleports,
              "collision_details": collision_details, "evaluation_error": evaluation_error,
              "due": len(due), "inserted": inserted, "completed": arrived, "active": len(final_vehicles),
              "planned_due_by_cutoff": len(scheduled),
              "not_yet_due_at_termination": max(0, len(scheduled) - len(due)),
              "uncompleted_planned_demand": max(0, len(scheduled) - arrived),
              "completed_in_observation": arrived - completed_at_activation if activated else 0,
              "inserted_in_observation": inserted - inserted_at_activation if activated else 0,
              "control_evaluations_in_observation": getattr(controller, "control_evaluations", 0) - evaluations_at_activation if activated else 0,
              "phase_switches_in_observation": sum(signal.switches for signal in controller.signals.values()) - switches_at_activation if activated else 0,
              "pending": max(0, len(due) - inserted), "max_pending": max_pending, "max_stopped": max_stopped,
              "system_vehicle_seconds": round(system_seconds, 2), "stopped_vehicle_seconds": round(stopped_seconds, 2),
              "accrued_loss_with_insertion_seconds": round(time_loss + insertion_delay + pending_delay, 2),
              "spillback_lane_seconds": round(spillback_lane_seconds, 2), "max_observed_service_gap_seconds": round(max_service_age, 2),
              "ramp_entered_merge_section_new_cohort": len(ramp_passed),
              "ramp_route_completions": ramp_boundary_completed, "ramp_lanes_used": sorted(lane_usage),
              "off_ramp_edge_exit_crossings": sum(event["ramp"] == "off_ramp" for event in crossing_events),
              "on_ramp_edge_exit_crossings": sum(event["ramp"] == "on_ramp" for event in crossing_events),
              "off_ramp_downstream_edge_exit_crossings": sum(event["ramp"] == "off_ramp_downstream" for event in crossing_events),
              "ramp_edge_sets": {key: sorted(edges) for key, edges in edge_sets.items()},
              "peak_halfhour_bins": peak_bins(target, now, end, crossing_events, cohorts, departure_times, system_boundaries),
              "fairness": {"vehicle_max_continuous_stopped_seconds": distribution(stopped_peaks.values()),
                           "incoming_lane_max_queued_service_age_seconds": distribution(movement_peaks.values()),
                           "boundary_backlog_age_seconds": distribution([now - when for vid, when in due.items() if vid not in departed_ids]),
                           "boundary_insertion_delay_seconds": distribution(insertion_ages)},
              "latency": {"decision_evaluation_ms": distribution([event["elapsed_ms"] for event in decision_events]),
                          "observation_decision_evaluation_ms": distribution([event["elapsed_ms"] for event in decision_events if event["period"] == "observation"]),
                          "solver_event_ms": distribution(solver_costs)},
              "solver_status_counts": {str(key): sum(event.get("status", "unknown") == key for event in solver_events)
                                       for key in sorted({event.get("status", "unknown") for event in solver_events}, key=str)},
              "solver_fallback_events": sum(bool(event.get("fallback", event.get("fallback_reason"))) for event in solver_events),
              "solver_max_mass_residual": max((abs(event["mass_residual"]) for event in solver_events if isinstance(event.get("mass_residual"), (int, float))), default=0),
              "maximum_waiting_plus_insertion_seconds": round(maximum_driver_delay, 2),
              "wall_seconds": round(time.perf_counter() - started, 2),
              "controller": {k: v for k, v in status.items() if k != "signals"},
              "settings": asdict(settings), "tuning": TUNING, "inputs": hashes, "inputs_start": hashes_start,
              "inputs_end": hashes_end, "input_changed": hashes_start != hashes or hashes_end != hashes,
              "case_inputs_start": case_inputs_start, "case_inputs_end": case_inputs_end, "checkpoints": checkpoints,
              "network_sha256": semantic_network_hash(build["network"]) if "network" in build else None,
              "network_raw_sha256": case_inputs_start.get("network"),
              "frozen_inputs_verified": snapshot_verified,
              "demand_sha256": hashlib.sha256(Path(build["routes"]).read_bytes()).hexdigest() if "routes" in build else None,
              "scope": scope, "sumo_version": sumo_version,
              "metric_note": "System/stopped vehicle-seconds integrate every observation step, including pending boundary demand. Ramp edge-exit crossings are actual transitions; route completions remain the unchanged score metric. Halfhour bins use [begin,end), offered due demand is evaluator-only, and coverage_complete is required to interpret zero crossings. Fairness p95/max summarize each vehicle's maximum observed continuous stopped age, each queued incoming lane's maximum observed service age, and exact insertion delays for vehicles overlapping observation; undefined populations are null. Deadlock scans include warmup; red-signal candidates are diagnostic only."}
    conservation = inserted - arrived - len(final_vehicles)
    if conservation:
        result["conservation_error"] = conservation
        result["unexplained_removals"] = max(0, conservation)
        result["missing_departed_vehicle_ids"] = sorted(departed_ids - set(final_vehicles) - all_arrived_ids)
    result["invalid_reasons"] = validity_reasons(result)
    result["valid"] = not result["invalid_reasons"]
    (folder / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    (folder / "trace.json").write_text(json.dumps(trace, indent=2) + "\n")
    (folder / "decision-events.json").write_text(json.dumps(decision_events, indent=2) + "\n")
    (folder / "solver-events.json").write_text(json.dumps(solver_events, indent=2) + "\n")
    (folder / "ramp-crossings.json").write_text(json.dumps(crossing_events, indent=2) + "\n")
    if controller is not None and hasattr(controller, "shared_state"):
        (folder / "shared-state.json").write_text(json.dumps(controller.shared_state, indent=2) + "\n")
    if collect_forecast and controller is not None and hasattr(controller, "forecast_samples"):
        episode_id = ":".join(str(value) for value in (family, scenario, seed, hour, minutes, policy, initialization, predictive, network_variant, driver_profile))
        diagnostic_prefix = not result["valid"]
        failure_time = min((row["time"] for row in lane_change_deadlocks.values()), default=now) if diagnostic_prefix else None
        samples = [dict(sample, episode_id=episode_id, episode_valid=result["valid"], valid=result["valid"],
                        invalid_reasons=list(result["invalid_reasons"]), diagnostic_prefix=diagnostic_prefix, failure_time=failure_time)
                   for sample in controller.forecast_samples
                   if not diagnostic_prefix or sample.get("time", float("inf")) + settings.interval <= failure_time]
        exported = samples if result["valid"] or collect_diagnostic_prefix else []
        (folder / "forecast-samples.json").write_text(json.dumps(exported, indent=2) + "\n")
        result["forecast_collection"] = {"episode_id": episode_id, "episode_valid": result["valid"],
            "diagnostic_prefix": diagnostic_prefix, "failure_time": failure_time,
            "available_sample_count": len(controller.forecast_samples), "prefix_sample_count": len(samples), "exported_sample_count": len(exported)}
        (folder / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def failed_case(case, error):
    """Keep a receipt even if a worker itself exits before it can write one."""
    policy, scenario, hour, seed, minutes, output, hashes, edits, predictive, family, initialization, *options = case
    result = {"evaluation_schema": SCHEMA_VERSION, "policy": policy, "controller_family": family, "scenario": scenario,
              "hour": hour, "seed": seed, "observation_minutes": minutes, "warmup_minutes": 15,
              "initialization": initialization, "network_variant": options[3] if len(options) > 3 else "original", "driver_profile": options[4] if len(options) > 4 else "nominal", "predictive": predictive, "valid": False, "safe": False,
              "complete": False, "evaluation_error": type(error).__name__ + ": " + str(error),
              "actual_end": hour * 3600 - 900, "inputs": hashes, "inputs_end": {}, "frozen_inputs_verified": False,
              "demand_sha256": None, "scope": {}, "collision_vehicle_reports": 0, "teleports": 0,
              "inserted": 0, "completed": 0, "active": 0, "pending": 0, "completed_in_observation": 0,
              "system_vehicle_seconds": 0, "stopped_vehicle_seconds": 0, "ramp_route_completions": 0, "wall_seconds": 0}
    result["invalid_reasons"] = validity_reasons(result)
    folder = Path(output) / (f"{scenario}-{hour}-{seed}-{policy}" + ("-no-forecast" if not predictive else ""))
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", choices=["adaptive", "intelligent"], default="adaptive")
    parser.add_argument("--driver-profile", choices=["nominal", "varied"], default="nominal")
    parser.add_argument("--network-variant", choices=["original", "route-safe"], default="original")
    parser.add_argument("--initialization", choices=["fixed", "policy"], default="fixed",
                        help="fixed: recover from identical fixed-control congestion; policy: each policy controls from the common empty start")
    parser.add_argument("--modes", nargs="+", choices=sorted(MODES | {"max_pressure"} | MPC_MODES), default=["fixed", "local", "coordinated"])
    parser.add_argument("--scenarios", nargs="+", choices=sorted(TRAFFIC_MODES), default=["baseline", "merge"])
    parser.add_argument("--hours", nargs="+", type=int, choices=[16, 17], default=[16, 17])
    parser.add_argument("--seeds", nargs="+", type=int, default=[42, 43])
    parser.add_argument("--minutes", type=int, default=10)
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--no-forecast", action="store_true", help="Prediction ablation; keep storage, legal phases, and objective")
    parser.add_argument("--forecast-model", type=Path)
    parser.add_argument("--collect-forecast", action="store_true", help="Save MPC forecast_samples per case")
    parser.add_argument("--collect-diagnostic-prefix", action="store_true", help="Explicitly export invalid episode prefixes, labeled with validity and failure time")
    parser.add_argument("--forecast-data", type=Path, help="Optional combined collection output; implies --collect-forecast")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--lost-time-amortization", type=float, default=None)
    parser.add_argument("--stall-seconds", type=float, default=None)
    args = parser.parse_args()
    for key in ("lost_time_amortization", "stall_seconds"):
        if getattr(args, key) is not None:
            TUNING[key] = getattr(args, key)
    os.environ["TC_TUNING"] = json.dumps(TUNING)
    if not 1 <= args.minutes <= 150 or not 1 <= args.jobs <= 4:
        parser.error("Use 1–150 observation minutes and 1–4 workers")
    if any(mode in MPC_MODES | {"max_pressure"} for mode in args.modes) and args.controller != "intelligent":
        parser.error("mpc, mpc_learned, and max_pressure use the intelligent controller family")
    if "mpc_learned" in args.modes and not args.forecast_model:
        parser.error("mpc_learned requires --forecast-model")
    model_path = str(args.forecast_model.resolve()) if args.forecast_model else None
    if model_path and not Path(model_path).is_file():
        parser.error("Forecast model does not exist")
    if "mpc_learned" in args.modes:
        from traffic_forecasting import ArrivalPredictor
        predictor = ArrivalPredictor("learned", model_path)
        if predictor.model is None:
            parser.error("mpc_learned requires a validated frozen forecast artifact: " + str(predictor.model_error))
    args.out.mkdir(parents=True, exist_ok=False)
    hashes = input_hashes(model_path)
    for name, source in input_paths(model_path).items():
        destination = args.out / name if name.startswith("implementation/") else args.out / "inputs" / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
    if input_hashes(model_path) != hashes or not verify_snapshot(args.out, hashes):
        raise SystemExit("Source/input changed while freezing the evaluation snapshot")
    edits = ScenarioEditor(network_variant=args.network_variant).read()
    (args.out / "inputs.json").write_text(json.dumps(hashes, indent=2) + "\n")
    cases = [(p, s, h, seed, args.minutes, str(args.out.resolve()), hashes, edits, not args.no_forecast,
              args.controller, args.initialization, model_path, args.collect_forecast or bool(args.forecast_data), args.collect_diagnostic_prefix, args.network_variant, args.driver_profile)
             for seed, s, h, p in itertools.product(args.seeds, args.scenarios, args.hours, args.modes)]
    manifest = {"case_count": len(cases), "cases": [list(case[:5]) for case in cases], "complete": False,
                "evaluation_schema": SCHEMA_VERSION, "inputs_start": hashes}
    (args.out / "run.json").write_text(json.dumps(manifest, indent=2) + "\n")
    results = []
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        futures = {pool.submit(run_case, case): case for case in cases}
        for future in as_completed(futures):
            try:
                result = future.result()
            except Exception as error:
                result = failed_case(futures[future], error)
            results.append(result)
            (args.out / "results.json").write_text(json.dumps(results, indent=2) + "\n")
            print(json.dumps({key: result.get(key) for key in ("policy", "scenario", "hour", "seed", "safe", "valid", "invalid_reasons", "completed", "pending", "system_vehicle_seconds", "ramp_route_completions", "wall_seconds")}), flush=True)
    manifest["complete"] = len(results) == len(cases)
    manifest["inputs_end"] = input_hashes(model_path)
    manifest["valid"] = manifest["complete"] and batch_valid(results) and manifest["inputs_end"] == hashes
    (args.out / "run.json").write_text(json.dumps(manifest, indent=2) + "\n")
    if args.forecast_data:
        samples = []
        for path in sorted(args.out.glob("*/forecast-samples.json")):
            samples.extend(json.loads(path.read_text()))
        args.forecast_data.parent.mkdir(parents=True, exist_ok=True)
        with args.forecast_data.open("x") as destination:
            json.dump(samples, destination, indent=2)
            destination.write("\n")
    print(f"Saved {len(results)} run receipts to {args.out.resolve() / 'results.json'}", flush=True)
    if not manifest["valid"]:
        raise SystemExit("At least one run failed validity; inspect invalid_reasons before reporting controller success.")


if __name__ == "__main__":
    main()
