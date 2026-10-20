#!/usr/bin/env python3
"""Paired, reproducible controller experiments including boundary backlog."""
import argparse
import bisect
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict
import hashlib
import itertools
import json
import os
from pathlib import Path
import shutil
import time
import xml.etree.ElementTree as ET

from adaptive_controller import AdaptiveController, Settings, MODES, CENTRES
from intelligent_coordinator import IntelligentCoordinator
from scenario_editor import ScenarioEditor, SUMO, DATA
from traffic_behavior import MODES as TRAFFIC_MODES
import traci
import traci.constants as tc


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


def input_hashes():
    names = ("user-network.net.xml", "astoria-corrected.net.xml", "scenario-full-area.rou.xml", "full-area-demand.json", "scenario-edits.json")
    result = {n: hashlib.sha256((DATA / n).read_bytes()).hexdigest() for n in names if (DATA / n).exists()}
    for name in ("adaptive_controller.py", "intelligent_coordinator.py", "signal_groups.py", "traffic_behavior.py", "scenario_editor.py", "evaluate_controller.py", "score_evaluation.py"):
        result["implementation/" + name] = hashlib.sha256((Path(__file__).parent / name).read_bytes()).hexdigest()
    return result


# Spawned workers re-import this module; tuning travels through the environment.
TUNING = json.loads(os.environ.get("TC_TUNING", "{}"))


def run_case(case):
    policy, scenario, hour, seed, minutes, output, hashes, edits, predictive, family, initialization = case
    if input_hashes() != hashes:
        raise RuntimeError("Simulation inputs changed during evaluation. Rerun to obtain a matched comparison.")
    name = f"{scenario}-{hour}-{seed}-{policy}" + ("-no-forecast" if not predictive else "")
    folder = Path(output) / name
    folder.mkdir(parents=True, exist_ok=True)
    target, end = hour * 3600, hour * 3600 + minutes * 60
    build = ScenarioEditor().build(edits, folder, scenario, target - 900, seed)
    scheduled = roster(build["routes"], end)
    planned_times = sorted(scheduled.values())
    trip_path = folder / "trips.xml"
    args = build["args"] + ["--end", str(end), "--tripinfo-output", str(trip_path),
                            "--tripinfo-output.write-unfinished", "true", "--tripinfo-output.write-undeparted", "true",
                            "--collision-output", str(folder / "collisions.xml"), "--log", str(folder / "sumo.log")]
    started = time.perf_counter()
    traci.start([str(SUMO)] + args)
    sumo_version = traci.getVersion()[1]
    settings = Settings(predictive=predictive, **TUNING)
    controller_class = IntelligentCoordinator if family == "intelligent" else AdaptiveController
    controller = controller_class(traci, build["network"], policy if initialization == "policy" else "fixed", settings)
    settings = controller.settings
    traci.simulation.subscribe((tc.VAR_TIME, tc.VAR_DEPARTED_VEHICLES_IDS, tc.VAR_ARRIVED_VEHICLES_IDS,
                                tc.VAR_COLLIDING_VEHICLES_NUMBER, tc.VAR_TELEPORT_STARTING_VEHICLES_NUMBER))
    inserted, departed_ids, arrived = 0, set(), 0
    system_seconds = stopped_seconds = spillback_lane_seconds = 0.0
    ramp_passed, ramp_seen, lane_usage = set(), set(), set()
    collisions, collision_events, teleports = 0, [], 0
    collision_details = []
    lane_change_deadlocks = {}
    trace, max_pending, max_stopped, max_service_age = [], 0, 0, 0.0
    last_sample, decision_costs = target - 1, []
    activated = False
    completed_at_activation = inserted_at_activation = 0
    evaluations_at_activation = switches_at_activation = 0
    try:
        while traci.simulation.getTime() < end:
            traci.simulationStep()
            sim = traci.simulation.getSubscriptionResults()
            now = sim[tc.VAR_TIME]
            for vid in sim[tc.VAR_DEPARTED_VEHICLES_IDS]:
                departed_ids.add(vid)
                inserted += 1
                traci.vehicle.subscribe(vid, (tc.VAR_LANE_ID, tc.VAR_LANEPOSITION, tc.VAR_SPEED, tc.VAR_LENGTH, tc.VAR_TIMELOSS))
            arrived += len(sim[tc.VAR_ARRIVED_VEHICLES_IDS])
            collisions += sim[tc.VAR_COLLIDING_VEHICLES_NUMBER]
            teleports += sim[tc.VAR_TELEPORT_STARTING_VEHICLES_NUMBER]
            if collisions or teleports:
                events = traci.simulation.getCollisions()
                collision_events.extend(str(event) for event in events)
                for event in events:
                    collision_details.append({"time": now, "collision": str(event), "vehicles": {
                        vehicle: {"lane": traci.vehicle.getLaneID(vehicle), "previous_lane": controller.last_external.get(vehicle),
                                  "position": traci.vehicle.getLanePosition(vehicle), "angle": traci.vehicle.getAngle(vehicle)}
                        for vehicle in (event.collider, event.victim)}})
                break
            if now >= target and not activated:
                completed_at_activation, inserted_at_activation = arrived, inserted
                evaluations_at_activation = getattr(controller, "control_evaluations", 0)
                switches_at_activation = sum(signal.switches for signal in controller.signals.values())
                controller.mode = policy
                controller.forecast_errors.clear()
                activated = True
            controller.step(now)
            if now >= target and now >= last_sample + 1 - 1e-6:
                elapsed = min(1.0, now - max(target, last_sample))
                observations = controller.vehicle_observations()
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
                        max_service_age = max(max_service_age, now - controller.last_service.get(lane, now))
                for vid, (lane, position, speed, length) in observations.items():
                    if lane.startswith("25345887_"):
                        ramp_seen.add(vid)
                        lane_usage.add(lane)
                    if vid in ramp_seen and lane.startswith("1134690192_"):
                        ramp_passed.add(vid)
                if now - last_sample >= 1 - 1e-6:
                    last_sample = now
                if int(round((now - target) * 5)) % 150 == 0:
                    status = controller.status()
                    trace.append({"time": round(now, 1), "active": active, "stopped": stopped, "pending": pending,
                                  "ramp_entered_merge_section_new_cohort": len(ramp_passed),
                                  "controller": {k: v for k, v in status.items() if k != "signals"},
                                  "signals": {t: status["signals"][t] for t in CENTRES if t in status["signals"]}})
                    decision_costs.append(controller.elapsed_ms)
                    # Microscopic artifact check: a car stopped > 300 s that still
                    # needs an urgent strategic lane change is a lane-change deadlock
                    # (e.g. on sub-car-length edges), not a signal-control outcome.
                    for vid, (lane, position, speed, length) in observations.items():
                        if (speed < .1 and not lane.startswith(":") and traci.lane.getLength(lane) - position < length + 1
                                and traci.vehicle.getWaitingTime(vid) > 300 and traci.vehicle.getLeader(vid, 10) is None):
                            state = traci.vehicle.getLaneChangeState(vid, 1)[0] | traci.vehicle.getLaneChangeState(vid, -1)[0]
                            if state & tc.LCA_STRATEGIC and state & tc.LCA_URGENT:
                                lane_change_deadlocks[vid] = lane
        now = traci.simulation.getTime()
        final_vehicles = controller.vehicle_observations()
    finally:
        traci.close()
    due = {v: t for v, t in scheduled.items() if t <= now}
    pending_delay = sum(now - t for v, t in due.items() if v not in departed_ids)
    time_loss, insertion_delay, maximum_driver_delay = 0.0, 0.0, 0.0
    routes_root = ET.parse(build["routes"]).getroot()
    ramp_routes = {r.get("id") for r in routes_root.findall("route") if "25345887" in r.get("edges").split()}
    ramp_trip_ids = {v.get("id") for v in routes_root.findall("vehicle") if v.get("route") in ramp_routes}
    for flow in routes_root.findall("flow"):
        if flow.get("route") in ramp_routes:
            ramp_trip_ids.update(f'{flow.get("id")}.{i}' for i in range(int(flow.get("number"))))
    ramp_boundary_completed = 0
    if trip_path.exists():
        for row in ET.parse(trip_path).getroot().findall("tripinfo"):
            if float(row.get("depart", "-1")) >= 0:
                time_loss += float(row.get("timeLoss", "0"))
                insertion_delay += max(0, float(row.get("departDelay", "0")))
                maximum_driver_delay = max(maximum_driver_delay, float(row.get("waitingTime", "0")) + max(0, float(row.get("departDelay", "0"))))
                if row.get("id") in ramp_trip_ids and target <= float(row.get("arrival", "-1")) <= now:
                    ramp_boundary_completed += 1
    maximum_driver_delay = max(maximum_driver_delay, max((now - t for v, t in due.items() if v not in departed_ids), default=0))
    status = controller.status()
    result = {"policy": policy, "controller_family": family, "scenario": scenario, "hour": hour, "seed": seed, "predictive": predictive,
              "initialization": initialization,
              "observation_minutes": minutes, "warmup_minutes": 15, "actual_end": now,
              "safe": collisions == 0 and teleports == 0, "lane_change_deadlocks": lane_change_deadlocks,
              "valid": not lane_change_deadlocks, "collision_vehicle_reports": collisions,
              "collision_events": collision_events, "teleports": teleports,
              "collision_details": collision_details,
              "due": len(due), "inserted": inserted, "completed": arrived, "active": len(final_vehicles),
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
              "maximum_waiting_plus_insertion_seconds": round(maximum_driver_delay, 2),
              "wall_seconds": round(time.perf_counter() - started, 2),
              "controller": {k: v for k, v in status.items() if k != "signals"},
              "settings": asdict(settings), "tuning": TUNING, "inputs": hashes,
              "demand_sha256": hashlib.sha256(Path(build["routes"]).read_bytes()).hexdigest(),
              "scope": controller.topology.metadata(), "sumo_version": sumo_version,
              "metric_note": "System/stopped vehicle-seconds integrate the observation window. Ramp route completions count arrivals at the outer exit during this window, including warm-up vehicles. Accrued loss includes warm-up, finished/unfinished trips, and pending insertion. Due includes warm-up arrivals; future demand is excluded."}
    if result["inserted"] != result["completed"] + result["active"]:
        result["conservation_error"] = result["inserted"] - result["completed"] - result["active"]
    if family == "intelligent":
        result["scope"]["native_green_timing_signals"] = sorted(controller.native_ids)
        result["scope"]["reference_phase_counts"] = controller.reference_counts
        result["scope"]["joint_signal_groups"] = {key: group.follower for key, group in controller.groups.items()}
    (folder / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    (folder / "trace.json").write_text(json.dumps(trace, indent=2) + "\n")
    if hasattr(controller, "shared_state"):
        (folder / "shared-state.json").write_text(json.dumps(controller.shared_state, indent=2) + "\n")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", choices=["adaptive", "intelligent"], default="adaptive")
    parser.add_argument("--initialization", choices=["fixed", "policy"], default="fixed",
                        help="fixed: recover from identical fixed-control congestion; policy: each policy controls from the common empty start")
    parser.add_argument("--modes", nargs="+", choices=sorted(MODES | {"max_pressure"}), default=["fixed", "local", "coordinated"])
    parser.add_argument("--scenarios", nargs="+", choices=sorted(TRAFFIC_MODES), default=["baseline", "merge"])
    parser.add_argument("--hours", nargs="+", type=int, choices=[16, 17], default=[16, 17])
    parser.add_argument("--seeds", nargs="+", type=int, default=[42, 43])
    parser.add_argument("--minutes", type=int, default=10)
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--no-forecast", action="store_true", help="Prediction ablation; keep storage, legal phases, and objective")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--lost-time-amortization", type=float, default=None)
    parser.add_argument("--stall-seconds", type=float, default=None)
    args = parser.parse_args()
    for key in ("lost_time_amortization", "stall_seconds"):
        if getattr(args, key) is not None:
            TUNING[key] = getattr(args, key)
    os.environ["TC_TUNING"] = json.dumps(TUNING)
    if not 1 <= args.minutes <= 60 or not 1 <= args.jobs <= 4:
        parser.error("Use 1–60 observation minutes and 1–4 workers")
    args.out.mkdir(parents=True, exist_ok=False)
    snapshot_folder = args.out / "implementation"
    snapshot_folder.mkdir(exist_ok=True)
    for name in ("adaptive_controller.py", "intelligent_coordinator.py", "signal_groups.py", "traffic_behavior.py", "scenario_editor.py", "evaluate_controller.py", "sumo_runtime.py", "score_evaluation.py"):
        shutil.copy2(Path(__file__).parent / name, snapshot_folder / name)
    hashes, edits = input_hashes(), ScenarioEditor().read()
    input_folder = args.out / "inputs"
    input_folder.mkdir(exist_ok=True)
    for name in hashes:
        if not name.startswith("implementation/"):
            shutil.copy2(DATA / name, input_folder / name)
    (args.out / "inputs.json").write_text(json.dumps(hashes, indent=2) + "\n")
    cases = [(p, s, h, seed, args.minutes, str(args.out.resolve()), hashes, edits, not args.no_forecast, args.controller, args.initialization)
             for seed, s, h, p in itertools.product(args.seeds, args.scenarios, args.hours, args.modes)]
    manifest = {"case_count": len(cases), "cases": [list(case[:5]) for case in cases], "complete": False}
    (args.out / "run.json").write_text(json.dumps(manifest, indent=2) + "\n")
    results = []
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        futures = {pool.submit(run_case, c): c[:4] for c in cases}
        for future in as_completed(futures):
            result = future.result()
            results.append(result)
            (args.out / "results.json").write_text(json.dumps(results, indent=2) + "\n")
            print(json.dumps({k: result[k] for k in ("policy", "scenario", "hour", "seed", "safe", "completed", "pending", "system_vehicle_seconds", "ramp_route_completions", "wall_seconds")}), flush=True)
    manifest["complete"] = len(results) == len(cases)
    (args.out / "run.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Saved {len(results)} matched runs to {args.out.resolve() / 'results.json'}", flush=True)
    if any(not r["safe"] or r.get("conservation_error") for r in results):
        raise SystemExit("At least one run failed safety/conservation; do not report it as a controller success.")


if __name__ == "__main__":
    main()
