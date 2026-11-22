#!/usr/bin/env python3
"""Run one controller for one full evening on baseline-v2 and score it.

Every policy sees the same network, demand, seed and SUMO settings
(3:45-6:30 PM, 0.2 s steps, cars are never removed, junction collisions are
checked). Controllers run the fixed-time plan during the 15-minute warm-up and
take over at 4:00 PM, as in evaluate_controller.py.

Policies (how much information each signal uses):
  webster            fixed-time plan, no sensors (the baseline)
  fixed              controllers' own fixed reference: same Webster green times,
                     rebuilt as protected phases with all-red
  local              each signal uses only its own approach queues
  coordinated        signals also use neighbours' plans and downstream space
  int-fixed          intelligent coordinator's fixed reference (Webster greens,
                     SUMO's own phases), no sensors
  int-local          intelligent coordinator, own queues only
  int-coordinated    intelligent coordinator with neighbour information
  max_pressure       max-pressure (queue in minus queue out), whole network
  mpc                network model-predictive control (whole network)
  anchored           keeps the fixed-time plan and adjusts it only for empty
                     greens, full exits and queues about to spill back
                     (anchored_controller.py)

Usage: python compare_controllers.py --policy coordinated --seed 42 --json out.json
"""
from pathlib import Path
import argparse
import json
import sys
import time
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
SIM = HERE.parent if (HERE.parent / "adaptive_controller.py").exists() else HERE
sys.path.insert(0, str(SIM))
from sumo_runtime import configure  # noqa: E402

REAL_053167 = [834, 810, 910, 837]
RAMP_EXIT = "993553211"
START, ACTIVATE, DEMAND_END, END = 56700, 57600, 64800, 66600
POLICIES = ["webster", "fixed", "local", "coordinated", "int-fixed", "int-local", "int-coordinated", "max_pressure", "mpc", "anchored"]


def clock(t):
    return "%d:%02d" % (t // 3600 - 12, t % 3600 // 60)


def bake_signals(net, add, out):
    """Copy of the network with the fixed-time plan written in, so the
    controllers read the same green times as the webster baseline."""
    import re
    text = Path(net).read_text(encoding="utf-8")
    blocks = {m.group(1): m.group(0) for m in
              re.finditer(r'<tlLogic id="([^"]+)".*?</tlLogic>', Path(add).read_text(encoding="utf-8"), re.S)}

    def swap(m):
        new = blocks.get(m.group(1))
        return re.sub(r'programID="[^"]*"', 'programID="0"', new, count=1) if new else m.group(0)
    Path(out).write_text(re.sub(r'<tlLogic id="([^"]+)".*?</tlLogic>', swap, text, flags=re.S), encoding="utf-8")


def make_controller(traci, policy, network):
    import os
    from adaptive_controller import AdaptiveController, Settings as BaseSettings

    def Settings():
        # Same tuning hook as evaluate_controller.py: TC_TUNING='{"lost_time_amortization": 3}'
        return BaseSettings(**json.loads(os.environ.get("TC_TUNING", "{}")))
    if policy == "webster":
        return None
    if policy in ("fixed", "local", "coordinated"):
        return AdaptiveController(traci, network, "fixed", Settings())
    if policy == "mpc":
        import os
        from network_mpc import NetworkMPC
        # Planning time budget per 5 s decision (team default 250 ms). When many
        # runs share one CPU, wall-clock time stretches; MPC_DEADLINE_MS lets a
        # run give the solver the time it would have on its own processor.
        NetworkMPC.deadline_ms = float(os.environ.get("MPC_DEADLINE_MS", NetworkMPC.deadline_ms))
        controller = NetworkMPC(traci, network, mode="mpc", settings=Settings())
        controller.mode = "fixed"
        return controller
    if policy == "anchored":
        from anchored_controller import AnchoredController
        return AnchoredController(traci, network, "fixed", Settings())
    from intelligent_coordinator import IntelligentCoordinator
    return IntelligentCoordinator(traci, network, "fixed", Settings())


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--policy", choices=POLICIES, required=True)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--net", default=str(HERE / "baseline-v2.net.xml"))
    p.add_argument("--webster-net", default=str(HERE / "baseline-v2-webster.net.xml"),
                   help="network with the fixed-time plan written in (controllers read their reference timing from it)")
    p.add_argument("--signals", default=str(HERE / "baseline-v2-signals.add.xml"))
    p.add_argument("--routes", default=str(HERE / "baseline-v2.rou.xml"))
    p.add_argument("--end", type=float, default=END)
    p.add_argument("--json")
    p.add_argument("--collision-output", help="write SUMO's collision list (where and when) to this file")
    a = p.parse_args()
    _, sumo = configure()
    import traci
    import traci.constants as tc

    if a.policy != "webster" and not Path(a.webster_net).exists():
        bake_signals(a.net, a.signals, a.webster_net)
    network = a.net if a.policy == "webster" else a.webster_net
    tripinfo = Path(a.json or f"{a.policy}-{a.seed}.json").with_suffix(".tripinfo.xml")
    cmd = [str(sumo), "-n", network, "-r", a.routes, "-b", str(START), "--step-length", "0.2",
           "--lateral-resolution", "0.4", "--time-to-impatience", "30", "--time-to-teleport", "-1",
           "--collision.check-junctions", "true", "--collision.action", "warn",
           "--tripinfo-output", str(tripinfo), "--seed", str(a.seed),
           "--no-step-log", "true", "--no-warnings", "true"]
    if a.policy == "webster":
        cmd += ["--additional-files", a.signals]
    if a.collision_output:
        cmd += ["--collision-output", a.collision_output]
    traci.start(cmd)
    controller = make_controller(traci, a.policy, network)
    policy_mode = {"int-fixed": "fixed", "int-local": "local", "int-coordinated": "coordinated"}.get(a.policy, a.policy)
    traci.simulation.subscribe((tc.VAR_TIME, tc.VAR_DEPARTED_VEHICLES_IDS, tc.VAR_ARRIVED_VEHICLES_IDS,
                                tc.VAR_COLLIDING_VEHICLES_NUMBER, tc.VAR_TELEPORT_STARTING_VEHICLES_NUMBER))
    seen, bins, stuck = set(), {}, []
    arrived = collisions = teleports = step = 0
    waiting_at_6, vehicle_seconds, control_seconds = None, 0.0, 0.0
    first_collision = None
    now = START
    while now < a.end - 1e-6:
        traci.simulationStep()
        step += 1
        sim = traci.simulation.getSubscriptionResults()
        now = sim[tc.VAR_TIME]
        for vid in sim[tc.VAR_DEPARTED_VEHICLES_IDS]:
            traci.vehicle.subscribe(vid, (tc.VAR_LANE_ID, tc.VAR_LANEPOSITION, tc.VAR_SPEED, tc.VAR_LENGTH))
        arrived += len(sim[tc.VAR_ARRIVED_VEHICLES_IDS])
        if sim[tc.VAR_COLLIDING_VEHICLES_NUMBER] and first_collision is None:
            first_collision = clock(now)
        collisions += sim[tc.VAR_COLLIDING_VEHICLES_NUMBER]
        teleports += sim[tc.VAR_TELEPORT_STARTING_VEHICLES_NUMBER]
        if controller is not None:
            if now >= ACTIVATE and controller.mode != policy_mode:
                controller.mode = policy_mode
            started = time.perf_counter()
            controller.step(now)
            control_seconds += time.perf_counter() - started
        if step % 5 == 0:
            for v in traci.edge.getLastStepVehicleIDs(RAMP_EXIT):
                if v not in seen:
                    seen.add(v)
                    if now >= ACTIVATE:
                        b = int((now - ACTIVATE) // 1800)
                        bins[b] = bins.get(b, 0) + 1
            if now >= ACTIVATE:
                # vehicle-seconds in the system, including cars waiting to enter the map
                vehicle_seconds += (traci.vehicle.getIDCount() + len(traci.simulation.getPendingVehicles())) * 1.0
        if step % 4500 == 0:
            stuck.append((clock(now), sum(1 for v in traci.vehicle.getIDList()
                                          if traci.vehicle.getWaitingTime(v) > 180)))
        if waiting_at_6 is None and now >= DEMAND_END:
            waiting_at_6 = len(traci.simulation.getPendingVehicles())
    traci.close()
    events = getattr(controller, "solver_events", [])
    after = [e for e in events if e.get("time", 0) >= ACTIVATE]
    fallbacks = {}
    for e in after:
        if e.get("fallback"):
            fallbacks[e.get("fallback_reason")] = fallbacks.get(e.get("fallback_reason"), 0) + 1

    # Delay of finished trips (seconds lost against free flow), from SUMO tripinfo
    losses = [float(t.get("timeLoss")) for t in ET.parse(tripinfo).getroot().iter("tripinfo")
              if float(t.get("depart")) >= ACTIVATE]
    ramp = [bins.get(i, 0) for i in range(4)]
    import os
    result = dict(policy=a.policy, seed=a.seed, tuning=json.loads(os.environ.get("TC_TUNING", "{}")), ramp_per_30min_4to6pm=ramp, nysdot_053167=REAL_053167,
                  stuck_over_3min=stuck, arrived=arrived, not_yet_entered_at_6pm=waiting_at_6,
                  collisions=collisions, first_collision=first_collision, teleports=teleports,
                  vehicle_hours_in_system_4pm_on=round(vehicle_seconds / 3600, 1),
                  finished_trips_after_4pm=len(losses),
                  mean_delay_finished_s=round(sum(losses) / max(len(losses), 1), 1),
                  controller_cpu_seconds=round(control_seconds, 1),
                  mpc_decisions=len(after), mpc_fallbacks=fallbacks,
                  adjustments=getattr(controller, "adjustments", None),
                  phase_switches=sum(s.switches for s in getattr(controller, "signals", {}).values()),
                  valid=collisions == 0 and teleports == 0,
                  gridlock_at_end=stuck[-1][1] > 100 if stuck else None)
    tripinfo.unlink(missing_ok=True)
    print(f"{a.policy} seed {a.seed}: ramp {ramp}, stuck@end {stuck[-1][1] if stuck else '?'}, arrived {arrived}, "
          f"delay {result['mean_delay_finished_s']} s, veh-h {result['vehicle_hours_in_system_4pm_on']}, "
          f"collisions {collisions}, teleports {teleports}, gridlock {result['gridlock_at_end']}")
    if a.json:
        Path(a.json).write_text(json.dumps(result, indent=1))


if __name__ == "__main__":
    main()
