#!/usr/bin/env python3
"""Run one full evening (3:45-6:30 PM) and report whether it is a valid run.

Reports: EB off-ramp cars per 30 min (cars entering Astoria Blvd N from the
ramp, edge 993553211) next to the NYSDOT 053167 count, cars stuck > 3 min
every 15 min, arrivals, cars still waiting to enter at 6:00 PM, collisions
and teleports. A run is valid only with 0 collisions and 0 teleports.
"Gridlock" means more than 100 cars are still stuck > 3 min at 6:30 PM,
after demand ended at 6:00 PM (the network did not clear).

Usage:
  python check_run.py [--seed 42] [--net baseline-v2.net.xml]
                      [--routes baseline-v2.rou.xml] [--signals baseline-v2-signals.add.xml]
"""
from pathlib import Path
import argparse
import json
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from sumo_runtime import configure  # noqa: E402

REAL_053167 = [834, 810, 910, 837]          # 4:00-6:00 PM, cars per 30 min
RAMP_EXIT = "993553211"
START, FIRST_BIN, DEMAND_END, END = 56700, 57600, 64800, 66600


def clock(t):
    return "%d:%02d" % (t // 3600 - 12, t % 3600 // 60)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--net", default=str(HERE / "baseline-v2.net.xml"))
    p.add_argument("--routes", default=str(HERE / "baseline-v2.rou.xml"))
    p.add_argument("--signals", default=str(HERE / "baseline-v2-signals.add.xml"))
    p.add_argument("--impatience", default="30", help="--time-to-impatience in s (SUMO default 300)")
    p.add_argument("--json", help="also write the numbers to this file")
    a, extra = p.parse_known_args()
    _, sumo = configure()
    import traci
    cmd = [str(sumo), "-n", a.net, "-r", a.routes, "-b", str(START), "--step-length", "0.2",
           "--lateral-resolution", "0.4", "--time-to-impatience", a.impatience, "--time-to-teleport", "-1",
           "--seed", str(a.seed), "--no-step-log", "true", "--no-warnings", "true"]
    if a.signals and a.signals != "-":
        cmd += ["--additional-files", a.signals]
    traci.start(cmd + extra)
    seen, bins, stuck = set(), {}, []
    arrived = collisions = teleports = step = 0
    waiting_at_6 = None
    while traci.simulation.getTime() < END:
        traci.simulationStep()
        step += 1
        t = traci.simulation.getTime()
        arrived += traci.simulation.getArrivedNumber()
        collisions += traci.simulation.getCollidingVehiclesNumber()
        teleports += traci.simulation.getStartingTeleportNumber()
        if step % 5 == 0:
            for v in traci.edge.getLastStepVehicleIDs(RAMP_EXIT):
                if v not in seen:
                    seen.add(v)
                    if t >= FIRST_BIN:
                        b = int((t - FIRST_BIN) // 1800)
                        bins[b] = bins.get(b, 0) + 1
        if step % 4500 == 0:
            stuck.append((clock(t), sum(1 for v in traci.vehicle.getIDList()
                                        if traci.vehicle.getWaitingTime(v) > 180)))
        if waiting_at_6 is None and t >= DEMAND_END:
            waiting_at_6 = len(traci.simulation.getPendingVehicles())
    traci.close()
    ramp = [bins.get(i, 0) for i in range(4)]
    result = dict(seed=a.seed, ramp_per_30min_4to6pm=ramp, nysdot_053167=REAL_053167,
                  stuck_over_3min=stuck, arrived=arrived, not_yet_entered_at_6pm=waiting_at_6,
                  collisions=collisions, teleports=teleports,
                  valid=collisions == 0 and teleports == 0,
                  gridlock_at_end=stuck[-1][1] > 100 if stuck else None)
    print(f"seed {a.seed}: ramp per 30 min 4-6 PM {ramp} (NYSDOT {REAL_053167})")
    print("  cars stuck > 3 min: " + ", ".join(f"{c} {n}" for c, n in stuck))
    print(f"  arrived {arrived}, not yet entered at 6 PM {waiting_at_6}, "
          f"collisions {collisions}, teleports {teleports} -> {'VALID' if result['valid'] else 'NOT VALID'}"
          f"{', ends in gridlock' if result['gridlock_at_end'] else ', network clears'}")
    if a.json:
        Path(a.json).write_text(json.dumps(result, indent=1))


if __name__ == "__main__":
    main()
