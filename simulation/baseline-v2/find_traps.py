#!/usr/bin/env python3
"""List car lanes that cars can drive into but never leave (dead-end lanes).

Usage: python find_traps.py [network]   (default: baseline-v2.net.xml)
Lanes at the edge of the map are skipped (cars leave the simulation there).
"""
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from sumo_runtime import configure  # noqa: E402


def main():
    configure()
    import sumolib
    net = sumolib.net.readNet(sys.argv[1] if len(sys.argv) > 1 else str(HERE / "baseline-v2.net.xml"))
    feeders = {}
    for edge in net.getEdges():
        for lane in edge.getLanes():
            for c in lane.getOutgoing():
                if c.getFromLane().allows("passenger"):
                    feeders[c.getToLane().getID()] = feeders.get(c.getToLane().getID(), 0) + 1
    traps = []
    for edge in net.getEdges():
        if edge.getFunction() == "internal" or not edge.getToNode().getOutgoing():
            continue
        for lane in edge.getLanes():
            if not lane.allows("passenger"):
                continue
            exits = [c for c in lane.getOutgoing() if c.getToLane().allows("passenger")]
            if not exits:
                traps.append((lane.getID(), lane.getLength(), feeders.get(lane.getID(), 0)))
    for lane_id, length, fed in traps:
        print(f"{lane_id}  length {length:.1f} m  fed by {fed} car connections")
    print("car lanes with no exit:", len(traps))


if __name__ == "__main__":
    main()
