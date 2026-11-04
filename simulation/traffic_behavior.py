"""Explicit sensitivity assumptions, not a calibrated reconstruction.

SUMO SL2015 handles real lateral motion and gap acceptance. No visual jitter,
forced teleports, collision disabling, or fabricated lane counts are used.
"""
from collections import Counter
import random
import xml.etree.ElementTree as ET

MODES = {"baseline", "merge", "disorder", "surge", "incident"}
MAINLINE = ("374433288", "1134690192", "122625709")


def apply_behavior(root, mode, begin=56700, end=64800, seed=42):
    if mode not in MODES:
        raise ValueError("Unknown traffic scenario")
    # Deterministic longitudinal following isolates the controller experiment.
    # Default acceleration noise produced a real SL2015 side collision at a
    # short lane-addition connector. Off-centre side-by-side traffic produced
    # body overlaps on curved fan-outs. Keep native continuous lane changes,
    # but centred alignment and zero noise are declared scenario assumptions.
    for kind in root.findall("vType"):
        kind.attrib.update(laneChangeModel="SL2015", latAlignment="center", maxSpeedLat="0.8",
                           minGapLat="0.25", lcCooperative="0.8", lcAssertive="1.0", lcImpatience="0", lcSigma="0", sigma="0")
    rng = random.Random(seed)
    if mode == "baseline":
        for kind in root.findall("vType"):
            kind.attrib.update(impatience="off", jmSigmaMinor="0", jmTimegapMinor="2", jmExtraGap="1", jmAdvance="0")
        return 0
    # Mixed cooperation and lateral response make the merge less orderly. Gap
    # acceptance remains neutral; the 35% share is a scenario assumption.
    ET.SubElement(root, "vType", id="merge_driver", vClass="passenger", laneChangeModel="SL2015",
                  latAlignment="center", maxSpeedLat="0.6", minGapLat="0.2", minGap="2.5",
                  lcPushy="0", lcAssertive="1.0", lcCooperative="0.1" if mode == "disorder" else "0.3", lcSpeedGain="1.5", lcImpatience="0", lcSigma="0")
    ET.SubElement(root, "vType", id="mainline", vClass="passenger", laneChangeModel="SL2015",
                  latAlignment="center", maxSpeedLat="0.7", minGapLat="0.25",
                  lcCooperative="0.15", lcKeepRight="0", lcSpeedGain="0", lcAssertive="1", lcImpatience="0", lcSigma="0")
    paths = {r.get("id"): r.get("edges").split() for r in root.findall("route")}
    mainline_counts = Counter()
    for vehicle in root.findall("vehicle"):
        edges = paths[vehicle.get("route")]
        if "374433288" in edges:
            mainline_counts[int(float(vehicle.get("depart"))) // 900] += 1
        if vehicle.get("type") == "car" and rng.random() < .35:
            vehicle.set("type", "merge_driver")
            # Lane changes still have native lateral motion. Random departures
            # and off-centre alignment failed physical collision checks here.
            vehicle.set("departPosLat", "center")
    # Tight imported connectors can overlap when a yielding driver advances
    # while another vehicle is still crossing. Conservative, shared junction
    # settings avoid that numerical failure without disabling collision checks.
    # These are experimental assumptions, not measured Astoria driver traits.
    for kind in root.findall("vType"):
        kind.attrib.update(impatience="off", jmSigmaMinor="0", jmTimegapMinor="2",
                           jmExtraGap="1", jmAdvance="0", sigma="0")
    # The downloaded map ends shortly after the merge; it does not include the
    # next exit. Represent constrained receiving traffic as an explicit demand
    # sensitivity, not a claim that the absent exit is simulated.
    ET.SubElement(root, "route", id="receiving_mainline", edges=" ".join(MAINLINE))
    added = 0
    for bucket in range(begin // 900, end // 900):
        per_lane_hour = 1750 if bucket * 900 >= 61200 else 1600
        count = max(0, round(per_lane_hour * 3 / 4) - mainline_counts[bucket])
        for index in range(count):
            depart = bucket * 900 + (index + .5) * 900 / count
            ET.SubElement(root, "vehicle", id=f"receiving_{bucket}_{index}", type="mainline",
                          route="receiving_mainline", depart=f"{depart:.2f}", departLane="random", departSpeed="desired")
        added += count
    return added


def receiving_constraint(directory, mode="merge", observation_time=57600):
    """Explicit downstream-boundary queue, standing in for the absent exit.

    Drivers retaining this lane encounter 5.5 m/s traffic at 4 pm and 4 m/s
    at 5 pm. These speeds are sensitivity settings, not measured speeds.
    Other mainline lanes retain their normal limits.
    """
    root = ET.Element("additional")
    sign = ET.SubElement(root, "variableSpeedSign", id="assumed_receiving_queue", lanes="122625709_0")
    ET.SubElement(sign, "step", time="0", speed="5.5")
    ET.SubElement(sign, "step", time="61200", speed="4")
    if mode == "incident":
        # A downstream virtual speed sign models a receiving-lane obstruction.
        # 0.1 m/s preserves normal braking/collision checks; no teleport or
        # surprise stopped vehicle is injected into a moving lane.
        ET.SubElement(sign, "step", time=str(observation_time + 300), speed="0.1")
        ET.SubElement(sign, "step", time=str(observation_time + 360), speed="5.5" if observation_time < 61200 else "4")
        sign[:] = sorted(sign, key=lambda step: float(step.get("time")))
    path = directory / "receiving-constraint.add.xml"
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    return path


def demand_surge(root, observation_time):
    """20% extra base traffic for one 10-minute burst; a labeled assumption."""
    vehicles = root.findall("vehicle")
    for index, vehicle in enumerate(vehicles):
        depart = float(vehicle.get("depart"))
        if observation_time + 120 <= depart < observation_time + 720 and index % 5 == 0:
            attrs = dict(vehicle.attrib)
            attrs.update(id="surge_" + vehicle.get("id"), depart=f"{depart + .1:.2f}")
            ET.SubElement(root, "vehicle", **attrs)
