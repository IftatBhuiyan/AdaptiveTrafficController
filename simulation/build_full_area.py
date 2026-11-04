#!/usr/bin/env python3
"""Build boundary-to-boundary traffic using the saved map and four count profiles.

The four stations were counted on different days. Their time profiles are a
composite scenario, not a reconstruction of one observed day. Uncounted traffic
and OD choices are explicit modelling assumptions. Count constraints apply to
requested trips; congestion may prevent all of them from crossing on time.
"""
from collections import Counter, defaultdict
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import random
import sys
import xml.etree.ElementTree as ET
from sumo_runtime import configure

HERE = Path(__file__).resolve().parent
DATA = HERE / "astoria-osm-import"
SUMO_HOME, SUMO_BINARY = configure()
import sumolib
import traci

STATIONS = [
    ("054474", "2025-05-29", {"1459805124"}),
    ("053167", "2023-03-29", {"959678463"}),
    ("053166", "2026-06-17", {"25345887"}),
    ("053169", "2026-06-17", {"375324638", "375324639"}),
]
BEGIN, END = 56700, 64800  # 15-minute warm-up before the 4 pm observation


def main():
    net = sumolib.net.readNet(str(DATA / "astoria-corrected.net.xml"))
    bounds = ET.parse(DATA / "map.osm").getroot().find("bounds").attrib
    traci.start([str(SUMO_BINARY), "-n", str(DATA / "astoria-corrected.net.xml"), "--no-step-log", "true"])
    try:
        def outside(point):
            lon, lat = traci.simulation.convertGeo(*point)
            return not (float(bounds["minlon"]) < lon < float(bounds["maxlon"])
                        and float(bounds["minlat"]) < lat < float(bounds["maxlat"]))
        starts = [e for e in net.getEdges() if e.allows("passenger") and e.is_fringe(e.getIncoming())
                  and outside(e.getFromNode().getCoord()) and e.getType() != "highway.service"]
        ends = [e for e in net.getEdges() if e.allows("passenger") and e.is_fringe(e.getOutgoing())
                and outside(e.getToNode().getCoord()) and e.getType() != "highway.service"]
        core_bounds = [*traci.simulation.convertGeo(float(bounds["minlon"]), float(bounds["minlat"]), fromGeo=True),
                       *traci.simulation.convertGeo(float(bounds["maxlon"]), float(bounds["maxlat"]), fromGeo=True)]
    finally:
        traci.close()

    candidates = {}
    def add_route(path):
        if not path:
            return
        ids = tuple(e.getID() for e in path)
        if len(ids) != len(set(ids)) or sum(e.getLength() for e in path) < 400:
            return
        for a, b in zip(path, path[1:]):
            if not any(c.getDirection() not in ("t", "T") for c in a.getConnections(b)):
                return
        candidates[ids] = path

    markers = [net.getEdge(e) for _, _, edges in STATIONS for e in sorted(edges)]
    for start in starts:
        for end in ends:
            direct, _ = net.getShortestPath(start, end, vClass="passenger")
            add_route(direct)
            for marker in markers:
                first, _ = net.getShortestPath(start, marker, vClass="passenger")
                last, _ = net.getShortestPath(marker, end, vClass="passenger")
                if first and last:
                    add_route(first + last[1:])

    # Shortest paths alone bypass some perfectly valid streets (including the
    # straight movement across 33rd). Include legal through-routes for every
    # reachable road so the full-area scenario can exercise those movements.
    covered = {edge for ids in candidates for edge in ids}
    for via in net.getEdges():
        if via.getID() in covered or not via.allows("passenger"):
            continue
        incoming = [net.getShortestPath(start, via, vClass="passenger") for start in starts]
        outgoing = [net.getShortestPath(via, end, vClass="passenger") for end in ends]
        possible = sorted(((a[1] + b[1], a[0] + b[0][1:])
                           for a in incoming for b in outgoing if a[0] and b[0]), key=lambda item: item[0])
        before = len(candidates)
        for _, path in possible:
            add_route(path)
            if len(candidates) >= before + 3:
                break
        covered = {edge for ids in candidates for edge in ids}

    groups = defaultdict(list)
    for ids, path in candidates.items():
        signature = tuple(i for i, (_, _, markers) in enumerate(STATIONS) if markers.intersection(ids))
        groups[signature].append((ids, path))
    print("Boundary origins / exits / routes:", len(starts), len(ends), len(candidates))
    print("Counter combinations:", {str(k): len(v) for k, v in groups.items()})
    for i in range(len(STATIONS)):
        if (i,) not in groups:
            raise RuntimeError(f"No independent boundary route for station {STATIONS[i][0]}")

    with (DATA / "traffic-profiles.csv").open() as f:
        profiles = list(csv.DictReader(f))
    rng = random.Random(42)
    root = ET.Element("routes")
    ET.SubElement(root, "vType", id="car", vClass="passenger", color="0.12,0.52,0.73")
    ET.SubElement(root, "vType", id="truck", vClass="truck", color="0.30,0.38,0.43")
    route_ids = {}
    for index, ids in enumerate(candidates):
        route_ids[ids] = "area_" + hashlib.sha256(" ".join(ids).encode()).hexdigest()[:12]
        ET.SubElement(root, "route", id=route_ids[ids], edges=" ".join(ids))
    vehicles, plan, source_counts = [], [], Counter()
    measured_total = background_total = 0
    by_signature = defaultdict(int)

    def schedule(entries, count, begin, end, kind):
        weights = [1 / max(1, sum(e.getLength() for e in path) / 1000) ** 1.5 for _, path in entries]
        for index in range(count):
            ids, path = rng.choices(entries, weights=weights, k=1)[0]
            depart = begin + (index + rng.random()) * (end - begin) / count
            is_truck = rng.random() < 0.04 and all(e.allows("truck") for e in path)
            vehicles.append((depart, kind, ids, "truck" if is_truck else "car"))
            source_counts[ids[0]] += 1

    for bin_index in range(BEGIN // 900, END // 900):
        targets = [int(profiles[bin_index][f"station_{station}_{date}"]) for station, date, _ in STATIONS]
        # Fit OD-combination flows to all counters jointly. Shared trips cross
        # multiple counters once, rather than becoming duplicate injections.
        fit = {key: float(max(1, len(entries))) for key, entries in groups.items() if key}
        for _ in range(1500):
            for i, target in enumerate(targets):
                total = sum(value for key, value in fit.items() if i in key)
                ratio = target / total if total else 0
                for key in fit:
                    if i in key:
                        fit[key] *= ratio
            if max(abs(sum(v for k, v in fit.items() if i in k) - target)
                   for i, target in enumerate(targets)) < 1e-6:
                break
        allocation = {key: math.floor(value) for key, value in fit.items() if len(key) > 1}
        for i, target in enumerate(targets):
            allocation[(i,)] = target - sum(v for k, v in allocation.items() if i in k)
        if any(v < 0 for v in allocation.values()):
            raise RuntimeError("Count constraints could not be reconciled")
        reconstructed = [sum(v for k, v in allocation.items() if i in k) for i in range(len(STATIONS))]
        assert reconstructed == targets
        begin, end = bin_index * 900, (bin_index + 1) * 900
        for key, count in allocation.items():
            schedule(groups[key], count, begin, end, "count")
            measured_total += count
            by_signature[str(key)] += count
        plan.append({"begin": begin, "counts": dict(zip([s[0] for s in STATIONS], targets)),
                     "unique_count_driven_trips": sum(allocation.values())})

    # Additional boundary-to-boundary trips on uncounted movements, with no
    # crossing of the four constrained counters. Rates below are assumptions.
    background_by_start = defaultdict(list)
    for entry in groups[()]:
        background_by_start[entry[0][0]].append(entry)
    for origin, entries in background_by_start.items():
        edge = net.getEdge(origin)
        if edge.getType() == "highway.motorway":
            hourly = 300 * len(edge.getLanes())
        elif edge.getType() in {"highway.secondary", "highway.primary", "highway.trunk"}:
            hourly = 60 * len(edge.getLanes())
        else:
            hourly = 30
        count = int(hourly * (END - BEGIN) / 3600)
        schedule(entries, count, BEGIN, END, "context")
        background_total += count

    for index, (depart, kind, ids, vehicle_type) in enumerate(sorted(vehicles)):
        ET.SubElement(root, "vehicle", id=f"{kind}_{index}", type=vehicle_type,
                      route=route_ids[ids], depart=f"{depart:.2f}", departLane="best", departSpeed="desired")
    ET.indent(root)
    ET.ElementTree(root).write(DATA / "scenario-full-area.rou.xml", encoding="utf-8", xml_declaration=True)
    info = {"source_map": "map.osm", "seed": 42, "begin": BEGIN, "end": END,
            "core_bounds": core_bounds, "origins": [e.getID() for e in starts], "exits": [e.getID() for e in ends],
            "routes": len(candidates), "count_driven_vehicles": measured_total,
            "estimated_context_vehicles": background_total, "total_vehicles": len(vehicles),
            "station_edges": {station: sorted(edges) for station, _, edges in STATIONS},
            "station_dates": {station: date for station, date, _ in STATIONS},
            "count_plan": plan, "source_counts": dict(source_counts), "route_combinations": dict(by_signature),
            "assumptions": ["Profiles combine separate count days; this is not one observed day.",
                            "Boundary OD choices and uncounted connecting traffic are estimated.",
                            "4 percent truck share is applied uniformly as a scenario assumption.",
                            "Original generated static signal plans are retained.",
                            "Requested route counts match counters; achieved throughput can differ under congestion."],
            "geometry_sources": ["https://gis.dot.ny.gov/hostingny/rest/services/Roadways/RDM_Traffic_Stations/FeatureServer/1",
                                 "https://gis.dot.ny.gov/hostingny/rest/services/Roadways/Traffic_Monitoring/FeatureServer/0"]}
    (DATA / "full-area-demand.json").write_text(json.dumps(info, indent=2) + "\n")
    print(json.dumps({k: info[k] for k in ("routes", "count_driven_vehicles", "estimated_context_vehicles", "total_vehicles")}, indent=2))


if __name__ == "__main__":
    main()
