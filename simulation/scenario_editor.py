"""Persistent route and signal edits for the shared browser simulation."""
from collections import Counter
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

from sumo_runtime import configure
from traffic_behavior import apply_behavior, receiving_constraint, demand_surge

SUMO_DIR, SUMO = configure()
import sumolib

DATA = Path(__file__).resolve().parent / "astoria-osm-import"
EDIT_FILE = DATA / "scenario-edits.json"
BASE_NET = DATA / "astoria-corrected.net.xml"
USER_NET = DATA / "user-network.net.xml"
BASE_ROUTES = DATA / "scenario-full-area.rou.xml"
DEFAULT = {"version": 1, "routes": {}, "signals": {}, "markers": {}, "roads": {}, "surfaces": {}, "artifacts": {}, "timings": {}, "controls": {},
           "nodes": {}, "geometry": {}, "splits": {}, "removed": {}, "drawings": {}}
# Roads the demand plan, the ramp-merge model, and the controllers refer to by ID.
# Splitting or deleting them would rename or remove what those components measure.
STRUCTURAL = {"25345887", "964223914", "1134690192", "374433288", "122625709"}
DRAWING_KINDS = {"sidewalk": "#d9d6cc", "median": "#b9c3b3", "plaza": "#e2dccb", "crosswalk": "#f1efe6", "custom": "#cfd6cc"}
DEFAULT_PEDESTRIANS = 120  # people per hour using a drawn crosswalk
MAX_POINTS = 400


def polyline_length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def offset_line(points, distance):
    """Shift a polyline sideways; positive distance is to the right of travel."""
    result = []
    for i, p in enumerate(points):
        a, b = points[max(0, i - 1)], points[min(len(points) - 1, i + 1)]
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy) or 1
        result.append([p[0] + dy / length * distance, p[1] - dx / length * distance])
    return result


def inside(point, polygon):
    x, y, yes = point[0], point[1], False
    for (ax, ay), (bx, by) in zip(polygon, polygon[-1:] + polygon[:-1]):
        if (ay > y) != (by > y) and x < (bx - ax) * (y - ay) / (by - ay) + ax:
            yes = not yes
    return yes


def segments_cross(a, b, c, d):
    def side(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    return side(a, b, c) * side(a, b, d) < 0 and side(c, d, a) * side(c, d, b) < 0


def line_touches_polygon(line, polygon):
    if any(inside(p, polygon) for p in line):
        return True
    ring = list(zip(polygon, polygon[1:] + polygon[:1]))
    return any(segments_cross(a, b, c, d) for a, b in zip(line, line[1:]) for c, d in ring)


def samples(line, count=9):
    """Evenly spaced points with their right-hand unit normals along a polyline."""
    total, result = polyline_length(line), []
    for k in range(count):
        target, run = total * (k + .5) / count, 0
        for a, b in zip(line, line[1:]):
            length = math.dist(a, b)
            if length and run + length >= target:
                t = (target - run) / length
                result.append(([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])], [(b[1] - a[1]) / length, -(b[0] - a[0]) / length]))
                break
            run += length
    return result


class ScenarioEditor:
    netedit_process = None
    def __init__(self):
        self.base_net = USER_NET if USER_NET.exists() else BASE_NET
        self.net = sumolib.net.readNet(str(self.base_net))
        self.plan = json.loads((DATA / "full-area-demand.json").read_text())
        self.routes = ET.parse(BASE_ROUTES).getroot()
        self.paths = {r.get("id"): r.get("edges").split() for r in self.routes.findall("route")}
        self.counts = Counter(v.get("route") for v in self.routes.findall("vehicle"))
        self.truck_routes = {v.get("route") for v in self.routes.findall("vehicle") if v.get("type") == "truck"}
        self.nodes = {j.getID(): j for j in self.net.getNodes()}
        self.edges = {e.getID(): e for e in self.net.getEdges()}
        self.artifacts = {p.get("id") for p in ET.parse(DATA / "astoria.poly.xml").getroot().findall("poly")}
        self.names = {}
        for way in ET.parse(DATA / "map.osm").getroot().findall("way"):
            tags = {t.get("k"): t.get("v") for t in way.findall("tag")}
            self.names[way.get("id")] = tags.get("name", "")
        self.root = ET.parse(self.base_net).getroot()
        self.bounds = [float(v) for v in self.root.find("location").get("convBoundary").split(",")]
        self.protected = set(self.plan["origins"]) | set(self.plan["exits"]) | STRUCTURAL
        self.protected.update(edge for edges in self.plan["station_edges"].values() for edge in edges)

    def node_position(self, jid, edits=None):
        moved = (edits or {}).get("nodes", {}).get(jid)
        return [float(moved[0]), float(moved[1])] if moved else list(self.nodes[jid].getCoord())

    def base_shape(self, eid):
        """SUMO edge shape: the left border for spreadType right, the centre otherwise."""
        element = self.root.find(f"edge[@id='{eid}']")
        if element is not None and element.get("shape"):
            return [[float(n) for n in point.split(",")] for point in element.get("shape").split()]
        edge = self.edges[eid]
        return [list(edge.getFromNode().getCoord()), list(edge.getToNode().getCoord())]

    def label(self, edge):
        name = self.names.get(edge.lstrip("-").split("#")[0], "")
        return (name + " · " if name else "Road · ") + edge

    def read(self):
        return self.validate(json.loads(EDIT_FILE.read_text()) if EDIT_FILE.exists() else DEFAULT)

    def catalog(self):
        root = self.root
        roads = {}
        for edge in root.findall("edge"):
            if edge.get("function"):
                continue
            lanes = edge.findall("lane")
            roads[edge.get("id")] = {
                "laneCount": len(lanes), "speed": float(lanes[0].get("speed")),
                "closed": lanes[0].get("disallow") == "all", "spreadType": edge.get("spreadType", "right"),
                "from": edge.get("from"), "to": edge.get("to"), "type": edge.get("type", ""),
                "name": self.names.get(edge.get("id").lstrip("-").split("#")[0], ""),
                "length": float(lanes[0].get("length")), "shape": self.base_shape(edge.get("id")),
                "protected": edge.get("id") in self.protected,
                "lanes": [{"index": int(lane.get("index")), "width": float(lane.get("width", "3.2")),
                           "shape": [[float(n) for n in point.split(",")] for point in lane.get("shape").split()]}
                          for lane in lanes]}
        junctions = {}
        for node in root.findall("junction"):
            jid = node.get("id")
            if jid.startswith(":") or jid not in self.nodes:
                continue
            junctions[jid] = {"shape": [[float(n) for n in p.split(",")] for p in node.get("shape").split()] if node.get("shape") else [],
                              "x": float(node.get("x")), "y": float(node.get("y")),
                              "controlled": "traffic_light" in node.get("type", ""), "controlType": node.get("type"),
                              "incoming": [e.getID() for e in self.nodes[jid].getIncoming()],
                              "outgoing": [e.getID() for e in self.nodes[jid].getOutgoing()]}
        artifacts = {p.get("id"): {"shape": [[float(n) for n in v.split(",")] for v in p.get("shape").split()],
                                  "hidden": False, "color": None}
                     for p in ET.parse(DATA / "astoria.poly.xml").getroot().findall("poly")}
        return {"edits": self.read(), "base": {"roads": roads, "junctions": junctions, "artifacts": artifacts,
                                              "bounds": self.bounds, "drawingKinds": DRAWING_KINDS},
                "routes": [{"id": rid, "from": edges[0], "to": edges[-1], "edges": edges,
                            "count": self.counts[rid], "label": f"{rid} · {self.label(edges[0])} → {self.label(edges[-1])}",
                            "search": " ".join([rid, *edges, *(self.label(e) for e in edges)])}
                           for rid, edges in sorted(self.paths.items(), key=lambda r: -self.counts[r[0]])
                           if self.counts[rid]],
                "origins": [{"id": e, "label": self.label(e)} for e in self.plan["origins"]],
                "exits": [{"id": e, "label": self.label(e)} for e in self.plan["exits"]]}

    def preview(self, spec, route_id="", network=None):
        network = network or self.net
        if not isinstance(spec, dict):
            raise ValueError("Choose a route first.")
        start, end = spec.get("from"), spec.get("to")
        via = spec.get("via", [])
        if start not in self.plan["origins"] or end not in self.plan["exits"]:
            raise ValueError("Choose an outer-map entrance and exit, so cars continue through the study area.")
        if not isinstance(via, list) or len(via) > 12 or not all(isinstance(v, str) for v in via):
            raise ValueError("A route can have up to 12 waypoints.")
        vclass = "truck" if route_id in self.truck_routes else "passenger"
        route = []
        try:
            for a, b in zip([start] + via, via + [end]):
                path, _ = network.getShortestPath(network.getEdge(a), network.getEdge(b), vClass=vclass)
                if not path:
                    raise ValueError("Those roads cannot be connected in that direction. Choose another waypoint or exit.")
                ids = [e.getID() for e in path]
                route.extend(ids if not route else ids[1:])
        except KeyError:
            raise ValueError("One of the selected roads is not in this map.") from None
        if len(route) != len(set(route)):
            raise ValueError("This route loops back onto the same road. Remove a waypoint or choose another exit.")
        return {"edges": route, "length": round(sum(network.getEdge(e).getLength() for e in route))}

    def points(self, value, minimum, what):
        if not isinstance(value, list) or not minimum <= len(value) <= MAX_POINTS:
            raise ValueError(f"{what} needs between {minimum} and {MAX_POINTS} points.")
        if not all(isinstance(p, list) and len(p) == 2 and all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in p) for p in value):
            raise ValueError(f"Invalid {what.lower()} coordinates.")
        margin = 300
        if not all(self.bounds[0] - margin <= p[0] <= self.bounds[2] + margin and self.bounds[1] - margin <= p[1] <= self.bounds[3] + margin for p in value):
            raise ValueError(f"{what} points must stay within 300 m of the map.")
        cleaned = [[round(float(p[0]), 3), round(float(p[1]), 3)] for p in value]
        return [p for i, p in enumerate(cleaned) if i == 0 or math.dist(p, cleaned[i - 1]) > .001]

    def shape(self, value):
        value = self.points(value, 3, "A shape")
        if value[0] == value[-1]:
            value = value[:-1]
        if len(value) < 3:
            raise ValueError("A shape needs at least three distinct corners.")
        def side(a, b, c):
            return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        edges = list(zip(value, value[1:] + value[:1]))
        for i, (a, b) in enumerate(edges):
            for j, (c, d) in enumerate(edges[i + 1:], start=i + 1):
                if j == i + 1 or (i == 0 and j == len(edges) - 1):
                    continue
                if side(a, b, c) * side(a, b, d) < 0 and side(c, d, a) * side(c, d, b) < 0:
                    raise ValueError("Shape edges cannot cross. Move the corner back outside the opposite edge.")
        area = abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(value, value[1:] + value[:1]))) / 2
        if area < 1:
            raise ValueError("The shape is too small or crosses over itself.")
        return value

    def validate(self, edits):
        if not isinstance(edits, dict) or edits.get("version", 1) != 1:
            raise ValueError("Unsupported edit file.")
        clean = {"version": 1, **{key: {} for key in DEFAULT if key != "version"}}
        for key in clean.keys() - {"version"}:
            if not isinstance(edits.get(key, {}), dict) or len(edits.get(key, {})) > 1000:
                raise ValueError("Invalid edit list.")
        for rid, spec in edits.get("routes", {}).items():
            if rid not in self.paths and not (rid.startswith("custom_") and rid.replace("_", "").isalnum()):
                raise ValueError("Unknown route.")
            self.preview(spec, rid)
            cleaned = {"from": spec["from"], "to": spec["to"], "via": list(spec.get("via", []))}
            if rid.startswith("custom_"):
                rate = spec.get("rate", 60)
                if isinstance(rate, bool) or not isinstance(rate, (int, float)) or not math.isfinite(rate) or not 1 <= rate <= 3000:
                    raise ValueError("Custom traffic must be between 1 and 3,000 vehicles per hour.")
                cleaned["rate"] = rate
            clean["routes"][rid] = cleaned
        for jid, enabled in edits.get("signals", {}).items():
            if jid not in self.nodes or type(enabled) is not bool:
                raise ValueError("Choose a junction on the map for the signal.")
            node = self.nodes[jid]
            if enabled and (not node.getIncoming() or not node.getOutgoing()):
                raise ValueError("An outer entrance or exit cannot be a signalized intersection.")
            clean["signals"][jid] = enabled
        for jid, kind in edits.get("controls", {}).items():
            if jid not in self.nodes or kind not in {"traffic_light", "priority", "priority_stop", "allway_stop"}:
                raise ValueError("Choose a valid junction control.")
            clean["controls"][jid] = kind
        for jid, point in edits.get("markers", {}).items():
            if jid not in self.nodes or not isinstance(point, list) or len(point) != 2:
                raise ValueError("Invalid signal marker.")
            if not all(isinstance(v, (int, float)) and math.isfinite(v) and abs(v) <= 100 for v in point):
                raise ValueError("Keep a marker within 100 metres of its junction. Use Move control to change the actual junction.")
            clean["markers"][jid] = point
        for eid, spec in edits.get("roads", {}).items():
            if eid not in self.edges or not isinstance(spec, dict):
                raise ValueError("Choose a road on this map.")
            lanes, speed, closed = spec.get("lanes"), spec.get("speed"), spec.get("closed", False)
            if type(lanes) is not int or not 1 <= lanes <= 8:
                raise ValueError("Choose between 1 and 8 lanes.")
            if not isinstance(speed, (int, float)) or not math.isfinite(speed) or not 1 <= speed <= 40:
                raise ValueError("Choose a speed between 3 and 89 mph.")
            if type(closed) is not bool:
                raise ValueError("Invalid road closure.")
            clean["roads"][eid] = {"lanes": lanes, "speed": speed, "closed": closed}
        for jid, shape in edits.get("surfaces", {}).items():
            if jid not in self.nodes:
                raise ValueError("Choose a junction on this map.")
            clean["surfaces"][jid] = self.shape(shape)
        for pid, spec in edits.get("artifacts", {}).items():
            if pid not in self.artifacts or not isinstance(spec, dict):
                raise ValueError("Choose a building or map polygon.")
            item = {}
            if "hidden" in spec:
                if type(spec["hidden"]) is not bool:
                    raise ValueError("Invalid visibility setting.")
                item["hidden"] = spec["hidden"]
            if "shape" in spec:
                item["shape"] = self.shape(spec["shape"])
            if "color" in spec:
                if not isinstance(spec["color"], str) or not re.fullmatch(r"#[0-9a-fA-F]{6}", spec["color"]):
                    raise ValueError("Choose a valid polygon colour.")
                item["color"] = spec["color"]
            clean["artifacts"][pid] = item
        for jid, spec in edits.get("timings", {}).items():
            if jid not in self.nodes or not isinstance(spec, dict):
                raise ValueError("Choose a signalized junction.")
            cycle, offset = spec.get("cycle"), spec.get("offset", 0)
            if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (cycle, offset)) or not 30 <= cycle <= 240:
                raise ValueError("Signal cycles must be between 30 and 240 seconds.")
            clean["timings"][jid] = {"cycle": cycle, "offset": offset % cycle}
        for jid, point in edits.get("nodes", {}).items():
            if jid not in self.nodes or not isinstance(point, list) or len(point) != 2:
                raise ValueError("Choose a junction on this map to move.")
            [point] = self.points([point], 1, "A junction")
            if math.dist(point, self.nodes[jid].getCoord()) > 150:
                raise ValueError("Keep a junction within 150 m of its original position. Larger moves change which streets connect.")
            if math.dist(point, self.nodes[jid].getCoord()) > .005:
                clean["nodes"][jid] = point
        for eid, flag in edits.get("removed", {}).items():
            if eid not in self.edges or type(flag) is not bool:
                raise ValueError("Choose a road on this map to remove.")
            if eid in self.protected:
                raise ValueError(f"{self.label(eid)} carries counted or ramp traffic that the simulation measures. It cannot be removed.")
            if flag:
                clean["removed"][eid] = True
        for eid, spec in edits.get("geometry", {}).items():
            if eid not in self.edges or not isinstance(spec, dict):
                raise ValueError("Choose a road on this map to reshape.")
            if eid in clean["removed"]:
                continue
            item = {}
            if "shape" in spec:
                shape = self.points(spec["shape"], 2, "A road centreline")
                base = self.base_shape(eid)
                edge = self.edges[eid]
                for end, jid in ((0, edge.getFromNode().getID()), (-1, edge.getToNode().getID())):
                    # Road ends belong to their junctions: keep them where the junction (after any move) puts them.
                    delta = [a - b for a, b in zip(self.node_position(jid, clean), self.nodes[jid].getCoord())]
                    shape[end] = [round(base[end][0] + delta[0], 3), round(base[end][1] + delta[1], 3)]
                if polyline_length(shape) < 1:
                    raise ValueError("A road needs at least one metre of length.")
                if shape != base:
                    item["shape"] = shape
            if "widths" in spec:
                widths = spec["widths"]
                count = clean["roads"].get(eid, {}).get("lanes", len(self.edges[eid].getLanes()))
                if not isinstance(widths, list) or len(widths) != count:
                    raise ValueError(f"Give one width for each of the {count} lanes.")
                if not all(isinstance(w, (int, float)) and not isinstance(w, bool) and math.isfinite(w) and 1.5 <= w <= 12 for w in widths):
                    raise ValueError("Lane widths must be between 1.5 and 12 metres.")
                item["widths"] = [round(float(w), 2) for w in widths]
            if item:
                clean["geometry"][eid] = item
        for eid, positions in edits.get("splits", {}).items():
            if eid not in self.edges or not isinstance(positions, list) or len(positions) > 8:
                raise ValueError("Choose a road on this map to split, with at most 8 cuts.")
            if eid in clean["removed"] or not positions:
                continue
            if eid in self.protected:
                raise ValueError(f"{self.label(eid)} is measured by the demand plan or the ramp model. Splitting would rename it; split a neighbouring road instead.")
            # netconvert measures split positions along the edge geometry, not the (cut-back) lane length.
            shape = clean["geometry"].get(eid, {}).get("shape") or self.base_shape(eid)
            length = polyline_length(shape)
            if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in positions):
                raise ValueError("Split positions must be distances in metres.")
            positions = sorted(round(float(v), 2) for v in positions)
            if positions[0] < 2 or positions[-1] > length - 2 or any(b - a < 2 for a, b in zip(positions, positions[1:])):
                raise ValueError(f"Keep each cut at least 2 m from the ends of this {length:.0f} m road and from other cuts.")
            clean["splits"][eid] = positions
        for did, spec in edits.get("drawings", {}).items():
            if not (isinstance(did, str) and did.startswith("custom_") and did.replace("_", "").isalnum()) or not isinstance(spec, dict):
                raise ValueError("Invalid drawn surface.")
            kind = spec.get("kind", "custom")
            if kind not in DRAWING_KINDS:
                raise ValueError("Choose a surface type: sidewalk, median, plaza, crosswalk, or custom.")
            item = {"kind": kind, "shape": self.shape(spec.get("shape"))}
            if spec.get("color"):
                if not isinstance(spec["color"], str) or not re.fullmatch(r"#[0-9a-fA-F]{6}", spec["color"]):
                    raise ValueError("Choose a valid surface colour.")
                item["color"] = spec["color"]
            if spec.get("label"):
                if not isinstance(spec["label"], str) or len(spec["label"]) > 60:
                    raise ValueError("Keep surface labels under 60 characters.")
                item["label"] = spec["label"]
            if "pedestrians" in spec and kind == "crosswalk":
                rate = spec["pedestrians"]
                if isinstance(rate, bool) or not isinstance(rate, (int, float)) or not math.isfinite(rate) or not 0 <= rate <= 2000:
                    raise ValueError("Pedestrians must be between 0 and 2,000 per hour.")
                item["pedestrians"] = round(float(rate))
            clean["drawings"][did] = item
        clean["effects"] = self.surface_effects(clean)
        return clean

    def lane_shapes(self, eid, edits):
        """Vehicle lane centre lines (index 0 rightmost) after any staged reshaping, from the base network."""
        edge = self.edges[eid]
        shape = edits.get("geometry", {}).get(eid, {}).get("shape")
        lanes = [[list(p) for p in lane.getShape()] for lane in edge.getLanes()]
        if not shape:
            return lanes, [lane.getWidth() for lane in edge.getLanes()]
        # A reshaped road: rebuild lane lines from the new centre/border line, as netconvert will.
        widths = edits["geometry"][eid].get("widths") or [lane.getWidth() for lane in edge.getLanes()]
        total = sum(widths)
        spread = self.root.find(f"edge[@id='{eid}']").get("spreadType", "right")
        result, from_left = [], 0
        for index in range(len(widths) - 1, -1, -1):
            centre = from_left + widths[index] / 2 - (0 if spread == "right" else total / 2)
            result.insert(0, offset_line(shape, centre))
            from_left += widths[index]
        return result, widths

    def surface_effects(self, edits):
        """What each drawn surface does in SUMO.

        sidewalk  -> a pedestrian lane on every road whose right edge it covers
        crosswalk -> a signal/priority crossing at the nearest junction, sidewalks at that junction, and pedestrians
        median    -> closes vehicle lanes it covers and removes turns that cut through it
        plaza     -> makes roads it covers pedestrian-only
        custom    -> display only
        """
        effects = {"sidewalks": {}, "crossings": [], "medians": [], "lane_closures": {}, "pedestrian_only": [], "notes": {}}
        for did, drawing in sorted(edits.get("drawings", {}).items()):
            polygon, kind = drawing["shape"], drawing["kind"]
            xs, ys = [p[0] for p in polygon], [p[1] for p in polygon]
            box = (min(xs) - 15, min(ys) - 15, max(xs) + 15, max(ys) + 15)
            near = [eid for eid, edge in self.edges.items() if eid not in edits.get("removed", {})
                    and any(box[0] <= x <= box[2] and box[1] <= y <= box[3] for x, y in self.lane_shapes(eid, edits)[0][0])]
            if kind == "sidewalk":
                matched, refused = [], []
                for eid in near:
                    lanes, widths = self.lane_shapes(eid, edits)
                    points = samples(lanes[0])
                    covered = [p for p, n in points if inside([p[0] + n[0] * (widths[0] / 2 + .9), p[1] + n[1] * (widths[0] / 2 + .9)], polygon)
                               and not inside(p, polygon)]
                    if len(covered) < .6 * len(points):
                        continue
                    if eid in self.protected:
                        refused.append(eid)
                        continue
                    width = []
                    for p, n in points:
                        d = widths[0] / 2 + .25
                        while d < widths[0] / 2 + 10 and inside([p[0] + n[0] * d, p[1] + n[1] * d], polygon):
                            d += .25
                        width.append(d - widths[0] / 2)
                    width = sorted(width)[len(width) // 2]
                    effects["sidewalks"][eid] = round(min(6, max(1, max(width, effects["sidewalks"].get(eid, 0)))), 2)
                    matched.append(eid)
                # Measured: a sidewalk without a crossing leaves every signal plan unchanged.
                note = (f"Simulated: pedestrian lane on {', '.join(self.label(e) for e in matched)}." if matched else
                        "Display only: it does not run along the right-hand edge of a road. Use the Sidewalk-along-road tool, or drag its points onto a road edge.")
                if refused:
                    note += f" Not added to {', '.join(self.label(e) for e in refused)}: the controller measures that road's lanes by number."
                effects["notes"][did] = note
            elif kind == "crosswalk":
                centre = [sum(xs) / len(xs), sum(ys) / len(ys)]
                # The crosswalk belongs to the junction whose own approaches it crosses (ties: the closest).
                best = None
                for jid, node in self.nodes.items():
                    distance = math.dist(self.node_position(jid, edits), centre)
                    if distance > 60 or not node.getIncoming() or not node.getOutgoing():
                        continue
                    crossed = []
                    for edge in node.getIncoming() + node.getOutgoing():
                        if edge.getID() in edits.get("removed", {}):
                            continue
                        lanes, _ = self.lane_shapes(edge.getID(), edits)
                        # Only the part of the road within 30 m of this junction counts.
                        end = lanes[0][-1] if edge.getToNode() is node else lanes[0][0]
                        if any(line_touches_polygon(lane, polygon) for lane in lanes) and min(math.dist(p, end) for p in polygon) < 30:
                            crossed.append(edge.getID())
                    if crossed and (best is None or (len(crossed), -distance) > (len(best[1]), -best[2])):
                        best = (jid, crossed, distance)
                if best is None:
                    effects["notes"][did] = "Display only: draw the crosswalk across a road, within 30 m of a junction."
                    continue
                jid, crossed, _ = best
                node = self.nodes[jid]
                walkable = [e.getID() for e in node.getIncoming() + node.getOutgoing()
                            if e.getID() not in self.protected and e.getID() not in edits.get("removed", {})]
                for eid in walkable:
                    effects["sidewalks"].setdefault(eid, 2.0)
                width = max(2.0, min(8.0, (max(xs) - min(xs) + max(ys) - min(ys)) / 2 if len(polygon) < 4 else
                                     min(math.dist(a, b) for a, b in zip(polygon, polygon[1:] + polygon[:1]))))
                rate = drawing.get("pedestrians", DEFAULT_PEDESTRIANS)
                effects["crossings"].append({"drawing": did, "node": jid, "edges": crossed, "width": round(width, 2), "pedestrians": rate})
                signal = "traffic_light" in node.getType()
                effects["notes"][did] = (f"Simulated: crossing over {', '.join(self.label(e) for e in crossed)} at junction {jid}"
                                         f"{' (signalized: SUMO regenerates this one light’s plan with a pedestrian phase; other lights keep theirs)' if signal else ''}, "
                                         f"sidewalks on this junction’s ordinary roads, and {rate} pedestrians per hour.")
            elif kind == "median":
                closed = {}
                for eid in near:
                    if eid in self.protected:
                        continue
                    lanes, _ = self.lane_shapes(eid, edits)
                    for index, lane in enumerate(lanes):
                        points = samples(lane)
                        if sum(inside(p, polygon) for p, _ in points) >= .6 * len(points):
                            closed.setdefault(eid, set()).add(index)
                for eid, indices in closed.items():
                    if len(indices) == len(self.edges[eid].getLanes()):
                        indices.discard(0)  # a median never removes the last lane; use Remove road for that
                    if indices:
                        effects["lane_closures"].setdefault(eid, set()).update(indices)
                effects["medians"].append({"drawing": did, "shape": polygon})
                lanes_text = ", ".join(f"{self.label(e)} lane {', '.join(str(i + 1) for i in sorted(v))}" for e, v in closed.items() if v)
                effects["notes"][did] = ("Simulated: blocks turns whose path crosses it" + (f" and closes {lanes_text}." if lanes_text else "."))
            elif kind == "plaza":
                covered, refused = [], []
                for eid in near:
                    points = samples([list(p) for p in self.edges[eid].getShape()])
                    if sum(inside(p, polygon) for p, _ in points) < .6 * len(points):
                        continue
                    (refused if eid in self.protected else covered).append(eid)
                effects["pedestrian_only"].extend(e for e in covered if e not in effects["pedestrian_only"])
                note = (f"Simulated: {', '.join(self.label(e) for e in covered)} become pedestrian-only; car routes are re-planned around them."
                        if covered else "Display only: it does not cover most of any road.")
                if refused:
                    note += f" {', '.join(self.label(e) for e in refused)} stay open: the simulation measures traffic there."
                effects["notes"][did] = note
            else:
                effects["notes"][did] = "Display only: custom surfaces are drawn on the map but do not change the simulation."
        for eid in effects["pedestrian_only"]:
            effects["sidewalks"].pop(eid, None)
            effects["lane_closures"].pop(eid, None)
        effects["lane_closures"] = {e: sorted(v) for e, v in effects["lane_closures"].items()}
        return effects

    def median_deletions(self, network, medians):
        """Connections whose path through the junction cuts a median."""
        if not medians:
            return []
        built = sumolib.net.readNet(str(network), withInternal=True)
        deletions = []
        for edge in built.getEdges():
            for lane in edge.getLanes():
                for connection in lane.getOutgoing():
                    via = connection.getViaLaneID()
                    if not via:
                        continue
                    path = [list(p) for p in built.getLane(via).getShape()]
                    if any(line_touches_polygon(path, median["shape"]) for median in medians):
                        deletions.append((edge.getID(), connection.getTo().getID(), lane.getIndex(), connection.getToLane().getIndex()))
        return deletions

    def pedestrian_flows(self, network, crossings, start_time):
        """Short walks across each drawn crosswalk, kept on that junction's sidewalks.

        Road lanes here also admit pedestrians, so long walks would wander into traffic. Each walk
        goes from a sidewalk at one end of the crossing straight to a sidewalk at the other end.
        """
        routes = ET.Element("routes")
        if not crossings:
            return routes
        root = ET.parse(network).getroot()
        lengths = {e.get("id"): float(e.find("lane").get("length")) for e in root.findall("edge") if not e.get("function")}
        ends = {e.get("id"): (e.get("from"), e.get("to")) for e in root.findall("edge") if not e.get("function")}
        links = [(c.get("from"), c.get("to")) for c in root.findall("connection")]
        for crossing in crossings:
            if not crossing["pedestrians"]:
                continue
            ids = [e.get("id") for e in root.findall("edge") if e.get("function") == "crossing"
                   and set(e.get("crossingEdges", "").split()) == set(crossing["edges"])]
            if not ids:
                continue
            cid = ids[0]
            areas = {a for a, b in links if b == cid} | {b for a, b in links if a == cid}
            sides = []
            for area in sorted(areas):
                sides.append(sorted({b for a, b in links if a == area and b in lengths} | {a for a, b in links if b == area and a in lengths}))
            if len(sides) != 2 or not all(sides):
                continue
            pairs = [(a, b) for a in sides[0] for b in sides[1]] + [(b, a) for a in sides[0] for b in sides[1]]
            period = 3600 * len(pairs) / crossing["pedestrians"]
            jid = crossing["node"]
            for index, (a, b) in enumerate(pairs):
                depart = max(0, lengths[a] - 8) if ends[a][1] == jid else min(8, lengths[a])
                arrive = min(8, lengths[b]) if ends[b][0] == jid else max(0, lengths[b] - 8)
                flow = ET.SubElement(routes, "personFlow", id=f"walk_{crossing['drawing']}_{index}", begin=str(start_time), end="64800",
                                     period=f"{period:.1f}", departPos=f"{depart:.1f}")
                ET.SubElement(flow, "walk", edges=f"{a} {b}", arrivalPos=f"{arrive:.1f}")
        return routes

    def build(self, edits, directory, traffic_mode="baseline", start_time=56700, seed=42):
        edits = self.validate(edits)
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        network = self.base_net
        controls = {j: "traffic_light" if enabled else "priority" for j, enabled in edits["signals"].items()}
        controls.update(edits["controls"])
        effects = edits["effects"]
        surfaces = effects["sidewalks"] or effects["crossings"] or effects["lane_closures"] or effects["pedestrian_only"] or effects["medians"]
        structural = edits["roads"] or edits["geometry"] or edits["splits"] or edits["removed"] or edits["nodes"] or surfaces
        if controls or structural or edits["surfaces"]:
            nodes = ET.Element("nodes")
            for jid in controls.keys() | edits["surfaces"].keys() | edits["nodes"].keys():
                attrs = {"id": jid}
                if jid in controls:
                    attrs["type"] = controls[jid]
                if jid in edits["nodes"]:
                    attrs["x"], attrs["y"] = (f"{v:.3f}" for v in edits["nodes"][jid])
                if jid in edits["surfaces"]:
                    shape = edits["surfaces"][jid]
                    if jid in edits["nodes"]:
                        # A custom paved area follows its junction.
                        delta = [a - b for a, b in zip(edits["nodes"][jid], self.nodes[jid].getCoord())]
                        shape = [[x + delta[0], y + delta[1]] for x, y in shape]
                    attrs["shape"] = " ".join(f"{x:.3f},{y:.3f}" for x, y in shape)
                ET.SubElement(nodes, "node", **attrs)
            node_file = directory / "signals.nod.xml"
            ET.ElementTree(nodes).write(node_file, encoding="utf-8", xml_declaration=True)
            network = directory / "edited.net.xml"
            converter = SUMO.with_name("netconvert" + SUMO.suffix)
            command = [str(converter), "-s", str(self.base_net), "-n", str(node_file), "-o", str(network),
                       "--tls.default-type", "static", "--tls.cycle.time", "90"]
            if structural:
                roads = ET.Element("edges")
                resets = set()
                elements = {}
                for eid, spec in edits["roads"].items():
                    if eid in edits["removed"]:
                        continue
                    attrs = {"id": eid, "numLanes": str(spec["lanes"]), "speed": str(spec["speed"])}
                    if spec["closed"]:
                        attrs["disallow"] = "all"
                    elements[eid] = ET.SubElement(roads, "edge", **attrs)
                    if spec["lanes"] != len(self.edges[eid].getLanes()) or spec["closed"]:
                        resets.add(eid)
                        resets.update(e.getID() for e in self.edges[eid].getFromNode().getIncoming())
                for eid, spec in edits["geometry"].items():
                    element = (elements[eid] if eid in elements else elements.setdefault(eid, ET.SubElement(roads, "edge", id=eid)))
                    if "shape" in spec:
                        element.set("shape", " ".join(f"{x:.3f},{y:.3f}" for x, y in spec["shape"]))
                    for index, width in enumerate(spec.get("widths", [])):
                        ET.SubElement(element, "lane", index=str(index), width=f"{width:.2f}")
                for eid, positions in edits["splits"].items():
                    element = (elements[eid] if eid in elements else elements.setdefault(eid, ET.SubElement(roads, "edge", id=eid)))
                    for pos in positions:
                        ET.SubElement(element, "split", pos=f"{pos:.2f}")
                def element(eid):
                    return (elements[eid] if eid in elements else elements.setdefault(eid, ET.SubElement(roads, "edge", id=eid)))
                for eid, width in effects["sidewalks"].items():
                    element(eid).set("sidewalkWidth", f"{width:.2f}")
                for eid in effects["pedestrian_only"]:
                    element(eid).set("allow", "pedestrian")
                    element(eid).attrib.pop("disallow", None)
                for eid, indices in effects["lane_closures"].items():
                    lanes = {int(l.get("index")): l for l in element(eid).findall("lane")}
                    for index in indices:
                        lane = lanes[index] if index in lanes else ET.SubElement(element(eid), "lane", index=str(index))
                        lane.set("disallow", "all")
                for eid in edits["removed"]:
                    ET.SubElement(roads, "delete", id=eid)
                road_file = directory / "roads.edg.xml"
                ET.ElementTree(roads).write(road_file, encoding="utf-8", xml_declaration=True)
                command += ["-e", str(road_file)]
                if resets or effects["crossings"]:
                    connections = ET.Element("connections")
                    for eid in sorted(resets):
                        ET.SubElement(connections, "connection", **{"from": eid, "reset": "true"})
                    for crossing in effects["crossings"]:
                        signal = "traffic_light" in controls.get(crossing["node"], self.nodes[crossing["node"]].getType())
                        ET.SubElement(connections, "crossing", node=crossing["node"], edges=" ".join(crossing["edges"]),
                                      width=f"{crossing['width']:.2f}", priority="false" if signal else "true")
                    con_file = directory / "roads.con.xml"
                    ET.ElementTree(connections).write(con_file, encoding="utf-8", xml_declaration=True)
                    command += ["-x", str(con_file)]
            disabled = [jid for jid, kind in controls.items() if kind != "traffic_light" and "traffic_light" in self.nodes[jid].getType()]
            if disabled:
                command += ["--tls.unset", ",".join(disabled)]
            result = subprocess.run(command, capture_output=True, text=True, timeout=45)
            if result.returncode:
                raise ValueError("SUMO could not build those signal changes: " + result.stderr[-1200:])
            if effects["crossings"]:
                built_root = ET.parse(network).getroot()
                made = [set(e.get("crossingEdges", "").split()) for e in built_root.findall("edge") if e.get("function") == "crossing"]
                for crossing in effects["crossings"]:
                    if set(crossing["edges"]) not in made:
                        raise ValueError(f"SUMO could not place the crosswalk at junction {crossing['node']}: it needs a sidewalk on both ends. "
                                         "Draw it across one street, between two corners that have ordinary (not ramp or measured) roads.")
            deletions = self.median_deletions(network, effects["medians"])
            if deletions:
                deletes = ET.Element("connections")
                for a, b, i, j in deletions:
                    ET.SubElement(deletes, "delete", **{"from": a, "to": b, "fromLane": str(i), "toLane": str(j)})
                delete_file = directory / "medians.con.xml"
                ET.ElementTree(deletes).write(delete_file, encoding="utf-8", xml_declaration=True)
                trimmed = directory / "median.net.xml"
                result = subprocess.run([str(converter), "-s", str(network), "-x", str(delete_file), "-o", str(trimmed),
                                         "--tls.default-type", "static", "--tls.cycle.time", "90"], capture_output=True, text=True, timeout=45)
                if result.returncode:
                    raise ValueError("SUMO could not apply the median: " + result.stderr[-1200:])
                network = trimmed
        if edits["timings"]:
            tree = ET.parse(network)
            edge_nodes = {e.get("id"): e.get("to") for e in tree.getroot().findall("edge")}
            node_tls = {edge_nodes.get(c.get("from")): c.get("tl") for c in tree.getroot().findall("connection") if c.get("tl")}
            programs = {p.get("id"): p for p in tree.getroot().findall("tlLogic")}
            for jid, spec in edits["timings"].items():
                program = programs.get(node_tls.get(jid))
                if program is None:
                    if controls.get(jid, "traffic_light") != "traffic_light":
                        continue
                    raise ValueError("This junction has no traffic light. Enable it before editing timing.")
                phases = program.findall("phase")
                green = [p for p in phases if "y" not in p.get("state").lower() and "g" in p.get("state").lower()]
                fixed = sum(float(p.get("duration")) for p in phases if p not in green)
                available = spec["cycle"] - fixed
                total = sum(float(p.get("duration")) for p in green)
                if not total or available < 3 * len(green):
                    raise ValueError("The cycle is too short to preserve the yellow and clearance phases.")
                for phase in green:
                    phase.set("duration", str(float(phase.get("duration")) * available / total))
                program.set("offset", str(spec["offset"]))
            network = directory / "timed.net.xml"
            tree.write(network, encoding="utf-8", xml_declaration=True)
        edited_net = sumolib.net.readNet(str(network))
        root = ET.parse(BASE_ROUTES).getroot()
        lookup = {r.get("id"): r for r in root.findall("route")}
        if structural or self.base_net == USER_NET:
            for rid, route in lookup.items():
                edges = route.get("edges").split()
                vclass = "truck" if rid in self.truck_routes else "passenger"
                valid = all(edited_net.hasEdge(e) and edited_net.getEdge(e).allows(vclass) for e in edges)
                if valid:
                    valid = all(edited_net.getEdge(b) in edited_net.getEdge(a).getAllowedOutgoing(vclass) for a, b in zip(edges, edges[1:]))
                if not valid:
                    spec = {"from": edges[0], "to": edges[-1], "via": []}
                    try:
                        route.set("edges", " ".join(self.preview(spec, rid, edited_net)["edges"]))
                    except ValueError as error:
                        raise ValueError(f"Road change disconnects {rid}: {error}") from error
        for rid, spec in edits["routes"].items():
            edges = self.preview(spec, rid, edited_net)["edges"]
            if rid in lookup:
                lookup[rid].set("edges", " ".join(edges))
            else:
                ET.SubElement(root, "route", id=rid, edges=" ".join(edges))
                ET.SubElement(root, "flow", id=rid + "_flow", type="car", route=rid,
                              begin="57600", end="64800", number=str(round(spec["rate"] * 2)),
                              departLane="best", departSpeed="desired")
        apply_behavior(root, traffic_mode, seed=seed)
        if traffic_mode == "surge":
            demand_surge(root, start_time + 900)
        definitions = [e for e in root if e.tag in ("vType", "route")]
        traffic = [e for e in root if e.tag == "vehicle" and float(e.get("depart")) >= start_time]
        for flow in root.findall("flow"):
            start, end = float(flow.get("begin")), float(flow.get("end"))
            if end <= start_time:
                continue
            if start < start_time:
                flow.set("number", str(round(int(flow.get("number")) * (end - start_time) / (end - start))))
                flow.set("begin", str(start_time))
            traffic.append(flow)
        planned = sum(1 if e.tag == "vehicle" else int(e.get("number")) for e in traffic)
        traffic = sorted(traffic,
                         key=lambda e: float(e.get("depart", e.get("begin", "0"))))
        root[:] = definitions + traffic
        routes = directory / "edited.rou.xml"
        ET.ElementTree(root).write(routes, encoding="utf-8", xml_declaration=True)
        route_files = str(routes)
        walkers = self.pedestrian_flows(network, effects["crossings"], start_time)
        if len(walkers):
            people = directory / "pedestrians.rou.xml"
            ET.ElementTree(walkers).write(people, encoding="utf-8", xml_declaration=True)
            route_files += "," + str(people)
        args = ["-c", str(DATA / "astoria-full-area.sumocfg"), "-n", str(network), "-r", route_files, "--begin", str(start_time), "--seed", str(seed), "--collision.check-junctions", "true", "--collision.action", "warn"]
        if traffic_mode != "baseline":
            args += ["-a", str(DATA / "astoria.poly.xml") + "," + str(receiving_constraint(directory, traffic_mode, start_time + 900))]
        check = subprocess.run([str(SUMO)] + args + ["--end", str(start_time + .2), "--no-step-log", "true"],
                               capture_output=True, text=True, timeout=45)
        if check.returncode:
            raise ValueError("SUMO rejected those edits: " + check.stderr[-1200:])
        return {"network": network, "routes": routes, "args": args,
                "edits": edits, "planned": planned}

    def open_netedit(self):
        if ScenarioEditor.netedit_process is not None and ScenarioEditor.netedit_process.poll() is None:
            return {"file": str(USER_NET), "message": "NetEdit is already running. Look for its window in XQuartz on Mac or on your taskbar on Windows."}
        executable = SUMO.with_name("netedit" + SUMO.suffix)
        if not executable.exists():
            raise ValueError("NetEdit is not installed. Install the full SUMO package to use the advanced editor.")
        environment = os.environ.copy()
        if sys.platform == "darwin":
            xquartz = Path("/Applications/Utilities/XQuartz.app")
            if not xquartz.exists():
                raise ValueError("NetEdit needs XQuartz on Mac. Install it from xquartz.org; the browser editor works without it.")
            subprocess.run(["/usr/bin/open", "-a", str(xquartz)], check=True, timeout=10)
            display = subprocess.run(["/bin/launchctl", "getenv", "DISPLAY"], capture_output=True, text=True, timeout=3).stdout.strip()
            displays = list(dict.fromkeys(d for d in (display, environment.get("DISPLAY"), ":0") if d))
            # XQuartz can take a moment to start accepting display connections.
            for attempt in range(50):
                ready = False
                for candidate_display in displays:
                    environment["DISPLAY"] = candidate_display
                    try:
                        ready = subprocess.run(["/opt/X11/bin/xdpyinfo"], env=environment,
                                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=.5).returncode == 0
                    except (OSError, subprocess.TimeoutExpired):
                        ready = False
                    if ready:
                        break
                if ready:
                    break
                time.sleep(.2)
            else:
                raise ValueError("XQuartz did not become ready. Open XQuartz from Applications → Utilities, then try NetEdit again.")
        if not USER_NET.exists():
            shutil.copy2(self.base_net, USER_NET)
        with tempfile.TemporaryFile(mode="w+") as log:
            process = subprocess.Popen([str(executable), "-s", str(USER_NET), "--output-file", str(USER_NET)],
                                       env=environment, stdout=log, stderr=subprocess.STDOUT)
            try:
                process.wait(timeout=.8)
            except subprocess.TimeoutExpired:
                ScenarioEditor.netedit_process = process
                return {"file": str(USER_NET), "message": "NetEdit is running in its own window. Save there, then use Reload saved network here."}
            log.seek(0)
            detail = log.read()[-1000:]
        raise ValueError("NetEdit exited before opening. " + detail)

    def save(self, edits):
        clean = self.validate(edits)
        clean.pop("effects", None)
        text = json.dumps(clean, indent=2) + "\n"
        with tempfile.NamedTemporaryFile(mode="w", dir=DATA, suffix=".tmp", delete=False, encoding="utf-8") as f:
            f.write(text)
            temporary = f.name
        os.replace(temporary, EDIT_FILE)
