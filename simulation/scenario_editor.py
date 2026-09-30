"""Persistent route and signal edits for the shared browser simulation."""
from collections import Counter
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from sumo_runtime import configure

HOME, SUMO = configure()
import sumolib

DATA = Path(__file__).resolve().parent / "astoria-osm-import"
EDIT_FILE = DATA / "scenario-edits.json"
BASE_NET = DATA / "astoria-corrected.net.xml"
USER_NET = DATA / "user-network.net.xml"
BASE_ROUTES = DATA / "scenario-full-area.rou.xml"
DEFAULT = {"version": 1, "routes": {}, "signals": {}, "markers": {}, "roads": {}, "surfaces": {}, "artifacts": {}, "timings": {}}


class ScenarioEditor:
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

    def label(self, edge):
        name = self.names.get(edge.lstrip("-").split("#")[0], "")
        return (name + " · " if name else "Road · ") + edge

    def read(self):
        return self.validate(json.loads(EDIT_FILE.read_text()) if EDIT_FILE.exists() else DEFAULT)

    def catalog(self):
        return {"edits": self.read(),
                "routes": [{"id": rid, "from": edges[0], "to": edges[-1], "edges": edges,
                            "count": self.counts[rid], "label": f"{rid} · {self.label(edges[0])} → {self.label(edges[-1])}"}
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

    def shape(self, value):
        if not isinstance(value, list) or not 3 <= len(value) <= 200:
            raise ValueError("A shape needs between 3 and 200 corners.")
        if not all(isinstance(p, list) and len(p) == 2 and all(isinstance(v, (int, float)) and math.isfinite(v) for v in p) for p in value):
            raise ValueError("Invalid shape coordinates.")
        if value[0] == value[-1]:
            value = value[:-1]
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
        return clean

    def build(self, edits, directory):
        edits = self.validate(edits)
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        network = self.base_net
        if edits["signals"] or edits["roads"] or edits["surfaces"]:
            nodes = ET.Element("nodes")
            for jid in edits["signals"].keys() | edits["surfaces"].keys():
                attrs = {"id": jid}
                if jid in edits["signals"]:
                    attrs["type"] = "traffic_light" if edits["signals"][jid] else "priority"
                if jid in edits["surfaces"]:
                    attrs["shape"] = " ".join(f"{x:.3f},{y:.3f}" for x, y in edits["surfaces"][jid])
                ET.SubElement(nodes, "node", **attrs)
            node_file = directory / "signals.nod.xml"
            ET.ElementTree(nodes).write(node_file, encoding="utf-8", xml_declaration=True)
            network = directory / "edited.net.xml"
            converter = SUMO.with_name("netconvert" + SUMO.suffix)
            command = [str(converter), "-s", str(self.base_net), "-n", str(node_file), "-o", str(network),
                       "--tls.default-type", "static", "--tls.cycle.time", "90"]
            if edits["roads"]:
                roads = ET.Element("edges")
                resets = set()
                for eid, spec in edits["roads"].items():
                    attrs = {"id": eid, "numLanes": str(spec["lanes"]), "speed": str(spec["speed"])}
                    if spec["closed"]:
                        attrs["disallow"] = "all"
                    ET.SubElement(roads, "edge", **attrs)
                    if spec["lanes"] != len(self.edges[eid].getLanes()) or spec["closed"]:
                        resets.add(eid)
                        resets.update(e.getID() for e in self.edges[eid].getFromNode().getIncoming())
                road_file = directory / "roads.edg.xml"
                ET.ElementTree(roads).write(road_file, encoding="utf-8", xml_declaration=True)
                command += ["-e", str(road_file)]
                if resets:
                    connections = ET.Element("connections")
                    for eid in sorted(resets):
                        ET.SubElement(connections, "connection", **{"from": eid, "reset": "true"})
                    con_file = directory / "roads.con.xml"
                    ET.ElementTree(connections).write(con_file, encoding="utf-8", xml_declaration=True)
                    command += ["-x", str(con_file)]
            disabled = [jid for jid, enabled in edits["signals"].items() if not enabled]
            if disabled:
                command += ["--tls.unset", ",".join(disabled)]
            result = subprocess.run(command, capture_output=True, text=True, timeout=45)
            if result.returncode:
                raise ValueError("SUMO could not build those signal changes: " + result.stderr[-1200:])
        if edits["timings"]:
            tree = ET.parse(network)
            edge_nodes = {e.get("id"): e.get("to") for e in tree.getroot().findall("edge")}
            node_tls = {edge_nodes.get(c.get("from")): c.get("tl") for c in tree.getroot().findall("connection") if c.get("tl")}
            programs = {p.get("id"): p for p in tree.getroot().findall("tlLogic")}
            for jid, spec in edits["timings"].items():
                program = programs.get(node_tls.get(jid))
                if program is None:
                    if edits["signals"].get(jid) is False:
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
        if edits["roads"] or self.base_net == USER_NET:
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
        extra = 0
        for rid, spec in edits["routes"].items():
            edges = self.preview(spec, rid, edited_net)["edges"]
            if rid in lookup:
                lookup[rid].set("edges", " ".join(edges))
            else:
                ET.SubElement(root, "route", id=rid, edges=" ".join(edges))
                extra += round(spec["rate"] * 2)
                ET.SubElement(root, "flow", id=rid + "_flow", type="car", route=rid,
                              begin="57600", end="64800", number=str(round(spec["rate"] * 2)),
                              departLane="best", departSpeed="desired")
        definitions = [e for e in root if e.tag in ("vType", "route")]
        traffic = sorted([e for e in root if e.tag in ("vehicle", "flow")],
                         key=lambda e: float(e.get("depart", e.get("begin", "0"))))
        root[:] = definitions + traffic
        routes = directory / "edited.rou.xml"
        ET.ElementTree(root).write(routes, encoding="utf-8", xml_declaration=True)
        args = ["-c", str(DATA / "astoria-full-area.sumocfg"), "-n", str(network), "-r", str(routes)]
        check = subprocess.run([str(SUMO)] + args + ["--end", "57600.2", "--no-step-log", "true"],
                               capture_output=True, text=True, timeout=45)
        if check.returncode:
            raise ValueError("SUMO rejected those edits: " + check.stderr[-1200:])
        return {"network": network, "routes": routes, "args": args,
                "edits": edits, "planned": self.plan["total_vehicles"] + extra}

    def open_netedit(self):
        if not USER_NET.exists():
            shutil.copy2(self.base_net, USER_NET)
        executable = SUMO.with_name("netedit" + SUMO.suffix)
        if not executable.exists():
            raise ValueError("NetEdit is not installed. Install the full SUMO package to use the advanced editor.")
        environment = os.environ.copy()
        if os.name != "nt":
            environment.setdefault("DISPLAY", ":0")
        subprocess.Popen([str(executable), "-s", str(USER_NET), "--output-file", str(USER_NET)], env=environment)
        return str(USER_NET)

    def save(self, edits):
        text = json.dumps(self.validate(edits), indent=2) + "\n"
        with tempfile.NamedTemporaryFile(mode="w", dir=DATA, suffix=".tmp", delete=False, encoding="utf-8") as f:
            f.write(text)
            temporary = f.name
        os.replace(temporary, EDIT_FILE)
