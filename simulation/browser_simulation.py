#!/usr/bin/env python3
"""Serve the project's actual SUMO traffic simulation on localhost.

No web framework or map service is required. SUMO computes all vehicle motion;
the browser draws its lane geometry, traffic lights, and live vehicle positions.
"""
from contextlib import ExitStack
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import hashlib
import json
import os
from pathlib import Path
import queue
import sys
import subprocess
import tempfile
import threading
import time
import xml.etree.ElementTree as ET
from sumo_runtime import configure
from scenario_editor import ScenarioEditor, DRAWING_KINDS
from traffic_behavior import MODES
from adaptive_controller import AdaptiveController, MODES as ADAPTIVE_MODES
from intelligent_coordinator import IntelligentCoordinator

CONTROLLER_MODES = ADAPTIVE_MODES | {"intelligent"}


def make_controller(network, selected):
    """The intelligent coordinator warms up on the imported plan, then decides."""
    if selected == "intelligent":
        return IntelligentCoordinator(traci, network, "fixed")
    return AdaptiveController(traci, network, "fixed")


def active_mode(selected):
    return "coordinated" if selected == "intelligent" else selected

HERE = Path(__file__).resolve().parent
DATA = HERE / "astoria-osm-import"
SUMO_HOME, SUMO_BINARY = configure()
import traci
import traci.constants as tc

BEGIN, END = 56700, 66600
commands = queue.Queue()
snapshot = {"ready": False, "running": False, "error": None}
network_data = None
stop_event = threading.Event()
editor = ScenarioEditor()
edit_job = {"status": "idle", "error": None, "id": 0}
edit_lock = threading.Lock()


def points(value):
    return [[float(n) for n in pair.split(",")] for pair in value.split()]


def geometry(network_file, route_file, edits):
    osm = ET.parse(DATA / "map.osm").getroot()
    names = {}
    for way in osm.findall("way"):
        tags = {t.get("k"): t.get("v") for t in way.findall("tag")}
        names[way.get("id")] = tags.get("name", "")
    root = ET.parse(network_file).getroot()
    lanes, junctions, polygons, signals, stops = [], [], [], [], []
    inventory = json.loads((DATA / "intersection-controls.json").read_text()).get("junctions", {})
    lane_lookup = {}
    edge_nodes = {}
    demand = json.loads((DATA / "full-area-demand.json").read_text())
    counted = {edge for edges in demand["station_edges"].values() for edge in edges}
    for edge in root.findall("edge"):
        edge_id = edge.get("id")
        edge_nodes[edge_id] = edge.get("to")
        name = edge.get("name", names.get(edge_id.lstrip("-").split("#")[0], ""))
        function = edge.get("function")
        vehicle_lanes = [int(l.get("index")) for l in edge.findall("lane") if l.get("allow") != "pedestrian"]
        for lane in edge.findall("lane"):
            walk = function in {"crossing", "walkingarea"} or (not function and lane.get("allow") == "pedestrian")
            item = {"id": lane.get("id"), "edge": edge_id, "name": name,
                    "index": int(lane.get("index")), "laneCount": len(edge.findall("lane")),
                    "shape": points(lane.get("shape")), "width": float(lane.get("width", "3.2")),
                    "speed": float(lane.get("speed")), "closed": lane.get("disallow") == "all",
                    "internal": function in {"internal", "crossing", "walkingarea"}, "route": edge_id in counted,
                    # Pedestrian infrastructure is drawn separately: walking areas are polygons, crossings zebras.
                    "pedestrian": ("walkingarea" if function == "walkingarea" else "crossing" if function == "crossing" else "sidewalk") if walk else None,
                    "curb": bool(vehicle_lanes) and int(lane.get("index")) == min(vehicle_lanes)}
            lanes.append(item)
            lane_lookup[item["id"]] = item
    for junction in root.findall("junction"):
        if junction.get("shape"):
            junctions.append({"id": junction.get("id"), "shape": points(junction.get("shape")),
                              "x": float(junction.get("x")), "y": float(junction.get("y")),
                              "controlled": "traffic_light" in junction.get("type", ""),
                              "controlType": junction.get("type"),
                              "controlSource": "Your saved override" if junction.get("id") in edits.get("controls", {}) or junction.get("id") in edits.get("signals", {}) else inventory.get(junction.get("id"), {}).get("basis", "Imported network"),
                              "reviewNeeded": inventory.get(junction.get("id"), {}).get("review_needed", False),
                              "streets": inventory.get(junction.get("id"), {}).get("streets", []),
                              "editable": not junction.get("id").startswith(":")})
    for connection in root.findall("connection"):
        if connection.get("state") in {"s", "w"}:
            lane = lane_lookup.get(connection.get("from") + "_" + connection.get("fromLane"))
            if lane and not lane["internal"]:
                stops.append({"junction": edge_nodes[connection.get("from")], "point": lane["shape"][-1]})
        if connection.get("tl"):
            lane_id = connection.get("from") + "_" + connection.get("fromLane")
            lane = lane_lookup.get(lane_id)
            if lane:
                jid = edge_nodes[connection.get("from")]
                delta = edits.get("markers", {}).get(jid, [0, 0])
                signals.append({"tls": connection.get("tl"), "index": int(connection.get("linkIndex")),
                                "junction": jid,
                                "point": [lane["shape"][-1][0] + delta[0], lane["shape"][-1][1] + delta[1]]})
    poly_file = DATA / "astoria.poly.xml"
    if poly_file.exists():
        for poly in ET.parse(poly_file).getroot().findall("poly"):
            edit = edits.get("artifacts", {}).get(poly.get("id"), {})
            polygons.append({"id": poly.get("id"), "type": poly.get("type", ""),
                             "shape": edit.get("shape", points(poly.get("shape"))),
                             "hidden": edit.get("hidden", False), "color": edit.get("color")})
    for did, drawing in edits.get("drawings", {}).items():
        polygons.append({"id": did, "type": "drawing." + drawing["kind"], "shape": drawing["shape"], "hidden": False,
                         "color": drawing.get("color") or DRAWING_KINDS[drawing["kind"]], "label": drawing.get("label", ""), "drawing": True,
                         # SUMO draws simulated sidewalks and crossings itself; the sketch is only shown while editing.
                         "simulated": edits.get("effects", {}).get("notes", {}).get(did, "").startswith("Simulated") and drawing["kind"] in {"sidewalk", "crosswalk"}})
    bounds = [float(v) for v in root.find("location").get("convBoundary").split(",")]
    return {"lanes": lanes, "junctions": junctions, "polygons": polygons, "signals": signals, "stops": stops,
            "edits": {k: v for k, v in edits.items() if k != "effects"}, "surface_notes": edits.get("effects", {}).get("notes", {}),
            "bounds": bounds, "core_bounds": demand["core_bounds"],
            "origins": [lane_lookup[e + "_0"]["shape"][0] for e in demand["origins"]],
            "exits": [lane_lookup[e + "_0"]["shape"][-1] for e in demand["exits"]],
            "paths": {r.get("id"): r.get("edges").split() for r in ET.parse(route_file).getroot().findall("route")},
            "revision": hashlib.sha256(Path(network_file).read_bytes() + json.dumps(edits, sort_keys=True).encode()).hexdigest()[:12],
            "title": "Astoria Boulevard"}


def simulate():
    global snapshot, network_data, editor, edit_job
    temporary = ExitStack()
    try:
        traffic_mode, target_time = "merge", 57600
        controller_mode = "intelligent"
        candidate = editor.build(editor.read(), temporary.enter_context(tempfile.TemporaryDirectory(prefix="astoria-session-")), traffic_mode)
        network_data = geometry(candidate["network"], candidate["routes"], candidate["edits"])
        demand = json.loads((DATA / "full-area-demand.json").read_text())
        observed = candidate["planned"]
        args = candidate["args"] + ["--no-warnings", "true"]
        traci.start([str(SUMO_BINARY)] + args)
        controller = make_controller(candidate["network"], controller_mode)
        network_data["control_area"] = controller.topology.metadata()
        running, speed = True, 4
        entered, arrived, teleports, collisions = 0, 0, 0, 0
        warmup_until = target_time
        step_count = 0
        visited = set()
        while not stop_event.is_set():
            while not commands.empty():
                command = commands.get_nowait()
                if command.get("action") == "pause":
                    running = False
                elif command.get("action") == "play":
                    running = not (collisions or teleports)
                elif command.get("action") == "speed":
                    speed = command["value"]
                elif command.get("action") in {"restart", "controller"}:
                    if command.get("action") == "controller":
                        controller_mode = command["value"]
                    traci.load(args)
                    controller = make_controller(candidate["network"], controller_mode)
                    entered = arrived = teleports = collisions = 0
                    visited = set()
                    warmup_until = target_time
                    running = True
                elif command.get("action") in {"apply-edits", "reload-network", "traffic"}:
                    running = False
                    snapshot = {**snapshot, "running": False, "updating": True}
                    edit_job = {"status": "building", "error": None, "id": edit_job["id"]}
                    switched = False
                    try:
                        new_editor = ScenarioEditor()
                        edits = command["edits"] if "edits" in command else new_editor.read()
                        folder = temporary.enter_context(tempfile.TemporaryDirectory(prefix="astoria-edit-"))
                        requested = command.get("value", {}) if command.get("action") == "traffic" else {}
                        new_mode = requested.get("mode", traffic_mode)
                        new_time = requested.get("hour", target_time // 3600) * 3600
                        built = new_editor.build(edits, folder, new_mode, new_time - 900)
                        new_geometry = geometry(built["network"], built["routes"], built["edits"])
                        new_args = built["args"] + ["--no-warnings", "true"]
                        traci.load(new_args)
                        switched = True
                        new_controller = make_controller(built["network"], controller_mode)
                        if command.get("action") == "apply-edits":
                            new_editor.save(built["edits"])
                        editor, candidate, args = new_editor, built, new_args
                        traffic_mode, target_time = new_mode, new_time
                        network_data = new_geometry
                        controller = new_controller
                        network_data["control_area"] = controller.topology.metadata()
                        observed = built["planned"]
                        entered = arrived = teleports = collisions = 0
                        visited = set()
                        warmup_until = target_time
                        running = True
                        edit_job = {"status": "saved", "error": None, "id": edit_job["id"]}
                    except Exception as error:
                        if switched:
                            traci.load(args)
                            controller = make_controller(candidate["network"], controller_mode)
                            entered = arrived = teleports = collisions = 0
                            visited = set()
                        edit_job = {"status": "failed", "error": str(error), "id": edit_job["id"]}
            started = time.monotonic()
            if running and traci.simulation.getTime() < END:
                traci.simulationStep()
                departed = traci.simulation.getDepartedIDList()
                entered += len(departed)
                arrived += traci.simulation.getArrivedNumber()
                teleports += traci.simulation.getStartingTeleportNumber()
                collisions += traci.simulation.getCollidingVehiclesNumber()
                for person in traci.simulation.getDepartedPersonIDList():
                    traci.person.subscribe(person, (tc.VAR_POSITION, tc.VAR_ANGLE, tc.VAR_SPEED))
                for vehicle in departed:
                    traci.vehicle.subscribe(vehicle, (tc.VAR_POSITION, tc.VAR_ANGLE, tc.VAR_SPEED,
                                                     tc.VAR_LENGTH, tc.VAR_WIDTH, tc.VAR_VEHICLECLASS, tc.VAR_ROAD_ID,
                                                     tc.VAR_ROUTE_ID, tc.VAR_LANE_ID, tc.VAR_LANEPOSITION))
                if collisions or teleports:
                    running = False
            current = traci.simulation.getTime()
            if running:
                controller.mode = active_mode(controller_mode) if current >= target_time else "fixed"
                controller.step(current)
            step_count += 1
            if running and current < warmup_until and step_count % 10:
                continue
            vehicles = []
            for vehicle, values in traci.vehicle.getAllSubscriptionResults().items():
                vehicles.append([vehicle, *values[tc.VAR_POSITION], values[tc.VAR_ANGLE],
                                 values[tc.VAR_SPEED], values[tc.VAR_LENGTH], values[tc.VAR_WIDTH],
                                 values[tc.VAR_VEHICLECLASS], values[tc.VAR_ROAD_ID],
                                 values[tc.VAR_ROUTE_ID], values[tc.VAR_LANE_ID]])
                visited.add(values[tc.VAR_ROAD_ID])
            persons = [[pid, round(v[tc.VAR_POSITION][0], 2), round(v[tc.VAR_POSITION][1], 2), round(v[tc.VAR_ANGLE]), round(v[tc.VAR_SPEED], 2)]
                       for pid, v in traci.person.getAllSubscriptionResults().items()]
            tls = {light: traci.trafficlight.getRedYellowGreenState(light) for light in traci.trafficlight.getIDList()}
            if current >= END:
                running = False
            snapshot = {"ready": True, "running": running, "time": current, "speed": speed,
                        "vehicles": vehicles, "persons": persons, "tls": tls, "entered": entered, "arrived": arrived,
                        "waiting": sum(v[4] < 0.1 for v in vehicles), "observed": observed,
                        "teleports": teleports, "collisions": collisions, "error": None,
                        "visited_edges": sorted(visited), "scenario": "full-area-corrected",
                        "app": "astoria-traffic-simulation",
                        "revision": network_data["revision"], "updating": False,
                        "traffic_mode": traffic_mode, "target_hour": target_time // 3600,
                        "controller_mode": controller_mode, "controller": controller.status(),
                        "safety_stop": collisions > 0 or teleports > 0,
                        "edit_error": edit_job["error"] if edit_job["status"] == "failed" else None,
                        "ramp_waiting": sum(v[4] < .1 and v[8] in {"25345887", "964223914", "1134690192"} for v in vehicles),
                        "pending": len(traci.simulation.getPendingVehicles()),
                        "count_driven": demand["count_driven_vehicles"],
                        "estimated_context": demand["estimated_context_vehicles"],
                        "warming": current < warmup_until}
            if current < warmup_until and running:
                continue
            time.sleep(max(0.001, (0.2 / speed if running else 0.1) - (time.monotonic() - started)))
    except Exception as error:
        snapshot = {**snapshot, "ready": False, "running": False, "error": str(error)}
        raise
    finally:
        if traci.isLoaded():
            traci.close()
        temporary.close()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def send(self, value, content_type="application/json", status=200):
        content = value if isinstance(value, bytes) else json.dumps(value).encode()
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(content)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        path = self.path.split("?")[0]
        if path == "/":
            self.send((HERE / "traffic_view.html").read_bytes(), "text/html; charset=utf-8")
        elif path == "/state":
            self.send(snapshot)
        elif path == "/network":
            self.send(network_data)
        elif path == "/traffic_view.js":
            self.send((HERE / "traffic_view.js").read_bytes(), "text/javascript; charset=utf-8")
        elif path in {"/editor_geometry.mjs", "/map_editor.js"}:
            self.send((HERE / path[1:]).read_bytes(), "text/javascript; charset=utf-8")
        elif path == "/editor":
            try:
                self.send({**editor.catalog(), "job": edit_job})
            except ValueError as error:
                self.send({"error": str(error)}, status=400)
        elif path == "/editor-job":
            self.send(edit_job)
        else:
            self.send_error(404)

    def do_POST(self):
        global edit_job
        origin = self.headers.get("Origin")
        port = self.server.server_address[1]
        if origin and origin not in {f"http://127.0.0.1:{port}", f"http://localhost:{port}"}:
            self.send({"error": "Edits must come from this local simulation page."}, status=403)
            return
        if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
            self.send({"error": "Expected JSON."}, status=415)
            return
        if self.path not in {"/control", "/route-preview", "/apply-edits", "/netedit", "/surface-preview"}:
            self.send_error(404)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length < 8 * 1024 * 1024:
                raise ValueError("Invalid request size")
            command = json.loads(self.rfile.read(length))
            if not isinstance(command, dict):
                raise ValueError("Expected an object.")
            if self.path == "/route-preview":
                self.send(editor.preview(command.get("route"), command.get("id", "")))
                return
            if self.path == "/surface-preview":
                effects = editor.validate(command)["effects"]
                self.send({"notes": effects["notes"]})
                return
            if self.path == "/netedit":
                self.send(editor.open_netedit())
                return
            if self.path == "/apply-edits":
                validated = editor.validate(command)
                with edit_lock:
                    if edit_job["status"] in {"queued", "building"}:
                        self.send({"error": "An edit is already being applied."}, status=409)
                        return
                    edit_job = {"status": "queued", "error": None, "id": time.time_ns()}
                commands.put({"action": "apply-edits", "edits": validated})
                self.send(edit_job)
                return
            if command.get("action") not in {"play", "pause", "restart", "speed", "reload-network", "traffic", "controller"}:
                raise ValueError("Unknown action")
            if command["action"] == "traffic":
                value = command.get("value")
                if not isinstance(value, dict) or value.get("mode") not in MODES or value.get("hour") not in {16, 17}:
                    raise ValueError("Choose 4 pm or 5 pm and a traffic scenario.")
            if command["action"] == "speed" and command.get("value") not in {1, 4, 10, 20}:
                raise ValueError("Unsupported speed")
            if command["action"] == "controller" and command.get("value") not in CONTROLLER_MODES:
                raise ValueError("Choose intelligent, coordinated, local, or fixed signal control.")
            commands.put(command)
            self.send({"ok": True})
        except (ValueError, TypeError, KeyError, OSError, subprocess.SubprocessError) as error:
            self.send({"error": str(error)}, status=400)


def main(port=8011, open_browser=False):
    import webbrowser
    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    worker = threading.Thread(target=simulate, daemon=True)
    worker.start()
    url = f"http://127.0.0.1:{port}"
    print(f"Astoria simulation: {url}\nKeep this terminal open. Press Ctrl+C to stop.", flush=True)
    if open_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        server.server_close()
        worker.join(timeout=5)


if __name__ == "__main__":
    main()
