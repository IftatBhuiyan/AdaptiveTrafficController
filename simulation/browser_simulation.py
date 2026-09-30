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
import tempfile
import threading
import time
import xml.etree.ElementTree as ET
from sumo_runtime import configure
from scenario_editor import ScenarioEditor

HERE = Path(__file__).resolve().parent
DATA = HERE / "astoria-osm-import"
SUMO_HOME, SUMO_BINARY = configure()
import traci
import traci.constants as tc

BEGIN, END = 57600, 66600
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
    lanes, junctions, polygons, signals = [], [], [], []
    lane_lookup = {}
    edge_nodes = {}
    demand = json.loads((DATA / "full-area-demand.json").read_text())
    counted = {edge for edges in demand["station_edges"].values() for edge in edges}
    for edge in root.findall("edge"):
        edge_id = edge.get("id")
        edge_nodes[edge_id] = edge.get("to")
        name = edge.get("name", names.get(edge_id.lstrip("-").split("#")[0], ""))
        for lane in edge.findall("lane"):
            item = {"id": lane.get("id"), "edge": edge_id, "name": name,
                    "index": int(lane.get("index")), "laneCount": len(edge.findall("lane")),
                    "shape": points(lane.get("shape")), "width": float(lane.get("width", "3.2")),
                    "speed": float(lane.get("speed")), "closed": lane.get("disallow") == "all",
                    "internal": edge.get("function") == "internal", "route": edge_id in counted}
            lanes.append(item)
            lane_lookup[item["id"]] = item
    for junction in root.findall("junction"):
        if junction.get("shape"):
            junctions.append({"id": junction.get("id"), "shape": points(junction.get("shape")),
                              "x": float(junction.get("x")), "y": float(junction.get("y")),
                              "controlled": "traffic_light" in junction.get("type", ""),
                              "editable": not junction.get("id").startswith(":")})
    for connection in root.findall("connection"):
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
    bounds = [float(v) for v in root.find("location").get("convBoundary").split(",")]
    return {"lanes": lanes, "junctions": junctions, "polygons": polygons, "signals": signals,
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
        candidate = editor.build(editor.read(), temporary.enter_context(tempfile.TemporaryDirectory(prefix="astoria-session-")))
        network_data = geometry(candidate["network"], candidate["routes"], candidate["edits"])
        demand = json.loads((DATA / "full-area-demand.json").read_text())
        observed = candidate["planned"]
        args = candidate["args"] + ["--no-warnings", "true"]
        traci.start([str(SUMO_BINARY)] + args)
        running, speed = True, 4
        entered, arrived, teleports, collisions = 0, 0, 0, 0
        warmup_until = BEGIN + 300
        visited = set()
        while not stop_event.is_set():
            while not commands.empty():
                command = commands.get_nowait()
                if command.get("action") == "pause":
                    running = False
                elif command.get("action") == "play":
                    running = True
                elif command.get("action") == "speed":
                    speed = command["value"]
                elif command.get("action") == "restart":
                    traci.load(args)
                    entered = arrived = teleports = collisions = 0
                    visited = set()
                    warmup_until = BEGIN + 300
                    running = True
                elif command.get("action") in {"apply-edits", "reload-network"}:
                    running = False
                    snapshot = {**snapshot, "running": False, "updating": True}
                    edit_job = {"status": "building", "error": None, "id": edit_job["id"]}
                    switched = False
                    try:
                        new_editor = ScenarioEditor()
                        edits = command["edits"] if "edits" in command else new_editor.read()
                        folder = temporary.enter_context(tempfile.TemporaryDirectory(prefix="astoria-edit-"))
                        built = new_editor.build(edits, folder)
                        new_geometry = geometry(built["network"], built["routes"], built["edits"])
                        new_args = built["args"] + ["--no-warnings", "true"]
                        traci.load(new_args)
                        switched = True
                        new_editor.save(built["edits"])
                        editor, candidate, args = new_editor, built, new_args
                        network_data = new_geometry
                        observed = built["planned"]
                        entered = arrived = teleports = collisions = 0
                        visited = set()
                        warmup_until = BEGIN + 300
                        running = True
                        edit_job = {"status": "saved", "error": None, "id": edit_job["id"]}
                    except Exception as error:
                        if switched:
                            traci.load(args)
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
                for vehicle in departed:
                    traci.vehicle.subscribe(vehicle, (tc.VAR_POSITION, tc.VAR_ANGLE, tc.VAR_SPEED,
                                                     tc.VAR_LENGTH, tc.VAR_WIDTH, tc.VAR_VEHICLECLASS, tc.VAR_ROAD_ID,
                                                     tc.VAR_ROUTE_ID, tc.VAR_LANE_ID))
            current = traci.simulation.getTime()
            vehicles = []
            for vehicle, values in traci.vehicle.getAllSubscriptionResults().items():
                vehicles.append([vehicle, *values[tc.VAR_POSITION], values[tc.VAR_ANGLE],
                                 values[tc.VAR_SPEED], values[tc.VAR_LENGTH], values[tc.VAR_WIDTH],
                                 values[tc.VAR_VEHICLECLASS], values[tc.VAR_ROAD_ID],
                                 values[tc.VAR_ROUTE_ID], values[tc.VAR_LANE_ID]])
                visited.add(values[tc.VAR_ROAD_ID])
            tls = {light: traci.trafficlight.getRedYellowGreenState(light) for light in traci.trafficlight.getIDList()}
            if current >= END:
                running = False
            snapshot = {"ready": True, "running": running, "time": current, "speed": speed,
                        "vehicles": vehicles, "tls": tls, "entered": entered, "arrived": arrived,
                        "waiting": sum(v[4] < 0.1 for v in vehicles), "observed": observed,
                        "teleports": teleports, "collisions": collisions, "error": None,
                        "visited_edges": sorted(visited), "scenario": "full-area-corrected",
                        "app": "astoria-traffic-simulation",
                        "revision": network_data["revision"], "updating": False,
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
        if self.path not in {"/control", "/route-preview", "/apply-edits", "/netedit"}:
            self.send_error(404)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length < 262144:
                raise ValueError("Invalid request size")
            command = json.loads(self.rfile.read(length))
            if not isinstance(command, dict):
                raise ValueError("Expected an object.")
            if self.path == "/route-preview":
                self.send(editor.preview(command.get("route"), command.get("id", "")))
                return
            if self.path == "/netedit":
                self.send({"file": editor.open_netedit()})
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
            if command.get("action") not in {"play", "pause", "restart", "speed", "reload-network"}:
                raise ValueError("Unknown action")
            if command["action"] == "speed" and command.get("value") not in {1, 4, 10, 20}:
                raise ValueError("Unsupported speed")
            commands.put(command)
            self.send({"ok": True})
        except (ValueError, TypeError, KeyError, OSError) as error:
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
