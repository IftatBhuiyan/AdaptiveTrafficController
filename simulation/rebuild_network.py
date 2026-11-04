#!/usr/bin/env python3
"""Reapply site corrections and map-supported intersection controls."""
from pathlib import Path
import json
import math
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from sumo_runtime import configure

HERE = Path(__file__).resolve().parent / "astoria-osm-import"


def road_edges(node):
    return [e for e in node.getIncoming() + node.getOutgoing() if e.allows("passenger")]


def is_intersection(node):
    edges = road_edges(node)
    neighbours = {e.getFromNode().getID() for e in edges} | {e.getToNode().getID() for e in edges}
    neighbours.discard(node.getID())
    return len(neighbours) >= 3


def main():
    _, sumo = configure()
    import sumolib
    import traci
    converter = sumo.with_name("netconvert" + sumo.suffix)
    osm = ET.parse(HERE / "map.osm").getroot()
    ways = {w.get("id"): {n.get("ref") for n in w.findall("nd")} for w in osm.findall("way")}
    names = {w.get("id"): {t.get("k"): t.get("v") for t in w.findall("tag")}.get("name", "") for w in osm.findall("way")}
    with tempfile.TemporaryDirectory(prefix="astoria-network-") as folder:
        intermediate = Path(folder) / "reviewed.net.xml"
        subprocess.run([str(converter), "-s", str(HERE / "astoria.net.xml"),
                        "-n", str(HERE / "site-corrections.nod.xml"),
                        "-e", str(HERE / "site-corrections.edg.xml"),
                        "-x", str(HERE / "site-corrections.con.xml"),
                        "--tls.default-type", "static", "--tls.cycle.time", "90", "-o", str(intermediate)], check=True)
        net = sumolib.net.readNet(str(intermediate))
        changes, transfers, stop_sources, stop_edges, unresolved = {}, {}, {}, {}, []
        # A two-way approach has two incoming edges but is not necessarily a
        # street intersection. Keep actual mid-block crossings separate.
        for source in net.getNodes():
            if "traffic_light" not in source.getType() or is_intersection(source):
                continue
            queue, seen, targets = [(0, source)], set(), []
            while queue:
                distance, node = queue.pop(0)
                if node.getID() in seen or distance > 80:
                    continue
                seen.add(node.getID())
                if node is not source and is_intersection(node):
                    targets.append((distance, node))
                    continue
                for edge in node.getOutgoing():
                    if edge.allows("passenger"):
                        queue.append((distance + edge.getLength(), edge.getToNode()))
            if targets:
                distance, target = min(targets, key=lambda t: t[0])
                if distance < 40 or len([e for e in source.getIncoming() if e.allows("passenger")]) == 1:
                    changes[source.getID()] = "priority"
                    changes[target.getID()] = "traffic_light"
                    transfers[source.getID()] = target.getID()
        # Recover discarded stop markers only on the same OSM way and within
        # 35 m upstream. Do not replace unknown junctions with guessed lights.
        traci.start([str(sumo), "-n", str(intermediate), "--no-step-log", "true"])
        try:
            for source in osm.findall("node"):
                tags = {t.get("k"): t.get("v") for t in source.findall("tag")}
                if tags.get("highway") != "stop":
                    continue
                sid = source.get("id")
                point = traci.simulation.convertGeo(float(source.get("lon")), float(source.get("lat")), fromGeo=True)
                if net.hasNode(sid) and is_intersection(net.getNode(sid)):
                    candidates = [e for e in net.getNode(sid).getIncoming() if e.allows("passenger")]
                else:
                    candidates = []
                    for edge in net.getEdges():
                        eid = edge.getID()
                        if sid not in ways.get(eid.lstrip("-").split("#")[0], set()) or not edge.allows("passenger"):
                            continue
                        if tags.get("direction") == "forward" and eid.startswith("-"):
                            continue
                        if tags.get("direction") == "backward" and not eid.startswith("-"):
                            continue
                        if math.dist(point, edge.getToNode().getCoord()) < 35 and sumolib.geomhelper.distancePointToPolygon(point, edge.getShape()) < 8:
                            candidates.append(edge)
                if not candidates:
                    unresolved.append({"osm_node": sid, "reason": "No unambiguous retained approach within 35 m; may be outside the retained street network."})
                    continue
                edge = min(candidates, key=lambda e: math.dist(point, e.getToNode().getCoord()))
                node, jid = edge.getToNode(), edge.getToNode().getID()
                if not is_intersection(node):
                    unresolved.append({"osm_node": sid, "junction": jid, "reason": "Intersecting street is absent from retained network; stop not moved to an unrelated location."})
                    continue
                if "traffic_light" in changes.get(jid, node.getType()):
                    unresolved.append({"osm_node": sid, "junction": jid, "reason": "OSM stop marker conflicts with mapped signal; signal retained pending field review."})
                    continue
                allway = tags.get("stop") == "all"
                ambiguous = sid == jid and not allway
                changes[jid] = "allway_stop" if allway or ambiguous else "priority_stop"
                stop_sources.setdefault(jid, []).append(sid)
                if ambiguous:
                    unresolved.append({"osm_node": sid, "junction": jid, "reason": "Stop present; approach scope unspecified. All-way stop is a provisional assumption."})
                if not allway and not ambiguous:
                    stop_edges.setdefault(jid, set()).add(edge.getID())
        finally:
            traci.close()
        patch, priorities = ET.Element("nodes"), ET.Element("edges")
        for jid, kind in sorted(changes.items()):
            ET.SubElement(patch, "node", id=jid, type=kind)
        for jid, stopped in stop_edges.items():
            for edge in net.getNode(jid).getIncoming():
                if edge.allows("passenger"):
                    ET.SubElement(priorities, "edge", id=edge.getID(), priority="1" if edge.getID() in stopped else "10")
        ET.indent(patch); ET.indent(priorities)
        node_file, edge_file = HERE / "intersection-controls.nod.xml", HERE / "intersection-priorities.edg.xml"
        ET.ElementTree(patch).write(node_file, encoding="utf-8", xml_declaration=True)
        ET.ElementTree(priorities).write(edge_file, encoding="utf-8", xml_declaration=True)
        command = [str(converter), "-s", str(intermediate), "-n", str(node_file), "-e", str(edge_file),
                   "--tls.default-type", "static", "--tls.cycle.time", "90", "-o", str(HERE / "astoria-corrected.net.xml")]
        if transfers:
            command += ["--tls.unset", ",".join(transfers)]
        subprocess.run(command, check=True)
        final = sumolib.net.readNet(str(HERE / "astoria-corrected.net.xml"))
        inventory = {}
        for node in final.getNodes():
            if not road_edges(node):
                continue
            jid = node.getID()
            streets = sorted({names.get(e.getID().lstrip("-").split("#")[0], "") for e in road_edges(node)} - {""})
            basis = "OSM signal / reviewed junction" if "traffic_light" in node.getType() else "OSM road priority; control not field-verified"
            if jid in stop_sources:
                basis = "OSM stop markers " + ", ".join(stop_sources[jid])
            if jid in transfers:
                basis = "Approach signal relocated to " + transfers[jid]
            if all("motorway" in e.getType() for e in road_edges(node)):
                basis = "Freeway lane continuation / merge; no street signal"
            incoming = [e for e in node.getIncoming() if e.allows("passenger")]
            incoming_streets = {names.get(e.getID().lstrip("-").split("#")[0], "Unnamed approach") for e in incoming}
            if basis.startswith("OSM road priority") and len(incoming) < 2:
                basis = "Single incoming approach / road branch; no conflicting incoming street"
            elif basis.startswith("OSM road priority") and len(incoming_streets) == 1:
                basis = "Road continuation with a side exit; priority turns (not a missing signal)"
            conflicts = [item["reason"] for item in unresolved if item.get("junction") == jid and ("conflicts" in item["reason"] or "scope unspecified" in item["reason"])]
            if conflicts:
                basis += "; " + " ".join(conflicts)
            inventory[jid] = {"type": node.getType(), "streets": streets, "basis": basis,
                              "review_needed": bool(conflicts) or (basis.startswith("OSM road priority") and is_intersection(node))}
        audit = {"basis": "No traffic lights inferred solely from intersection size. Controls come from saved OSM tags and reviewed site corrections; timings remain assumed.",
                 "approach_to_intersection": transfers, "stop_sources": stop_sources,
                 "unresolved": unresolved, "junctions": inventory}
        (HERE / "intersection-controls.json").write_text(json.dumps(audit, indent=2) + "\n")
        print(f"Relocated {len(transfers)} approach signals; restored {len(stop_sources)} stop controls; {len(unresolved)} source ambiguities.")


if __name__ == "__main__":
    main()
