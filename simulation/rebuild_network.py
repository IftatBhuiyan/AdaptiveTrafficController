#!/usr/bin/env python3
"""Reapply the reviewed site corrections to the original SUMO import."""
from pathlib import Path
import json
import math
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from sumo_runtime import configure

HERE = Path(__file__).resolve().parent / "astoria-osm-import"

if __name__ == "__main__":
    home, sumo = configure()
    import sumolib
    netconvert = sumo.with_name("netconvert" + sumo.suffix)
    with tempfile.TemporaryDirectory(prefix="astoria-network-") as folder:
        intermediate = Path(folder) / "reviewed.net.xml"
        subprocess.run([str(netconvert), "--sumo-net-file", str(HERE / "astoria.net.xml"),
                        "--node-files", str(HERE / "site-corrections.nod.xml"),
                        "--edge-files", str(HERE / "site-corrections.edg.xml"),
                        "--connection-files", str(HERE / "site-corrections.con.xml"),
                        "--tls.default-type", "static", "--tls.cycle.time", "90",
                        "--output-file", str(intermediate)], check=True)
        network = sumolib.net.readNet(str(intermediate))
        existing = [j for j in network.getNodes() if "traffic_light" in j.getType()]
        disabled, added, transfers = set(), set(), {}
        for source in existing:
            incoming = [e for e in source.getIncoming() if e.allows("passenger")]
            outgoing = [e for e in source.getOutgoing() if e.allows("passenger")]
            if len(incoming) != 1 or len(outgoing) != 1:
                continue
            distance, node, seen = 0, source, set()
            while len(outgoing) == 1 and distance < 80 and node.getID() not in seen:
                seen.add(node.getID())
                distance += outgoing[0].getLength()
                node = outgoing[0].getToNode()
                incoming = [e for e in node.getIncoming() if e.allows("passenger")]
                outgoing = [e for e in node.getOutgoing() if e.allows("passenger")]
                if len(incoming) >= 2 and len(outgoing) >= 2:
                    if distance < 80:
                        disabled.add(source.getID())
                        if "traffic_light" not in node.getType():
                            added.add(node.getID())
                        transfers[source.getID()] = node.getID()
                    break
        # Complete fixed-time control for genuine multi-approach surface
        # junctions. These added controls are scenario assumptions, not a claim
        # that every location has been surveyed. Do not signalize freeway merges.
        for node in network.getNodes():
            if "traffic_light" in node.getType():
                continue
            incoming = [e for e in node.getIncoming() if e.allows("passenger")]
            outgoing = [e for e in node.getOutgoing() if e.allows("passenger")]
            neighbours = {e.getFromNode().getID() for e in incoming} | {e.getToNode().getID() for e in outgoing}
            if len(incoming) < 2 or len(outgoing) < 2 or len(neighbours) < 3:
                continue
            if any("motorway" in e.getType() for e in incoming + outgoing):
                continue
            if min((math.dist(node.getCoord(), light.getCoord()) for light in existing), default=1000) > 25:
                added.add(node.getID())
        patch = ET.Element("nodes")
        for jid in sorted(disabled):
            ET.SubElement(patch, "node", id=jid, type="priority")
        for jid in sorted(added):
            ET.SubElement(patch, "node", id=jid, type="traffic_light", tlType="static")
        ET.indent(patch)
        patch_file = HERE / "intersection-controls.nod.xml"
        ET.ElementTree(patch).write(patch_file, encoding="utf-8", xml_declaration=True)
        subprocess.run([str(netconvert), "-s", str(intermediate), "-n", str(patch_file),
                        "--tls.default-type", "static", "--tls.cycle.time", "90",
                        "-o", str(HERE / "astoria-corrected.net.xml")], check=True)
        audit = {"added_fixed_time_controls": sorted(added), "removed_approach_controls": sorted(disabled),
                 "approach_to_intersection": transfers,
                 "basis": "OSM approach-marker relocation and inferred control for multi-approach surface junctions; field verification and real timing plans remain needed."}
        (HERE / "intersection-controls.json").write_text(json.dumps(audit, indent=2) + "\n")
        print(f"Added {len(added)} intersection controls; removed {len(disabled)} approach-only controls.")
