#!/usr/bin/env python3
"""Build the baseline-v2 network (baseline-v2.net.xml) from the OSM map.

Steps (all inputs are in this folder or in ../astoria-osm-import):
 1. Re-import map.osm with the same options as astoria.net.xml, plus
    --osm.turn-lanes true, so turn lanes follow the map's turn:lanes tags.
 2. Run the team's rebuild_network.py on that import, using this folder's
    site-corrections.edg.xml / .con.xml (the repo ones plus the bus-lane and
    dead-lane fixes) and the repo's site-corrections.nod.xml.
 3. Give the junction under I-278 a protected signal program with all-red
    (cluster-under-i278.tll.xml).
 4. Join junction pairs that were split by 0.2-1 m edges (junction-joins.nod.xml).
 5. Remove plain geometry nodes (netconvert --geometry.remove). This merges the
    4-8 m pieces of Astoria Blvd S before 33rd St (and 6 other pieces) into
    their neighbours, so drivers have room to change lanes before the signal.

Usage:  python simulation/baseline-v2/build_network.py
The repo's own astoria.net.xml / astoria-corrected.net.xml are not changed.
"""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
SIM = HERE.parent
IMPORT = SIM / "astoria-osm-import"
sys.path.insert(0, str(SIM))

from sumo_runtime import configure  # noqa: E402
import rebuild_network  # noqa: E402


def main():
    _, sumo = configure()
    netconvert = str(sumo.with_name("netconvert" + sumo.suffix))
    with tempfile.TemporaryDirectory(prefix="baseline-v2-") as tmp:
        work = Path(tmp)
        shutil.copy(IMPORT / "map.osm", work / "map.osm")
        shutil.copy(IMPORT / "site-corrections.nod.xml", work / "site-corrections.nod.xml")
        for name in ("site-corrections.edg.xml", "site-corrections.con.xml"):
            shutil.copy(HERE / name, work / name)

        print("1/5 importing map.osm with turn lanes")
        subprocess.run([netconvert, "--osm-files", str(work / "map.osm"),
                        "--output.original-names", "true", "--proj.utm", "true",
                        "--geometry.remove", "true", "--roundabouts.guess", "true",
                        "--tls.guess-signals", "true", "--tls.guess-signals.slack", "1",
                        "--tls.cycle.time", "90", "--tls.default-type", "static",
                        "--ramps.guess", "true", "--keep-edges.by-vclass", "passenger",
                        "--junctions.join", "true", "--osm.turn-lanes", "true",
                        "--no-warnings", "true", "-o", str(work / "astoria.net.xml")], check=True)

        print("2/5 applying site corrections (rebuild_network.py)")
        rebuild_network.HERE = work
        rebuild_network.main()

        print("3/5 protected signal program for the junction under I-278")
        step3 = work / "step3.net.xml"
        subprocess.run([netconvert, "-s", str(work / "astoria-corrected.net.xml"),
                        "--tllogic-files", str(HERE / "cluster-under-i278.tll.xml"),
                        "--no-turnarounds", "true", "--offset.disable-normalization", "true",
                        "--no-warnings", "true", "-o", str(step3)], check=True)

        print("4/5 joining junctions split by sub-meter edges")
        step4 = work / "step4.net.xml"
        subprocess.run([netconvert, "-s", str(step3), "-n", str(HERE / "junction-joins.nod.xml"),
                        "--no-turnarounds", "true", "--offset.disable-normalization", "true",
                        "--no-warnings", "true", "-o", str(step4)], check=True)

        print("5/5 merging road pieces that are only geometry points")
        subprocess.run([netconvert, "-s", str(step4), "--geometry.remove", "true",
                        "--no-turnarounds", "true", "--offset.disable-normalization", "true",
                        "--no-warnings", "true", "-o", str(HERE / "baseline-v2.net.xml")], check=True)
    print("wrote", HERE / "baseline-v2.net.xml")


if __name__ == "__main__":
    main()
