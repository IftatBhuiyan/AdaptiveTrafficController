#!/usr/bin/env python3
"""Diagnose the I-278 EB off-ramp (NYSDOT station 053167) under fixed-time control.

Answers three questions for one SUMO run:
  1. How many vehicles cross the off-ramp every 30 minutes (vs. the NYSDOT count)?
  2. Which lanes fill up, how early, and for what share of the run?
  3. Where do off-ramp vehicles try to go after the ramp?

Usage (from simulation/astoria-osm-import):
  python ../analysis/ramp_diagnosis.py run --out ../../experiments/ramp-diag
  python ../analysis/ramp_diagnosis.py report --out ../../experiments/ramp-diag

`run` starts SUMO with the full-area scenario on user-network.net.xml and fixed-time
signals, writing queue, trip and ramp-count outputs. It takes 15-20 minutes.
`report` reads those outputs and prints the tables. Outputs stay out of git.
"""
import argparse, collections, json, os, shutil, subprocess, sys
import xml.etree.ElementTree as ET
from pathlib import Path

RAMP_FEED, RAMP, MAINLINE = "5706905", "959678463", "25346083"
STATION = "station_053167_2023-03-29"   # column in traffic-profiles.csv
BEGIN, END, PERIOD = 57600, 66600, 1800  # 4:00-6:30 PM, 30-minute bins
FULL = 0.9                               # lane "full" = queue >= 90% of lane length


def sumo_binary():
    home = os.environ.get("SUMO_HOME")
    if home and (Path(home) / "bin" / "sumo").exists():
        return str(Path(home) / "bin" / "sumo")
    found = shutil.which("sumo")
    if not found:
        sys.exit("SUMO not found: install SUMO 1.27.1 or set SUMO_HOME.")
    return found


def run(out):
    out.mkdir(parents=True, exist_ok=True)
    add = out / "ramp_counts.add.xml"
    add.write_text(f'<additional><edgeData id="ramp" file="{(out/"ramp_counts.xml").resolve()}" '
                   f'begin="{BEGIN}" end="{END}" period="{PERIOD}" excludeEmpty="true"/></additional>\n')
    cmd = [sumo_binary(), "-c", "astoria-full-area.sumocfg", "-n", "user-network.net.xml",
           "--additional-files", f"astoria.poly.xml,{add.resolve()}",
           "--queue-output", str(out / "queue.xml"), "--tripinfo-output", str(out / "trips.xml"),
           "--duration-log.statistics", "true", "--no-warnings", "true"]
    print("Running:", " ".join(cmd))
    subprocess.run(cmd, check=True)


def street_names():
    names = {}
    for _, el in ET.iterparse("map.osm"):
        if el.tag == "way":
            tags = {t.get("k"): t.get("v") for t in el if t.tag == "tag"}
            names[el.get("id")] = tags.get("name") or tags.get("ref") or tags.get("highway", "?")
        if el.tag in ("way", "node"):
            el.clear()
    return lambda edge: names.get(edge.lstrip("-").split("#")[0], edge)


def report(out):
    road = street_names()

    # 1) ramp crossings vs NYSDOT
    import csv
    rows = list(csv.reader(open("traffic-profiles.csv")))
    col = rows[0].index(STATION)
    start = [i for i, r in enumerate(rows) if r[0] == "04:00 pm"][0]
    measured = [sum(int(r[col]) for r in rows[start + 2*k:start + 2*k + 2]) for k in range(5)]
    model = [0] * 5
    for iv in ET.parse(out / "ramp_counts.xml").getroot().iter("interval"):
        k = int((float(iv.get("begin")) - BEGIN) // PERIOD)
        for e in iv.iter("edge"):
            if e.get("id") == RAMP and 0 <= k < 5:
                model[k] = int(e.get("left"))
    print("\n1) EB off-ramp vehicles per 30 min (4:00, 4:30, 5:00, 5:30, 6:00)")
    print("   NYSDOT count:", measured)
    print("   Model       :", model)

    # 2) lane fullness
    net = ET.parse("user-network.net.xml").getroot()
    length = {ln.get("id"): float(ln.get("length")) for e in net.iter("edge")
              if e.get("function") != "internal" for ln in e.findall("lane")}
    full, first, steps = collections.Counter(), {}, 0
    for _, el in ET.iterparse(out / "queue.xml"):
        if el.tag == "data":
            steps += 1; t = float(el.get("timestep"))
            for q in el.iter("lane"):
                lid = q.get("id")
                if lid in length and float(q.get("queueing_length")) >= FULL * length[lid]:
                    full[lid] += 1; first.setdefault(lid, t)
            el.clear()
    clock = lambda t: "%d:%02d" % (int(t)//3600 - 12, int(t) % 3600 // 60)
    print("\n2) Lanes full for more than half the run, in the order they filled")
    for lid in sorted((l for l in full if full[l] / steps > 0.5), key=lambda l: first[l])[:20]:
        print(f"   {clock(first[lid])} PM  {lid:20} {road(lid.rsplit('_', 1)[0])[:26]:26} "
              f"{length[lid]:5.0f} m  full {100*full[lid]/steps:3.0f}%")

    # 3) where off-ramp vehicles go next
    routes, nxt = {}, collections.Counter()
    for _, el in ET.iterparse("scenario-full-area.rou.xml"):
        if el.tag == "route" and el.get("id"):
            routes[el.get("id")] = el.get("edges").split()
        elif el.tag == "vehicle":
            edges = routes.get(el.get("route"), [])
            if RAMP in edges:
                i = edges.index(RAMP)
                after = [e for e in edges[i + 1:] if not road(e).startswith("Astoria Boulevard North")]
                nxt[road(after[0]) if after else "ends on Astoria Blvd N"] += 1
    total = sum(nxt.values())
    print(f"\n3) Where the {total} off-ramp vehicles go after Astoria Blvd N")
    for name, n in nxt.most_common():
        print(f"   {n:5} ({100*n/total:3.0f}%)  {name}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["run", "report"])
    ap.add_argument("--out", type=Path, default=Path("../../experiments/ramp-diag"))
    a = ap.parse_args()
    (run if a.command == "run" else report)(a.out)
