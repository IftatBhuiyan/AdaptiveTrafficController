#!/usr/bin/env python3
"""Build the baseline-v2 demand (baseline-v2.rou.xml) from the repo's evening demand.

Input: ../astoria-osm-import/scenario-full-area.rou.xml (3:45-6:00 PM, 14,094 vehicles)
Steps:
 1. Re-route on baseline-v2.net.xml with duarouter --repair (lane/turn fixes
    changed which turns exist).
 2. Driver setting: lcImpatience=1, lcTimeToImpatience=30 on both vTypes, so a
    driver blocked for 30 s accepts smaller gaps (SUMO default never does).
 3. U-turn fix: trips that leave I-278 at the EB off-ramp and get right back on
    at the WB on-ramp (959678463 ... 25345887) are split in two: the off-ramp
    car goes to a normal destination, and the on-ramp car comes along Hoyt Ave S,
    which is where NYSDOT station 053166 says WB on-ramp traffic comes from. Its
    start is drawn from the local streets where today's Hoyt Ave S traffic
    starts (not the RFK Bridge exit ramp).
    On-ramp trips that came round via 31st St under the highway (683926426)
    are re-sourced the same way.
 4. Loop fix: routes that pass the same junction twice, or are >30% longer than
    the direct path, are replaced by the direct path (they circled the block
    through the interchange).
Usage: python simulation/baseline-v2/make_demand.py
"""
from pathlib import Path
import collections
import random
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
SIM = HERE.parent
sys.path.insert(0, str(SIM))
from sumo_runtime import configure  # noqa: E402

NET = HERE / "baseline-v2.net.xml"
SOURCE = SIM / "astoria-osm-import" / "scenario-full-area.rou.xml"
OUT = HERE / "baseline-v2.rou.xml"
OFF_RAMP, WB_ON_RAMP, UNDER_I278 = "959678463", "25345887", "683926426"
HOYT_AT_31ST = "497263582#0"   # Hoyt Ave S entering the 31st St junction
IMPATIENCE = ' lcImpatience="1" lcTimeToImpatience="30"'


def duarouter(sumo, args):
    tool = str(sumo.with_name("duarouter" + sumo.suffix))
    subprocess.run([tool, "-n", str(NET), "--no-warnings", "true", "--ignore-errors", "true"] + args,
                   check=True, stdout=subprocess.DEVNULL)


def vehicles(text):
    return re.findall(r"<vehicle .*?</vehicle>", text, re.S)


def edges_of(block):
    return re.search(r'edges="([^"]+)"', block).group(1).split()


def resource(text, sumo, work):
    """Step 3: split off-ramp -> on-ramp U-turns and re-source 31st St on-ramp trips."""
    vtypes = re.findall(r"<vType [^>]*/>", text)
    blocks = vehicles(text)

    def kind(es):
        if WB_ON_RAMP not in es:
            return None
        before = es[:es.index(WB_ON_RAMP)]
        if OFF_RAMP in before:
            return "U"
        if UNDER_I278 in before:
            return "via31"
        return None

    rng = random.Random(13)
    pool = [edges_of(b)[-1] for b in blocks if OFF_RAMP in edges_of(b) and kind(edges_of(b)) != "U"]
    # On-ramp cars start where today's Hoyt Ave S traffic starts (same mix of
    # local streets, not the RFK Bridge ramp, which would be another U-turn),
    # and are routed via Hoyt Ave S at 31st St (HOYT_AT_31ST).
    import sumolib
    net = sumolib.net.readNet(str(NET))
    starts = [edges_of(b)[0] for b in blocks
              if HOYT_AT_31ST in edges_of(b) and edges_of(b).index(HOYT_AT_31ST) > 0 and not kind(edges_of(b))
              and "motorway" not in net.getEdge(edges_of(b)[0]).getType()]
    trips, drop, n_u, n_31 = [], set(), 0, 0
    for b in blocks:
        es = edges_of(b)
        k = kind(es)
        if not k:
            continue
        vid = re.search(r'id="([^"]+)"', b).group(1)
        vtype = re.search(r'type="([^"]+)"', b).group(1)
        depart = re.search(r'depart="([^"]+)"', b).group(1)
        drop.add(vid)
        common = f'type="{vtype}" depart="{depart}" departLane="best" departSpeed="desired"'
        if k == "U":
            n_u += 1
            trips.append((float(depart), f'<trip id="{vid}_A" {common} from="{es[0]}" to="{rng.choice(pool)}"/>'))
            trips.append((float(depart), f'<trip id="{vid}_B" {common} from="{rng.choice(starts)}" '
                                         f'to="{es[-1]}" via="{HOYT_AT_31ST}"/>'))
        else:
            n_31 += 1
            trips.append((float(depart), f'<trip id="{vid}_W" {common} from="{rng.choice(starts)}" '
                                         f'to="{es[-1]}" via="{HOYT_AT_31ST}"/>'))
    trips.sort()
    trip_file, routed = work / "resourced.trips.xml", work / "resourced.rou.xml"
    trip_file.write_text("<routes>\n" + "\n".join(vtypes) + "\n" + "\n".join(t for _, t in trips) + "\n</routes>\n")
    duarouter(sumo, ["--route-files", str(trip_file), "-o", str(routed)])
    keep = [b for b in blocks if re.search(r'id="([^"]+)"', b).group(1) not in drop]
    new = vehicles(routed.read_text())
    allv = sorted(keep + new, key=lambda b: float(re.search(r'depart="([\d.]+)"', b).group(1)))
    print(f"  U-turn trips split: {n_u}; 31st St on-ramp trips re-sourced: {n_31}; "
          f"new trips routed: {len(new)} of {len(trips)}")
    return "<routes>\n" + "\n".join("    " + v for v in vtypes) + "\n" + "\n".join(allv) + "\n</routes>\n"


def deloop(path):
    """Step 4: replace looping / >30% detour routes by the direct path."""
    import sumolib
    net = sumolib.net.readNet(str(NET))
    tree = ET.parse(path)
    cache, n, extra = {}, 0, 0.0
    for v in tree.getroot().iter("vehicle"):
        ro = v.find("route")
        es = ro.get("edges").split()
        key = (es[0], es[-1])
        if key not in cache:
            cache[key] = net.getShortestPath(net.getEdge(es[0]), net.getEdge(es[-1]), vClass="passenger")
        best, cost = cache[key]
        if not best:
            continue
        nodes = [net.getEdge(e).getToNode().getID() for e in es]
        length = sum(net.getEdge(e).getLength() for e in es)
        if len(nodes) != len(set(nodes)) or length > 1.3 * cost:
            n += 1
            extra += length - cost
            ro.set("edges", " ".join(e.getID() for e in best))
    tree.write(path, encoding="UTF-8", xml_declaration=True)
    print(f"  looping routes replaced: {n} (average {extra / max(n, 1):.0f} m shorter)")


def main():
    _, sumo = configure()
    with tempfile.TemporaryDirectory(prefix="baseline-v2-demand-") as tmp:
        work = Path(tmp)
        print("1/4 re-routing the evening demand on baseline-v2.net.xml")
        repaired = work / "repaired.rou.xml"
        duarouter(sumo, ["--route-files", str(SOURCE), "--repair", "true", "-o", str(repaired)])
        print("2/4 driver impatience setting")
        text = re.sub(r"(<vType [^>]*?)\s*/>", lambda m: m.group(1) + IMPATIENCE + "/>", repaired.read_text())
        print("3/4 U-turn and on-ramp source fix")
        OUT.write_text(resource(text, sumo, work))
    print("4/4 removing looping routes")
    deloop(OUT)
    print("wrote", OUT, "with", len(vehicles(OUT.read_text())), "vehicles")


if __name__ == "__main__":
    main()
