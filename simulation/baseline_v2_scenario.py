"""The baseline-v2 working scenario as a controller-ready SUMO launch.

baseline-v2 (see baseline-v2/README.md) is the network and demand where the
off-ramp flows all evening under fixed time. Its Webster signal timing lives in
an additional file; the signal-safety layer (adaptive_controller.Topology) reads
phase durations from the network file. ``merged_network`` writes a copy of the
network whose signal programmes are exactly the additional-file programmes, so
SUMO and the controller see one consistent fixed-time reference. Nothing in
baseline-v2/ is modified.
"""
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
FOLDER = HERE / "baseline-v2"
NETWORK = FOLDER / "baseline-v2.net.xml"
ROUTES = FOLDER / "baseline-v2.rou.xml"
SIGNALS = FOLDER / "baseline-v2-signals.add.xml"
BEGIN, DEMAND_END, END = 56700, 64800, 66600  # 3:45 PM, 6:00 PM, 6:30 PM
OFF_RAMP_EXIT = "993553211"  # cars entering Astoria Blvd N from the EB off-ramp (check_run.py)
SCENARIO = "baseline-v2"


def merged_network(directory):
    """Copy the network with the additional-file signal programmes embedded."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    target = directory / "baseline-v2-merged.net.xml"
    tree = ET.parse(NETWORK)
    root = tree.getroot()
    timed = {logic.get("id"): logic for logic in ET.parse(SIGNALS).getroot().findall("tlLogic")}
    for logic in root.findall("tlLogic"):
        replacement = timed.get(logic.get("id"))
        if replacement is None:
            continue
        old = [phase.get("state") for phase in logic.findall("phase")]
        new = [phase.get("state") for phase in replacement.findall("phase")]
        if len(old[0]) != len(new[0]):
            raise ValueError("Signal timing file does not match the network at " + logic.get("id"))
        for phase in list(logic):
            logic.remove(phase)
        for phase in replacement.findall("phase"):
            logic.append(phase)
        logic.set("offset", replacement.get("offset", "0"))
    tree.write(target, encoding="utf-8", xml_declaration=True)
    return target


def sumo_args(network, seed, begin=BEGIN, routes=ROUTES):
    """check_run.py's settings, plus junction collision checks that only report."""
    return ["-n", str(network), "-r", str(routes), "-b", str(begin), "--step-length", "0.2",
            "--lateral-resolution", "0.4", "--time-to-impatience", "30", "--time-to-teleport", "-1",
            "--seed", str(seed), "--no-step-log", "true", "--no-warnings", "true",
            "--collision.check-junctions", "true", "--collision.action", "warn"]


def departures(routes=ROUTES, begin=BEGIN):
    """Scheduled departure times. Evaluator-only: never passed to a controller."""
    times = (float(vehicle.get("depart")) for vehicle in ET.parse(routes).getroot().findall("vehicle"))
    return sorted(time for time in times if time >= begin)


def entry_edges(routes=ROUTES):
    """Edges where vehicles enter. Used for the training target only."""
    return sorted({vehicle.find("route").get("edges").split()[0] for vehicle in ET.parse(routes).getroot().findall("vehicle")})


def input_hashes():
    return {path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in (NETWORK, ROUTES, SIGNALS)}
