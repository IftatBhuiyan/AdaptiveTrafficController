"""Explicit sensitivity assumptions, not a calibrated reconstruction.

SUMO SL2015 handles real lateral motion and gap acceptance. No visual jitter,
forced teleports, collision disabling, or fabricated lane counts are used.
"""
from collections import Counter
import copy
import hashlib
import random
import xml.etree.ElementTree as ET

MODES = {"baseline", "merge", "disorder", "surge", "incident"}
MAINLINE = ("374433288", "1134690192", "122625709")


def _driver_rng(seed, key):
    return random.Random(int.from_bytes(hashlib.sha256(f"{seed}\0{key}".encode()).digest(), "big"))


def _legal_route(net, edges, vclass):
    return bool(edges) and all(net.hasEdge(e) and net.getEdge(e).allows(vclass) for e in edges) and all(
        net.getEdge(b) in net.getEdge(a).getAllowedOutgoing(vclass) for a, b in zip(edges, edges[1:]))


def _driver_route_choices(net, original, catalogue, protected, vclass, seed):
    """At most three legal alternatives, without changing observed crossings."""
    signature = tuple(e for e in original if e in protected)
    length = lambda edges: sum(net.getEdge(e).getLength() for e in edges)
    limit = 1.25 * length(original)
    choices = [original]

    def consider(edges):
        edges = tuple(edges)
        if (edges not in choices and edges[0] == original[0] and edges[-1] == original[-1]
                and len(set(edges)) == len(edges) and length(edges) <= limit + 1e-6
                and tuple(e for e in edges if e in protected) == signature
                and _legal_route(net, edges, vclass)):
            choices.append(edges)

    rng = _driver_rng(seed, "routes:" + vclass + ":" + " ".join(original))
    candidates = sorted(catalogue.get((original[0], original[-1], signature), set()))
    rng.shuffle(candidates)
    for edges in candidates:
        if _legal_route(net, edges, vclass):
            consider(edges)
        if len(choices) == 4:
            return choices
    branches = [(index, e.getID()) for index, eid in enumerate(original[:-1])
                for e in sorted(net.getEdge(eid).getAllowedOutgoing(vclass), key=lambda edge: edge.getID())
                if e.getID() != original[index + 1] and e.allows(vclass)]
    rng.shuffle(branches)
    for index, alternate in branches[:12]:
        anchor = next((i for i in range(index + 1, len(original)) if original[i] in protected), len(original) - 1)
        fixed = original[:index + 1] + original[anchor + 1:]
        budget = limit - length(fixed)
        if budget <= 0:
            continue
        path, _ = net.getShortestPath(net.getEdge(alternate), net.getEdge(original[anchor]),
                                      vClass=vclass, maxCost=budget)
        if path:
            consider(original[:index + 1] + tuple(e.getID() for e in path) + original[anchor + 1:])
        if len(choices) == 4:
            break
    return choices


def apply_driver_variation(root, network_path, plan, seed, fixed_routes=()):
    """Opt-in seeded driver/route sensitivity; call after apply_behavior.

    This only prepares simulator inputs. It changes neither demand schedules
    nor collision/teleport enforcement and is never an input to a controller.
    Existing baseline behavior is unchanged unless this function is called.
    """
    from sumo_runtime import configure
    configure()
    import sumolib
    net = sumolib.net.readNet(str(network_path))
    input_hash = hashlib.sha256(ET.tostring(root)).hexdigest()
    types = {kind.get("id"): kind for kind in root.findall("vType")}
    route_elements = {route.get("id"): route for route in root.findall("route")}
    routes = {rid: tuple(route.get("edges", "").split()) for rid, route in route_elements.items()}
    protected = {e for edges in plan.get("station_edges", {}).values() for e in edges} | {"25345883#2"}
    protected.update(e for edges in plan.get("cohort_edges", {}).values() for e in edges)
    catalogue = {}
    for edges in routes.values():
        if edges:
            catalogue.setdefault((edges[0], edges[-1], tuple(e for e in edges if e in protected)), set()).add(edges)
    pools, parameters = {}, {}
    for tid, base in sorted(types.items()):
        vclass = base.get("vClass", "passenger")
        if vclass not in ("passenger", "truck"):
            continue
        rng = _driver_rng(seed, "types:" + tid)
        width = float(base.get("width", "2.4" if vclass == "truck" else "1.8"))
        cooperation = max(0, min(1, float(base.get("lcCooperative", "1"))))
        lane_width = min((lane.getWidth() for edge in net.getEdges() for lane in edge.getLanes()
                          if lane.allows(vclass)), default=width)
        offset = max(0, min(.1, (lane_width - width) / 2 - .1))
        pools[tid] = []
        for index in range(16):
            clone = copy.deepcopy(base)
            clone_id = f"driver_{tid}_{index}"
            if clone_id in types:
                raise ValueError(f"Driver profile type already exists: {clone_id}")
            accel = float(base.get("accel", "1.3" if vclass == "truck" else "2.6")) * rng.uniform(.85, 1.05)
            decel = float(base.get("decel", "4" if vclass == "truck" else "4.5")) * rng.uniform(.9, 1)
            tau = max(1.1, float(base.get("tau", "1"))) * rng.uniform(1, 1.5)
            clone.attrib.update(id=clone_id, speedFactor=f"{rng.uniform(.84, 1):.4f}", speedDev="0",
                                accel=f"{accel:.4f}", decel=f"{decel:.4f}", apparentDecel=f"{decel:.4f}",
                                tau=f"{tau:.4f}", minGap=f"{max(2.5, float(base.get('minGap', '2.5'))) * rng.uniform(1, 1.2):.4f}",
                                laneChangeModel="SL2015", latAlignment=f"{rng.uniform(-offset, offset):.4f}",
                                minGapLat="0.25", maxSpeedLat="0.6",
                                lcCooperative=f"{rng.uniform(.7 * cooperation, min(1, 1.3 * cooperation)):.4f}",
                                lcAssertive=f"{rng.uniform(.75, 1):.4f}", lcStrategic=f"{rng.uniform(.8, 1.5):.4f}",
                                lcStrategicLookahead=f"{rng.uniform(1500, 3500):.2f}", lcPushy="0",
                                lcImpatience="0", lcSigma="0", sigma="0", impatience="off",
                                jmSigmaMinor="0", jmTimegapMinor="2", jmExtraGap="1", jmAdvance="0")
            pools[tid].append(clone)
            parameters[clone_id] = {**clone.attrib, "base_type": tid}
            root.append(clone)
    choices_cache, route_ids = {}, {edges: rid for rid, edges in sorted(routes.items())}
    varied, eligible, rerouted, fixed_stops, fixed_edits, flows = 0, 0, 0, 0, 0, 0
    fixed_routes = set(fixed_routes)
    class_counts, assignments, route_assignments = Counter(), [], []
    for entity in [item for item in root if item.tag in ("vehicle", "flow")]:
        tid = entity.get("type", "DEFAULT_VEHTYPE")
        if tid not in pools:
            continue
        rng = _driver_rng(seed, "vehicle:" + entity.get("id"))
        clone = rng.choice(pools[tid])
        entity.set("type", clone.get("id"))
        entity.set("departPosLat", "center")
        if "speedFactor" in entity.attrib:
            entity.set("speedFactor", clone.get("speedFactor"))
        varied += entity.tag == "vehicle"
        flows += entity.tag == "flow"
        vclass = types[tid].get("vClass", "passenger")
        class_counts[vclass] += 1
        rid = entity.get("route")
        original = routes.get(rid)
        route = route_elements.get(rid)
        if not original or not _legal_route(net, original, vclass):
            raise ValueError(f"Driver variation requires a legal named route for {entity.get('id')}")
        if rid in fixed_routes:
            # Explicit map-editor paths express user intent, including via
            # waypoints that are not part of a measured-station signature.
            choices = [original]
            fixed_edits += 1
        elif entity.find("stop") is not None or (route is not None and route.find("stop") is not None):
            choices = [original]
            fixed_stops += 1
        else:
            key = (original, vclass)
            if key not in choices_cache:
                choices_cache[key] = _driver_route_choices(net, original, catalogue, protected, vclass, seed)
            choices = choices_cache[key]
        selected = rng.choice(choices)
        eligible += len(choices) > 1 and entity.tag == "vehicle"
        if selected != original:
            if selected not in route_ids:
                new_id = "driver_route_" + hashlib.sha256(" ".join(selected).encode()).hexdigest()[:16]
                if new_id in routes:
                    raise ValueError(f"Driver route id already exists: {new_id}")
                root.append(ET.Element("route", id=new_id, edges=" ".join(selected)))
                route_ids[selected] = new_id
            entity.set("route", route_ids[selected])
            rerouted += entity.tag == "vehicle"
        assignments.append(f"{entity.get('id')}:{entity.get('type')}:{entity.get('route')}")
        route_assignments.append(f"{entity.get('id')}:{entity.get('route')}")
    # SUMO requires definitions before references. Keep traffic order intact.
    root[:] = [item for item in root if item.tag in ("vType", "route")] + [
        item for item in root if item.tag not in ("vType", "route")]
    return {"profile": "varied", "seed": seed, "input_sha256": input_hash,
            "vehicles_varied": varied, "flow_groups_varied": flows, "class_entities": dict(class_counts),
            "parameters": parameters, "profiles_per_base_type": 16,
            "base_parameters": {tid: dict(types[tid].attrib) for tid in pools},
            "randomization": {"speed_factor_range": [.84, 1], "speed_deviation": 0,
                              "acceleration_factor_range": [.85, 1.05], "deceleration_factor_range": [.9, 1],
                              "tau_factor_range": [1, 1.5], "minimum_tau_s": 1.1,
                              "cooperation_factor_range": [.7, 1.3], "cooperation_clamp": [0, 1],
                              "assertiveness_range": [.75, 1], "maximum_preferred_lateral_offset_m": .1,
                              "speed_gain_and_keep_right": "Inherited unchanged from each scenario's base type"},
            "assignment_sha256": hashlib.sha256("\n".join(assignments).encode()).hexdigest(),
            "route_assignment_sha256": hashlib.sha256("\n".join(route_assignments).encode()).hexdigest(),
            "routes": {"strategy": "legal catalogue and bounded static branch detours", "max_alternatives": 3,
                       "branch_probes": 12, "max_length_ratio": 1.25, "protected_edges": sorted(protected),
                       "eligible_vehicles": eligible, "rerouted_vehicles": rerouted,
                       "retained_vehicles": varied - rerouted, "coverage": rerouted / varied if varied else 0,
                       "fixed_stop_entities": fixed_stops, "fixed_editor_route_entities": fixed_edits,
                       "generated_routes": len(route_ids) - len(set(routes.values()))},
            "assumptions": "Seeded sensitivity, not calibrated driver behavior. Desired speeds never exceed posted limits; "
                           "preferred lateral offset <=0.10 m with zero noise. Native safety checks retained. "
                           "Departures, counts, OD and ordered station/cohort crossings preserved. "
                           "Flow members share their flow group's profile. Live collision acceptance remains required."}


def apply_behavior(root, mode, begin=56700, end=64800, seed=42):
    if mode not in MODES:
        raise ValueError("Unknown traffic scenario")
    # Deterministic longitudinal following isolates the controller experiment.
    # Default acceleration noise produced a real SL2015 side collision at a
    # short lane-addition connector. Off-centre side-by-side traffic produced
    # body overlaps on curved fan-outs. Keep native continuous lane changes,
    # but centred alignment and zero noise are declared scenario assumptions.
    for kind in root.findall("vType"):
        kind.attrib.update(laneChangeModel="SL2015", latAlignment="center", maxSpeedLat="0.8",
                           minGapLat="0.25", lcCooperative="0.8", lcAssertive="1.0", lcImpatience="0", lcSigma="0", sigma="0")
    rng = random.Random(seed)
    if mode == "baseline":
        for kind in root.findall("vType"):
            kind.attrib.update(impatience="off", jmSigmaMinor="0", jmTimegapMinor="2", jmExtraGap="1", jmAdvance="0")
        return 0
    # Mixed cooperation and lateral response make the merge less orderly. Gap
    # acceptance remains neutral; the 35% share is a scenario assumption.
    ET.SubElement(root, "vType", id="merge_driver", vClass="passenger", laneChangeModel="SL2015",
                  latAlignment="center", maxSpeedLat="0.6", minGapLat="0.2", minGap="2.5",
                  lcPushy="0", lcAssertive="1.0", lcCooperative="0.1" if mode == "disorder" else "0.3", lcSpeedGain="1.5", lcImpatience="0", lcSigma="0")
    ET.SubElement(root, "vType", id="mainline", vClass="passenger", laneChangeModel="SL2015",
                  latAlignment="center", maxSpeedLat="0.7", minGapLat="0.25",
                  lcCooperative="0.15", lcKeepRight="0", lcSpeedGain="0", lcAssertive="1", lcImpatience="0", lcSigma="0")
    paths = {r.get("id"): r.get("edges").split() for r in root.findall("route")}
    mainline_counts = Counter()
    for vehicle in root.findall("vehicle"):
        edges = paths[vehicle.get("route")]
        if "374433288" in edges:
            mainline_counts[int(float(vehicle.get("depart"))) // 900] += 1
        if vehicle.get("type") == "car" and rng.random() < .35:
            vehicle.set("type", "merge_driver")
            # Lane changes still have native lateral motion. Random departures
            # and off-centre alignment failed physical collision checks here.
            vehicle.set("departPosLat", "center")
    # Tight imported connectors can overlap when a yielding driver advances
    # while another vehicle is still crossing. Conservative, shared junction
    # settings avoid that numerical failure without disabling collision checks.
    # These are experimental assumptions, not measured Astoria driver traits.
    for kind in root.findall("vType"):
        kind.attrib.update(impatience="off", jmSigmaMinor="0", jmTimegapMinor="2",
                           jmExtraGap="1", jmAdvance="0", sigma="0")
    # The downloaded map ends shortly after the merge; it does not include the
    # next exit. Represent constrained receiving traffic as an explicit demand
    # sensitivity, not a claim that the absent exit is simulated.
    ET.SubElement(root, "route", id="receiving_mainline", edges=" ".join(MAINLINE))
    added = 0
    for bucket in range(begin // 900, end // 900):
        per_lane_hour = 1750 if bucket * 900 >= 61200 else 1600
        count = max(0, round(per_lane_hour * 3 / 4) - mainline_counts[bucket])
        for index in range(count):
            depart = bucket * 900 + (index + .5) * 900 / count
            ET.SubElement(root, "vehicle", id=f"receiving_{bucket}_{index}", type="mainline",
                          route="receiving_mainline", depart=f"{depart:.2f}", departLane="random", departSpeed="desired")
        added += count
    return added


def receiving_constraint(directory, mode="merge", observation_time=57600):
    """Explicit downstream-boundary queue, standing in for the absent exit.

    Drivers retaining this lane encounter 5.5 m/s traffic at 4 pm and 4 m/s
    at 5 pm. These speeds are sensitivity settings, not measured speeds.
    Other mainline lanes retain their normal limits.
    """
    root = ET.Element("additional")
    sign = ET.SubElement(root, "variableSpeedSign", id="assumed_receiving_queue", lanes="122625709_0")
    ET.SubElement(sign, "step", time="0", speed="5.5")
    ET.SubElement(sign, "step", time="61200", speed="4")
    if mode == "incident":
        # A downstream virtual speed sign models a receiving-lane obstruction.
        # 0.1 m/s preserves normal braking/collision checks; no teleport or
        # surprise stopped vehicle is injected into a moving lane.
        ET.SubElement(sign, "step", time=str(observation_time + 300), speed="0.1")
        ET.SubElement(sign, "step", time=str(observation_time + 360), speed="5.5" if observation_time < 61200 else "4")
        sign[:] = sorted(sign, key=lambda step: float(step.get("time")))
    path = directory / "receiving-constraint.add.xml"
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    return path


def demand_surge(root, observation_time):
    """20% extra base traffic for one 10-minute burst; a labeled assumption."""
    vehicles = root.findall("vehicle")
    for index, vehicle in enumerate(vehicles):
        depart = float(vehicle.get("depart"))
        if observation_time + 120 <= depart < observation_time + 720 and index % 5 == 0:
            attrs = dict(vehicle.attrib)
            attrs.update(id="surge_" + vehicle.get("id"), depart=f"{depart + .1:.2f}")
            ET.SubElement(root, "vehicle", **attrs)
