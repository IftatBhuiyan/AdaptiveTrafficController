"""Causal, finite-storage predictive control over a signal neighbourhood.

The traffic model uses current detector observations and past transfers only.
It never reads future departures or vehicle routes. Protected green actions
are derived from SUMO connections/conflicts, with yellow and all-red transitions.
"""
from collections import defaultdict, deque
from dataclasses import dataclass
import math
import time
import xml.etree.ElementTree as ET

MODES = {"fixed", "local", "coordinated"}
CENTRES = ("astoria_31st_north", "astoria_31st_south", "astoria_33rd")


@dataclass(frozen=True)
class Settings:
    interval: float = 5.0
    horizon: float = 30.0
    minimum_green: float = 8.0
    maximum_green: float = 50.0
    maximum_service_gap: float = 90.0
    all_red: float = 2.0
    # Extra all-red allowed while a *moving* vehicle is still crossing a path
    # that conflicts with the next green. Stopped (box-blocking) vehicles never
    # extend it: an unbounded hold froze whole neighbourhoods in all-red.
    clearance_extension: float = 3.0
    storage_per_vehicle: float = 7.5
    saturation_flow: float = .45  # vehicles / second / incoming lane; learned online
    signal_hops: int = 5
    neighbourhood_metres: float = 650.0
    forecast_margin: float = 1.0
    predictive: bool = True
    # Intelligent mode: a green must run lost_time_amortization x (yellow +
    # all-red) before it is cut for its allocation or an overdue cohort
    # (Lammer-Helbing setup cost; SCOOT/SCATS longer greens when congested).
    lost_time_amortization: float = 0.0
    stall_seconds: float = 20.0  # selected on seed 42; validated on held-out 45/46
    imported_actions: bool = False
    entire_network: bool = False


@dataclass
class Lane:
    id: str
    edge: str
    length: float
    speed: float
    index: int


@dataclass(frozen=True)
class Movement:
    incoming: str
    outgoing: str
    index: int
    direction: str


@dataclass
class Signal:
    id: str
    junctions: tuple
    greens: tuple
    movements: tuple
    yellow: float
    stage: str = "native"
    current: int = 0
    target: int = 0
    since: float = 0.0
    until: float = 0.0
    switches: int = 0
    reason: str = "Waiting for the existing clearance phase"
    predicted_service: float = 0.0
    scores: tuple = ()
    green_durations: tuple = ()
    offset: float = 0.0
    masked: frozenset = frozenset()


def select_scope(graph, positions, centres, hops=5, radius=650):
    """Union of five-signal road distance and roughly five urban blocks."""
    chosen = {c for c in centres if c in graph}
    distances = {c: 0 for c in chosen}
    queue = deque(chosen)
    while queue:
        node = queue.popleft()
        if distances[node] >= hops:
            continue
        for other in graph[node]:
            if other not in distances:
                distances[other] = distances[node] + 1
                chosen.add(other)
                queue.append(other)
    origins = [positions[c] for c in centres if c in positions]
    chosen.update(k for k, p in positions.items() if any(math.dist(p, o) <= radius for o in origins))
    return chosen, distances


def yellow_state(green):
    return "".join("y" if c in "gG" else "r" for c in green)


def usable_supply(stock, capacity, entry_gap, speed, discharge, horizon, margin=1):
    """An entrance blocked by stopped traffic has no immediate receiving space."""
    free = max(0.0, capacity - stock - margin)
    if entry_gap < 6 and speed < .5:
        return 0.0
    return max(0.0, free + discharge * horizon)


def protected_actions(original, movements, lanes, conflicts):
    """Conflict-checked simulation actions; no permissive crossing greens.

    One connector per incoming-lane/outgoing-edge path in each action avoids
    simultaneous fan-out trajectories. Every connector retains a green action.
    """
    groups = defaultdict(list)
    for m in movements:
        groups[m.incoming, lanes[m.outgoing].edge].append(m)
    selected = []
    for values in groups.values():
        preferred = [m for m in values if any(g[m.index] == "G" for g in original)] or values
        selected.append(min(preferred, key=lambda m: (abs(lanes[m.incoming].index - lanes[m.outgoing].index), m.index)))
    def incompatible(a, b):
        # Shared-lane alternatives within one approach must remain available,
        # otherwise a red turning car blocks a green through queue behind it.
        # The duplicate fan-out for one road path was removed above.
        if a.incoming == b.incoming:
            return False
        return a.outgoing == b.outgoing or conflicts(a.index, b.index)
    states, covered = [], set()
    def add(candidates):
        batches = []
        for m in candidates:
            group = next((g for g in batches if all(not incompatible(m, other) for other in g)), None)
            if group is None:
                group = []
                batches.append(group)
            group.append(m)
        for group in batches:
            state = ["r"] * len(original[0])
            for m in group:
                state[m.index] = "G"
                covered.add(m.index)
            value = "".join(state)
            if value not in states:
                states.append(value)
    for green in original:
        add([m for m in selected if green[m.index] == "G"])
    missing_approaches = {lanes[m.incoming].edge for m in selected if m.index not in covered}
    by_approach = defaultdict(list)
    for m in selected:
        if lanes[m.incoming].edge in missing_approaches:
            by_approach[lanes[m.incoming].edge].append(m)
    for group in by_approach.values():
        add(group)
    by_lane = defaultdict(list)
    for m in selected:
        by_lane[m.incoming].append(m)
    for group in by_lane.values():
        if not any(all(s[m.index] == "G" for m in group) for s in states):
            add(group)
    chosen = {m.index for m in selected}
    for alternate in movements:
        if alternate.index in chosen:
            continue
        # Redundant road paths still carry lane preferences. A route probe
        # confirmed SUMO can wait forever on its preferred red connector even
        # when another connector to the same road is green. Preserve each in a
        # separate protected action, never as simultaneous fan-out trajectories.
        lane_group = [m for m in selected if m.incoming == alternate.incoming and lanes[m.outgoing].edge != lanes[alternate.outgoing].edge]
        add(lane_group + [alternate])
    return tuple(states), tuple(movements)


def head_prefix(queue, probability):
    """Expected consecutive departures before the first blocked turning car."""
    p = min(1, max(0, probability))
    if p >= 1 - 1e-9:
        return max(0, queue)
    return p * (1 - p ** max(0, queue)) / max(1e-9, 1 - p)


def partition_approaches(paths, lanes):
    """Observe the whole upstream corridor, allocating shared lanes once."""
    options = defaultdict(list)
    for end, path in paths.items():
        for i, lane in enumerate(path):
            distance = sum(lanes[p].length for p in path[i + 1:])
            options[lane].append((abs(lanes[end].index - lanes[lane].index), distance, end))
    owners = {lane: min(choices)[2] for lane, choices in options.items()}
    return {end: tuple(l for l in path if owners[l] == end) for end, path in paths.items()}


def connector_exits(root, external_lanes):
    """Current connector geometry, not a vehicle's hidden future route.

    Split internal connectors can precede a road too short to hold a car. Map
    each to its unambiguous next road and the intervening connector distance.
    """
    lengths = {l.get("id"): float(l.get("length")) for e in root.findall("edge")
               if e.get("function") == "internal" for l in e.findall("lane")}
    links = defaultdict(set)
    for c in root.findall("connection"):
        source = c.get("from") + "_" + c.get("fromLane")
        destination = c.get("via") or c.get("to") + "_" + c.get("toLane")
        if source in lengths:
            links[source].add(destination)
    result = {}
    for source in lengths:
        queue, seen, exits = deque([(source, 0.0)]), set(), {}
        while queue:
            current, distance = queue.popleft()
            if current in seen:
                continue
            seen.add(current)
            for target in links[current]:
                if target in external_lanes:
                    exits[target] = min(exits.get(target, math.inf), distance)
                elif target in lengths:
                    queue.append((target, distance + lengths[target]))
        if len(exits) == 1:
            result[source] = next(iter(exits.items()))
    return lengths, result


class Topology:
    def __init__(self, path, settings=Settings()):
        self.settings = settings
        root = ET.parse(path).getroot()
        self.lanes, self.edges, self.successors, self.predecessors = {}, {}, defaultdict(list), defaultdict(list)
        self.node_positions = {j.get("id"): (float(j.get("x")), float(j.get("y"))) for j in root.findall("junction")}
        self.node_types = {j.get("id"): j.get("type") for j in root.findall("junction")}
        for edge in root.findall("edge"):
            if edge.get("function") == "internal":
                continue
            self.edges[edge.get("id")] = (edge.get("from"), edge.get("to"))
            for lane in edge.findall("lane"):
                allow, deny = lane.get("allow", "").split(), lane.get("disallow", "").split()
                if (allow and "passenger" not in allow) or "all" in deny or "passenger" in deny:
                    continue
                self.lanes[lane.get("id")] = Lane(lane.get("id"), edge.get("id"), float(lane.get("length")),
                                                 float(lane.get("speed")), int(lane.get("index")))
        movement_lists, node_tls = defaultdict(list), {}
        for c in root.findall("connection"):
            a, b = c.get("from") + "_" + c.get("fromLane"), c.get("to") + "_" + c.get("toLane")
            if a not in self.lanes or b not in self.lanes:
                continue
            direction = c.get("dir", "s")
            if direction not in ("t", "T"):
                self.successors[a].append((b, direction))
                self.predecessors[b].append((a, direction))
            if c.get("tl"):
                movement_lists[c.get("tl")].append(Movement(a, b, int(c.get("linkIndex")), direction))
                node_tls[self.edges[c.get("from")][1]] = c.get("tl")
        self.signals = {}
        self.network_movements = {key: tuple(value) for key, value in movement_lists.items()}
        self.program_phases = {program.get("id"): tuple((float(phase.get("duration")), phase.get("state"))
                                                       for phase in program.findall("phase")) for program in root.findall("tlLogic")}
        for program in root.findall("tlLogic"):
            tid = program.get("id")
            if tid in self.signals or not movement_lists[tid]:
                continue
            phases = program.findall("phase")
            greens = tuple(dict.fromkeys(p.get("state") for p in phases if "y" not in p.get("state").lower() and "g" in p.get("state").lower()))
            if len(greens) < 2:
                continue
            yellow = max([float(p.get("duration")) for p in phases if "y" in p.get("state").lower()] + [3.0])
            junctions = tuple(n for n, t in node_tls.items() if t == tid)
            durations = tuple(max(settings.minimum_green, next(float(p.get("duration")) for p in phases if p.get("state") == g)) for g in greens)
            self.signals[tid] = Signal(tid, junctions, greens, tuple(movement_lists[tid]), yellow, green_durations=durations, offset=float(program.get("offset", "0")))
        self.node_tls = node_tls
        import sumolib
        model = sumolib.net.readNet(str(path), withFoes=True)
        connection_info = defaultdict(dict)
        for edge in model.getEdges():
            for lane in edge.getLanes():
                for c in lane.getOutgoing():
                    if c.getTLSID():
                        connection_info[c.getTLSID()][c.getTLLinkIndex()] = (edge.getToNode(), c.getJunctionIndex())
        self.conflicts = {}
        for tid, signal in self.signals.items():
            info = connection_info[tid]
            def conflicts(a, b, info=info):
                if a not in info or b not in info:
                    return True
                na, ia = info[a]
                nb, ib = info[b]
                if na.getID() != nb.getID():
                    return False
                if ia < 0 or ib < 0:
                    return True
                return na.areFoes(ia, ib) or na.areFoes(ib, ia)
            self.conflicts[tid] = conflicts
            original, durations = signal.greens, signal.green_durations
            if not settings.imported_actions:
                signal.greens, signal.movements = protected_actions(original, signal.movements, self.lanes, conflicts)
                signal.green_durations = tuple(next((d for g, d in zip(original, durations) if all(c != "G" or g[i] == "G" for i, c in enumerate(s))), 12.0) for s in signal.greens)
        self.owner = {m.incoming: tid for tid, signal in self.signals.items() for m in signal.movements}
        # First reachable signals on directed roads form the communication graph.
        self.graph = {tid: set() for tid in self.signals}
        for tid, signal in self.signals.items():
            queue, seen = deque(m.outgoing for m in signal.movements), set()
            while queue:
                lane = queue.popleft()
                if lane in seen:
                    continue
                seen.add(lane)
                nxt = self.owner.get(lane)
                if nxt and nxt != tid:
                    self.graph[tid].add(nxt)
                    self.graph[nxt].add(tid)
                    continue
                queue.extend(l for l, _ in self.successors[lane])
        positions = {tid: tuple(sum(self.node_positions[j][i] for j in s.junctions) / len(s.junctions) for i in range(2)) for tid, s in self.signals.items()}
        self.scope, self.hops = select_scope(self.graph, positions, CENTRES, settings.signal_hops, settings.neighbourhood_metres)
        if settings.entire_network:
            self.scope = set(self.graph)
        self.signals = {tid: s for tid, s in self.signals.items() if tid in self.scope}
        internal = [lane.get("id") for edge in root.findall("edge") if edge.get("function") == "internal" for lane in edge.findall("lane")]
        self.internal_lanes = {tid: tuple(l for l in internal if any(l.startswith(":" + j + "_") for j in s.junctions)) for tid, s in self.signals.items()}
        self.incoming = sorted({m.incoming for s in self.signals.values() for m in s.movements})
        self.approaches = partition_approaches({lid: self.walk(lid, reverse=True) for lid in self.incoming}, self.lanes)
        self.receivers = {m.outgoing: self.walk(m.outgoing) for s in self.signals.values() for m in s.movements}
        self.forced_merges = {}
        for path in self.receivers.values():
            end = path[-1]
            meta = self.lanes[end]
            if not self.successors[end]:
                adjacent = [l for l in self.lanes.values() if l.edge == meta.edge and self.successors[l.id]]
                if adjacent:
                    self.forced_merges[end] = self.walk(min(adjacent, key=lambda l: abs(l.index - meta.index)).id)
        self.detectors = sorted(set(l for path in (*self.approaches.values(), *self.receivers.values(), *self.forced_merges.values()) for l in path))
        if settings.entire_network:
            self.detectors = sorted(self.lanes)
        self.cell_owner = {lane: end for end, path in self.approaches.items() for lane in path}
        self.internal_lengths, self.connector_exits = connector_exits(root, self.lanes)
        internal_links = defaultdict(set)
        for c in root.findall("connection"):
            a = c.get("from") + "_" + c.get("fromLane")
            b = c.get("via") or c.get("to") + "_" + c.get("toLane")
            if a in self.internal_lengths and b in self.internal_lengths:
                internal_links[a].add(b)
        self.connector_movements = {}
        # Internal lane -> signal link indices whose trajectory it carries, so
        # clearance can wait only for vehicles on paths that conflict with the
        # next green rather than for any vehicle anywhere in the box.
        self.internal_link_index = defaultdict(set)
        for c in root.findall("connection"):
            if c.get("tl") not in self.signals or not c.get("via"):
                continue
            movement = (c.get("from") + "_" + c.get("fromLane"), c.get("to") + "_" + c.get("toLane"))
            queue, seen = deque([c.get("via")]), set()
            while queue:
                lane = queue.popleft()
                if lane in seen:
                    continue
                seen.add(lane)
                self.connector_movements[lane] = movement
                self.internal_link_index[lane].add(int(c.get("linkIndex")))
                queue.extend(internal_links[lane])
        self.capacities = {end: max(1, sum(self.lanes[l].length for l in path) / settings.storage_per_vehicle) for end, path in self.approaches.items()}
        self.movement_keys = {(m.incoming, m.outgoing) for s in self.signals.values() for m in s.movements}
        self.turn_options = defaultdict(dict)
        for s in self.signals.values():
            for m in s.movements:
                self.turn_options[m.incoming][m.outgoing] = m.direction
        self.receiving_groups = defaultdict(set)
        for path in self.receivers.values():
            self.receiving_groups[path[-1]].update(path)

    def walk(self, lane, reverse=False):
        path, distance = [lane], self.lanes[lane].length
        links = self.predecessors if reverse else self.successors
        # The ramp's lane drop is 388 m from its entrance. Cutting receiving
        # observations at 250 m missed the actual merge and learned admissions
        # to it rather than completed merges. Use the neighbourhood distance
        # downstream; upstream ETA observations remain bounded to 250 m.
        limit = 250 if reverse else self.settings.neighbourhood_metres
        while distance < limit:
            current = path[-1]
            boundary = self.edges[self.lanes[current].edge][0 if reverse else 1]
            if boundary in self.node_tls:
                break
            candidates = [(l, d) for l, d in links[current] if l not in path]
            straight = [(l, d) for l, d in candidates if d == "s"]
            candidates = straight or candidates
            targets = {self.lanes[l].edge for l, _ in candidates}
            if len(targets) != 1:
                break
            nxt = min(candidates, key=lambda pair: abs(self.lanes[pair[0]].index - self.lanes[current].index))[0]
            path.append(nxt)
            distance += self.lanes[nxt].length
        return tuple(reversed(path)) if reverse else tuple(path)

    def metadata(self):
        return {"signals": sorted(self.signals), "junctions": sorted(j for s in self.signals.values() for j in s.junctions),
                "signal_hops": self.settings.signal_hops, "radius_metres": self.settings.neighbourhood_metres,
                "entire_network": self.settings.entire_network,
                "neighbours": {t: sorted(self.graph[t] & self.scope) for t in self.signals},
                "detector_lanes": len(self.detectors), "horizon_seconds": self.settings.horizon,
                "active_links": {tid: sorted({m.index for m in s.movements}) for tid, s in self.signals.items()},
                "phase_count": sum(len(s.greens) for s in self.signals.values()),
                "reference": "Imported SUMO greens with yielding turns and common custom clearance" if self.settings.imported_actions else "Protected, conflict-checked simulation phases; imported green durations plus 12 s auxiliary protected turns"}


class AdaptiveController:
    def __init__(self, api, network, mode="coordinated", settings=Settings()):
        if mode not in MODES:
            raise ValueError("Unknown signal controller")
        self.api, self.mode, self.settings = api, mode, settings
        self.topology = Topology(network, settings)
        self.signals = self.topology.signals
        self.last_tick = -math.inf
        self.last_external, self.turn_counts, self.last_service = {}, defaultdict(float), {}
        self.last_crossing = {}
        self.arrival_rate, self.discharge_rate = defaultdict(float), defaultdict(lambda: settings.saturation_flow)
        self.exit_rate = defaultdict(lambda: .25)
        self.exit_samples, self.exit_since_tick = defaultdict(int), defaultdict(int)
        self.previous_count, self.predictions = {}, deque()
        self.forecast_errors, self.predicted_arrivals = [], 0
        self.elapsed_ms = 0.0
        self.decisions, self.messages, self.blocked = 0, 0, set()
        self.rows, self.forecasts, self.plans = {}, {}, {}
        self.queued, self.neighbour_arrivals = {}, {}
        self.reservations = {}
        self.release_delays, self.raw_lanes = {}, {}
        self.phase_service, self.learning_eligible = {}, {}
        self.ready_since = {}
        self.flow_since_tick = defaultdict(int)
        self.constants = __import__("traci.constants", fromlist=["constants"])
        c = self.constants
        # Install the common protected reference before the first vehicle is
        # inserted. Runtime policy changes restart the scenario; no occupied
        # intersection is reprogrammed by this constructor.
        if api.vehicle.getIDCount():
            raise ValueError("Create the controller before advancing SUMO; occupied intersections cannot be reprogrammed instantly.")
        for signal in self.signals.values():
            phases = []
            for green, duration in zip(signal.greens, signal.green_durations):
                phases.extend((api.trafficlight.Phase(duration, green),
                               api.trafficlight.Phase(signal.yellow, yellow_state(green)),
                               api.trafficlight.Phase(settings.all_red, "r" * len(green))))
            program = "imported_reference" if settings.imported_actions else "protected_reference"
            logic = api.trafficlight.Logic(program, c.TRAFFICLIGHT_TYPE_STATIC, 0, tuple(phases))
            api.trafficlight.setProgramLogic(signal.id, logic)
            api.trafficlight.setProgram(signal.id, program)
            if signal.offset:
                position = signal.offset % sum(p.duration for p in phases)
                for index, phase in enumerate(phases):
                    if position < phase.duration:
                        api.trafficlight.setPhase(signal.id, index)
                        api.trafficlight.setPhaseDuration(signal.id, phase.duration - position)
                        break
                    position -= phase.duration
        for lane in self.topology.detectors:
            api.lane.subscribe(lane, (c.LAST_STEP_VEHICLE_NUMBER, c.LAST_STEP_MEAN_SPEED, c.LAST_STEP_VEHICLE_HALTING_NUMBER, c.LAST_STEP_VEHICLE_ID_LIST))
        for lane in {l for lanes in self.topology.internal_lanes.values() for l in lanes}:
            api.lane.subscribe(lane, (c.LAST_STEP_VEHICLE_NUMBER, c.LAST_STEP_VEHICLE_HALTING_NUMBER))

    def vehicle_observations(self):
        c = self.constants
        result = {}
        for vid, v in self.api.vehicle.getAllSubscriptionResults().items():
            if c.VAR_LANE_ID in v and c.VAR_LANEPOSITION in v:
                result[vid] = (v[c.VAR_LANE_ID], v[c.VAR_LANEPOSITION], v[c.VAR_SPEED], v.get(c.VAR_LENGTH, 5))
        return result

    def observe(self, now, vehicles):
        c = self.constants
        data = self.api.lane.getAllSubscriptionResults()
        self.raw_lanes = data
        self.rows = {}
        for lane in self.topology.detectors:
            r = data.get(lane, {})
            ids = r.get(c.LAST_STEP_VEHICLE_ID_LIST, ())
            positions = [vehicles[v][1] for v in ids if v in vehicles]
            stock = r.get(c.LAST_STEP_VEHICLE_NUMBER, 0)
            # Empty roads have no observed mean speed (-1 in SUMO), not zero
            # receiving capacity. Free-flow speed is the explicit model prior
            # until vehicles arrive; otherwise short empty entries look blocked.
            speed = max(0, r.get(c.LAST_STEP_MEAN_SPEED, 0)) if stock else self.topology.lanes[lane].speed
            self.rows[lane] = {"stock": stock,
                               "queue": r.get(c.LAST_STEP_VEHICLE_HALTING_NUMBER, 0),
                               "speed": speed,
                               "entry_gap": min(positions, default=self.topology.lanes[lane].length), "ids": ids}
        connector_cars = defaultdict(list)
        for vid, (lane, position, speed, length) in vehicles.items():
            following = self.topology.connector_exits.get(lane)
            if following is None:
                continue
            road, distance = following
            incoming = self.topology.cell_owner.get(road)
            if incoming is None:
                continue
            path = self.topology.approaches[incoming]
            remainder = sum(self.topology.lanes[l].length for l in path[path.index(road):])
            distance += max(0, self.topology.internal_lengths[lane] - position) + remainder
            connector_cars[incoming].append((vid, distance, speed))
        self.learning_eligible = {}
        for incoming in self.topology.incoming:
            signal = self.signals[self.topology.owner[incoming]]
            moves = [m for m in signal.movements if m.incoming == incoming]
            roads = {self.topology.lanes[m.outgoing].edge for m in moves}
            all_green = signal.stage == "green" and all(any(signal.greens[signal.current][m.index] == "G" for m in moves if self.topology.lanes[m.outgoing].edge == edge) for edge in roads)
            free = all(sum(self.rows[p]["stock"] for p in self.topology.receivers[m.outgoing]) * self.settings.storage_per_vehicle < .85 * sum(self.topology.lanes[p].length for p in self.topology.receivers[m.outgoing]) for m in moves)
            self.learning_eligible[incoming] = all_green and free
            path = self.topology.approaches[incoming]
            ready = (any(v in vehicles and vehicles[v][2] < .1
                         and self.topology.lanes[lane].length - vehicles[v][1]
                         + sum(self.topology.lanes[p].length for p in path[index + 1:]) < 12
                         for index, lane in enumerate(path) for v in self.rows[lane]["ids"])
                     or any(speed < .1 and distance < 12 for _, distance, speed in connector_cars[incoming]))
            if ready:
                self.ready_since.setdefault(incoming, now)
            else:
                self.ready_since.pop(incoming, None)
        transfers = self.flow_since_tick
        self.flow_since_tick = defaultdict(int)
        for end, reservoir in self.topology.receiving_groups.items():
            if self.exit_since_tick[end] or any(self.rows[l]["stock"] for l in reservoir):
                observed = self.exit_since_tick[end] / self.settings.interval
                self.exit_rate[end] = .7 * self.exit_rate[end] + .3 * min(.7, observed)
                self.exit_samples[end] += 1
        self.exit_since_tick = defaultdict(int)
        self.forecasts = {}
        steps = math.ceil(self.settings.horizon / self.settings.interval)
        for incoming, path in self.topology.approaches.items():
            q = sum(self.rows[l]["queue"] for l in path) + sum(speed < .1 for _, _, speed in connector_cars[incoming])
            self.queued[incoming] = q
            if q and incoming not in self.last_service:
                self.last_service[incoming] = now
            if not q:
                self.last_service[incoming] = now
            arrivals = [0.0] * steps
            for _, distance, speed in connector_cars[incoming]:
                if speed >= .1:
                    bucket = int(distance / max(2, speed) / self.settings.interval)
                    if bucket < steps:
                        arrivals[bucket] += 1
            for index, lane in enumerate(path):
                remainder = sum(self.topology.lanes[l].length for l in path[index + 1:])
                for vid in self.rows[lane]["ids"]:
                    if vid not in vehicles:
                        continue
                    _, position, speed, _ = vehicles[vid]
                    if speed < .1:
                        continue
                    eta = (self.topology.lanes[lane].length - position + remainder) / max(2, speed)
                    bucket = int(max(0, eta) / self.settings.interval)
                    if bucket < steps:
                        arrivals[bucket] += 1
            count = sum(self.rows[l]["stock"] for l in path) + len(connector_cars[incoming])
            entered = max(0, count - self.previous_count.get(incoming, count) + transfers[incoming])
            self.arrival_rate[incoming] = .8 * self.arrival_rate[incoming] + .2 * entered / self.settings.interval
            self.previous_count[incoming] = count
            if transfers[incoming]:
                sample = min(.7, transfers[incoming] / self.settings.interval)
                self.discharge_rate[incoming] = .85 * self.discharge_rate[incoming] + .15 * sample
            for bucket in range(steps):
                arrivals[bucket] += self.arrival_rate[incoming] * self.settings.interval * .25 * bucket / steps
            self.forecasts[incoming] = arrivals
        # Compare predicted net occupancy against later observed occupancy.
        while self.predictions and self.predictions[0][0] <= now:
            _, prediction = self.predictions.popleft()
            errors = [abs(self.previous_count.get(l, 0) - n) for l, n in prediction.items()]
            if errors:
                self.forecast_errors.append(sum(errors) / len(errors))
        prediction = {}
        for lane in self.topology.incoming:
            owner = self.signals[self.topology.owner[lane]]
            active = owner.stage == "green" and any(m.incoming == lane and owner.greens[owner.current][m.index] in "gG" for m in owner.movements)
            departure = min(self.queued[lane], self.discharge_rate[lane] * self.settings.interval) if active else 0
            prediction[lane] = max(0, self.previous_count[lane] + self.forecasts[lane][0] - departure)
        self.predictions.append((now + self.settings.interval, prediction))

    def observe_transfers(self, now, vehicles):
        def crossed(vid, incoming, outgoing):
            movement = (incoming, outgoing)
            if self.last_crossing.get(vid) == movement:
                return
            self.last_crossing[vid] = movement
            if self.learning_eligible.get(incoming, False):
                self.turn_counts[movement] += 1
            self.flow_since_tick[incoming] += 1
            self.last_service[incoming] = now
        for vid, (lane, position, speed, length) in vehicles.items():
            if lane.startswith(":"):
                movement = self.topology.connector_movements.get(lane)
                if movement:
                    # A simulation step can cross an entire 0.2 m road. Its
                    # connector identifies the actual served movement even if
                    # that external lane was never sampled.
                    crossed(vid, *movement)
                continue
            old = self.last_external.get(vid)
            if old in self.topology.receiving_groups and lane not in self.topology.receiving_groups[old]:
                self.exit_since_tick[old] += 1
            if old and old != lane and (old, lane) in self.topology.movement_keys:
                # Selectively served/capacity-censored departures do not reveal
                # arrival turning shares. Learn only during full-lane service
                # with receiving space; otherwise retain explicit priors.
                crossed(vid, old, lane)
            if lane in self.topology.lanes:
                self.last_external[vid] = lane
        for vid, old in self.last_external.items():
            if vid not in vehicles and old in self.topology.receiving_groups:
                self.exit_since_tick[old] += 1
        self.last_external = {v: lane for v, lane in self.last_external.items() if v in vehicles}
        self.last_crossing = {v: movement for v, movement in self.last_crossing.items() if v in vehicles}

    def turn_share(self, incoming, outgoing):
        weights, prior = defaultdict(float), {}
        for lane, direction in self.topology.turn_options[incoming].items():
            edge = self.topology.lanes[lane].edge
            weights[edge] += self.turn_counts[incoming, lane]
            prior[edge] = .1 if direction in ("t", "T") else 10 if direction == "s" else 5
        for edge, value in prior.items():
            weights[edge] += value
        return weights[self.topology.lanes[outgoing].edge] / max(1e-9, sum(weights.values()))

    def green_budget(self, signal, phase):
        return self.settings.horizon

    def active_movements(self, signal, phase):
        return [movement for movement in signal.movements if signal.greens[phase][movement.index] in "gG"]

    def forecast_demand(self, lane, until):
        return sum(count * max(0.0, min(1.0, until / self.settings.interval - bucket))
                   for bucket, count in enumerate(self.forecasts[lane]))

    def receiving(self, outgoing, plans):
        path = self.topology.receivers[outgoing]
        reservoir = self.topology.receiving_groups[path[-1]]
        capacity = sum(self.topology.lanes[l].length for l in reservoir) / self.settings.storage_per_vehicle
        stock = sum(self.rows[l]["stock"] for l in reservoir)
        entry = self.rows[path[0]]
        end = path[-1]
        owner = self.topology.owner.get(end)
        release = 0.0
        delay = 0.0
        if owner in self.signals:
            signal = self.signals[owner]
            target = plans.get(owner, signal.current)
            green = signal.greens[target]
            available = any(m.incoming == end for m in self.active_movements(signal, target))
            if available and (self.mode == "coordinated" or signal.stage == "green"):
                release = self.discharge_rate[end]
                if signal.stage == "native":
                    delay = self.settings.horizon
                elif signal.stage == "yellow":
                    delay = max(0, signal.until - self.last_tick) + self.settings.all_red
                elif signal.stage == "all_red":
                    extension = self.settings.clearance_extension if self.clearance_blocked(signal, self.raw_lanes) else 0
                    delay = max(0, signal.until - self.last_tick) + extension
                elif target != signal.current:
                    delay = signal.yellow + self.settings.all_red
        else:
            # An upstream ramp queue can be stopped while its far end keeps
            # discharging. Learn that actual outflow instead of treating the
            # stopped entrance's speed as the bottleneck's capacity.
            release = self.exit_rate[end]
        # A forced lane drop also depends on the receiving lane alongside it.
        if end in self.topology.forced_merges:
            merging = self.topology.forced_merges[end]
            length = sum(self.topology.lanes[l].length for l in merging)
            density = min(1.0, sum(self.rows[l]["stock"] for l in merging) * self.settings.storage_per_vehicle / max(1, length))
            if self.exit_samples[end] < 3:
                release *= max(.02, 1 - density)
        available = max(0, self.settings.horizon - delay)
        if owner in self.signals:
            available = min(available, self.green_budget(signal, target))
        supply = usable_supply(stock, capacity, entry["entry_gap"], entry["speed"], release,
                               available, self.settings.forecast_margin)
        if capacity < 1:
            # A sub-car-length connector cannot store a vehicle. Flow passes
            # through only while the next controller can discharge it.
            supply = max(0, release * available - stock)
        self.release_delays[outgoing] = delay
        if supply < 1:
            self.blocked.add(outgoing)
        return stock, capacity, supply, release

    def propagated_arrivals(self, plans):
        steps = math.ceil(self.settings.horizon / self.settings.interval)
        result = {l: [0.0] * steps for l in self.topology.incoming}
        if not self.settings.predictive:
            return result
        for sender, phase in plans.items():
            s = self.signals[sender]
            if s.stage == "native":
                continue
            clear = s.yellow + self.settings.all_red if phase != s.current else 0
            if s.stage == "yellow":
                clear = max(0, s.until - self.last_tick) + self.settings.all_red
            elif s.stage == "all_red":
                clear = max(0, s.until - self.last_tick)
            green = s.greens[phase]
            requests = {}
            by_lane = defaultdict(list)
            for m in self.active_movements(s, phase):
                by_lane[m.incoming].append(m)
            group_totals = defaultdict(float)
            for lane, moves in by_lane.items():
                p = min(1, sum(self.turn_share(lane, m.outgoing) for m in moves))
                q = self.queued[lane] + self.forecast_demand(lane, min(self.settings.horizon, clear + self.green_budget(s, phase)))
                maximum = min(head_prefix(q, p), self.discharge_rate[lane] * min(max(0, self.settings.horizon - clear), self.green_budget(s, phase)))
                for m in moves:
                    requests[m] = maximum * self.turn_share(lane, m.outgoing) / max(1e-9, p)
                    group_totals[self.topology.receivers[m.outgoing][-1]] += requests[m]
            allocations = {}
            for m, requested in requests.items():
                group = self.topology.receivers[m.outgoing][-1]
                grant = self.reservations.get(group, {}).get(sender, 0)
                allocations[m] = min(requested, grant * requested / max(1e-9, group_totals[group]))
            # Moving cohorts upstream of a signal may arrive while its green is
            # active. Release them at their predicted source ETA, not at t=0.
            for lane, moves in by_lane.items():
                queue = float(self.queued[lane])
                total_arrived, sent = 0.0, 0.0
                p = min(1, sum(self.turn_share(lane, m.outgoing) for m in moves if allocations[m] > 0))
                for bucket in range(steps):
                    arriving = self.forecasts[lane][bucket]
                    total_arrived += arriving
                    queue += arriving
                    effective = max(0, min((bucket + 1) * self.settings.interval, clear + self.green_budget(s, phase))
                                    - max(bucket * self.settings.interval, clear))
                    remaining = max(0, head_prefix(self.queued[lane] + total_arrived, p) - sent)
                    possible = min(queue, remaining, self.discharge_rate[lane] * effective)
                    for m in moves:
                        share = self.turn_share(lane, m.outgoing)
                        count = min(allocations[m], possible * share / max(1e-9, p))
                        allocations[m] -= count
                        sent += count
                        queue = max(0, queue - count)
                        end = self.topology.cell_owner.get(m.outgoing)
                        if end not in result or not count:
                            continue
                        path = self.topology.approaches[end]
                        travel = sum(self.topology.lanes[p].length / max(3, self.rows[p]["speed"]) for p in path)
                        departure = max(bucket * self.settings.interval, clear) + effective / 2
                        destination_bucket = int((departure + travel) / self.settings.interval)
                        if destination_bucket < steps:
                            result[end][destination_bucket] += count
        return result

    def reserve_receiving_space(self, plans):
        """Simultaneous proportional grants for feeders sharing one reservoir."""
        requested, budgets = defaultdict(lambda: defaultdict(float)), {}
        for tid, phase in plans.items():
            signal = self.signals[tid]
            green = signal.greens[phase]
            duration = self.settings.horizon - (signal.yellow + self.settings.all_red if phase != signal.current else 0)
            duration = min(duration, self.green_budget(signal, phase))
            by_lane = defaultdict(list)
            for m in self.active_movements(signal, phase):
                by_lane[m.incoming].append(m)
            for incoming, movements in by_lane.items():
                shares = {m.outgoing: self.turn_share(incoming, m.outgoing) for m in movements}
                probability = min(1, sum(shares.values()))
                demand = self.queued[incoming]
                if self.settings.predictive:
                    clearance = signal.yellow + self.settings.all_red if phase != signal.current else 0.0
                    demand += self.forecast_demand(incoming, min(self.settings.horizon, clearance + duration))
                possible = min(head_prefix(demand, probability), self.discharge_rate[incoming] * max(0, duration))
                for m in movements:
                    group = self.topology.receivers[m.outgoing][-1]
                    stock, capacity, supply, release = self.receiving(m.outgoing, plans)
                    budgets[group] = max(budgets.get(group, 0), supply)
                    requested[group][tid] += possible * shares[m.outgoing] / max(1e-9, probability)
        grants = {}
        for group, requests in requested.items():
            total = sum(requests.values())
            factor = min(1, budgets[group] / max(1e-9, total))
            grants[group] = {tid: amount * factor for tid, amount in requests.items()}
        return grants

    def score(self, signal, phase, plans):
        cfg = self.settings
        green = signal.greens[phase]
        switching = phase != signal.current
        clearance = signal.yellow + cfg.all_red if switching else 0
        incoming = sorted({m.incoming for m in signal.movements})
        queue = {l: float(self.queued[l]) for l in incoming}
        original = dict(queue)
        outputs = {m.outgoing for m in signal.movements}
        resources = {o: self.receiving(o, plans) for o in outputs}
        groups = {o: self.topology.receivers[o][-1] for o in outputs}
        stock = {groups[o]: r[0] for o, r in resources.items()}
        capacities = {groups[o]: r[1] for o, r in resources.items()}
        releases = {groups[o]: r[3] for o, r in resources.items()}
        delays = {groups[o]: self.release_delays.get(o, 0) for o in outputs}
        used_grants = defaultdict(float)
        served_by_lane = defaultdict(float)
        grants = {}
        for group in stock:
            others = sum(count for tid, count in self.reservations.get(group, {}).items() if tid != signal.id)
            total = max(0, capacities[group] - stock[group] - cfg.forecast_margin + releases[group] * cfg.horizon)
            if capacities[group] < 1:
                total = max(0, releases[group] * max(0, cfg.horizon - delays[group]) - stock[group])
            grants[group] = max(0, total - others)
        cost, served = 0.0, 0.0
        steps = math.ceil(cfg.horizon / cfg.interval)
        arrived_by_lane = defaultdict(float)
        head_epochs = {}
        def room_for(outgoing, bucket, transit):
            group = groups[outgoing]
            room = max(0, capacities[group] - stock[group] - cfg.forecast_margin)
            if capacities[group] < 1:
                room = transit[group]
            if bucket == 0 and resources[outgoing][2] < 1:
                room = 0
            return min(room, max(0, grants[group] - used_grants[group]))
        for bucket in range(steps):
            # Detector-based arrivals reach the stop line at their estimated ETA.
            for l in incoming:
                if cfg.predictive:
                    incoming_count = self.forecasts[l][bucket] + self.neighbour_arrivals[l][bucket]
                    queue[l] += incoming_count
                    arrived_by_lane[l] += incoming_count
            transit_budget = {}
            for group in stock:
                release_time = max(0, min(cfg.interval, (bucket + 1) * cfg.interval - delays[group]))
                transit_budget[group] = max(0, releases[group] * release_time - stock[group])
                stock[group] = max(0, stock[group] - releases[group] * release_time)
            effective = max(0, min(cfg.interval, (bucket + 1) * cfg.interval - clearance))
            for lane in incoming:
                active = [m for m in signal.movements if m.incoming == lane and green[m.index] in "gG"]
                if not active or not effective:
                    continue
                shares = {m.outgoing: self.turn_share(lane, m.outgoing) for m in active}
                usable = {m.outgoing for m in active if room_for(m.outgoing, bucket, transit_budget) > .01}
                fraction = min(1, sum(shares[o] for o in usable))
                mask = frozenset(usable)
                if lane not in head_epochs or head_epochs[lane][0] != mask:
                    head_epochs[lane] = (mask, queue[lane], arrived_by_lane[lane], served_by_lane[lane])
                _, initial, arrivals_at_reset, served_at_reset = head_epochs[lane]
                prefix = served_at_reset + head_prefix(initial + arrived_by_lane[lane] - arrivals_at_reset, fraction)
                possible = min(queue[lane], max(0, prefix - served_by_lane[lane]), self.discharge_rate[lane] * effective)
                transferred = 0
                for movement in active:
                    if movement.outgoing not in usable:
                        continue
                    weight = 1 if green[movement.index] == "G" else .55
                    group = groups[movement.outgoing]
                    supply = room_for(movement.outgoing, bucket, transit_budget)
                    flow = min(supply, possible * shares[movement.outgoing] / max(1e-9, fraction) * weight)
                    if capacities[group] < 1:
                        transit_budget[group] -= flow
                    else:
                        stock[group] += flow
                    used_grants[group] += flow
                    transferred += flow
                queue[lane] = max(0, queue[lane] - transferred)
                served_by_lane[lane] += transferred
                served += transferred
            for lane, q in queue.items():
                capacity = self.topology.capacities[lane]
                density = min(.98, q / capacity)
                # Waiting + convex congestion cost makes a nearly full short
                # approach more urgent than the same count on a long road.
                cost += cfg.interval * (q + .4 * q * density / (1 - density))
                age = max(0, self.last_tick - self.last_service.get(lane, self.last_tick))
                cost += cfg.interval * min(3, age / 90) * q * .2
            for group, count in stock.items():
                density = min(.98, count / max(1, capacities[group]))
                cost += cfg.interval * .15 * count * density / (1 - density)
            # Receiving space is shared across every lane feeding that output.
            # The model never transfers more than its conservative supply budget.
        if switching:
            cost += 5 + 2 * sum(original.values())
        self.phase_service[signal.id, phase] = dict(served_by_lane)
        return cost, served

    def change(self, signal, target, now):
        shown = "".join("r" if i in signal.masked else ch for i, ch in enumerate(signal.greens[signal.current]))
        signal.masked = frozenset()
        self.api.trafficlight.setRedYellowGreenState(signal.id, yellow_state(shown))
        signal.target, signal.stage = target, "yellow"
        signal.until = now + signal.yellow
        signal.switches += 1

    def clearance_blocked(self, signal, readings):
        """A moving vehicle is still crossing a path that conflicts with the next green.

        Only conflicting trajectories matter, and only while they are moving: a
        vehicle stopped in the box behind a full exit will not clear by waiting,
        and SUMO's junction model already keeps entering vehicles off occupied
        conflict areas. Holding all-red for it only spreads the jam.
        """
        c = self.constants
        conflicts = self.topology.conflicts.get(signal.id)
        green = signal.greens[signal.target]
        opening = [i for i, ch in enumerate(green) if ch in "gG"]
        for lane in self.topology.internal_lanes[signal.id]:
            row = readings.get(lane, {})
            if row.get(c.LAST_STEP_VEHICLE_NUMBER, 0) <= row.get(c.LAST_STEP_VEHICLE_HALTING_NUMBER, 0):
                continue
            links = self.topology.internal_link_index.get(lane)
            if not links or conflicts is None:
                return True
            if any(a != b and conflicts(a, b) for a in links for b in opening):
                return True
        return False

    def occupied_conflicts(self, signal, phase, readings):
        """Links of `phase` whose path crosses a box connector that still holds a car.

        Moving or stopped: a car stopped behind a full exit may start moving at
        any moment (a replay showed exactly that collision), so its conflicting
        links stay red until it leaves. Every other link of the phase opens.
        """
        conflicts = self.topology.conflicts.get(signal.id)
        if conflicts is None:
            return frozenset()
        green = signal.greens[phase]
        opening = [i for i, ch in enumerate(green) if ch in "gG"]
        masked = set()
        for lane in self.topology.internal_lanes[signal.id]:
            if not readings.get(lane, {}).get(self.constants.LAST_STEP_VEHICLE_NUMBER, 0):
                continue
            links = self.topology.internal_link_index.get(lane, ())
            masked.update(b for b in opening for a in links if a != b and green[a] not in "gG" and conflicts(a, b))
        return frozenset(masked)

    def show_green(self, signal, readings):
        masked = self.occupied_conflicts(signal, signal.current, readings)
        if masked != signal.masked or signal.stage != "green":
            green = signal.greens[signal.current]
            self.api.trafficlight.setRedYellowGreenState(signal.id, "".join("r" if i in masked else ch for i, ch in enumerate(green)))
            signal.masked = masked

    def advance(self, now):
        readings = None
        for signal in self.signals.values():
            if signal.stage == "green" and signal.masked:
                # Release held links as soon as the conflicting box connector empties.
                readings = readings or self.api.lane.getAllSubscriptionResults()
                self.show_green(signal, readings)
                if signal.masked:
                    signal.reason = "Green with conflicting turns held until the box clears"
            if signal.stage == "yellow" and now + 1e-6 >= signal.until:
                self.api.trafficlight.setRedYellowGreenState(signal.id, "r" * len(signal.greens[0]))
                signal.stage, signal.until = "all_red", now + self.settings.all_red
            elif signal.stage == "all_red" and now + 1e-6 >= signal.until:
                readings = self.api.lane.getAllSubscriptionResults()
                if now < signal.until + self.settings.clearance_extension and self.clearance_blocked(signal, readings):
                    signal.reason = "Clearance extended: a vehicle is still crossing a conflicting path"
                    continue
                signal.current = signal.target
                self.show_green(signal, readings)
                signal.stage, signal.since = "green", now
            elif signal.stage == "native":
                state = self.api.trafficlight.getRedYellowGreenState(signal.id)
                if state in signal.greens:
                    signal.current = signal.greens.index(state)
                    remaining = max(0, self.api.trafficlight.getNextSwitch(signal.id) - now)
                    duration = signal.green_durations[signal.current]
                    signal.stage, signal.since = "green", now - max(0, duration - remaining)
                    self.api.trafficlight.setRedYellowGreenState(signal.id, state)

    def step(self, now, vehicles=None):
        observations = vehicles if vehicles is not None else self.vehicle_observations()
        self.observe_transfers(now, observations)
        self.advance(now)
        if self.mode == "fixed":
            for s in self.signals.values():
                if s.stage == "green" and now - s.since >= s.green_durations[s.current]:
                    s.reason = "Imported fixed green schedule with common clearance protection"
                    self.change(s, (s.current + 1) % len(s.greens), now)
        if now + 1e-6 < self.last_tick + self.settings.interval:
            return
        started = time.perf_counter()
        self.last_tick = now
        self.blocked = set()
        self.observe(now, observations)
        plans = {t: s.target if s.stage in {"yellow", "all_red"} else s.current for t, s in self.signals.items()}
        feedback = set()
        for _ in range(2 if self.mode == "coordinated" else 1):
            self.reservations = self.reserve_receiving_space(plans) if self.mode == "coordinated" else {}
            self.neighbour_arrivals = self.propagated_arrivals(plans) if self.mode == "coordinated" else {l: [0.0] * math.ceil(self.settings.horizon / self.settings.interval) for l in self.topology.incoming}
            next_plans = {}
            for tid, signal in self.signals.items():
                if signal.stage != "green" or now - signal.since < self.settings.minimum_green:
                    next_plans[tid] = signal.target if signal.stage in {"yellow", "all_red"} else signal.current
                    continue
                if self.mode == "local" and not self.settings.imported_actions:
                    scores = []
                    for phase, green in enumerate(signal.greens):
                        benefit, service = 0.0, 0.0
                        by_lane = defaultdict(list)
                        for m in signal.movements:
                            if green[m.index] in "gG":
                                stock, capacity, _, release = self.receiving(m.outgoing, plans)
                                row = self.rows[m.outgoing]
                                room = usable_supply(stock, capacity, row["entry_gap"], row["speed"], 0, 0, self.settings.forecast_margin)
                                if capacity < 1 and self.release_delays.get(m.outgoing, 0) == 0:
                                    room = max(0, release * self.settings.interval - stock)
                                if room > 0:
                                    by_lane[m.incoming].append((m, stock, room))
                        served = {}
                        for lane, candidates in by_lane.items():
                            q = self.queued[lane]
                            shares = {m.outgoing: self.turn_share(lane, m.outgoing) for m, stock, room in candidates}
                            p = min(1, sum(shares.values()))
                            prefix = head_prefix(q, p)
                            density = min(.98, q / self.topology.capacities[lane])
                            age = min(3, max(0, now - self.last_service.get(lane, now)) / 90)
                            fraction = prefix / max(1, q)
                            downstream = sum(stock * shares[m.outgoing] / max(1e-9, p) for m, stock, room in candidates)
                            benefit += fraction * (q + .4 * q * density / (1 - density) + .2 * age * q) - .3 * downstream
                            flow = min(prefix, self.discharge_rate[lane] * self.settings.interval, sum(room for m, stock, room in candidates))
                            served[lane] = flow
                            service += flow
                        self.phase_service[tid, phase] = served
                        scores.append((-benefit, service))
                else:
                    scores = [self.score(signal, i, plans) for i in range(len(signal.greens))]
                signal.scores = tuple(round(s[0], 2) for s in scores)
                selected = min(range(len(scores)), key=lambda i: scores[i][0])
                aged = sorted((now - self.last_service.get(l, now), l) for l in {m.incoming for m in signal.movements} if self.queued[l] > 0)
                deadline = False
                for age, lane in reversed(aged):
                    if age < self.settings.maximum_service_gap:
                        break
                    eligible = [i for i in range(len(scores)) if self.phase_service.get((tid, i), {}).get(lane, 0) > .05]
                    if eligible:
                        selected = min(eligible, key=lambda i: scores[i][0])
                        deadline = True
                        break
                demanded = [i for i, green in enumerate(signal.greens) if scores[i][1] > .05 and any(green[m.index] in "gG" and self.queued[m.incoming] > 0 for m in signal.movements)]
                if now - signal.since >= self.settings.maximum_green and len(demanded) > 1 and (not deadline or selected == signal.current):
                    selected = min((i for i in demanded if i != signal.current), key=lambda i: scores[i][0])
                elif not deadline and scores[signal.current][0] - scores[selected][0] < max(2, abs(scores[signal.current][0]) * .05):
                    selected = signal.current
                if self.mode != "fixed" and selected == signal.current:
                    for lane, ready_at in self.ready_since.items():
                        if lane not in {m.incoming for m in signal.movements}:
                            continue
                        unchanged = now - max(ready_at, signal.since, self.last_service.get(lane, now))
                        if unchanged < 15:
                            continue
                        current_links = {m.index for m in signal.movements if m.incoming == lane and signal.greens[signal.current][m.index] == "G"}
                        if not current_links:
                            continue
                        alternatives = [i for i, g in enumerate(signal.greens) if i != signal.current
                                        and self.phase_service.get((tid, i), {}).get(lane, 0) > .001
                                        and {m.index for m in signal.movements if m.incoming == lane and g[m.index] == "G"} - current_links]
                        if alternatives:
                            selected = min(alternatives, key=lambda i: (i - signal.current) % len(signal.greens))
                            feedback.add(tid)
                            break
                signal.predicted_service = round(scores[selected][1], 2)
                next_plans[tid] = selected
            plans = next_plans
            if self.mode == "coordinated":
                self.messages += sum(len(self.topology.graph[t] & self.topology.scope) for t in self.signals)
        if self.mode != "fixed":
            for tid, selected in plans.items():
                signal = self.signals[tid]
                if signal.stage == "green" and selected != signal.current:
                    if tid in feedback:
                        signal.reason = "Try another legal turn: a green has not moved the ready head"
                    elif now - signal.since >= self.settings.maximum_green:
                        signal.reason = "Serve another approach after the maximum green"
                    else:
                        signal.reason = "Forecast arrivals and shared receiving space favour this movement" if self.mode == "coordinated" else "Current local queue"
                    self.change(signal, selected, now)
                    self.decisions += 1
                elif signal.stage == "green":
                    queued = sum(self.queued[m.incoming] for m in signal.movements)
                    if signal.predicted_service < .05 and queued:
                        signal.reason = "Holding while downstream receiving space is constrained"
                    else:
                        signal.reason = "Continue serving this movement" if self.mode == "local" else "Keep green: predicted benefit exceeds switching cost"
        self.plans = plans
        self.reservations = self.reserve_receiving_space(plans) if self.mode == "coordinated" else {}
        self.elapsed_ms = (time.perf_counter() - started) * 1000

    def status(self):
        return {"mode": self.mode, "signal_count": len(self.signals), "horizon": self.settings.horizon if self.mode == "coordinated" else self.settings.interval if self.mode == "local" else 0,
                "decisions": self.decisions, "neighbour_messages": self.messages,
                "shared_supply_reservations": sum(len(grants) > 1 for grants in self.reservations.values()),
                "merge_discharge_vehicles_per_hour": round(self.exit_rate["1134690192_0"] * 3600, 1),
                "blocked_receivers": len(self.blocked), "decision_ms": round(self.elapsed_ms, 2),
                "forecast_mae_vehicles": round(sum(self.forecast_errors) / len(self.forecast_errors), 2) if self.forecast_errors else None,
                "signals": {tid: {"phase": s.current, "stage": s.stage, "target": s.target, "reason": s.reason,
                                   "predicted_service": s.predicted_service, "scores": s.scores, "switches": s.switches,
                                   "queued_vehicles": sum(self.queued.get(l, 0) for l in {m.incoming for m in s.movements}),
                                   "expected_arrivals": round(sum(sum(self.forecasts.get(l, ())) for l in {m.incoming for m in s.movements}), 1)}
                            for tid, s in self.signals.items()}}
