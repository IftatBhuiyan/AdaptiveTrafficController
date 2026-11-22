"""Learned phase selection that can see a few blocks around each signal.

One small value network is shared by every controlled signal. Every five
seconds, for each signal that has finished its minimum green, it scores each
legal phase ("keep" is the current phase) and the highest score wins. The
network was fitted to how the number of vehicles upstream of the signal and of
its neighbours (including vehicles still waiting to enter the map) actually
changed after each logged decision (see train_neighborhood_ml.py).

What the model does NOT control: the list of phases, minimum green, yellow,
all-red, occupied-box clearance, the maximum green and the maximum service gap.
Those stay in IntelligentCoordinator / AdaptiveController and in
``legal_candidates`` below.

Two observation sets use the same code, model size, data and targets:

* ``local``: only this signal's own approaches and its own signal state.
* ``neighborhood``: local, plus the receiving roads downstream, the signals
  feeding each approach, traffic further upstream than the local detectors
  reach, and queue levels one and two signals away.

All observations are present or past values. No vehicle routes and no scheduled
departures are read. They are ideal simulated measurements (exact counts and
speeds per lane); a real site would need stop-line, advance and exit detectors
or camera/radar counts to approximate them.
"""
from collections import defaultdict, deque
import hashlib
import json
import math
from pathlib import Path
import random
import time

import numpy as np

from adaptive_controller import Settings
from intelligent_coordinator import IntelligentCoordinator

REWARD_HORIZON_TICKS = 16      # 80 s of consequences per decision
DISCOUNT = 0.95                # per five-second tick
NEIGHBOUR_REWARD_WEIGHT = 0.5  # queues at adjacent signals count half
REWARD_SCALE = 10.0
CATCHMENT_LIMIT = 1500.0       # metres upstream a signal's catchment may reach
POLICIES = {"fixed", "max_pressure", "max_pressure_guarded", "cycle_guarded", "ml"}
# "advance": keep the current green or move to the next green of the fixed
# plan's order (green lengths adapt, the order does not). "phase": any phase.
ACTION_SPACES = {"advance", "phase"}

_SIDE = ("queue", "moving", "arrivals_10s", "arrivals_30s", "max_occupancy", "max_service_age",
         "arrival_rate_30s", "queue_change_30s")
_DOWN = ("mean_occupancy", "max_occupancy", "blocked_demand_fraction", "min_free_space",
         "green_fraction", "halting_fraction")
_UP = ("queue", "queue_released", "flow_10s", "far_vehicles", "far_delay")
LOCAL_FEATURES = (("is_keep", "green_elapsed", "switch_lost_time", "phase_count", "nominal_green",
                   "since_candidate_served", "relief_phase")
                  + tuple(f"{side}.{name}" for side in ("served", "unserved", "current") for name in _SIDE))
DOWNSTREAM_FEATURES = tuple(f"{side}.down_{name}" for side in ("served", "unserved") for name in _DOWN)
UPSTREAM_FEATURES = tuple(f"{side}.up_{name}" for side in ("served", "unserved") for name in _UP)
AREA_FEATURES = ("hop1.mean_occupancy", "hop1.queue", "hop1.max_occupancy", "hop1.count",
                 "hop2.mean_occupancy", "hop2.queue")
FEATURE_NAMES = LOCAL_FEATURES + DOWNSTREAM_FEATURES + UPSTREAM_FEATURES + AREA_FEATURES
VARIANTS = {
    "local": LOCAL_FEATURES,
    "local_downstream": LOCAL_FEATURES + DOWNSTREAM_FEATURES,
    "local_upstream": LOCAL_FEATURES + UPSTREAM_FEATURES,
    "neighborhood": FEATURE_NAMES,
}
SENSING = {
    "local": "Own approaches (to the previous signal, at most 250 m) and own signal state.",
    "neighborhood": "Local, plus receiving roads to the next signal, the signals feeding each approach "
                    "(their queues, greens and recent departures), vehicles further upstream than 250 m, "
                    "and queues one and two controlled signals away.",
    "ideal": "Exact per-lane counts, stopped counts, speeds and positions from the simulator. "
             "Not roadside-sensor measurements.",
}


class QModel:
    """A frozen multilayer perceptron stored as JSON."""

    def __init__(self, payload):
        self.payload = payload
        self.variant = payload["variant"]
        self.action_space = payload.get("action_space", "phase")
        self.feature_names = tuple(payload["feature_names"])
        unknown = set(self.feature_names) - set(FEATURE_NAMES)
        if unknown:
            raise ValueError("Model uses features this controller does not compute: " + ", ".join(sorted(unknown)))
        self.columns = np.array([FEATURE_NAMES.index(name) for name in self.feature_names])
        self.mean = np.array(payload["mean"], dtype=np.float64)
        self.std = np.array(payload["std"], dtype=np.float64)
        self.layers = [(np.array(layer["W"], dtype=np.float64), np.array(layer["b"], dtype=np.float64))
                       for layer in payload["layers"]]
        self.target_mean, self.target_std = payload["target"]["mean"], payload["target"]["std"]
        if payload.get("weights_sha256") != weights_hash(payload):
            raise ValueError("Model file does not match its recorded weight hash")
        self.id = payload["weights_sha256"][:12]

    @classmethod
    def load(cls, path):
        return cls(json.loads(Path(path).read_text()))

    def predict(self, features):
        """Predicted discounted area reward for rows of the full feature matrix."""
        hidden = (np.asarray(features, dtype=np.float64)[:, self.columns] - self.mean) / self.std
        for index, (weights, bias) in enumerate(self.layers):
            hidden = hidden @ weights + bias
            if index < len(self.layers) - 1:
                hidden = np.maximum(hidden, 0.0)
        return hidden[:, 0] * self.target_std + self.target_mean


def weights_hash(payload):
    content = json.dumps([payload["feature_names"], payload["mean"], payload["std"], payload["layers"],
                          payload["target"]], sort_keys=True)
    return hashlib.sha256(content.encode()).hexdigest()


class NeighborhoodController(IntelligentCoordinator):
    """Fixed, max-pressure and learned policies on one phase list and one safety layer."""

    def __init__(self, api, network, policy="fixed", settings=Settings(), model=None,
                 epsilon=0.0, seed=0, record=False, entry_edges=(), action_space="phase"):
        if policy not in POLICIES or action_space not in ACTION_SPACES:
            raise ValueError("Unknown signal policy")
        if policy == "ml" and model is None:
            raise ValueError("The learned policy needs a trained model file")
        super().__init__(api, network, "fixed", settings)
        self.mode = policy
        self.model = QModel.load(model) if isinstance(model, (str, Path)) else model
        self.action_space = self.model.action_space if self.model is not None else action_space
        self.epsilon, self.rng, self.record = epsilon, random.Random(seed), record
        self.signal_ids = tuple(sorted(key for key in self.signals if key not in self.followers))
        self.signal_index = {key: index for index, key in enumerate(self.signal_ids)}
        self.lanes_of = {key: tuple(sorted({m.incoming for m in self.signals[key].movements})) for key in self.signals}
        graph = self.topology.graph
        self.hop1 = {key: tuple(sorted(graph[key] & self.signals.keys())) for key in self.signals}
        self.hop2 = {key: tuple(sorted({far for near in self.hop1[key] for far in self.hop1[near]}
                                       - set(self.hop1[key]) - {key})) for key in self.signals}
        # Which signal movements release traffic toward each controlled approach.
        self.feeders = defaultdict(list)
        native_movements = defaultdict(int)
        for key in self.native_ids:
            for movement in self.topology.network_movements.get(key, ()):
                native_movements[movement.incoming] += 1
        for key, signal in self.signals.items():
            for movement in signal.movements:
                end = self.topology.receivers[movement.outgoing][-1]
                if end in self.lanes_of.get(self.topology.owner.get(end), ()) and self.topology.owner[end] != key:
                    self.feeders[end].append((key, movement, None))
        for key in self.native_ids:
            for movement in self.topology.network_movements.get(key, ()):
                end = self.topology.walk(movement.outgoing)[-1]
                if end in self.topology.owner:
                    self.feeders[end].append((key, movement, 1.0 / native_movements[movement.incoming]))
        # Catchment: every lane whose traffic reaches this stop line before any
        # other signal. Each lane belongs to its nearest stop line.
        import heapq
        topology = self.topology
        distance, owner_lane = {}, {}
        heap = [(0.0, lane, lane) for lane in topology.incoming]
        while heap:
            metres, lane, source = heapq.heappop(heap)
            if lane in distance:
                continue
            distance[lane], owner_lane[lane] = metres, source
            for previous, _ in topology.predecessors[lane]:
                boundary = topology.edges[topology.lanes[previous].edge][1]
                if previous in distance or boundary in topology.node_tls:
                    continue
                reach = metres + topology.lanes[lane].length
                if reach <= CATCHMENT_LIMIT:
                    heapq.heappush(heap, (reach, previous, source))
        self.catchment = defaultdict(list)
        for lane, source in owner_lane.items():
            self.catchment[source].append(lane)
        self.far_lanes = {incoming: tuple(lane for lane in self.catchment[incoming]
                                          if lane not in topology.approaches[incoming]) for incoming in topology.incoming}
        # Entry backlog is used only to compute the training target. Vehicles
        # waiting to enter on an edge are charged to the signals whose
        # catchment contains that edge, by their share of its catchment lanes.
        self.backlog_share = {}
        for edge in entry_edges:
            owners = defaultdict(float)
            for lane, source in owner_lane.items():
                if topology.lanes[lane].edge == edge:
                    owners[topology.owner[source]] += 1.0
            total = sum(owners.values())
            if total:
                self.backlog_share[edge] = {key: count / total for key, count in owners.items()}
        # The fixed plan's own green and cycle lengths are never undercut by
        # the fairness rules below.
        self.cycle_length = {key: sum(signal.green_durations[:self.reference_counts[key]])
                             + self.reference_counts[key] * (signal.yellow + self.settings.all_red)
                             for key, signal in self.signals.items()}
        self.queue_history = deque(maxlen=7)       # 30 s of approach queues
        self.departure_history = deque(maxlen=3)   # 10 s of stop-line departures
        self.phase_last_green = {}
        self.tick = -1
        self.cache = {}
        self.samples, self.load_log = [], []
        self.model_ms, self.decision_log = 0.0, {}
        self.random_actions = self.forced_actions = self.model_decisions = 0

    # ---- observations -------------------------------------------------
    def prepare(self, now):
        """Per-tick quantities shared by every candidate phase."""
        topology, rows = self.topology, self.rows
        self.queue_history.append(dict(self.queued))
        self.departure_history.append(dict(self.observed_departures))
        past_queue, past_departures = self.queue_history[0], self.departure_history[0]
        lane = {}
        for incoming in topology.incoming:
            queue = float(self.queued[incoming])
            stock = float(sum(rows[cell]["stock"] for cell in topology.approaches[incoming]))
            forecast = self.forecasts[incoming]
            lane[incoming] = (queue, max(0.0, stock - queue), sum(forecast[:2]), sum(forecast),
                              min(1.5, stock / topology.capacities[incoming]),
                              min(3.0, max(0.0, now - self.last_service.get(incoming, now)) / 90.0) if queue else 0.0,
                              self.arrival_rate[incoming] * 30.0, queue - past_queue.get(incoming, queue))
        departed = {incoming: self.observed_departures.get(incoming, 0) - past_departures.get(incoming, 0)
                    for incoming in topology.incoming}
        shares = {}
        def share(incoming, outgoing):
            key = incoming, outgoing
            if key not in shares:
                shares[key] = self.turn_share(incoming, outgoing)
            return shares[key]
        upstream = {}
        for incoming in topology.incoming:
            queue = released = flow = 0.0
            for key, movement, native_share in self.feeders.get(incoming, ()):
                if native_share is None:
                    signal = self.signals[key]
                    weight = share(movement.incoming, movement.outgoing)
                    waiting = self.queued[movement.incoming] * weight
                    green = signal.stage == "green" and signal.greens[signal.current][movement.index] in "gG"
                    flow += departed[movement.incoming] * weight
                else:
                    waiting = rows.get(movement.incoming, {}).get("queue", 0) * native_share
                    state = self.native_states.get(key, {}).get("state", "")
                    green = movement.index < len(state) and state[movement.index] in "gG"
                queue += waiting
                released += waiting if green else 0.0
            far = [rows[cell] for cell in self.far_lanes[incoming]]
            far_delay = sum(row["stock"] * max(0.0, 1.0 - row["speed"] / topology.lanes[cell].speed)
                            for row, cell in zip(far, self.far_lanes[incoming]))
            upstream[incoming] = (queue, released, flow, float(sum(row["stock"] for row in far)), far_delay)
        downstream = {}
        for outgoing, path in topology.receivers.items():
            end = path[-1]
            reservoir = topology.receiving_groups[end]
            capacity = max(1.0, sum(topology.lanes[cell].length for cell in reservoir) / self.settings.storage_per_vehicle)
            stock = sum(rows[cell]["stock"] for cell in reservoir)
            halting = sum(rows[cell]["queue"] for cell in reservoir)
            entry = rows[path[0]]
            blocked = float(entry["stock"] > 0 and entry["entry_gap"] < 6 and entry["speed"] < .5)
            green = 1.0
            if end in topology.owner:
                signal = self.signals[topology.owner[end]]
                green = float(signal.stage == "green" and any(
                    m.incoming == end and signal.greens[signal.current][m.index] in "gG" for m in signal.movements))
            elif end in self.native_owners:
                key = self.native_owners[end]
                state = self.native_states.get(key, {}).get("state", "")
                green = float(any(m.incoming == end and m.index < len(state) and state[m.index] in "gG"
                                  for m in topology.network_movements[key]))
            downstream[outgoing] = (min(1.5, stock / capacity), blocked, min(20.0, max(0.0, capacity - stock)) / 20.0,
                                    green, halting / max(1.0, stock))
        area = {}
        for key, lanes in self.lanes_of.items():
            occupancy = [lane[incoming][4] for incoming in lanes]
            area[key] = (sum(lane[incoming][0] for incoming in lanes), sum(occupancy) / max(1, len(occupancy)),
                         max(occupancy, default=0.0))
        for key, signal in self.signals.items():
            if signal.stage == "green":
                self.phase_last_green[key, signal.current] = now
        self.cache = {"lane": lane, "share": share, "upstream": upstream, "downstream": downstream, "area": area}

    def area_load(self):
        """Training target input: vehicles upstream of each signal, plus entry backlog.

        Summed over all signals this is the vehicle count the system metric
        integrates (active plus not-yet-inserted), up to vehicles past the
        last signal. Entry backlog is not an observation and no model input
        contains it.
        """
        rows = self.rows
        load = {key: float(sum(rows[lane]["stock"] for incoming in lanes for lane in self.catchment[incoming]))
                for key, lanes in self.lanes_of.items()}
        for edge, owners in self.backlog_share.items():
            waiting = len(self.api.edge.getPendingVehicles(edge))
            for key, share in owners.items():
                load[key] += waiting * share
        return [load[key] + NEIGHBOUR_REWARD_WEIGHT * sum(load[other] for other in self.hop1[key])
                for key in self.signal_ids]

    def phase_weights(self, signal, phase):
        """How much of each approach lane, and of each movement, a phase serves."""
        share, green = self.cache["share"], signal.greens[phase]
        lanes, movements = defaultdict(float), []
        for movement in signal.movements:
            state = green[movement.index]
            weight = share(movement.incoming, movement.outgoing) * (1.0 if state == "G" else .5 if state == "g" else 0.0)
            lanes[movement.incoming] += weight
            movements.append((movement, share(movement.incoming, movement.outgoing), weight))
        return lanes, movements

    def features(self, key, phases, now):
        signal, cache = self.signals[key], self.cache
        lane, upstream, downstream = cache["lane"], cache["upstream"], cache["downstream"]
        lanes = self.lanes_of[key]

        def local_block(weights):
            used = [(incoming, weight) for incoming, weight in weights if weight > 1e-6]
            block = [sum(weight * lane[incoming][column] for incoming, weight in used) for column in range(4)]
            block.append(max((lane[incoming][4] for incoming, _ in used), default=0.0))
            block.append(max((lane[incoming][5] for incoming, _ in used), default=0.0))
            block.extend(sum(weight * lane[incoming][column] for incoming, weight in used) for column in (6, 7))
            return block

        def upstream_block(weights):
            return [sum(weight * upstream[incoming][column] for incoming, weight in weights) for column in range(5)]

        def downstream_block(movements):
            used = [(movement, weight) for movement, weight in movements if weight > 1e-6]
            if not used:
                return [0.0, 0.0, 0.0, 1.0, 1.0, 0.0]
            total = sum(weight for _, weight in used)
            demand = [(lane[movement.incoming][0] + lane[movement.incoming][2]) * weight for movement, weight in used]
            rows = [downstream[movement.outgoing] for movement, _ in used]
            return [sum(weight * row[0] for (_, weight), row in zip(used, rows)) / total,
                    max(row[0] for row in rows),
                    sum(amount * row[1] for amount, row in zip(demand, rows)) / (sum(demand) + 1e-6),
                    min(row[2] for row in rows),
                    sum(weight * row[3] for (_, weight), row in zip(used, rows)) / total,
                    sum(weight * row[4] for (_, weight), row in zip(used, rows)) / total]

        current_lanes, _ = self.phase_weights(signal, signal.current)
        current = local_block([(incoming, current_lanes[incoming]) for incoming in lanes])
        elapsed = min(2.0, max(0.0, now - signal.since) / 50.0)
        hop1 = [cache["area"][other] for other in self.hop1[key]]
        hop2 = [cache["area"][other] for other in self.hop2[key]]
        area = [sum(row[1] for row in hop1) / max(1, len(hop1)), sum(row[0] for row in hop1) / 50.0,
                max((row[2] for row in hop1), default=0.0), len(hop1) / 6.0,
                sum(row[1] for row in hop2) / max(1, len(hop2)), sum(row[0] for row in hop2) / 100.0]
        result = np.empty((len(phases), len(FEATURE_NAMES)), dtype=np.float32)
        for row, phase in enumerate(phases):
            weights, movements = self.phase_weights(signal, phase)
            served = [(incoming, weights[incoming]) for incoming in lanes]
            unserved = [(incoming, 1.0 - weights[incoming]) for incoming in lanes]
            keep = phase == signal.current
            since = min(3.0, (now - self.phase_last_green.get((key, phase), now - 360.0)) / 120.0)
            values = [float(keep), elapsed, 0.0 if keep else (signal.yellow + self.settings.all_red) / 10.0,
                      len(signal.greens) / 6.0, signal.green_durations[phase] / 60.0, 0.0 if keep else since,
                      float(phase >= self.reference_counts[key])]
            values += local_block(served) + local_block(unserved) + current
            values += downstream_block([(movement, weight) for movement, _, weight in movements])
            values += downstream_block([(movement, share - weight) for movement, share, weight in movements])
            values += upstream_block(served) + upstream_block(unserved) + area
            result[row] = values
        return result

    # ---- rules the model cannot override ------------------------------
    def legal_candidates(self, signal, now):
        """Phases the policy may choose now, after maximum green and service-gap rules."""
        phases = list(range(len(signal.greens)))
        if self.action_space == "advance":
            following = (signal.current + 1) % self.reference_counts[signal.id]
            phases = [signal.current, following] if following != signal.current else [signal.current]
        green = signal.greens[signal.current]
        served_now = {m.incoming for m in signal.movements if green[m.index] in "gG"}
        waiting = [(now - self.last_service.get(incoming, now), incoming) for incoming in self.lanes_of[signal.id]
                   if incoming not in served_now and self.queued[incoming] > 0]
        if not waiting:
            return phases, None
        age, oldest = max(waiting)
        if age >= max(self.settings.maximum_service_gap, self.cycle_length[signal.id]):
            serving = [phase for phase in phases if any(
                m.incoming == oldest and signal.greens[phase][m.index] in "gG" for m in signal.movements)]
            if self.action_space == "advance" and len(phases) > 1:
                serving = phases[1:]  # the fixed order reaches the waiting approach
            if serving:
                return serving, "Service-gap rule: an approach has waited %d s" % age
        if now - signal.since >= max(self.settings.maximum_green, signal.green_durations[signal.current]):
            return ([phase for phase in phases if phase != signal.current] or phases,
                    "Maximum green reached with others waiting")
        return phases, None

    def pressure_weight(self, signal, phase):
        """Varaiya (2013) max pressure, the same weight IntelligentCoordinator.max_pressure uses."""
        green, weight = signal.greens[phase], 0.0
        for movement in signal.movements:
            if green[movement.index] in "gG":
                share = self.cache["share"](movement.incoming, movement.outgoing)
                downstream = self.topology.cell_owner.get(movement.outgoing)
                weight += share * (self.queued.get(movement.incoming, 0) - self.queued.get(downstream, 0))
        return weight

    def base_choice(self, signal, candidates, now):
        if self.mode == "cycle_guarded":
            if now - signal.since < signal.green_durations[signal.current] and signal.current in candidates:
                return signal.current
            following = (signal.current + 1) % self.reference_counts[signal.id]
            return following if following in candidates else candidates[0]
        weights = {phase: self.pressure_weight(signal, phase) for phase in candidates}
        best = max(candidates, key=lambda phase: (weights[phase], phase == signal.current))
        if signal.current in candidates and weights[best] <= weights[signal.current] + 1e-9:
            return signal.current
        return best

    # ---- control loop --------------------------------------------------
    def step(self, now, vehicles=None):
        if self.mode in {"fixed", "max_pressure"}:
            return super().step(now, vehicles)
        observations = vehicles if vehicles is not None else self.vehicle_observations()
        self.current_vehicles = observations
        self.observe_transfers(now, observations)
        self.advance(now)
        if now + 1e-6 < self.last_tick + self.settings.interval:
            return
        started = time.perf_counter()
        self.last_tick = now
        self.blocked = set()
        self.observe(now, observations)
        self.prepare(now)
        self.tick += 1
        if self.record:
            self.load_log.append(self.area_load())
        deciding = []
        for key in self.signal_ids:
            signal = self.signals[key]
            if signal.stage != "green" or now - signal.since < self.settings.minimum_green:
                continue
            candidates, rule = self.legal_candidates(signal, now)
            if len(candidates) == 1:
                self.forced_actions += candidates[0] != signal.current
                deciding.append((key, candidates, rule, None))
                continue
            needs_features = self.mode == "ml" or self.record
            deciding.append((key, candidates, rule, self.features(key, candidates, now) if needs_features else None))
        values, self.model_ms = {}, 0.0
        if self.mode == "ml":
            batch = [(key, matrix) for key, _, _, matrix in deciding if matrix is not None]
            if batch:
                model_started = time.perf_counter()
                scores = self.model.predict(np.concatenate([matrix for _, matrix in batch]))
                self.model_ms = (time.perf_counter() - model_started) * 1000
                offset = 0
                for key, matrix in batch:
                    values[key] = scores[offset:offset + len(matrix)]
                    offset += len(matrix)
        for key, candidates, rule, matrix in deciding:
            signal = self.signals[key]
            explored = False
            if len(candidates) == 1:
                choice = candidates[0]
                reason = rule or "Only one legal phase"
            elif self.epsilon and self.rng.random() < self.epsilon:
                choice, explored = self.rng.choice(candidates), True
                reason = "Exploration: random legal phase (training data only)"
                self.random_actions += 1
            elif self.mode == "ml":
                scores = values[key]
                order = max(range(len(candidates)), key=lambda i: (scores[i], candidates[i] == signal.current))
                choice = candidates[order]
                self.model_decisions += 1
                keep = candidates.index(signal.current) if signal.current in candidates else None
                signal.scores = tuple(round(float(-score), 2) for score in scores)
                self.decision_log[key] = {"phases": list(candidates), "values": [round(float(s), 3) for s in scores]}
                reason = ("Learned value prefers phase %d (%.2f) over %s" % (
                    choice + 1, scores[order], "keeping (%.2f)" % scores[keep] if keep is not None else "other phases")
                    if choice != signal.current else "Learned value: keeping this green is best (%.2f)" % scores[order])
                if rule:
                    reason = rule + ". " + reason
            else:
                choice = self.base_choice(signal, candidates, now)
                reason = rule or ("Max pressure with service rules" if self.mode == "max_pressure_guarded"
                                  else "Fixed order with service rules")
            if self.record and matrix is not None:
                self.samples.append((self.tick, self.signal_index[key], matrix[candidates.index(choice)],
                                     explored, len(candidates), choice == signal.current))
            signal.reason = reason
            if choice != signal.current:
                self.change(signal, choice, now)
                self.decisions += 1
        self.control_evaluations += len(self.signal_ids)
        self.elapsed_ms = (time.perf_counter() - started) * 1000

    def training_arrays(self):
        """Logged decisions with the discounted change in area load that followed each one.

        Measuring load relative to its value at decision time removes a term
        that depends only on the state (it cannot change which phase scores
        highest) and that no observation could predict, the entry backlog.
        """
        load = np.array(self.load_log, dtype=np.float64).reshape(-1, len(self.signal_ids))
        rows = [sample for sample in self.samples if sample[0] + REWARD_HORIZON_TICKS < len(load)]
        if not rows:
            return None
        ticks = np.array([row[0] for row in rows])
        signals = np.array([row[1] for row in rows])
        returns = np.zeros(len(rows))
        for offset in range(1, REWARD_HORIZON_TICKS + 1):
            returns -= DISCOUNT ** (offset - 1) * (load[ticks + offset, signals] - load[ticks, signals]) / REWARD_SCALE
        return {"features": np.stack([row[2] for row in rows]).astype(np.float32),
                "returns": returns.astype(np.float32), "tick": ticks.astype(np.int32),
                "signal": signals.astype(np.int16), "explored": np.array([row[3] for row in rows]),
                "candidates": np.array([row[4] for row in rows], dtype=np.int8),
                "keep": np.array([row[5] for row in rows])}

    def status(self):
        status = super().status()
        learned = self.mode == "ml"
        status.update(controller="neighborhood_ml" if learned else self.mode, policy=self.mode,
                      action_space=self.action_space,
                      model_variant=self.model.variant if learned else None,
                      model_id=self.model.id if learned else None,
                      model_ms=round(self.model_ms, 3), model_decisions=self.model_decisions,
                      forced_by_service_rules=self.forced_actions, random_actions=self.random_actions,
                      horizon=REWARD_HORIZON_TICKS * self.settings.interval if learned else status.get("horizon", 0),
                      sensing=SENSING.get(self.model.variant, SENSING["neighborhood"]) if learned else None)
        return status


def reward_definition():
    return ("Area load = vehicles on roads upstream of the signal (back to the previous signal or the map edge) "
            "plus vehicles waiting to enter there, plus %.1f x the same for directly connected controlled signals. "
            "Target = minus the discounted sum, over the next %d five-second ticks (discount %.2f), of the change "
            "in area load since the decision." % (NEIGHBOUR_REWARD_WEIGHT, REWARD_HORIZON_TICKS, DISCOUNT))
