"""Receding-horizon network control over a bounded catalogue of legal schedules.

The solver is deterministic coordinate search on the *whole network rollout*,
not independent next-phase scores. It returns a feasible local incumbent; it
does not claim a globally optimal solution. See MPC.md for the model limits.
"""
from collections import defaultdict, deque
import math
import time
import xml.etree.ElementTree as ET

import numpy as np

from adaptive_controller import Settings, head_prefix
from intelligent_coordinator import IntelligentCoordinator
from traffic_forecasting import ArrivalPredictor, make_features


class NetworkMPC(IntelligentCoordinator):
    interval = 5.0
    horizon_steps = 6
    deadline_ms = 250.0
    candidate_limit = 8
    maximum_evaluations = 192

    def __init__(self, api, network, mode="mpc", settings=Settings(), model_path=None, boundary_exits=None):
        if mode not in {"fixed", "mpc", "mpc_learned"}:
            raise ValueError("Unknown MPC policy")
        super().__init__(api, network, "fixed", settings)
        self.mode = mode
        self.predictor = ArrivalPredictor("learned" if mode == "mpc_learned" else "simple", model_path)
        self.boundary_exit_edges = set(boundary_exits) if boundary_exits is not None else None
        self.lane_ids = tuple(sorted(self.topology.lanes))
        self.lane_index = {lane: i for i, lane in enumerate(self.lane_ids)}
        self.capacities = np.array([self.topology.lanes[l].length / settings.storage_per_vehicle for l in self.lane_ids])
        # A zero-indegree lane can be an orphan inside the map after a
        # topology repair. Learn actual insertion locations from current
        # observations instead of inventing boundary demand there.
        self.boundaries = set()
        self.unassigned_pending = 0
        self.entry_counts = defaultdict(int)
        self.histories = defaultdict(lambda: deque(maxlen=12))
        self.seen = set()
        self.movement_counts = defaultdict(float)
        self.previous_roads = {}
        self.forecast_samples = []
        self.pending_samples = {}
        self.solver_events = []
        self.prediction_errors = []
        self.last_prediction = None
        self.optimization_round = 0
        self.last_plan = {}
        self._build_movements(ET.parse(network).getroot())

    def _build_movements(self, root):
        # Connectors are geometry, not knowledge of any vehicle's future route.
        movements = []
        seen = set()
        for c in root.findall("connection"):
            a, b = c.get("from") + "_" + c.get("fromLane"), c.get("to") + "_" + c.get("toLane")
            if a not in self.lane_index or b not in self.lane_index or (a, b) in seen:
                continue
            seen.add((a, b))
            movements.append((a, b, c.get("tl"), int(c.get("linkIndex", "0")), c.get("dir", "s"), None))
        # Eliminate zero-storage portal cells from new transfers. A feeder's
        # discharge enters the receiver's exit directly, under BOTH green gates.
        # Existing cars on the short cells remain in the state and must drain.
        expanded = []
        for a, b, tid, link, direction, _ in movements:
            group = self.groups.get(tid)
            if group and b in group.cells:
                path = self.topology.receivers.get(b, (b,))
                downstream = self.signals[group.follower]
                exits = [m for m in downstream.movements if m.incoming == path[-1]]
                for m in exits:
                    expanded.append((a, m.outgoing, tid, link, direction, (group.follower, m.index)))
            else:
                expanded.append((a, b, tid, link, direction, None))
        self.movements = tuple(expanded)
        self.src = np.array([self.lane_index[m[0]] for m in expanded], dtype=int)
        self.dst = np.array([self.lane_index[m[1]] for m in expanded], dtype=int)
        through_edges = {self.topology.lanes[m[0]].edge for m in expanded if m[4] not in ("t", "T")}
        self.sinks = np.array([i for i, l in enumerate(self.lane_ids)
                               if (self.topology.lanes[l].edge in self.boundary_exit_edges if self.boundary_exit_edges is not None
                                   else self.topology.lanes[l].edge not in through_edges)], dtype=int)
        self.actuators = tuple(sorted(set(self.signals) - set(self.followers)))
        self.rate = np.full(len(self.lane_ids), self.settings.saturation_flow * self.interval)
        self.previous_roads = {}
        self.gate_indices = {key: np.array([j for j, m in enumerate(self.movements)
                                            if m[2] == key or self.followers.get(m[2]) == key], dtype=int)
                             for key in self.actuators}
        self.gate_cache = {}
        self.permission_cache = {}
        self.occupied_cache = {}

    def observe_transfers(self, now, vehicles):
        if hasattr(self, "seen"):
            for vid, (lane, _, _, _) in vehicles.items():
                old = self.previous_roads.get(vid)
                if lane in self.lane_index:
                    if vid not in self.seen:
                        self.boundaries.add(lane)
                        self.entry_counts[lane] += 1
                    if old is not None and old != lane and self.learning_eligible.get(old, False):
                        self.movement_counts[old, lane] += 1
                    self.previous_roads[vid] = lane
            self.seen.update(vehicles)
            self.previous_roads = {v: l for v, l in self.previous_roads.items() if v in vehicles}
        super().observe_transfers(now, vehicles)

    def observe(self, now, vehicles):
        super().observe(now, vehicles)
        if not hasattr(self, "histories"):
            return
        try:
            pending = len(self.api.simulation.getPendingVehicles()) / max(1, len(self.boundaries))
        except (AttributeError, RuntimeError):
            pending = 0.0
        for lane in sorted(self.boundaries):
            count = self.entry_counts.pop(lane, 0)
            if lane in self.pending_samples:
                self.forecast_samples.append(dict(self.pending_samples[lane], target=count))
            self.histories[lane].append(count)
            row = self.rows.get(lane, {})
            features = make_features(self.histories[lane], row.get("stock", 0), row.get("queue", 0), pending, now)
            self.pending_samples[lane] = {"lane": lane, "time": now, "features": list(features),
                                          "history": list(self.histories[lane])}

    def _schedule(self, key, first, dwell, second=None, span=10.0):
        """Grid gates conservatively round yellow/all-red UP, never down."""
        s = self.signals[key]
        dt, H = self.interval, self.horizon_steps
        current = s.current
        result = np.full(H, current, dtype=int)
        clearance = s.yellow + self.settings.all_red
        group = self.groups.get(key)
        if group:
            clearance += self.signals[group.follower].yellow + self.settings.all_red
        if s.stage == "native":
            # Until the inherited safety manager observes an announced green,
            # an offset native yellow/red is not a promise of immediate service.
            return np.full(H, -1, dtype=int), s.target
        if s.stage != "green":
            committed = s.target
            wait = max(0.0, s.until - self.last_tick)
            if s.stage == "yellow":
                wait += self.settings.all_red
            if group and self.signals[group.follower].current != group.follower_phases[committed]:
                wait += self.signals[group.follower].yellow + self.settings.all_red
            result[:] = committed
            result[:min(H, math.ceil(wait / dt))] = -1
            return result, committed
        if first == current:
            return result, current
        begin = min(H, math.ceil(max(dwell, self.settings.minimum_green - (self.last_tick - s.since), 0) / dt))
        end = min(H, begin + math.ceil(clearance / dt))
        result[begin:end] = -1
        result[end:] = first
        if second is not None:
            next_begin = min(H, end + math.ceil(max(span, self.settings.minimum_green) / dt))
            next_end = min(H, next_begin + math.ceil(clearance / dt))
            result[next_begin:next_end] = -1
            result[next_end:] = second
        return result, first if begin == 0 else current

    def _catalogue(self, key):
        s = self.signals[key]
        if s.stage != "green":
            return [self._schedule(key, s.target, 0)]
        # Pruning changes search coverage, not the network objective or dynamics.
        def urgency(phase):
            lanes = {m.incoming for m in s.movements if s.greens[phase][m.index] in "gG"}
            return sum(self.queued.get(l, 0) * (1 + max(self.cohort_waits.get(l, (0,))) / 90)
                       for l in lanes)
        alternatives = sorted((p for p in range(len(s.greens)) if p != s.current), key=lambda p: (-urgency(p), p))[:3]
        remaining = max(0.0, self.settings.maximum_green - (self.last_tick - s.since))
        demanded = [p for p in alternatives if urgency(p) > .05]
        # An expired green with competing service gets an explicit reference
        # transition; it cannot perpetually restart a new fifty-second horizon.
        continuation = self._schedule(key, demanded[0], math.floor(remaining / self.interval) * self.interval) if demanded and remaining < self.interval * self.horizon_steps else self._schedule(key, s.current, 0)
        options = [continuation]
        for phase in alternatives:
            options.append(self._schedule(key, phase, 0))
            if remaining >= 10:
                options.append(self._schedule(key, phase, 10))
        if alternatives:
            options.append(self._schedule(key, alternatives[0], 0, s.current, 10))
        unique = {}
        for schedule, action in options:
            unique.setdefault((tuple(schedule), action), (schedule, action))
        return list(unique.values())[:self.candidate_limit]

    def _permission(self, key, phase, link):
        if phase < 0:
            return 0.0
        signal = self.signals[key]
        mask_key = key, phase
        if mask_key not in self.occupied_cache:
            self.occupied_cache[mask_key] = self.occupied_conflicts(signal, phase, self.raw_lanes)
        if link in self.occupied_cache[mask_key]:
            return 0.0
        ch = signal.greens[phase][link]
        if ch == 'G':
            return 1.0
        if ch != 'g':
            return 0.0
        cache_key = key, phase
        if cache_key not in self.permission_cache:
            self.permission_cache[cache_key] = {m.index for m in self.active_movements(self.signals[key], phase)}
        return .35 if link in self.permission_cache[cache_key] else 0.0

    def _gates(self, schedules):
        H = self.horizon_steps
        phases = {key: list(schedule[0])[:H] for key, schedule in schedules.items()}
        for leader, group in self.groups.items():
            phases[group.follower] = [group.follower_phases[p] if p >= 0 else -1 for p in phases[leader]]
        native = {}
        for key, state in self.native_states.items():
            program = self.topology.program_phases[key]
            phase, remaining = state["phase"], max(0, state["next_switch"] - self.last_tick)
            patterns = []
            for k in range(H):
                point = k * self.interval
                while point >= remaining:
                    phase = (phase + 1) % len(program)
                    remaining += max(.001, program[phase][0])
                patterns.append(program[phase][1])
            native[key] = patterns
        gates = np.ones((H, len(self.movements)))
        for j, (_, _, key, link, _, portal) in enumerate(self.movements):
            if key in phases:
                for k, phase in enumerate(phases[key]):
                    gates[k, j] = self._permission(key, phase, link)
            elif key in native:
                for k in range(H):
                    ch = native[key][k][link]
                    gates[k, j] = 1.0 if ch == "G" else .35 if ch == "g" else 0.0
            if portal:
                receiver, index = portal
                for k, phase in enumerate(phases[receiver]):
                    if (phase < 0 or self.signals[receiver].greens[phase][index] != "G"
                            or self._permission(receiver, phase, index) < 1):
                        gates[k, j] = 0.0
        return gates

    def _changed_gates(self, gates, key, option):
        cache_key = (key, tuple(option[0]))
        cached = self.gate_cache.get(cache_key)
        indices = self.gate_indices[key]
        if cached is None:
            values = np.ones((self.horizon_steps, len(indices)))
            for column, j in enumerate(indices):
                _, _, tid, link, _, portal = self.movements[j]
                for k, phase in enumerate(option[0][:self.horizon_steps]):
                    if tid in self.followers:
                        phase = self.groups[key].follower_phases[phase] if phase >= 0 else -1
                    value = self._permission(tid, phase, link)
                    if portal:
                        receiver, port_link = portal
                        p = option[0][k]
                        target = self.groups[key].follower_phases[p] if p >= 0 else -1
                        if (target < 0 or self.signals[receiver].greens[target][port_link] != 'G'
                                or self._permission(receiver, target, port_link) < 1):
                            value = 0.0
                    values[k, column] = value
            self.gate_cache[cache_key] = values
            cached = values
        result = gates.copy()
        result[:, indices] = cached
        return result

    def _state(self, vehicles):
        N, H, dt = len(self.lane_ids), self.horizon_steps, self.interval
        stock = np.array([self.rows[l]["stock"] for l in self.lane_ids], dtype=float)
        ready = np.zeros(N)
        arrivals = np.zeros((H + 1, N))
        for lane, position, speed, _ in vehicles.values():
            connector = self.topology.connector_exits.get(lane)
            if lane not in self.lane_index:
                if connector is None or connector[0] not in self.lane_index:
                    continue
                road, remaining = connector
                i = self.lane_index[road]
                stock[i] += 1
                eta = (max(0, self.topology.internal_lengths[lane] - position) + remaining + self.topology.lanes[road].length) / max(2.0, speed)
            else:
                i = self.lane_index[lane]
                eta = max(0.0, self.topology.lanes[lane].length - position) / max(2.0, speed)
            if eta < dt:
                ready[i] += 1
            else:
                arrivals[min(H, int(eta / dt)), i] += 1
        # Priors belong to destination ROADS, not each duplicate fan-out
        # connector. Otherwise a three-lane exit invents triple turning demand.
        road_groups = defaultdict(list)
        for j, (a, b, _, _, direction, _) in enumerate(self.movements):
            road_groups[a, self.topology.lanes[b].edge].append(j)
        weights = np.zeros(len(self.movements))
        for indices in road_groups.values():
            counts = [self.movement_counts[self.movements[j][0], self.movements[j][1]] for j in indices]
            direction = self.movements[indices[0]][4]
            prior = 10 if direction == 's' else .1 if direction in 'tT' else 5
            total = sum(counts) + prior
            for j, count in zip(indices, counts):
                weights[j] = total * (count + 1) / (sum(counts) + len(indices))
        totals = np.bincount(self.src, weights=weights, minlength=N)
        shares = weights / np.maximum(1e-9, totals[self.src])
        delay = np.array([max(1, math.ceil(self.topology.lanes[l].length / max(3, self.rows[l]["speed"]) / dt)) for l in self.lane_ids])
        external = np.zeros(N)
        try:
            pending_total = len(self.api.simulation.getPendingVehicles())
        except (AttributeError, RuntimeError):
            pending_total = 0
        for lane in self.boundaries:
            row = self.rows[lane]
            external[self.lane_index[lane]] = self.predictor.predict(self.histories[lane], row["stock"], row["queue"], pending_total / max(1, len(self.boundaries)), self.last_tick)
        # Entry assignment is an estimate from PAST insertions; exact external
        # roster/route assignment is evaluator-only and never enters this model.
        prior = np.zeros(N)
        for lane in self.boundaries:
            prior[self.lane_index[lane]] = 1 + sum(self.histories[lane])
        self.unassigned_pending = pending_total if prior.sum() == 0 else 0
        backlog = pending_total * prior / max(1, prior.sum())
        age = np.array([max(self.cohort_waits.get(l, (0,))) for l in self.lane_ids])
        rates = getattr(self, 'discharge_rate', {})
        prior_rate = getattr(getattr(self, 'settings', None), 'saturation_flow', .45)
        self.rate = np.array([rates.get(lane, prior_rate) * dt for lane in self.lane_ids])
        self.entry_blocked = np.array([self.rows[l].get('entry_gap', self.topology.lanes[l].length) < 6
                                       and self.rows[l]['speed'] < .5 and self.rows[l]['stock'] > 0
                                       for l in self.lane_ids])
        return stock, ready, arrivals, shares, delay, external, backlog, age

    def _rollout(self, state, gates, switches=0, details=False):
        stock, ready, arrivals, shares, delay, external, backlog, age = (x.copy() for x in state)
        N, H, dt = len(stock), self.horizon_steps, self.interval
        # An inherited overfull lane cannot be made to disappear, nor receive
        # additional stock until it drains below its geometric storage limit.
        capacity = np.maximum(self.capacities, stock)
        cost = 2.0 * switches + dt * (H + 2) * getattr(self, 'unassigned_pending', 0)
        residual = 0.0
        blocked = getattr(self, 'entry_blocked', np.zeros(N, dtype=bool))
        for k in range(H):
            before = stock.sum() + backlog.sum()
            ready += arrivals[k]
            # Insertion forecasts are an admission envelope, not new offered
            # demand on top of already observed pending vehicles. Only its
            # excess over existing backlog supplies a persistence lower bound
            # for unseen arrivals. Hidden latent demand stays unidentifiable.
            offered = np.maximum(0, external - backlog)
            backlog += offered
            admitted = np.minimum(np.minimum(backlog, external), np.maximum(0, self.capacities - stock))
            admitted[blocked] = 0
            stock += admitted
            backlog -= admitted
            for i in np.flatnonzero(admitted):
                if k + delay[i] <= H:
                    arrivals[k + delay[i], i] += admitted[i]
            sending = np.minimum(ready, self.rate)
            sink = np.zeros(N)
            sink[self.sinks] = sending[self.sinks]
            probability = np.bincount(self.src, weights=shares * gates[k], minlength=N)
            p = np.clip(probability, 0, 1)
            prefix = np.where(p > 1 - 1e-8, ready,
                              p * (1 - np.power(p, ready)) / np.maximum(1e-8, 1 - p))
            desired = shares * gates[k] * np.minimum(sending, prefix)[self.src] / np.maximum(1e-9, p[self.src])
            if len(self.sinks):
                desired[np.isin(self.src, self.sinks)] = 0
            flow = np.zeros_like(desired)
            # Start at zero and increase feasible flow monotonically. Starting
            # from unconstrained demand and reducing it can leave an upstream
            # admission relying on a downstream release that was just reduced.
            requested = np.bincount(self.dst, weights=desired, minlength=N)
            for _ in range(8):
                outgoing = np.bincount(self.src, weights=flow, minlength=N) + sink
                room = np.maximum(0, self.capacities - stock + outgoing)
                room[blocked] = 0
                factor = np.minimum(1, room / np.maximum(1e-9, requested))
                updated = desired * factor[self.dst]
                if np.max(np.abs(updated - flow), initial=0) < 1e-8:
                    flow = updated
                    break
                flow = updated
            outgoing = np.bincount(self.src, weights=flow, minlength=N) + sink
            incoming = np.bincount(self.dst, weights=flow, minlength=N)
            # Final validation catches a non-converged cyclic supply allocation.
            if np.any(incoming > np.maximum(0, self.capacities - stock + outgoing) + 1e-6):
                return (float("inf"), None) if details else float("inf")
            ready = np.maximum(0, ready - outgoing)
            stock += incoming - outgoing
            for i in np.flatnonzero(incoming):
                if k + delay[i] <= H:
                    arrivals[k + delay[i], i] += incoming[i]
            if np.any(stock < -1e-6) or np.any(stock > capacity + 1e-6):
                return (float("inf"), None) if details else float("inf")
            residual = max(residual, abs(stock.sum() + backlog.sum() + sink.sum() - before - offered.sum()))
            congestion = np.maximum(0, stock - .8 * np.maximum(1, self.capacities))
            cost += dt * (stock.sum() + backlog.sum() + .5 * ready.sum()
                          + 2 * congestion.sum() + .25 * np.dot(ready, np.minimum(3, age / 90)))
        cost += dt * (stock.sum() + 2 * backlog.sum())
        if details:
            return cost, {"stock": stock.tolist(), "mass_residual": residual,
                          "predicted_oldest_wait_slack": float(np.maximum(0, age + H * dt - 90)[ready > .1].max(initial=0))}
        return cost

    def _fallback(self, now):
        for key in self.actuators:
            s = self.signals[key]
            if s.stage != "green" or now - s.since < max(self.settings.minimum_green, s.green_durations[s.current]):
                continue
            if any(self.queued.get(m.incoming, 0) > 0 for m in s.movements):
                self.change(s, (s.current + 1) % self.reference_counts[key], now)
                s.reason = "MPC fallback: advance reference through normal clearance"

    def step(self, now, vehicles=None):
        if self.mode == "fixed":
            return super().step(now, vehicles)
        observations = vehicles if vehicles is not None else self.vehicle_observations()
        self.current_vehicles = observations
        self.observe_transfers(now, observations)
        self.advance(now)
        if now + 1e-6 < self.last_tick + self.interval:
            return
        start = time.perf_counter()
        self.last_tick = now
        self.blocked = set()
        self.observe(now, observations)
        self.gate_cache = {}
        self.permission_cache = {}
        self.occupied_cache = {}
        if any(lane not in self.raw_lanes for lane in self.topology.detectors):
            raise RuntimeError('MPC observation snapshot is incomplete; run cannot be trusted')
        catalogues = {key: self._catalogue(key) for key in self.actuators}
        schedules = {key: values[0] for key, values in catalogues.items()}
        observed_signals = {key: {"stage": signal.stage, "phase": signal.current, "target": signal.target,
                                   "since": signal.since, "until": signal.until if math.isfinite(signal.until) else None}
                            for key, signal in self.signals.items()}
        state = self._state(observations)
        mismatch = False
        if self.last_prediction is not None:
            self.prediction_errors.append(float(np.abs(state[0] - self.last_prediction).mean()))
            mismatch = self.prediction_errors[-1] > 2.0
        gates = self._gates(schedules)
        switch_cost = lambda: sum(np.count_nonzero(np.diff(s[0]) != 0) for s in schedules.values())
        best = self._rollout(state, gates, switch_cost())
        initial = best
        evaluated, searched = 1, set()
        order = list(self.actuators)
        offset = (11 * self.optimization_round) % max(1, len(order))
        order = order[offset:] + order[:offset]
        for _ in range(2):
            changed = False
            for key in order:
                if evaluated >= self.maximum_evaluations or (time.perf_counter() - start) * 1000 >= self.deadline_ms - 30:
                    break
                previous = schedules[key]
                chosen = previous
                for option in catalogues[key]:
                    if evaluated >= self.maximum_evaluations or (time.perf_counter() - start) * 1000 >= self.deadline_ms - 30:
                        break
                    schedules[key] = option
                    trial_gates = self._changed_gates(gates, key, option)
                    switch_count = sum(np.count_nonzero(np.diff(s[0]) != 0) for s in schedules.values())
                    value = self._rollout(state, trial_gates, switch_count)
                    evaluated += 1
                    if value + 1e-6 < best:
                        best, chosen, changed = value, option, True
                schedules[key] = chosen
                gates = self._changed_gates(gates, key, chosen)
                searched.add(key)
            if not changed:
                break
        final, trajectory = self._rollout(state, gates, switch_cost(), details=True)
        elapsed = (time.perf_counter() - start) * 1000
        fallback = mismatch or not np.isfinite(final) or trajectory is None or elapsed > self.deadline_ms - 5
        if fallback:
            self._fallback(now)
            self.last_prediction = None
        else:
            for key, (_, action) in schedules.items():
                s = self.signals[key]
                if s.stage == "green" and action != s.current:
                    self.change(s, action, now)
                    self.decisions += 1
                s.reason = "MPC: chosen future greens reduce predicted network queues and spillback"
            # Only a one-interval prediction is comparable to next observation.
            H = self.horizon_steps
            self.horizon_steps = 1
            _, one = self._rollout(state, gates[:1], details=True)
            self.horizon_steps = H
            self.last_prediction = np.array(one["stock"]) if one else None
            self.last_plan = {key: {"phases": option[0].tolist(), "first_action": option[1]} for key, option in schedules.items()}
        self.elapsed_ms = (time.perf_counter() - start) * 1000
        self.control_evaluations += len(self.actuators)
        self.optimization_round += 1
        self.solver_events.append({"time": now, "elapsed_ms": self.elapsed_ms, "search_ms": elapsed,
                                   "status": "fallback" if fallback else "feasible_local_incumbent",
                                   "fallback": fallback,
                                   "fallback_reason": ("prediction_error_above_2_vehicles_per_lane" if mismatch else "deadline" if elapsed > self.deadline_ms - 5 else "unrealizable_trajectory") if fallback else None,
                                   "predicted_wait_slack_seconds": trajectory["predicted_oldest_wait_slack"] if trajectory else None,
                                   "deadline_exceeded": self.elapsed_ms > self.deadline_ms, "evaluated": evaluated, "actuators_searched": len(searched),
                                   "initial_objective": initial if np.isfinite(initial) else None,
                                   "objective": final if np.isfinite(final) else None,
                                   "mass_residual": trajectory["mass_residual"] if trajectory else None,
                                   "unassigned_pending": self.unassigned_pending,
                                   "observed_signals": observed_signals,
                                   "planned_schedules": {key: {"phases": option[0].tolist(), "first_action": option[1]} for key, option in schedules.items()},
                                   "applied_targets": {key: signal.target if signal.stage in {"yellow", "all_red"} else signal.current for key, signal in self.signals.items()}})
        self.shared_state["mpc"] = dict(self.solver_events[-1], schedules=self.last_plan)

    def status(self):
        result = super().status()
        if hasattr(self, "solver_events"):
            result.update(controller="network_mpc", optimizer="bounded_network_schedule_coordinate_search",
                          solver=self.solver_events[-1] if self.solver_events else None,
                          prediction_mae=float(np.mean(self.prediction_errors)) if self.prediction_errors else None,
                          predictor=self.predictor.metadata,
                          forecast_target="observed boundary insertions; censored by entry congestion")
        return result
