"""Opt-in simulation heuristic: finite storage, arrivals, and switching loss.

Retains imported SUMO G/g semantics, not real-world signal certification.
Capacity-aware pressure is motivated by Gregoire et al. (arXiv:1309.6484);
uses their equation 5 with exponent 4 and a common network capacity scale.
Moving-demand motivation: Zhang et al. (arXiv:2112.10107). This forecast/age
heuristic does not inherit either paper's performance or stability guarantees.
"""
from collections import defaultdict
from dataclasses import replace
import time

from adaptive_controller import AdaptiveController, Settings, MODES as ADAPTIVE_MODES, head_prefix, usable_supply
from signal_groups import build_portal_groups


def capacity_pressure(queue, capacity, reference):
    capacity = max(1.0, capacity)
    reference = max(capacity, reference)
    occupancy = min(1.0, max(0.0, queue) / capacity)
    normalized = (queue / reference + (2.0 - capacity / reference) * occupancy ** 4) / (1.0 + occupancy ** 3)
    return reference * min(1.0, max(0.0, normalized))


def relief_phases(signal, conflicts):
    states = []
    masks = {tuple(index for index, state in enumerate(green) if state in "gG") for green in signal.greens}
    for green in signal.greens:
        for target in signal.movements:
            if green[target.index] != "g" or target.direction in ("t", "T"):
                continue
            state = ["r"] * len(green)
            for movement in signal.movements:
                if (green[movement.index] in "gG" and
                        (movement.incoming == target.incoming or not conflicts(target.index, movement.index))):
                    state[movement.index] = green[movement.index]
            mask = tuple(index for index, value in enumerate(state) if value in "gG")
            if mask not in masks:
                states.append("".join(state))
                masks.add(mask)
    return tuple(states)


MODES = ADAPTIVE_MODES | {"max_pressure"}


class IntelligentCoordinator(AdaptiveController):
    def __init__(self, api, network, mode="coordinated", settings=Settings()):
        if mode not in MODES:
            raise ValueError("Unknown signal controller")
        super().__init__(api, network, "fixed" if mode == "max_pressure" else mode,
                         replace(settings, imported_actions=True, entire_network=True))
        self.mode = mode
        self.reference_counts = {key: len(signal.greens) for key, signal in self.signals.items()}
        for key, signal in self.signals.items():
            extra = relief_phases(signal, self.topology.conflicts[key])
            signal.greens += extra
            signal.green_durations += (self.settings.minimum_green,) * len(extra)
        self.groups = build_portal_groups(self.topology, self.reference_counts, self.settings)
        self.followers = {group.follower: key for key, group in self.groups.items()}
        self.current_vehicles = {}
        for key, group in self.groups.items():
            for signal, phase in ((self.signals[key], 0), (self.signals[group.follower], group.follower_phases[0])):
                signal.current = signal.target = phase
                signal.stage, signal.since = "green", api.simulation.getTime()
                api.trafficlight.setRedYellowGreenState(signal.id, signal.greens[phase])
            for lane in group.cells:
                api.lane.subscribe(lane, (self.constants.LAST_STEP_VEHICLE_NUMBER, self.constants.LAST_STEP_MEAN_SPEED,
                                         self.constants.LAST_STEP_VEHICLE_HALTING_NUMBER, self.constants.LAST_STEP_VEHICLE_ID_LIST))
        self.control_evaluations = 0
        self.waiting_since = {}
        self.cohort_waits = {}
        self.waiting_heads = {}
        self.phase_failures = {}
        self.last_departure = {}
        self.observed_departures = defaultdict(int)
        self.shared_state = {}
        self.native_states = {}
        self.native_arrivals = {}
        self.native_waiting = set()
        self.native_early_clearances = 0
        self.simulation_step = api.simulation.getDeltaT()
        self.native_ids = set(api.trafficlight.getIDList()) - self.signals.keys()
        self.native_owners = {movement.incoming: key for key in self.native_ids
                              for movement in self.topology.network_movements.get(key, ())}
        constants = self.constants
        for key in self.native_ids:
            api.trafficlight.subscribe(key, (constants.TL_RED_YELLOW_GREEN_STATE,
                                            constants.TL_CURRENT_PHASE, constants.TL_NEXT_SWITCH))
        self.unsignalized = {junction: {"type": kind, "incoming": tuple(lane for lane, meta in self.topology.lanes.items()
                                                                      if self.topology.edges[meta.edge][1] == junction)}
                            for junction, kind in self.topology.node_types.items()
                            if kind in {"priority", "priority_stop", "allway_stop", "right_before_left"}}
        reservoirs = (sum(self.topology.lanes[lane].length for lane in lanes) / self.settings.storage_per_vehicle
                      for lanes in self.topology.receiving_groups.values())
        self.pressure_scale = max(1.0, *self.topology.capacities.values(), *reservoirs)

    def observe_transfers(self, now, vehicles):
        previous = dict(self.flow_since_tick)
        super().observe_transfers(now, vehicles)
        for lane, count in self.flow_since_tick.items():
            transferred = count - previous.get(lane, 0)
            if transferred:
                self.last_departure[lane] = now
                self.observed_departures[lane] += transferred

    def change(self, signal, target, now):
        if signal.id in self.followers:
            return
        if self.mode == "fixed":
            target %= self.reference_counts[signal.id]
        super().change(signal, target, now)

    def portal_clear(self, group):
        readings = self.api.lane.getAllSubscriptionResults()
        if any(lane not in readings or readings[lane].get(self.constants.LAST_STEP_VEHICLE_NUMBER, 1) for lane in group.cells):
            return False
        return not any(lane in group.exits and position < length + 1
                       for lane, position, _, length in self.current_vehicles.values())

    def advance(self, now):
        held = {}
        for key, group in self.groups.items():
            leader, follower = self.signals[key], self.signals[group.follower]
            if leader.stage != "all_red":
                continue
            target = group.follower_phases[leader.target]
            if follower.stage == "green" and follower.current != target and self.portal_clear(group):
                super().change(follower, target, now)
            if follower.stage != "green" or follower.current != target:
                held[key], leader.until = leader.until, float("inf")
                leader.reason = "Joint clearance: wait for the receiving green and empty connection"
        super().advance(now)
        for key, until in held.items():
            self.signals[key].until = until

    def transition_time(self, signal, phase):
        if phase == signal.current:
            return 0.0
        delay = signal.yellow + self.settings.all_red
        group = getattr(self, "groups", {}).get(signal.id)
        if group and group.follower_phases[phase] != self.signals[group.follower].current:
            delay += self.signals[group.follower].yellow
        return delay

    def active_movements(self, signal, phase):
        green = signal.greens[phase]
        conflicts = self.topology.conflicts[signal.id]
        priority = [movement for movement in signal.movements if green[movement.index] == "G"
                    and (self.queued.get(movement.incoming, 0) > 0 or sum(self.forecasts.get(movement.incoming, ())[:2]) > .5)]
        return [movement for movement in signal.movements if green[movement.index] in "gG"
                and (green[movement.index] == "G" or not any(
                    other.incoming != movement.incoming and conflicts(movement.index, other.index) for other in priority))]

    def observe(self, now, vehicles):
        previous_rates = dict(self.discharge_rate)
        previous_queues = dict(self.queued)
        previous_ready = set(self.ready_since)
        super().observe(now, vehicles)
        for lane in self.topology.incoming:
            signal = self.signals[self.topology.owner[lane]]
            saturated = (previous_queues.get(lane, 0) >= self.settings.saturation_flow * self.settings.interval
                         and lane in previous_ready and self.learning_eligible.get(lane, False)
                         and signal.stage == "green" and now - signal.since >= self.settings.interval)
            if not saturated:
                self.discharge_rate[lane] = previous_rates.get(lane, self.settings.saturation_flow)
        constants = self.constants
        self.native_states = {key: {"state": row[constants.TL_RED_YELLOW_GREEN_STATE],
                                    "phase": row[constants.TL_CURRENT_PHASE], "next_switch": row[constants.TL_NEXT_SWITCH]}
                              for key, row in self.api.trafficlight.getAllSubscriptionResults().items() if key in self.native_ids}
        waiting, cohorts = {}, defaultdict(list)
        native_arrivals, native_waiting = defaultdict(list), set()
        for vehicle, (lane, position, speed, length) in vehicles.items():
            road, connector_distance = self.topology.connector_exits.get(lane, (lane, 0))
            native = self.native_owners.get(road)
            if native is not None:
                distance = self.topology.lanes[road].length - position
                if lane in self.topology.internal_lengths:
                    distance += connector_distance + self.topology.internal_lengths[lane]
                if speed < .1:
                    native_waiting.add(native)
                else:
                    native_arrivals[native].append(max(0.0, distance) / max(2.0, speed))
            incoming = self.topology.cell_owner.get(road)
            if incoming is None:
                continue
            previous = self.waiting_since.get(vehicle)
            same_queue = previous is not None and (previous[0] == incoming or
                         self.topology.owner.get(incoming) is not None and
                         self.topology.owner.get(previous[0]) == self.topology.owner[incoming])
            if speed >= .1 and not same_queue:
                continue
            since = previous[1] if same_queue else now
            waiting[vehicle] = (incoming, since)
            path = self.topology.approaches[incoming]
            distance = sum(self.topology.lanes[cell].length for cell in path[path.index(road):])
            if lane in self.topology.internal_lengths:
                distance += connector_distance + self.topology.internal_lengths[lane] - position
            else:
                distance -= position
            cohorts[incoming].append((distance, max(0.0, now - since), vehicle))
        self.waiting_since = waiting
        self.cohort_waits = {lane: [age for _, age, _ in sorted(values)] for lane, values in cohorts.items()}
        self.waiting_heads = {lane: min(values)[2] for lane, values in cohorts.items()}
        self.native_arrivals, self.native_waiting = dict(native_arrivals), native_waiting
        self.shared_state = {
            "time": now,
            "signals": {key: {"stage": signal.stage, "phase": signal.current, "target": signal.target,
                              "green_elapsed": max(0, now - signal.since) if signal.stage == "green" else 0,
                              "state": signal.greens[signal.current] if signal.stage == "green" else
                                       "r" * len(signal.greens[0]) if signal.stage == "all_red" else
                                       "".join("y" if state in "gG" else "r" for state in signal.greens[signal.current])}
                        for key, signal in self.signals.items()},
            "lanes": {lane: {"vehicles": row["stock"], "stopped": row["queue"], "speed": row["speed"],
                             "entry_gap": row["entry_gap"]} for lane, row in self.rows.items()},
            "approaches": {lane: {"queue": self.queued[lane], "arrivals": tuple(self.forecasts[lane]),
                                   "discharge_rate": self.discharge_rate[lane],
                                   "waiting_cohorts": tuple(self.cohort_waits.get(lane, ())),
                                   "last_departure": self.last_departure.get(lane)} for lane in self.topology.incoming},
            "unsignalized": {junction: {"type": node["type"],
                                         "queued": sum(self.rows[lane]["queue"] for lane in node["incoming"]),
                                         "vehicles": sum(self.rows[lane]["stock"] for lane in node["incoming"])}
                             for junction, node in self.unsignalized.items()},
        }
        self.shared_state["signals"].update({key: dict(value, stage="native") for key, value in self.native_states.items()})

    def plan_native_clearance(self, now):
        proposals = {}
        for key, arrivals in self.native_arrivals.items():
            if key in self.native_waiting or key not in self.native_states:
                continue
            state = self.native_states[key]
            phases = self.topology.program_phases[key]
            phase = state["phase"]
            duration, green = phases[phase]
            if "y" in green.lower() or "g" not in green.lower():
                continue
            remaining = state["next_switch"] - now
            if not self.settings.interval <= remaining <= self.settings.horizon or duration - remaining < self.settings.minimum_green:
                continue
            clearance = 0.0
            for offset in range(1, len(phases)):
                span, pattern = phases[(phase + offset) % len(phases)]
                if "g" in pattern.lower():
                    break
                clearance += span
            if clearance <= 0 or min(arrivals) < clearance + 2:
                continue
            if any(remaining <= arrival <= remaining + clearance for arrival in arrivals):
                proposals[key] = now + self.simulation_step
        return proposals

    def green_budget(self, signal, phase):
        if signal.id in getattr(self, "followers", {}):
            return self.settings.horizon
        nominal = signal.green_durations[phase] if signal.green_durations else self.settings.maximum_green
        maximum = nominal if self.mode == "fixed" else min(self.settings.maximum_green, nominal * 1.5)
        elapsed = max(0, self.last_tick - signal.since) if phase == signal.current and signal.stage == "green" else 0.0
        return max(0.0, maximum - elapsed)

    def receiving(self, outgoing, plans):
        stock, capacity, supply, release = super().receiving(outgoing, plans)
        path = self.topology.receivers[outgoing]
        controlled = self.topology.owner.get(path[-1])
        if controlled in self.signals:
            signal = self.signals[controlled]
            if self.known_stalled(signal, plans.get(controlled, signal.current)):
                entry = self.rows[path[0]]
                supply = usable_supply(stock, capacity, entry["entry_gap"], entry["speed"], 0, 0, self.settings.forecast_margin)
                return stock, capacity, supply if capacity >= 1 else 0.0, 0.0
        owner = self.native_owners.get(path[-1])
        if owner is None:
            return stock, capacity, supply, release
        state = self.native_states.get(owner)
        if state is None:
            return stock, capacity, 0.0, 0.0
        phases = self.topology.program_phases[owner]
        links = {movement.index for movement in self.topology.network_movements[owner] if movement.incoming == path[-1]}
        remaining, available = self.settings.horizon, 0.0
        phase = state["phase"]
        duration = max(0.0, state["next_switch"] - self.last_tick)
        while remaining > 1e-6:
            span = min(remaining, duration)
            if any(phases[phase][1][index] in "gG" for index in links):
                available += span
            remaining -= span
            phase = (phase + 1) % len(phases)
            duration = max(.001, phases[phase][0])
        entry = self.rows[path[0]]
        supply = usable_supply(stock, capacity, entry["entry_gap"], entry["speed"], release, available, self.settings.forecast_margin)
        if capacity < 1:
            supply = max(0, release * available - stock)
        if supply < 1:
            self.blocked.add(outgoing)
        return stock, capacity, supply, release * available / self.settings.horizon

    def reserve_receiving_space(self, plans):
        available = {key: phase for key, phase in plans.items() if not self.known_stalled(self.signals[key], phase)}
        return super().reserve_receiving_space(available)

    def propagated_arrivals(self, plans):
        available = {key: phase for key, phase in plans.items() if not self.known_stalled(self.signals[key], phase)}
        return super().propagated_arrivals(available)

    def phase_demand(self, signal, phase):
        if self.known_stalled(signal, phase):
            return 0.0
        demand = 0.0
        by_lane = defaultdict(dict)
        for movement in self.active_movements(signal, phase):
            state = signal.greens[phase][movement.index]
            if state in "gG":
                by_lane[movement.incoming][movement.outgoing] = 1.0 if state == "G" else .5
        for lane, destinations in by_lane.items():
            arriving = 0.0
            if self.mode == "coordinated" and self.settings.predictive:
                buckets = max(1, int(10 / self.settings.interval))
                arriving = sum(self.forecasts[lane][:buckets]) + sum(self.neighbour_arrivals.get(lane, ())[:buckets])
            share = min(1.0, sum(self.turn_share(lane, outgoing) * weight for outgoing, weight in destinations.items()))
            demand += (self.queued[lane] + arriving) * share
        return demand

    def phase_wait(self, signal, phase):
        service = self.phase_service.get((signal.id, phase), {})
        ages = [age for lane, count in service.items() if count > .05
                for age in self.cohort_waits.get(lane, ())[:max(1, int(count))]]
        return max(ages, default=0.0)

    def service_priority(self, signal, phase):
        weighted, discharge_time = 0.0, self.settings.minimum_green
        if phase == signal.current and signal.stage == "green":
            discharge_time = max(self.settings.interval, self.settings.minimum_green - (self.last_tick - signal.since))
        for lane, count in self.phase_service.get((signal.id, phase), {}).items():
            weighted += count
            for index, age in enumerate(self.cohort_waits.get(lane, ())):
                served = min(1.0, max(0.0, count - index))
                weighted += served * age / self.settings.maximum_service_gap
            discharge_time = max(discharge_time, count / max(.05, self.discharge_rate[lane]))
        clearance = self.transition_time(signal, phase)
        if phase != signal.current and sum(self.phase_service.get((signal.id, signal.current), {}).values()) > .05:
            clearance *= 2
        return weighted / (discharge_time + clearance)

    def head_signature(self, signal, phase):
        lanes = {movement.incoming for movement in signal.movements
                 if signal.greens[phase][movement.index] in "gG"}
        heads = getattr(self, "waiting_heads", {})
        return tuple(sorted((lane, heads[lane]) for lane in lanes if lane in heads and lane in self.ready_since))

    def known_stalled(self, signal, phase):
        previous = getattr(self, "phase_failures", {}).get((signal.id, phase))
        return (previous is not None and self.last_tick - previous[1] < self.settings.horizon
                and bool(previous[0]) and previous[0] == self.head_signature(signal, phase))

    def turn_share(self, incoming, outgoing):
        probability = super().turn_share(incoming, outgoing)
        destinations = {destination for destination in self.topology.turn_options[incoming]
                        if self.topology.lanes[destination].edge == self.topology.lanes[outgoing].edge}
        return probability / max(1, len(destinations))

    def score(self, signal, phase, plans):
        if self.known_stalled(signal, phase):
            self.phase_service[signal.id, phase] = {}
            return 0.0, 0.0
        cfg = self.settings
        group = getattr(self, "groups", {}).get(signal.id)
        if group:
            plans = dict(plans, **{group.follower: group.follower_phases[phase]})
        green = signal.greens[phase]
        clearance = self.transition_time(signal, phase)
        duration = min(max(0.0, cfg.horizon - clearance), self.green_budget(signal, phase))
        by_lane = defaultdict(list)
        resources, budgets = {}, {}
        for movement in self.active_movements(signal, phase):
            outgoing = movement.outgoing
            group = self.topology.receivers[outgoing][-1]
            if outgoing not in resources:
                resources[outgoing] = self.receiving(outgoing, plans)
            stock, capacity, supply, release = resources[outgoing]
            others = sum(amount for owner, amount in self.reservations.get(group, {}).items()
                         if owner != signal.id)
            budgets[group] = max(budgets.get(group, 0.0), max(0.0, supply - others))
            by_lane[movement.incoming].append(movement)
        requests, totals = [], defaultdict(float)
        for lane, movements in by_lane.items():
            queue = self.queued[lane]
            demand = float(queue)
            if cfg.predictive and self.mode == "coordinated":
                arrivals = self.forecasts[lane]
                neighbours = self.neighbour_arrivals.get(lane, ())
                for bucket, arriving in enumerate(arrivals):
                    eta = (bucket + .5) * cfg.interval
                    predicted = arriving + (neighbours[bucket] if bucket < len(neighbours) else 0.0)
                    demand += predicted * max(0.0, (clearance + duration - max(clearance, eta)) / max(1e-9, duration))
            shares = {movement.outgoing: self.turn_share(lane, movement.outgoing) for movement in movements}
            probability = min(1.0, sum(shares.values()))
            possible = min(head_prefix(demand, probability), self.discharge_rate[lane] * duration)
            upstream = capacity_pressure(demand, self.topology.capacities[lane], self.pressure_scale)
            waits = getattr(self, "cohort_waits", {}).get(lane, ())
            age = sum(min(3.0, wait / cfg.maximum_service_gap) for wait in waits) / max(1, len(waits))
            for outgoing, share in shares.items():
                group = self.topology.receivers[outgoing][-1]
                stock, downstream_capacity, supply, release = resources[outgoing]
                downstream = capacity_pressure(stock, downstream_capacity, self.pressure_scale)
                pressure = upstream - downstream + age
                requested = possible * share / max(1e-9, probability)
                if supply <= 0:
                    requested = 0.0
                requests.append((lane, group, requested, pressure))
                totals[group] += requested
        benefit, served = 0.0, defaultdict(float)
        for lane, group, requested, pressure in requests:
            allocation = requested * min(1.0, budgets[group] / max(1e-9, totals[group]))
            served[lane] += allocation
            benefit += allocation * pressure
        self.phase_service[signal.id, phase] = dict(served)
        return -benefit, sum(served.values())

    def select_phase(self, signal, scores, now):
        if signal.stage != "green":
            return signal.target, signal.reason
        if now - signal.since < self.settings.minimum_green:
            return signal.current, "Complete minimum green"
        demand = [self.phase_demand(signal, phase) for phase in range(len(signal.greens))]
        alternatives = [(signal.current + offset) % len(signal.greens) for offset in range(1, len(signal.greens))]
        called = [phase for phase in alternatives if demand[phase] > .05]
        serviceable = [phase for phase in called if scores[phase][1] > .05]
        if serviceable:
            called = serviceable
        if not called:
            return signal.current, "Resting green: no one is waiting on red"
        current_lanes = {movement.incoming for movement in signal.movements
                         if signal.greens[signal.current][movement.index] in "gG"}
        recent_departure = max((self.last_departure.get(lane, signal.since) for lane in current_lanes), default=signal.since)
        ready = [self.ready_since[lane] for lane in current_lanes if lane in self.ready_since]
        stalled = bool(ready) and now - max(signal.since, recent_departure, min(ready)) >= self.settings.stall_seconds
        if stalled:
            self.phase_failures[signal.id, signal.current] = (self.head_signature(signal, signal.current), now)
        elapsed = now - signal.since
        nominal = signal.green_durations[signal.current]
        current_wait = self.phase_wait(signal, signal.current)
        competing_wait = max(self.phase_wait(signal, phase) for phase in called)
        overdue = competing_wait >= self.settings.maximum_service_gap and competing_wait > current_wait + self.settings.interval
        priorities = {phase: self.service_priority(signal, phase) for phase in called + [signal.current]}
        extending = (not stalled and demand[signal.current] > .05 and not overdue
                     and current_wait + nominal >= competing_wait
                     and priorities[signal.current] > max(priorities[phase] for phase in called))
        limit = min(self.settings.maximum_green, nominal * (1.5 if extending else 1.0))
        lost = signal.yellow + self.settings.all_red
        amortized = elapsed >= max(self.settings.minimum_green, 2 * lost)
        earned = elapsed >= self.settings.lost_time_amortization * lost
        if demand[signal.current] < .05 or stalled or (earned and (elapsed >= limit or (overdue and amortized))):
            reason = ("Advance: green stopped producing departures" if stalled else
                      "Advance: observed demand cleared" if demand[signal.current] < .05 else
                      "Advance: an older waiting cohort needs service" if overdue else
                      "Advance: allocated green completed")
            target = max(called, key=lambda phase: (priorities[phase], -scores[phase][0]))
            return target, reason
        return signal.current, "Extend: observed departures and pressure justify service" if extending else "Serve current arrival cohort"

    def max_pressure(self, now):
        """Plain max pressure (Varaiya 2013) on the same phases and clearances.

        Phase weight: sum over its green movements of turn share x queue on the
        incoming approach minus the queue on the receiving approach. Decided every
        interval after the minimum green; no prediction, coordination, or caps.
        """
        for key, signal in self.signals.items():
            if key in self.followers or signal.stage != "green" or now - signal.since < self.settings.minimum_green:
                continue
            weights = []
            for phase, green in enumerate(signal.greens):
                weight = 0.0
                for movement in signal.movements:
                    if green[movement.index] in "gG":
                        downstream = self.topology.cell_owner.get(movement.outgoing)
                        weight += (self.turn_share(movement.incoming, movement.outgoing) * self.queued.get(movement.incoming, 0)
                                   - self.queued.get(downstream, 0) * self.turn_share(movement.incoming, movement.outgoing))
                weights.append(weight)
            best = max(range(len(weights)), key=lambda phase: weights[phase])
            signal.scores = tuple(round(-w, 2) for w in weights)
            if weights[best] > weights[signal.current] + 1e-9:
                signal.reason = "Max pressure: serve the largest queue difference"
                self.change(signal, best, now)
                self.decisions += 1
            else:
                signal.reason = "Max pressure: current phase has the largest queue difference"

    def step(self, now, vehicles=None):
        observations = vehicles if vehicles is not None else self.vehicle_observations()
        self.current_vehicles = observations
        if self.mode == "max_pressure":
            self.observe_transfers(now, observations)
            self.advance(now)
            if now + 1e-6 >= self.last_tick + self.settings.interval:
                self.last_tick = now
                self.blocked = set()
                self.observe(now, observations)
                # A 10 s decision step: benchmarks show 5 s max pressure loses
                # mostly to switching loss, not to its pressure rule.
                if now + 1e-6 >= getattr(self, "last_pressure_decision", -1e9) + 10:
                    self.last_pressure_decision = now
                    self.max_pressure(now)
                self.control_evaluations += len(self.signals)
            return
        if self.mode == "fixed":
            previous_tick = self.last_tick
            super().step(now, observations)
            if self.last_tick != previous_tick:
                self.control_evaluations += len(self.signals)
            return
        self.observe_transfers(now, observations)
        self.advance(now)
        if now + 1e-6 < self.last_tick + self.settings.interval:
            return
        started = time.perf_counter()
        self.last_tick = now
        self.blocked = set()
        self.observe(now, observations)
        native_proposals = self.plan_native_clearance(now) if self.mode == "coordinated" else {}
        for key, switch_at in native_proposals.items():
            self.native_states[key]["next_switch"] = switch_at
        plans = {key: signal.target if signal.stage in {"yellow", "all_red"} else signal.current
                 for key, signal in self.signals.items()}
        for _ in range(2 if self.mode == "coordinated" else 1):
            self.reservations = self.reserve_receiving_space(plans) if self.mode == "coordinated" else {}
            self.neighbour_arrivals = self.propagated_arrivals(plans) if self.mode == "coordinated" else {}
            next_plans = {}
            for key, signal in self.signals.items():
                if key in self.followers:
                    continue
                scores = [self.score(signal, phase, plans) for phase in range(len(signal.greens))]
                selected, signal.reason = self.select_phase(signal, scores, now)
                signal.scores = tuple(round(score[0], 2) for score in scores)
                signal.predicted_service = round(scores[selected][1], 2)
                next_plans[key] = selected
            for key, group in self.groups.items():
                next_plans[group.follower] = group.follower_phases[next_plans[key]]
            plans = next_plans
        self.control_evaluations += len(self.signals)
        self.plans = plans
        self.shared_state["planned_phases"] = dict(plans)
        self.shared_state["native_clearance_proposals"] = native_proposals
        self.shared_state["reservations"] = {group: dict(grants) for group, grants in self.reservations.items()}
        self.shared_state["decisions"] = {key: {"scores": signal.scores, "predicted_service": signal.predicted_service,
                                               "reason": signal.reason} for key, signal in self.signals.items()}
        self.shared_state["failed_service_observations"] = [
            {"signal": key, "phase": phase, "heads": heads, "observed_at": observed}
            for (key, phase), (heads, observed) in self.phase_failures.items() if now - observed < self.settings.horizon]
        for key, target in plans.items():
            if key in self.followers:
                continue
            signal = self.signals[key]
            if signal.stage == "green" and target != signal.current:
                self.change(signal, target, now)
                self.decisions += 1
        for key in native_proposals:
            self.api.trafficlight.setPhaseDuration(key, 0)
            self.native_early_clearances += 1
        if self.mode == "coordinated":
            self.messages += 2 * sum(len(self.topology.graph[key] & self.topology.scope) for key in self.signals)
        self.elapsed_ms = (time.perf_counter() - started) * 1000

    def status(self):
        status = super().status()
        for key, signal in self.signals.items():
            lanes = {movement.incoming for movement in signal.movements}
            ages = [age for lane in lanes for age in self.cohort_waits.get(lane, ())]
            status["signals"][key].update(oldest_observed_wait_seconds=round(max(ages, default=0), 1),
                                         mean_observed_wait_seconds=round(sum(ages) / max(1, len(ages)), 1),
                                         tracked_waiting_vehicles=len(ages),
                                         observed_departures=sum(self.observed_departures[lane] for lane in lanes))
        status.update(controller="intelligent", control_evaluations=self.control_evaluations,
                      phase_switches=sum(signal.switches for signal in self.signals.values()),
                      oldest_observed_wait_seconds=round(max((age for ages in self.cohort_waits.values() for age in ages), default=0), 1),
                      observed_lanes=len(self.rows), shared_signal_states=len(self.signals) + len(self.native_states),
                      native_early_clearances=self.native_early_clearances,
                      joint_signal_groups={key: group.follower for key, group in self.groups.items()},
                      yielding_relief_phases=sum(len(signal.greens) - self.reference_counts[key] for key, signal in self.signals.items()),
                      stop_controlled_junctions=sum(node["type"] in {"priority_stop", "allway_stop"} for node in self.unsignalized.values()),
                      horizon=self.settings.horizon if self.mode != "fixed" else 0)
        return status
