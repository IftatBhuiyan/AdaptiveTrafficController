"""Plan-anchored adaptive control: run the fixed-time plan, adjust it only for a reason.

Our controller comparison (baseline-v2/CONTROLLERS.md) showed that the
Webster fixed-time plan beats controllers that re-choose the phase every 5 s:
they switch more often and give green time to the wrong phases, and forcing
longer greens everywhere made it worse. So this controller does not replace
the plan. SUMO keeps running the plan itself (same phases, green times,
yellow, all-red and offsets), and the controller only shortens, lengthens or
skips a green, in these situations:

1. Rest: nobody is waiting on red, so the current green stays on instead of
   cycling through empty phases.
2. Gap-out: the green has no queue and no car arriving within 10 s, while
   cars wait on red. The green ends early (after at least half its plan).
   Not on short links between two close junctions (for example 31st St
   between Astoria Blvd N and the junction under I-278, room for 3-4 cars):
   their cars come in platoons from the signal just upstream, and ending the
   green between platoons blocked that signal and locked the interchange.
3. Blocked exit: every car the green could serve is held because the road it
   goes to is full (information from the next signal's approach). The green
   ends early; the time helps the other approaches instead.
4. Spillback guard: a queue on green fills 85% of its approach, so it will
   soon block the junction behind it, while every queue on red is under 50%.
   The green is extended (up to 50% more, at most 30 s). The opposite case (a
   red queue about to spill back, the green queue small) ends the green after
   70% of its plan, but not while the green still moves cars off a short
   link: a short link spills back after 3-4 cars, so a "small" queue there
   is already the bigger risk (cutting those greens locked seed 42 at 5:15).
5. Skip: a green that starts with nobody waiting and nobody arriving, while
   cars wait elsewhere, is skipped (again, not on short links).

Signals joined to another signal by a road of fewer than 4 cars (the
junction under I-278, Astoria Blvd / 31st St N and S, and the signal next to
Astoria Blvd / 33rd St) keep the plan untouched: the plan's offsets time them
together, and adjusting any one of them locked the interchange in testing.

What each signal uses: its own approach queues and arriving cars (like loop
detectors), and how full the roads after it are (shared by the next signal).

Usage (see baseline-v2/compare_controllers.py, --policy anchored):
    controller = AnchoredController(traci, "baseline-v2-webster.net.xml", mode="fixed")
    ...  # each simulation step:
    controller.mode = "anchored"   # from 4:00 PM; "fixed" leaves the plan untouched
    controller.step(now)
The network file must contain the fixed-time plan (programID "0"), because
the controller reads the planned green times from it, and every departed
vehicle must be subscribed to VAR_SPEED and VAR_LANEPOSITION.
"""
from collections import defaultdict
from dataclasses import replace

from adaptive_controller import Settings, Topology

TICK = 2.0                 # seconds between decisions
MINIMUM_GREEN = 5.0        # never end a green earlier than this (or its plan, if shorter)
GAP_SECONDS = 10.0         # gap-out needs no car arriving within this time
GAP_OUT_SHARE = .5         # ... and at least this share of the planned green served
SPILLBACK = .85            # queue fills this share of an approach: about to reach the junction behind it
COMFORTABLE = .5           # queue under this share: no spillback risk soon
MINIMUM_QUEUE = 4          # ignore smaller queues on normal approaches (a few cars are not a spillback risk)
SHORT_LINK = 8.0           # approaches holding fewer cars than this are short links between two close
                           # junctions: any queue there counts, and their greens never gap out or get skipped,
                           # because their cars come in platoons from the signal just upstream
EXTENSION_SHARE = .5       # a spillback extension adds at most this share of the planned green
EXTENSION_SECONDS = 30.0   # ... and at most this many seconds
CUT_SHARE = .7             # a green is cut for a spilling red queue only after this share of its plan
BLOCKED_TICKS = 2          # exits must be full for this many decisions in a row
TIGHT_LINK = 4.0           # a signal fed by a road this short (in cars) straight from another signal
                           # keeps the plan exactly: the two are timed together by the plan's offsets


def is_green(state):
    return "y" not in state.lower() and any(ch in "gG" for ch in state)


class AnchoredController:
    def __init__(self, api, network, mode="anchored", settings=Settings()):
        self.api, self.mode = api, mode
        self.settings = replace(settings, imported_actions=True, entire_network=True)
        self.topology = topo = Topology(network, self.settings)
        self.constants = c = __import__("traci.constants", fromlist=["constants"])
        self.ids = sorted(topo.signals)
        self.phases = {key: topo.program_phases[key] for key in self.ids}
        self.movements = {key: topo.network_movements[key] for key in self.ids}
        self.plan_locked = self.tightly_linked(topo)
        self.exit_of = {m.outgoing: topo.receivers[m.outgoing][-1] for key in self.ids for m in self.movements[key]}
        lanes = {lane for key in self.ids for m in self.movements[key] for lane in topo.approaches.get(m.incoming, (m.incoming,))}
        lanes |= {lane for end in self.exit_of.values() for lane in topo.receiving_groups.get(end, (end,))}
        for lane in lanes:
            api.lane.subscribe(lane, (c.LAST_STEP_VEHICLE_NUMBER, c.LAST_STEP_VEHICLE_HALTING_NUMBER,
                                      c.LAST_STEP_VEHICLE_ID_LIST))
        for key in self.ids:
            api.trafficlight.subscribe(key, (c.TL_CURRENT_PHASE, c.TL_NEXT_SWITCH))
        self.phase, self.started = {}, {}
        self.blocked_ticks = defaultdict(int)
        self.last_tick = -1e9
        self.queue, self.arriving, self.room, self.occupancy = {}, {}, {}, {}
        self.adjustments = defaultdict(int)
        self.by_signal = defaultdict(lambda: defaultdict(int))
        self.reasons = {}
        self.decisions = 0

    def tightly_linked(self, topo):
        """Signals fed by a road of fewer than TIGHT_LINK cars that starts at another signal.

        On baseline-v2 these are the junction under I-278, Astoria Blvd / 31st St
        N and S, and the signal next to Astoria Blvd / 33rd St. Their roads in
        between hold 3-4 cars, so any change to one green shifts the platoons the
        next signal expects; adjusting them locked seeds 11 and 42 even with the
        short-link rules below. They run the plan untouched.
        """
        signal_at = {topo.edges[topo.lanes[m.incoming].edge][1]: key
                     for key in self.ids for m in self.movements[key] if m.incoming in topo.lanes}
        locked = set()
        for key in self.ids:
            for m in self.movements[key]:
                path = topo.approaches.get(m.incoming, (m.incoming,))
                if topo.capacities.get(m.incoming, TIGHT_LINK) >= TIGHT_LINK or path[0] not in topo.lanes:
                    continue
                upstream = signal_at.get(topo.edges[topo.lanes[path[0]].edge][0])
                if upstream and upstream != key:
                    locked.add(key)
        return locked

    # --- observations -----------------------------------------------------------
    def observe(self):
        topo, c = self.topology, self.constants
        lanes = self.api.lane.getAllSubscriptionResults()
        vehicles = self.api.vehicle.getAllSubscriptionResults()
        self.queue, self.arriving, self.occupancy, self.room = {}, {}, {}, {}
        for key in self.ids:
            for m in self.movements[key]:
                lane = m.incoming
                if lane in self.queue:
                    continue
                path = topo.approaches.get(lane, (lane,))
                queue = sum(lanes.get(cell, {}).get(c.LAST_STEP_VEHICLE_HALTING_NUMBER, 0) for cell in path)
                arriving, remainder = 0, 0.0
                for cell in reversed(path):  # from the stop line backwards
                    length = topo.lanes[cell].length
                    for vid in lanes.get(cell, {}).get(c.LAST_STEP_VEHICLE_ID_LIST, ()):
                        row = vehicles.get(vid, {})
                        speed = row.get(c.VAR_SPEED, 0)
                        if speed >= .1 and (length - row.get(c.VAR_LANEPOSITION, 0) + remainder) / max(2.0, speed) <= GAP_SECONDS:
                            arriving += 1
                    remainder += length
                capacity = topo.capacities.get(lane, 1.0)
                self.queue[lane], self.arriving[lane] = queue, arriving
                self.occupancy[lane] = queue / capacity if queue >= min(MINIMUM_QUEUE, capacity / 2) else 0.0
        for end in set(self.exit_of.values()):
            group = topo.receiving_groups.get(end, (end,))
            capacity = sum(topo.lanes[l].length for l in group) / self.settings.storage_per_vehicle
            self.room[end] = capacity - sum(lanes.get(l, {}).get(c.LAST_STEP_VEHICLE_NUMBER, 0) for l in group)

    # --- helpers --------------------------------------------------------------------
    def served(self, key, state):
        return {m.incoming for m in self.movements[key] if state[m.index] in "gG"}

    def waiting_on_red(self, key, state):
        """Someone queues on a lane that has a red turn in this phase.

        A lane whose straight movement is green can still hold a car waiting at
        the stop line for its red left turn, so any queued lane with a red
        movement counts. Otherwise a resting green could hold that car forever.
        """
        return any(self.queue.get(m.incoming, 0) > 0 for m in self.movements[key] if state[m.index] not in "gG")

    def exits_full(self, key, state, served):
        """Every queued lane on green leads only to roads with no room."""
        checked = False
        for lane in served:
            if self.queue.get(lane, 0) < 1:
                continue
            exits = [self.exit_of[m.outgoing] for m in self.movements[key] if m.incoming == lane and state[m.index] in "gG"]
            if any(self.room.get(end, 0) >= 1 for end in exits):
                return False
            checked = True
        return checked

    def feeds_short_link(self, lanes):
        return any(self.topology.capacities.get(lane, SHORT_LINK) < SHORT_LINK for lane in lanes)

    def next_green(self, key, phase):
        phases = self.phases[key]
        for offset in range(1, len(phases)):
            index = (phase + offset) % len(phases)
            if is_green(phases[index][1]):
                return index
        return phase

    def act(self, key, kind, reason):
        self.adjustments[kind] += 1
        self.by_signal[key][kind] += 1
        self.reasons[key] = reason
        self.decisions += 1

    # --- decisions --------------------------------------------------------------------
    def on_green_start(self, key, phase, now):
        """Skip a green that nobody needs while someone waits elsewhere."""
        if key in self.plan_locked:
            return
        served = self.served(key, self.phases[key][phase][1])
        if not self.queue or any(self.queue.get(lane, 0) or self.arriving.get(lane, 0) for lane in served):
            return
        if self.feeds_short_link(served):
            return
        if not self.waiting_on_red(key, self.phases[key][phase][1]):
            return
        target = self.next_green(key, phase)
        if target != phase:
            # The previous green's yellow and all-red have just run, so the next
            # green can start directly.
            self.api.trafficlight.setPhase(key, target)
            self.phase[key], self.started[key] = target, now
            self.act(key, "skipped_phase", "Skip: nobody needs this green")

    def decide(self, key, phase, now, remaining):
        if key in self.plan_locked:
            return
        duration, state = self.phases[key][phase]
        elapsed = now - self.started[key]
        served = self.served(key, state)
        if not self.waiting_on_red(key, state):
            if remaining <= TICK + .5:
                self.api.trafficlight.setPhaseDuration(key, 2 * TICK)
                self.act(key, "rest", "Rest: nobody is waiting on red")
            return
        if elapsed < min(MINIMUM_GREEN, duration):
            return
        if (elapsed >= GAP_OUT_SHARE * duration and not any(self.queue.get(lane, 0) for lane in served)
                and not any(self.arriving.get(lane, 0) for lane in served) and not self.feeds_short_link(served)):
            self.api.trafficlight.setPhaseDuration(key, 0)
            return self.act(key, "gap_out", "Gap-out: green is empty and cars wait on red")
        if self.exits_full(key, state, served):
            self.blocked_ticks[key] += 1
            if self.blocked_ticks[key] >= BLOCKED_TICKS:
                self.blocked_ticks[key] = 0
                self.api.trafficlight.setPhaseDuration(key, 0)
                return self.act(key, "blocked_exit", "Exit full: the road this green feeds has no room")
            return
        self.blocked_ticks[key] = 0
        red = {m.incoming for m in self.movements[key]} - served
        red_risk = max((self.occupancy.get(lane, 0) for lane in red), default=0.0)
        green_risk = max((self.occupancy.get(lane, 0) for lane in served), default=0.0)
        if green_risk >= SPILLBACK and red_risk < COMFORTABLE:
            limit = duration + min(EXTENSION_SHARE * duration, EXTENSION_SECONDS)
            if remaining <= TICK + .5 and elapsed + remaining < limit:
                self.api.trafficlight.setPhaseDuration(key, min(2 * TICK, limit - elapsed))
                self.act(key, "spillback_extension", "Extend: this queue is about to block the junction behind it")
            return
        busy_short_links = [lane for lane in served if self.queue.get(lane, 0) or self.arriving.get(lane, 0)]
        if (red_risk >= SPILLBACK and green_risk < COMFORTABLE and elapsed >= CUT_SHARE * duration
                and not self.feeds_short_link(busy_short_links)):
            self.api.trafficlight.setPhaseDuration(key, 0)
            return self.act(key, "spillback_cut", "Cut: a queue on red is about to block the junction behind it")

    def step(self, now, vehicles=None):
        c = self.constants
        lights = self.api.trafficlight.getAllSubscriptionResults()
        active = self.mode == "anchored"
        tick = active and now + 1e-6 >= self.last_tick + TICK
        if tick:
            self.last_tick = now
            self.observe()
        for key in self.ids:
            row = lights.get(key)
            if not row:
                continue
            phase = row[c.TL_CURRENT_PHASE]
            if self.phase.get(key) != phase:
                self.phase[key], self.started[key] = phase, now
                if active and is_green(self.phases[key][phase][1]):
                    self.on_green_start(key, phase, now)
            elif tick and is_green(self.phases[key][phase][1]):
                self.decide(key, phase, now, row[c.TL_NEXT_SWITCH] - now)

    def status(self):
        return {"controller": "anchored", "mode": self.mode, "signal_count": len(self.ids),
                "plan_locked": sorted(self.plan_locked),
                "decisions": self.decisions, "adjustments": dict(self.adjustments)}
