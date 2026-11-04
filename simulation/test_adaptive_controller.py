"""Safety and causality contracts for the new controller, with plain logic."""
from collections import defaultdict
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from adaptive_controller import AdaptiveController, Settings, Signal, Movement, Lane, select_scope, usable_supply, protected_actions, partition_approaches, head_prefix, connector_exits
from intelligent_coordinator import IntelligentCoordinator, capacity_pressure, relief_phases
from signal_groups import joint_program


class PhaseSafety(unittest.TestCase):
    def clearance_case(self, inside, conflicting=True):
        """Signal switching GGrr -> rrGG with one internal lane carrying link 0."""
        states, cars = [], {"inside": dict(inside)}
        api = SimpleNamespace(trafficlight=SimpleNamespace(setRedYellowGreenState=lambda t, s: states.append(s)),
                              lane=SimpleNamespace(getAllSubscriptionResults=lambda: cars))
        c = AdaptiveController.__new__(AdaptiveController)
        s = Signal("signal", ("junction",), ("GGrr", "rrGG"), (), 3, stage="green")
        c.signals, c.api, c.settings = {"signal": s}, api, Settings()
        c.constants = SimpleNamespace(LAST_STEP_VEHICLE_NUMBER=18, LAST_STEP_VEHICLE_HALTING_NUMBER=20)
        c.topology = SimpleNamespace(internal_lanes={"signal": ("inside",)}, internal_link_index={"inside": {0}},
                                     conflicts={"signal": lambda a, b: conflicting and {a, b} == {0, 2}})
        return c, s, states, cars

    def test_moving_conflicting_vehicle_extends_clearance_but_only_briefly(self):
        c, s, states, cars = self.clearance_case({18: 1, 20: 0})
        c.change(s, 1, 10)
        c.advance(12.9)
        self.assertEqual(states, ["yyrr"])
        c.advance(13)
        self.assertEqual(states[-1], "rrrr")
        c.advance(15)
        self.assertEqual(states[-1], "rrrr", "A vehicle still crossing a conflicting path delays the next green")
        c.advance(15 + Settings().clearance_extension + .01)
        self.assertEqual(s.stage, "green", "Clearance extension is capped; a signal never stays all-red indefinitely")
        self.assertEqual(states[-1], "rrrG", "Only the turn crossing the occupied connector stays red")

    def test_moving_conflicting_vehicle_that_clears_releases_green_immediately(self):
        c, s, states, cars = self.clearance_case({18: 1, 20: 0})
        c.change(s, 1, 10)
        c.advance(13)
        c.advance(15)
        cars["inside"][18] = 0
        c.advance(15.2)
        self.assertEqual(states, ["yyrr", "rrrr", "rrGG"])

    def test_stopped_box_blocker_holds_only_its_conflicting_link(self):
        # Replay (merge, seed 45): a car stopped in the box restarted as the
        # cross green began and was hit. Its conflicting link waits; the rest go.
        c, s, states, cars = self.clearance_case({18: 1, 20: 1})
        c.change(s, 1, 10)
        c.advance(13)
        c.advance(15)
        self.assertEqual((s.stage, states[-1]), ("green", "rrrG"), "The signal still decides; non-conflicting turns open")
        c.advance(20)
        self.assertEqual(states[-1], "rrrG")
        cars["inside"][18] = 0
        c.advance(20.2)
        self.assertEqual(states[-1], "rrGG", "Held link opens the moment the box connector empties")
        c.change(s, 0, 30)
        self.assertEqual(states[-1], "rryy")

    def test_non_conflicting_vehicle_does_not_extend_clearance(self):
        c, s, states, cars = self.clearance_case({18: 1, 20: 0}, conflicting=False)
        c.change(s, 1, 10)
        c.advance(13)
        c.advance(15)
        self.assertEqual(states[-1], "rrGG")

    def test_full_stopped_entrance_overrides_a_large_discharge_forecast(self):
        self.assertEqual(usable_supply(20, 20, 1, 0, 10, 30), 0)
        self.assertGreater(usable_supply(5, 20, 20, 8, .2, 30), 0)

    def test_five_signal_hops_is_not_five_arbitrary_nodes(self):
        graph = {str(i): {str(j) for j in (i - 1, i + 1) if 0 <= j < 9} for i in range(9)}
        positions = {str(i): (100 * i, 0) for i in range(9)}
        scope, distance = select_scope(graph, positions, ("0",), hops=5, radius=0)
        self.assertEqual(scope, {str(i) for i in range(6)})
        self.assertEqual(distance["5"], 5)

    def test_two_feeders_share_one_finite_supply_budget(self):
        c = AdaptiveController.__new__(AdaptiveController)
        c.settings = Settings(predictive=False)
        move = lambda lane: SimpleNamespace(incoming=lane, outgoing="shared", index=0)
        c.signals = {t: SimpleNamespace(current=0, yellow=3, greens=("G",), movements=(move(t),)) for t in ("a", "b")}
        c.queued, c.discharge_rate = {"a": 20, "b": 20}, defaultdict(lambda: .5)
        c.topology = SimpleNamespace(receivers={"shared": ("shared",)})
        c.receiving = lambda o, p: (8, 10, 2, 0)
        c.turn_share = lambda a, b: 1
        grants = c.reserve_receiving_space({"a": 0, "b": 0})["shared"]
        self.assertAlmostEqual(sum(grants.values()), 2)
        self.assertAlmostEqual(grants["a"], grants["b"])

    def test_permissive_crossing_is_separated_without_losing_a_road_movement(self):
        lanes = {k: Lane(k, k, 30, 10, 0) for k in ("in-a", "in-b", "out-a", "out-b")}
        moves = (Movement("in-a", "out-a", 0, "s"), Movement("in-b", "out-b", 1, "l"))
        states, retained = protected_actions(("Gg",), moves, lanes, lambda a, b: True)
        self.assertEqual(states, ("Gr", "rG"))
        self.assertEqual(len(retained), 2)
        self.assertTrue(all("g" not in s for s in states))

    def test_lane_fanout_does_not_hide_the_long_upstream_queue(self):
        lanes = {k: Lane(k, "road", length, 10, index) for k, length, index in
                 [("upstream", 150, 0), ("left", 5, 1), ("right", 5, 0)]}
        result = partition_approaches({"left": ("upstream", "left"), "right": ("upstream", "right")}, lanes)
        flattened = [l for path in result.values() for l in path]
        self.assertEqual(flattened.count("upstream"), 1)
        self.assertEqual(set(flattened), set(lanes))

    def test_one_blocked_turn_can_stop_a_shared_lane_prefix(self):
        self.assertAlmostEqual(head_prefix(10, .5), .9990234375)
        self.assertEqual(head_prefix(10, 1), 10)
        self.assertEqual(head_prefix(10, 0), 0)

    def test_every_lane_preference_retains_a_green_without_simultaneous_fanout(self):
        lanes = {k: Lane(k, edge, 30, 10, index) for k, edge, index in
                 [("in", "in", 0), ("out-0", "out", 0), ("out-1", "out", 1)]}
        moves = (Movement("in", "out-0", 0, "s"), Movement("in", "out-1", 1, "s"))
        states, retained = protected_actions(("GG",), moves, lanes, lambda a, b: False)
        self.assertEqual(set(states), {"Gr", "rG"})
        self.assertEqual(retained, moves)

    def test_internal_queue_before_a_sub_car_length_road_is_observable(self):
        root = ET.fromstring('''<net>
          <edge id=":a" function="internal"><lane id=":a_0" length="5"/></edge>
          <edge id=":b" function="internal"><lane id=":b_0" length="2"/></edge>
          <connection from=":a" fromLane="0" to="tiny" toLane="0" via=":b_0"/>
          <connection from=":b" fromLane="0" to="tiny" toLane="0"/>
        </net>''')
        lengths, exits = connector_exits(root, {"tiny_0"})
        self.assertEqual(exits[":a_0"], ("tiny_0", 2))
        self.assertEqual(exits[":b_0"], ("tiny_0", 0))
        c = AdaptiveController.__new__(AdaptiveController)
        c.settings, c.constants = Settings(), SimpleNamespace(LAST_STEP_VEHICLE_ID_LIST=1,
            LAST_STEP_VEHICLE_NUMBER=2, LAST_STEP_VEHICLE_HALTING_NUMBER=3, LAST_STEP_MEAN_SPEED=4)
        c.topology = SimpleNamespace(detectors=("tiny_0", "out_0"), internal_lengths=lengths,
            connector_exits=exits, cell_owner={"tiny_0": "tiny_0"}, approaches={"tiny_0": ("tiny_0",)},
            lanes={"tiny_0": Lane("tiny_0", "tiny", .2, 10, 0), "out_0": Lane("out_0", "out", 100, 10, 0)},
            incoming=("tiny_0",), owner={"tiny_0": "light"}, receiving_groups={}, receivers={"out_0": ("out_0",)})
        c.api = SimpleNamespace(lane=SimpleNamespace(getAllSubscriptionResults=lambda: {}))
        c.signals = {"light": Signal("light", (), ("G",), (Movement("tiny_0", "out_0", 0, "s"),), 3, stage="green")}
        c.ready_since, c.queued, c.last_service, c.previous_count, c.forecast_errors = {}, {}, {}, {}, []
        c.flow_since_tick, c.exit_since_tick = defaultdict(int), defaultdict(int)
        c.arrival_rate, c.discharge_rate = defaultdict(float), defaultdict(lambda: .45)
        c.predictions = __import__("collections").deque()
        c.observe(10, {"waiting": (":a_0", 4, 0, 5)})
        self.assertEqual(c.queued["tiny_0"], 1)
        self.assertEqual(c.ready_since["tiny_0"], 10)
        self.assertEqual(c.previous_count["tiny_0"], 1)
        # SUMO reports no mean speed on an empty lane. An empty 0.2 m entry
        # must not masquerade as stopped traffic blocking a larger receiver.
        entry = c.rows["tiny_0"]
        self.assertGreater(usable_supply(0, 10, entry["entry_gap"], entry["speed"], 0, 0), 0)
        # A head can also wait on the preceding external road when the final
        # road is shorter than the stopping gap. It must remain ready/visible.
        c.topology.lanes["upstream_0"] = Lane("upstream_0", "upstream", 50, 10, 0)
        c.topology.detectors = ("upstream_0", "tiny_0", "out_0")
        c.topology.approaches["tiny_0"] = ("upstream_0", "tiny_0")
        c.api.lane.getAllSubscriptionResults = lambda: {"upstream_0": {1: ["waiting"], 2: 1, 3: 1, 4: 0}}
        c.ready_since.clear()
        c.observe(20, {"waiting": ("upstream_0", 49, 0, 5)})
        self.assertEqual(c.queued["tiny_0"], 1)
        self.assertEqual(c.ready_since["tiny_0"], 20)

    def test_connector_departure_is_counted_once_even_when_tiny_road_is_skipped(self):
        c = AdaptiveController.__new__(AdaptiveController)
        c.topology = SimpleNamespace(connector_movements={":box_0": ("tiny_0", "out_0")},
            receiving_groups={}, movement_keys={("tiny_0", "out_0")}, lanes={"tiny_0": None, "out_0": None})
        c.last_external, c.last_crossing, c.last_service = {"v": "tiny_0"}, {}, {}
        c.learning_eligible = {"tiny_0": True}
        c.turn_counts, c.flow_since_tick, c.exit_since_tick = defaultdict(float), defaultdict(int), defaultdict(int)
        c.observe_transfers(1, {"v": (":box_0", 1, 3, 5)})
        c.observe_transfers(1.2, {"v": (":box_0", 2, 3, 5)})
        c.observe_transfers(1.4, {"v": ("out_0", 1, 3, 5)})
        self.assertEqual(c.flow_since_tick["tiny_0"], 1)
        self.assertEqual(c.turn_counts["tiny_0", "out_0"], 1)

    def test_ready_head_recovery_does_not_depend_on_a_large_flow_prediction(self):
        c = AdaptiveController.__new__(AdaptiveController)
        s = Signal("light", (), ("Gr", "rG"),
                   (Movement("in", "a", 0, "s"), Movement("in", "b", 1, "l")), 3, stage="green")
        c.signals, c.settings, c.mode, c.last_tick = {"light": s}, Settings(), "coordinated", 0
        c.topology = SimpleNamespace(graph={"light": set()}, scope={"light"})
        c.queued, c.ready_since, c.last_service = {"in": 1}, {"in": 0}, {"in": 0}
        c.phase_service, c.messages, c.decisions = {}, 0, 0
        c.observe_transfers = c.observe = lambda *args: None
        c.advance = lambda now: None
        c.reserve_receiving_space = c.propagated_arrivals = lambda plans: {}
        def score(signal, phase, plans):
            c.phase_service[signal.id, phase] = {"in": .22}
            return phase, .22
        c.score = score
        targets = []
        c.change = lambda signal, target, now: targets.append(target)
        c.step(20, {})
        self.assertEqual(targets, [1], "A waiting head with zero actual departures must be able to try another legal turn")


class IntelligentAccounting(unittest.TestCase):
    def test_joint_program_never_feeds_a_red_zero_storage_exit(self):
        leader = Signal("lead", (), ("GG",),
                        (Movement("first", "port_a", 0, "s"), Movement("second", "port_b", 1, "s")),
                        3, green_durations=(20,))
        follower = Signal("follow", (), ("Gr", "rG"),
                          (Movement("port_a", "out", 0, "s"), Movement("port_b", "out", 1, "s")), 3)
        greens, durations, targets, references = joint_program(leader, follower, {0: "port_a", 1: "port_b"}, 1, 8)
        self.assertEqual(sum(durations[:references]), 20)
        self.assertEqual({index for green in greens[:references] for index, state in enumerate(green) if state == "G"}, {0, 1})
        for green, target in zip(greens, targets):
            self.assertTrue(all(value == "r" or follower.greens[target][index] == "G" for index, value in enumerate(green)))

    def test_portal_clearance_waits_for_the_vehicle_tail(self):
        controller = IntelligentCoordinator.__new__(IntelligentCoordinator)
        controller.constants = SimpleNamespace(LAST_STEP_VEHICLE_NUMBER=18)
        readings = {"port": {18: 0}}
        controller.api = SimpleNamespace(lane=SimpleNamespace(getAllSubscriptionResults=lambda: readings))
        group = SimpleNamespace(cells={"port"}, exits={"exit"})
        controller.current_vehicles = {"car": ("exit", 1, 5, 5)}
        self.assertFalse(controller.portal_clear(group))
        controller.current_vehicles["car"] = ("exit", 7, 5, 5)
        self.assertTrue(controller.portal_clear(group))
        readings.clear()
        self.assertFalse(controller.portal_clear(group))

    def test_joint_handoff_drains_before_changing_receiver_and_reopening(self):
        for reverse in (False, True):
            with self.subTest(reverse=reverse):
                controller = IntelligentCoordinator.__new__(IntelligentCoordinator)
                leader = Signal("lead", (), ("Gr", "rG"), (), 3, stage="green")
                follower = Signal("follow", (), ("Gr", "rG"), (), 3, stage="green")
                signals = [leader, follower]
                controller.signals = {signal.id: signal for signal in (reversed(signals) if reverse else signals)}
                controller.settings, controller.mode = Settings(), "coordinated"
                controller.groups = {"lead": SimpleNamespace(follower="follow", follower_phases=(0, 1),
                                                              cells={"port"}, exits={"exit"})}
                controller.followers = {"follow": "lead"}
                controller.constants = SimpleNamespace(LAST_STEP_VEHICLE_NUMBER=18)
                controller.current_vehicles = {}
                readings, displayed = {"port": {18: 1}}, {"lead": "Gr", "follow": "Gr"}
                controller.api = SimpleNamespace(
                    lane=SimpleNamespace(getAllSubscriptionResults=lambda: readings),
                    trafficlight=SimpleNamespace(setRedYellowGreenState=lambda signal, state: displayed.update({signal: state})))
                controller.topology = SimpleNamespace(internal_lanes={"lead": (), "follow": ()}, conflicts={})
                controller.change(leader, 1, 0)
                controller.advance(3)
                controller.advance(20)
                self.assertEqual(displayed, {"lead": "rr", "follow": "Gr"})
                readings["port"][18] = 0
                controller.advance(21)
                self.assertEqual(displayed, {"lead": "rr", "follow": "yr"})
                controller.advance(24)
                self.assertEqual(displayed, {"lead": "rr", "follow": "rr"})
                controller.advance(26)
                self.assertEqual(displayed, {"lead": "rr", "follow": "rG"})
                controller.advance(26.2)
                self.assertEqual(displayed, {"lead": "rG", "follow": "rG"})

    def test_yielding_turn_is_not_promised_against_a_busy_priority_stream(self):
        controller = IntelligentCoordinator.__new__(IntelligentCoordinator)
        controller.queued, controller.forecasts = {"through": 10, "left": 5}, {}
        controller.topology = SimpleNamespace(conflicts={"light": lambda first, second: first != second})
        signal = Signal("light", (), ("Gg",),
                        (Movement("through", "out", 0, "s"), Movement("left", "side", 1, "l")), 3)
        self.assertEqual([movement.incoming for movement in controller.active_movements(signal, 0)], ["through"])
        extra = relief_phases(signal, controller.topology.conflicts["light"])
        self.assertEqual(extra, ("rg",))
        signal.greens += extra
        self.assertEqual([movement.incoming for movement in controller.active_movements(signal, 1)], ["left"])

    def test_failed_service_expires_or_resets_when_the_head_changes(self):
        controller = IntelligentCoordinator.__new__(IntelligentCoordinator)
        controller.settings, controller.last_tick = Settings(), 100
        controller.ready_since = {"in": 50}
        controller.waiting_heads = {"in": "first"}
        signal = Signal("light", (), ("G",), (Movement("in", "out", 0, "s"),), 3)
        controller.phase_failures = {("light", 0): (controller.head_signature(signal, 0), 90)}
        self.assertTrue(controller.known_stalled(signal, 0))
        controller.waiting_heads["in"] = "second"
        self.assertFalse(controller.known_stalled(signal, 0))
        controller.waiting_heads["in"] = "first"
        controller.last_tick = 121
        self.assertFalse(controller.known_stalled(signal, 0))

    def test_approaching_platoon_is_not_a_stalled_queue(self):
        controller = IntelligentCoordinator.__new__(IntelligentCoordinator)
        controller.settings = Settings()
        controller.last_departure, controller.ready_since = {}, {}
        controller.phase_demand = lambda signal, phase: 1
        controller.phase_wait = lambda signal, phase: 0
        controller.service_priority = lambda signal, phase: 2 if phase == 0 else 1
        signal = Signal("light", (), ("Gr", "rG"),
                        (Movement("first", "out", 0, "s"), Movement("second", "out", 1, "s")),
                        3, stage="green", green_durations=(30, 30))
        selected, _ = controller.select_phase(signal, [(-10, 1), (-5, 1)], 20)
        self.assertEqual(selected, 0, "Approaching vehicles are not evidence of a blocked stop line")

    def test_waiting_cohorts_survive_partial_service(self):
        controller = IntelligentCoordinator.__new__(IntelligentCoordinator)
        controller.waiting_since = {}
        controller.discharge_rate, controller.queued, controller.ready_since = {}, {}, {}
        controller.signals, controller.rows, controller.unsignalized = {}, {}, {}
        controller.constants = SimpleNamespace()
        controller.native_ids = set()
        controller.native_owners = {}
        controller.api = SimpleNamespace(trafficlight=SimpleNamespace(getAllSubscriptionResults=lambda: {}))
        controller.topology = SimpleNamespace(
            connector_exits={}, internal_lengths={}, incoming=(), owner={"approach": "signal"}, cell_owner={"approach": "approach"},
            approaches={"approach": ("approach",)}, lanes={"approach": SimpleNamespace(length=100)})
        with patch.object(AdaptiveController, "observe"):
            controller.observe(10, {"first": ("approach", 95, 0, 5)})
            controller.observe(40, {"first": ("approach", 95, 0, 5), "second": ("approach", 85, 0, 5)})
            self.assertEqual(controller.cohort_waits["approach"], [30, 0])
            controller.observe(50, {"first": ("exit", 10, 8, 5), "second": ("approach", 95, 0, 5)})
            self.assertEqual(controller.cohort_waits["approach"], [10])
            self.assertNotIn("first", controller.waiting_since)
            controller.observe(55, {"second": ("approach", 98, 1, 5)})
            self.assertEqual(controller.cohort_waits["approach"], [15])
            controller.topology.cell_owner["adjacent"] = "adjacent"
            controller.topology.owner["adjacent"] = "signal"
            controller.topology.approaches["adjacent"] = ("adjacent",)
            controller.topology.lanes["adjacent"] = SimpleNamespace(length=100)
            controller.observe(60, {"second": ("adjacent", 98, 1, 5)})
            self.assertEqual(controller.cohort_waits["adjacent"], [20])

    def test_native_red_restricts_predicted_downstream_release(self):
        controller = IntelligentCoordinator.__new__(IntelligentCoordinator)
        controller.settings, controller.last_tick = Settings(), 100
        controller.native_owners = {"tiny": "native"}
        controller.native_states = {"native": {"phase": 0, "next_switch": 130}}
        controller.topology = SimpleNamespace(
            owner={}, receivers={"out": ("tiny",)}, program_phases={"native": ((30, "r"), (30, "G"))},
            network_movements={"native": (Movement("tiny", "exit", 0, "s"),)})
        controller.signals = {}
        controller.rows, controller.blocked = {"tiny": {"entry_gap": 100, "speed": 10}}, set()
        with patch.object(AdaptiveController, "receiving", return_value=(0, .2, 13.5, .45)):
            self.assertEqual(controller.receiving("out", {})[2], 0)
            controller.native_states["native"]["next_switch"] = 110
            self.assertAlmostEqual(controller.receiving("out", {})[2], 9)

    def controller(self, supply=30):
        controller = IntelligentCoordinator.__new__(IntelligentCoordinator)
        controller.settings, controller.mode, controller.last_tick = Settings(imported_actions=True), "coordinated", 20
        controller.pressure_scale = 30
        controller.topology = SimpleNamespace(
            conflicts={"light": lambda first, second: False},
            turn_options={"in": {"out_0": "s", "out_1": "s"}},
            lanes={"out_0": SimpleNamespace(edge="out"), "out_1": SimpleNamespace(edge="out")},
            receivers={"out_0": ("shared",), "out_1": ("shared",)}, capacities={"in": 20})
        controller.turn_counts = defaultdict(float)
        controller.queued, controller.discharge_rate = {"in": 20}, {"in": .45}
        controller.forecasts, controller.neighbour_arrivals = {"in": [0] * 6}, {}
        controller.reservations, controller.last_service, controller.phase_service = {}, {"in": 0}, {}
        controller.receiving = lambda outgoing, plans: (0, 30, supply, 0)
        signal = Signal("light", (), ("Gg", "rr"),
                        (Movement("in", "out_0", 0, "s"), Movement("in", "out_1", 1, "s")), 3)
        return controller, signal

    def test_fanout_probability_and_service_are_not_duplicated(self):
        controller, signal = self.controller()
        self.assertEqual(sum(controller.turn_share("in", outgoing) for outgoing in ("out_0", "out_1")), 1)
        _, service = controller.score(signal, 0, {})
        self.assertAlmostEqual(service, .45 * 30)

    def test_shared_reservoir_has_one_supply_budget(self):
        controller, signal = self.controller(supply=3)
        _, service = controller.score(signal, 0, {})
        self.assertAlmostEqual(service, 3)

    def test_blocked_exit_cannot_receive_predicted_service(self):
        controller, signal = self.controller(supply=0)
        self.assertEqual(controller.score(signal, 0, {}), (0, 0))

    def test_switching_loses_clearance_service_time(self):
        controller, signal = self.controller()
        held = controller.score(signal, 0, {})[1]
        signal.current = 1
        switched = controller.score(signal, 0, {})[1]
        self.assertAlmostEqual(held - switched, .45 * 5)

    def test_service_cannot_exceed_the_remaining_green(self):
        controller, signal = self.controller()
        controller.last_tick = 48
        signal.stage, signal.since, signal.green_durations = "green", 0, (40, 40)
        self.assertAlmostEqual(controller.score(signal, 0, {})[1], .45 * 2)
        controller.last_tick = 50
        self.assertEqual(controller.score(signal, 0, {})[1], 0)

    def test_arrivals_after_green_ends_do_not_receive_service_or_reservations(self):
        controller, signal = self.controller()
        controller.queued["in"] = 0
        controller.forecasts["in"] = [0, 0, 0, 0, 0, 4]
        controller.last_tick = 48
        signal.stage, signal.since, signal.green_durations = "green", 0, (40, 40)
        self.assertEqual(controller.score(signal, 0, {})[1], 0)
        controller.signals = {signal.id: signal}
        reservations = controller.reserve_receiving_space({signal.id: 0})
        self.assertEqual(sum(sum(grants.values()) for grants in reservations.values()), 0)

    def test_completed_minimum_green_is_not_charged_again(self):
        controller, signal = self.controller()
        controller.cohort_waits = {}
        controller.phase_service = {(signal.id, 0): {"in": 1}, (signal.id, 1): {"in": 1}}
        signal.stage, signal.since = "green", controller.last_tick
        just_started = controller.service_priority(signal, 0)
        signal.since -= 20
        self.assertGreater(controller.service_priority(signal, 0), just_started)
        self.assertGreater(controller.service_priority(signal, 0), controller.service_priority(signal, 1))

    def test_low_density_pressure_does_not_favour_small_roads(self):
        for capacity in (10, 100):
            self.assertAlmostEqual(capacity_pressure(.001, capacity, 500) / .001, 1, places=6)

    def test_full_roads_exert_the_same_bounded_pressure(self):
        for capacity in (10, 100):
            self.assertAlmostEqual(capacity_pressure(capacity, capacity, 500), 500)
            self.assertEqual(capacity_pressure(capacity * 2, capacity, 500), 500)


if __name__ == "__main__":
    unittest.main()
