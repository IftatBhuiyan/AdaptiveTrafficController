"""Decision rules of the plan-anchored controller, on a two-phase signal with no SUMO."""
from collections import defaultdict
from types import SimpleNamespace
import unittest

from adaptive_controller import Movement
from anchored_controller import AnchoredController

# Phase 0: approach "a" green for 30 s, phase 2: approach "b" green for 20 s.
PHASES = ((30.0, "GGrr"), (3.0, "yyrr"), (20.0, "rrGG"), (3.0, "rryy"))


def controller(queue=None, arriving=None, occupancy=None, room=None, mode="anchored"):
    calls = []
    lights = SimpleNamespace(setPhaseDuration=lambda key, value: calls.append(("duration", value)),
                             setPhase=lambda key, value: calls.append(("phase", value)))
    c = AnchoredController.__new__(AnchoredController)
    c.api, c.mode = SimpleNamespace(trafficlight=lights), mode
    c.ids = ["s"]
    c.phases = {"s": PHASES}
    c.movements = {"s": (Movement("a", "x0", 0, "s"), Movement("a", "x1", 1, "l"),
                         Movement("b", "y0", 2, "s"), Movement("b", "y1", 3, "r"))}
    c.exit_of = {"x0": "x", "x1": "x", "y0": "y", "y1": "y"}
    c.queue = {"a": 0, "b": 0, **(queue or {})}
    c.arriving = {"a": 0, "b": 0, **(arriving or {})}
    c.occupancy = {"a": 0.0, "b": 0.0, **(occupancy or {})}
    c.room = {"x": 10, "y": 10, **(room or {})}
    c.phase, c.started = {"s": 0}, {"s": 0.0}
    c.topology = SimpleNamespace(capacities={"a": 20.0, "b": 20.0})
    c.blocked_ticks = defaultdict(int)
    c.adjustments = defaultdict(int)
    c.by_signal = defaultdict(lambda: defaultdict(int))
    c.reasons, c.decisions = {}, 0
    c.plan_locked = set()
    return c, calls


class AnchoredRules(unittest.TestCase):
    def test_rest_keeps_green_when_nobody_waits_on_red(self):
        c, calls = controller(queue={"a": 2})
        c.decide("s", 0, now=29.0, remaining=1.0)
        self.assertEqual(calls, [("duration", 4.0)])
        self.assertEqual(c.adjustments["rest"], 1)

    def test_no_rest_while_a_car_waits_for_a_red_turn_on_a_green_lane(self):
        # Lane "a": straight (link 0) green, left (link 1) red in phase 0. A car
        # waiting to turn left must get its phase; resting here locked seed 42.
        c, calls = controller(queue={"a": 2})
        c.phases["s"] = ((30.0, "GrGG"), (3.0, "yryy"), (20.0, "rGrr"), (3.0, "ryrr"))
        c.decide("s", 0, now=29.0, remaining=1.0)
        self.assertEqual(calls, [])
        self.assertEqual(c.adjustments["rest"], 0)

    def test_rest_waits_until_the_planned_end_is_near(self):
        c, calls = controller()
        c.decide("s", 0, now=10.0, remaining=20.0)
        self.assertEqual(calls, [], "The plan still runs; the green is only kept past its end")

    def test_gap_out_ends_empty_green_after_half_the_plan(self):
        c, calls = controller(queue={"b": 3})
        c.decide("s", 0, now=16.0, remaining=14.0)
        self.assertEqual(calls, [("duration", 0)])
        self.assertEqual(c.adjustments["gap_out"], 1)

    def test_no_gap_out_before_half_the_plan(self):
        c, calls = controller(queue={"b": 3})
        c.decide("s", 0, now=10.0, remaining=20.0)
        self.assertEqual(calls, [])

    def test_no_gap_out_while_a_platoon_is_arriving(self):
        c, calls = controller(queue={"b": 3}, arriving={"a": 2})
        c.decide("s", 0, now=16.0, remaining=14.0)
        self.assertEqual(calls, [])

    def test_short_link_green_never_gaps_out(self):
        # 31st St between Astoria Blvd N and the junction under I-278 holds 3-4
        # cars; its platoons come from the signal just upstream.
        c, calls = controller(queue={"b": 3})
        c.topology.capacities["a"] = 3.5
        c.decide("s", 0, now=20.0, remaining=10.0)
        self.assertEqual(calls, [])

    def test_short_link_green_is_not_skipped(self):
        c, calls = controller(queue={"a": 4})
        c.topology.capacities["b"] = 3.5
        c.on_green_start("s", 2, now=40.0)
        self.assertEqual(calls, [])

    def test_full_exit_ends_green_after_two_decisions(self):
        c, calls = controller(queue={"a": 5, "b": 3}, room={"x": 0})
        c.decide("s", 0, now=10.0, remaining=20.0)
        self.assertEqual(calls, [], "One reading is not enough; the exit may free up")
        c.decide("s", 0, now=12.0, remaining=18.0)
        self.assertEqual(calls, [("duration", 0)])
        self.assertEqual(c.adjustments["blocked_exit"], 1)

    def test_exit_with_room_resets_the_blocked_count(self):
        c, calls = controller(queue={"a": 5, "b": 3}, room={"x": 0})
        c.decide("s", 0, now=10.0, remaining=20.0)
        c.room["x"] = 5
        c.decide("s", 0, now=12.0, remaining=18.0)
        c.room["x"] = 0
        c.decide("s", 0, now=14.0, remaining=16.0)
        self.assertEqual(calls, [])

    def test_spillback_extension_is_capped(self):
        c, calls = controller(queue={"a": 20, "b": 1}, occupancy={"a": .9})
        c.decide("s", 0, now=29.0, remaining=1.0)
        self.assertEqual(calls, [("duration", 4.0)])
        c.decide("s", 0, now=44.0, remaining=0.5)
        self.assertEqual(calls[-1], ("duration", 1.0), "At most 50% more than the 30 s plan")
        calls.clear()
        c.decide("s", 0, now=45.0, remaining=0.0)
        self.assertEqual(calls, [])

    def test_no_extension_when_a_red_queue_is_also_long(self):
        c, calls = controller(queue={"a": 20, "b": 12}, occupancy={"a": .9, "b": .6})
        c.decide("s", 0, now=29.0, remaining=1.0)
        self.assertEqual(calls, [])

    def test_spilling_red_queue_cuts_green_after_70_percent(self):
        c, calls = controller(queue={"a": 3, "b": 20}, occupancy={"b": .9})
        c.decide("s", 0, now=15.0, remaining=15.0)
        self.assertEqual(calls, [])
        c.decide("s", 0, now=21.0, remaining=9.0)
        self.assertEqual(calls, [("duration", 0)])
        self.assertEqual(c.adjustments["spillback_cut"], 1)

    def test_green_moving_cars_off_a_short_link_is_not_cut_for_spillback(self):
        # Lane "a" holds 3.5 cars; one car there reads as a small queue, but
        # cutting this green blocked the signal upstream and locked seed 42.
        c, calls = controller(queue={"a": 1, "b": 20}, occupancy={"b": .9})
        c.topology.capacities["a"] = 3.5
        c.decide("s", 0, now=25.0, remaining=5.0)
        self.assertEqual(calls, [])

    def test_empty_short_link_green_can_still_be_cut_for_spillback(self):
        c, calls = controller(queue={"b": 20}, occupancy={"b": .9})
        c.topology.capacities["a"] = 3.5
        c.decide("s", 0, now=25.0, remaining=5.0)
        self.assertEqual(calls, [("duration", 0)])

    def test_empty_green_is_skipped_when_someone_waits(self):
        c, calls = controller(queue={"a": 4})
        c.on_green_start("s", 2, now=40.0)
        self.assertEqual(calls, [("phase", 0)])
        self.assertEqual(c.phase["s"], 0)

    def test_green_is_not_skipped_for_arriving_cars(self):
        c, calls = controller(queue={"a": 4}, arriving={"b": 1})
        c.on_green_start("s", 2, now=40.0)
        self.assertEqual(calls, [])

    def test_tightly_linked_signal_keeps_the_plan(self):
        c, calls = controller(queue={"b": 3})
        c.plan_locked = {"s"}
        c.decide("s", 0, now=16.0, remaining=14.0)
        c.on_green_start("s", 2, now=40.0)
        self.assertEqual(calls, [])

    def test_tightly_linked_finds_signals_joined_by_a_tiny_road(self):
        from adaptive_controller import Lane, Movement as M
        c = AnchoredController.__new__(AnchoredController)
        c.ids = ["up", "down"]
        c.movements = {"up": (M("in_0", "tiny_0", 0, "s"),), "down": (M("tiny_0", "out_0", 0, "s"),)}
        topo = SimpleNamespace(
            lanes={"in_0": Lane("in_0", "in", 100, 13, 0), "tiny_0": Lane("tiny_0", "tiny", 25, 13, 0)},
            edges={"in": ("n0", "n1"), "tiny": ("n1", "n2")},
            approaches={}, capacities={"in_0": 13.0, "tiny_0": 3.3})
        self.assertEqual(c.tightly_linked(topo), {"down"})
        topo.capacities["tiny_0"] = 6.0
        self.assertEqual(c.tightly_linked(topo), set())

    def test_fixed_mode_leaves_the_plan_alone(self):
        c, calls = controller(queue={"b": 3}, mode="fixed")
        c.constants = SimpleNamespace(TL_CURRENT_PHASE=0x28, TL_NEXT_SWITCH=0x2d)
        c.api.trafficlight.getAllSubscriptionResults = lambda: {"s": {0x28: 0, 0x2d: 14.0}}
        c.last_tick = -1e9
        c.step(16.0)
        self.assertEqual(calls, [])


if __name__ == "__main__":
    unittest.main()
