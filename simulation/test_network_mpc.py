"""Conservation and control regressions for the finite-horizon traffic model."""
import unittest
from types import SimpleNamespace
import numpy as np

from network_mpc import NetworkMPC


class NetworkModelTests(unittest.TestCase):
    def test_route_safe_repair_preserves_programs_and_only_opens_geometry_continuations(self):
        import xml.etree.ElementTree as ET
        from scenario_editor import ScenarioEditor
        original, repaired = ScenarioEditor(), ScenarioEditor(network_variant='route-safe')
        programs = lambda root: {p.get('id'): ET.tostring(p) for p in root.findall('tlLogic')}
        self.assertEqual(programs(original.root), programs(repaired.root))
        self.assertEqual(len(repaired.paths), len(original.paths))
        cleared = [c for c in repaired.root.findall('connection') if c.get('keepClear') in {'0','false'}]
        self.assertEqual(len(cleared), 4)
        self.assertEqual({c.get('from') for c in cleared}, {'1273808663#0','1273808663#1'})
        self.assertTrue(any(c.get('from') == '25345883#2' and c.get('toLane') == '0' for c in repaired.root.findall('connection')))

    def test_final_build_retains_portal_smoothing_after_netconvert(self):
        import tempfile
        import xml.etree.ElementTree as ET
        from scenario_editor import ScenarioEditor
        editor = ScenarioEditor(network_variant='route-safe')
        with tempfile.TemporaryDirectory() as folder:
            built = editor.build(editor.read(), folder)
            root = ET.parse(built['network']).getroot()
            self.assertEqual(len(built['model_repairs']['changed_lanes']), 7)
            def points(edge):
                lane = root.find(f"edge[@id='{edge}']/lane[@index='1']")
                return [list(map(float, p.split(','))) for p in lane.get('shape').split()]
            a, b = points('683926427#0'), points(':7784017558_0')
            dot = sum((a[-1][i]-a[-2][i])*(b[-1][i]-b[0][i]) for i in (0,1))
            self.assertGreater(dot, 0)

    def model(self, capacity=(2, 2), sink=True):
        m = NetworkMPC.__new__(NetworkMPC)
        m.horizon_steps, m.interval = 6, 5
        m.capacities = np.array(capacity, dtype=float)
        m.rate = np.array([1., 1.])
        m.src, m.dst = np.array([0]), np.array([1])
        m.sinks = np.array([1] if sink else [], dtype=int)
        return m

    def state(self, stock=(2, 0), ready=(2, 0), delay=(1, 1), backlog=(0, 0)):
        return (np.array(stock, dtype=float), np.array(ready, dtype=float), np.zeros((7, 2)),
                np.ones(1), np.array(delay), np.zeros(2), np.array(backlog, dtype=float), np.zeros(2))

    def test_green_transfers_conserve_stock_and_travel_prevents_instant_exit(self):
        model = self.model()
        cost, end = model._rollout(self.state(), np.ones((6, 1)), details=True)
        self.assertTrue(np.isfinite(cost))
        self.assertLess(end['mass_residual'], 1e-9)
        self.assertAlmostEqual(sum(end['stock']), 0)
        model.horizon_steps = 1
        _, first = model._rollout(self.state(), np.ones((1, 1)), details=True)
        self.assertAlmostEqual(sum(first['stock']), 2)
        self.assertAlmostEqual(first['stock'][1], 1)

    def test_full_dead_end_is_not_fictional_exit_capacity(self):
        model = self.model(sink=False)
        _, end = model._rollout(self.state(stock=(2, 2), ready=(2, 2)), np.ones((6, 1)), details=True)
        self.assertEqual(end['stock'], [2, 2])
        self.assertLess(end['mass_residual'], 1e-9)

    def test_no_storage_transfer_requires_unmasked_receiver_in_both_gate_paths(self):
        model = NetworkMPC.__new__(NetworkMPC)
        model.horizon_steps = 2
        model.groups = {'feeder': SimpleNamespace(follower='receiver', follower_phases={0: 0})}
        model.followers = {'receiver': 'feeder'}
        model.signals = {'receiver': SimpleNamespace(greens=('G',))}
        model.movements = (('in', 'out', 'feeder', 0, 's', ('receiver', 0)),)
        model.native_states = {}
        model.gate_indices = {'feeder': np.array([0])}
        model.gate_cache = {}
        model._permission = lambda key, phase, link: 0.0 if key == 'receiver' else 1.0
        option = (np.zeros(2, dtype=int), 0)
        self.assertFalse(model._gates({'feeder': option}).any())
        self.assertFalse(model._changed_gates(np.ones((2, 1)), 'feeder', option).any())

    def test_red_keeps_demand_and_receiving_backlog_visible(self):
        model = self.model(sink=False)
        _, end = model._rollout(self.state(backlog=(3, 0)), np.zeros((6, 1)), details=True)
        self.assertEqual(end['stock'], [2, 0])
        self.assertLess(end['mass_residual'], 1e-9)
        red = model._rollout(self.state(), np.zeros((6, 1)))
        model.sinks = np.array([1])
        green = model._rollout(self.state(), np.ones((6, 1)))
        self.assertLess(green, red)

    def test_future_green_sequence_changes_cost_before_first_action_changes(self):
        model = self.model()
        a, b = np.zeros((6, 1)), np.zeros((6, 1))
        a[1:3] = 1
        b[4:] = 1
        self.assertEqual(a[0, 0], b[0, 0])
        self.assertLess(model._rollout(self.state(), a), model._rollout(self.state(), b))

    def test_insertion_prediction_does_not_duplicate_observed_pending_vehicles(self):
        model = self.model(capacity=(20, 20), sink=False)
        model.horizon_steps = 1
        state = list(self.state(stock=(0, 0), ready=(0, 0), backlog=(5, 0)))
        state[5] = np.array([5., 0.])
        _, end = model._rollout(state, np.zeros((1, 1)), details=True)
        self.assertEqual(end['stock'], [5, 0])

    def test_upstream_stopped_vehicle_cannot_skip_five_hundred_metres(self):
        model = self.model(capacity=(100, 100))
        model.lane_ids = ('a', 'b')
        model.lane_index = {'a': 0, 'b': 1}
        model.rows = {'a': {'stock': 1, 'speed': 0}, 'b': {'stock': 0, 'speed': 10}}
        model.topology = SimpleNamespace(connector_exits={}, lanes={
            'a': SimpleNamespace(length=500, edge='a'), 'b': SimpleNamespace(length=10, edge='b')})
        model.movements = [('a', 'b', None, 0, 's', None)]
        model.movement_counts = __import__('collections').defaultdict(float)
        model.cohort_waits, model.boundaries = {}, set()
        model.api = SimpleNamespace(simulation=SimpleNamespace(getPendingVehicles=lambda: []))
        state = model._state({'car': ('a', 0, 0, 5)})
        self.assertEqual(state[1][0], 0)
        _, end = model._rollout(state, np.ones((6, 1)), details=True)
        self.assertEqual(end['stock'], [1, 0])

    def test_overfull_receiver_must_drain_before_accepting_replacement(self):
        model = self.model()
        model.horizon_steps = 1
        _, end = model._rollout(self.state(stock=(2, 4), ready=(2, 4)), np.ones((1, 1)), details=True)
        self.assertEqual(end['stock'], [2, 3])

    def test_maximum_green_rounds_switch_earlier_not_past_deadline(self):
        from collections import defaultdict
        m = NetworkMPC.__new__(NetworkMPC)
        m.interval, m.horizon_steps, m.last_tick = 5, 6, 155
        m.settings = SimpleNamespace(minimum_green=8, maximum_green=50, all_red=2)
        m.groups, m.cohort_waits, m.queued = {}, {}, {'b': 1}
        s = SimpleNamespace(current=0, target=0, since=108, stage='green', yellow=3,
                            greens=('Gr', 'rG'), movements=(SimpleNamespace(incoming='a', index=0), SimpleNamespace(incoming='b', index=1)))
        m.signals = {'a': s}
        for phases, first in m._catalogue('a'):
            self.assertEqual(first, 1)

    def test_candidate_honors_elapsed_minimum_and_pending_clearance(self):
        m = NetworkMPC.__new__(NetworkMPC)
        m.settings = SimpleNamespace(minimum_green=8, all_red=2)
        m.interval, m.horizon_steps, m.last_tick = 5, 6, 100
        m.groups = {}
        signal = SimpleNamespace(current=0, target=1, since=97, stage='green', yellow=3, until=0)
        m.signals = {'a': signal}
        schedule, action = m._schedule('a', 1, 0)
        self.assertEqual(action, 0)
        self.assertEqual(list(schedule[:3]), [0, -1, 1])
        signal.stage, signal.until = 'yellow', 106
        schedule, action = m._schedule('a', 0, 0)
        self.assertEqual(action, 1)
        self.assertEqual(list(schedule[:3]), [-1, -1, 1])


if __name__ == '__main__':
    unittest.main()
