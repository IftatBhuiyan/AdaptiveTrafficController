"""Regression checks for falsely accepted experiments and missing receipts."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from evaluation_validity import validity, validity_reasons
from score_evaluation import paired_score
import evaluate_controller as evaluator


SOURCE = Path(__file__).resolve().parent


def receipt(policy="fixed", new=False):
    row = {"policy": policy, "controller_family": "intelligent", "scenario": "merge", "hour": 16,
           "seed": 42, "observation_minutes": 10, "warmup_minutes": 15, "initialization": "fixed",
           "actual_end": 16 * 3600 + 600, "safe": True, "valid": True,
           "collision_vehicle_reports": 0, "teleports": 0, "inserted": 100, "completed": 80, "active": 20,
           "completed_in_observation": 20, "system_vehicle_seconds": 800, "stopped_vehicle_seconds": 400,
           "ramp_route_completions": 10, "inputs": {"network": "network-hash", "implementation/controller.py": "code-hash"},
           "demand_sha256": "demand-hash", "scope": {"signals": ["a"]}}
    if new:
        row.update(evaluation_schema=2, complete=True, inputs_start=dict(row["inputs"]), inputs_end=dict(row["inputs"]),
                   frozen_inputs_verified=True, network_sha256="network-hash",
                   case_inputs_start={"network": "network-hash", "routes": "demand-hash"},
                   case_inputs_end={"network": "network-hash", "routes": "demand-hash"})
    return row


class EvaluationValidity(unittest.TestCase):
    def test_unobservable_deadlock_or_changed_generated_inputs_cannot_pass(self):
        row = receipt(new=True)
        row['diagnostic_errors'] = {'lane_change_deadlock_scan': 'missing observation'}
        self.assertIn('deadlock_observation_unavailable', validity_reasons(row))
        row.pop('diagnostic_errors')
        row['case_inputs_end']['network'] = 'different'
        self.assertIn('generated_inputs_changed_during_run', validity_reasons(row))

    def test_deadlocked_safe_run_scores_zero_and_batch_fails(self):
        fixed, candidate = receipt(), receipt("coordinated")
        candidate.update(valid=False, lane_change_deadlocks={"car": "tiny_0"})
        score = paired_score(fixed, candidate)
        self.assertEqual(score["score"], 0)
        self.assertFalse(score["valid"])
        self.assertIn("lane_change_deadlocks", score["invalid_reasons"]["candidate"])
        self.assertFalse(evaluator.batch_valid([fixed, candidate]))

    def test_deadlock_invalidates_even_when_old_valid_flag_is_true(self):
        row = receipt()
        row["lane_change_deadlocks"] = {"car": "tiny_0"}
        self.assertFalse(validity(row))

    def test_invalid_fixed_reference_invalidates_pair(self):
        fixed = receipt()
        fixed["valid"] = False
        score = paired_score(fixed, receipt("mpc"))
        self.assertFalse(score["valid"])
        self.assertFalse(score["regression_guardrails_pass"])
        self.assertEqual(score["score"], 0)
        self.assertEqual(score["invalid_reasons"]["reference"], ["receipt_marked_invalid"])

    def test_incomplete_nonfinite_and_negative_metrics_fail(self):
        for update, reason in (({"actual_end": 16 * 3600}, "incomplete_observation_window"),
                               ({"system_vehicle_seconds": float("nan")}, "invalid_metric:system_vehicle_seconds"),
                               ({"system_vehicle_seconds": float("inf")}, "invalid_metric:system_vehicle_seconds"),
                               ({"stopped_vehicle_seconds": -1}, "invalid_metric:stopped_vehicle_seconds"),
                               ({"complete": False}, "incomplete_run")):
            with self.subTest(update=update):
                row = receipt()
                row.update(update)
                self.assertIn(reason, validity_reasons(row))
                self.assertEqual(paired_score(receipt(), dict(row, policy="mpc"))["score"], 0)

    def test_collision_teleport_removal_and_conservation_fail(self):
        for update in ({"collision_vehicle_reports": 1}, {"teleports": 1}, {"active": 19}, {"unexplained_removals": 1}):
            with self.subTest(update=update):
                row = receipt()
                row.update(update)
                self.assertFalse(validity(row))

    def test_new_rows_require_same_frozen_implementation(self):
        fixed, candidate = receipt(new=True), receipt("mpc", new=True)
        candidate["inputs"]["implementation/controller.py"] = "different"
        candidate["inputs_end"] = dict(candidate["inputs"])
        with self.assertRaisesRegex(ValueError, "implementation"):
            paired_score(fixed, candidate)
        with self.assertRaises(ValueError):
            paired_score(fixed, candidate, historical=True)

    def test_changed_source_or_generated_network_invalidates_run(self):
        row = receipt(new=True)
        self.assertTrue(validity(row))
        row["inputs_end"]["implementation/controller.py"] = "changed"
        self.assertIn("inputs_changed_during_run", validity_reasons(row))
        row = receipt(new=True)
        row["case_inputs_end"]["network"] = "changed"
        self.assertIn("generated_inputs_changed_during_run", validity_reasons(row))

    def test_historical_inspection_explicitly_labeled(self):
        fixed, candidate = receipt(), receipt("coordinated")
        candidate["inputs"]["implementation/controller.py"] = "old-other-code"
        with self.assertRaises(ValueError):
            paired_score(fixed, candidate)
        score = paired_score(fixed, candidate, historical=True)
        self.assertEqual(score["comparison_class"], "historical")
        self.assertEqual(score["score"], 80)

    def test_network_variant_or_actual_network_mismatch_refused(self):
        fixed, candidate = receipt(new=True), receipt("mpc", new=True)
        candidate["network_variant"] = "route-safe"
        with self.assertRaises(ValueError):
            paired_score(fixed, candidate)
        candidate.pop("network_variant")
        candidate["network_sha256"] = "different-network"
        with self.assertRaises(ValueError):
            paired_score(fixed, candidate)

    def test_old_receipt_validity_and_score_remain_readable(self):
        self.assertTrue(validity(receipt()))
        self.assertEqual(paired_score(receipt(), receipt("coordinated"))["score"], 80)

    def test_deadlock_score_cli_preserves_report_and_exits_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "results.json"
            out = Path(directory) / "score.json"
            candidate = receipt("coordinated")
            candidate.update(valid=False, lane_change_deadlocks={"car": "tiny_0"})
            path.write_text(json.dumps([receipt(), candidate]))
            result = subprocess.run([sys.executable, str(SOURCE / "score_evaluation.py"), str(path), "--out", str(out)],
                                    capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            report = json.loads(out.read_text())
            self.assertEqual(report["score"], 0)
            self.assertFalse(report["valid"])
            self.assertFalse(report["target_met"])

    def test_peak_bins_count_actual_crossings_and_offered_backlog(self):
        start = 16 * 3600
        events = [{"ramp": "on_ramp", "time": start + 10}, {"ramp": "on_ramp", "time": start + 1800},
                  {"ramp": "off_ramp", "time": start + 1799}]
        cohorts = {"on_ramp": {"a": start + 5, "b": start + 1800, "warm": start - 10}, "off_ramp": {"c": start + 20}}
        rows = evaluator.peak_bins(start, start + 9000, start + 9000, events, cohorts, {"a": start + 6})
        self.assertEqual(len(rows), 5)
        self.assertTrue(all(row["coverage_complete"] for row in rows))
        self.assertEqual(rows[0]["on_ramp"]["edge_exit_crossings"], 1)
        self.assertEqual(rows[1]["on_ramp"]["edge_exit_crossings"], 1)
        self.assertEqual(rows[0]["off_ramp"]["edge_exit_crossings"], 1)
        self.assertEqual(rows[0]["on_ramp"]["offered_due"], 1)
        self.assertEqual(rows[0]["on_ramp"]["boundary_backlog_start"], 1)
        self.assertEqual(rows[0]["on_ramp"]["observed_due_inserted"], 1)
        self.assertEqual(rows[0]["on_ramp"]["due_conservation_error"], 0)
        self.assertTrue(rows[2]["on_ramp"]["zero_crossings_with_demand"])

    def test_partial_bin_or_no_ramp_demand_never_claims_zero_crossing_collapse(self):
        start = 16 * 3600
        rows = evaluator.peak_bins(start, start + 600, start + 9000, [], {"on_ramp": {"a": start + 5}, "off_ramp": {}}, {})
        self.assertEqual(rows[0]["coverage_seconds"], 600)
        self.assertFalse(rows[0]["coverage_complete"])
        self.assertFalse(any(row["on_ramp"]["zero_crossings_with_demand"] for row in rows))
        rows = evaluator.peak_bins(start, start + 9000, start + 9000, [], {"on_ramp": {}, "off_ramp": {}}, {})
        self.assertFalse(any(row["on_ramp"]["zero_crossings_with_demand"] for row in rows))

    def test_empty_fairness_is_undefined_and_nonfinite_summary_fails(self):
        self.assertIsNone(evaluator.distribution([])["p95"])
        self.assertEqual(evaluator.distribution([0, 10, 20])["p95"], 19)
        row = receipt()
        row["fairness"] = {"wait": {"count": 1, "p95": float("nan"), "max": 1}}
        self.assertFalse(validity(row))

    def test_bin_conservation_failure_is_not_a_pass(self):
        row = receipt()
        row["peak_halfhour_bins"] = [{"vehicle_conservation_error": 1}]
        self.assertIn("bin_vehicle_conservation", validity_reasons(row))

    def test_network_semantic_hash_ignores_only_generated_comments(self):
        with tempfile.TemporaryDirectory() as directory:
            one, two = Path(directory) / "one.xml", Path(directory) / "two.xml"
            one.write_text('<net><!-- generated at first/path --><edge id="a"/></net>')
            two.write_text('<net><!-- generated at second/path --><edge id="a"/></net>')
            self.assertEqual(evaluator.semantic_network_hash(one), evaluator.semantic_network_hash(two))
            two.write_text('<net><!-- generated at second/path --><edge id="b"/></net>')
            self.assertNotEqual(evaluator.semantic_network_hash(one), evaluator.semantic_network_hash(two))

    def test_snapshot_verifies_copied_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "implementation").mkdir()
            path = root / "implementation" / "controller.py"
            path.write_bytes(b"frozen")
            hashes = {"implementation/controller.py": hashlib.sha256(b"frozen").hexdigest()}
            self.assertTrue(evaluator.verify_snapshot(root, hashes))
            path.write_bytes(b"changed")
            self.assertFalse(evaluator.verify_snapshot(root, hashes))

    def test_setup_failure_still_writes_partial_result_receipt(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            route = root / "routes.xml"
            route.write_text('<routes><route id="r" edges="25345887"/><vehicle id="v" route="r" depart="56700"/></routes>')
            network = root / "network.xml"
            network.write_text('<net/>')
            build = {"routes": str(route), "network": str(network), "args": []}
            case = ("fixed", "merge", 16, 42, 10, str(root), {"source": "hash"}, {}, True, "adaptive", "fixed")
            with patch.object(evaluator, "input_hashes", return_value={"source": "hash"}), \
                 patch.object(evaluator, "verify_snapshot", return_value=True), \
                 patch.object(evaluator, "ScenarioEditor") as editor, \
                 patch.object(evaluator.traci, "start", side_effect=RuntimeError("test simulator unavailable")):
                editor.return_value.build.return_value = build
                result = evaluator.run_case(case)
            self.assertFalse(result["valid"])
            self.assertIn("test simulator unavailable", result["evaluation_error"])
            saved = json.loads((root / "merge-16-42-fixed" / "result.json").read_text())
            self.assertEqual(result, saved)
            self.assertFalse(evaluator.batch_valid([result]))

    def test_collision_detail_failure_still_preserves_collision_and_removal(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            route = root / "routes.xml"
            route.write_text('<routes><route id="r" edges="25345887"/><vehicle id="a" route="r" depart="56700"/><vehicle id="b" route="r" depart="56700"/></routes>')
            network = root / "network.xml"
            network.write_text('<net/>')
            case = ("fixed", "merge", 16, 42, 10, str(root), {"source": "hash"}, {}, True, "adaptive", "fixed")
            now = 16 * 3600 - 899.8
            sim = {evaluator.tc.VAR_TIME: now, evaluator.tc.VAR_DEPARTED_VEHICLES_IDS: ["a", "b"],
                   evaluator.tc.VAR_ARRIVED_VEHICLES_IDS: [], evaluator.tc.VAR_COLLIDING_VEHICLES_NUMBER: 2,
                   evaluator.tc.VAR_TELEPORT_STARTING_VEHICLES_NUMBER: 0}
            controller = SimpleNamespace(settings=evaluator.Settings(), last_external={}, signals={},
                topology=SimpleNamespace(metadata=lambda: {}), status=lambda: {"signals": {}},
                vehicle_observations=lambda: {"a": ("25345887_0", 1, 0, 5)})
            with patch.object(evaluator, "input_hashes", return_value={"source": "hash"}), \
                 patch.object(evaluator, "verify_snapshot", return_value=True), \
                 patch.object(evaluator, "ScenarioEditor") as editor, \
                 patch.object(evaluator, "AdaptiveController", return_value=controller), \
                 patch.object(evaluator, "traci") as api:
                editor.return_value.build.return_value = {"routes": str(route), "network": str(network), "args": []}
                api.getVersion.return_value = (1, "fake")
                api.simulation.getSubscriptionResults.return_value = sim
                api.simulation.getTime.return_value = now
                api.simulation.getCollisions.return_value = [SimpleNamespace(collider="a", victim="b")]
                api.vehicle.getLaneID.side_effect = RuntimeError("vehicle removed")
                api.vehicle.getIDList.return_value = ["a"]
                result = evaluator.run_case(case)
            self.assertFalse(result["valid"])
            self.assertEqual(result["collision_vehicle_reports"], 2)
            self.assertEqual(result["unexplained_removals"], 1)
            self.assertEqual(result["missing_departed_vehicle_ids"], ["b"])
            self.assertIn("unavailable", result["collision_details"][0]["vehicles"]["b"])

    def test_worker_failure_receipt_is_retained(self):
        with tempfile.TemporaryDirectory() as directory:
            case = ("mpc", "merge", 16, 42, 150, directory, {}, {}, True, "intelligent", "fixed")
            row = evaluator.failed_case(case, RuntimeError("worker lost"))
            self.assertFalse(row["valid"])
            self.assertTrue((Path(directory) / "merge-16-42-mpc" / "result.json").exists())
            self.assertIn("evaluation_error", row["invalid_reasons"])


if __name__ == "__main__":
    unittest.main()
