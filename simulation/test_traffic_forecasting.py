"""Plain-logic checks for the insertion forecaster; no SUMO or efficacy claim."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from traffic_forecasting import (
    ArrivalPredictor, FEATURE_NAMES, HISTORY_BINS, INTERVAL_SECONDS,
    make_features, simple_prediction,
)
from train_forecaster import (
    ALPHAS, fit_ridge, load_records, select_model, train_and_report, validate_splits,
)


def record(episode, queue=0.0, target=1.0, when=0.0, valid=True):
    history = [1.0, 1.0, 1.0]
    return {"episode_id": episode, "lane": "boundary_0", "time": when, "history": history,
            "features": make_features(history, queue=queue, now=when), "target": target,
            "_episode_metadata": {"valid": valid}}


class ForecastingTests(unittest.TestCase):
    def test_features_are_bounded_past_counts_and_time_of_day(self):
        history = list(range(1, 31))
        features = make_features(history, occupancy=12, queue=3, pending=2, now=21600)
        self.assertEqual(features[:4], [30.0, 29.0, 28.0, 24.5])
        self.assertEqual(features[4:7], [12.0, 3.0, 2.0])
        self.assertAlmostEqual(features[7], 1.0)
        self.assertAlmostEqual(features[8], 0.0)
        self.assertEqual(make_features([2])[:4], [2.0, 0.0, 0.0, 2.0])
        self.assertEqual(simple_prediction([999] + history), simple_prediction(history[-HISTORY_BINS:]))
        self.assertTrue(all(np.isfinite(make_features([1e308] * 12))))

    def test_train_only_scaling_and_validation_only_alpha_selection(self):
        train = [record(f"train_{index}", queue=index, target=10 * index) for index in range(4)]
        validation = [record("validation", queue=3, target=15)]
        model, scores = select_model(train, validation)
        self.assertEqual(model["feature_mean"][5], 1.5)
        self.assertAlmostEqual(model["feature_scale"][5], np.std([0, 1, 2, 3]))
        self.assertEqual(model["feature_scale"][4], 1.0)  # Constant occupancy remains safe.
        self.assertEqual(model["alpha"], 100.0)
        self.assertEqual([row["alpha"] for row in scores], list(ALPHAS))
        different_validation = [record("validation", queue=3, target=30)]
        alternate, _ = select_model(train, different_validation)
        self.assertEqual(alternate["alpha"], 0.1)
        self.assertEqual(model["feature_mean"], alternate["feature_mean"])
        self.assertEqual(model["feature_max"], alternate["feature_max"])

    def test_clipping_domain_guard_and_nonfinite_fallback_are_explicit(self):
        rows = [record("first", queue=0), record("second", queue=4)]
        model = fit_ridge(rows, 1.0)
        model["intercept"] = -10.0
        model["coefficients"] = [0.0] * len(FEATURE_NAMES)
        with tempfile.TemporaryDirectory() as temporary:
            model_path = Path(temporary) / "model.json"
            model_path.write_text(json.dumps(model))
            predictor = ArrivalPredictor("learned", model_path)
            self.assertEqual(predictor.predict([1, 1, 1], queue=2), 0.0)
            self.assertEqual(predictor.predict([1, 1, 1], queue=5), 1.0)
            self.assertEqual(predictor.predict([1, 1, 1], queue=float("nan")), 1.0)
            self.assertTrue(np.isfinite(predictor.predict([float("inf"), 1, 1])))
            counters = predictor.metadata["counters"]
            self.assertEqual(counters["learned_predictions"], 1)
            self.assertEqual(counters["fallback_out_of_domain"], 1)
            self.assertEqual(counters["fallback_nonfinite_or_invalid"], 2)
            self.assertEqual(predictor.metadata["model_sha256"], hashlib.sha256(model_path.read_bytes()).hexdigest())
        missing = ArrivalPredictor("learned")
        self.assertEqual(missing.predict([2, 2]), 2.0)
        self.assertEqual(missing.counters["fallback_missing_or_invalid_model"], 1)

    def test_whole_episode_collisions_and_future_feature_leakage_are_rejected(self):
        splits = {"train": [record("same")], "validation": [record("same")], "test": [record("other")]}
        with self.assertRaisesRegex(ValueError, "both train and validation"):
            validate_splits(splits)
        leaked = record("test", target=9)
        leaked["features"][0] = leaked["target"]
        splits = {"train": [record("train")], "validation": [record("validation")], "test": [leaked]}
        with self.assertRaisesRegex(ValueError, "current/past"):
            validate_splits(splits)
        splits["test"] = [record("test", when=10), record("test", when=5)]
        with self.assertRaisesRegex(ValueError, "chronological"):
            validate_splits(splits)

    def test_nested_episode_validity_and_declared_split_are_preserved(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "episodes.json"
            row = record("invalid_prefix")
            row.pop("_episode_metadata")
            path.write_text(json.dumps({"episodes": [{"episode_id": "invalid_prefix", "valid": False,
                                                       "invalid_reasons": ["lane_change_deadlocks"],
                                                       "split": "test", "records": [row]}]}))
            rows, sources = load_records([path])
            self.assertFalse(rows[0]["_episode_metadata"]["valid"])
            self.assertEqual(rows[0]["_episode_metadata"]["invalid_reasons"], ["lane_change_deadlocks"])
            self.assertEqual(sources[0]["sha256"], hashlib.sha256(path.read_bytes()).hexdigest())
            with self.assertRaisesRegex(ValueError, "does not match"):
                load_records([path], "train")

    def test_test_is_loaded_once_after_freeze_and_cannot_change_model_selection(self):
        train = [record(f"train_{index}", queue=index, target=10 * index) for index in range(4)]
        validation = [record("validation", queue=3, target=30)]
        test = [record("test", queue=2, target=1000, valid=False)]
        with tempfile.TemporaryDirectory() as temporary:
            out, report_path = Path(temporary) / "model.json", Path(temporary) / "report.json"
            calls = []

            def test_loader():
                self.assertTrue(out.exists())
                frozen = json.loads(out.read_text())
                self.assertEqual(frozen["alpha"], 0.1)
                self.assertNotIn("test", frozen["metadata"]["data_split"])
                calls.append(1)
                return test, [{"path": "test.jsonl", "sha256": "diagnostic-test-source"}]

            report = train_and_report(train, validation, test_loader, out, report_path,
                                      {"train": [], "validation": []})
            self.assertEqual(len(calls), 1)
            self.assertEqual(report["selected_alpha"], 0.1)
            self.assertEqual(report["simple"]["record_count"], report["learned"]["record_count"])
            self.assertEqual(report["simple"]["nonzero_target_count"], 1)
            self.assertEqual(report["evidence_status"], "observed_data_diagnostic_validity_unproven_or_invalid")
            self.assertFalse(report["data_split"]["test"]["episode_metadata"]["test"]["valid"])
            self.assertEqual(report["model_sha256"], hashlib.sha256(out.read_bytes()).hexdigest())
            with self.assertRaises(FileExistsError):
                train_and_report(train, validation, test_loader, out, report_path,
                                 {"train": [], "validation": []})
            self.assertEqual(len(calls), 1)
            self.assertEqual(json.loads(out.read_text())["interval_seconds"], INTERVAL_SECONDS)


if __name__ == "__main__":
    unittest.main()
