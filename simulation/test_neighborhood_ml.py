"""Checks for the learned neighbourhood controller (one short SUMO run included)."""
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from sumo_runtime import configure

configure()

import baseline_v2_scenario as scenario  # noqa: E402
from neighborhood_ml import (AREA_FEATURES, DOWNSTREAM_FEATURES, FEATURE_NAMES, LOCAL_FEATURES, QModel,  # noqa: E402
                             UPSTREAM_FEATURES, VARIANTS, weights_hash)
from train_neighborhood_ml import fit_network  # noqa: E402


def random_model(variant, seed=0):
    rng = np.random.default_rng(seed)
    names = VARIANTS[variant]
    sizes = (len(names), 8, 1)
    payload = {"variant": variant, "feature_names": list(names), "mean": [0.0] * len(names), "std": [1.0] * len(names),
               "layers": [{"W": rng.normal(0, .3, (a, b)).tolist(), "b": [0.0] * b} for a, b in zip(sizes, sizes[1:])],
               "target": {"mean": 0.0, "std": 1.0}, "training": {"train_seeds": [1], "validation_seeds": [2]}}
    payload["weights_sha256"] = weights_hash(payload)
    return payload


class Features(unittest.TestCase):
    def test_names_are_unique_and_local_is_a_strict_subset(self):
        self.assertEqual(len(FEATURE_NAMES), len(set(FEATURE_NAMES)))
        self.assertLess(set(VARIANTS["local"]), set(VARIANTS["neighborhood"]))
        self.assertEqual(VARIANTS["neighborhood"], FEATURE_NAMES)

    def test_local_view_has_no_neighbour_columns(self):
        outside = set(DOWNSTREAM_FEATURES) | set(UPSTREAM_FEATURES) | set(AREA_FEATURES)
        self.assertFalse(outside & set(VARIANTS["local"]))
        self.assertEqual(VARIANTS["local"], LOCAL_FEATURES)

    def test_local_model_ignores_neighbour_observations(self):
        model = QModel(random_model("local"))
        rng = np.random.default_rng(1)
        rows = rng.normal(size=(20, len(FEATURE_NAMES)))
        changed = rows.copy()
        changed[:, len(LOCAL_FEATURES):] += 100
        np.testing.assert_allclose(model.predict(rows), model.predict(changed))
        neighbourhood = QModel(random_model("neighborhood"))
        self.assertFalse(np.allclose(neighbourhood.predict(rows), neighbourhood.predict(changed)))


class Model(unittest.TestCase):
    def test_edited_weights_are_rejected(self):
        payload = random_model("local")
        payload["layers"][0]["W"][0][0] += 1
        with self.assertRaises(ValueError):
            QModel(payload)

    def test_unknown_feature_is_rejected(self):
        payload = random_model("local")
        payload["feature_names"][0] = "future_departures"
        payload["weights_sha256"] = weights_hash(payload)
        with self.assertRaises(ValueError):
            QModel(payload)

    def test_fit_learns_a_simple_function_and_predict_matches(self):
        rng = np.random.default_rng(0)
        x = rng.normal(size=(6000, 5))
        y = np.maximum(x[:, 0], 0) - .5 * x[:, 1] + .05 * rng.normal(size=6000)
        weights, biases, error, _ = fit_network(x[:5000], y[:5000], x[5000:], y[5000:], seed=0)
        self.assertLess(error, .05 * float(np.var(y)))
        hidden = x[5000:]
        for index, (w, b) in enumerate(zip(weights, biases)):
            hidden = hidden @ w + b
            if index < len(weights) - 1:
                hidden = np.maximum(hidden, 0)
        self.assertAlmostEqual(float(np.mean((hidden[:, 0] - y[5000:]) ** 2)), error, places=9)


class ShortRun(unittest.TestCase):
    """Three simulated minutes on baseline-v2 with a random-weight model."""

    @classmethod
    def setUpClass(cls):
        from ml_episode import run_episode
        cls.folder = tempfile.TemporaryDirectory(prefix="neighborhood-ml-test-")
        root = Path(cls.folder.name)
        network = scenario.merged_network(root / "network")
        model = root / "model.json"
        model.write_text(json.dumps(random_model("neighborhood")))
        cls.result = run_episode({"policy": "ml", "model": str(model), "seed": 1, "epsilon": .2, "record": True,
                                  "folder": str(root / "run"), "network": str(network), "end": scenario.BEGIN + 180})
        cls.samples = np.load(root / "run" / "samples.npz")

    @classmethod
    def tearDownClass(cls):
        cls.folder.cleanup()

    def test_run_is_valid_and_signal_rules_hold(self):
        self.assertTrue(self.result["valid"], self.result["invalid_reasons"])
        audit = self.result["signal_audit"]
        self.assertEqual(audit["violations"], 0)
        self.assertGreater(audit["checks"]["greens_ended"], 5)
        self.assertEqual(self.result["collisions"], 0)

    def test_vehicle_accounting_balances(self):
        self.assertEqual(self.result["conservation_error"], 0)
        self.assertEqual(self.result["due"], self.result["inserted"] + self.result["pending"])

    def test_logged_decisions_are_finite_and_complete(self):
        features = self.samples["features"]
        self.assertEqual(features.shape[1], len(FEATURE_NAMES))
        self.assertTrue(np.isfinite(features).all())
        self.assertTrue(np.isfinite(self.samples["returns"]).all())
        self.assertTrue(self.samples["explored"].any())
        keep = features[:, FEATURE_NAMES.index("is_keep")]
        np.testing.assert_array_equal(keep.astype(bool), self.samples["keep"])
        # Every logged decision happened after the eight-second minimum green.
        self.assertGreaterEqual(features[:, FEATURE_NAMES.index("green_elapsed")].min() * 50, 8 - 1e-3)

    def test_model_decided_and_latency_was_recorded(self):
        self.assertGreater(self.result["controller"]["model_decisions"], 0)
        self.assertGreater(self.result["latency_ms"]["decision_tick"]["count"], 30)
        self.assertIsNotNone(self.result["latency_ms"]["model_forward"]["p95"])


class KeepOrAdvance(unittest.TestCase):
    """The frozen models choose only between keeping and the next planned green."""

    def test_choices_are_keep_or_next_and_rules_hold(self):
        from ml_episode import run_episode
        with tempfile.TemporaryDirectory(prefix="neighborhood-ml-advance-") as folder:
            root = Path(folder)
            network = scenario.merged_network(root / "network")
            payload = random_model("neighborhood", seed=3)
            payload["action_space"] = "advance"
            payload["weights_sha256"] = weights_hash(payload)
            model = root / "model.json"
            model.write_text(json.dumps(payload))
            result = run_episode({"policy": "ml", "model": str(model), "seed": 2, "epsilon": .2, "record": True,
                                  "folder": str(root / "run"), "network": str(network), "end": scenario.BEGIN + 180})
            samples = np.load(root / "run" / "samples.npz")
        self.assertTrue(result["valid"], result["invalid_reasons"])
        self.assertEqual(result["controller"]["action_space"], "advance")
        self.assertEqual(result["signal_audit"]["violations"], 0)
        self.assertLessEqual(int(samples["candidates"].max()), 2)
        self.assertGreater(result["controller"]["model_decisions"], 0)


if __name__ == "__main__":
    unittest.main()
