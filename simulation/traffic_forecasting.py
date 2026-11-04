"""Forecast observed boundary insertions over the next five seconds.

Inserted vehicles are congestion-censored observations, not latent offered demand.
Only boundary entries use this predictor; internal arrivals belong to the conserved
traffic model and must not be added here a second time.
"""
import hashlib
import json
import math
from pathlib import Path


INTERVAL_SECONDS = 5
HISTORY_BINS = 12
EWMA_ALPHA = 0.35
FEATURE_NAMES = (
    "lag_1", "lag_2", "lag_3", "rolling_mean", "entry_occupancy",
    "entry_queue", "pending_per_entry", "time_sin", "time_cos",
)
TARGET_DEFINITION = "Observed vehicle insertion count at one boundary entry lane in the next 5 seconds."
CENSORING = (
    "Congestion can block insertion: observed insertions do not reveal latent offered demand. "
    "Features use current/past observations only; no routes or future departure schedule. "
    "Internal arrivals are derived by the conserved traffic model, not independently forecast."
)


def _nonnegative(value):
    result = float(value)
    if not math.isfinite(result) or result < 0:
        raise ValueError("Observed counts and entry state must be finite and nonnegative")
    return result


def _history(history):
    return [_nonnegative(value) for value in list(history)[-HISTORY_BINS:]]


def make_features(history, occupancy=0.0, queue=0.0, pending=0.0, now=0.0):
    """Use completed bins, oldest to newest; pending is already per entry lane.

    Occupancy and queue keep their observation units, which must remain consistent
    between collection and deployment. Missing lags are zero; the rolling mean
    covers available bins, up to one minute. ``now`` is simulation time in seconds.
    """
    counts = _history(history)
    when = float(now)
    if not math.isfinite(when):
        raise ValueError("Observation time must be finite")
    lags = [counts[-lag] if len(counts) >= lag else 0.0 for lag in (1, 2, 3)]
    angle = 2.0 * math.pi * (when % 86400.0) / 86400.0
    return lags + [sum(count / len(counts) for count in counts) if counts else 0.0,
                   _nonnegative(occupancy), _nonnegative(queue), _nonnegative(pending),
                   math.sin(angle), math.cos(angle)]


def simple_prediction(history):
    """A bounded-history EWMA; unusable observations contribute zero."""
    counts = []
    for value in list(history)[-HISTORY_BINS:]:
        try:
            counts.append(_nonnegative(value))
        except (TypeError, ValueError, OverflowError):
            counts.append(0.0)
    estimate = counts[0] if counts else 0.0
    for count in counts[1:]:
        estimate = EWMA_ALPHA * count + (1.0 - EWMA_ALPHA) * estimate
    return max(0.0, estimate) if math.isfinite(estimate) else 0.0


def _within_domain(model, features):
    # The support guard is frozen from TRAIN only. It never expands on validation,
    # test, or live observations, including features constant in training.
    return all(lower - 1e-9 <= value <= upper + 1e-9
               for value, lower, upper in zip(features, model["feature_min"], model["feature_max"]))


def _model_prediction(model, features):
    prediction = model["intercept"] + sum(
        coefficient * ((value - mean) / scale)
        for value, mean, scale, coefficient in zip(
            features, model["feature_mean"], model["feature_scale"], model["coefficients"]))
    if not math.isfinite(prediction):
        raise ValueError("Model produced a nonfinite prediction")
    return max(0.0, prediction)


def _validate_model(model):
    if model.get("schema_version") != 1 or model.get("kind") != "ridge":
        raise ValueError("Unsupported insertion-forecast model")
    if model.get("feature_names") != list(FEATURE_NAMES):
        raise ValueError("Forecast feature order does not match")
    if model.get("interval_seconds") != INTERVAL_SECONDS or model.get("history_bins") != HISTORY_BINS:
        raise ValueError("Forecast observation window does not match")
    if not isinstance(model.get("metadata", {}), dict):
        raise ValueError("Model metadata must be an object")
    for key in ("feature_mean", "feature_scale", "coefficients", "feature_min", "feature_max"):
        values = [float(value) for value in model[key]]
        if len(values) != len(FEATURE_NAMES) or not all(math.isfinite(value) for value in values):
            raise ValueError("Invalid model vector: " + key)
        model[key] = values
    if not all(value > 0 for value in model["feature_scale"]):
        raise ValueError("Model scales must be positive")
    if not all(lower <= upper for lower, upper in zip(model["feature_min"], model["feature_max"])):
        raise ValueError("Model support bounds are reversed")
    model["intercept"] = float(model["intercept"])
    if not math.isfinite(model["intercept"]):
        raise ValueError("Model intercept must be finite")
    return model


class ArrivalPredictor:
    """Nonnegative next-bin forecast, with explicit bounded fallback accounting."""

    def __init__(self, kind="simple", model_path=None):
        if kind not in ("simple", "learned", "ridge"):
            raise ValueError("Forecast kind must be simple or learned")
        self.kind = "learned" if kind == "ridge" else kind
        self.model = None
        self.model_sha256 = None
        self.model_error = None
        self.counters = {"predictions": 0, "simple_predictions": 0, "learned_predictions": 0,
                         "fallback_nonfinite_or_invalid": 0, "fallback_out_of_domain": 0,
                         "fallback_missing_or_invalid_model": 0, "fallback_nonfinite_prediction": 0}
        if self.kind == "learned":
            try:
                if model_path is None:
                    raise ValueError("A learned forecast requires a frozen model file")
                raw = Path(model_path).read_bytes()
                self.model_sha256 = hashlib.sha256(raw).hexdigest()
                self.model = _validate_model(json.loads(raw))
            except (OSError, ValueError, KeyError, TypeError, OverflowError) as error:
                self.model_error = str(error)

    @property
    def metadata(self):
        return {"kind": self.kind, "model_sha256": self.model_sha256,
                "model_error": self.model_error,
                "data_split": self.model.get("metadata", {}).get("data_split") if self.model else None,
                "training_metadata": self.model.get("metadata") if self.model else None,
                "target_definition": TARGET_DEFINITION, "censoring": CENSORING,
                "interval_seconds": INTERVAL_SECONDS, "history_bins": HISTORY_BINS,
                "ewma_alpha": EWMA_ALPHA, "counters": dict(self.counters)}

    def predict(self, history, occupancy=0.0, queue=0.0, pending=0.0, now=0.0):
        counts = list(history)
        self.counters["predictions"] += 1
        fallback = simple_prediction(counts)
        if self.kind == "simple":
            self.counters["simple_predictions"] += 1
            return fallback
        if self.model is None:
            self.counters["fallback_missing_or_invalid_model"] += 1
            return fallback
        try:
            features = make_features(counts, occupancy, queue, pending, now)
        except (TypeError, ValueError, OverflowError):
            self.counters["fallback_nonfinite_or_invalid"] += 1
            return fallback
        if not _within_domain(self.model, features):
            self.counters["fallback_out_of_domain"] += 1
            return fallback
        try:
            prediction = _model_prediction(self.model, features)
        except (TypeError, ValueError, OverflowError):
            self.counters["fallback_nonfinite_prediction"] += 1
            return fallback
        self.counters["learned_predictions"] += 1
        return prediction
