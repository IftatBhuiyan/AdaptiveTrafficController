"""Fit a global ridge model for congestion-censored boundary insertion counts.

Examples::

    python simulation/train_forecaster.py --train train.jsonl --validation val.jsonl \
        --test test.jsonl --out model.json --report report.json
    python simulation/train_forecaster.py --data split_records.jsonl \
        --out model.json --report report.json

Each record has episode_id, lane, time, features, history, target, and optionally
split (train/validation/test) and episode validity fields. JSON objects containing
``records`` or ``episodes: [{episode_id, valid, records: [...]}]`` are also accepted.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from traffic_forecasting import (
    CENSORING, FEATURE_NAMES, HISTORY_BINS, INTERVAL_SECONDS, TARGET_DEFINITION,
    _model_prediction, _validate_model, _within_domain, make_features, simple_prediction,
)


ALPHAS = (0.1, 1.0, 10.0, 100.0)
SPLITS = ("train", "validation", "test")
VALIDITY_KEYS = ("valid", "validity", "simulation_valid", "episode_valid", "diagnostic_prefix", "failure_time", "invalid_reasons", "validation", "errors")


def _expand_records(value, inherited=None):
    inherited = dict(inherited or {})
    if isinstance(value, list):
        for item in value:
            yield from _expand_records(item, inherited)
        return
    if not isinstance(value, dict):
        raise ValueError("Dataset rows must be JSON objects")
    if "records" in value or "episodes" in value:
        inherited.update({key: item for key, item in value.items() if key not in ("records", "episodes")})
        yield from _expand_records(value.get("records", value.get("episodes")), inherited)
        return
    row = {key: item for key, item in inherited.items() if key in ("episode_id", "split") + VALIDITY_KEYS}
    row.update(value)
    row["_episode_metadata"] = {key: item for key, item in inherited.items() if key != "split"}
    row["_episode_metadata"].update({key: value[key] for key in VALIDITY_KEYS if key in value})
    yield row


def load_records(paths, split=None):
    """Read each file once, preserving its original record order and validity."""
    records, sources = [], []
    for filename in paths:
        path = Path(filename)
        raw = path.read_bytes()
        sources.append({"path": str(path.resolve()), "sha256": hashlib.sha256(raw).hexdigest()})
        text = raw.decode("utf-8")
        if path.suffix.lower() == ".jsonl":
            values = [json.loads(line) for line in text.splitlines() if line.strip()]
        else:
            values = json.loads(text)
        for row in _expand_records(values):
            declared = row.get("split")
            if split is not None and declared is not None and declared != split:
                raise ValueError(f"{path}: row split {declared!r} does not match {split!r}")
            row["split"] = split if split is not None else declared
            if row["split"] not in SPLITS:
                raise ValueError(f"{path}: every unassigned row requires train/validation/test split")
            records.append(row)
    return records, sources


def validate_records(records):
    last_time = {}
    for row in records:
        if not isinstance(row.get("episode_id"), str) or not row["episode_id"]:
            raise ValueError("Every row requires a nonempty string episode_id")
        if not isinstance(row.get("lane"), str) or not row["lane"]:
            raise ValueError("Every row requires a boundary-entry lane")
        target = float(row["target"])
        when = float(row["time"])
        features = [float(value) for value in row["features"]]
        if len(features) != len(FEATURE_NAMES) or not all(math.isfinite(value) for value in features):
            raise ValueError("Every row requires a finite, ordered feature vector")
        if not math.isfinite(target) or target < 0 or not math.isfinite(when):
            raise ValueError("Target counts and observation times must be finite; counts must be nonnegative")
        if row.get("feature_names", list(FEATURE_NAMES)) != list(FEATURE_NAMES):
            raise ValueError("Dataset feature order does not match FEATURE_NAMES")
        # Reconstruct every history/time feature without using the next-bin label.
        expected = make_features(row["history"], *features[4:7], now=when)
        if not all(math.isclose(actual, wanted, rel_tol=1e-8, abs_tol=1e-8)
                   for actual, wanted in zip(features, expected)):
            raise ValueError("Features do not match current/past history and observation time")
        key = row["episode_id"], row["lane"]
        if key in last_time and when <= last_time[key]:
            raise ValueError("Per-episode lane observations must be unique and chronological; no row shuffling")
        last_time[key] = when
        row["features"], row["target"], row["time"] = features, target, when


def validate_splits(datasets):
    owners = {}
    for split in SPLITS:
        rows = datasets[split]
        if not rows:
            raise ValueError(f"The {split} split must contain observations")
        validate_records(rows)
        for episode in {row["episode_id"] for row in rows}:
            if episode in owners:
                raise ValueError(f"Episode {episode!r} occurs in both {owners[episode]} and {split}")
            owners[episode] = split


def episode_metadata(rows):
    result = {}
    for row in rows:
        episode = row["episode_id"]
        metadata = row.get("_episode_metadata", {})
        combined = result.setdefault(episode, {})
        for key, value in metadata.items():
            if key in combined and combined[key] != value:
                raise ValueError(f"Conflicting episode metadata for {episode!r}: {key}")
            combined[key] = value
    return result


def split_metadata(rows, sources):
    return {"record_count": len(rows), "episode_ids": sorted({row["episode_id"] for row in rows}),
            "sources": sources, "episode_metadata": episode_metadata(rows)}


def fit_ridge(rows, alpha):
    """Standardize TRAIN only and leave the intercept unpenalized."""
    features = np.asarray([row["features"] for row in rows], dtype=np.float64)
    targets = np.asarray([row["target"] for row in rows], dtype=np.float64)
    mean = features.mean(axis=0)
    scale = features.std(axis=0)
    scale = np.where(scale > 1e-12, scale, 1.0)
    standardized = (features - mean) / scale
    intercept = float(targets.mean())
    coefficients = np.linalg.solve(standardized.T @ standardized + float(alpha) * np.eye(len(FEATURE_NAMES)),
                                   standardized.T @ (targets - intercept))
    model = {"schema_version": 1, "kind": "ridge", "feature_names": list(FEATURE_NAMES),
             "interval_seconds": INTERVAL_SECONDS, "history_bins": HISTORY_BINS,
             "alpha": float(alpha), "feature_mean": mean.tolist(), "feature_scale": scale.tolist(),
             "feature_min": features.min(axis=0).tolist(), "feature_max": features.max(axis=0).tolist(),
             "coefficients": coefficients.tolist(), "intercept": intercept,
             "target_definition": TARGET_DEFINITION, "censoring": CENSORING,
             "domain_policy": "TRAIN feature min/max, tolerance 1e-9; otherwise EWMA fallback"}
    return _validate_model(model)


def predictions(model, rows):
    values, supported = [], 0
    for row in rows:
        if _within_domain(model, row["features"]):
            values.append(_model_prediction(model, row["features"]))
            supported += 1
        else:
            values.append(simple_prediction(row["history"]))
    return values, {"learned_predictions": supported, "fallback_out_of_domain": len(rows) - supported}


def metrics(rows, values):
    targets = np.asarray([row["target"] for row in rows], dtype=np.float64)
    estimates = np.asarray(values, dtype=np.float64)
    residual = estimates - targets
    nonzero = targets > 0
    return {"record_count": len(rows), "mae": float(np.abs(residual).mean()),
            "rmse": float(np.sqrt(np.square(residual).mean())), "bias": float(residual.mean()),
            "observed_total": float(targets.sum()), "predicted_total": float(estimates.sum()),
            "nonzero_target_count": int(nonzero.sum()), "zero_target_count": int((~nonzero).sum()),
            "nonzero_prediction_count": int((estimates > 0).sum()),
            "nonzero_target_mae": float(np.abs(residual[nonzero]).mean()) if nonzero.any() else None}


def select_model(train, validation):
    candidates = []
    models = []
    for alpha in ALPHAS:
        model = fit_ridge(train, alpha)
        values, coverage = predictions(model, validation)
        candidates.append({"alpha": alpha, **metrics(validation, values), **coverage})
        models.append(model)
    # Stable first-minimum tie breaking keeps the predeclared alpha order.
    chosen = min(range(len(candidates)), key=lambda index: candidates[index]["rmse"])
    return models[chosen], candidates


def _write_exclusive(path, value):
    text = json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n"
    with Path(path).open("x", encoding="utf-8") as destination:
        destination.write(text)
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def train_and_report(train, validation, test_loader, out, report_path, sources, test_read_isolated=True):
    """Freeze model before invoking test_loader exactly once; never fit on TEST."""
    if Path(out).resolve() == Path(report_path).resolve():
        raise ValueError("Model and report destinations must differ")
    if Path(out).exists() or Path(report_path).exists():
        raise FileExistsError("Model and report destinations must both be new files")
    # Split disjointness is rechecked once TEST is loaded. TRAIN/validation are
    # checked before fitting, and a collision aborts without a comparison report.
    validate_records(train)
    validate_records(validation)
    if not train or not validation:
        raise ValueError("TRAIN and validation must both contain observations")
    train_ids = {row["episode_id"] for row in train}
    validation_ids = {row["episode_id"] for row in validation}
    if train_ids & validation_ids:
        raise ValueError("TRAIN and validation episodes must be disjoint")
    model, candidates = select_model(train, validation)
    model["metadata"] = {"data_split": {"train": split_metadata(train, sources["train"]),
                                           "validation": split_metadata(validation, sources["validation"])},
                         "selection": "Lowest validation RMSE with deployed support guard; TRAIN fit only",
                         "alpha_candidates": list(ALPHAS), "validation_scores": candidates,
                         "test_used_for_fit_or_selection": False,
                         "numpy_version": np.__version__}
    model_hash = _write_exclusive(out, model)
    test, test_sources = test_loader()
    datasets = {"train": train, "validation": validation, "test": test}
    validate_splits(datasets)
    learned, coverage = predictions(model, test)
    simple = [simple_prediction(row["history"]) for row in test]
    test_meta = split_metadata(test, test_sources)
    report = {"schema_version": 1, "model_path": str(Path(out).resolve()), "model_sha256": model_hash,
              "target_definition": TARGET_DEFINITION, "censoring": CENSORING,
              "data_split": {**model["metadata"]["data_split"], "test": test_meta},
              "selected_alpha": model["alpha"], "selection_metric": "validation RMSE",
              "test_read_after_model_frozen": test_read_isolated, "test_used_for_fit_or_selection": False,
              "simple": metrics(test, simple), "learned": {**metrics(test, learned), **coverage},
              "confidence_limitations": [
                  "The target is observed, congestion-censored insertion count, not offered demand.",
                  "Whole episodes are disjoint; rows within episodes and lanes remain correlated.",
                  "No confidence interval or controller efficacy is established by these forecast errors.",
                  "Unsupported TEST features use the same EWMA fallback as deployment.",
                  "Simulation validity is preserved below; invalid prefixes remain diagnostic evidence only."]}
    if not test_read_isolated:
        report["confidence_limitations"].append(
            "Combined --data inputs are parsed before fitting to partition rows; use separate TEST files for read isolation.")
    validity = [metadata.get("valid", metadata.get("simulation_valid"))
                for split in report["data_split"].values() for metadata in split["episode_metadata"].values()]
    report["evidence_status"] = ("observed_simulation_forecast_evaluation" if validity and all(item is True for item in validity)
                                 else "observed_data_diagnostic_validity_unproven_or_invalid")
    _write_exclusive(report_path, report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--data", nargs="+", type=Path, help="Files whose rows declare their split")
    for split in SPLITS:
        parser.add_argument("--" + split, nargs="+", type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    args = parser.parse_args(argv)
    if args.data and any(getattr(args, split) for split in SPLITS):
        parser.error("Use --data with declared splits, or the three explicit split flags")
    if not args.data and not all(getattr(args, split) for split in SPLITS):
        parser.error("Provide --train, --validation, and --test, or --data")
    try:
        if args.data:
            # A combined source must be decoded to obtain its declared partitions.
            # TEST rows stay unused until fitting/selection and freezing are done.
            rows, shared_sources = load_records(args.data)
            train = [row for row in rows if row["split"] == "train"]
            validation = [row for row in rows if row["split"] == "validation"]
            test_loader = lambda: ([row for row in rows if row["split"] == "test"], shared_sources)
            sources = {"train": shared_sources, "validation": shared_sources}
        else:
            train, train_sources = load_records(args.train, "train")
            validation, validation_sources = load_records(args.validation, "validation")
            test_loader = lambda: load_records(args.test, "test")
            sources = {"train": train_sources, "validation": validation_sources}
        report = train_and_report(train, validation, test_loader, args.out, args.report, sources,
                                  test_read_isolated=not bool(args.data))
    except (OSError, ValueError, KeyError, TypeError, OverflowError, np.linalg.LinAlgError) as error:
        parser.exit(2, f"Forecast training failed: {error}\n")
    print(json.dumps({"model": str(args.out), "report": str(args.report),
                      "selected_alpha": report["selected_alpha"], "test_simple_mae": report["simple"]["mae"],
                      "test_learned_mae": report["learned"]["mae"],
                      "evidence_status": report["evidence_status"]}, indent=2))


if __name__ == "__main__":
    main()
