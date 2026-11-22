#!/usr/bin/env python3
"""Collect logged episodes and fit the neighbourhood value model.

Two steps, both reproducible from the command line:

  collect  Run whole baseline-v2 evenings under a behaviour policy with a small
           share of random legal phase choices, and log every decision with the
           discounted area reward that followed it.
  fit      Fit one small network per observation set to those logged rewards.
           Every observation set gets the same episodes, targets, network size,
           optimiser settings and random seed; only the visible columns differ.

The fitted value is "what followed this choice under the logging policy", so
acting on its maximum is one policy-improvement step. Running ``collect`` again
with the fitted models as behaviour policies and refitting on all data repeats
that step (see neighborhood-ml/README.md for the exact rounds that were run).
"""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import hashlib
import json
from pathlib import Path
import time

import numpy as np

import baseline_v2_scenario as scenario
from neighborhood_ml import (DISCOUNT, FEATURE_NAMES, NEIGHBOUR_REWARD_WEIGHT, REWARD_HORIZON_TICKS, SENSING,
                             VARIANTS, reward_definition, weights_hash)

HIDDEN = (64, 64)
EPOCHS, BATCH, LEARNING_RATE, WEIGHT_DECAY, PATIENCE = 60, 2048, 1e-3, 1e-5, 8


def run(spec):
    from ml_episode import run_episode
    return run_episode(spec)


def collect(args):
    args.out.mkdir(parents=True, exist_ok=False)
    network = scenario.merged_network(args.out / "network")
    behaviours = []
    for item in args.behaviour:
        item, _, epsilon = item.partition("@")
        name, _, model = item.partition("=")
        if name == "ml" and not model:
            raise SystemExit("Use ml=<model.json> for a learned behaviour policy")
        behaviours.append((name, str(Path(model).resolve()) if model else None, float(epsilon) if epsilon else args.epsilon))
    specs = []
    for index, seed in enumerate(args.seeds):
        policy, model, epsilon = behaviours[index % len(behaviours)]
        label = policy if not model else "ml-" + Path(model).stem
        specs.append({"policy": policy, "model": model, "seed": seed, "label": label, "epsilon": epsilon,
                      "action_space": args.action_space,
                      "record": True, "network": str(network), "explore_seed": seed * 7919 + 1,
                      "folder": str(args.out / f"{label}-seed{seed}"), "end": scenario.BEGIN + args.minutes * 60})
    manifest = {"purpose": args.purpose, "epsilon": args.epsilon, "minutes": args.minutes,
                "inputs": scenario.input_hashes(), "episodes": [{k: v for k, v in s.items() if k != "network"} for s in specs]}
    (args.out / "collection.json").write_text(json.dumps(manifest, indent=1) + "\n")
    from ml_episode import describe
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        for future in as_completed([pool.submit(run, spec) for spec in specs]):
            print(json.dumps(describe(future.result())), flush=True)


def load(folders, tolerated=()):
    """Logged decisions from valid episodes, with an episode index per row.

    ``tolerated`` lists invalid reasons that do not exclude an episode from
    fitting (a logged collision does not remove vehicles in this scenario)."""
    features, returns, episode, explored, names = [], [], [], [], []
    spaces = set()
    for folder in folders:
        for path in sorted(Path(folder).glob("*/samples.npz")):
            result = json.loads((path.parent / "result.json").read_text())
            if set(result["invalid_reasons"]) - set(tolerated):
                print("skipping invalid episode", path.parent, result["invalid_reasons"], flush=True)
                continue
            data = np.load(path)
            spaces.add(result["controller"]["action_space"])
            features.append(data["features"])
            returns.append(data["returns"])
            explored.append(data["explored"])
            episode.append(np.full(len(data["returns"]), len(names)))
            names.append({"folder": str(path.parent), "seed": result["seed"], "policy": result["label"],
                          "samples": int(len(data["returns"])), "collisions": result["collisions"],
                          "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
    if not names:
        raise SystemExit("No valid logged episodes found")
    if len(spaces) != 1:
        raise SystemExit(f"Episodes mix action spaces {sorted(spaces)}; fit one at a time")
    names[0]["action_space"] = spaces.pop()
    return (np.concatenate(features), np.concatenate(returns).astype(np.float64), np.concatenate(episode),
            np.concatenate(explored), names)


def fit_network(train_x, train_y, val_x, val_y, seed):
    """Plain Adam on a Huber loss; the epoch with the lowest validation error is kept."""
    rng = np.random.default_rng(seed)
    sizes = (train_x.shape[1],) + HIDDEN + (1,)
    weights = [rng.normal(0, np.sqrt(2.0 / a), (a, b)) for a, b in zip(sizes, sizes[1:])]
    biases = [np.zeros(b) for b in sizes[1:]]
    moments = [[np.zeros_like(p), np.zeros_like(p)] for p in weights + biases]

    def forward(x):
        activations = [x]
        for index, (w, b) in enumerate(zip(weights, biases)):
            x = x @ w + b
            if index < len(weights) - 1:
                x = np.maximum(x, 0)
            activations.append(x)
        return activations

    best, best_error, stale, step, history = None, np.inf, 0, 0, []
    for epoch in range(EPOCHS):
        order = rng.permutation(len(train_x))
        for start in range(0, len(order), BATCH):
            rows = order[start:start + BATCH]
            activations = forward(train_x[rows])
            error = activations[-1][:, 0] - train_y[rows]
            gradient = (np.clip(error, -1, 1) / len(rows))[:, None]
            gradients_w, gradients_b = [], []
            for index in range(len(weights) - 1, -1, -1):
                gradients_w.append(activations[index].T @ gradient + WEIGHT_DECAY * weights[index])
                gradients_b.append(gradient.sum(0))
                if index:
                    gradient = (gradient @ weights[index].T) * (activations[index] > 0)
            step += 1
            parameters = weights + biases
            for parameter, grad, moment in zip(parameters, gradients_w[::-1] + gradients_b[::-1], moments):
                moment[0] = .9 * moment[0] + .1 * grad
                moment[1] = .999 * moment[1] + .001 * grad * grad
                parameter -= LEARNING_RATE * (moment[0] / (1 - .9 ** step)) / (np.sqrt(moment[1] / (1 - .999 ** step)) + 1e-8)
        validation = float(np.mean((forward(val_x)[-1][:, 0] - val_y) ** 2))
        history.append(round(validation, 5))
        if validation < best_error - 1e-5:
            best_error, stale = validation, 0
            best = [w.copy() for w in weights], [b.copy() for b in biases]
        else:
            stale += 1
            if stale >= PATIENCE:
                break
    return best[0], best[1], best_error, history


def fit(args):
    args.out.mkdir(parents=True, exist_ok=False)
    started = time.time()
    tolerated = ("collisions",) if args.keep_collision_episodes else ()
    train_x, train_y, train_episode, train_explored, train_names = load(args.train, tolerated)
    val_x, val_y, _, val_explored, val_names = load(args.validation, tolerated)
    overlap = {row["seed"] for row in train_names} & {row["seed"] for row in val_names}
    if overlap:
        raise SystemExit(f"Training and validation share simulation seeds {sorted(overlap)}")
    target_mean, target_std = float(train_y.mean()), float(train_y.std())
    y, vy = (train_y - target_mean) / target_std, (val_y - target_mean) / target_std
    report = {"created_unix": int(started), "seed": args.seed, "train_rows": int(len(y)), "validation_rows": int(len(vy)),
              "kept_collision_episodes": bool(args.keep_collision_episodes), "train_episodes": train_names, "validation_episodes": val_names, "models": {}}
    for variant in args.variants:
        columns = [FEATURE_NAMES.index(name) for name in VARIANTS[variant]]
        x, vx = train_x[:, columns].astype(np.float64), val_x[:, columns].astype(np.float64)
        mean, std = x.mean(0), x.std(0)
        std[std < 1e-6] = 1.0
        weights, biases, error, history = fit_network((x - mean) / std, y, (vx - mean) / std, vy, args.seed)
        payload = {
            "schema": 1, "kind": "neighbourhood-phase-value-mlp", "variant": variant, "scenario": scenario.SCENARIO,
            "action_space": train_names[0]["action_space"],
            "feature_names": list(VARIANTS[variant]), "mean": mean.tolist(), "std": std.tolist(),
            "layers": [{"W": w.tolist(), "b": b.tolist()} for w, b in zip(weights, biases)],
            "target": {"mean": target_mean, "std": target_std, "horizon_ticks": REWARD_HORIZON_TICKS,
                       "discount": DISCOUNT, "neighbour_weight": NEIGHBOUR_REWARD_WEIGHT,
                       "definition": reward_definition()},
            "sensing": {"observes": SENSING.get(variant, "See feature_names."), "measurement": SENSING["ideal"]},
            "training": {"hidden": list(HIDDEN), "epochs_run": len(history), "batch": BATCH,
                         "learning_rate": LEARNING_RATE, "weight_decay": WEIGHT_DECAY, "loss": "Huber(1) on standardised target",
                         "seed": args.seed, "train_rows": int(len(y)), "validation_rows": int(len(vy)),
                         "validation_mse_standardised": round(error, 5), "validation_mse_by_epoch": history,
                         "train_seeds": sorted({row["seed"] for row in train_names}),
                         "validation_seeds": sorted({row["seed"] for row in val_names}),
                         "inputs": scenario.input_hashes()},
        }
        payload["weights_sha256"] = weights_hash(payload)
        (args.out / f"{variant}.json").write_text(json.dumps(payload) + "\n")
        # Error on the randomly chosen actions alone: the part of the data where
        # the choice was not a function of the state.
        from neighborhood_ml import QModel
        predicted = (QModel(payload).predict(val_x) - target_mean) / target_std
        random_error = float(np.mean((predicted[val_explored] - vy[val_explored]) ** 2))
        report["models"][variant] = {"features": len(columns), "validation_mse": round(error, 5),
                                     "validation_r2": round(1 - error / float(np.var(vy)), 5),
                                     "validation_mse_random_actions": round(random_error, 5),
                                     "epochs_run": len(history), "weights_sha256": payload["weights_sha256"]}
        print(variant, json.dumps(report["models"][variant]), flush=True)
    report["seconds"] = round(time.time() - started, 1)
    (args.out / "fit-report.json").write_text(json.dumps(report, indent=1) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="command", required=True)
    c = commands.add_parser("collect", help="log episodes under a behaviour policy with exploration")
    c.add_argument("--out", type=Path, required=True)
    c.add_argument("--seeds", type=int, nargs="+", required=True)
    c.add_argument("--behaviour", nargs="+", default=["max_pressure_guarded", "cycle_guarded"],
                   help="policies cycled over the seeds: max_pressure_guarded, cycle_guarded, or ml=<model.json>; "
                        "append @0.1 to give one policy its own exploration share")
    c.add_argument("--epsilon", type=float, default=0.05, help="share of decisions replaced by a random legal phase")
    c.add_argument("--minutes", type=int, default=165, help="simulated minutes from 3:45 PM (165 = full evening)")
    c.add_argument("--jobs", type=int, default=8)
    c.add_argument("--purpose", default="train", choices=["train", "validation"])
    c.add_argument("--action-space", default="advance", choices=["advance", "phase"],
                   help="advance: keep or move to the next green in the fixed order; phase: any phase. "
                        "A learned behaviour policy uses the action space stored in its model file")
    c.set_defaults(run=collect)
    f = commands.add_parser("fit", help="fit one model per observation set")
    f.add_argument("--train", type=Path, nargs="+", required=True, help="collection folders used for fitting")
    f.add_argument("--validation", type=Path, nargs="+", required=True, help="collection folders used for early stopping")
    f.add_argument("--variants", nargs="+", choices=sorted(VARIANTS), default=["local", "neighborhood"])
    f.add_argument("--seed", type=int, default=0)
    f.add_argument("--keep-collision-episodes", action="store_true",
                   help="fit on episodes whose only defect is a logged collision (never used for reported results)")
    f.add_argument("--out", type=Path, required=True)
    f.set_defaults(run=fit)
    args = parser.parse_args()
    args.run(args)


if __name__ == "__main__":
    main()
