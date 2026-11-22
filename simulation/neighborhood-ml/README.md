# Learned signal control with a neighbourhood view

A small learned controller for the Astoria Blvd / I-278 signals, built to answer
one question: **does a controller make better signal decisions when it can see
a few blocks around its intersection than when it sees only its own
approaches?** The same model, data, target and training settings are used for
both; only the visible observations differ.

## Results (held-out test, frozen models)

**Short answer.** On 8 unseen evenings, the learned controller with the
neighbourhood view had lower whole-system vehicle-time than fixed time running
through the same safety layer on all 8 (median −356 vehicle-hours, about
−4.4%; exact sign test p = 0.008), and it was the only policy with no
gridlocked evening. Against the scenario's own fixed-time plan run directly by
SUMO, which is itself a little better than the safety-layer version, it was
lower on 7 of 8 evenings (median −215, about −2.7%; p = 0.07). Against the local-only version of the **same** model, the
neighbourhood view made no statistically detectable difference in typical
evenings (better on 5 of 8 seeds, p = 0.73). It did avoid both evenings where
the local model gridlocked, but the neighbourhood model itself gridlocked on
1 of 3 validation evenings, so robustness is suggestive, not established.
Both learned models carry about 40% fewer off-ramp vehicles than fixed time.

Protocol: models frozen at 23:52 on 2026-11-18 (`models/freeze.json`, hashes
checked again after the test). Test seeds 3, 5, 7, 11, 13, 21, 42 and 99 were
never used for fitting, early stopping or model selection. Full evening
3:45–6:30 PM, no exploration, every policy on the same seed sees identical
demand. 48 runs, all complete. Full tables: `results/test-report.md`; every
run's metrics: `results/test-runs.json`.

**System vehicle-hours, including vehicles waiting to enter** (lower is better)

| Seed | Fixed (SUMO) | Fixed (safety layer) | Max pressure | Max pressure + rules | Learned local | Learned neighbourhood |
|---|---|---|---|---|---|---|
| 3 | 13,732 † | 8,241 † | 15,756 † | 9,445 † | 7,616 | 7,751 |
| 5 | 7,892 | 7,966 † | 13,133 † | 9,055 | 8,406 † | 7,689 |
| 7 | 7,862 | 8,086 | 14,693 † | 10,246 † | 7,711 | 7,636 |
| 11 | 7,988 | 7,913 | 15,723 † | 10,374 † | 7,872 | 7,659 |
| 13 | 7,713 | 8,249 † | 15,841 † | 9,349 | 8,046 | 7,843 |
| 21 | 7,828 | 8,050 | 15,176 † | 9,169 | 7,559 | 7,564 |
| 42 | 7,857 | 7,974 | 14,323 † | 9,202 | 13,806 † | 7,689 |
| 99 | 7,910 † | 8,164 | 14,375 † | 10,746 † | 7,668 | 7,857 |
| **Median** | 7,877 | 8,068 | 14,934 | 9,397 | 7,792 | **7,689** |
| Evenings with > 100 vehicles stuck 3+ min at 6:30 PM (†) | 2 | 3 | 8 | 4 | 2 | **0** |

Outcomes are bimodal: an evening either clears (about 7,500–8,300) or locks up
around the interchange block (13,000+), so medians and per-seed values are
more informative than means. With 8 seeds a two-sided sign test reaches
p < 0.05 only when one policy wins all 8.

**Paired by seed** (system vehicle-hours)

| Comparison | Seeds lower / higher | Median difference | Sign-test p |
|---|---|---|---|
| Neighbourhood vs local (same model, the research question) | 5 / 3 | −139 | 0.73 |
| Neighbourhood vs fixed (same safety layer) | 8 / 0 | −356 | 0.008 |
| Local vs fixed (same safety layer) | 6 / 2 | −289 | 0.29 |
| Neighbourhood vs fixed (SUMO runs the plan) | 7 / 1 | −215 | 0.07 |
| Neighbourhood vs max pressure + rules | 8 / 0 | −1,649 | 0.008 |
| Max pressure + rules vs fixed (same safety layer) | 0 / 8 | +1,215 | 0.008 |

"Fixed (safety layer)" isolates the learned decisions: same phase list, same
minimum green, yellow, all-red and box-clearance code, only the timing
decision differs. "Fixed (SUMO)" is the scenario's own reference, with SUMO
running the Webster programmes and no controller in the loop.

**Other measures** (median over the 8 seeds unless stated)

| Policy | Completed trips | Stopped veh-h | Spillback lane-h | Off-ramp vehicles | Longest stop, p95 (s) | Vehicles ever stopped > 3 min | Signal-rule violations (total) | Lane collisions (total) | Junction overlap reports (total) | Decision tick p95 (ms) | Model p95 (ms) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Fixed (SUMO) | 11,429 | 791 | 43.4 | 1,761 | 549 | 990 | – | 4 | 84 | – | – |
| Fixed (safety layer) | 11,364 | 774 | 44.4 | 1,640 | 426 | 1,135 | 0 | 0 | 67 | 33 | – |
| Max pressure | 5,876 | 2,720 | 133.2 | 148 | 9,144 | 1,468 | 0 | 0 | 5 | 35 | – |
| Max pressure + rules | 10,157 | 964 | 50.2 | 524 | 576 | 1,750 | 0 | 0 | 37 | 58 | – |
| Learned local | 11,494 | 614 | 38.2 | 1,190 | 280 | 926 | 0 | 0 | 47 | 67 | 0.19 |
| Learned neighbourhood | 11,418 | 527 | 32.5 | 967 | 203 | 441 | 0 | 0 | 26 | 71 | 0.22 |

### Where neighbourhood information helped, hurt, or made no difference

- **No measurable difference in typical evenings.** When both learned models
  cleared (6 seeds), their system vehicle-time was within about 3% of each
  other: neighbourhood lower on 3, local lower on 3.
- **Helped on the two bad evenings.** On seeds 5 and 42 the local model let the
  block around the interchange lock up (689 and 1,387 vehicles stuck at 6:30);
  the neighbourhood model kept it clear. It also had about half as many
  vehicles stuck over 3 minutes (441 vs 926) and fewer junction overlap reports.
- **Not reliably.** On validation seed 212 the neighbourhood model gridlocked
  (14,178) while the local model did not (7,606). Across all 11 unseen
  evenings (3 validation + 8 test), the neighbourhood model gridlocked on 1 and
  the local model on 2. That is too few events to call a difference.
- **Prediction, offline.** On the same validation data, adding observation
  groups to the same network raises the variance explained in the training
  target from R² 0.516 (local) to 0.527 (+downstream), 0.554 (+upstream) and
  0.611 (full neighbourhood). The neighbourhood columns, mostly upstream and
  area queues, do predict consequences better. That is offline evidence, not
  a closed-loop result.
- **Hurt the off-ramp.** The neighbourhood model passed the fewest off-ramp
  vehicles of any working policy (median 967 over the evening, vs 1,640 for
  fixed time and 1,190 for the local model).

### What the learned controllers actually do, and what it costs

Both learned controllers hold back the off-ramp traffic heading into the 31st
St block (off-ramp flow drops from about 300 to about 160–190 vehicles per half
hour). This appears to be how they keep the Astoria Blvd N → 31st St →
Astoria Blvd S → 33rd St loop from filling, which is the gridlock baseline-v2's
README describes; the per-approach green times were not logged, so that
mechanism is inferred from flows and gridlock counts, not checked directly.
Total entry backlog at the end of the evening is about the same as fixed time
(median 3,835 vs 3,743 vehicles not yet entered; the extra ~90 waiting
vehicles are a small cost next to the −356 vehicle-hour difference). On the real site that waiting
would happen on I-278. The baseline-v2 off-ramp already carries only about 38%
of the NYSDOT count under fixed time, so a controller that reduces it further
should not be called an improvement for the ramp. A deployable version would
need an explicit off-ramp queue limit or a weight on freeway spillback in its
objective.

### Validity, safety and limits

- **Signal rules.** `SignalAudit` checked every displayed state in every
  controller run: 0 violations of minimum green, yellow, all-red, clearance or
  legal phases across all 40 controlled test runs.
- **Collisions.** 0 lane collisions in any controlled test run; 4 under the
  unmodified fixed-time scenario. "Junction overlap" reports (two vehicles
  overlapping inside a junction, from `--collision.check-junctions`) occur
  under every policy, including unmodified fixed time (84 over 8 evenings).
  They are counted, not treated as invalidating, and their cause in the network
  was not investigated. This is simulator evidence only, not proof of safety.
- **Reference check.** The test harness reproduces `baseline-v2/check_run.py`
  exactly on seed 3 (6,506 arrivals, 1,231 stuck at 6:30 PM in both). That
  evening gridlocks under fixed time, while the baseline-v2 README lists seed 3
  as clearing; the README's runs may predate a later file change. It affects
  all policies equally.
- **Latency.** Decision ticks took 67–71 ms at p95 while ten simulations shared
  the machine; almost all of that is computing observations. The network
  itself takes about 0.2 ms. Decisions are every 5 s.
- **Sensing.** All observations are exact simulator values. Real detectors
  would be noisier and sparser, and the neighbourhood view needs data sharing
  between controllers.
- **Scope.** One network, one demand profile, eight test evenings and one
  evening period. The demand is the team's baseline-v2 estimate, not a
  calibrated day.

## Scenario

All training and testing uses **baseline-v2** (`simulation/baseline-v2/`): the
network and evening demand where the off-ramp keeps flowing under fixed time.
The older editable network (`astoria-osm-import`) is not used here; its
full-evening runs all failed validity in `simulation/MPC.md`.

`baseline_v2_scenario.py` launches it with the settings of
`baseline-v2/check_run.py` (0.2 s steps, no teleports, 30 s impatience) and
embeds the Webster signal timing from `baseline-v2-signals.add.xml` into a copy
of the network, so the fixed-time reference and the controllers read the same
programmes. Nothing under `baseline-v2/` is changed.

One addition: `--collision.check-junctions true` with `--collision.action warn`.
This only reports. It showed that the unmodified fixed-time scenario itself
produces "junction" overlap reports (two vehicles overlapping inside a
junction, about 20 events per evening), which `check_run.py` does not look for.
These are counted per policy as *junction overlap reports* and kept apart from
lane collisions. Their cause in the network was not investigated.

## Design

### Controlled signals and legal actions

The controller subclasses `IntelligentCoordinator`, so it uses the existing
phase list and safety layer unchanged. On baseline-v2 that is 36 signals with
two or more green phases (81 single-phase signals keep their fixed programmes).

- **Action (keep or advance):** every 5 s, each signal that is green and past
  its minimum green chooses between keeping its current green and moving to
  the next green of the fixed plan's order. The model sets how long each green
  lasts; the order of phases stays the fixed plan's. A free choice among all
  phases (`--action-space phase`) is still in the code but lost to fixed time
  on validation (see *Development history*).
- **Outside the model** (the model cannot change these): the phase list, 8 s
  minimum green, yellow, 2 s all-red, the occupied-box clearance and link
  holding in `adaptive_controller.py`, and two fairness rules in
  `NeighborhoodController.legal_candidates`:
  - *maximum green:* after max(50 s, that phase's fixed-plan green), the
    current phase is removed from the choices if another approach is waiting;
  - *service gap:* if a waiting approach has not been served for max(90 s, the
    signal's fixed-plan cycle), the signal must advance.
- `ml_episode.SignalAudit` reads every displayed state back from SUMO and
  checks these rules independently in every run.

### Neighbourhood

For a signal, the neighbourhood is:

- its **receiving roads**: each exit lane followed downstream to the next
  signal (controlled or not) or 650 m;
- its **feeders**: the signal movements that release traffic toward each of its
  approaches;
- roads **further upstream** than its own approach detectors reach (beyond
  250 m, back to the previous signal or the map edge);
- the controlled signals **one and two hops** away on the road graph.

### Observations

59 numbers per (signal, candidate phase), all present or past values:

| Group | Count | What | In `local` | In `neighborhood` |
|---|---|---|---|---|
| Own signal | 7 | keep/switch, green elapsed, lost time, phase count, plan green, time since candidate last ran, relief phase | yes | yes |
| Own approaches | 24 | for lanes the candidate serves, lanes it does not, and lanes the current phase serves: stopped vehicles, moving vehicles, arrivals expected in 10 s and 30 s, fullest lane, longest time unserved, recent arrival rate, queue change over 30 s | yes | yes |
| Downstream | 12 | receiving roads of served and unserved movements: mean and worst occupancy, share of demand facing a blocked entrance, least free space, whether the next signal is green for them, stopped share | no | yes |
| Upstream | 10 | feeders of served and unserved lanes: their queue bound for us, the part of it that currently has green, departures in the last 10 s; vehicles and delay beyond the local 250 m | no | yes |
| Area | 6 | queue and occupancy at signals one and two hops away | no | yes |

History enters through the 30 s queue change, the smoothed arrival rate, the
10 s departure counts and the time since each phase last ran.

Not observed: vehicle routes, scheduled departures, and the number of vehicles
waiting to enter the map.

These are **simulator-perfect** values: exact per-lane counts, stopped counts,
mean speeds and vehicle positions. A real installation would need stop-line and
advance detectors for the local group, exit detectors for the downstream group,
and shared data from adjacent controllers for the rest, each with noise this
project has not modelled.

### Learning objective

The measure of success is whole-system vehicle-time including vehicles that are
due but cannot yet enter. The training target is that quantity cut into areas:

- *Area load* of a signal = vehicles on the roads upstream of it (back to the
  previous signal or the map edge) + vehicles waiting to enter on those roads
  + 0.5 × the same for directly connected controlled signals.
- *Target* for a decision = minus the discounted sum, over the next 80 s
  (16 ticks, discount 0.95), of the change in area load since the decision.

The entry backlog appears only in this target. It is never a model input.

### Learning method

A single network (inputs → 64 → 64 → 1, plain NumPy) scores a
(signal, candidate phase) pair. The same weights serve all 36 signals.

1. **Collect.** Run whole evenings under a behaviour policy in which 10–20% of
   decisions are replaced by a random choice between keeping and advancing. Log each decision's
   59 numbers and the target that followed.
2. **Fit.** Regress the target on the logged numbers. Choosing the highest
   predicted value is then one policy-improvement step over the behaviour
   policy.
3. **Repeat** with the fitted models as behaviour policies, refitting on all
   data.

This was chosen over deeper reinforcement learning because it is the smallest
method that learns from the consequences of signal decisions in this
simulator: no bootstrapped targets, no replay tuning, a few dozen simulated
evenings, and it trains in minutes on a laptop. A graph network or multi-agent
RL was not needed to test the question; the neighbourhood enters as extra
input columns.

### Development history

Recorded so the final design is not mistaken for the first attempt. All of
these used only training and validation seeds.

1. **First target: stopped vehicles near the signal.** Fitted well
   (validation R² 0.98), but the stopped-vehicle count ignored vehicles that
   could not enter the map, which is most of the system delay here. Replaced by
   the area-load target above. Archived in
   `experiments/neighborhood-ml/superseded-reward-v1/`.
2. **Free phase choice.** With the area-load target and any phase allowed,
   both models lost clearly to fixed time on validation seeds 211–212
   (system vehicle-hours: local 14,969 and 11,605; neighbourhood 10,651 and
   10,301; fixed time 9,766 and 7,737). A likely cause, not checked against
   the logged decisions, is that leaving the plan's phase order broke the
   progression between adjacent signals. Archived in
   `experiments/neighborhood-ml/superseded-free-phase-choice/` and
   `validation-round0/`.
3. **Keep or advance, three rounds.** Round 0 learned from fixed-order
   behaviour with exploration; rounds 1 and 2 learned from the previous
   round's models. Validation system vehicle-hours (fixed time ≈ 7,930–8,020):

   | Round | Local, seeds 211/212/213 | Neighbourhood, seeds 211/212/213 |
   |---|---|---|
   | 0 | 9,293 / 9,832 / – | 8,293 / 8,614 / – |
   | 1 | 7,718 / 7,732 / 7,735 | 8,107 / 11,377 / 11,140 |
   | 2 | 7,792 / 7,606 / 7,693 | 7,636 / 14,178 / 7,586 |

   Round 2 was frozen for both models. The selection rule in
   `models/freeze.json` was written after these validation numbers were seen;
   it was written before any test seed was run.

### Holding everything else constant

`local` and `neighborhood` models share: episodes, targets, network size,
optimiser, epochs, early-stopping rule, random seed, the phase list and every
safety and fairness rule. `fit` reads one logged dataset and gives each model a
different subset of columns.

## Files

| File | Purpose |
|---|---|
| `../neighborhood_ml.py` | Observations, model loading, `NeighborhoodController` |
| `../baseline_v2_scenario.py` | baseline-v2 launch settings and merged network |
| `../ml_episode.py` | Runs one evening, measures it, audits signal rules |
| `../train_neighborhood_ml.py` | `collect` and `fit` |
| `../evaluate_neighborhood_ml.py` | Matched comparison on unseen seeds |
| `../test_neighborhood_ml.py` | Unit checks and a 3-minute SUMO run |
| `models/local.json`, `models/neighborhood.json` | Frozen models used by the browser |
| `results/` | `test-report.md` (all tables) and `test-runs.json` (every test run's metrics) |
| `make_report.py` | Builds those two files from evaluation folders |

Raw run folders (about 2 MB of logged decisions per episode) are in the
git-ignored `experiments/neighborhood-ml/`.

## Reproduce

From the repository root. Python 3.12+, NumPy (`requirements-mpc.txt`) and SUMO 1.27.1.
One evening takes about 11 minutes alone and about 20 minutes when ten run at
once on a 10-core Mac. `E=experiments/neighborhood-ml`.

```sh
# Round 0: fixed-plan order with 10% and 20% random keep/advance choices
python3 simulation/train_neighborhood_ml.py collect --out $E/adv-round0-train-a --seeds 101 102 103 104 --behaviour cycle_guarded@0.1 cycle_guarded@0.2 --jobs 4
python3 simulation/train_neighborhood_ml.py collect --out $E/adv-round0-train-b --seeds 105 106 107 108 --behaviour cycle_guarded@0.1 cycle_guarded@0.2 --jobs 4
python3 simulation/train_neighborhood_ml.py collect --out $E/adv-round0-validation --purpose validation --seeds 201 202 --behaviour cycle_guarded@0.1 cycle_guarded@0.2 --jobs 2
python3 simulation/train_neighborhood_ml.py fit --train $E/adv-round0-train-a $E/adv-round0-train-b --validation $E/adv-round0-validation --out $E/adv-models-round0

# Round 1: the round-0 models drive, 10% random choices; refit on everything
python3 simulation/train_neighborhood_ml.py collect --out $E/adv-round1-train --seeds 111 112 113 114 --behaviour ml=$E/adv-models-round0/local.json@0.1 ml=$E/adv-models-round0/neighborhood.json@0.1 --jobs 4
python3 simulation/train_neighborhood_ml.py fit --train $E/adv-round0-train-a $E/adv-round0-train-b $E/adv-round1-train --validation $E/adv-round0-validation --out $E/adv-models-round1

# Round 2
python3 simulation/train_neighborhood_ml.py collect --out $E/adv-round2-train --seeds 121 122 123 124 125 126 --behaviour ml=$E/adv-models-round1/local.json@0.1 ml=$E/adv-models-round1/neighborhood.json@0.1 --jobs 6
python3 simulation/train_neighborhood_ml.py fit --train $E/adv-round0-train-a $E/adv-round0-train-b $E/adv-round1-train $E/adv-round2-train --validation $E/adv-round0-validation --out $E/adv-models-round2

# Choose the round on validation seeds (no exploration). Runs actually made:
#   adv-validation-round0: fixed, round-0 models, seeds 211 212
#   adv-validation-round1: round-1 models, seeds 211 212
#   adv-validation-round1b: fixed, round-1 models, seed 213
#   adv-validation-round2: round-2 models, seeds 211 212 213
# Equivalent for the frozen round:
python3 simulation/evaluate_neighborhood_ml.py --purpose validation --policies r2_local=$E/adv-models-round2/local.json r2_neighborhood=$E/adv-models-round2/neighborhood.json --seeds 211 212 213 --jobs 6 --out $E/adv-validation-round2
cp $E/adv-models-round2/local.json $E/adv-models-round2/neighborhood.json $E/adv-models-round2/fit-report.json simulation/neighborhood-ml/models/

# Held-out test: seeds no model was fitted, early-stopped or selected on.
# Run as made: five policies first (interrupted by a restart after 12 runs and
# continued with --resume), then guarded max pressure added with --resume.
P="fixed_native fixed max_pressure max_pressure_guarded ml_local=simulation/neighborhood-ml/models/local.json ml_neighborhood=simulation/neighborhood-ml/models/neighborhood.json"
python3 simulation/evaluate_neighborhood_ml.py --purpose test --policies $P --seeds 3 5 7 11 13 21 42 99 --jobs 10 --out $E/test-frozen-round2
# (add --resume to continue an interrupted batch; finished runs with the same inputs and model are kept)

# Tables in the Results section
python3 simulation/neighborhood-ml/make_report.py $E/test-frozen-round2 --out simulation/neighborhood-ml/results

# Offline ablation (validation R² by observation group)
python3 simulation/train_neighborhood_ml.py fit --train $E/adv-round0-train-a $E/adv-round0-train-b $E/adv-round1-train $E/adv-round2-train --validation $E/adv-round0-validation --variants local local_downstream local_upstream neighborhood --out $E/adv-ablation-round2

# Reference check against the team's script
python3 simulation/baseline-v2/check_run.py --seed 3

# Tests
cd simulation && python3 -m unittest test_neighborhood_ml
```

`evaluate` refuses seeds a model was fitted or early-stopped on. Each run folder
holds `result.json` with every metric, input hashes, the model hash, the
signal-rule audit and collision events.

## Browser demonstration

```sh
python3 run.py --scenario baseline-v2
```

The page opens on baseline-v2 with **Learned · neighbourhood view** selected.
The **Network** menu switches between baseline-v2 and the editable Astoria
network; the controller menu offers the learned neighbourhood model, the
learned local model, max pressure and fixed time on baseline-v2. As before,
the first 15 minutes are fixed-time warm-up and the chosen controller takes
over at 4 pm. The controller panel shows the model id, what it observes, each
central signal's last decision with the two scores, how many decisions the
service rules forced, and decision time. Map editing, traffic scenarios and
driver profiles belong to the editable network and are disabled on
baseline-v2. `python3 run.py` without the flag starts the editable network as
before.

The browser uses seed 42. On that evening the held-out test had the local
model gridlock (1,387 vehicles stuck at 6:30 PM) while the neighbourhood model
cleared. The browser adds a 15-minute fixed-time warm-up before 4 PM, so its
run is not identical to the test run and the outcome may differ.
