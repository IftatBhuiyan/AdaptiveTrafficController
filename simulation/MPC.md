# Network MPC and forecast experiment

This is a simulation research controller, not a field signal plan. It optimizes future phase schedules jointly over the loaded network. The search is restricted and local; it does not claim global optimality, stability, or collision-free microscopic operation.

## State and sensing

Use five-second intervals, a 30-second horizon (six intervals), and a 250 ms soft planning target. The state for each passenger-accessible external lane i consists of stock n_i, discharge-ready stock q_i, and delayed arrivals r_i(l). Moving stock is included in n_i exactly once; its ETA determines when it can enter q_i. Stopped vehicles far upstream also need travel time. Unambiguous internal connectors are assigned to their next external lane and remain counted. Ambiguous internal paths are not reconstructed from a vehicle's hidden route. Those vehicles remain visible to the independent evaluator and physical occupancy checks, but are not allocated to a lane in this fluid state. The model mass residual therefore checks represented stock, rather than proving complete microscopic fleet accounting.

The controller observes current vehicle lane/position/speed, lane subscriptions, current signal programs, actual completed transfers, waiting cohorts, and the current insertion backlog. It never reads the demand roster, future departures, or vehicle routes. The evaluator can read these separately for complete accounting.

The boundary set starts empty and adds actual insertion lanes as vehicles appear. A zero-indegree orphan lane is not an invented origin. Backlog distribution uses past insertion counts; until any origin is observed, unassigned pending stock remains visible as constant objective cost and a solver diagnostic. Network exit edges come from the static boundary definition, independently of individual vehicle routes. Observed insertion counts alone are congestion-censored and do not identify offered demand at each entry. Current total pending stock is observed, but its entry assignment is estimated; this implementation does not reconstruct an uncensored aggregate arrival process from pending-stock changes. Let lambda_i be a next-bin insertion forecast and b_i estimated existing pending stock. The forecast is a conservative admission envelope; it does not identify demand outside the network:

    a_i = max(0, lambda_i - b_i)
    admitted_i = min(b_i + a_i, lambda_i, max(0, C_i - n_i))
    b_i(next) = b_i + a_i - admitted_i

Thus five predicted insertions from five already-pending cars do not become ten new cars. Capping admissions by a censored forecast can also underpredict backlog discharge after an action opens receiving space; this is a limitation of the current counterfactual model. This deliberately makes no claim to recover unobserved offered arrivals. The next-bin insertion estimate is held constant across the six-step rollout and refreshed at the next observation; external forecasts are not recursively updated within that rollout. Exact system-time evaluation still includes every due-but-uninserted vehicle.

## Actions and transitions

The optimization is `min_{s_j in S_j for every actuator j} J(s_1,...,s_36)` subject to the dynamics, receiving, and transition constraints below. A decision variable selects one future schedule from each actuator's catalogue. There are 36 independent actuators for 37 multi-phase signals because the three-metre pair is one joint actuator. All other signals and stop/priority junctions remain observed context. Stop signs are never converted into signals. Native single-green signal gates are forecast from their currently observed program and next switch.

Each catalogue has at most eight schedules: legal continuation, up to three competing phases with immediate/ten-second switch choices, and a bounded second switch. Candidates start from actual elapsed green and outstanding transition commitments. Minimum green is eight seconds; the competing-demand maximum is fifty seconds, including elapsed history. Maximum-green dwell is rounded earlier where a five-second grid cannot hit the limit exactly. Yellow/all-red is rounded up in the planning model. Runtime clearance, occupied-link masking, and joint portal/tail checks remain authoritative; they may delay a predicted opening.

The joint programme restricts feeder greens to compatible receiving priority greens. New predicted transfers bypass zero-storage portal cells into the receiver's exit under both signal gates. Existing portal stock remains represented and must drain. The physical simulator, rather than this fluid approximation, determines internal trajectories and body overlap.

## Dynamics and receiving constraints

For each lane, geometric capacity C_i is length / 7.5 m. Initial overfull stock is retained for state accounting, but cannot create extra receiving capacity. With movement transfers f_ij and network exits e_i:

    n_i(k+1) = n_i(k) + admitted_i(k)
               + sum_j f_ji(k) - sum_j f_ij(k) - e_i(k)
    0 <= n_i(k+1) <= max(C_i, n_i(0))
    sum_j f_ji(k) <= max(0, C_i - n_i(k) - admitted_i(k) + outgoing_i(k))

Admissions consume receiving room before internal transfers are allocated. A currently blocked entrance sets both admission and incoming transfer capacity to zero. Shared lanes have one sending budget. Turn shares use past completed movements during full-green, unobstructed service, with declared straight/turn/U-turn priors. Priors are normalized by destination road before splitting lane fan-outs, so additional lane connections do not invent extra turning demand. Queued cars' intended routes are never read. For green-weighted serviceable probability p, the shared-lane head-prefix approximation is

    h(q,p) = p * (1 - p**q) / (1-p),  h(q,1) = q.

Sending is bounded by ready demand, saturation flow and this prefix. Imported yielding greens retain lower service permission; currently busy conflicting priority streams suppress yielding-service predictions. The planning model also masks currently occupied conflicting links. An observed blocked lane entrance remains closed in the forecast until a new observation; this is conservative and may underestimate future receiving space. Destination receiving space is allocated proportionally by a deterministic, monotone eight-pass supply calculation, starting with zero flow. Every intermediate allocation is feasible; finite propagation may underpredict discharge through long chains. A lane with no outgoing connector is not treated as an exit when other lanes of the same road continue.

Transfers enter destination physical stock immediately and its ready queue after a travel lag ceil(length / max(3 m/s, observed mean speed) / 5 s). This is a compact fluid approximation: it does not model individual car-following, exact turning order, pedestrians, or unsignalized gap acceptance. Residual error and live SUMO failures remain essential evidence.

## Objective and bounded solver

The implemented objective, in vehicle-second-like units with declared research weights, is

    J = 2 * number_of_schedule_state_changes
        + 5 * sum_k [ sum_i n_i + sum_i b_i + 0.5 * sum_i q_i
                      + 2 * sum_i max(0, n_i - 0.8*C_i)
                      + 0.25 * sum_i q_i * min(3, observed_age_i / 90) ]
        + 5 * [sum_i n_i(H) + 2 * sum_i b_i(H)].

Unassigned boundary backlog adds the constant 5*(H+2)*unassigned_pending to this objective; it is retained but cannot direct an actuator until its entry location is observed. The congestion term uses a one-vehicle normalization floor for very short lanes. The terminal term discourages postponing congestion beyond the horizon. Internal transfers do not earn fictitious completed-trip rewards. Waiting-age weights are soft priorities, not maximum-wait guarantees. Predicted waiting slack, observed continuous stopped time, incoming-lane service ages, and boundary waiting are reported separately.

A centralized coordinate search evaluates complete network trajectories for future schedules. It keeps a feasible incumbent, accepts strict objective improvements, makes up to two sweeps with at most 192 search evaluations, plus final validation and an optional one-step prediction, and rotates actuator order across decisions. Candidate construction, model evaluation, validation and dispatch are timed. A margin stops new candidate work before the deadline. This budget starts after collecting the current vehicle snapshot and advancing inherited transitions; the evaluator separately times the entire decision call. Python/OS scheduling and dispatch can overrun the budget, and every overrun remains visible rather than being a hard real-time guarantee. This is actual receding-horizon optimization over a restricted schedule set; it is not an exact mixed-integer solver. A SciPy MILP preflight was possible, but deterministic constrained flow simulation avoids assigning flow decisions the real green cannot execute. Distributed/hierarchical machinery was not added.

Apply only the first action, then observe and replan. Non-finite/unrealizable trajectories, deadline exhaustion, or a fresh mean prediction error above two vehicles per lane cause an explicit reference fallback. It advances a completed reference green through the same safety manager, retaining all pending clearance and portal commitments. Solver events report reason, total latency, evaluated candidates, actuator coverage, model mass residual, objective, observed signal states, planned schedules, and applied targets. RNG-inclusive rolling simulator checkpoints support failure replay. A feasible incumbent is not proof of good traffic performance.

## Forecast comparison

The simple predictor is a bounded-history EWMA (alpha 0.35). The learned predictor is a small standardized ridge regression. Inputs are three insertion-count lags, rolling mean, entry stock/queue, pending-per-entry estimate, and time-of-day sine/cosine. Its label is the next five-second observed insertion count. Internal arrivals are derived by conserved upstream flow, not forecast a second time.

For standardized TRAIN features Z and insertion labels y, the fitted coefficients solve

    (Z.T Z + alpha*I) beta = Z.T (y - mean_TRAIN(y))
    predicted_insertions = max(0, mean_TRAIN(y) + standardized_features.T beta).

The intercept is not penalized. Constant TRAIN feature columns use scale one. The deployed predictor uses EWMA whenever any current feature lies outside the TRAIN min/max support box.

Whole seed/condition episodes are split into training, validation, and untouched test sets. Standardization/support bounds are fitted on training only. Ridge alpha is chosen from 0.1, 1, 10, 100 on validation RMSE of the deployed support-guarded ridge/EWMA combination. Diagnostic-prefix records may enter fitting and validation; validity labels do not exclude those rows. The model is written and hashed before opening separate test files. Unsupported or invalid features fall back to the same EWMA, with explicit counters. Invalid simulation prefixes retain their diagnostic label; forecasting error measured on such data cannot establish controller efficacy. The saved collection snapshots used an earlier candidate-origin population. Before fitting, a causal adapter reconstructs histories and pending-per-entry values using only insertion counts already observed at each time, removes never-observed candidate origins, and preserves the raw archives and adapter hash. Collection also precedes the final portal geometry correction. This domain shift limits the comparison even with untouched test scenarios.

Use exactly the same optimizer, deadlines, objective, action catalogue, inputs and driving profile for MPC and learned MPC. Learning is experimental and not selected by the browser's MPC option unless evidence supports it. The baseline should remain simple if validation/test or closed-loop results do not support learning.

## Evaluation and use

Install the optional Python 3.12+ environment with `python -m pip install -r requirements-mpc.txt`; SUMO can use the existing installation. The original simulation remains available without installing the optional numeric dependency.

    python simulation/evaluate_controller.py --controller intelligent \
      --network-variant route-safe --driver-profile varied --initialization policy \
      --modes fixed max_pressure coordinated mpc --scenarios baseline merge \
      --hours 16 --seeds 42 --minutes 10 --jobs 3 --out experiments/new-development

For learned comparisons provide the frozen `--forecast-model` to the whole batch and add `mpc_learned`. Use `--minutes 150 --hours 16` for 16:00–18:30. `--collect-forecast` records boundary histories; `--collect-diagnostic-prefix` explicitly permits prefixes of invalid episodes and labels their failure time.

The published diagnostic artifact is `research/mpc-2026-10/arrival-ridge-diagnostic.json`. From the repository root, pass `--forecast-model research/mpc-2026-10/arrival-ridge-diagnostic.json --modes fixed max_pressure coordinated mpc mpc_learned` to compare all five policies with that frozen model. This artifact is retained for reproducibility; the results below do not justify enabling learned forecasting by default.

The CLI `route-safe` model variant is opt-in. It removes ingress to one wholly dead lane and changes keep-clear only on four connections at two single-road geometry continuations whose next segments cannot store a car. After the last network conversion, it also corrects a measured backward fold in seven portal lane geometry records, preserving lane IDs, connections, signal programs, and road-to-road demand paths. A controlled two-car lateral fixture reproduced a collision with the original geometry and completed both trips without collision using the final builder output. This is a model-defect regression check, not full-network safety proof. Physical lane counts, real intersection controls, and collision checking are preserved. The original saved network is never overwritten. Compare native original/model-repair diagnostics separately from controller effects.

The fixed comparator repeats the imported-duration reference with the existing common clearance and joint-pair safeguards; it is not an unmodified native SUMO program. Max pressure uses the existing ten-second queue-difference comparator with those same safeguards. The current heuristic is `coordinated` in the intelligent controller family. The heuristic also retains its existing native early-clearance proposals; MPC treats observed native timing as fixed forecast context. Native original-network probes remain a separate model diagnostic.

Scorer and evaluator now share validity reasons. Collision, teleport/removal, persistent strategic deadlock, incomplete coverage, changed snapshots, or broken accounting invalidates a run. The legacy `safe` field reports collision/teleport status; `valid` and its reasons are the acceptance gate including strategic stalls. Invalid runs and invalid fixed references score zero; they stay in the denominator. Ramp edge crossings and route completions are separate, with half-hour demand/backlog and coverage. A zero flow bin indicates collapse only when the bin is fully observed and demand/backlog is present. Source, generated network, demand, model and simulator receipts identify each experiment.

System and stopped vehicle-time cover the scored observation window. Fairness reports four distinct populations: vehicles stopped during observation, lanes with observed queues, vehicles still waiting at the insertion boundary, and vehicles admitted or still pending during observation. Continuous-stop and backlog ages can begin during warm-up; unfinished waits are censored at termination. They are observed delay distributions, not guarantees about eventual service. Predictor counters and receipt-level solver totals include warm-up; the analysis labels that scope and also reconstructs observation-only solver latency and fallbacks.

The browser uses the declared repaired network and varied drivers by default, and offers Network MPC explicitly as experimental. The command-line evaluator keeps original/nominal defaults, so comparisons must select the intended factors explicitly. Browser interaction, unit checks, short diagnostics and full simulations are separate evidence categories. Inspect actual local experiment receipts before claiming a performance or safety target.

## Seeded driver and route variation

`--driver-profile varied` is a separate scenario factor; `nominal` preserves the previous input generation. The browser exposes both and defaults to varied. Every controller in a matched batch receives the exact same sampled traffic. Seeds and assignment hashes make the realization reproducible.

Per-trip seeded profiles vary speed factors within 0.84–1.0 (zero speed noise), acceleration, comfortable deceleration, desired following headway, minimum gap, cooperation around each scenario's base value, conservative assertiveness, strategic lookahead, and lateral target offsets no larger than 0.10 m. Classes/dimensions and zero mainline speed-gain/keep-right settings are retained. Red-running, pushiness, collision disabling and teleporting are not introduced. These lawful parameter bounds still require microscopic validation; they do not make an unsafe network safe.

Route choices use legal alternatives with the same origin, exit, and exact ordered measured-station/cohort edges, bounded to 1.25 times original length. Explicit map-editor routes and mandatory stops retain their paths. Departures and vehicle counts do not change. Variation is sampled after surge generation and time filtering, so surge copies receive independently seeded profiles and the assignment receipt describes the written traffic population. In one full 14,094-trip generation check (seed 46), 8,044 trips had alternatives and 5,599 took another route; this is coverage of the generator, not measured real-world route choice.

The frozen protocol under the local `experiments/mpc-20261021` directory separates older nominal diagnostics, the additional varied-driver development/collection cases, and untouched varied-driver test cases. The 18:00–18:30 portion of a full run is a drain period after the saved base departures end. Ramp direction labels remain generic because the saved station GIS and older slides disagree about eastbound/westbound for the off-ramp proxy.

## Measured forecasting result

The frozen fit used 37,588 TRAIN observations from eight episodes and 16,740 validation observations from four episodes; validation selected alpha 0.1. The model was written at 04:21:32 UTC and TEST samples were first read at 04:54:59 UTC on October 22. The artifact SHA-256 is recorded in every comparison receipt. Neither the receiver-mask correction nor subsequent analysis refitted the model or selected parameters from TEST.

| Predictor | TEST MAE (vehicles / entry / 5 s) | TEST RMSE | Nonzero-label MAE | Total predicted / observed |
| --- | ---: | ---: | ---: | ---: |
| EWMA, alpha 0.35 | 0.31732 | 0.46210 | 0.69759 | 2626.15 / 2482 |
| Guarded ridge, alpha 0.1 | 0.28793 | 0.41612 | 0.64724 | 2496.87 / 2482 |

These 8,380 correlated observations come from two invalid episodes, with 1,945 nonzero labels. Every TEST feature timestamp is between 15:45:05.2 and 15:57:05.2; next-bin labels remain before 16:00. Later samples were excluded when a strategic stall was detected. Thus the approximately 9.3% MAE and 10.0% RMSE improvements describe diagnostic warm-up forecasting, not peak-period or field efficacy. All of these particular TEST features were within TRAIN support. The limited TRAIN time-of-day support also forces later peak-period inputs outside the learned support box; deployed forecasting then uses EWMA. Full-run counters report that exposure explicitly. This dataset therefore cannot establish a learned peak-period forecasting contribution.

An independent reconstruction reproduced both error reports exactly. A separate algebra check verified TRAIN standardization, the saved ridge normal equations (maximum residual 2.96e-12), matrix-product predictions, and closed-form EWMA weights, without refitting. The raw archives, causal reconstruction recipe, frozen model, and separate `train-observed.jsonl`, `validation-observed.jsonl`, and `test-observed.jsonl` are retained locally. Final-network held-out condition errors and closed-loop traffic results remain separate analyses. The browser's MPC uses the simple predictor; learned forecasting is an explicit research comparison only.

## Full-period comparison

All 20 held-out full-period cases finished 16:00–18:30: five policies, baseline and merge demand, seeds 201 and 202. Every comparison uses the frozen receiver-mask-corrected implementation, repaired network, and identical seeded varied drivers. Each policy starts from the same empty 15:45 network and controls its own warm-up. These are continuous-operation comparisons; their 16:00 queues can differ because of earlier policy decisions.

**None of the 20 full-period cases passes validity.** All have long strategic stalls; none reports a collision or teleport. Invalid fixed references make every paired acceptance score zero. These measurements remain descriptive evidence of failed cases.

| Condition | Seed | Policy | Completed exits | System vehicle-hours | Stopped vehicle-hours | Pending | Flagged vehicles |
| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: |
| baseline | 201 | fixed | 4367 | 14258.36 | 3034.64 | 7536 | 1 |
| baseline | 201 | max_pressure | 4330 | 14411.30 | 2956.77 | 7633 | 2 |
| baseline | 201 | coordinated | 4379 | 14380.20 | 2940.87 | 7620 | 2 |
| baseline | 201 | mpc | 4336 | 14483.58 | 3075.81 | 7611 | 2 |
| baseline | 201 | mpc_learned | 4380 | 14396.64 | 3015.24 | 7593 | 5 |
| baseline | 202 | fixed | 4424 | 14181.72 | 2939.61 | 7536 | 3 |
| baseline | 202 | max_pressure | 4380 | 14398.47 | 3056.06 | 7566 | 3 |
| baseline | 202 | coordinated | 4416 | 14227.42 | 2934.99 | 7556 | 3 |
| baseline | 202 | mpc | 4373 | 14379.26 | 3082.02 | 7555 | 4 |
| baseline | 202 | mpc_learned | 4361 | 14498.07 | 3065.44 | 7621 | 3 |
| merge | 201 | fixed | 12890 | 15997.46 | 3024.63 | 7575 | 2 |
| merge | 201 | max_pressure | 12875 | 16018.93 | 2989.81 | 7606 | 1 |
| merge | 201 | coordinated | 12881 | 15897.98 | 2951.19 | 7600 | 4 |
| merge | 201 | mpc | 12855 | 16135.59 | 3138.73 | 7592 | 4 |
| merge | 201 | mpc_learned | 12917 | 15993.13 | 3129.17 | 7514 | 7 |
| merge | 202 | fixed | 12973 | 16237.10 | 2960.74 | 7547 | 4 |
| merge | 202 | max_pressure | 12919 | 16371.97 | 3063.22 | 7578 | 2 |
| merge | 202 | coordinated | 12897 | 16264.04 | 3011.27 | 7603 | 2 |
| merge | 202 | mpc | 12904 | 16320.40 | 3089.54 | 7580 | 4 |
| merge | 202 | mpc_learned | 12901 | 16447.56 | 3084.04 | 7581 | 3 |

Summing the four matched full-period cases per policy:

| Policy | Valid / 4 | Completed exits | System vehicle-hours | Stopped vehicle-hours | Whole-decision p95 range, ms |
| --- | ---: | ---: | ---: | ---: | ---: |
| fixed | 0 | 34654 | 60674.63 | 11959.62 | 18.7–27.7 |
| max_pressure | 0 | 34504 | 61200.65 | 12065.86 | 10.3–15.1 |
| coordinated | 0 | 34573 | 60769.64 | 11838.33 | 39.3–62.0 |
| mpc | 0 | 34468 | 61318.84 | 12386.10 | 226.5–228.7 |
| mpc_learned | 0 | 34559 | 61335.40 | 12293.89 | 226.3–228.4 |

Simple MPC accumulated 1.06% more system vehicle-time than fixed; learned MPC accumulated 1.09% more. Learned MPC was 0.027% worse than simple MPC on this total. These small differences in invalid runs do not support an efficacy claim or adoption of learning. Across the four learned-MPC full cases, 25,828 forecasts used ridge and 241,822 used the domain fallback (90.4%). These counters include warm-up.

All 20 cases recorded **zero off-ramp crossings**, including all 80 fully observed half-hour bins after 16:30 with demand or boundary backlog. The 18:00–18:30 drain period also failed to restore flow. Ramp route completions, on-ramp and downstream crossings, fairness populations and waiting ages are separately reported in the per-case CSV.

The 15 held-out short stress cases cover disorder, surge and incident, seed 203, ten scored minutes after warm-up. All finished their windows with zero collision reports or teleports. Only the simple- and learned-MPC surge cases were valid; the other 13 failed the stall gate. Their fixed references are invalid, so neither valid short candidate establishes an accepted paired score.

| Policy | Valid / 3 | Completed exits | System vehicle-hours | Stopped vehicle-hours | Flagged vehicle cases |
| --- | ---: | ---: | ---: | ---: | ---: |
| fixed | 0 | 3062 | 751.30 | 322.76 | 4 |
| max_pressure | 0 | 3075 | 748.96 | 333.77 | 4 |
| coordinated | 0 | 3058 | 748.32 | 318.13 | 5 |
| mpc | 1 | 3020 | 755.82 | 333.39 | 5 |
| mpc_learned | 1 | 3017 | 752.63 | 333.08 | 5 |

The final-network fixed-policy forecast exports for all seven held-out conditions contain **no fully peak-period targets**; surge/incident exports also contain no post-perturbation targets. Their invalid-prefix errors do not test the advertised stress regime. Empty windows remain undefined. The original seed-201 warm-up comparison and repeated seed-201 fixed cases are not independent replications.

Independent checks matched exits against SUMO trip XML, verified due = inserted + pending, frozen inputs and matched settings, and audited all 17,640 logged solver events. Applied nonfallback plans respected the search budget, represented-stock conservation and non-increasing model objective. Across full and stress runs, observation solver logs contain three fallbacks and seven deadline-exceeded flags. Full-period whole-call maxima were 263.3 ms for simple MPC and 309.7 ms for learned MPC. The 250 ms target is soft; SUMO waits during computation.

The optimizer contributes future schedule search with finite receiving capacity and conserved flow. That computational contribution did not translate into a measured full-period traffic gain here. The learned model improves warm-up insertion error, but limited data support and frequent fallback do not establish a peak-period contribution. The browser retains simple forecasting.

Committed handoff artifacts: [all 45 development/full/stress case metrics](../research/mpc-2026-10/case-metrics.csv), [frozen diagnostic ridge model](../research/mpc-2026-10/arrival-ridge-diagnostic.json), and [protocol, input hashes and provenance](../research/mpc-2026-10/provenance.json). Raw failed cases, checkpoints and the complete independent audit remain in the local ignored experiment directory.

## Verification and unresolved failures

The final local Python suite has 95 passes, zero failures, and zero skips; the JavaScript geometry suite has eight passes, zero failures, and zero skips. These include regression checks for the scorer/deadlock disagreement, conserved receiving flow, delayed travel, pending-demand duplication, geometry repair persistence, seeded route legality, explicit editor waypoints, and final surge-population receipts.

A frozen dense diagnostic completed 21 MPC decisions with 602 active vehicles, zero fallbacks/collision reports/teleports, 222.4–227.9 ms planning latency, and maximum model mass residual 2.27e-13 vehicles. The search covered 16–32 actuators per decision and reduced its predicted objective by 0.41% on average (0.07–1.16%). These are improvements within the planning model, not measured full-period traffic benefits. This diagnostic includes the receiver occupied-link mask on both no-storage gate paths. The separate seed-42 development comparison predates that final mask correction and remains labeled development evidence.

Browser verification reached 16:06:10 with 835 active vehicles and 302 adaptive changes, then paused. The displayed last decision took 223.37 ms. Controller/driver selectors, live decisions, and pause worked with no captured console warnings/errors. This earlier browser session precedes the last receiver-mask backend correction and does not supply final-backend or full-period acceptance; no fresh browser pass is claimed for publication.

Two collision mechanisms were independently reproduced in older saved cases: curved SL2015 taper/body overlap remains unresolved; an old signal-clearance case opened conflicting movement while a stopped car occupied the connector. Current occupied-link masking addresses the latter mechanism, but transition constraints alone do not establish collision freedom. The portal smoothing regression is a separate model correction.

A native five-change diagnostic reached 18:30 with zero collision reports, yet had only 26 observation-window off-ramp crossings and zero after 16:30. Correcting the local keep-clear freeze did not restore the whole ramp corridor. The final seven-lane geometry variant also failed to restore off-ramp flow in the matched full-period matrix above.

The earlier dead-lane-ingress deletion also leaves an upstream orphan lane (`683926428#0_2`) with no outgoing connection while another road still admits traffic into it. Saved source tags indicate two left lanes, whereas converted turn connectivity supplies one. The receiving road and its merged source ways have no lane-count or width tags; the single 3.2 m receiver is an unverified import assumption. Adding a second receiving lane would invent capacity, and a second turn into the existing lane would invent a merge. Verified striping/as-built or field imagery is needed to choose a defensible repair. This network-fidelity defect is unresolved and remains in the frozen comparison. Other flagged stalls involve very short turning pockets and residual lane-choice blockage. Three of nine flagged vehicles in the first five audited nominal training/validation receipts later arrived; the detector establishes a long urgent strategic stall, not a permanent deadlock in every case. Frozen invalid flags remain visible and unchanged.

A further public-source check did not resolve the receiving layout. The exact [DCP LION segment 0346182](https://services5.arcgis.com/GfwWNkhOj9bNBqoJ/ArcGIS/rest/services/LION/FeatureServer/0/query?where=OBJECTID%3D201040&outFields=*&returnGeometry=true&outSR=4326&f=pjson) lists two travel lanes total on a two-way street, without a lane-survey date. [DOT's April 2026 proposal, page 14](https://www.nyc.gov/html/dot/downloads/pdf/31-st-northern-blvd-20-ave-apr2026-2.pdf#page=14) depicts two southbound lanes at this block, but is a proposed design rather than an as-built record; its typical existing section explicitly excludes this crossing. [The 2014 diagrams](https://www.nyc.gov/html/dot/downloads/pdf/2014-05-07-astoria-boulevard-qn-cb1.pdf#page=7) leave the receiving striping obscured. These sources do not establish the current lane-to-lane connection. Their records and limits are retained with the failure diagnosis; the benchmark remains unchanged.

Eight cases from a superseded development snapshot were cancelled incomplete after the final portal correction; their partial artifacts and cancellation receipt remain preserved, with zero acceptance passes. Older development, training/validation prefixes, final diagnostics, and held-out comparisons are separate evidence classes. No skipped or cancelled case becomes a pass.

Failure replays, independent geometry fixtures, raw receipts, the frozen protocol/implementations, exact forecast reconstruction recipe, tests, and the browser screenshot are under the local `experiments/mpc-20261021` directory. Astra reviewed the formulation and experimental plan twice: its first review required executable flow, common validity/accounting, committed transitions/fallback, identifiable insertion targets, measured deadlines, and honest fairness limits. The revised bounded plan was approved. The implementation remains the root agent's responsibility.

## Sources and project context

The October 21 meeting (especially page 31), Week 5 script/slides, pasted side proposal, earlier project chats, PR #2 and saved failed experiments informed this implementation. Earlier scores describe their frozen versions, not this controller.

- Capacity limits and congestion propagation: [Gregoire et al., Capacity-aware back-pressure control](https://arxiv.org/abs/1309.6484).
- [SUMO safety documentation](https://sumo.dlr.de/docs/Simulation/Safety.html) distinguishes signal conflicts and sublane/body collisions.
- [NYC DOT APS inventory](https://data.cityofnewyork.us/Transportation/Accessible-Pedestrian-Signal-Locations/de3m-c5p4) and [Street Sign Work Orders](https://data.cityofnewyork.us/Transportation/Street-Sign-Work-Orders/qt6m-xctn) support the map reviews; exact queried records are preserved in `map_control_review.json`.
