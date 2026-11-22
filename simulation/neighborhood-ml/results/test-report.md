Held-out seeds: 3, 5, 7, 11, 13, 21, 42, 99. Full evening 3:45–6:30 PM, no exploration, frozen models.
Seeds with every policy finished: 8 of 8.

**System vehicle-hours incl. vehicles waiting to enter (lower is better)**

| Seed | fixed_native | fixed | max_pressure | max_pressure_guarded | ml_local | ml_neighborhood |
|---|---|---|---|---|---|---|
| 3 | 13,732 | 8,241 | 15,756 | 9,445 | 7,616 | 7,751 |
| 5 | 7,892 | 7,966 | 13,133 | 9,055 | 8,406 | 7,689 |
| 7 | 7,862 | 8,086 | 14,693 | 10,246 | 7,711 | 7,636 |
| 11 | 7,988 | 7,913 | 15,723 | 10,374 | 7,872 | 7,659 |
| 13 | 7,713 | 8,249 | 15,841 | 9,349 | 8,046 | 7,843 |
| 21 | 7,828 | 8,050 | 15,176 | 9,169 | 7,559 | 7,564 |
| 42 | 7,857 | 7,974 | 14,323 | 9,202 | 13,806 | 7,689 |
| 99 | 7,910 | 8,164 | 14,375 | 10,746 | 7,668 | 7,857 |
| **median** | 7,877 | 8,068 | 14,934 | 9,397 | 7,792 | 7,689 |

**Completed trips (higher is better)**

| Seed | fixed_native | fixed | max_pressure | max_pressure_guarded | ml_local | ml_neighborhood |
|---|---|---|---|---|---|---|
| 3 | 6,506 | 11,334 | 5,534 | 10,086 | 11,591 | 11,412 |
| 5 | 11,473 | 11,422 | 6,713 | 10,445 | 10,001 | 11,392 |
| 7 | 11,524 | 11,360 | 5,979 | 8,757 | 11,491 | 11,465 |
| 11 | 11,398 | 11,412 | 5,551 | 8,619 | 11,498 | 11,480 |
| 13 | 11,447 | 10,229 | 5,498 | 10,228 | 11,385 | 11,378 |
| 21 | 11,512 | 11,353 | 5,772 | 10,353 | 11,625 | 11,484 |
| 42 | 11,411 | 11,429 | 6,136 | 10,324 | 6,394 | 11,425 |
| 99 | 11,394 | 11,368 | 6,122 | 8,171 | 11,614 | 11,257 |
| **median** | 11,429 | 11,364 | 5,876 | 10,157 | 11,494 | 11,418 |

**Vehicles stopped > 3 min at 6:30 PM (over 100 = gridlock)**

| Seed | fixed_native | fixed | max_pressure | max_pressure_guarded | ml_local | ml_neighborhood |
|---|---|---|---|---|---|---|
| 3 | 1,231 | 159 | 1,420 | 200 | 0 | 0 |
| 5 | 23 | 106 | 1,330 | 5 | 689 | 0 |
| 7 | 23 | 0 | 1,460 | 1,069 | 0 | 3 |
| 11 | 23 | 65 | 1,411 | 1,084 | 0 | 5 |
| 13 | 23 | 577 | 1,424 | 0 | 0 | 1 |
| 21 | 24 | 32 | 1,404 | 0 | 0 | 5 |
| 42 | 78 | 34 | 1,434 | 0 | 1,387 | 0 |
| 99 | 168 | 0 | 1,438 | 969 | 0 | 0 |
| **median** | 24 | 50 | 1,422 | 102 | 0 | 0 |

**Summary per policy** (medians over seeds where every policy finished)

| Policy | Runs | Over 100 stuck at 6:30 PM | Stopped veh-h | Spillback lane-h | Off-ramp vehicles | Longest stop p95 (s) | Vehicles stopped > 3 min | Lane collisions | Junction overlap reports | Signal-rule violations | Decision tick p95 (ms) | Model p95 (ms) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Fixed time (SUMO runs the plan) | 8 | 2 | 791 | 43.4 | 1,761 | 549 | 990 | 4 | 84 | – | – | – |
| Fixed time (same safety layer as ML) | 8 | 3 | 774 | 44.4 | 1,640 | 426 | 1,135 | 0 | 67 | 0 | 33.4 | – |
| Max pressure (no max-green/service-gap rules) | 8 | 8 | 2,720 | 133.2 | 148 | 9,144 | 1,468 | 0 | 5 | 0 | 34.7 | – |
| Max pressure + same service rules as ML | 8 | 4 | 964 | 50.2 | 524 | 576 | 1,750 | 0 | 37 | 0 | 57.7 | – |
| Learned, local view | 8 | 2 | 614 | 38.2 | 1,190 | 280 | 926 | 0 | 47 | 0 | 67.0 | 0.19 |
| Learned, neighbourhood view | 8 | 0 | 527 | 32.5 | 967 | 203 | 441 | 0 | 26 | 0 | 71.0 | 0.22 |

**Paired by seed** (system vehicle-hours; negative = candidate lower)

| Candidate vs reference | Seeds | Candidate lower / higher | Median difference | Exact sign-test p |
|---|---|---|---|---|
| ml_neighborhood vs ml_local | 8 | 5 / 3 | -139 | 0.727 |
| ml_neighborhood vs fixed | 8 | 8 / 0 | -356 | 0.008 |
| ml_local vs fixed | 8 | 6 / 2 | -289 | 0.289 |
| ml_neighborhood vs fixed_native | 8 | 7 / 1 | -215 | 0.070 |
| ml_local vs fixed_native | 8 | 5 / 3 | -133 | 0.727 |
| ml_neighborhood vs max_pressure_guarded | 8 | 8 / 0 | -1,649 | 0.008 |
| ml_local vs max_pressure_guarded | 8 | 7 / 1 | -1,719 | 0.070 |
| max_pressure_guarded vs fixed | 8 | 0 / 8 | 1,215 | 0.008 |

With 8 seeds the two-sided sign test reaches p < 0.05 only at 8/0 (p = 0.008); 7/1 gives p = 0.070.

