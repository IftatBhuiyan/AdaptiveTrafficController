# Astoria Traffic Simulation

Our traffic simulation for the Astoria Boulevard / I-278 area.

It runs SUMO in the background and shows the cars, roads, and traffic lights in your browser. You can watch the simulation and edit the scenario.

The next controller version adds experimental network MPC and seeded driver/route variation. See [the model, constraints, forecasting experiment, and verification limits](simulation/MPC.md). MPC needs the optional Python 3.12+ NumPy dependency in `requirements-mpc.txt`.

## 1. First-time setup

You need:

- **Python 3.10 or newer:** [Download Python](https://www.python.org/downloads/)
- **SUMO 1.27.1:** [Download SUMO](https://sumo.dlr.de/docs/Downloads.php)
- **Git** to clone the project: [Download Git](https://git-scm.com/downloads)

If you already have these installed, skip to the next step.

You do **not** need Node.js or an API key. XQuartz is not needed for the browser simulation. On a Mac, it is only needed if you open SUMO’s separate graphical tools, such as NetEdit.

## 2. Download the project

Open **PowerShell on Windows** or **Terminal on Mac**, then run:

```sh
git clone https://github.com/IftatBhuiyan/AdaptiveTrafficController.git
cd AdaptiveTrafficController
```

This downloads the shared version from the main branch, including the simulation and editor.

## 3. Start the simulation

### Windows

Double-click **Start simulation.bat** in the project folder.

Or run this command from that folder:

```sh
py -3 run.py
```

### Mac

Run this command from the project folder:

```sh
python3 run.py
```

You can also double-click **Start simulation.command**.

### What happens next?

Your browser opens at **http://127.0.0.1:8011**. Give it a few seconds to load the map and bring vehicles into the area.

**Keep the terminal window open while using the simulation.** To stop it, return to the terminal and press **Ctrl+C**.

The next time you want to use it, just start it again. You do not need to clone or install everything again.

## 4. Use the simulation

- **Pause / Play:** stop or resume playback.
- **Speed:** choose how quickly simulation time passes.
- **Restart:** start the simulation over.
- **4 pm / 5 pm:** run a 15-minute warm-up before the selected time, so you see an already-loaded network. Each selection is an independent scenario, not a continuation of the earlier run. Warm-up may take a minute or more.
- **Signal controller:** intelligent coordinator is selected by default; you can also choose experimental Network MPC, the previous coordinated adaptive controller, local capacity-aware, or fixed time. Network MPC needs the optional numeric dependency described below. Changing it restarts the same traffic scenario with a common fixed-control warm-up.
- **Driver behavior:** seeded variation is the browser default. It varies legal route choices, desired speeds, acceleration/gaps, cooperation, and small lateral alignment preferences. The original driver preset remains available. The seed is repeatable for matched comparisons; the distributions are assumptions, not calibrated Astoria behavior.
- **Traffic scenario:** choose count-based baseline, constrained merge, low cooperation, a demand surge, or a temporary receiving-lane blockage. The last four are labeled assumptions, not measured ground truth.
- **Drag the map:** move around.
- **Scroll or use + / −:** zoom in or out.
- **Study area:** return to the main area.
- **All roads:** see the full imported network.

Blue vehicles are moving. Orange vehicles are stopped or queued. Red octagons are stop signs. A purple **?** means the intersection's control still needs checking in the field; select it under **Controls** to inspect its source.

Expand the controller summary below the map to see queues, predicted arrivals, and the reason for each central light's decision. Green outlines mark the adaptive-control neighbourhood. A split red/green marker means some turns are allowed and others are not. A detected collision or teleport stops playback; restart to run again.

## Adaptive controller

The current network selects **37 signals** around the three interchange junctions: five signal-to-signal hops, plus signals within 650 metres. It observes surrounding roads and the unsignalized ramp merge; it does not invent a light on the freeway.

- **Coordinated adaptive:** every five seconds, compare legal green actions over a 30-second horizon. Use current queues, approaching vehicles, recent arrivals/turns/discharge, and neighbours' proposed releases. Controllers share limited receiving space so two feeders cannot each assume the whole ramp is available. Blocked entrances and shared-lane turning obstruction constrain predicted service.
- **Local capacity-aware:** use present queues and immediate downstream room, without neighbourhood arrival prediction.
- **Fixed time:** repeat a protected reference schedule, using imported green durations and additional 12-second turn actions where needed.

The previous coordinated adaptive, local, and fixed browser modes share protected simulation actions. The intelligent coordinator instead retains imported priority/yielding greens and uses its own joint-signal programme. Both controller families preserve yellow intervals and at least two seconds of all-red. Moving conflicting vehicles can extend clearance by up to three seconds; links conflicting with an occupied connector stay red while other links can open. Adaptive control has an eight-second minimum green, a 50-second maximum-green policy when another movement is serviceable, and an aging rule for waiting approaches. It also tries another legal connector when a ready car fails to depart as predicted. Tiny roads and cars waiting on their preceding connectors remain observable.

The controller reads **present observations and history**, not hidden vehicle routes or future scheduled departures. The evaluator alone reads the demand roster to count vehicles still waiting outside the network. These are ideal simulated observations, not deployed roadside sensors. This is a research prototype: controller phase programmes change the imported fixed cycle, drawn crossings can add SUMO-generated pedestrian phases, and real signal plans, pedestrian timing, and driver behaviour still need field calibration.

## 5. Edit roads, lights, and traffic

Click **Edit map**. The simulation pauses so you can work.

Choose a tool:

- **Precision** (default) with the tool palette on the left of the map. Click any road, junction, paved area, building, or drawn surface and every point becomes a handle. Tools and shortcuts: **V** select and drag points or whole objects, **A** add a point where you click, **X** delete a point, **S** split a road at an exact distance (shown in metres while you hover), **D** draw a sidewalk, median, plaza, or crosswalk corner by corner, **W** generate a sidewalk that hugs a road edge, **M** measure, **H** pan, **G** toggle snapping. Dragging a junction, or a road end, moves every road attached to that junction. Selected points can be typed in as X/Y metres or nudged with the arrow keys (0.1 m, Shift for 1 m). **Add points evenly**, **Reduce points**, **Smooth curve**, and **Straighten** refine an outline. A road's panel also sets each lane's width, stages precise cuts, or removes the road. **⌘Z / Ctrl+Z** undoes any staged edit; **Shift+⌘Z** redoes. Snapping targets road edges, junction centres, drawn surfaces, then the grid; hold **Alt** to drag freely. Roads that the demand plan or ramp model measures cannot be split or removed.
- **Drawn surfaces change the simulation.** The panel says what each one does before you apply it.
  - **Sidewalk:** becomes a SUMO pedestrian lane on every road whose right-hand edge it covers.
  - **Crosswalk:** becomes a real crossing at the junction whose road it spans. That junction's ordinary roads get sidewalks, and pedestrians cross at the rate you set, 120 per hour by default. At a traffic light, SUMO regenerates that one light's plan with a pedestrian phase. Turning cars yield to walkers.
  - **Median:** closes the lanes it covers and removes turns whose path cuts through it.
  - **Plaza:** makes the roads it covers pedestrian-only, and car routes are re-planned around them.
  - **Custom:** display only.
  - Ramp and counted roads never get sidewalks, closures, or plazas, because the controller and demand plan refer to their lanes by number. Pedestrians are drawn as small purple dots.
- **Roads:** click a road to change its lane count, speed limit, or close it. The lane count previews immediately; vehicle paths change after applying.
- **Routes:** select a route or click a car. Change its entrance, exit, and waypoints. **Add traffic route** lets you add vehicles at a rate you choose.
- **Controls:** click a junction to choose a light, all-way stop, minor-approach stop, or priority/yield. **Move to another junction** changes where the actual light operates. **Drag marker** only changes its display position. For a stop on minor approaches, SUMO's road priorities determine which approaches stop. Cycle/offset settings guide the reference greens; added protected turns and occupied-box clearance can extend the fixed cycle, while adaptive control chooses its own green durations.
- **Shapes:** click an outlined junction, building, or terrain area, then **Zoom to shape**. Drag the numbered handles. To cut, draw a line across the shape and click the side to **keep**. **Undo shape** reverses the last change; **Restore original shape** previews the baseline shape. These tools edit the paved polygon, not road connectivity.

When you are ready, click **Apply changes & restart**. The app checks the changes, saves them, and restarts the run. Geometry edits are rebuilt by SUMO's netconvert: lanes, junction shapes, and connections follow your points, cut roads get a new junction, and routes through removed or cut roads are re-planned.

If an edit is rejected, read the message in the editor. For example, a route must connect through valid roads and finish at an outer exit.

### Experimental coordinator

The experimental intelligent coordinator is the browser default and is also
available for offline evaluation. Its performance and safety targets have not
been met across the tested scenarios. The previous controller remains selectable.
It retains imported SUMO
priority/yielding greens, scores capacity-aware pressure with observed arrivals
and service age, and accounts for switching clearance. This is a simulation
heuristic, not a proven stability guarantee or a field-certified controller.
Compare its policies within the same phase family and demand:

```sh
python3 simulation/evaluate_controller.py --controller intelligent --initialization policy --modes fixed coordinated --scenarios baseline merge --hours 16 --seeds 42 45 46 --minutes 10 --jobs 3 --out /tmp/traffic-intelligent-new-run
```

Use a new output directory each time. Results snapshot inputs and implementation,
include insertion backlog, and distinguish control evaluations from phase switches.

The intelligent controller observes the entire loaded network: all 119 signal
states, 439 road lanes, and 14 stop-controlled junctions in the current map. It
chooses phases at 37 multi-phase signals. At the other 82 signals it can move an
upcoming yellow/red interval earlier into an observed arrival gap, preserving
the imported yellow/red durations and minimum green. Their live schedules also
inform downstream forecasts. A shared
snapshot records current signal states, arrival forecasts, receiving-space
reservations, and proposed next phases before changes are applied. Two planning
passes reconcile neighboring proposals from the same observation. Stop signs
remain stop signs; their queues inform the surrounding traffic observations.

Waiting ages follow observed vehicles until they leave an approach, including
creeping within a queue. When the first group leaves, later arrivals keep their
own shorter waiting ages. These are times since the first observed stop, not
unobserved travel histories. Waiting ages survive lane changes at the same
intersection. Phase priority uses predicted discharge and the ages of the
vehicles that can actually be served, divided by service and switching time.
Minimum green, bounded maximum green when competing demand exists, downstream
storage, and observed departures constrain extensions. A stopped front vehicle
that repeatedly fails to move temporarily suppresses that phase's predicted
service; this evidence expires after 30 seconds or when the front vehicle changes.
Approaching vehicles are distinguished from stopped, blocked vehicles.
Forecast discharge is limited by the green time actually remaining, including
in neighboring signals' receiving-space and arrival predictions.

Yielding turns are not predicted to discharge freely against busy conflicting
priority movements. The adaptive policy may choose compatible subsets of an
imported green to clear a blocked turn; these only remove green permissions and
never upgrade yielding `g` to priority `G`. The current map has 35 such optional
states before joint-signal grouping. The fixed reference never cycles through
the optional states. Both policies retain the same yellow/all-red
transition rules. Learned saturation discharge is updated only under backlogged,
unobstructed full-green conditions, so low demand is not mistaken for low capacity.

The closely spaced `astoria_31st_north` and `GS_5104280319` signals share a joint
programme: their 3.18–3.41 m connecting roads cannot store a full vehicle. Upstream
greens are restricted to movements with a compatible downstream priority green.
When changing receiving branches, upstream closes first; downstream stays open
until connecting lanes and exiting vehicle tails clear, then changes through
yellow/all-red before upstream reopens. Missing lane observations block clearance.
Both fixed and adaptive policies use this constraint. The fixed joint sequence
has six reference stages with 70 seconds of total green in the current map;
elsewhere fixed control retains the imported sequence. This changes the reference
programme, so comparisons require freshly matched fixed runs, not historical
independent-signal results. A collision-free replay is diagnostic evidence, not
proof of safety across traffic conditions.

This combines capacity-aware pressure ideas from
[Gregoire et al.](https://arxiv.org/abs/1309.6484), queue-plus-moving-demand ideas
from [Zhang et al.](https://arxiv.org/abs/2112.10107), and service/setup-time
prioritization motivated by [Lammer and Helbing](https://arxiv.org/abs/0802.0403).
The combined controller is a heuristic, with its own measured results; it does
not inherit the individual papers' theoretical guarantees. No future vehicle
routes or future departure schedules are supplied to the controller.

Score a matched result file with:

```sh
python3 simulation/score_evaluation.py /tmp/traffic-intelligent-new-run/results.json
```

Use `--seeds 45 46` to score held-out seeds separately and `--out scores.json`
to save a new score artifact. Incomplete batches cannot pass the target. Each
intelligent run also saves `shared-state.json` with the observed network,
waiting cohorts, proposed phases, and recent failed-service observations.

The fixed reference scores 80/100. Each component is `min(100, 80 * ratio)`:
completed trips (40%) and ramp completions (10%) use candidate/reference;
system vehicle-seconds including entry backlog (30%) and stopped vehicle-seconds
(20%) use reference/candidate. The target is an average of at least 88, with no
reported collisions or teleports, complete runs, and balanced vehicle accounting.
Each case must also retain at least 95% of reference completions and 80% of ramp
completions, with system time no more than 105% of reference. This is a declared
benchmark score, not an accuracy percentage or a probability of safe operation.
Development cases and held-out cases must be reported separately.

The scorer and evaluator share one validity check. Strategic lane-change deadlocks,
collisions, teleports/removals, incomplete observation windows, changed source or
generated inputs, and failed vehicle accounting invalidate a run. Invalid candidates
or fixed references score zero and cause a failing CLI exit. Historical receipts
without current validity instrumentation remain historical evidence.

Two evaluation protocols answer different questions. The default
`--initialization fixed` gives every policy 15 minutes of fixed-control traffic,
then tests congestion recovery for the observation window. With
`--initialization policy`, each policy controls from the same empty initial
network and identical demand stream; the first 15 minutes are excluded from the
scored observation window. This tests continuous operation and queue prevention.
The scorer rejects comparisons between these protocols. Recovery results must
not be presented as continuous-operation results, or vice versa.

### Where are edits saved?

Browser edits are saved in:

```text
simulation/astoria-osm-import/scenario-edits.json
```

They load automatically the next time you run the project.

### Need to draw or reconnect roads?

Open **Advanced network editing → Open NetEdit**.

Make your changes in NetEdit, save **user-network.net.xml**, then return to the browser and click **Reload saved network**. Browser overrides still apply over that network, so reset any overlapping browser edits if necessary.

Deleting entrances, exits, or roads used by the traffic counts may also require updating the routes.

## 6. Share changes with your partner

Each computer runs its own copy. Saving in the editor does **not** automatically update your partner’s computer.

To share browser edits, run these commands from the project folder:

```sh
git add simulation/astoria-osm-import/scenario-edits.json
git commit -m "Update traffic scenario"
git push
```

If you used NetEdit, also add `simulation/astoria-osm-import/user-network.net.xml` before committing.

Your partner can get the update with:

```sh
git pull --ff-only
```

Then restart the simulation. Save and commit your own work before pulling your partner’s changes.

## Common problems

- **The browser did not open:** visit **http://127.0.0.1:8011** yourself.
- **The page cannot connect:** make sure the simulation is running and its terminal is still open.
- **“SUMO was not found”:** install SUMO, reopen the terminal, and try again. The launcher checks its normal installation folder and `SUMO_HOME`.
- **The port is already in use:** another copy may already be running. Use that copy, stop it with Ctrl+C, or start on another port with `py -3 run.py --port 8012` on Windows or `python3 run.py --port 8012` on Mac.
- **`py` or `python3` is not found:** install Python and reopen the terminal.

## About the data

We use our downloaded OpenStreetMap network and four collected traffic-count profiles. The profiles come from different dates, and some connecting traffic, routes, and signal settings are estimated.

This is a working research simulation, not a calibrated reconstruction of one measured afternoon. Its delays and queues must not be reported as real-world measurements.

The corrected network restores OSM stop controls instead of guessing a light at every multi-road junction. Control sources and unresolved cases are in `simulation/astoria-osm-import/intersection-controls.json`. A road branching **out** of a one-way road does not necessarily need a light; a downstream queue can extend through that junction.

The constrained-merge scenario uses SUMO's sublane model for continuous lateral movement. Its mainline demand (1,600 vehicles/lane/hour at 4 pm; 1,750 at 5 pm), 35% less-cooperative/slower-lateral-response driver mixture, and downstream receiving-lane speeds (5.5 m/s and 4 m/s) are explicit assumptions in `simulation/traffic_behavior.py`. Drivers seek lane centres but still change lanes continuously; off-centre, side-by-side traffic produced footprint collisions on this network's curved fan-outs. Longitudinal acceleration noise is also disabled after a development variant failed side-collision checks. Neutral gap acceptance and collision checks remain enabled. The low-cooperation scenario further reduces that subset's cooperation; the surge adds about 20% base traffic for ten minutes; the blockage reduces receiving-lane speed for 60 seconds before restoring it. These are bounded sensitivity experiments, not learned human-driver distributions or an engine fix.

The next freeway exit is outside the downloaded network; its effect is represented by this boundary condition, not an invented exit. Informal four-to-six-abreast behaviour is not yet calibrated against video or measured road widths.

See [traffic data and sources](simulation/astoria-osm-import/TRAFFIC-DATA.md) for details.

### Run a matched controller comparison

From the project folder:

```sh
python3 simulation/evaluate_controller.py --modes fixed local coordinated --scenarios baseline merge --hours 16 17 --seeds 42 43 --minutes 10 --jobs 2 --out experiments/controller-comparison
```

On Windows, replace `python3` with `py -3`. Each case has 15 minutes of identical fixed-control warm-up before the observation window. `--scenarios disorder surge incident` selects stress cases; `--no-forecast` removes prediction while retaining the coordinated storage/safety model.

Results retain the exact implementation, input hashes, SUMO version, trips, collision log, and time traces. Compare **system vehicle-seconds (active plus due-but-uninserted vehicles)**, completed exits, ramp-route completions, boundary backlog, and spillback exposure. Completed-car average delay alone hides stuck and uninserted traffic. Any collision, teleport, or failed conservation check invalidates a case; short successful runs do not establish field safety or statistical superiority.

<details>
<summary>Optional: install through Python or regenerate the baseline</summary>

If SUMO is not already installed, Windows users can install the pinned Python packages in a separate environment:

```sh
py -3 -m venv .venv
.venv\Scripts\python -m pip install -r requirements.txt
.venv\Scripts\python run.py
```

Use that last command for later runs with this environment. On Mac, the official SUMO installer is the simpler setup because the Python package can require additional system libraries.

You do not need to regenerate the map or routes for normal use. If you are deliberately rebuilding the baseline, run these from the project folder:

```sh
python simulation/rebuild_network.py
python simulation/build_full_area.py
```

Use `py -3` or `python3` instead of `python` if that is how you normally start Python.

The original map and earlier runs remain in [simulation/astoria-osm-import](simulation/astoria-osm-import/README.md).

</details>

Map data © [OpenStreetMap contributors](https://www.openstreetmap.org/copyright), ODbL 1.0.
