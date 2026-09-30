# Astoria traffic simulation

A browser view of a real SUMO simulation, using our downloaded Astoria / I-278 street network and four collected traffic-count profiles. Vehicles use lanes, obey signals, queue, and travel between outer-map entrances and exits. The simulation currently uses fixed-time signals.

## Run it

Install **Python 3.10 or newer** and **SUMO 1.27.1**. If SUMO is already installed, the launcher finds it automatically through `SUMO_HOME`, your PATH, or its standard installation folder.

After cloning this branch:

```sh
git clone --branch codex/astoria-osm-import https://github.com/IftatBhuiyan/AdaptiveTrafficController.git
cd AdaptiveTrafficController
```

On Windows, double-click **Start simulation.bat**, or run:

```sh
py -3 run.py
```

On macOS or Linux:

```sh
python3 run.py
```

The browser opens at **http://127.0.0.1:8011**. Keep the terminal open; press Ctrl+C there to stop. Use `--port 8012` if that port is occupied, or `--no-browser` to start without opening a tab. No Node.js, frontend build, or browser API key is required. The browser simulation uses headless SUMO, so it does not require XQuartz.

If you prefer installing SUMO through Python, create a virtual environment and install the pinned packages in `requirements.txt`. For example on Windows:

```sh
py -3 -m venv .venv
.venv\Scripts\python -m pip install -r requirements.txt
.venv\Scripts\python run.py
```

On macOS, the official SUMO installer is the simplest dependency setup; the Python binary package can still require system libraries.

## Edit the scenario

Click **Edit map**. Playback pauses while you make changes.

- **Roads:** select a road, change lane count or speed, or close it. Lane connections are rebuilt and affected routes are checked.
- **Routes:** select a route or click a car, choose an entrance and exit, and add waypoints by clicking roads. Add a custom traffic stream with a vehicles-per-hour rate.
- **Lights:** click a junction to add or remove signal control, move the control to another junction, adjust its cycle and offset, or reposition its displayed markers. Moving a marker alone does not change where vehicles stop.
- **Shapes:** select a junction surface, building, or terrain polygon. Drag corner handles, add/remove corners, move the shape, or hide/recolour a map artifact.

Click **Save & restart simulation** to apply. Invalid or disconnected routes are rejected before saving. Edits are stored in `simulation/astoria-osm-import/scenario-edits.json`; share that file with the rest of the project so both partners use the same scenario.

For drawing or reconnecting roads and editing individual lane connections, expand **Advanced network editing → Open NetEdit**. Save `user-network.net.xml` in NetEdit, then click **Reload saved network**. Share that network file too. The native NetEdit application needs XQuartz on macOS. Large topology changes, particularly deleting an entrance, exit, or counted link, can require updating the demand and station mapping.

## What the model represents

The 4–6 pm demand combines saved 15-minute profiles from stations 054474, 053167, 053166, and 053169. Those profiles come from different dates. Routes are fitted jointly to the four requested counts so a trip crossing two stations is not injected twice. Additional traffic on uncounted connections is estimated. The initial route choice, 4% truck share, and signal timing are modelling assumptions; queues and achieved station throughput still need calibration.

The reviewed base network includes the two-lane eastbound segment between 31st and 33rd Streets, consolidated junction controls at 31st/33rd, the corrected paved junction shape, and additional fixed-time control at surrounding surface intersections. Added signal locations outside the reviewed intersections are inferred scenario controls, not a claim of a complete field survey. Their list and provenance are in `intersection-controls.json`.

The original `map.osm` and `astoria.net.xml` are retained. The reusable correction inputs are `site-corrections.nod.xml`, `site-corrections.edg.xml`, and `site-corrections.con.xml`. To rebuild the reviewed base and its demand:

```sh
python simulation/rebuild_network.py
python simulation/build_full_area.py
```

The browser and standalone `astoria-full-area.sumocfg` use the same corrected SUMO model. The demand generator uses seed 42 and records its inputs, count reconciliation, and assumptions in `full-area-demand.json`. See [traffic sources](simulation/astoria-osm-import/TRAFFIC-DATA.md). Map data © [OpenStreetMap contributors](https://www.openstreetmap.org/copyright), ODbL 1.0.

## Earlier runs

The original smoke/stress scenarios and the single-station replay remain in [simulation/astoria-osm-import](simulation/astoria-osm-import/README.md), with their recorded outputs and limitations. They are earlier pipeline checks; use `run.py` for the current full-area model and saved browser edits.
