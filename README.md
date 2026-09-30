# Astoria Traffic Simulation

Our traffic simulation for the Astoria Boulevard / I-278 area.

It runs SUMO in the background and shows the cars, roads, and traffic lights in your browser. You can watch the simulation and edit the scenario.

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
- **Drag the map:** move around.
- **Scroll or use + / −:** zoom in or out.
- **Study area:** return to the main area.
- **All roads:** see the full imported network.

Blue vehicles are moving. Orange vehicles are stopped or queued.

## 5. Edit roads, lights, and traffic

Click **Edit map**. The simulation pauses so you can work.

Choose a tool:

- **Roads:** click a road to change its lane count, speed limit, or close it.
- **Routes:** select a route or click a car. Change its entrance, exit, and waypoints. **Add traffic route** lets you add vehicles at a rate you choose.
- **Lights:** click a junction to add or remove a light, or adjust its timing. **Move control** changes which junction controls traffic. **Move marker** only changes where the light is drawn.
- **Shapes:** click a junction surface, building, or terrain shape. Drag its corners, move it, or hide a map artifact.

When you are ready, click **Save & restart simulation**. The app checks the changes, saves them, and restarts the run.

If an edit is rejected, read the message in the editor. For example, a route must connect through valid roads and finish at an outer exit.

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

This is a working research baseline with **fixed-time lights**. It still needs calibration before we can treat its delays or queues as real-world measurements.

See [traffic data and sources](simulation/astoria-osm-import/TRAFFIC-DATA.md) for details.

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
