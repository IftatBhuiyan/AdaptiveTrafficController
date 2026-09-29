# Initial SUMO import and fixed-time run record

Run date: September 28, 2026
SUMO: 1.27.1
Input: user-supplied OpenStreetMap extract, copied as map.osm

## Imported network

- Source OSM file: 8.5 MB; the extract reports 9,955 nodes and 2,227 ways, including 562 road ways and 83 traffic-signal marker nodes.
- netconvert produced 321 junctions, 330 non-internal road edges, and 125 traffic-light program records.
- Converted simulation bounds: approximately 1,778 m by 2,918 m.
- All generated signal programs are static 90-second cycles with heuristic phase splits and yellow/all-red transitions. OSM provided signal locations, not actual NYC phase timing plans.
- Import log: netconvert.log.

The import completed, but visual geometry review is still needed. The log contains warnings about incomplete public-transport relations/stops, several signal markers that control no links, two junction shapes offset from their source node by more than 20 m, and some likely missing-turn/lane assumptions. These are cleanup items, not warnings to hide.

## Reproducible demand runs

Both demand files are synthetic, fringe-to-fringe trips across the full imported network. The same seed, 42, was used. The network and fixed signal programs were held constant. Traffic volumes and OD pairs are not calibrated to NYC observations; neither scenario isolates a particular ramp movement.

The smoke demand inserts 40 vehicles with a 15-second headway from 0 to 600 seconds; the simulation runs through 900 seconds. The stress demand inserts 180 vehicles with a 5-second headway from 0 to 900 seconds; its simulation runs through 1,800 seconds.

Results:

Smoke inserted and completed 40 of 40 vehicles with none unfinished. Mean trip duration was 188.68 s, mean waiting time 58.67 s, mean time loss 85.50 s, and maximum reported lane queue length 18.38 m. SUMO queue output defines queueing_length as the distance from the junction to the last queued vehicle, with a default queue-speed threshold of 1.39 m/s (5 km/h). The run reported zero collisions and zero teleports.

Stress inserted and completed 180 of 180 vehicles with none unfinished. Mean trip duration was 214.18 s, mean waiting time 80.47 s, mean time loss 112.36 s, and maximum reported lane queue length 43.53 m. The run reported zero collisions and zero teleports. See the [SUMO queue-output definition](https://sumo.dlr.de/docs/Simulation/Output/QueueOutput.html).

These results show the imported network and fixed-time programs can carry two reproducible synthetic demand profiles. The higher-volume profile produced higher waiting and queue measurements. Since the scenarios differ in demand rate and horizon, this is not a controlled controller comparison and does not establish a traffic-control improvement.

## Generated outputs

- Smoke: runs/smoke.log, runs/smoke-tripinfo.xml, runs/smoke-queue.xml, runs/smoke-summary.xml.
- Stress: runs/stress.log, runs/stress-tripinfo.xml, runs/stress-queue.xml, runs/stress-summary.xml.

The GUI app was not visually inspected in this execution session. Headless SUMO 1.27.1 successfully loaded each SUMO configuration and completed each configured horizon.
