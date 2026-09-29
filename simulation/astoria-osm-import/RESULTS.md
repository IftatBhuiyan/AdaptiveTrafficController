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

## Observed-profile route replay (exploratory, September 29, 2026)

`build_observed_profile_replay.py` generated a 24-hour eastbound route replay from all 96 15-minute bins at NYSDOT station 054474 on May 29, 2025 (12,677 vehicles). It uses a 4% truck share derived from that station's 2025 AADT summary, applied uniformly across intervals because the report did not provide interval-level vehicle classes. The selected OSM route starts west of the count segment, passes through the local Astoria Boulevard corridor, and ends at the 33rd Street boundary. No cross-street or other-approach traffic is included.

SUMO 1.27.1 ran the route to 25 hours, allowing a one-hour drain. Of 12,677 requested vehicles, 10,603 entered the network by the end; 10,570 completed, 33 remained in the network, and 2,074 were still waiting to depart. Completed trips averaged 218.79 s duration, 129.10 s waiting, and 173.11 s time loss. The candidate station proxy edge `1459805124` recorded 10,080 vehicles leaving during the 24-hour measurement window, 20.5% below the observed 12,677. The run reported zero collisions and zero teleports.

This is a diagnostic route replay, not a calibrated-area result. It applies a measured count at the upstream boundary as a simplification; its route-edge/station match still needs visual review. It omits side-street traffic, turns, pedestrians, and other measured profiles, and it uses heuristic 90-second signal programs. The insertion backlog and count mismatch show that the current network/path/signal assumptions do not yet reproduce the observed day. Do not present its delays or queues as real Astoria performance or evidence of improvement. Reconcile the 31st Street movement diagram, station geometry, and signal phases before building the multi-approach scenario.

The generated vehicle definitions are in `scenario-observed-054474.rou.xml`; rerun `python3 build_observed_profile_replay.py` to rebuild them. `runs/observed-054474-tripinfo.xml` and `runs/observed-054474-edge-data.xml` preserve per-trip and 15-minute candidate-edge measurements. The bulky per-second SUMO summary was parsed for the aggregate insertion/collision counts above and then discarded; the run can recreate it if needed.

## Generated outputs

- Smoke: runs/smoke.log, runs/smoke-tripinfo.xml, runs/smoke-queue.xml, runs/smoke-summary.xml.
- Stress: runs/stress.log, runs/stress-tripinfo.xml, runs/stress-queue.xml, runs/stress-summary.xml.
- Observed-profile replay: runs/observed-054474-tripinfo.xml and runs/observed-054474-edge-data.xml.

The GUI app was not visually inspected in this execution session. Headless SUMO 1.27.1 successfully loaded each SUMO configuration and completed each configured horizon.
