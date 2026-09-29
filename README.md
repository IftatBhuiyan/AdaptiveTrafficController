# AdaptiveTrafficController

## Current shared progress

The first runnable SUMO project is in [simulation/astoria-osm-import](simulation/astoria-osm-import/README.md). It was generated from the OpenStreetMap extract supplied for the Astoria Boulevard / I-278 study area.

It currently contains:

- The OSM source extract and SUMO-converted road network.
- Two fixed-time simulation configurations with seeded synthetic traffic routes.
- Logs and trip, queue, and summary outputs from smoke and higher-volume runs.
- Import assumptions, OSM attribution, run instructions, and remaining network-review items.
- [NYSDOT traffic-count evidence and calibration limits](simulation/astoria-osm-import/TRAFFIC-DATA.md), with the screenshot-transcribed 2023–2025 AADT values in `traffic-counts.csv`.

The OSM signal markers do not contain actual NYC phase timings. SUMO generated static timing programs for this initial runnable baseline. The traffic demand is synthetic and has not been calibrated to NYC counts. These runs verify the simulation pipeline; they do not establish congestion reduction or real-world effectiveness.

The supplied NYSDOT screenshots add AADT evidence for Astoria Boulevard and nearby I-278 segments. AADT is not hourly or turning-movement demand, so it is not sufficient on its own to build realistic peak-period routes. The next step is to obtain the station data histories and turning counts, map stations to network edges, and create conserved directional/time-varying demand.

Both students should review and understand the network, assumptions, and results before using or extending it. Record each student's actual contributions separately as work continues.
