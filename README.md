# AdaptiveTrafficController

## Current shared progress

The SUMO project is in [simulation/astoria-osm-import](simulation/astoria-osm-import/README.md). It was generated from the supplied OpenStreetMap extract for the Astoria Boulevard / I-278 study area and now includes a first observed-profile route replay.

It currently contains:

- The OSM source extract and SUMO-converted road network.
- Two fixed-time smoke/stress configurations with seeded synthetic routes, plus one 24-hour replay of an observed NYSDOT station profile.
- Logs and trip, queue, and summary outputs from smoke and higher-volume runs.
- Import assumptions, OSM attribution, run instructions, and remaining network-review items.
- [Traffic data evidence and calibration limits](simulation/astoria-osm-import/TRAFFIC-DATA.md), with screenshot-transcribed 2023–2025 AADT values, complete 15-minute reports from four NYSDOT stations, and a NYC DOT 2026 PM-peak intersection-count reference.

The OSM signal markers do not contain actual NYC phase timings, so SUMO's static 90-second programs are assumptions. The two smoke/stress route sets are synthetic; the separate 054474 replay uses an observed count profile but remains a single-stream diagnostic. None of these scenarios is calibrated as a complete Astoria network or establishes congestion reduction.

The supplied NYSDOT screenshots add AADT evidence for Astoria Boulevard and nearby I-278 segments. The package now also records one complete 15-minute count day from each of four NYSDOT stations and NYC DOT's 2026 PM-peak intersection totals at 21st and 31st Streets. The station days are from different years, and one observed daily total differs substantially from its AADT; see the data note before describing demand as calibrated. The next step is to digitize the intersection movement diagram, map stations to SUMO edges, and create conserved directional/time-varying demand.

Both students should review and understand the network, assumptions, and results before using or extending it. Record each student's actual contributions separately as work continues.
