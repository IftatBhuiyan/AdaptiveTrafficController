# Astoria OSM to SUMO project

This project imports the supplied OpenStreetMap extract, provides two synthetic fixed-time smoke/stress scenarios, and includes a first 24-hour route replay driven by an observed Astoria Boulevard count profile. The OSM file is the user's source data; the generated SUMO network keeps the whole connected area in that extract.

## Files

- map.osm: supplied OpenStreetMap source extract.
- astoria.net.xml: SUMO network generated from the OSM extract with SUMO 1.27.1.
- scenario-smoke.rou.xml: seeded, low-volume route set for checking the network.
- scenario-stress.rou.xml: seeded, higher-volume route set for initial queue stress.
- astoria-smoke.sumocfg, astoria-stress.sumocfg: runnable fixed-time scenario configurations.
- build_observed_profile_replay.py, scenario-observed-054474.rou.xml, and astoria-observed-054474.sumocfg: reproducible 24-hour replay from one complete observed station profile.
- netconvert.log: conversion output and warnings.
- runs/: smoke/stress outputs plus the observed-profile replay's trip and candidate-edge data.
- RESULTS.md: import counts, run settings, measured outputs, and limitations.
- TRAFFIC-DATA.md, traffic-counts.csv, and traffic-profiles.csv: NYSDOT AADT summaries and complete 15-minute station profiles, plus NYC DOT peak-hour intersection evidence and calibration limits.

The OSM source declares ODbL 1.0 and contains the standard OpenStreetMap attribution. Keep that attribution if the data or derived map is redistributed: © OpenStreetMap contributors, https://www.openstreetmap.org/copyright.

## Important assumptions

The network shape and road attributes come from the provided OSM extract. The OSM data has traffic-signal marker locations, not signal phase plans or actual timings. SUMO therefore guessed signal-controlled junctions and generated fixed programs; the resulting static 90-second programs are simulation assumptions, not verified NYC signal timings.

Traffic demand is generated synthetically by SUMO's randomTrips.py, with a fixed seed and fringe-to-fringe trips. The 40-trip smoke scenario inserts one trip every 15 seconds for 600 seconds. The 180-trip stress scenario inserts one trip every 5 seconds for 900 seconds. Neither profile is calibrated to observed Astoria traffic. Routes are selected across the imported network, so these are network smoke/stress scenarios, not directional peak or ramp-specific scenarios yet. See RESULTS.md for measured outputs and limitations.

The observed-profile replay uses the 054474 eastbound 15-minute station profile from May 29, 2025, traversing one assumed upstream-to-33rd-Street route. It applies the station's 4% AADT truck share uniformly because the daily report does not give interval-specific classes. It omits all cross traffic and other approaches, and its path/phase mapping is not yet visually approved. It is a diagnostic load replay, not a complete traffic-area model. Its initial run left substantial demand waiting at the upstream boundary; see RESULTS.md and TRAFFIC-DATA.md.

## Run from this directory

For a visible run, open SUMO sumo-gui and load astoria-smoke.sumocfg or astoria-stress.sumocfg with File > Open Simulation Configuration. On the current Mac installation, a headless run from this folder is:

    export SUMO_HOME='/Library/Frameworks/EclipseSUMO.framework/Versions/Current/EclipseSUMO/share/sumo'
    "$SUMO_HOME/bin/sumo-gui" -c astoria-smoke.sumocfg

Select astoria-stress.sumocfg to run the higher-volume profile. Headless equivalents are:

    "$SUMO_HOME/bin/sumo" -c astoria-smoke.sumocfg
    "$SUMO_HOME/bin/sumo" -c astoria-stress.sumocfg

To regenerate and run the 054474 observed-profile replay:

    python3 build_observed_profile_replay.py
    "$SUMO_HOME/bin/sumo" -c astoria-observed-054474.sumocfg

On another operating system, use its installed SUMO-GUI and choose the same configuration file. The output XML is written beneath runs/. The network is fixed-time; it has no adaptive controller or information sharing. The command-line runs were verified on the current Mac; GUI rendering could not be observed in this desktop automation session.

## Rebuild the network

    export SUMO_HOME='/Library/Frameworks/EclipseSUMO.framework/Versions/Current/EclipseSUMO/share/sumo'
    "$SUMO_HOME/bin/netconvert" \
      --osm-files map.osm \
      --output-file astoria.net.xml \
      --geometry.remove --roundabouts.guess --ramps.guess --junctions.join \
      --tls.guess-signals --tls.guess-signals.slack 1 \
      --tls.default-type static --tls.cycle.time 90 \
      --keep-edges.by-vclass passenger --output.original-names

This rebuild reproduces the import settings used for the generated network. Inspect netconvert.log and review geometry and guessed signal programs in NetEdit before treating them as accurate.

## Next calibration work

1. Review the converted network in SUMO GUI/NetEdit and create a junction/route inventory.
2. Compare the four primary intersections and ramp/merge area against the annotated map and OSM attributes.
3. Determine which real signal plans and turning counts are available; label unavailable values rather than guessing.
4. Replace uniform random trips with controlled directional and ramp-demand scenarios only after selecting the exact source/destination edges together.
5. Use matched network, routes, time window, and seed when comparing future controllers.

NYSDOT AADT summaries and complete 15-minute count-day profiles for nearby Astoria Boulevard/I-278 stations are recorded in traffic-counts.csv and traffic-profiles.csv. NYC DOT's 2026 study also provides PM-peak intersection-volume evidence. See TRAFFIC-DATA.md for source dates, an unresolved AADT/profile discrepancy at one station, and remaining work to build conserved routes and turning flows. The existing smoke/stress runs remain preliminary pipeline checks, not calibrated peak-traffic scenarios.

The project now includes synthetic smoke/stress runs and one measured-profile replay on an assumed eastbound corridor path. It still does not claim a calibrated Astoria model, a complete multi-approach scenario, or a measured traffic improvement.
