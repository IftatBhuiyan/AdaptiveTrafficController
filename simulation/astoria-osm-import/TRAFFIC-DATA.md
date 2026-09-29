# Traffic data: source, current evidence, and calibration limits

## Source

The screenshots supplied with the project show the New York State Department of Transportation (NYSDOT) Traffic Data Viewer (TDV), powered by Drakewell. NYSDOT describes the TDV as a source of AADT and additional traffic statistics, including vehicle volume, classification, and speed information, with downloadable data and more information available through each site's dashboard/data links:

- [NYSDOT Traffic Data Viewer information](https://www.dot.ny.gov/tdv)
- [NYSDOT TDV map](https://nysdottrafficdata.drakewell.com/publicmultinodemap.asp)
- [NYSDOT Highway Data Services](https://www.dot.ny.gov/highway-data-services)

The values in `traffic-counts.csv` are transcribed from the five screenshots supplied for this project on September 29, 2026. They are recorded as screenshot evidence, not yet independently reconciled with the downloadable station records. Four visible station summaries include 2023, 2024, and 2025 AADT for Astoria Boulevard and I-278 links near 31st Street. The Astoria Boulevard screenshot also shows single-unit/combo-unit AADT and truck percentages. Blank fields mean the screenshot did not show a value; they are not zeroes.

## What these numbers can and cannot do

AADT is an annual daily average for a roadway station/segment. It is useful to anchor and sanity-check daily traffic scale. By itself, it does not provide:

- Hour-by-hour arrival profiles for a morning or evening peak.
- Turning proportions at the 31st, 32nd, or 33rd Street intersections.
- Origin-destination paths through the study network.
- Signal phase plans, cycle lengths, or offsets.

Therefore the current 40-vehicle smoke and 180-vehicle stress routes remain software checks only. Do not distribute each station's AADT independently as SUMO flow: the same vehicles may cross several count stations, which would double-count demand and violate network flow conservation.

## Next calibration steps

1. In the TDV, open each station's Site Data/Site Dashboard and export the underlying count history/report, including collection dates, hourly or interval volumes, and direction where available. Store the downloaded source file and its retrieval date with the project.
2. Identify the closest usable hourly/continuous count profile. If the ramp stations only provide annualized counts, ask NYSDOT Highway Data Services Bureau whether hourly reports or count records are available for those station IDs.
3. Inventory the imported SUMO boundary edges and map each count station to its matching road segment and direction. Do not assume the OSM network's guessed signal timings are real.
4. Obtain turning movement counts or defensible turn ratios for the signalized Astoria Boulevard intersections. Use station AADT to constrain/check approach totals, not to invent turns.
5. Build a conserved set of boundary OD/route flows, use observed temporal profiles, and validate simulated counts at held-out stations/time intervals before comparing signal controllers.

Until those checks are complete, describe the model as an OSM-based SUMO network with preliminary NYSDOT AADT evidence—not a calibrated digital twin. Preserve screenshot-to-record provenance and report any unavailable inputs explicitly.
