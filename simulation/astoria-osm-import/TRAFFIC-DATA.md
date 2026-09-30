# Traffic data: source, current evidence, and calibration limits

## Source

The screenshots supplied with the project show the New York State Department of Transportation (NYSDOT) Traffic Data Viewer (TDV), powered by Drakewell. NYSDOT describes the TDV as a source of AADT and additional traffic statistics, including vehicle volume, classification, and speed information, with downloadable data and more information available through each site's dashboard/data links:

- [NYSDOT Traffic Data Viewer information](https://www.dot.ny.gov/tdv)
- [NYSDOT TDV map](https://nysdottrafficdata.drakewell.com/publicmultinodemap.asp)
- [NYSDOT Highway Data Services](https://www.dot.ny.gov/highway-data-services)

The values in `traffic-counts.csv` are transcribed from the five screenshots supplied for this project on September 29, 2026, and were checked against the NYSDOT viewer popups. Blank fields mean the screenshot did not show a value; they are not zeroes.

The TDV's Site Data daily reports were also inspected. `traffic-profiles.csv` contains the 96 observed 15-minute total-flow bins from one complete count day for each station. The source report links are [054474, May 29 2025](https://nysdottrafficdata.drakewell.com/tfdayreport.asp?node=NYSDOT_SC&cosit=054474000000&reportdate=2025-05-29&enddate=2025-05-29), [053167, March 29 2023](https://nysdottrafficdata.drakewell.com/tfdayreport.asp?node=NYSDOT_SC&cosit=053167000000&reportdate=2023-03-29&enddate=2023-03-29), [053166, June 17 2026](https://nysdottrafficdata.drakewell.com/tfdayreport.asp?node=NYSDOT_SC&cosit=053166000000&reportdate=2026-06-17&enddate=2026-06-17), and [053169, June 17 2026](https://nysdottrafficdata.drakewell.com/tfdayreport.asp?node=NYSDOT_SC&cosit=053169000000&reportdate=2026-06-17&enddate=2026-06-17). Each report identified an all-lane, 15-minute total-flow series. Adjacent available days were partial (fewer than 96 intervals), so only the complete day per station is included. The observations are not contemporaneous across all four locations.

The selected-day totals are 12,677 (054474), 29,228 (053167), 29,024 (053166), and 8,015 (053169) vehicles. The screenshot AADT values for 2025 are 11,523, 26,137, 32,608, and 17,027 respectively. These are different statistics and, for three sites, different years. The 053169 difference is especially large and needs investigation. Check count coverage, direction, station geometry, and report quality rather than silently forcing agreement.

A candidate geometry check used NYSDOT's station line and the OSM source geometry: station 054474 overlaps the Astoria Boulevard South way pieces that SUMO names `5707919#1`, `1459805124`, and `1273808663#0` through `#2`. These edges were used in the first profile replay route. This is a working map match only; the full corridor, lane coverage, stop lines, and signal connections still need visual inspection in NetEdit.

NYC DOT's March 11, 2026 [Astoria Boulevard Safety Enhancements presentation](https://www.nyc.gov/html/dot/downloads/pdf/astoria-blvd-21-st-33-st-mar2026.pdf) covers 21st to 33rd Streets. Its sample-count appendix says the corridor's highest traffic volume occurs in the 4:30–5:30 PM peak hour, reports 1,286 vehicles through Astoria Boulevard & 31st Street with about 2% trucks, and 2,017 vehicles through Astoria Boulevard & 21st Street with about 5% trucks. The presentation includes movement-by-movement arrows at 31st Street; digitize and verify those individual movements from the figure before using turn ratios. The count observation date is not stated in the cited slides.

## What these numbers can and cannot do

AADT is an annual daily average for a roadway station/segment. It is useful to anchor and sanity-check daily traffic scale. By itself, it does not provide:

- Hour-by-hour arrival profiles for a morning or evening peak.
- Turning proportions at the 31st, 32nd, or 33rd Street intersections.
- Origin-destination paths through the study network.
- Signal phase plans, cycle lengths, or offsets.

Therefore the current 40-vehicle smoke and 180-vehicle stress routes remain software checks only. Do not distribute each station's AADT independently as SUMO flow: the same vehicles may cross several count stations, which would double-count demand and violate network flow conservation.

The 054474 profile has been run once as a single eastbound route through the imported corridor. This test used the station's observed profile at the upstream boundary and omitted other approaches; it did not reproduce the full station volume within the 24-hour measurement window. Its results are diagnostic only; see `RESULTS.md` before reusing the scenario.

## Next calibration steps

1. Map each station segment to SUMO edges and verify direction and lane coverage against the OSM network and route descriptions.
2. Transcribe the NYC DOT 31st Street movement diagram into an audited movement matrix; check whether the 21st Street counts also help constrain the network boundary.
3. Explain and resolve the observed-vs-AADT discrepancy for 053169; ask NYSDOT if the station report does not explain it.
4. Build a conserved boundary OD/route matrix. Do not inject each station count independently, because a vehicle may cross multiple counted links.
5. Match the 15-minute station profiles and PM-peak intersection totals, report any missing inputs, and only then compare controllers.

Until those checks are complete, describe the model as an OSM-based SUMO network with NYSDOT 15-minute station profiles and NYC DOT peak-hour intersection evidence—not a calibrated digital twin. Preserve report links, retrieval dates, station IDs, and the stated data limitations.
