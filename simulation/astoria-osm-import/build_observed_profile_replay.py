#!/usr/bin/env python3
"""Build a one-day SUMO replay for the complete 054474 count profile.

This is a single eastbound Astoria Boulevard through-stream, not a complete
OD matrix or calibrated multi-approach traffic model. The 4% truck share is
the station's 2025 AADT summary applied uniformly because interval-level
classification was not available in the selected daily report.
"""

import csv
import xml.etree.ElementTree as ET
from pathlib import Path


HERE = Path(__file__).resolve().parent
PROFILE_FILE = HERE / "traffic-profiles.csv"
OUTPUT_FILE = HERE / "scenario-observed-054474.rou.xml"
PROFILE_COLUMN = "station_054474_2025-05-29"
TRUCK_SHARE = 0.04
INTERVAL_SECONDS = 15 * 60
ROUTE_ID = "astoria_eb_054474_replay"
ROUTE_EDGES = (
    "516825822#1 516825822#4 516825822#5 139919646#1 722441539#0 "
    "5707919#1 1459805124 1273808663#0 1273808663#1 1273808663#2"
)


def main() -> None:
    with PROFILE_FILE.open(newline="", encoding="utf-8") as source:
        rows = list(csv.DictReader(source))

    if len(rows) != 96 or PROFILE_COLUMN not in rows[0]:
        raise SystemExit("Expected 96 intervals and profile column " + PROFILE_COLUMN)

    root = ET.Element("routes")
    ET.SubElement(root, "vType", id="passenger", vClass="passenger")
    ET.SubElement(root, "vType", id="truck", vClass="truck")
    ET.SubElement(root, "route", id=ROUTE_ID, edges=ROUTE_EDGES)

    cumulative_volume = 0
    cumulative_trucks = 0
    for index, row in enumerate(rows):
        volume = int(row[PROFILE_COLUMN])
        cumulative_volume += volume
        truck_total = round(cumulative_volume * TRUCK_SHARE)
        trucks = truck_total - cumulative_trucks
        cumulative_trucks = truck_total
        passengers = volume - trucks
        begin = index * INTERVAL_SECONDS
        end = begin + INTERVAL_SECONDS

        for vehicle_type, count in (("passenger", passengers), ("truck", trucks)):
            if count:
                ET.SubElement(
                    root,
                    "flow",
                    id=f"count_{index:02d}_{vehicle_type}",
                    type=vehicle_type,
                    route=ROUTE_ID,
                    begin=str(begin),
                    end=str(end),
                    number=str(count),
                    departLane="best",
                    departSpeed="desired",
                )

    ET.indent(root, space="    ")
    ET.ElementTree(root).write(OUTPUT_FILE, encoding="utf-8", xml_declaration=True)
    print(f"Wrote {len(rows)} intervals: {cumulative_volume} vehicles, {cumulative_trucks} trucks")
    print(f"Output: {OUTPUT_FILE.name}")


if __name__ == "__main__":
    main()
