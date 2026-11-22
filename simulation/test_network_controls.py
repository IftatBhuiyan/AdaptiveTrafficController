"""Regression checks for real imported-network defects (no SUMO run needed)."""
from pathlib import Path
import copy
import json
import unittest
import xml.etree.ElementTree as ET

DATA = Path(__file__).resolve().parent / "astoria-osm-import"


class NetworkControls(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = ET.parse(DATA / "astoria-corrected.net.xml").getroot()

    def test_osm_all_way_stops_are_not_inferred_traffic_lights(self):
        nodes = {j.get("id"): j.get("type") for j in self.root.findall("junction")}
        for jid in ("42818224", "42818227"):
            self.assertEqual(nodes[jid], "allway_stop")
        self.assertEqual(nodes["42849685"], "priority_stop")

    def test_both_ramp_lanes_reach_the_taper(self):
        connections = self.root.findall("connection")
        for source in ("1273808663#2", "560353398#0"):
            self.assertEqual({c.get("fromLane") for c in connections if c.get("from") == source and c.get("to") == "25345887"}, {"0", "1"})
        self.assertEqual({c.get("fromLane") for c in connections if c.get("from") == "25345887" and c.get("to") == "964223914"}, {"0", "1"})

    def test_left_turn_onto_31st_street_has_two_lanes(self):
        # 88% of EB off-ramp traffic turns left from Astoria Blvd N onto 31st St.
        # One turn lane capped it near 340 cars per 30 min (NYSDOT 053167: ~818),
        # and Astoria Blvd N lane 2 had no outgoing connection.
        lanes = [l for e in self.root.findall("edge") if e.get("id") == "683926426"
                 for l in e.findall("lane")]
        self.assertEqual(len(lanes), 2)
        connections = self.root.findall("connection")
        turn = {(c.get("fromLane"), c.get("toLane")) for c in connections
                if c.get("from") == "683926428#1" and c.get("to") == "683926426"}
        self.assertIn(("2", "0"), turn)
        self.assertIn(("3", "1"), turn)
        self.assertTrue(any(c.get("from") == "683926428#1" and c.get("fromLane") == "2"
                            for c in connections), "Astoria Blvd N lane 2 is a dead end")
        for lane in ("0", "1"):
            self.assertTrue(any(c.get("from") == "683926426" and c.get("fromLane") == lane
                                and c.get("to") == "497263586#1" for c in connections))

    def test_receiver_observation_reaches_the_actual_forced_merge(self):
        from sumo_runtime import configure
        configure()
        from adaptive_controller import Topology
        topology = Topology(DATA / "astoria-corrected.net.xml")
        for lane in ("25345887_0", "25345887_1"):
            self.assertEqual(topology.receivers[lane][-1], "1134690192_0")
        self.assertIn("1134690192_1", topology.forced_merges["1134690192_0"])
        self.assertIn("122625709_0", topology.detectors)

    def test_source_review_keeps_historical_only_control_unresolved(self):
        inventory = json.loads((DATA / "intersection-controls.json").read_text())
        self.assertTrue(inventory["junctions"]["42833271"]["review_needed"])
        for jid in ("588175755", "1934650076", "42842481",
                    "cluster_42818218_9776116426_9776116432_9776116443_#1more"):
            self.assertFalse(inventory["junctions"][jid]["review_needed"])
        self.assertEqual({item["osm_node"] for item in inventory["resolved_source_conflicts"]},
                         {"5866672576", "42842481"})
        self.assertEqual(len(inventory["unresolved"]), 10)

    def test_source_review_is_invalidated_when_approaches_change(self):
        from sumo_runtime import configure
        configure()
        import sumolib
        from rebuild_network import apply_map_reviews, MAP_REVIEW
        net = sumolib.net.readNet(str(DATA / "astoria-corrected.net.xml"))
        reviews = json.loads(MAP_REVIEW.read_text())
        # A changed ramp lane count/priority must reopen the map warning.
        reviews["junctions"]["588175755"]["incoming"]["1001721292"] = [2, 11]
        inventory = json.loads((DATA / "intersection-controls.json").read_text())
        other_unresolved = copy.deepcopy(inventory["unresolved"])
        apply_map_reviews(inventory, net, reviews)
        self.assertTrue(inventory["junctions"]["588175755"]["review_needed"])
        self.assertIn("no longer matches", inventory["junctions"]["588175755"]["basis"])
        self.assertEqual(inventory["unresolved"], other_unresolved)
        self.assertEqual(len(inventory["resolved_source_conflicts"]), 2)

    def test_source_review_does_not_dismiss_a_new_source_conflict(self):
        from sumo_runtime import configure
        configure()
        import sumolib
        from rebuild_network import apply_map_reviews, MAP_REVIEW
        net = sumolib.net.readNet(str(DATA / "astoria-corrected.net.xml"))
        inventory = json.loads((DATA / "intersection-controls.json").read_text())
        new_conflict = {"junction": "1934650076", "osm_node": "new-stop-marker",
                        "reason": "OSM stop marker conflicts with mapped control."}
        inventory["unresolved"].append(new_conflict)
        apply_map_reviews(inventory, net, json.loads(MAP_REVIEW.read_text()))
        self.assertTrue(inventory["junctions"]["1934650076"]["review_needed"])
        self.assertIn(new_conflict, inventory["unresolved"])


if __name__ == "__main__":
    unittest.main()
