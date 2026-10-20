"""Regression checks for real imported-network defects (no SUMO run needed)."""
from pathlib import Path
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

    def test_receiver_observation_reaches_the_actual_forced_merge(self):
        from sumo_runtime import configure
        configure()
        from adaptive_controller import Topology
        topology = Topology(DATA / "astoria-corrected.net.xml")
        for lane in ("25345887_0", "25345887_1"):
            self.assertEqual(topology.receivers[lane][-1], "1134690192_0")
        self.assertIn("1134690192_1", topology.forced_merges["1134690192_0"])
        self.assertIn("122625709_0", topology.detectors)


if __name__ == "__main__":
    unittest.main()
