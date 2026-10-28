"""Seeded driver inputs and legal route variation on a small synthetic network."""
import copy
from pathlib import Path
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

from sumo_runtime import configure
from traffic_behavior import apply_behavior, apply_driver_variation


class DriverVariation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _, sumo = configure()
        import sumolib
        cls.folder = tempfile.TemporaryDirectory(prefix="driver-variation-test-")
        cls.addClassCleanup(cls.folder.cleanup)
        folder = Path(cls.folder.name)
        nodes, edges = ET.Element("nodes"), ET.Element("edges")
        for nid, x, y in (("o", -100, 0), ("a", 0, 0), ("u", 60, 40),
                          ("l", 60, -40), ("v", 60, 60), ("c", 60, 160),
                          ("b", 120, 0), ("x", 220, 0)):
            ET.SubElement(nodes, "node", id=nid, x=str(x), y=str(y), type="priority")
        for eid, source, target in (("begin", "o", "a"), ("upper1", "a", "u"),
                                    ("upper2", "u", "b"), ("lower1", "a", "l"),
                                    ("lower2", "l", "b"), ("middle1", "a", "v"),
                                    ("middle2", "v", "b"), ("long1", "a", "c"),
                                    ("long2", "c", "b"), ("exit", "b", "x")):
            attrs = {"id": eid, "from": source, "to": target, "numLanes": "1", "width": "3.2", "speed": "13.89"}
            if eid.startswith("lower"):
                attrs["allow"] = "passenger"
            ET.SubElement(edges, "edge", attrs)
        ET.ElementTree(nodes).write(folder / "nodes.xml")
        ET.ElementTree(edges).write(folder / "edges.xml")
        cls.network = folder / "network.net.xml"
        subprocess.run([str(sumo.with_name("netconvert")), "-n", str(folder / "nodes.xml"),
                        "-e", str(folder / "edges.xml"), "-o", str(cls.network),
                        "--no-turnarounds", "true"], check=True, capture_output=True, text=True)
        cls.net = sumolib.net.readNet(str(cls.network))
        cls.plan = {"station_edges": {"entry": ["begin"], "exit": ["exit"]}}

    def routes(self):
        root = ET.Element("routes")
        ET.SubElement(root, "vType", id="car", vClass="passenger", length="4.7", width="1.9")
        ET.SubElement(root, "vType", id="truck", vClass="truck", length="10.2", width="2.45")
        ET.SubElement(root, "route", id="original", edges="begin upper1 upper2 exit")
        for index in range(60):
            ET.SubElement(root, "vehicle", id=f"v{index}", type="truck" if index % 3 == 0 else "car",
                          route="original", depart=f"{index * .3:.1f}", departLane="best", departSpeed="desired")
        ET.SubElement(root, "flow", id="flow", type="car", route="original", begin="30", end="40", number="4")
        apply_behavior(root, "baseline")
        return root

    def test_same_seed_matches_and_other_seed_changes_inputs(self):
        first, same, changed = self.routes(), self.routes(), self.routes()
        a = apply_driver_variation(first, self.network, self.plan, 46)
        b = apply_driver_variation(same, self.network, self.plan, 46)
        c = apply_driver_variation(changed, self.network, self.plan, 47)
        self.assertEqual(ET.tostring(first), ET.tostring(same))
        self.assertEqual(a, b)
        self.assertNotEqual(a["assignment_sha256"], c["assignment_sha256"])
        self.assertNotEqual(a["route_assignment_sha256"], c["route_assignment_sha256"])
        self.assertGreater(a["routes"]["rerouted_vehicles"], 0)

    def test_legal_parameters_and_original_vehicle_dimensions(self):
        root = self.routes()
        metadata = apply_driver_variation(root, self.network, self.plan, 46)
        for profile in metadata["parameters"].values():
            self.assertLessEqual(float(profile["speedFactor"]), 1)
            self.assertEqual(profile["speedDev"], "0")
            self.assertGreaterEqual(float(profile["tau"]), 1.1)
            self.assertGreaterEqual(float(profile["minGap"]), 2.5)
            self.assertGreaterEqual(float(profile["lcCooperative"]), .2)
            self.assertLessEqual(float(profile["lcAssertive"]), 1)
            self.assertLessEqual(abs(float(profile["latAlignment"])), .1)
            self.assertEqual(profile["sigma"], "0")
            self.assertEqual(profile["lcSigma"], "0")
            self.assertEqual(profile["lcPushy"], "0")
            if profile["base_type"] == "truck":
                self.assertEqual((profile["vClass"], profile["length"], profile["width"]), ("truck", "10.2", "2.45"))
            else:
                self.assertEqual((profile["vClass"], profile["length"], profile["width"]), ("passenger", "4.7", "1.9"))

    def test_routes_keep_permissions_od_measurements_and_detour_bound(self):
        root = self.routes()
        original = {v.get("id"): dict(v.attrib) for v in root if v.tag in ("vehicle", "flow")}
        apply_driver_variation(root, self.network, self.plan, 46)
        routes = {r.get("id"): r.get("edges").split() for r in root.findall("route")}
        types = {t.get("id"): t.get("vClass") for t in root.findall("vType")}
        baseline_length = sum(self.net.getEdge(e).getLength() for e in routes["original"])
        for vehicle in [v for v in root if v.tag in ("vehicle", "flow")]:
            edges = routes[vehicle.get("route")]
            vclass = types[vehicle.get("type")]
            self.assertEqual((edges[0], edges[-1]), ("begin", "exit"))
            self.assertEqual([e for e in edges if e in {"begin", "exit"}], ["begin", "exit"])
            self.assertLessEqual(sum(self.net.getEdge(e).getLength() for e in edges), 1.25 * baseline_length)
            self.assertTrue(all(self.net.getEdge(e).allows(vclass) for e in edges))
            self.assertTrue(all(self.net.getEdge(b) in self.net.getEdge(a).getAllowedOutgoing(vclass)
                                for a, b in zip(edges, edges[1:])))
            self.assertEqual(vehicle.get("departPosLat"), "center")
            for name in ("depart", "begin", "end", "number", "departLane", "departSpeed"):
                self.assertEqual(vehicle.get(name), original[vehicle.get("id")].get(name))
        self.assertEqual(len(root.findall("vehicle")), 60)
        self.assertEqual(len(root.findall("flow")), 1)

    def test_low_cooperation_and_zero_speed_gain_keep_scenario_intent(self):
        root = self.routes()
        root.find("vType[@id='car']").attrib.update(lcCooperative="0.1", lcSpeedGain="0", lcKeepRight="0")
        root.find("vType[@id='truck']").attrib.update(lcCooperative="0.15", lcSpeedGain="0", lcKeepRight="0")
        metadata = apply_driver_variation(root, self.network, self.plan, 46)
        for profile in metadata["parameters"].values():
            base = float(metadata["base_parameters"][profile["base_type"]]["lcCooperative"])
            self.assertGreaterEqual(float(profile["lcCooperative"]), .7 * base - 1e-4)
            self.assertLessEqual(float(profile["lcCooperative"]), 1.3 * base + 1e-4)
            self.assertEqual(profile["lcSpeedGain"], "0")
            self.assertEqual(profile["lcKeepRight"], "0")

    def test_required_intermediate_crossing_blocks_alternatives(self):
        root = self.routes()
        plan = copy.deepcopy(self.plan)
        plan["station_edges"]["middle"] = ["upper1"]
        metadata = apply_driver_variation(root, self.network, plan, 46)
        self.assertEqual(metadata["routes"]["eligible_vehicles"], 0)
        self.assertEqual(metadata["routes"]["rerouted_vehicles"], 0)
        self.assertEqual(metadata["routes"]["coverage"], 0)
        self.assertTrue(all(v.get("route") == "original" for v in root.findall("vehicle")))

    def test_vehicle_with_mandatory_stop_keeps_route(self):
        root = self.routes()
        ET.SubElement(root.find("vehicle"), "stop", lane="upper1_0", duration="10")
        metadata = apply_driver_variation(root, self.network, self.plan, 46)
        self.assertEqual(root.find("vehicle").get("route"), "original")
        self.assertEqual(metadata["routes"]["fixed_stop_entities"], 1)

    def test_explicit_editor_route_keeps_its_unmeasured_waypoint(self):
        root = self.routes()
        metadata = apply_driver_variation(root, self.network, self.plan, 46, fixed_routes={"original"})
        self.assertTrue(all(v.get("route") == "original" for v in root if v.tag in ("vehicle", "flow")))
        self.assertEqual(metadata["routes"]["rerouted_vehicles"], 0)
        self.assertEqual(metadata["routes"]["fixed_editor_route_entities"], 61)

    def test_builder_receipt_covers_filtered_surge_and_preserves_editor_waypoint(self):
        from scenario_editor import ScenarioEditor
        editor = ScenarioEditor(network_variant="route-safe")
        edits = editor.read()
        rid = "area_cd1421f60b86"
        edits["routes"][rid] = {"from": "25346083", "to": "11316055", "via": ["683926427#0"], "rate": 1}
        with tempfile.TemporaryDirectory() as folder:
            build = editor.build(edits, folder, "surge", 57600, 46, driver_profile="varied")
            root = ET.parse(build["routes"]).getroot()
            vehicles = root.findall("vehicle")
            self.assertEqual(build["driver_variation"]["vehicles_varied"], len(vehicles))
            self.assertTrue(any(v.get("id").startswith("surge_") for v in vehicles))
            paths = {r.get("id"): r.get("edges").split() for r in root.findall("route")}
            original_ids = {v.get("id") for v in editor.routes.findall("vehicle") if v.get("route") == rid}
            selected = [v for v in vehicles if v.get("id").removeprefix("surge_") in original_ids]
            self.assertTrue(selected)
            self.assertTrue(all("683926427#0" in paths[v.get("route")] for v in selected))


if __name__ == "__main__":
    unittest.main()
