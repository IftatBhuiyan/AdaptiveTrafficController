"""Validation and build checks for the precision map editor (needs SUMO's netconvert)."""
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from scenario_editor import ScenarioEditor


class PrecisionEdits(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.editor = ScenarioEditor()
        catalog = cls.editor.catalog()
        cls.roads, cls.junctions = catalog["base"]["roads"], catalog["base"]["junctions"]
        # A plain two-way residential junction: two roads in, two roads out, one lane each.
        for jid, node in cls.junctions.items():
            roads = node["incoming"] + node["outgoing"]
            if (node["controlType"] == "priority" and len(node["incoming"]) == 2 and len(node["outgoing"]) == 2
                    and all(cls.roads[r]["laneCount"] == 1 and not cls.roads[r]["protected"] and cls.roads[r]["length"] > 30 for r in roads)):
                cls.node, cls.road = jid, node["outgoing"][0]
                break

    def test_catalog_exposes_centrelines_and_junction_positions(self):
        road = self.roads[self.road]
        self.assertEqual(road["from"], self.node)
        self.assertGreaterEqual(len(road["shape"]), 2)
        self.assertAlmostEqual(road["shape"][0][0], self.junctions[self.node]["x"], places=1)
        self.assertTrue(self.roads["25345887"]["protected"])

    def test_validation_rejects_unsafe_edits(self):
        with self.assertRaisesRegex(ValueError, "150 m"):
            self.editor.validate({"nodes": {self.node: [self.junctions[self.node]["x"] + 400, self.junctions[self.node]["y"]]}})
        with self.assertRaisesRegex(ValueError, "cannot be removed"):
            self.editor.validate({"removed": {"25345887": True}})
        with self.assertRaisesRegex(ValueError, "rename"):
            self.editor.validate({"splits": {"25345887": [10]}})
        with self.assertRaisesRegex(ValueError, "2 m"):
            self.editor.validate({"splits": {self.road: [1]}})
        with self.assertRaisesRegex(ValueError, "one width for each"):
            self.editor.validate({"geometry": {self.road: {"widths": [3, 3]}}})
        with self.assertRaisesRegex(ValueError, "surface type"):
            self.editor.validate({"drawings": {"custom_1": {"kind": "lava", "shape": [[0, 0], [1, 0], [1, 1]]}}})

    def test_road_ends_follow_their_junction(self):
        x, y = self.junctions[self.node]["x"], self.junctions[self.node]["y"]
        shape = [list(p) for p in self.roads[self.road]["shape"]]
        shape[0] = [999, 999]  # a stale end is corrected server-side
        clean = self.editor.validate({"nodes": {self.node: [x + 3, y + 4]}, "geometry": {self.road: {"shape": shape}}})
        self.assertEqual(clean["nodes"][self.node], [round(x + 3, 3), round(y + 4, 3)])
        start = clean["geometry"][self.road]["shape"][0]
        self.assertAlmostEqual(start[0], self.roads[self.road]["shape"][0][0] + 3, places=2)
        self.assertAlmostEqual(start[1], self.roads[self.road]["shape"][0][1] + 4, places=2)

    def test_build_applies_move_reshape_width_split_and_drawing(self):
        x, y = self.junctions[self.node]["x"], self.junctions[self.node]["y"]
        road = self.roads[self.road]
        shape = [list(p) for p in road["shape"]]
        a, b = shape[0], shape[-1]
        shape = [a, [(a[0] + b[0]) / 2 + 6, (a[1] + b[1]) / 2 + 6], b]
        other = self.junctions[self.node]["incoming"][0]
        edits = {"nodes": {self.node: [x + 3, y + 4]},
                 "geometry": {self.road: {"shape": shape, "widths": [4.5]}},
                 "splits": {other: [12]},
                 "drawings": {"custom_walk": {"kind": "sidewalk", "shape": [[x, y], [x + 10, y], [x + 10, y + 3]]}}}
        with tempfile.TemporaryDirectory() as folder:
            built = self.editor.build(edits, folder)
            root = ET.parse(built["network"]).getroot()
            node = root.find(f"junction[@id='{self.node}']")
            self.assertAlmostEqual(float(node.get("x")), x + 3, places=1)
            lane = root.find(f"edge[@id='{self.road}']/lane")
            self.assertEqual(lane.get("width"), "4.50")
            self.assertTrue(any(e.get("id").startswith(other + ".") for e in root.findall("edge")))
            self.assertEqual(built["edits"]["drawings"]["custom_walk"]["kind"], "sidewalk")
            # The reshaped road bends through the new interior point.
            points = [[float(n) for n in p.split(",")] for p in lane.get("shape").split()]
            self.assertTrue(any(math.dist(p, shape[1]) < 3 for p in points))


if __name__ == "__main__":
    unittest.main()


def tl_programs(path):
    root = ET.parse(path).getroot()
    return {t.get("id"): [(p.get("duration"), p.get("state")) for p in t.findall("phase")] for t in root.findall("tlLogic")}


def strip(line, near, far):
    """Polygon between two right-hand offsets of a line (matches stripPolygon in editor_geometry.mjs)."""
    from scenario_editor import offset_line
    return offset_line(line, near) + offset_line(line, far)[::-1]


class SimulatedSurfaces(unittest.TestCase):
    """Drawn surfaces must change SUMO, and only where they are drawn."""

    @classmethod
    def setUpClass(cls):
        cls.editor = ScenarioEditor()
        cls.base_programs = tl_programs(cls.editor.base_net)
        cls.node = "astoria_31st_north"
        lane = cls.editor.edges["5703719#17"].getLanes()
        # A crosswalk band across both directions of 31st St, 6-10 m before the stop line.
        stop = lane[0].getShape()[-1]
        before = lane[0].getShape()[-2]
        dx, dy = stop[0] - before[0], stop[1] - before[1]
        length = math.hypot(dx, dy)
        ux, uy = dx / length, dy / length
        nx, ny = uy, -ux  # right of travel
        a = [stop[0] - ux * 10, stop[1] - uy * 10]
        b = [stop[0] - ux * 6, stop[1] - uy * 6]
        cls.crosswalk = [[a[0] + nx * 6, a[1] + ny * 6], [b[0] + nx * 6, b[1] + ny * 6],
                         [b[0] - nx * 16, b[1] - ny * 16], [a[0] - nx * 16, a[1] - ny * 16]]

    def test_freehand_sidewalk_along_a_road_edge_becomes_a_pedestrian_lane(self):
        edge = self.editor.edges["46139826#0"]
        lane0 = [list(p) for p in edge.getLanes()[0].getShape()]
        half = edge.getLanes()[0].getWidth() / 2
        clean = self.editor.validate({"drawings": {"custom_walk": {"kind": "sidewalk", "shape": strip(lane0, half + .1, half + 2.1)}}})
        self.assertIn("46139826#0", clean["effects"]["sidewalks"])
        self.assertAlmostEqual(clean["effects"]["sidewalks"]["46139826#0"], 2.0, delta=.5)
        self.assertIn("Simulated", clean["effects"]["notes"]["custom_walk"])

    def test_sidewalks_are_refused_on_measured_roads(self):
        edge = self.editor.edges["25345887"]
        lane0 = [list(p) for p in edge.getLanes()[0].getShape()]
        half = edge.getLanes()[0].getWidth() / 2
        clean = self.editor.validate({"drawings": {"custom_ramp": {"kind": "sidewalk", "shape": strip(lane0, half + .1, half + 2.1)}}})
        self.assertNotIn("25345887", clean["effects"]["sidewalks"])
        self.assertIn("measures", clean["effects"]["notes"]["custom_ramp"])

    def test_crosswalk_builds_a_crossing_pedestrians_and_touches_only_its_signal(self):
        edits = {"drawings": {"custom_x": {"kind": "crosswalk", "shape": self.crosswalk, "pedestrians": 240}}}
        clean = self.editor.validate(edits)
        [crossing] = clean["effects"]["crossings"]
        self.assertEqual(crossing["node"], self.node)
        self.assertEqual(set(crossing["edges"]), {"5703719#17", "-5703719#18"})
        with tempfile.TemporaryDirectory() as folder:
            built = self.editor.build(edits, folder)
            root = ET.parse(built["network"]).getroot()
            crossings = [e for e in root.findall("edge") if e.get("function") == "crossing"]
            self.assertEqual([set(e.get("crossingEdges").split()) for e in crossings], [{"5703719#17", "-5703719#18"}])
            changed = {t for t, phases in tl_programs(built["network"]).items() if phases != self.base_programs.get(t)}
            self.assertEqual(changed, {self.node})
            routes = built["args"][built["args"].index("-r") + 1].split(",")
            self.assertEqual(len(routes), 2)
            walks = ET.parse(routes[1]).getroot().findall("personFlow")
            self.assertTrue(walks)
            for flow in walks:
                self.assertEqual(len(flow.find("walk").get("edges").split()), 2)

    def test_plaza_makes_covered_road_pedestrian_only_and_median_closes_lane(self):
        edge = self.editor.edges["46139826#0"]
        shape = [list(p) for p in edge.getShape()]
        plaza = strip(shape, -12, 12)
        clean = self.editor.validate({"drawings": {"custom_p": {"kind": "plaza", "shape": plaza}}})
        self.assertIn("46139826#0", clean["effects"]["pedestrian_only"])
        lane = [list(p) for p in edge.getLanes()[-1].getShape()]
        width = edge.getLanes()[-1].getWidth()
        median = strip(lane, -width / 2 + .2, width / 2 - .2)
        clean = self.editor.validate({"drawings": {"custom_m": {"kind": "median", "shape": median}}})
        self.assertEqual(clean["effects"]["lane_closures"].get("46139826#0"), [len(edge.getLanes()) - 1])
        with tempfile.TemporaryDirectory() as folder:
            built = self.editor.build({"drawings": {"custom_m": {"kind": "median", "shape": median}}}, folder)
            root = ET.parse(built["network"]).getroot()
            last = root.find("edge[@id='46139826#0']").findall("lane")[-1]
            self.assertEqual(last.get("disallow"), "all")

    def test_custom_surfaces_stay_display_only(self):
        clean = self.editor.validate({"drawings": {"custom_c": {"kind": "custom", "shape": self.crosswalk}}})
        self.assertFalse(any(clean["effects"][k] for k in ("sidewalks", "crossings", "pedestrian_only", "lane_closures")))
        self.assertIn("Display only", clean["effects"]["notes"]["custom_c"])


class MedianTurns(unittest.TestCase):
    def test_median_in_a_junction_removes_turns_that_cut_through_it(self):
        editor = ScenarioEditor()
        node = editor.nodes["astoria_31st_north"]
        x, y = node.getCoord()
        square = [[x - 1.5, y - 1.5], [x + 1.5, y - 1.5], [x + 1.5, y + 1.5], [x - 1.5, y + 1.5]]
        edits = {"drawings": {"custom_m": {"kind": "median", "shape": square}}}
        with tempfile.TemporaryDirectory() as folder:
            first = Path(folder) / "probe"
            first.mkdir()
            deletions = editor.median_deletions(editor.base_net, editor.validate(edits)["effects"]["medians"])
            self.assertTrue(deletions)
            try:
                built = editor.build(edits, folder)
            except ValueError as error:
                # Acceptable only as an explicit routing refusal, never a crash.
                self.assertIn("disconnects", str(error))
                return
            root = ET.parse(built["network"]).getroot()
            kept = {(c.get("from"), c.get("to"), c.get("fromLane"), c.get("toLane")) for c in root.findall("connection")}
            for a, b, i, j in deletions:
                self.assertNotIn((a, b, str(i), str(j)), kept)


class BrowserSidewalkStrips(unittest.TestCase):
    """The Sidewalk-along-road tool's strip (map_editor.js sidewalkFor) must be recognised server-side."""

    def test_right_and_left_strips_land_on_the_right_road(self):
        editor = ScenarioEditor()
        roads = editor.catalog()["base"]["roads"]

        def browser_strip(eid, width=2.0):
            base = roads[eid]
            total = sum(lane["width"] for lane in base["lanes"])
            near = (0 if base["spreadType"] == "right" else -total / 2) + total
            return strip(base["shape"], near, near + width)

        pairs = [(a, b) for a, r in roads.items() for b, q in roads.items()
                 if r["from"] == q["to"] and r["to"] == q["from"] and a < b and not r["protected"] and not q["protected"]
                 and r["length"] > 40 and q["length"] > 40]
        self.assertTrue(pairs)
        for eid, reverse in pairs[:4]:
            ends = lambda e: {roads[e]["from"], roads[e]["to"]}
            for target, other in ((eid, reverse), (reverse, eid)):
                # "Left of travel" on one direction is the right-hand strip of the opposite road.
                effects = editor.validate({"drawings": {"custom_s": {"kind": "sidewalk", "shape": browser_strip(target)}}})["effects"]
                self.assertIn(target, effects["sidewalks"], effects["notes"])
                self.assertNotIn(other, effects["sidewalks"], "a strip must never land on the opposite direction")
                for extra in set(effects["sidewalks"]) - {target}:
                    self.assertTrue(ends(extra) & ends(target), f"{extra} does not adjoin {target}")
