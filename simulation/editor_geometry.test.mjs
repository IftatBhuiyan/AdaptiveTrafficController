import test from 'node:test';
import assert from 'node:assert/strict';
import {previewLanes, cutPolygon, polygonArea} from './editor_geometry.mjs';

test('four-to-five-lane preview widens a centered road and draws five lane centers', () => {
  const base = {spreadType: 'center', lanes: Array.from({length: 4}, (_, index) => ({index, width: 3.2, shape: [[0, (index - 1.5) * 3.2], [50, (index - 1.5) * 3.2]]}))};
  const result = previewLanes(base, {lanes: 5, speed: 11, closed: false});
  assert.equal(result.length, 5);
  assert.deepEqual(result.map(l => l.shape[0][1]), [-6.4, -3.2, 0, 3.2, 6.4]);
  assert.equal(result.at(-1).shape[0][1] - result[0].shape[0][1] + 3.2, 16);
});

test('cut retains the side the user clicks and does not mutate the original shape', () => {
  const original = [[0, 0], [10, 0], [10, 10], [0, 10]];
  assert.equal(polygonArea(cutPolygon(original, [4, -2], [4, 12], [1, 5])), 40);
  assert.equal(polygonArea(cutPolygon(original, [4, -2], [4, 12], [8, 5])), 60);
  assert.equal(polygonArea(original), 100);
  assert.throws(() => cutPolygon(original, [15, 0], [15, 10], [1, 5]), /must cross/);
});

import {lanesFromShape, roadBorders, stripPolygon, densify, simplify, smooth, snapPoint, projectToPolyline, pointAlong, polylineLength} from './editor_geometry.mjs';

test('lanesFromShape places right-spread lanes to the right of the border and centred lanes around the shape', () => {
  const shape = [[0, 0], [100, 0]];
  const right = lanesFromShape(shape, 2, [3.2, 3.2], 'right');
  const near = (actual, expected) => actual.forEach((v, i) => assert.ok(Math.abs(v - expected[i]) < 1e-9, `${v} ≠ ${expected[i]}`));
  near(right.map(l => l.shape[0][1]), [-4.8, -1.6]);
  const centre = lanesFromShape(shape, 2, null, 'center');
  near(centre.map(l => l.shape[0][1]), [-1.6, 1.6]);
  const wide = lanesFromShape(shape, 2, [4, 2], 'right');
  near(wide.map(l => l.shape[0][1]), [-4, -1]);
  near(roadBorders(shape, 2, null, 'right').right[0], [0, -6.4]);
});

test('stripPolygon builds a sidewalk strip hugging the road border', () => {
  const strip = stripPolygon([[0, 0], [10, 0]], 6.4, 8.4);
  assert.equal(strip.length, 4);
  assert.ok(Math.abs(strip[0][1] + 6.4) < 1e-9 && strip[0][0] === 0);
  assert.ok(Math.abs(strip[2][1] + 8.4) < 1e-9 && strip[2][0] === 10);
});

test('densify, simplify and smooth refine and reduce a line without moving its ends', () => {
  const line = [[0, 0], [10, 0], [10, 10]];
  const dense = densify(line, 2.5);
  assert.equal(dense.length, 9);
  assert.deepEqual(dense[0], [0, 0]);
  assert.deepEqual(dense.at(-1), [10, 10]);
  assert.deepEqual(simplify(dense, 0.01), line);
  const soft = smooth(line, 2);
  assert.deepEqual(soft[0], [0, 0]);
  assert.deepEqual(soft.at(-1), [10, 10]);
  assert.ok(soft.length > line.length);
  assert.ok(soft.every(p => p[0] <= 10 && p[1] >= 0));
  assert.equal(densify([[0, 0], [4, 0], [4, 4], [0, 4]], 2, true).length, 8);
});

test('snapPoint prefers vertices, then lines, then the grid', () => {
  const road = {kind: 'road', points: [[0, 0], [10, 0]]};
  assert.deepEqual(snapPoint([0.4, 0.3], {targets: [road], tolerance: 1}).point, [0, 0]);
  const line = snapPoint([5, 0.6], {targets: [road], tolerance: 1});
  assert.deepEqual(line.point, [5, 0]);
  assert.equal(line.guide.kind, 'line');
  assert.deepEqual(snapPoint([5.3, 7.7], {grid: 0.5, targets: [road], tolerance: 1}).point, [5.5, 7.5]);
  assert.equal(snapPoint([5.3, 7.7], {targets: [road]}).guide, null);
});

test('projectToPolyline and pointAlong agree on distances along a road', () => {
  const road = [[0, 0], [10, 0], [10, 10]];
  const hit = projectToPolyline([10, 3], road);
  assert.equal(hit.segment, 1);
  assert.equal(hit.along, 13);
  assert.deepEqual(pointAlong(road, 13).point, [10, 3]);
  assert.equal(polylineLength(road), 20);
});

import {snapSettings} from './editor_geometry.mjs';
test('snap reach is capped in metres when zoomed out and the grid hides when too fine', () => {
  assert.deepEqual(snapSettings(0.5, 0.5), {tolerance: 1.5, grid: 0});
  assert.deepEqual(snapSettings(20, 0.5), {tolerance: 0.5, grid: 0.5});
  const road = {kind: 'road', points: [[0, 0], [10, 0]]};
  const far = snapPoint([5, 4], {targets: [road], ...snapSettings(0.5, 0.5)});
  assert.equal(far.guide, null);
  assert.deepEqual(far.point, [5, 4]);
});
