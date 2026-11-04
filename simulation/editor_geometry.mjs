// Geometry used by the editor preview. SUMO rebuilds exact connections on save.
export function offsetLine(points, distance) {
  return points.map((p, i) => {
    const a = points[Math.max(0, i - 1)], b = points[Math.min(points.length - 1, i + 1)];
    const dx = b[0] - a[0], dy = b[1] - a[1], length = Math.hypot(dx, dy) || 1;
    return [p[0] + dy / length * distance, p[1] - dx / length * distance];
  });
}

export function previewLanes(base, spec) {
  const width = base.lanes[0].width;
  const middle = base.lanes[Math.floor(base.lanes.length / 2)];
  const center = offsetLine(middle.shape, (middle.index - (base.lanes.length - 1) / 2) * width);
  const left = base.lanes[base.lanes.length - 1].shape;
  return Array.from({length: spec.lanes}, (_, index) => ({
    index, laneCount: spec.lanes, width, speed: spec.speed, closed: spec.closed,
    shape: base.spreadType === 'center'
      ? offsetLine(center, ((spec.lanes - 1) / 2 - index) * width)
      : offsetLine(left, (spec.lanes - 1 - index) * width)
  }));
}

export function polygonArea(points) {
  return Math.abs(points.reduce((sum, p, i) => {
    const q = points[(i + 1) % points.length]; return sum + p[0] * q[1] - q[0] * p[1];
  }, 0)) / 2;
}

export function lineSide(a, b, p) {
  return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]);
}

export function cutPolygon(points, a, b, keep) {
  if (Math.hypot(b[0] - a[0], b[1] - a[1]) < .1) throw Error('Draw a longer cut line.');
  const side = lineSide(a, b, keep);
  if (Math.abs(side) < .01) throw Error('Click beside the line, on the part you want to keep.');
  const sign = Math.sign(side), values = points.map(p => lineSide(a, b, p) * sign);
  if (!values.some(v => v < -.001) || !values.some(v => v > .001)) throw Error('The cut line must cross the selected shape.');
  const crossings = values.filter((v, i) => v * values[(i + 1) % values.length] < 0).length;
  if (crossings > 2) throw Error('This cut crosses several parts of the shape. Cut one corner at a time.');
  const result = [];
  for (let i = 0; i < points.length; i++) {
    const previous = points[(i + points.length - 1) % points.length], current = points[i];
    const prevSide = lineSide(a, b, previous) * sign, curSide = lineSide(a, b, current) * sign;
    if ((prevSide >= 0) !== (curSide >= 0)) {
      const t = prevSide / (prevSide - curSide);
      result.push([previous[0] + t * (current[0] - previous[0]), previous[1] + t * (current[1] - previous[1])]);
    }
    if (curSide >= 0) result.push([...current]);
  }
  const clean = result.filter((p, i) => i === 0 || Math.hypot(p[0] - result[i - 1][0], p[1] - result[i - 1][1]) > .001);
  if (clean.length > 1 && Math.hypot(clean[0][0] - clean.at(-1)[0], clean[0][1] - clean.at(-1)[1]) < .001) clean.pop();
  if (clean.length < 3 || polygonArea(clean) < 1) throw Error('That would remove the whole shape. Choose the other side.');
  return clean;
}

// ---- Precision geometry used by the map editor's point tools ----
// All functions are pure. Points are [x, y] in SUMO network metres.

export function polylineLength(points) {
  let total = 0;
  for (let i = 1; i < points.length; i++) total += Math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1]);
  return total;
}

// Closest point on a polyline or polygon. Returns the segment index, the
// fraction along it, the projected point, the distance, and the distance
// along the line from its first point.
export function projectToPolyline(p, points, closed = false) {
  let best = null, running = 0;
  const count = closed ? points.length : points.length - 1;
  for (let i = 0; i < count; i++) {
    const a = points[i], b = points[(i + 1) % points.length];
    const dx = b[0] - a[0], dy = b[1] - a[1], length = dx * dx + dy * dy;
    const t = length ? Math.max(0, Math.min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length)) : 0;
    const point = [a[0] + t * dx, a[1] + t * dy], distance = Math.hypot(p[0] - point[0], p[1] - point[1]);
    if (!best || distance < best.distance) best = {segment: i, t, point, distance, along: running + t * Math.sqrt(length)};
    running += Math.sqrt(length);
  }
  return best;
}

export function pointAlong(points, along) {
  let running = 0;
  for (let i = 1; i < points.length; i++) {
    const a = points[i - 1], b = points[i], length = Math.hypot(b[0] - a[0], b[1] - a[1]);
    if (running + length >= along || i === points.length - 1) {
      const t = length ? Math.max(0, Math.min(1, (along - running) / length)) : 0;
      return {point: [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])], direction: length ? [(b[0] - a[0]) / length, (b[1] - a[1]) / length] : [1, 0]};
    }
    running += length;
  }
  return {point: [...points[0]], direction: [1, 0]};
}

// Lane centre lines from a SUMO edge shape. For spreadType "right" the
// shape is the left border and lanes sit to its right (index 0 outermost);
// for "center" / "roadCenter" the shape runs through the middle of the lanes.
export function lanesFromShape(shape, laneCount, widths, spreadType = 'right') {
  const w = Array.from({length: laneCount}, (_, i) => widths?.[i] ?? widths?.[0] ?? 3.2);
  const total = w.reduce((s, v) => s + v, 0);
  const lanes = [];
  let fromLeft = 0;
  for (let index = laneCount - 1; index >= 0; index--) {
    const centreFromLeft = fromLeft + w[index] / 2;
    const offset = spreadType === 'right' ? centreFromLeft : centreFromLeft - total / 2;
    lanes[index] = {index, width: w[index], shape: offsetLine(shape, offset)};
    fromLeft += w[index];
  }
  return lanes;
}

// Border of a road: distance 0 is the shape itself for "right" roads; for
// centred roads the borders are ±total/2.
export function roadBorders(shape, laneCount, widths, spreadType = 'right') {
  const total = Array.from({length: laneCount}, (_, i) => widths?.[i] ?? widths?.[0] ?? 3.2).reduce((s, v) => s + v, 0);
  const left = spreadType === 'right' ? 0 : -total / 2, right = spreadType === 'right' ? total : total / 2;
  return {left: offsetLine(shape, left), right: offsetLine(shape, right)};
}

// A strip polygon between two parallel offsets of a line, e.g. a sidewalk
// hugging the right border of a road.
export function stripPolygon(line, nearOffset, farOffset) {
  const near = offsetLine(line, nearOffset), far = offsetLine(line, farOffset).reverse();
  return [...near, ...far];
}

export function translate(points, dx, dy) { return points.map(p => [p[0] + dx, p[1] + dy]); }

// Insert points so that no segment is longer than `spacing` metres.
export function densify(points, spacing, closed = false) {
  if (!(spacing > 0.05)) throw Error('Point spacing must be larger than 5 cm.');
  const result = [];
  const count = closed ? points.length : points.length - 1;
  for (let i = 0; i < count; i++) {
    const a = points[i], b = points[(i + 1) % points.length];
    const pieces = Math.max(1, Math.ceil(Math.hypot(b[0] - a[0], b[1] - a[1]) / spacing));
    for (let k = 0; k < pieces; k++) result.push([a[0] + (b[0] - a[0]) * k / pieces, a[1] + (b[1] - a[1]) * k / pieces]);
  }
  if (!closed) result.push([...points[points.length - 1]]);
  if (result.length > 400) throw Error('That spacing would create more than 400 points. Use a larger spacing.');
  return result;
}

// Douglas–Peucker simplification. Endpoints of open lines are always kept.
export function simplify(points, tolerance, closed = false) {
  if (!(tolerance >= 0)) throw Error('Tolerance must be zero or more.');
  const keep = new Array(points.length).fill(false);
  const stack = closed ? [[0, points.length - 1]] : [[0, points.length - 1]];
  keep[0] = keep[points.length - 1] = true;
  while (stack.length) {
    const [start, end] = stack.pop();
    let farthest = -1, distance = tolerance;
    for (let i = start + 1; i < end; i++) {
      const d = perpendicularDistance(points[i], points[start], points[end]);
      if (d > distance) { distance = d; farthest = i; }
    }
    if (farthest >= 0) { keep[farthest] = true; stack.push([start, farthest], [farthest, end]); }
  }
  const result = points.filter((_, i) => keep[i]);
  if (closed) {
    // For polygons also test the closing segment: keep a far corner opposite the start.
    if (result.length < 3) return [points[0], points[Math.floor(points.length / 3)], points[Math.floor(2 * points.length / 3)]].map(p => [...p]);
    return result;
  }
  return result.length >= 2 ? result : [points[0], points[points.length - 1]];
}

function perpendicularDistance(p, a, b) {
  const dx = b[0] - a[0], dy = b[1] - a[1], length = Math.hypot(dx, dy);
  if (!length) return Math.hypot(p[0] - a[0], p[1] - a[1]);
  return Math.abs(dy * p[0] - dx * p[1] + b[0] * a[1] - b[1] * a[0]) / length;
}

// Chaikin corner cutting: a few iterations turn a coarse outline into a
// refined curve. Endpoints of open lines stay fixed.
export function smooth(points, iterations = 1, closed = false) {
  let current = points.map(p => [...p]);
  for (let step = 0; step < iterations; step++) {
    const next = [];
    const count = closed ? current.length : current.length - 1;
    if (!closed) next.push([...current[0]]);
    for (let i = 0; i < count; i++) {
      const a = current[i], b = current[(i + 1) % current.length];
      next.push([a[0] * .75 + b[0] * .25, a[1] * .75 + b[1] * .25], [a[0] * .25 + b[0] * .75, a[1] * .25 + b[1] * .75]);
    }
    if (!closed) next.push([...current[current.length - 1]]);
    current = next;
    if (current.length > 400) throw Error('Smoothing would create more than 400 points. Simplify first.');
  }
  return current;
}

// Snap a point to a grid, nearby vertices, or nearby lines. `targets` is a
// list of {kind, points, closed}. Returns {point, guide} where guide
// describes what the point snapped to, or null.
export function snapPoint(p, {grid = 0, targets = [], tolerance = 1} = {}) {
  let best = null;
  for (const target of targets) {
    for (const [index, q] of target.points.entries()) {
      const d = Math.hypot(q[0] - p[0], q[1] - p[1]);
      if (d <= tolerance && (!best || d < best.distance)) best = {point: [...q], distance: d, guide: {kind: 'vertex', target, index}};
    }
  }
  if (!best) for (const target of targets) {
    if (target.points.length < 2) continue;
    const hit = projectToPolyline(p, target.points, target.closed);
    if (hit && hit.distance <= tolerance && (!best || hit.distance < best.distance)) best = {point: hit.point, distance: hit.distance, guide: {kind: 'line', target, segment: hit.segment}};
  }
  if (best) return {point: best.point, guide: best.guide};
  if (grid > 0) return {point: [Math.round(p[0] / grid) * grid, Math.round(p[1] / grid) * grid], guide: {kind: 'grid'}};
  return {point: [...p], guide: null};
}

export function bounds(points) {
  const xs = points.map(p => p[0]), ys = points.map(p => p[1]);
  return [Math.min(...xs), Math.min(...ys), Math.max(...xs), Math.max(...ys)];
}

// Snapping reach: 10 screen pixels, but never more than `maxMetres`, so a
// zoomed-out drag cannot jump to a neighbouring road. The grid only applies
// when its cells are at least 6 px wide on screen.
export function snapSettings(scale, grid, maxMetres = 1.5) {
  return {tolerance: Math.min(10 / scale, maxMetres), grid: grid > 0 && grid * scale >= 6 ? grid : 0};
}
