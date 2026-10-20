// Precision map editor: Photoshop-style point tools for roads, junctions,
// paved areas, buildings, and drawn surfaces. All edits are staged in the
// shared draft; SUMO's netconvert rebuilds lanes, connections, and junction
// shapes when the draft is applied.
import {snapSettings, lanesFromShape, roadBorders, stripPolygon, densify, simplify, smooth, snapPoint, projectToPolyline, pointAlong,
        polylineLength, translate, polygonArea, bounds} from './editor_geometry.mjs';

export const TOOLS = {
  select: {key: 'v', label: 'Select & move points', hint: 'Click a road, junction, or area to show its points. Drag a point or the whole object. Drag a road end to move its junction; every connected road follows. Double-click an outline to add a point.'},
  add: {key: 'a', label: 'Add point', hint: 'Click on an outline or road centreline to insert a point there.'},
  remove: {key: 'x', label: 'Delete point', hint: 'Click a point to delete it. Road ends and the last three corners of an area stay.'},
  split: {key: 's', label: 'Split road', hint: 'Hover a road to see the cut position in metres, then click to split it there. SUMO creates a junction at the cut.'},
  draw: {key: 'd', label: 'Draw surface', hint: 'Click to place corners of a sidewalk, median, plaza, or crosswalk. Enter or double-click finishes; Backspace removes the last corner; Esc cancels.'},
  sidewalk: {key: 'w', label: 'Sidewalk along road', hint: 'Hover a road to preview a sidewalk hugging its edge, then click to create it. Refine its points afterwards with Select.'},
  measure: {key: 'm', label: 'Measure', hint: 'Click two points to measure the distance between them.'},
  hand: {key: 'h', label: 'Pan', hint: 'Drag to pan the map. Scroll to zoom.'}
};

const MIN_ROAD_LENGTH = 1;

export function createPrecisionEditor(host) {
  const {$, canvas, ctx, xy, world} = host;
  const state = {
    tool: 'select', selection: null, vertices: new Set(), hover: null, drag: null, drawing: null, measure: [],
    snap: true, grid: 0.5, guide: null, pointer: null, altKey: false, lastClick: 0
  };
  const ids = () => ({draft: host.getDraft(), catalog: host.getCatalog(), net: host.getNet(), camera: host.getCamera()});
  const metres = pixels => pixels / host.getCamera().scale;

  // ---- Effective geometry, including staged edits ----
  function nodeBase(jid) {
    const {catalog, net} = ids();
    const base = catalog?.base?.junctions[jid];
    if (base) return [base.x, base.y];
    const node = net?.junctionById?.get(jid);
    return node ? [node.x, node.y] : null;
  }
  function nodePosition(jid) {
    const {draft} = ids();
    return draft?.nodes?.[jid] ? [...draft.nodes[jid]] : nodeBase(jid);
  }
  function nodeDelta(jid) {
    const {draft} = ids();
    const moved = draft?.nodes?.[jid];
    if (!moved) return [0, 0];
    const base = nodeBase(jid);
    return base ? [moved[0] - base[0], moved[1] - base[1]] : [0, 0];
  }
  function road(eid) { return ids().catalog?.base?.roads[eid]; }
  function roadShape(eid) {
    const {draft} = ids();
    const base = road(eid);
    if (!base) return null;
    const shape = (draft.geometry?.[eid]?.shape || base.shape).map(p => [...p]);
    const from = nodeDelta(base.from), to = nodeDelta(base.to);
    shape[0] = [base.shape[0][0] + from[0], base.shape[0][1] + from[1]];
    shape[shape.length - 1] = [base.shape.at(-1)[0] + to[0], base.shape.at(-1)[1] + to[1]];
    return shape;
  }
  function laneCount(eid) { const {draft} = ids(); return draft.roads?.[eid]?.lanes ?? road(eid).laneCount; }
  function roadWidths(eid) {
    const {draft} = ids();
    const base = road(eid), count = laneCount(eid);
    const widths = draft.geometry?.[eid]?.widths;
    if (widths && widths.length === count) return [...widths];
    return Array.from({length: count}, (_, i) => base.lanes[i]?.width ?? base.lanes[0].width);
  }
  function previewLanes(eid) {
    const base = road(eid);
    return lanesFromShape(roadShape(eid), laneCount(eid), roadWidths(eid), base.spreadType);
  }
  function borders(eid) { return roadBorders(roadShape(eid), laneCount(eid), roadWidths(eid), road(eid).spreadType); }
  function isRemoved(eid) { return !!ids().draft.removed?.[eid]; }
  // Roads whose displayed lanes differ from the loaded network.
  function touchedRoads() {
    const {draft, catalog} = ids();
    const touched = new Set([...Object.keys(draft.geometry || {}), ...Object.keys(draft.removed || {})]);
    for (const jid of Object.keys(draft.nodes || {})) {
      const node = catalog.base.junctions[jid];
      if (node) for (const eid of [...node.incoming, ...node.outgoing]) touched.add(eid);
    }
    return touched;
  }
  function junctionShape(jid) {
    const {draft, catalog} = ids();
    const base = catalog.base.junctions[jid];
    const shape = draft.surfaces?.[jid] || base?.shape || [];
    const delta = nodeDelta(jid);
    return delta[0] || delta[1] ? translate(shape, delta[0], delta[1]) : shape.map(p => [...p]);
  }
  function artifactShape(pid) {
    const {draft, catalog, net} = ids();
    return (draft.artifacts?.[pid]?.shape || catalog.base.artifacts[pid]?.shape || net.polygons.find(p => p.id === pid)?.shape || []).map(p => [...p]);
  }

  // ---- Selection model ----
  function selectionPoints(selection = state.selection) {
    if (!selection) return [];
    const {draft} = ids();
    switch (selection.kind) {
      case 'road': return roadShape(selection.id) || [];
      case 'node': return [nodePosition(selection.id)];
      case 'junction': return junctionShape(selection.id);
      case 'artifact': return artifactShape(selection.id);
      case 'drawing': return (draft.drawings?.[selection.id]?.shape || []).map(p => [...p]);
    }
    return [];
  }
  const closed = selection => selection && selection.kind !== 'road' && selection.kind !== 'node';
  function putPoints(points, selection = state.selection) {
    const {draft, catalog} = ids();
    switch (selection.kind) {
      case 'road': {
        const base = road(selection.id);
        if (polylineLength(points) < MIN_ROAD_LENGTH) throw Error('A road needs at least one metre of length.');
        draft.geometry[selection.id] = {...draft.geometry[selection.id], shape: points};
        return;
      }
      case 'node': moveNode(selection.id, points[0]); return;
      case 'junction': {
        const delta = nodeDelta(selection.id);
        draft.surfaces[selection.id] = translate(points, -delta[0], -delta[1]);
        return;
      }
      case 'artifact': draft.artifacts[selection.id] = {...draft.artifacts[selection.id], shape: points}; return;
      case 'drawing': draft.drawings[selection.id] = {...draft.drawings[selection.id], shape: points}; return;
    }
  }
  function moveNode(jid, point) {
    const {draft, catalog} = ids();
    const base = nodeBase(jid);
    if (Math.hypot(point[0] - base[0], point[1] - base[1]) > 150) throw Error('Keep a junction within 150 m of its original position.');
    if (Math.hypot(point[0] - base[0], point[1] - base[1]) < .005) delete draft.nodes[jid]; else draft.nodes[jid] = [point[0], point[1]];
    const node = catalog.base.junctions[jid];
    // Connected roads keep their interior points; only their ends follow (same rule as netconvert).
    for (const eid of [...node.incoming, ...node.outgoing]) {
      const geometry = draft.geometry?.[eid];
      if (geometry?.shape) geometry.shape = roadShape(eid);
    }
  }
  function describe(selection = state.selection) {
    if (!selection) return 'Nothing selected';
    const {catalog, net, draft} = ids();
    if (selection.kind === 'road') { const r = road(selection.id); return `${r.name || 'Road'} · ${selection.id}`; }
    if (selection.kind === 'node') { const node = net.junctionById?.get(selection.id); return `Junction ${node?.streets?.join(' / ') || ''} · ${selection.id}`.replace('  ', ' '); }
    if (selection.kind === 'junction') return `Paved area · ${selection.id}`;
    if (selection.kind === 'artifact') return `${net.polygons.find(p => p.id === selection.id)?.type || 'Map polygon'} · ${selection.id}`;
    if (selection.kind === 'drawing') return `${draft.drawings[selection.id]?.kind || 'surface'} · ${draft.drawings[selection.id]?.label || selection.id}`;
    return selection.id;
  }
  function select(selection, vertex = -1) {
    state.selection = selection; state.vertices = new Set(vertex >= 0 ? [vertex] : []);
    refreshPanel();
  }
  function clearSelection() { state.selection = null; state.vertices = new Set(); refreshPanel(); }

  // ---- Hit testing ----
  function vertexAt(p, selection = state.selection) {
    const tolerance = metres(9);
    let best = -1, distance = tolerance;
    selectionPoints(selection).forEach((q, i) => { const d = Math.hypot(q[0] - p[0], q[1] - p[1]); if (d < distance) { distance = d; best = i; } });
    return best;
  }
  function nodeAt(p) {
    const {catalog} = ids();
    const tolerance = metres(10);
    let best = null, distance = tolerance;
    for (const jid of Object.keys(catalog.base.junctions)) {
      const q = nodePosition(jid); const d = Math.hypot(q[0] - p[0], q[1] - p[1]);
      if (d < distance) { distance = d; best = jid; }
    }
    return best;
  }
  function roadAt(p) {
    const {catalog} = ids();
    const tolerance = metres(8);
    let best = null, distance = Infinity;
    for (const eid of Object.keys(catalog.base.roads)) {
      if (isRemoved(eid)) continue;
      const shape = roadShape(eid);
      const box = bounds(shape); const margin = 20;
      if (p[0] < box[0] - margin || p[0] > box[2] + margin || p[1] < box[1] - margin || p[1] > box[3] + margin) continue;
      const half = roadWidths(eid).reduce((s, w) => s + w, 0) / 2;
      const centre = road(eid).spreadType === 'right' ? lanesFromShape(shape, 1, [half * 2], 'right')[0].shape : shape;
      const hit = projectToPolyline(p, centre);
      const d = Math.max(0, hit.distance - half);
      if (d < tolerance && d < distance) { distance = d; best = {id: eid, hit: projectToPolyline(p, shape)}; }
    }
    return best;
  }
  function inside(p, shape) {
    let yes = false;
    for (let i = 0, j = shape.length - 1; i < shape.length; j = i++) {
      const a = shape[i], b = shape[j];
      if ((a[1] > p[1]) !== (b[1] > p[1]) && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]) yes = !yes;
    }
    return yes;
  }
  function polygonAt(p) {
    const {draft, catalog, net} = ids();
    const hits = [];
    for (const [id, drawing] of Object.entries(draft.drawings || {})) if (inside(p, drawing.shape)) hits.push({kind: 'drawing', id, area: polygonArea(drawing.shape)});
    for (const jid of Object.keys(catalog.base.junctions)) { const shape = junctionShape(jid); if (shape.length > 2 && inside(p, shape)) hits.push({kind: 'junction', id: jid, area: polygonArea(shape)}); }
    for (const poly of net.polygons) {
      if (poly.drawing) continue;
      const hidden = draft.artifacts?.[poly.id]?.hidden ?? poly.hidden;
      if (hidden) continue;
      const shape = artifactShape(poly.id);
      if (inside(p, shape)) hits.push({kind: 'artifact', id: poly.id, area: polygonArea(shape)});
    }
    hits.sort((a, b) => a.area - b.area);
    return hits[0] ? {kind: hits[0].kind, id: hits[0].id} : null;
  }
  function objectAt(p) {
    const node = nodeAt(p); if (node) return {kind: 'node', id: node};
    const r = roadAt(p); if (r) return {kind: 'road', id: r.id};
    return polygonAt(p);
  }

  // ---- Snapping ----
  function snapTargets(exclude) {
    const {catalog, draft} = ids();
    const p = state.pointer, reach = 60;
    const targets = [];
    if (!p) return targets;
    for (const eid of Object.keys(catalog.base.roads)) {
      if (isRemoved(eid) || (exclude?.kind === 'road' && exclude.id === eid)) continue;
      const shape = roadShape(eid), box = bounds(shape);
      if (p[0] < box[0] - reach || p[0] > box[2] + reach || p[1] < box[1] - reach || p[1] > box[3] + reach) continue;
      const b = borders(eid);
      targets.push({kind: 'road border', id: eid, points: b.left}, {kind: 'road border', id: eid, points: b.right});
    }
    for (const jid of Object.keys(catalog.base.junctions)) {
      if (exclude?.kind === 'node' && exclude.id === jid) continue;
      const q = nodePosition(jid);
      if (Math.hypot(q[0] - p[0], q[1] - p[1]) < reach) targets.push({kind: 'junction', id: jid, points: [q]});
    }
    for (const [id, drawing] of Object.entries(draft.drawings || {})) {
      if (exclude?.kind === 'drawing' && exclude.id === id) continue;
      targets.push({kind: 'surface', id, points: drawing.shape, closed: true});
    }
    return targets;
  }
  function snapped(p, exclude) {
    if (!state.snap || state.altKey) { state.guide = null; return [...p]; }
    const result = snapPoint(p, {targets: snapTargets(exclude), ...snapSettings(host.getCamera().scale, state.grid)});
    state.guide = result.guide ? {...result.guide, point: result.point} : null;
    return result.point;
  }

  // ---- Edits ----
  function insertVertex(selection, p) {
    const points = selectionPoints(selection);
    const hit = projectToPolyline(p, points, closed(selection));
    if (!hit || hit.distance > metres(14)) return false;
    points.splice(hit.segment + 1, 0, hit.point);
    putPoints(points, selection);
    state.selection = selection; state.vertices = new Set([hit.segment + 1]);
    host.changed(`Point added · ${points.length} points.`);
    return true;
  }
  function deleteVertices(selection, indices) {
    const points = selectionPoints(selection);
    const removable = [...indices].filter(i => selection.kind === 'road' ? i > 0 && i < points.length - 1 : true).sort((a, b) => b - a);
    if (!removable.length) return host.message(selection.kind === 'road' ? 'Road ends belong to their junctions. Move the junction instead.' : 'Choose a point to delete.', true);
    const minimum = closed(selection) ? 3 : 2;
    if (points.length - removable.length < minimum) return host.message(`Keep at least ${minimum} points.`, true);
    for (const i of removable) points.splice(i, 1);
    putPoints(points, selection); state.vertices = new Set();
    host.changed(`${removable.length} point${removable.length > 1 ? 's' : ''} deleted · ${points.length} left.`);
  }
  function refine(action) {
    const selection = state.selection;
    if (!selection || selection.kind === 'node') return host.message('Select a road or an area first.', true);
    const points = selectionPoints(selection), isClosed = closed(selection);
    try {
      let result;
      if (action === 'densify') result = densify(points, Number($('geo-spacing').value) || 2, isClosed);
      else if (action === 'simplify') result = simplify(points, Number($('geo-tolerance').value) || .3, isClosed);
      else if (action === 'smooth') result = smooth(points, 1, isClosed);
      else if (action === 'straighten') result = selection.kind === 'road' ? [points[0], points.at(-1)] : simplify(points, 1e9, true);
      if (isClosed && polygonArea(result) < 1) throw Error('That would collapse the area.');
      putPoints(result, selection); state.vertices = new Set();
      host.changed(`${{densify: 'Points added', simplify: 'Points reduced', smooth: 'Outline smoothed', straighten: 'Straightened'}[action]} · ${result.length} points.`);
    } catch (error) { host.message(error.message, true); }
  }
  function resetSelection() {
    const {draft} = ids(), selection = state.selection;
    if (!selection) return;
    if (selection.kind === 'road') { delete draft.geometry[selection.id]; delete draft.splits[selection.id]; delete draft.removed[selection.id]; }
    else if (selection.kind === 'node') moveNode(selection.id, nodeBase(selection.id));
    else if (selection.kind === 'junction') delete draft.surfaces[selection.id];
    else if (selection.kind === 'artifact') delete draft.artifacts[selection.id];
    else if (selection.kind === 'drawing') { delete draft.drawings[selection.id]; clearSelection(); }
    state.vertices = new Set();
    host.changed('Original geometry restored in the preview.');
  }
  function addSplit(eid, along) {
    const {draft} = ids(), base = road(eid);
    if (base.protected) return host.message(`${base.name || 'This road'} is measured by the demand plan or the ramp model. Splitting would rename it; split a neighbouring road instead.`, true);
    const length = polylineLength(roadShape(eid));
    along = Math.round(along * 100) / 100;
    if (along < 2 || along > length - 2) return host.message(`Keep the cut at least 2 m from both ends of this ${length.toFixed(1)} m road.`, true);
    const list = draft.splits[eid] || [];
    if (list.some(v => Math.abs(v - along) < 2)) return host.message('There is already a cut within 2 m of that point.', true);
    if (list.length >= 8) return host.message('At most 8 cuts per road.', true);
    draft.splits[eid] = [...list, along].sort((a, b) => a - b);
    select({kind: 'road', id: eid});
    host.changed(`Road split staged at ${along.toFixed(2)} m. SUMO creates the junction when you apply.`);
  }
  function finishDrawing() {
    const {draft} = ids();
    if (!state.drawing || state.drawing.length < 3) return host.message('Place at least three corners first.', true);
    if (polygonArea(state.drawing) < 1) return host.message('The surface is too small.', true);
    const id = 'custom_' + Date.now().toString(36);
    const kind = $('geo-draw-kind').value;
    draft.drawings[id] = {kind, shape: state.drawing, label: '', ...(kind === 'crosswalk' ? {pedestrians: 120} : {})};
    state.drawing = null;
    select({kind: 'drawing', id});
    host.changed('Surface drawn. Refine its points with Select, or set its type and colour.');
  }
  // SUMO puts sidewalks on the right of travel. "Left" therefore means the right-hand edge of the
  // opposite-direction road; a one-way road has no left sidewalk in SUMO.
  function reverseOf(eid) {
    const {catalog} = ids(), base = road(eid);
    return Object.entries(catalog.base.roads).find(([id, r]) => r.from === base.to && r.to === base.from && !isRemoved(id))?.[0] || null;
  }
  function sidewalkFor(eid) {
    const width = Math.max(.5, Math.min(10, Number($('geo-walk-width').value) || 2));
    const target = $('geo-walk-side').value === 'left' ? reverseOf(eid) : eid;
    if (!target) return null;
    const shape = roadShape(target);
    const total = roadWidths(target).reduce((s, w) => s + w, 0);
    const near = (road(target).spreadType === 'right' ? 0 : -total / 2) + total;
    return {road: target, strip: stripPolygon(shape, near, near + width)};
  }

  // ---- Pointer handling. Returns true when the event was consumed. ----
  function pointerDown(p, event) {
    state.altKey = event.altKey;
    const now = performance.now(), double = now - state.lastClick < 350; state.lastClick = now;
    if (state.tool === 'hand') return false;
    if (state.tool === 'measure') { state.measure = state.measure.length === 1 ? [state.measure[0], snapped(p)] : [snapped(p)]; return true; }
    if (state.tool === 'draw') {
      if (double && state.drawing?.length >= 3) { finishDrawing(); return true; }
      const q = snapped(p);
      state.drawing = [...(state.drawing || []), q];
      host.message(`${state.drawing.length} corner${state.drawing.length > 1 ? 's' : ''} placed. Enter or double-click to finish.`);
      return true;
    }
    if (state.tool === 'sidewalk') {
      const r = roadAt(p); if (!r) return host.message('Hover a road, then click.', true), true;
      const walk = sidewalkFor(r.id);
      if (!walk) return host.message('This road is one-way, so SUMO has no left-side sidewalk for it. Choose "Right of travel", or pick the other side\'s road.', true), true;
      if (road(walk.road).protected) return host.message(`${road(walk.road).name || walk.road} is a measured road; the controller refers to its lanes by number, so it cannot get a sidewalk.`, true), true;
      const {draft} = ids(); const id = 'custom_' + Date.now().toString(36);
      draft.drawings[id] = {kind: 'sidewalk', shape: walk.strip, label: (road(walk.road).name || walk.road) + ' sidewalk'};
      select({kind: 'drawing', id}); host.changed('Sidewalk created along the road edge. Switch to Select to refine its points.');
      return true;
    }
    if (state.tool === 'split') {
      const r = roadAt(p); if (!r) return host.message('Click on a road.', true), true;
      addSplit(r.id, r.hit.along); return true;
    }
    if (state.tool === 'add') {
      if (state.selection && insertVertex(state.selection, p)) return true;
      const target = objectAt(p);
      if (target && target.kind !== 'node') { if (insertVertex(target, p)) return true; select(target); return true; }
      return host.message('Click on an outline or road centreline.', true), true;
    }
    if (state.tool === 'remove') {
      if (state.selection) { const v = vertexAt(p); if (v >= 0) { deleteVertices(state.selection, [v]); return true; } }
      const target = objectAt(p);
      if (target && target.kind !== 'node') { select(target); const v = vertexAt(p); if (v >= 0) deleteVertices(target, [v]); else host.message('Now click one of its points.'); return true; }
      return host.message('Click a point of a road or area.', true), true;
    }
    // Select tool
    if (state.selection) {
      const v = vertexAt(p);
      if (v >= 0) {
        if (double && state.selection.kind !== 'node') return true;
        if (event.shiftKey) { state.vertices.has(v) ? state.vertices.delete(v) : state.vertices.add(v); refreshPanel(); return true; }
        if (!state.vertices.has(v)) state.vertices = new Set([v]);
        const points = selectionPoints();
        if (state.selection.kind === 'road' && (v === 0 || v === points.length - 1)) {
          // Dragging a road end drags its junction and every road attached to it.
          const base = road(state.selection.id); const jid = v === 0 ? base.from : base.to;
          state.drag = {kind: 'node', id: jid, start: p, origin: nodePosition(jid), snapshot: structuredClone(ids().draft)};
          host.message('Moving the junction: all connected roads follow.');
          return true;
        }
        state.drag = {kind: 'vertices', selection: state.selection, indices: [...state.vertices], start: p, points, snapshot: structuredClone(ids().draft)};
        refreshPanel();
        return true;
      }
      if (double) { if (state.selection.kind !== 'node' && insertVertex(state.selection, p)) return true; }
    }
    const target = objectAt(p);
    if (target) {
      const same = state.selection && target.kind === state.selection.kind && target.id === state.selection.id;
      if (!same) select(target);
      if (target.kind === 'node') state.drag = {kind: 'node', id: target.id, start: p, origin: nodePosition(target.id), snapshot: structuredClone(ids().draft)};
      else state.drag = {kind: 'body', selection: target, start: p, points: selectionPoints(target), snapshot: structuredClone(ids().draft), moved: false};
      host.message(TOOLS.select.hint);
      return true;
    }
    if (state.selection) { clearSelection(); host.message('Selection cleared.'); }
    return false;
  }
  function pointerMove(p, event) {
    state.pointer = p; state.altKey = event.altKey; state.guide = null;
    if (state.drag) {
      const drag = state.drag;
      try {
        if (drag.kind === 'node') { moveNode(drag.id, snapped(p, {kind: 'node', id: drag.id})); }
        else if (drag.kind === 'vertices') {
          const points = drag.points.map(q => [...q]);
          if (drag.indices.length === 1) points[drag.indices[0]] = snapped(p, drag.selection);
          else { const dx = p[0] - drag.start[0], dy = p[1] - drag.start[1]; for (const i of drag.indices) points[i] = [drag.points[i][0] + dx, drag.points[i][1] + dy]; }
          putPoints(points, drag.selection);
        } else if (drag.kind === 'body') {
          const dx = p[0] - drag.start[0], dy = p[1] - drag.start[1];
          if (Math.hypot(dx, dy) * host.getCamera().scale > 3) drag.moved = true;
          if (!drag.moved) return true;
          const points = drag.points.map(q => [...q]);
          const lock = drag.selection.kind === 'road' ? new Set([0, points.length - 1]) : new Set();
          for (const [i, q] of points.entries()) if (!lock.has(i)) { q[0] += dx; q[1] += dy; }
          putPoints(points, drag.selection);
        }
        host.invalidate();
      } catch (error) { host.message(error.message, true); }
      return true;
    }
    if (state.tool === 'hand') { canvas.style.cursor = 'grab'; return false; }
    state.hover = null;
    if (state.tool === 'select' || state.tool === 'add' || state.tool === 'remove') {
      const v = state.selection ? vertexAt(p) : -1;
      if (v >= 0) state.hover = {vertex: v};
      else { const target = objectAt(p); if (target) state.hover = {target}; }
      canvas.style.cursor = v >= 0 ? (state.tool === 'remove' ? 'not-allowed' : 'grab') : state.hover ? 'pointer' : state.tool === 'select' ? 'default' : 'crosshair';
    } else if (state.tool === 'split' || state.tool === 'sidewalk') {
      const r = roadAt(p); state.hover = r ? {road: r} : null; canvas.style.cursor = r ? 'crosshair' : 'default';
    } else { snapped(p); canvas.style.cursor = 'crosshair'; }
    return state.tool !== 'select' || !!state.hover;
  }
  function pointerUp(p) {
    const drag = state.drag; state.drag = null; state.guide = null;
    if (!drag) return false;
    if (drag.kind === 'body' && !drag.moved) { refreshPanel(); return true; }
    if (drag.kind === 'node') host.changed('Junction moved. Connected roads and its paved area follow; SUMO rebuilds the lanes when you apply.');
    else if (drag.kind === 'vertices') host.changed(`Point${drag.indices.length > 1 ? 's' : ''} moved.`);
    else host.changed(drag.selection.kind === 'road' ? 'Road interior moved; its ends stay with their junctions.' : 'Area moved.');
    refreshPanel();
    return true;
  }
  function cancel() {
    if (state.drag?.snapshot) { host.replaceDraft(state.drag.snapshot); state.drag = null; host.invalidate(); host.message('Move cancelled.'); return true; }
    if (state.drawing) { state.drawing = null; host.message('Drawing cancelled.'); return true; }
    if (state.measure.length) { state.measure = []; return true; }
    if (state.selection) { clearSelection(); host.message('Selection cleared.'); return true; }
    return false;
  }
  function keydown(event) {
    const typing = /^(INPUT|SELECT|TEXTAREA)$/.test(document.activeElement?.tagName);
    if (typing) return false;
    const key = event.key.toLowerCase();
    if (!event.metaKey && !event.ctrlKey) {
      for (const [name, tool] of Object.entries(TOOLS)) if (tool.key === key) { setTool(name); return true; }
      if (key === 'g') { state.snap = !state.snap; refreshPanel(); host.message(state.snap ? 'Snapping on: points snap to road edges, junctions, surfaces, then the grid. Hold Alt to drag freely.' : 'Snapping off.'); return true; }
      if (key === 'enter' && state.tool === 'draw' && state.drawing) { finishDrawing(); return true; }
      if (key === 'backspace' && state.tool === 'draw' && state.drawing) { state.drawing.pop(); if (!state.drawing.length) state.drawing = null; return true; }
      if ((key === 'backspace' || key === 'delete') && state.selection && state.vertices.size) { deleteVertices(state.selection, state.vertices); return true; }
      if (key.startsWith('arrow') && state.selection && (state.vertices.size || state.selection.kind === 'node')) {
        const step = event.shiftKey ? 1 : .1;
        const dx = key === 'arrowleft' ? -step : key === 'arrowright' ? step : 0, dy = key === 'arrowup' ? step : key === 'arrowdown' ? -step : 0;
        const points = selectionPoints();
        const indices = state.selection.kind === 'node' ? [0] : [...state.vertices];
        for (const i of indices) points[i] = [points[i][0] + dx, points[i][1] + dy];
        try { putPoints(points); host.changed(`Nudged ${step} m.`); } catch (error) { host.message(error.message, true); }
        return true;
      }
    }
    return false;
  }
  function setTool(name) {
    state.tool = name; state.drawing = null; state.measure = []; state.hover = null; state.guide = null;
    document.querySelectorAll('[data-ptool]').forEach(button => button.classList.toggle('active-tool', button.dataset.ptool === name));
    canvas.style.cursor = name === 'hand' ? 'grab' : name === 'select' ? 'default' : 'crosshair';
    host.message(TOOLS[name].hint);
    refreshPanel();
  }

  // ---- Properties panel ----
  function refreshPanel() {
    if (!$('geo-selected')) return;
    const {draft} = ids(), selection = state.selection;
    $('geo-selected').textContent = describe();
    $('geo-snap').checked = state.snap; $('geo-grid').value = state.grid;
    const points = selectionPoints();
    const single = state.vertices.size === 1 ? [...state.vertices][0] : selection?.kind === 'node' ? 0 : -1;
    $('geo-point-fields').hidden = single < 0;
    $('geo-points').textContent = selection ? `${points.length} point${points.length === 1 ? '' : 's'}${state.vertices.size ? ` · ${state.vertices.size} selected` : ''}` : 'Pick an object on the map with a palette tool.';
    if (single >= 0 && points[single]) { $('geo-x').value = points[single][0].toFixed(2); $('geo-y').value = points[single][1].toFixed(2); $('geo-point-title').textContent = selection.kind === 'node' ? 'Junction position (m)' : `Point ${single + 1} of ${points.length} (m)`; }
    $('geo-refine').hidden = !selection || selection.kind === 'node';
    $('geo-straighten').textContent = selection?.kind === 'road' ? 'Straighten road' : 'Convex outline';
    $('geo-reset').disabled = !selection;
    $('geo-road').hidden = selection?.kind !== 'road';
    $('geo-drawing').hidden = selection?.kind !== 'drawing';
    $('geo-undo').disabled = !host.canUndo(); $('geo-redo').disabled = !host.canRedo();
    $('palette-undo').disabled = !host.canUndo(); $('palette-redo').disabled = !host.canRedo();
    $('palette-snap').classList.toggle('active-tool', state.snap);
    if (selection?.kind === 'road') {
      const eid = selection.id, base = road(eid), widths = roadWidths(eid);
      const length = polylineLength(roadShape(eid));
      $('geo-road-info').textContent = `${laneCount(eid)} lane${laneCount(eid) > 1 ? 's' : ''} · ${length.toFixed(1)} m · ${base.spreadType === 'right' ? 'lanes to the right of the drawn line' : 'drawn line is the road centre'}${base.protected ? ' · measured road: cannot be split or removed' : ''}`;
      if (!$('geo-widths').contains(document.activeElement)) $('geo-widths').replaceChildren(...widths.map((w, i) => {
        const label = document.createElement('label'); label.className = 'inline'; label.textContent = `Lane ${i + 1} width (m)`;
        const input = document.createElement('input'); input.type = 'number'; input.min = 1.5; input.max = 12; input.step = .1; input.value = w;
        input.oninput = () => { const next = roadWidths(eid); next[i] = Math.max(1.5, Math.min(12, Number(input.value) || w)); draft.geometry[eid] = {...draft.geometry[eid], widths: next}; host.changed(`Lane ${i + 1} is now ${next[i]} m wide in the preview.`); };
        label.append(input); return label;
      }));
      $('geo-split-at').max = Math.max(2, length - 2).toFixed(2);
      $('geo-split-button').disabled = base.protected;
      $('geo-splits').replaceChildren(...(draft.splits[eid] || []).map(pos => {
        const button = document.createElement('button'); button.textContent = `Cut at ${pos.toFixed(2)} m · remove`;
        button.onclick = () => { draft.splits[eid] = draft.splits[eid].filter(v => v !== pos); if (!draft.splits[eid].length) delete draft.splits[eid]; host.changed('Cut removed.'); };
        return button;
      }));
      $('geo-removed').checked = isRemoved(eid); $('geo-removed').disabled = base.protected;
    }
    if (selection?.kind === 'drawing') {
      const drawing = draft.drawings[selection.id];
      if (drawing) {
        $('geo-draw-kind-edit').value = drawing.kind; $('geo-draw-color').value = drawing.color || ids().catalog.base.drawingKinds[drawing.kind]; $('geo-draw-label').value = drawing.label || '';
        $('geo-draw-effect').textContent = host.surfaceNote(selection.id);
        $('geo-draw-effect').classList.toggle('display-only', /^Display only/.test($('geo-draw-effect').textContent));
        $('geo-ped-label').hidden = drawing.kind !== 'crosswalk';
        if (document.activeElement !== $('geo-draw-peds')) $('geo-draw-peds').value = drawing.pedestrians ?? 120;
      }
    }
  }
  function bindPanel() {
    document.querySelectorAll('[data-ptool]').forEach(button => button.onclick = () => { host.activate(); setTool(button.dataset.ptool); });
    $('palette-snap').onclick = () => { state.snap = !state.snap; refreshPanel(); };
    $('palette-undo').onclick = $('geo-undo').onclick = () => host.undo();
    $('palette-redo').onclick = $('geo-redo').onclick = () => host.redo();
    $('geo-snap').onchange = event => { state.snap = event.target.checked; refreshPanel(); };
    $('geo-grid').onchange = event => { state.grid = Math.max(0, Math.min(10, Number(event.target.value) || 0)); refreshPanel(); };
    for (const action of ['densify', 'simplify', 'smooth', 'straighten']) $('geo-' + action).onclick = () => refine(action);
    $('geo-reset').onclick = resetSelection;
    $('geo-zoom').onclick = () => { const points = selectionPoints(); if (points.length) host.focus(points); else host.message('Select something first.', true); };
    const setCoordinate = () => {
      const points = selectionPoints(), single = state.vertices.size === 1 ? [...state.vertices][0] : state.selection?.kind === 'node' ? 0 : -1;
      if (single < 0 || !points[single]) return;
      const x = Number($('geo-x').value), y = Number($('geo-y').value);
      if (!Number.isFinite(x) || !Number.isFinite(y)) return;
      points[single] = [x, y];
      try { putPoints(points); host.changed('Point placed at the typed coordinates.'); } catch (error) { host.message(error.message, true); }
    };
    $('geo-x').onchange = $('geo-y').onchange = setCoordinate;
    $('geo-split-button').onclick = () => { if (state.selection?.kind === 'road') addSplit(state.selection.id, Number($('geo-split-at').value)); };
    $('geo-removed').onchange = event => {
      const {draft} = ids(); if (state.selection?.kind !== 'road') return;
      if (event.target.checked) draft.removed[state.selection.id] = true; else delete draft.removed[state.selection.id];
      host.changed(event.target.checked ? 'Road removal staged. Routes through it are re-planned when you apply.' : 'Road kept.');
    };
    const updateDrawing = () => {
      const {draft} = ids(); if (state.selection?.kind !== 'drawing') return;
      const drawing = draft.drawings[state.selection.id]; if (!drawing) return;
      const kind = $('geo-draw-kind-edit').value;
      if (kind !== drawing.kind) delete drawing.color; // a new type takes that type's colour
      drawing.kind = kind; drawing.label = $('geo-draw-label').value.slice(0, 60);
      if (kind === 'crosswalk') drawing.pedestrians = Math.max(0, Math.min(2000, Math.round(Number($('geo-draw-peds').value) || 0))); else delete drawing.pedestrians;
      host.changed('Surface updated. Checking what it does in SUMO…');
    };
    $('geo-draw-kind-edit').onchange = $('geo-draw-label').onchange = $('geo-draw-peds').onchange = updateDrawing;
    $('geo-draw-color').oninput = () => { const {draft} = ids(); const drawing = state.selection?.kind === 'drawing' && draft.drawings[state.selection.id]; if (drawing) { drawing.color = $('geo-draw-color').value; host.changed('Surface colour changed.'); } };
    $('geo-draw-delete').onclick = () => { const {draft} = ids(); if (state.selection?.kind === 'drawing') { delete draft.drawings[state.selection.id]; clearSelection(); host.changed('Surface deleted.'); } };
  }

  // ---- Rendering overlay ----
  function stroke(points, color, width, dash, close = false) {
    ctx.beginPath(); points.forEach((p, i) => { const q = xy(p); i ? ctx.lineTo(...q) : ctx.moveTo(...q); }); if (close) ctx.closePath();
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash || []); ctx.stroke(); ctx.setLineDash([]);
  }
  function handle(p, {fill = '#fff', stroke: edge = '#1d5f73', radius = 5, square = false} = {}) {
    const q = xy(p); ctx.fillStyle = fill; ctx.strokeStyle = edge; ctx.lineWidth = 1.5; ctx.beginPath();
    if (square) ctx.rect(q[0] - radius, q[1] - radius, radius * 2, radius * 2); else ctx.arc(q[0], q[1], radius, 0, Math.PI * 2);
    ctx.fill(); ctx.stroke();
  }
  function label(text, p, dy = -12) { const q = xy(p); ctx.font = '11px -apple-system,sans-serif'; ctx.textAlign = 'center'; ctx.lineWidth = 3; ctx.strokeStyle = '#ffffffd0'; ctx.strokeText(text, q[0], q[1] + dy); ctx.fillStyle = '#17444d'; ctx.fillText(text, q[0], q[1] + dy); }
  function draw() {
    const {draft, catalog, camera} = ids();
    if (!catalog?.base) return;
    const scale = camera.scale;
    if (state.snap && snapSettings(scale, state.grid).grid && scale * state.grid >= 12) {
      const [x0, y0] = world({clientX: canvas.getBoundingClientRect().left, clientY: canvas.getBoundingClientRect().top});
      const w = canvas.clientWidth / scale, h = canvas.clientHeight / scale;
      ctx.fillStyle = '#5b7a7a40';
      for (let x = Math.ceil(x0 / state.grid) * state.grid; x < x0 + w; x += state.grid) for (let y = Math.floor(y0 / state.grid) * state.grid; y > y0 - h; y -= state.grid) { const q = xy([x, y]); ctx.fillRect(q[0] - .5, q[1] - .5, 1, 1); }
    }
    // Centrelines and junction markers so everything is pickable.
    if (scale > 1.2) for (const eid of Object.keys(catalog.base.roads)) {
      const shape = roadShape(eid);
      stroke(shape, isRemoved(eid) ? '#c0392b' : '#ffffff55', isRemoved(eid) ? 2 : 1, isRemoved(eid) ? [6, 4] : [3, 5]);
      for (const pos of draft.splits?.[eid] || []) {
        const at = pointAlong(shape, pos), n = [-at.direction[1], at.direction[0]], half = roadWidths(eid).reduce((s, v) => s + v, 0) / 2 + 1.5;
        const centre = road(eid).spreadType === 'right' ? [at.point[0] + at.direction[1] * (half - 1.5), at.point[1] - at.direction[0] * (half - 1.5)] : at.point;
        stroke([[centre[0] - n[0] * half, centre[1] - n[1] * half], [centre[0] + n[0] * half, centre[1] + n[1] * half]], '#d8572a', 3);
        if (scale > 3) label(`cut ${pos.toFixed(1)} m`, centre, -10);
      }
    }
    if (scale > 1.2) for (const jid of Object.keys(catalog.base.junctions)) {
      const moved = !!draft.nodes?.[jid];
      handle(nodePosition(jid), {fill: moved ? '#f0b64a' : '#ffffffcc', stroke: moved ? '#9a5b00' : '#4f6f6c', radius: moved ? 5 : 3.5, square: true});
    }
    // Hover
    const hovered = state.hover?.target || state.hover?.road && {kind: 'road', id: state.hover.road.id};
    if (hovered && !(state.selection && hovered.kind === state.selection.kind && hovered.id === state.selection.id)) {
      if (hovered.kind === 'node') handle(nodePosition(hovered.id), {fill: '#19a5cc55', stroke: '#19a5cc', radius: 8, square: true});
      else stroke(selectionPoints(hovered), '#19a5cc', 2.5, null, closed(hovered));
    }
    // Selection with handles
    const selection = state.selection;
    if (selection) {
      const points = selectionPoints();
      if (selection.kind === 'road') {
        const b = borders(selection.id);
        stroke(b.left, '#e8b73b88', 1.5); stroke(b.right, '#e8b73b88', 1.5);
        stroke(points, '#c29222', 2.5);
      } else if (selection.kind !== 'node') {
        ctx.beginPath(); points.forEach((p, i) => { const q = xy(p); i ? ctx.lineTo(...q) : ctx.moveTo(...q); }); ctx.closePath(); ctx.fillStyle = '#efbb3e33'; ctx.fill();
        stroke(points, '#c29222', 2, null, true);
      }
      if (state.tool === 'add' || state.tool === 'select') {
        const count = closed(selection) ? points.length : points.length - 1;
        for (let i = 0; i < count && selection.kind !== 'node'; i++) {
          const a = points[i], c = points[(i + 1) % points.length], m = [(a[0] + c[0]) / 2, (a[1] + c[1]) / 2];
          if (Math.hypot(c[0] - a[0], c[1] - a[1]) * scale > 28) { const q = xy(m); ctx.strokeStyle = '#c2922299'; ctx.lineWidth = 1.2; ctx.beginPath(); ctx.moveTo(q[0] - 3, q[1]); ctx.lineTo(q[0] + 3, q[1]); ctx.moveTo(q[0], q[1] - 3); ctx.lineTo(q[0], q[1] + 3); ctx.stroke(); }
        }
      }
      points.forEach((p, i) => {
        const locked = selection.kind === 'road' && (i === 0 || i === points.length - 1);
        const active = state.vertices.has(i) || selection.kind === 'node';
        handle(p, {fill: active ? '#f6ce62' : state.hover?.vertex === i ? '#dff1f5' : '#fff', stroke: locked ? '#9a5b00' : '#1d5f73', radius: active ? 6.5 : 5.5, square: locked});
        if (scale > 4 && selection.kind !== 'node') label(String(i + 1), p);
      });
      if (selection.kind === 'node') {
        const node = catalog.base.junctions[selection.id];
        for (const eid of [...node.incoming, ...node.outgoing]) stroke(roadShape(eid), '#c2922266', 4);
      }
    }
    // Tool previews
    if (state.tool === 'split' && state.hover?.road) {
      const {id, hit} = state.hover.road, shape = roadShape(id), at = pointAlong(shape, hit.along), n = [-at.direction[1], at.direction[0]];
      const half = roadWidths(id).reduce((s, v) => s + v, 0) / 2 + 2;
      const centre = road(id).spreadType === 'right' ? [at.point[0] + at.direction[1] * (half - 2), at.point[1] - at.direction[0] * (half - 2)] : at.point;
      stroke([[centre[0] - n[0] * half, centre[1] - n[1] * half], [centre[0] + n[0] * half, centre[1] + n[1] * half]], road(id).protected ? '#b0413e' : '#d8572a', 3, [5, 4]);
      label(`${hit.along.toFixed(2)} m from start · ${(polylineLength(shape) - hit.along).toFixed(2)} m to end`, centre, -14);
    }
    if (state.tool === 'sidewalk' && state.hover?.road) {
      const walk = sidewalkFor(state.hover.road.id), strip = walk?.strip;
      if (strip) {
      ctx.beginPath(); strip.forEach((p, i) => { const q = xy(p); i ? ctx.lineTo(...q) : ctx.moveTo(...q); }); ctx.closePath(); ctx.fillStyle = '#d9d6cccc'; ctx.fill(); stroke(strip, '#8d8878', 1.5, null, true); }
      else label('One-way road: no left sidewalk in SUMO', pointAlong(roadShape(state.hover.road.id), state.hover.road.hit.along).point, -12);
    }
    if (state.drawing) {
      const points = state.pointer ? [...state.drawing, snapPreview()] : state.drawing;
      if (points.length > 2) { ctx.beginPath(); points.forEach((p, i) => { const q = xy(p); i ? ctx.lineTo(...q) : ctx.moveTo(...q); }); ctx.closePath(); ctx.fillStyle = '#9ec7d655'; ctx.fill(); }
      stroke(points, '#1d5f73', 2, [6, 4]);
      state.drawing.forEach((p, i) => handle(p, {radius: 4.5}));
      if (points.length > 1) label(`${polylineLength(points).toFixed(1)} m outline`, points.at(-1), 14);
    }
    if (state.measure.length) {
      const points = state.measure.length === 2 ? state.measure : [state.measure[0], snapPreview()];
      stroke(points, '#8b2f8f', 2, [4, 4]); points.forEach(p => handle(p, {radius: 4, stroke: '#8b2f8f'}));
      const d = Math.hypot(points[1][0] - points[0][0], points[1][1] - points[0][1]);
      label(`${d.toFixed(2)} m`, [(points[0][0] + points[1][0]) / 2, (points[0][1] + points[1][1]) / 2]);
    }
    if (state.guide) {
      const q = xy(state.guide.point); ctx.strokeStyle = '#d946ef'; ctx.lineWidth = 1.5; ctx.beginPath();
      ctx.moveTo(q[0] - 9, q[1]); ctx.lineTo(q[0] + 9, q[1]); ctx.moveTo(q[0], q[1] - 9); ctx.lineTo(q[0], q[1] + 9); ctx.stroke();
      if (state.guide.kind === 'line' && state.guide.target) { const t = state.guide.target, i = state.guide.segment; stroke([t.points[i], t.points[(i + 1) % t.points.length]], '#d946ef99', 2); }
      if (state.guide.kind !== 'grid') label(`snap · ${state.guide.target?.kind || ''}`, state.guide.point, 16);
    }
  }
  function snapPreview() { return state.pointer ? (state.snap && !state.altKey ? snapPoint(state.pointer, {targets: snapTargets(), ...snapSettings(host.getCamera().scale, state.grid)}).point : state.pointer) : [0, 0]; }

  return {
    state, TOOLS, setTool, draw, pointerDown, pointerMove, pointerUp, keydown, cancel, refreshPanel, bindPanel, select, clearSelection,
    previewLanes, touchedRoads, isRemoved, junctionShape, nodeDelta, nodePosition, roadShape,
    reset() { state.selection = null; state.vertices = new Set(); state.drag = null; state.drawing = null; state.measure = []; state.hover = null; state.guide = null; refreshPanel(); },
    // Keep the selection after undo/redo or a tab switch, but drop anything mid-gesture.
    resync() {
      state.drag = null; state.drawing = null; state.hover = null; state.guide = null;
      if (state.selection && !selectionPoints().length) state.selection = null;
      state.vertices = new Set([...state.vertices].filter(i => i < selectionPoints().length));
      refreshPanel();
    }
  };
}
