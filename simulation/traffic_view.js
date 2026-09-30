const $ = id => document.getElementById(id);
const canvas = $('map'), ctx = canvas.getContext('2d');
let net = null, state = null, previous = null, received = 0, previousReceived = 0;
let camera = {x: 0, y: 0, scale: 1}, width = 0, height = 0, drag = null;
let editing = false, tool = 'roads', pick = null, catalog = null, draft = null, dirty = false;
let selectedRoad = null, selectedSignal = null, selectedShape = null, selectedRoute = null;
let routeSpec = null, routePreview = [], previewRequest = 0, busy = false;

async function api(url, data) {
  const response = await fetch(url, data === undefined ? {} : {
    method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(data)
  });
  const result = await response.json();
  if (!response.ok || result.error) throw new Error(result.error || 'Request failed');
  return result;
}
const control = (action, value) => api('/control', {action, value});
function message(text, error = false) {
  $('editor-message').textContent = text;
  $('editor-message').classList.toggle('error', error);
}
function changed(text = 'Change staged. Save & restart to apply it to SUMO.') {
  if (state?.running) control('pause').catch(error => message(error.message, true));
  dirty = true; $('editor-toggle').textContent = 'Edit map •'; message(text);
}
function mode(value, text) {
  pick = value; canvas.classList.toggle('pick-mode', !!value); if (text) message(text);
}
function resize() {
  width = canvas.clientWidth; height = canvas.clientHeight;
  const d = devicePixelRatio || 1; canvas.width = width * d; canvas.height = height * d;
  ctx.setTransform(d, 0, 0, d, 0, 0);
}
function fit(all = false) {
  if (!net) return;
  const [a, b, c, d] = all ? net.bounds : net.core_bounds;
  camera = {x: (a + c) / 2, y: (b + d) / 2, scale: Math.min(width / (c - a), height / (d - b)) * .9};
}
function xy(p) { return [(p[0] - camera.x) * camera.scale + width / 2, (camera.y - p[1]) * camera.scale + height / 2]; }
function world(e) { const r = canvas.getBoundingClientRect(); return [camera.x + (e.clientX - r.left - width / 2) / camera.scale, camera.y - (e.clientY - r.top - height / 2) / camera.scale]; }
function path(points, close = false) {
  ctx.beginPath(); points.forEach((p, i) => { const q = xy(p); i ? ctx.lineTo(...q) : ctx.moveTo(...q); });
  if (close) ctx.closePath();
}
function offsetLine(points, d) {
  return points.map((p, i) => { const a = points[Math.max(0, i - 1)], b = points[Math.min(points.length - 1, i + 1)];
    const dx = b[0] - a[0], dy = b[1] - a[1], len = Math.hypot(dx, dy) || 1;
    return [p[0] + dy / len * d, p[1] - dx / len * d]; });
}
function shapePoints(shape) {
  if (!editing || !draft) return shape.shape;
  return shape.kind === 'junction' ? draft.surfaces[shape.id] || shape.shape : draft.artifacts[shape.id]?.shape || shape.shape;
}
function selectedPoints() { return selectedShape ? shapePoints(selectedShape) : []; }
function putShape(points) {
  if (selectedShape.kind === 'junction') draft.surfaces[selectedShape.id] = points;
  else draft.artifacts[selectedShape.id] = {...draft.artifacts[selectedShape.id], shape: points};
}
function draw() {
  ctx.clearRect(0, 0, width, height); ctx.fillStyle = '#e8ede2'; ctx.fillRect(0, 0, width, height);
  if (!net) { requestAnimationFrame(draw); return; }
  for (const p of net.polygons) {
    const edit = editing && draft ? draft.artifacts[p.id] || {} : {};
    const hidden = edit.hidden ?? p.hidden;
    if (hidden && !(editing && tool === 'shapes' && $('shape-kind').value === 'artifact')) continue;
    path(edit.shape || p.shape, true);
    ctx.fillStyle = edit.color || p.color || (/water/.test(p.type) ? '#c5dae0' : /park|grass|wood|garden|forest/.test(p.type) ? '#cbdcc0' : /building/.test(p.type) ? '#d5d7cb' : '#e0e3d7');
    if (hidden) { ctx.strokeStyle = '#b69f7d'; ctx.lineWidth = 1; ctx.stroke(); } else ctx.fill();
  }
  for (const j of net.junctions) {
    path(editing && draft?.surfaces[j.id] || j.shape, true); ctx.fillStyle = '#626c68'; ctx.fill();
  }
  ctx.lineCap = 'butt'; ctx.lineJoin = 'round';
  for (const lane of net.lanes) {
    if (lane.internal) continue;
    path(lane.shape); ctx.lineWidth = Math.max(1, lane.width * camera.scale);
    ctx.strokeStyle = lane.closed ? '#aa9386' : lane.route ? '#697f7e' : '#626c68'; ctx.stroke();
  }
  if (camera.scale > .7) {
    for (const lane of net.lanes) {
      if (lane.internal || lane.index === 0) continue;
      path(offsetLine(lane.shape, lane.width / 2)); ctx.lineWidth = Math.max(.5, .12 * camera.scale);
      ctx.strokeStyle = '#d3dbd1'; ctx.setLineDash([4 * camera.scale, 7 * camera.scale]); ctx.stroke();
    }
    ctx.setLineDash([]);
  }
  if (editing) {
    const highlight = new Set(tool === 'routes' ? routePreview : selectedRoad ? [selectedRoad.edge] : []);
    for (const lane of net.lanes) {
      if (lane.internal || !highlight.has(lane.edge)) continue;
      path(lane.shape); ctx.lineWidth = Math.max(3, lane.width * camera.scale); ctx.strokeStyle = '#e8b73b99'; ctx.stroke();
    }
    if (tool === 'signals') {
      for (const node of net.junctions) {
        if (!node.editable) continue;
        const p = xy([node.x, node.y]);
        ctx.strokeStyle = node.id === selectedSignal?.id ? '#e4a917' : '#35684690';
        ctx.lineWidth = node.id === selectedSignal?.id ? 3 : 1;
        ctx.beginPath(); ctx.arc(...p, node.id === selectedSignal?.id ? 10 : 5, 0, Math.PI * 2); ctx.stroke();
      }
    }
    if (selectedShape && tool === 'shapes') {
      const points = selectedPoints(); path(points, true); ctx.fillStyle = '#efbb3e44'; ctx.fill();
      ctx.strokeStyle = '#c29222'; ctx.lineWidth = 2; ctx.stroke();
      for (const p of points) { const q = xy(p); ctx.fillStyle = '#fff'; ctx.fillRect(q[0] - 4, q[1] - 4, 8, 8); ctx.strokeRect(q[0] - 4, q[1] - 4, 8, 8); }
    }
  }
  if (state) {
    const seen = new Set();
    for (const signal of net.signals) {
      if (editing && draft?.signals[signal.junction] === false) continue;
      const delta = editing && draft?.markers[signal.junction];
      const savedDelta = catalog?.edits.markers[signal.junction] || [0, 0];
      const point = delta ? [signal.point[0] + delta[0] - savedDelta[0], signal.point[1] + delta[1] - savedDelta[1]] : signal.point;
      const q = xy(point), key = q.map(Math.round).join(','); if (seen.has(key)) continue; seen.add(key);
      const color = (state.tls[signal.tls] || '')[signal.index];
      ctx.fillStyle = /[gG]/.test(color) ? '#57c28c' : /[yY]/.test(color) ? '#e6b552' : '#e47564';
      ctx.beginPath(); ctx.arc(q[0], q[1], Math.min(5, Math.max(2, camera.scale)), 0, Math.PI * 2); ctx.fill();
    }
    const old = new Map((previous?.vehicles || []).map(v => [v[0], v]));
    const fraction = state.running ? Math.min(1, (performance.now() - received) / Math.max(60, received - previousReceived)) : 1;
    for (const v of state.vehicles) {
      const ov = old.get(v[0]) || v, p = [ov[1] + (v[1] - ov[1]) * fraction, ov[2] + (v[2] - ov[2]) * fraction], q = xy(p);
      const factor = Math.max(camera.scale, 1.2), length = v[5] * factor, w = Math.max(3, v[6] * factor);
      ctx.save(); ctx.translate(...q); ctx.rotate(v[3] * Math.PI / 180);
      ctx.fillStyle = v[4] < .1 ? '#ce7243' : '#2686bb'; ctx.strokeStyle = '#173b4f'; ctx.lineWidth = .55;
      ctx.beginPath(); ctx.roundRect(-w / 2, -length / 2, w, length, Math.min(1.8, w / 3)); ctx.fill(); ctx.stroke();
      ctx.fillStyle = '#cceaf3'; ctx.fillRect(-w * .32, -length * .3, w * .64, length * .15); ctx.restore();
    }
  }
  ctx.fillStyle = '#5b6962'; ctx.font = '11px -apple-system,sans-serif'; ctx.textAlign = 'left';
  ctx.fillText('© OpenStreetMap contributors · drag to pan · scroll to zoom', 20, height - 8);
  requestAnimationFrame(draw);
}

function segmentDistance(p, a, b) {
  const dx = b[0] - a[0], dy = b[1] - a[1], length = dx * dx + dy * dy;
  const t = length ? Math.max(0, Math.min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length)) : 0;
  return Math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy);
}
function nearestRoad(point) {
  let best = null, distance = 20 / camera.scale;
  for (const lane of net.lanes) {
    if (lane.internal) continue;
    for (let i = 1; i < lane.shape.length; i++) { const d = segmentDistance(point, lane.shape[i - 1], lane.shape[i]); if (d < distance) { best = lane; distance = d; } }
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
function nearestJunction(p) {
  const atLight = net.signals.find(s => Math.hypot(p[0] - s.point[0], p[1] - s.point[1]) * camera.scale < 12);
  if (atLight) return net.junctions.find(j => j.id === atLight.junction);
  return net.junctions.filter(j => j.editable).sort((a, b) => Math.hypot(a.x - p[0], a.y - p[1]) - Math.hypot(b.x - p[0], b.y - p[1]))
    .find(j => Math.hypot(j.x - p[0], j.y - p[1]) * camera.scale < 30 || inside(p, j.shape));
}
function setTool(value) {
  tool = value; mode(null);
  document.querySelectorAll('[data-tool]').forEach(button => button.classList.toggle('selected', button.dataset.tool === value));
  for (const name of ['roads', 'routes', 'signals', 'shapes']) $('panel-' + name).hidden = name !== value;
}
function option(value, text) { const o = document.createElement('option'); o.value = value; o.textContent = text; return o; }
function populateRoutes() {
  const query = $('route-search').value.toLowerCase();
  const routes = catalog.routes.filter(r => r.label.toLowerCase().includes(query)).slice(0, 40);
  for (const id of Object.keys(draft.routes).filter(id => id.startsWith('custom_'))) routes.unshift({id, label: id + ' · added traffic'});
  if (selectedRoute && !routes.some(r => r.id === selectedRoute)) { const r = catalog.routes.find(r => r.id === selectedRoute); if (r) routes.unshift(r); }
  $('route-select').replaceChildren(...routes.map(r => option(r.id, r.label)));
  if (selectedRoute) $('route-select').value = selectedRoute;
}
async function previewRoute() {
  if (!routeSpec) return;
  const request = ++previewRequest;
  try {
    const result = await api('/route-preview', {id: selectedRoute, route: routeSpec});
    if (request !== previewRequest) return;
    routePreview = result.edges;
    draft.routes[selectedRoute] = structuredClone(routeSpec);
    changed();
    $('route-details').textContent = `${result.length.toLocaleString()} m · ${routeSpec.via.length} waypoint(s)`;
    message('Route preview is highlighted. Keep the change, then save to apply.');
  } catch (error) { if (request === previewRequest) { routePreview = []; message(error.message, true); } }
}
function selectRoute(id) {
  previewRequest++;
  selectedRoute = id; const base = catalog.routes.find(r => r.id === id);
  routeSpec = structuredClone(draft.routes[id] || {from: base?.from, to: base?.to, via: []});
  populateRoutes(); $('route-from').value = routeSpec.from; $('route-to').value = routeSpec.to;
  $('route-rate-label').hidden = !id.startsWith('custom_'); $('route-rate').value = routeSpec.rate || 60;
  routePreview = net.paths[id] || base?.edges || [];
  $('route-details').textContent = `${base?.count || Math.round((routeSpec.rate || 60) * 2)} planned vehicles on this route`;
}
function selectRoad(lane) {
  selectedRoad = lane; const spec = draft.roads[lane.edge] || {lanes: lane.laneCount, speed: lane.speed, closed: lane.closed};
  $('road-selected').textContent = (lane.name || 'Road') + ' · ' + lane.edge;
  $('road-lanes').value = spec.lanes; $('road-speed').value = Math.round(spec.speed / .44704); $('road-closed').checked = spec.closed;
}
function selectSignal(node) {
  selectedSignal = node; $('signal-selected').textContent = node.id;
  $('signal-enabled').checked = draft.signals[node.id] ?? node.controlled;
  $('signal-cycle').value = draft.timings[node.id]?.cycle || 90; $('signal-offset').value = draft.timings[node.id]?.offset || 0;
}
function selectShape(p, kind) {
  selectedShape = {...p, kind}; $('shape-selected').textContent = (kind === 'junction' ? 'Paved junction · ' : p.type + ' · ') + p.id;
  $('artifact-fields').hidden = kind !== 'artifact';
  if (kind === 'artifact') { $('artifact-hidden').checked = draft.artifacts[p.id]?.hidden ?? p.hidden; $('artifact-color').value = draft.artifacts[p.id]?.color || p.color || '#d5d7cb'; }
}
function mapClick(p) {
  if (!editing) return;
  if (pick === 'waypoint') {
    const lane = nearestRoad(p); if (!lane) return message('Click directly on a road.', true);
    routeSpec.via.push(lane.edge); mode(null); previewRoute(); return;
  }
  if (pick === 'move-control') {
    const node = nearestJunction(p); if (!node) return message('Choose another outlined junction.', true);
    draft.signals[selectedSignal.id] = false; draft.signals[node.id] = true;
    selectSignal(node); mode(null); changed('Actual traffic control moved in the draft. Save to rebuild the signal.'); return;
  }
  if (pick === 'move-marker') {
    const heads = net.signals.filter(s => s.junction === selectedSignal.id);
    const saved = catalog.edits.markers[selectedSignal.id] || [0, 0];
    const origin = heads.length ? [heads.reduce((s, h) => s + h.point[0], 0) / heads.length - saved[0], heads.reduce((s, h) => s + h.point[1], 0) / heads.length - saved[1]] : [selectedSignal.x, selectedSignal.y];
    draft.markers[selectedSignal.id] = [p[0] - origin[0], p[1] - origin[1]];
    mode(null); changed('Marker position staged. Its stop control stays at this junction.'); return;
  }
  if (pick === 'add-corner' || pick === 'remove-corner') {
    const points = structuredClone(selectedPoints());
    if (pick === 'add-corner') { let best = 0, distance = Infinity; for (let i = 0; i < points.length; i++) { const d = segmentDistance(p, points[i], points[(i + 1) % points.length]); if (d < distance) { best = i; distance = d; } } points.splice(best + 1, 0, p); }
    else { if (points.length <= 3) return message('A shape needs at least three corners.', true); let best = 0; for (let i = 1; i < points.length; i++) if (Math.hypot(points[i][0] - p[0], points[i][1] - p[1]) < Math.hypot(points[best][0] - p[0], points[best][1] - p[1])) best = i; points.splice(best, 1); }
    putShape(points); mode(null); changed(); return;
  }
  if (tool === 'roads') { const lane = nearestRoad(p); if (lane) selectRoad(lane); }
  if (tool === 'signals') { const node = nearestJunction(p); if (node) selectSignal(node); }
  if (tool === 'routes') {
    const vehicle = (state?.vehicles || []).find(v => Math.hypot(v[1] - p[0], v[2] - p[1]) * camera.scale < 12);
    if (vehicle && vehicle[9]) selectRoute(vehicle[9]);
  }
  if (tool === 'shapes') {
    const kind = $('shape-kind').value;
    const shapes = kind === 'junction' ? net.junctions.filter(j => j.editable) : net.polygons;
    const hit = [...shapes].reverse().find(s => inside(p, shapePoints({...s, kind})));
    if (hit) selectShape(hit, kind);
  }
}
canvas.onpointerdown = event => {
  const p = world(event); canvas.setPointerCapture(event.pointerId);
  if (editing && tool === 'shapes' && selectedShape) {
    const points = structuredClone(selectedPoints());
    const vertex = points.findIndex(q => Math.hypot(q[0] - p[0], q[1] - p[1]) * camera.scale < 9);
    if (pick === 'move-shape' || (vertex >= 0 && !pick)) { drag = {kind: 'shape', points, p, vertex: pick === 'move-shape' ? -1 : vertex}; return; }
  }
  drag = {kind: 'pan', x: event.clientX, y: event.clientY, cx: camera.x, cy: camera.y, moved: false};
};
canvas.onpointermove = event => {
  if (!drag) return;
  if (drag.kind === 'shape') {
    const p = world(event), points = structuredClone(drag.points);
    if (drag.vertex >= 0) points[drag.vertex] = p;
    else for (const q of points) { q[0] += p[0] - drag.p[0]; q[1] += p[1] - drag.p[1]; }
    putShape(points); return;
  }
  if (Math.hypot(event.clientX - drag.x, event.clientY - drag.y) > 4) drag.moved = true;
  if (!pick) { camera.x = drag.cx - (event.clientX - drag.x) / camera.scale; camera.y = drag.cy + (event.clientY - drag.y) / camera.scale; }
};
canvas.onpointerup = event => {
  if (drag?.kind === 'shape') { mode(null); changed(); }
  else if (drag && !drag.moved) mapClick(world(event));
  drag = null;
};
canvas.onwheel = event => { event.preventDefault(); camera.scale = Math.max(.1, Math.min(20, camera.scale * Math.exp(-event.deltaY * .001))); };

async function openEditor() {
  try {
    await control('pause');
    if (!dirty) { catalog = await api('/editor'); draft = structuredClone(catalog.edits); }
    for (const key of ['routes', 'signals', 'markers', 'roads', 'surfaces', 'artifacts', 'timings']) draft[key] ||= {};
    $('route-from').replaceChildren(...catalog.origins.map(e => option(e.id, e.label)));
    $('route-to').replaceChildren(...catalog.exits.map(e => option(e.id, e.label)));
    if (selectedRoute && !catalog.routes.some(r => r.id === selectedRoute) && !draft.routes[selectedRoute]) selectedRoute = null;
    populateRoutes(); if (!selectedRoute && catalog.routes.length) selectRoute(catalog.routes[0].id);
    editing = true; $('editor').hidden = false; document.body.classList.add('editing'); setTool(tool);
    message(dirty ? 'You have staged changes. Save to apply them.' : 'Simulation paused for editing. Pick a tool, then select something on the map.');
  } catch (error) { $('error').textContent = error.message; $('error').style.display = 'inline'; }
}
function closeEditor() { editing = false; $('editor').hidden = true; document.body.classList.remove('editing'); mode(null); }
$('editor-toggle').onclick = () => editing ? closeEditor() : openEditor(); $('editor-close').onclick = closeEditor;
document.querySelectorAll('[data-tool]').forEach(button => button.onclick = () => setTool(button.dataset.tool));
$('route-search').oninput = populateRoutes; $('route-select').onchange = event => selectRoute(event.target.value);
$('route-from').onchange = event => { routeSpec.from = event.target.value; previewRoute(); };
$('route-to').onchange = event => { routeSpec.to = event.target.value; previewRoute(); };
$('route-waypoint').onclick = () => { if (routeSpec) mode('waypoint', 'Click a road that this route should pass through.'); };
$('route-clear').onclick = () => { if (routeSpec) { routeSpec.via = []; previewRoute(); } };
$('route-new').onclick = () => {
  const base = routeSpec || catalog.routes[0];
  const id = 'custom_' + Date.now(); draft.routes[id] = {from: base.from, to: base.to, via: [...(base.via || [])], rate: 60};
  selectRoute(id); changed('Choose the new route’s entrance, exit, and hourly traffic.');
};
$('route-stage').onclick = async () => {
  try { if (!routeSpec) return; if (selectedRoute.startsWith('custom_')) routeSpec.rate = Number($('route-rate').value);
    const result = await api('/route-preview', {id: selectedRoute, route: routeSpec}); routePreview = result.edges;
    draft.routes[selectedRoute] = structuredClone(routeSpec); changed();
  } catch (error) { message(error.message, true); }
};
$('route-reset').onclick = () => { if (!selectedRoute) return; delete draft.routes[selectedRoute]; const id = selectedRoute.startsWith('custom_') ? catalog.routes[0].id : selectedRoute; selectRoute(id); changed('Route reset staged.'); };
$('road-stage').onclick = () => { if (!selectedRoad) return message('Select a road first.', true); draft.roads[selectedRoad.edge] = {lanes: Number($('road-lanes').value), speed: Number($('road-speed').value) * .44704, closed: $('road-closed').checked}; changed(); };
for (const id of ['road-lanes', 'road-speed', 'road-closed']) $(id).onchange = () => { if (selectedRoad) $('road-stage').onclick(); };
$('route-rate').onchange = () => { if (selectedRoute?.startsWith('custom_')) { routeSpec.rate = Number($('route-rate').value); draft.routes[selectedRoute] = structuredClone(routeSpec); changed(); } };
$('road-reset').onclick = () => { if (selectedRoad) { delete draft.roads[selectedRoad.edge]; changed('Road reset staged. Save to restore its base settings.'); } };
$('signal-enabled').onchange = event => { if (!selectedSignal) return message('Select a junction first.', true); draft.signals[selectedSignal.id] = event.target.checked; changed(); };
$('signal-move').onclick = () => { if (selectedSignal) mode('move-control', 'Click the junction where the actual traffic light should operate.'); else message('Select a light first.', true); };
$('signal-marker').onclick = () => { if (selectedSignal) mode('move-marker', 'Click the desired marker position. This does not move the stop control.'); else message('Select a light first.', true); };
$('signal-marker-reset').onclick = () => { if (selectedSignal) { delete draft.markers[selectedSignal.id]; changed(); } };
$('signal-timing').onclick = () => { if (!selectedSignal) return message('Select a junction first.', true); draft.timings[selectedSignal.id] = {cycle: Number($('signal-cycle').value), offset: Number($('signal-offset').value)}; changed(); };
$('shape-kind').onchange = () => { selectedShape = null; mode(null); $('shape-selected').textContent = 'No shape selected'; };
$('shape-move').onclick = () => { if (selectedShape) mode('move-shape', 'Drag the selected shape to move it.'); };
$('shape-add').onclick = () => { if (selectedShape) mode('add-corner', 'Click where the new corner should go.'); };
$('shape-remove').onclick = () => { if (selectedShape) mode('remove-corner', 'Click a corner to remove it.'); };
$('shape-reset').onclick = () => { if (!selectedShape) return; delete (selectedShape.kind === 'junction' ? draft.surfaces : draft.artifacts)[selectedShape.id]; changed('Shape reset staged. Save to restore its base shape.'); };
$('artifact-hidden').onchange = event => { if (selectedShape?.kind === 'artifact') { draft.artifacts[selectedShape.id] = {...draft.artifacts[selectedShape.id], hidden: event.target.checked}; changed(); } };
$('artifact-color').oninput = event => { if (selectedShape?.kind === 'artifact') { draft.artifacts[selectedShape.id] = {...draft.artifacts[selectedShape.id], color: event.target.value}; changed(); } };
async function waitForEdits() {
  try {
    const job = await api('/editor-job');
    if (job.status === 'failed') throw new Error(job.error || 'Could not apply edits');
    if (job.status === 'saved') {
      catalog = await api('/editor'); draft = structuredClone(catalog.edits); dirty = false;
      $('editor-toggle').textContent = 'Edit map'; message('Saved. The simulation is running with your edits.'); busy = false; $('editor-apply').disabled = false; return;
    }
    setTimeout(waitForEdits, 400);
  } catch (error) { busy = false; $('editor-apply').disabled = false; message(error.message, true); }
}
$('editor-apply').onclick = async () => {
  if (busy) return;
  try { busy = true; $('editor-apply').disabled = true; await api('/apply-edits', draft); message('Checking routes and rebuilding the edited simulation…'); waitForEdits(); }
  catch (error) { busy = false; $('editor-apply').disabled = false; message(error.message, true); }
};
$('open-netedit').onclick = async () => { try { await api('/netedit', {}); message('NetEdit opened. Save user-network.net.xml there, then click Reload saved network.'); } catch (error) { message(error.message, true); } };
$('reload-network').onclick = async () => { try { await control('reload-network'); message('Loading the saved NetEdit network…'); setTimeout(waitForEdits, 600); } catch (error) { message(error.message, true); } };
$('toggle').onclick = () => control(state?.running ? 'pause' : 'play'); $('restart').onclick = () => control('restart');
$('speed').onchange = event => control('speed', Number(event.target.value)); $('fit').onclick = () => fit(); $('all-roads').onclick = () => fit(true);
$('zoom-in').onclick = () => camera.scale *= 1.25; $('zoom-out').onclick = () => camera.scale /= 1.25;

async function poll() {
  try {
    const next = await api('/state');
    if (next.ready) {
      if (!net || net.revision !== next.revision) { const first = !net; net = await api('/network'); if (first) fit(); previous = null; }
      else previous = state && next.time >= state.time ? state : null;
      state = next; previousReceived = received; received = performance.now();
      $('clock').textContent = [Math.floor(next.time / 3600), Math.floor(next.time / 60) % 60, Math.floor(next.time) % 60].map(v => String(v).padStart(2, '0')).join(':');
      $('status').textContent = next.updating ? 'Applying scenario edits…' : !next.running ? 'Paused' : next.warming ? 'Filling the study area…' : 'SUMO running · ' + next.speed + '× speed';
      $('dot').style.background = next.running ? '#3b9565' : '#c19a4d'; $('toggle').textContent = next.running ? 'Pause' : 'Play';
      if (document.activeElement !== $('speed')) $('speed').value = next.speed;
      $('active').textContent = next.vehicles.length;
      for (const key of ['waiting', 'entered', 'arrived', 'observed']) $(key).textContent = next[key].toLocaleString();
      $('error').textContent = ''; $('error').style.display = 'none';
    }
  } catch (error) { $('status').textContent = 'Reconnecting to SUMO…'; $('error').style.display = 'inline'; $('error').textContent = ' ' + error.message; }
  setTimeout(poll, 100);
}
window.addEventListener('resize', resize); resize(); requestAnimationFrame(draw); poll();
