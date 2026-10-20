import {offsetLine, cutPolygon, polygonArea, lineSide} from './editor_geometry.mjs';
import {createPrecisionEditor} from './map_editor.js';
const DRAFT_KEYS = ['routes', 'signals', 'markers', 'roads', 'surfaces', 'artifacts', 'timings', 'controls', 'nodes', 'geometry', 'splits', 'removed', 'drawings'];
const $ = id => document.getElementById(id);
const canvas = $('map'), ctx = canvas.getContext('2d');
let net = null, state = null, previous = null, received = 0, previousReceived = 0;
let camera = {x: 0, y: 0, scale: 1}, width = 0, height = 0, drag = null;
let editing = false, tool = 'geometry', pick = null, catalog = null, draft = null, dirty = false;
let selectedRoad = null, selectedSignal = null, selectedShape = null, selectedRoute = null;
let routeSpec = null, routePreview = [], previewRequest = 0, busy = false;
let hover = null, pointer = null, pendingShapeAction = null, cutLine = [], shapeUndo = null;
let lanePreviewCache = null, cutPreview = null;
let requestedTraffic = null;
let requestedController = null;
let engineUnavailable = false, lastEngineTime = null, lastEngineAdvance = performance.now();
let committed = null, undoStack = [], redoStack = [], precision = null;

async function api(url, data) {
  const response = await fetch(url, data === undefined ? {} : {
    method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(data)
  });
  const result = await response.json();
  if (!response.ok || (result.error && url !== '/state')) throw new Error(result.error || 'Request failed');
  return result;
}
const control = (action, value) => api('/control', {action, value});
function message(text, error = false) {
  $('editor-message').textContent = text;
  $('editor-message').classList.toggle('error', error);
  $('map-instruction').textContent = text;
  $('map-instruction').classList.toggle('error', error);
  $('map-instruction').hidden = !editing;
}
function changed(text = 'Change staged. Save & restart to apply it to SUMO.') {
  if (state?.running) control('pause').catch(error => message(error.message, true));
  if (committed) { undoStack.push(committed); if (undoStack.length > 100) undoStack.shift(); redoStack = []; }
  committed = structuredClone(draft);
  dirty = true; lanePreviewCache = null; $('editor-toggle').textContent = 'Edit map •'; message(text); updateTools(); precision?.refreshPanel();
  refreshSurfaceNotes();
}
function restoreDraft(next) {
  draft = next; committed = structuredClone(draft); lanePreviewCache = null;
  dirty = JSON.stringify(draft) !== JSON.stringify(catalog.edits); $('editor-toggle').textContent = dirty ? 'Edit map •' : 'Edit map';
  precision?.resync(); refreshSurfaceNotes(); if (selectedRoad) selectRoad(selectedRoad); if (selectedSignal) selectSignal(selectedSignal); updateTools();
}
function undo() { if (!undoStack.length) return message('Nothing to undo.'); redoStack.push(structuredClone(draft)); restoreDraft(undoStack.pop()); message(`Undone · ${undoStack.length} more step${undoStack.length === 1 ? '' : 's'} available.`); }
function redo() { if (!redoStack.length) return message('Nothing to redo.'); undoStack.push(structuredClone(draft)); restoreDraft(redoStack.pop()); message('Redone.'); }
function focusPoints(points) {
  if (!points.length) return;
  const xs = points.map(p => p[0]), ys = points.map(p => p[1]);
  const panelWidth = $('editor').getBoundingClientRect().width;
  const usable = Math.max(220, width - panelWidth - 60);
  camera.scale = Math.min(12, usable / (Math.max(...xs) - Math.min(...xs) + 55), height / (Math.max(...ys) - Math.min(...ys) + 55));
  camera.x = (Math.min(...xs) + Math.max(...xs)) / 2 + (width - usable) / (2 * camera.scale);
  camera.y = (Math.min(...ys) + Math.max(...ys)) / 2;
}
function controlType(node) {
  if (!editing || !draft) return node.controlType;
  return draft.controls[node.id] || (node.id in draft.signals ? (draft.signals[node.id] ? 'traffic_light' : 'priority') : catalog?.base?.junctions[node.id]?.controlType || node.controlType);
}
function mode(value, text) {
  pick = value; canvas.classList.toggle('pick-mode', !!value); if (text) message(text); updateTools();
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
function displayedLanes() {
  if (!editing || !catalog?.base || !precision) return net.lanes;
  if (lanePreviewCache) return lanePreviewCache;
  // Roads whose lanes differ from the loaded network: lane-count edits, reshaped roads, moved junctions, removals.
  const ids = new Set([...Object.keys(net.edits?.roads || {}), ...Object.keys(net.edits?.geometry || {}), ...Object.keys(net.edits?.nodes || {}),
                       ...Object.keys(draft.roads), ...precision.touchedRoads()]);
  const lanes = net.lanes.filter(l => !ids.has(l.edge));
  for (const id of ids) {
    const base = catalog.base.roads[id];
    // Copy display fields from a vehicle lane: an applied sidewalk is lane 0 and must not tag the preview as pedestrian.
    const original = net.lanes.find(l => l.edge === id && !l.pedestrian) || net.lanes.find(l => l.edge === id);
    if (!base || !original || precision.isRemoved(id)) continue;
    const spec = draft.roads[id] || {lanes: base.laneCount, speed: base.speed, closed: base.closed};
    for (const lane of precision.previewLanes(id)) lanes.push({...original, ...lane, id: id + '_' + lane.index, laneCount: spec.lanes, speed: spec.speed, closed: spec.closed, preview: true, pedestrian: null, internal: false, curb: lane.index === 0});
  }
  lanePreviewCache = lanes;
  return lanes;
}
function baselineShape(shape) {
  return (shape.kind === 'junction' ? catalog?.base?.junctions[shape.id] : catalog?.base?.artifacts[shape.id])?.shape || shape.shape;
}
function shapePoints(shape) {
  if (!editing || !draft) return shape.shape;
  if (shape.kind === 'junction' && precision && catalog?.base?.junctions[shape.id]) return precision.junctionShape(shape.id);
  const saved = shape.kind === 'junction' ? net.edits?.surfaces[shape.id] : net.edits?.artifacts[shape.id]?.shape;
  return shape.kind === 'junction' ? draft.surfaces[shape.id] || (saved ? baselineShape(shape) : shape.shape)
    : draft.artifacts[shape.id]?.shape || (saved ? baselineShape(shape) : shape.shape);
}
function selectedPoints() { return selectedShape ? shapePoints(selectedShape) : []; }
function putShape(points) {
  if (selectedShape.kind === 'junction') draft.surfaces[selectedShape.id] = points;
  else draft.artifacts[selectedShape.id] = {...draft.artifacts[selectedShape.id], shape: points};
}
function rememberShape() { if(selectedShape){const values=selectedShape.kind==='junction'?draft.surfaces:draft.artifacts;shapeUndo={value:structuredClone(values[selectedShape.id]??null)};}updateTools(); }
function updateTools() {
  if (!$('editor-apply')) return;
  $('editor-apply').disabled = busy || !dirty || engineUnavailable;
  $('editor-apply').textContent = busy ? 'Applying changes…' : dirty ? 'Apply changes & restart' : 'No changes to apply';
  $('cancel-tool').hidden = !pick;
  $('shape-undo').disabled = !shapeUndo;
  $('shape-reset').disabled = !selectedShape;
  $('road-reset').disabled = !selectedRoad;
  for (const id of ['signal-move', 'signal-marker', 'signal-marker-reset', 'signal-cycle', 'signal-offset']) $(id).disabled = !selectedSignal;
  for (const id of ['signal-move','signal-marker','signal-cycle','signal-offset']) $(id).disabled = !selectedSignal || controlType(selectedSignal) !== 'traffic_light';
  document.querySelectorAll('[data-mode]').forEach(button => button.classList.toggle('active-tool', button.dataset.mode === pick || button.dataset.mode===pendingShapeAction));
}
function focusShape() { focusPoints(selectedPoints()); }
function draw() {
  ctx.clearRect(0, 0, width, height); ctx.fillStyle = '#e8ede2'; ctx.fillRect(0, 0, width, height);
  if (!net) { requestAnimationFrame(draw); return; }
  for (const p of net.polygons) {
    if (p.drawing && ((editing && draft) || p.simulated)) continue;
    const edit = editing && draft ? draft.artifacts[p.id] || {} : {};
    const hidden = editing ? edit.hidden ?? false : p.hidden;
    if (hidden && !(editing && tool === 'shapes' && $('shape-kind').value === 'artifact')) continue;
    path(editing ? shapePoints({...p, kind: 'artifact'}) : p.shape, true);
    ctx.fillStyle = edit.color || (!editing && p.color) || (/water/.test(p.type) ? '#c5dae0' : /park|grass|wood|garden|forest/.test(p.type) ? '#cbdcc0' : /building/.test(p.type) ? '#d5d7cb' : '#e0e3d7');
    if (hidden) { ctx.strokeStyle = '#b69f7d'; ctx.lineWidth = 1; ctx.stroke(); } else ctx.fill();
  }
  if (editing && draft?.drawings) for (const [id, d] of Object.entries(draft.drawings)) {
    path(d.shape, true); ctx.fillStyle = d.color || catalog?.base?.drawingKinds?.[d.kind] || '#d9d6cc'; ctx.fill(); ctx.strokeStyle = '#9b968a'; ctx.lineWidth = 1; ctx.stroke();
  }
  for (const j of net.junctions) {
    const reset = editing && precision && catalog?.base && (draft?.surfaces[j.id] || net.edits?.surfaces[j.id] || draft?.nodes?.[j.id] || net.edits?.nodes?.[j.id]);
    path(reset ? precision.junctionShape(j.id) : j.shape, true); ctx.fillStyle = '#626c68'; ctx.fill();
  }
  const lanes = displayedLanes();
  drawPedestrianInfrastructure();
  ctx.lineCap = 'butt'; ctx.lineJoin = 'round';
  for (const lane of lanes) {
    if (lane.internal || lane.pedestrian) continue;
    path(lane.shape); ctx.lineWidth = Math.max(1, lane.width * camera.scale);
    ctx.strokeStyle = lane.closed ? '#aa9386' : lane.route ? '#697f7e' : '#626c68'; ctx.stroke();
  }
  if (camera.scale > .7) {
    for (const lane of lanes) {
      if (lane.internal || lane.pedestrian || lane.index === 0 || lane.curb) continue;
      path(offsetLine(lane.shape, lane.width / 2)); ctx.lineWidth = Math.max(.5, .12 * camera.scale);
      ctx.strokeStyle = '#d3dbd1'; ctx.setLineDash([4 * camera.scale, 7 * camera.scale]); ctx.stroke();
    }
    ctx.setLineDash([]);
  }
  if (editing) {
    const highlight = new Set(tool === 'routes' ? routePreview : selectedRoad ? [selectedRoad.edge] : []);
    for (const lane of lanes) {
      if (lane.internal || lane.pedestrian || !highlight.has(lane.edge)) continue;
      path(lane.shape); ctx.lineWidth = Math.max(3, lane.width * camera.scale); ctx.strokeStyle = '#e8b73b99'; ctx.stroke();
    }
    if (tool === 'signals') {
      for (const node of net.junctions) {
        if (!node.editable) continue;
        const p = xy([node.x, node.y]);
        ctx.strokeStyle = node.id === selectedSignal?.id ? '#e4a917' : '#35684690';
        ctx.lineWidth = node.id === selectedSignal?.id ? 3 : 1;
        ctx.beginPath(); ctx.arc(...p, node.id === selectedSignal?.id ? 10 : 5, 0, Math.PI * 2); ctx.stroke();
        const was = node.controlled, now = controlType(node) === 'traffic_light';
        if (now && !was) { ctx.fillStyle = '#44b98b'; ctx.beginPath(); ctx.arc(...p, 8, 0, Math.PI * 2); ctx.fill(); }
        if (was && !now) { ctx.strokeStyle = '#bd4a40'; ctx.beginPath(); ctx.moveTo(p[0]-7,p[1]-7); ctx.lineTo(p[0]+7,p[1]+7); ctx.stroke(); }
      }
    }
    if (tool === 'shapes') {
      const kind = $('shape-kind').value;
      const shapes = kind === 'junction' ? net.junctions.filter(j => j.editable) : net.polygons;
      for (const shape of shapes) {
        path(shapePoints({...shape, kind}), true);
        ctx.strokeStyle = shape.id === hover?.id ? '#19a5cc' : '#719a9b70'; ctx.lineWidth = shape.id === hover?.id ? 3 : 1; ctx.stroke();
      }
    }
    if (selectedShape && tool === 'shapes') {
      const points = selectedPoints(); path(points, true); ctx.fillStyle = '#efbb3e44'; ctx.fill();
      ctx.strokeStyle = '#c29222'; ctx.lineWidth = 2; ctx.stroke();
      for (const [index, p] of points.entries()) { const q = xy(p); ctx.fillStyle = hover?.vertex === index ? '#f6ce62' : '#fff'; ctx.beginPath(); ctx.arc(...q, 6, 0, Math.PI*2); ctx.fill(); ctx.stroke();
        if (camera.scale > 3) { ctx.font='11px -apple-system,sans-serif'; ctx.fillStyle='#17444d'; ctx.textAlign='center'; ctx.fillText(String(index+1),q[0],q[1]-10); }
      }
      if (cutPreview) { path(cutPreview,true); ctx.fillStyle='#40bd8080';ctx.fill(); }
    }
    if (tool === 'signals' && pick === 'move-control' && selectedSignal && hover?.node) {
      path([[selectedSignal.x,selectedSignal.y],[hover.node.x,hover.node.y]]);ctx.strokeStyle='#deaa22';ctx.lineWidth=3;ctx.setLineDash([8,5]);ctx.stroke();ctx.setLineDash([]);
      const q=xy([hover.node.x,hover.node.y]);ctx.beginPath();ctx.arc(...q,14,0,Math.PI*2);ctx.stroke();
    }
    if (cutLine.length && pointer) { path([cutLine[0],cutLine[1]||pointer]);ctx.strokeStyle='#df4c4c';ctx.lineWidth=3;ctx.setLineDash([7,5]);ctx.stroke();ctx.setLineDash([]); }
    if (tool === 'geometry' && precision) precision.draw();
  }
  if (state) {
    if(state.controller_mode!=='fixed')for(const id of net.control_area?.junctions || []){
      const node=net.junctionById.get(id);if(!node)continue;const q=xy([node.x,node.y]);ctx.beginPath();ctx.arc(...q,Math.max(5,Math.min(13,camera.scale*3)),0,Math.PI*2);ctx.strokeStyle='#187b6980';ctx.lineWidth=1.5;ctx.stroke();
    }
    const heads = new Map();
    for (const signal of net.signals) {
      const supported=net.control_area?.active_links?.[signal.tls];if(supported&&!supported.includes(signal.index))continue;
      const node = net.junctionById.get(signal.junction);
      if (editing && node && controlType(node) !== 'traffic_light') continue;
      const delta = editing ? draft?.markers[signal.junction] || [0, 0] : null;
      const savedDelta = net.edits?.markers[signal.junction] || [0, 0];
      const moved = editing && precision ? precision.nodeDelta(signal.junction) : [0, 0];
      const point = delta ? [signal.point[0] + delta[0] - savedDelta[0] + moved[0], signal.point[1] + delta[1] - savedDelta[1] + moved[1]] : signal.point;
      const q = xy(point), key = q.map(Math.round).join(',');
      const color = (state.tls[signal.tls] || '')[signal.index];
      if(!heads.has(key))heads.set(key,{q,colors:[]});heads.get(key).colors.push(color);
    }
    for(const {q,colors} of heads.values()){
      const green=colors.some(c=>/[gG]/.test(c)),red=colors.some(c=>/[rR]/.test(c)),yellow=colors.some(c=>/[yY]/.test(c));
      const radius=Math.min(5,Math.max(2,camera.scale));ctx.fillStyle=yellow?'#e6b552':green?'#57c28c':'#e47564';ctx.beginPath();ctx.arc(...q,radius,0,Math.PI*2);ctx.fill();
      if(green&&red&&!yellow){ctx.fillStyle='#e47564';ctx.beginPath();ctx.moveTo(...q);ctx.arc(...q,radius,Math.PI/2,Math.PI*1.5);ctx.closePath();ctx.fill();}
    }
    const stopPoints = (net.stops || []).filter(s => {
      const node = net.junctionById.get(s.junction);
      return node && controlType(node).endsWith('_stop');
    });
    if (editing) for (const node of net.junctions) {
      if (controlType(node).endsWith('_stop') && !node.controlType.endsWith('_stop')) stopPoints.push({point:[node.x,node.y],junction:node.id});
    }
    const stopSeen = new Set();
    for (const stop of stopPoints) {
      const q=xy(stop.point),key=q.map(Math.round).join(',');if(stopSeen.has(key))continue;stopSeen.add(key);
      const radius=Math.min(7,Math.max(3,camera.scale*1.6));ctx.beginPath();
      for(let i=0;i<8;i++){const a=(i+.5)*Math.PI/4;const p=[q[0]+Math.cos(a)*radius,q[1]+Math.sin(a)*radius];i?ctx.lineTo(...p):ctx.moveTo(...p);}
      ctx.closePath();ctx.fillStyle='#b83d39';ctx.fill();ctx.strokeStyle='#fff';ctx.lineWidth=.7;ctx.stroke();
      if(radius>=6){ctx.fillStyle='#fff';ctx.font='bold 7px sans-serif';ctx.textAlign='center';ctx.fillText('S',q[0],q[1]+2.5);}
    }
    if(camera.scale>.8)for(const node of net.junctions){
      if(!node.reviewNeeded||net.edits?.controls?.[node.id])continue;
      const q=xy([node.x,node.y]);ctx.fillStyle='#795391';ctx.beginPath();ctx.arc(q[0],q[1],7,0,Math.PI*2);ctx.fill();ctx.fillStyle='#fff';ctx.font='bold 10px sans-serif';ctx.textAlign='center';ctx.fillText('?',q[0],q[1]+3.5);
    }
    const old = new Map((previous?.vehicles || []).map(v => [v[0], v]));
    const fraction = state.running ? Math.min(1, (performance.now() - received) / Math.max(60, received - previousReceived)) : 1;
    for (const v of state.vehicles) {
      const ov = old.get(v[0]) || v, p = [ov[1] + (v[1] - ov[1]) * fraction, ov[2] + (v[2] - ov[2]) * fraction], q = xy(p);
      const factor = Math.max(camera.scale, 1.2), length = v[5] * factor, w = Math.max(3, v[6] * factor);
      const angle=v[3]*Math.PI/180;
      // SUMO reports the front bumper, not the centre of the car.
      ctx.save(); ctx.translate(q[0]-Math.sin(angle)*length/2,q[1]+Math.cos(angle)*length/2); ctx.rotate(angle);
      ctx.fillStyle = v[4] < .1 ? '#ce7243' : '#2686bb'; ctx.strokeStyle = '#173b4f'; ctx.lineWidth = .55;
      ctx.beginPath(); ctx.roundRect(-w / 2, -length / 2, w, length, Math.min(1.8, w / 3)); ctx.fill(); ctx.stroke();
      ctx.fillStyle = '#cceaf3'; ctx.fillRect(-w * .32, -length * .3, w * .64, length * .15); ctx.restore();
    }
  }
  if (state?.persons?.length) {
    const radius = Math.max(1.6, .45 * camera.scale);
    ctx.fillStyle = '#5a3d8a'; ctx.strokeStyle = '#fff'; ctx.lineWidth = .8;
    for (const person of state.persons) { const q = xy([person[1], person[2]]); ctx.beginPath(); ctx.arc(q[0], q[1], radius, 0, Math.PI * 2); ctx.fill(); if (radius > 2.5) ctx.stroke(); }
  }
  ctx.fillStyle = '#5b6962'; ctx.font = '11px -apple-system,sans-serif'; ctx.textAlign = 'left';
  ctx.fillText('© OpenStreetMap contributors · drag to pan · scroll to zoom', 20, height - 8);
  requestAnimationFrame(draw);
}

// Sidewalks, walking areas and crosswalks that SUMO actually simulates.
function drawPedestrianInfrastructure() {
  const walks = net.lanes.filter(l => l.pedestrian);
  if (!walks.length) return;
  ctx.lineCap = 'butt'; ctx.lineJoin = 'round';
  for (const lane of walks) if (lane.pedestrian === 'walkingarea') { path(lane.shape, true); ctx.fillStyle = '#d9d6cc'; ctx.fill(); }
  for (const lane of walks) if (lane.pedestrian === 'sidewalk') {
    path(lane.shape); ctx.lineWidth = Math.max(1, lane.width * camera.scale); ctx.strokeStyle = '#d9d6cc'; ctx.stroke();
  }
  for (const lane of walks) if (lane.pedestrian === 'crossing') {
    path(lane.shape); ctx.lineWidth = Math.max(1, lane.width * camera.scale); ctx.strokeStyle = '#f4f2ea';
    ctx.setLineDash([.5 * camera.scale, .5 * camera.scale]); ctx.stroke(); ctx.setLineDash([]);
  }
}
let surfaceNotes = {}, surfaceTimer = null;
function refreshSurfaceNotes() {
  clearTimeout(surfaceTimer);
  if (!draft || !Object.keys(draft.drawings || {}).length) { surfaceNotes = {}; precision?.refreshPanel(); return; }
  surfaceTimer = setTimeout(async () => {
    try { surfaceNotes = (await api('/surface-preview', draft)).notes; }
    catch (error) { surfaceNotes = {}; message(error.message, true); }
    precision?.refreshPanel();
  }, 350);
}
function segmentDistance(p, a, b) {
  const dx = b[0] - a[0], dy = b[1] - a[1], length = dx * dx + dy * dy;
  const t = length ? Math.max(0, Math.min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length)) : 0;
  return Math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy);
}
function nearestRoad(point) {
  let best = null, distance = 20 / camera.scale;
  for (const lane of displayedLanes()) {
    if (lane.internal || lane.pedestrian) continue;
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
function shapeAt(p) {
  const kind = $('shape-kind').value;
  const shapes = kind === 'junction' ? net.junctions.filter(j => j.editable) : net.polygons;
  return shapes.map(s => ({...s, kind})).filter(s => inside(p, shapePoints(s)))
    .sort((a,b) => polygonArea(shapePoints(a)) - polygonArea(shapePoints(b)))[0] || null;
}
function closestVertex(p) {
  let vertex = -1, distance = 20 / camera.scale;
  selectedPoints().forEach((q,i) => { const d=Math.hypot(q[0]-p[0],q[1]-p[1]);if(d<distance){distance=d;vertex=i;} });
  return vertex;
}
function shapeAction(action) {
  if (!selectedShape) { pendingShapeAction=action; mode('select-shape','Step 1: click an outlined area on the map to select it.');return; }
  pendingShapeAction=null;cutLine=[];cutPreview=null;
  const prompts = {'move-shape':'Drag the selected area to move it. Esc cancels.',
    'add-corner':'Click the selected outline to add a handle, then drag that handle.',
    'remove-corner':'Click the numbered corner you want to remove. Esc cancels.',
    'cut-start':'Draw a line across the selected shape. Then click the side you want to keep.'};
  mode(action,prompts[action]);
}
function cancelTool() {
  if (drag?.originalDraft) draft=drag.originalDraft;
  drag=null;pendingShapeAction=null;cutLine=[];cutPreview=null;mode(null,'Tool cancelled. You can select another object or pan the map.');
}
function markerOffset(p) {
  const heads=net.signals.filter(s=>s.junction===selectedSignal.id),saved=net.edits?.markers[selectedSignal.id]||[0,0];
  const origin=heads.length?[heads.reduce((s,h)=>s+h.point[0],0)/heads.length-saved[0],heads.reduce((s,h)=>s+h.point[1],0)/heads.length-saved[1]]:[selectedSignal.x,selectedSignal.y];
  return [p[0]-origin[0],p[1]-origin[1]];
}
function setTool(value) {
  tool = value; pick=null;hover=null;pendingShapeAction=null;cutLine=[];cutPreview=null;canvas.classList.remove('pick-mode');
  document.querySelectorAll('[data-tool]').forEach(button => button.classList.toggle('selected', button.dataset.tool === value));
  for (const name of ['geometry', 'roads', 'routes', 'signals', 'shapes']) $('panel-' + name).hidden = name !== value;
  const help={geometry:'Choose a palette tool on the left, then click a road, junction, or area. Shortcuts: V select, A add point, X delete point, S split, D draw, W sidewalk, M measure, H pan, G snap.',roads:'Click a road. Lane changes appear immediately as a preview.',routes:'Search a street or route ID, choose a result, or click a car.',signals:'Click a light or outlined junction. Then choose what to change.',shapes:'Click an outlined area. Its numbered handles let you reshape it.'};
  message(help[value]);updateTools();
  if (precision) { if (value === 'geometry') precision.setTool(precision.state.tool); else precision.resync(); canvas.style.cursor = value === 'geometry' ? canvas.style.cursor : 'grab'; }
}
function option(value, text) { const o = document.createElement('option'); o.value = value; o.textContent = text; return o; }
function populateRoutes() {
  const query = $('route-search').value.toLowerCase();
  const matches = catalog.routes.filter(r => (r.search || r.label).toLowerCase().includes(query));
  const routes = matches.slice(0, 60);
  $('route-search-results').textContent = query ? `${matches.length} matching routes${matches.length>60?' · showing the first 60':''}` : 'Search any street used along a route, not only its entrance or exit.';
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
  selectedRoad = lane; const base=catalog.base?.roads[lane.edge]||lane;
  const spec = draft.roads[lane.edge] || {lanes: base.laneCount, speed: base.speed, closed: base.closed};
  $('road-selected').textContent = (lane.name || 'Road') + ' · ' + lane.edge;
  $('road-lanes').value = spec.lanes; $('road-speed').value = Math.round(spec.speed / .44704); $('road-closed').checked = spec.closed;
  $('road-preview-info').textContent=`${spec.lanes} lanes selected. Change the number to preview the width.`;message('Change the lane count to see the road widen or narrow. Apply changes to update vehicle paths.');updateTools();
}
function selectSignal(node) {
  selectedSignal = node; $('signal-selected').textContent = (node.streets?.join(' / ') || 'Junction') + ' · ' + node.id;
  const kind = controlType(node);
  $('control-type').value = ['traffic_light','allway_stop','priority_stop','priority'].includes(kind) ? kind : 'priority';
  $('control-source').textContent = node.controlSource + (node.reviewNeeded ? ' · Check this location in the field.' : '') + (kind==='priority_stop'?' Only SUMO minor approaches stop; major approaches retain priority.':'');
  $('signal-cycle').value = draft.timings[node.id]?.cycle || 90; $('signal-offset').value = draft.timings[node.id]?.offset || 0;
  message('Junction selected. Move control: choose a different junction. Move marker: drag or click a new display position.');updateTools();
}
function selectShape(p, kind) {
  selectedShape = {...p, kind};shapeUndo=null; $('shape-selected').textContent = (kind === 'junction' ? 'Paved junction · ' : p.type + ' · ') + p.id;
  $('artifact-fields').hidden = kind !== 'artifact';
  if (kind === 'artifact') { $('artifact-hidden').checked = draft.artifacts[p.id]?.hidden ?? p.hidden; $('artifact-color').value = draft.artifacts[p.id]?.color || p.color || '#d5d7cb'; }
  message(`${selectedPoints().length} corners selected. Drag a numbered handle, or choose a shape tool.`);updateTools();
  if (pendingShapeAction) {const action=pendingShapeAction;focusShape();shapeAction(action);}
}
function mapClick(p) {
  if (!editing) return;
  if (pick === 'waypoint') {
    const lane = nearestRoad(p); if (!lane) return message('Click directly on a road.', true);
    routeSpec.via.push(lane.edge); mode(null); previewRoute(); return;
  }
  if (pick === 'move-control') {
    const node = nearestJunction(p); if (!node) return message('Choose another outlined junction.', true);
    if(node.id===selectedSignal.id)return message('That is the current junction. Choose a different outlined junction, or use Move marker for a small visual adjustment.',true);
    delete draft.signals[selectedSignal.id]; delete draft.signals[node.id];
    draft.controls[selectedSignal.id] = 'priority'; draft.controls[node.id] = 'traffic_light';
    selectSignal(node); mode(null); changed('Actual traffic control moved in the draft. Save to rebuild the signal.'); return;
  }
  if (pick === 'move-marker') {
    draft.markers[selectedSignal.id] = markerOffset(p);
    mode(null); changed('Marker position staged. Its stop control stays at this junction.'); return;
  }
  if (pick === 'add-corner' || pick === 'remove-corner') {
    const points = structuredClone(selectedPoints());
    if (pick === 'add-corner') { let best = 0, distance = Infinity; for (let i = 0; i < points.length; i++) { const d = segmentDistance(p, points[i], points[(i + 1) % points.length]); if (d < distance) { best = i; distance = d; } }
      if(distance*camera.scale>25)return message('Click closer to the selected outline.',true);rememberShape();points.splice(best + 1, 0, p); }
    else { if (points.length <= 3) return message('A shape needs at least three corners.', true);const best=closestVertex(p);if(best<0)return message('Click one of the white numbered handles.',true);rememberShape();points.splice(best, 1); }
    putShape(points); mode(null); changed(); return;
  }
  if(pick==='cut-end'){cutLine[1]=p;mode('cut-side','Click the side you want to KEEP. The preview shows the retained area in green.');return;}
  if(pick==='cut-side'){
    try{const result=cutPolygon(selectedPoints(),...cutLine,p);rememberShape();putShape(result);cutLine=[];cutPreview=null;mode(null);changed('Shape cut. Apply changes to save it, or use Undo shape.');}catch(error){message(error.message,true);}return;
  }
  if (tool === 'roads') { const lane = nearestRoad(p); if (lane) selectRoad(lane); }
  if (tool === 'signals') { const node = nearestJunction(p); if (node) selectSignal(node); }
  if (tool === 'routes') {
    const vehicle = (state?.vehicles || []).find(v => Math.hypot(v[1] - p[0], v[2] - p[1]) * camera.scale < 12);
    if (vehicle && vehicle[9]) selectRoute(vehicle[9]);
  }
  if (tool === 'shapes') {
    const hit=shapeAt(p);if(hit)selectShape(hit,hit.kind);else message('Click inside an outlined area. Use Buildings / terrain for green areas and buildings.',true);
  }
}
canvas.onpointerdown = event => {
  const p = world(event); canvas.setPointerCapture(event.pointerId);
  if (editing && tool === 'geometry' && precision && precision.pointerDown(p, event)) { drag = {kind: 'precision'}; return; }
  if(editing&&pick==='move-marker'&&selectedSignal){drag={kind:'marker',originalDraft:structuredClone(draft)};draft.markers[selectedSignal.id]=markerOffset(p);return;}
  if(editing&&pick==='cut-start'&&selectedShape){drag={kind:'cut',p};cutLine=[p];return;}
  if (editing && tool === 'shapes' && selectedShape) {
    const points = structuredClone(selectedPoints());
    const vertex = closestVertex(p);
    if (pick === 'move-shape' || (vertex >= 0 && !pick)) {rememberShape(); drag = {kind: 'shape', points, p, originalDraft:structuredClone(draft), vertex: pick === 'move-shape' ? -1 : vertex}; return; }
  }
  drag = {kind: 'pan', x: event.clientX, y: event.clientY, cx: camera.x, cy: camera.y, moved: false};
};
canvas.onpointermove = event => {
  pointer=world(event);
  if (editing && tool === 'geometry' && precision) { if (drag?.kind === 'precision') { precision.pointerMove(pointer, event); return; } if (!drag) precision.pointerMove(pointer, event); }
  if(editing&&tool!=='geometry'){
    hover=tool==='shapes'?shapeAt(pointer):tool==='signals'?{node:nearestJunction(pointer)}:null;
    if(tool==='shapes'&&selectedShape){const vertex=closestVertex(pointer);if(vertex>=0)hover={...hover,vertex};canvas.style.cursor=vertex>=0&&!pick?'grab':pick?'crosshair':'default';}
    if(pick==='cut-side'){try{cutPreview=cutPolygon(selectedPoints(),...cutLine,pointer);}catch{cutPreview=null;}}
  }
  if (!drag) return;
  if(drag.kind==='marker'){draft.markers[selectedSignal.id]=markerOffset(pointer);return;}
  if(drag.kind==='cut'){cutLine=[drag.p,pointer];return;}
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
  if (drag?.kind === 'precision') { precision.pointerUp(world(event)); drag = null; return; }
  if(drag?.kind==='cut'){
    const p=world(event);if(Math.hypot(p[0]-drag.p[0],p[1]-drag.p[1])*camera.scale<6){cutLine=[drag.p];mode('cut-end','Click the other end of the cut line.');}
    else{cutLine=[drag.p,p];mode('cut-side','Click the side you want to KEEP. Esc cancels.');}
  }else if (drag?.kind === 'shape') { mode(null); changed('Shape preview updated. Apply to save, or Undo shape.'); }
  else if(drag?.kind==='marker'){mode(null);changed('Marker moved in the preview. Apply to save it.');}
  else if (drag && (!drag.moved || pick)) mapClick(world(event));
  drag = null;
};
canvas.onwheel = event => { event.preventDefault();const before=world(event);camera.scale = Math.max(.1, Math.min(30, camera.scale * Math.exp(-event.deltaY * .001)));const after=world(event);camera.x+=before[0]-after[0];camera.y+=before[1]-after[1]; };

async function openEditor() {
  try {
    await control('pause');
    if (!dirty) { catalog = await api('/editor'); draft = structuredClone(catalog.edits); }
    lanePreviewCache=null;
    for (const key of DRAFT_KEYS) draft[key] ||= {};
    if (!dirty) { committed = structuredClone(draft); undoStack = []; redoStack = []; }
    if (!precision) {
      precision = createPrecisionEditor({$, canvas, ctx, xy, world, getCamera: () => camera, getNet: () => net, getDraft: () => draft, getCatalog: () => catalog,
        changed, message, invalidate: () => { lanePreviewCache = null; }, replaceDraft: next => { draft = next; lanePreviewCache = null; },
        canUndo: () => undoStack.length > 0, canRedo: () => redoStack.length > 0, undo, redo,
        activate: () => { if (tool !== 'geometry') setTool('geometry'); }, focus: focusPoints,
        surfaceNote: id => surfaceNotes[id] ?? net.surface_notes?.[id] ?? 'Checking what this surface does in SUMO…'});
      precision.bindPanel(); window.precisionEditor = precision; // exposed for debugging in the browser console
    }
    $('route-from').replaceChildren(...catalog.origins.map(e => option(e.id, e.label)));
    $('route-to').replaceChildren(...catalog.exits.map(e => option(e.id, e.label)));
    if (selectedRoute && !catalog.routes.some(r => r.id === selectedRoute) && !draft.routes[selectedRoute]) selectedRoute = null;
    populateRoutes(); if (!selectedRoute && catalog.routes.length) selectRoute(catalog.routes[0].id);
    editing = true; $('editor').hidden = false; document.body.classList.add('editing'); setTool(tool);
    message(dirty ? 'You have staged changes. Save to apply them.' : 'Simulation paused for editing. Pick a tool, then select something on the map.');
  } catch (error) { $('error').textContent = error.message; $('error').style.display = 'inline'; }
}
function closeEditor() { editing = false; $('editor').hidden = true; document.body.classList.remove('editing'); mode(null);$('map-instruction').hidden=true;canvas.style.cursor='grab'; }
$('editor-toggle').onclick = () => editing ? closeEditor() : openEditor(); $('editor-close').onclick = closeEditor;
document.querySelectorAll('[data-tool]').forEach(button => button.onclick = () => setTool(button.dataset.tool));
$('route-search').oninput = populateRoutes; $('route-select').onchange = event => selectRoute(event.target.value);
$('route-search').onkeydown = event => {if(event.key==='Enter'&&$('route-select').value){selectRoute($('route-select').value);event.preventDefault();}};
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
function updateRoad(){
  if(!selectedRoad)return;
  if(!$('road-lanes').checkValidity()||!$('road-speed').checkValidity()){message('Enter 1–8 lanes and a valid speed limit.',true);return;}
  const count=Number($('road-lanes').value);
  draft.roads[selectedRoad.edge]={lanes:count,speed:Number($('road-speed').value)*.44704,closed:$('road-closed').checked};
  const saved=net.lanes.find(l=>l.edge===selectedRoad.edge)?.laneCount;
  $('road-preview-info').textContent=`${saved} saved → ${count} preview lanes. Junction connections rebuild when you apply.`;
  changed(`${count}-lane road preview shown on the map. Apply changes to run traffic on it.`);
}
for(const id of ['road-lanes','road-speed']) $(id).oninput=updateRoad;
$('road-closed').onchange=updateRoad;
$('route-rate').onchange = () => { if (selectedRoute?.startsWith('custom_')) { routeSpec.rate = Number($('route-rate').value); draft.routes[selectedRoute] = structuredClone(routeSpec); changed(); } };
$('road-reset').onclick = () => { if (selectedRoad) { delete draft.roads[selectedRoad.edge];selectRoad(selectedRoad);changed('Original road width restored in the preview. Apply to save.'); } };
$('control-type').onchange = event => { if (!selectedSignal) return message('Select a junction first.', true); delete draft.signals[selectedSignal.id];draft.controls[selectedSignal.id] = event.target.value;changed('Control preview updated. Apply to change the actual right-of-way rules.'); };
$('control-reset').onclick=()=>{if(!selectedSignal)return;delete draft.controls[selectedSignal.id];delete draft.signals[selectedSignal.id];selectSignal(selectedSignal);changed('Original intersection control restored in the preview.');};
$('signal-move').onclick = () => { if (selectedSignal) mode('move-control', 'Click the junction where the actual traffic light should operate.'); else message('Select a light first.', true); };
$('signal-marker').onclick = () => { if (selectedSignal) mode('move-marker', 'Click the desired marker position. This does not move the stop control.'); else message('Select a light first.', true); };
$('signal-marker-reset').onclick = () => { if (selectedSignal) { delete draft.markers[selectedSignal.id]; changed(); } };
function updateTiming(){if(!selectedSignal)return;if(!$('signal-cycle').checkValidity()||!$('signal-offset').checkValidity())return;draft.timings[selectedSignal.id]={cycle:Number($('signal-cycle').value),offset:Number($('signal-offset').value)};changed('Reference timing change ready. Apply to rebuild the signal plan.');}
$('signal-cycle').oninput=updateTiming;$('signal-offset').oninput=updateTiming;
$('shape-kind').onchange = () => { selectedShape = null; mode(null); $('shape-selected').textContent = 'No shape selected'; };
$('shape-move').onclick=()=>shapeAction('move-shape');$('shape-add').onclick=()=>shapeAction('add-corner');$('shape-remove').onclick=()=>shapeAction('remove-corner');$('shape-cut').onclick=()=>shapeAction('cut-start');
$('shape-zoom').onclick=()=>{if(selectedShape)focusShape();else message('Select an outlined shape first.',true);};
$('shape-undo').onclick=()=>{if(!selectedShape||!shapeUndo)return;const values=selectedShape.kind==='junction'?draft.surfaces:draft.artifacts;if(shapeUndo.value===null)delete values[selectedShape.id];else values[selectedShape.id]=shapeUndo.value;shapeUndo=null;changed('Last shape edit undone.');};
$('shape-reset').onclick=()=>{if(!selectedShape)return;rememberShape();delete(selectedShape.kind==='junction'?draft.surfaces:draft.artifacts)[selectedShape.id];changed('Original shape restored in the preview. Apply to save.');};
$('artifact-hidden').onchange = event => { if (selectedShape?.kind === 'artifact') { draft.artifacts[selectedShape.id] = {...draft.artifacts[selectedShape.id], hidden: event.target.checked}; changed(); } };
$('artifact-color').oninput = event => { if (selectedShape?.kind === 'artifact') { draft.artifacts[selectedShape.id] = {...draft.artifacts[selectedShape.id], color: event.target.value}; changed(); } };
async function waitForEdits() {
  if (engineUnavailable) { busy=false;updateTools();return; }
  try {
    const job = await api('/editor-job');
    if (job.status === 'failed') throw new Error(job.error || 'Could not apply edits');
    if (job.status === 'saved') {
      catalog = await api('/editor'); draft = structuredClone(catalog.edits); dirty = false;
      for (const key of DRAFT_KEYS) draft[key] ||= {};
      committed = structuredClone(draft); undoStack = []; redoStack = []; precision?.reset();
      lanePreviewCache=null;$('editor-toggle').textContent = 'Edit map'; message('Saved. The simulation is running with your edits.'); busy = false; updateTools();return;
    }
    setTimeout(waitForEdits, 400);
  } catch (error) { busy = false;updateTools(); message(error.message, true); }
}
$('editor-apply').onclick = async () => {
  if (busy) return;
  try { busy = true;updateTools(); await api('/apply-edits', draft); message('Checking routes and rebuilding the edited simulation…'); waitForEdits(); }
  catch (error) { busy = false;updateTools(); message(error.message, true); }
};
$('open-netedit').onclick=async()=>{const button=$('open-netedit');button.disabled=true;button.textContent='Launching…';message('Starting NetEdit. On Mac this also starts XQuartz; please allow a few seconds.');try{const result=await api('/netedit',{});message(result.message);}catch(error){message(error.message,true);}finally{button.disabled=false;button.textContent='Open NetEdit';}};
$('reload-network').onclick = async () => { try { await control('reload-network'); message('Loading the saved NetEdit network…'); setTimeout(waitForEdits, 600); } catch (error) { message(error.message, true); } };
$('toggle').onclick = () => control(state?.running ? 'pause' : 'play'); $('restart').onclick = () => control('restart');
$('speed').onchange = event => control('speed', Number(event.target.value)); $('fit').onclick = () => fit(); $('all-roads').onclick = () => fit(true);
async function changeTraffic(){
  if(dirty){$('peak-hour').value=state.target_hour;$('traffic-mode').value=state.traffic_mode;return message('Apply your staged map changes before switching the traffic scenario.',true);}
  requestedTraffic={hour:Number($('peak-hour').value),mode:$('traffic-mode').value};
  $('peak-hour').disabled=$('traffic-mode').disabled=true;
  try{await control('traffic',requestedTraffic);$('status').textContent='Building scenario and warming up…';}
  catch(error){requestedTraffic=null;$('peak-hour').disabled=$('traffic-mode').disabled=false;$('error').textContent=error.message;$('error').style.display='inline';}
}
$('peak-hour').onchange=changeTraffic;$('traffic-mode').onchange=changeTraffic;
$('controller-mode').onchange=async()=>{
  if(dirty){$('controller-mode').value=state.controller_mode;return message('Apply your staged map changes before comparing controllers.',true);}
  requestedController=$('controller-mode').value;$('controller-mode').disabled=true;
  try{await control('controller',requestedController);$('status').textContent='Restarting the same scenario with the selected controller…';}
  catch(error){requestedController=null;$('controller-mode').disabled=false;$('error').textContent=error.message;$('error').style.display='inline';}
};
$('zoom-in').onclick = () => camera.scale *= 1.25; $('zoom-out').onclick = () => camera.scale /= 1.25;
$('cancel-tool').onclick=cancelTool;
window.addEventListener('keydown',event=>{
  if(!editing)return;
  const typing=/^(INPUT|SELECT|TEXTAREA)$/.test(document.activeElement?.tagName);
  if((event.metaKey||event.ctrlKey)&&!typing&&event.key.toLowerCase()==='z'){event.shiftKey?redo():undo();event.preventDefault();return;}
  if((event.metaKey||event.ctrlKey)&&!typing&&event.key.toLowerCase()==='y'){redo();event.preventDefault();return;}
  if(event.key==='Escape'){if(tool==='geometry'&&precision?.cancel()){event.preventDefault();return;}if(pick){cancelTool();event.preventDefault();}return;}
  if(tool==='geometry'&&precision&&!typing&&precision.keydown(event))event.preventDefault();
});

function showEngineFailure(text) {
  engineUnavailable=true;requestedTraffic=requestedController=null;busy=false;
  $('status').textContent=text;$('dot').style.background='#b94837';
  for(const id of ['toggle','restart','controller-mode','peak-hour','traffic-mode'])$(id).disabled=true;
  $('error').textContent='Start the simulation launcher again. Any unapplied map changes are still staged in this page.';
  $('error').style.display='inline';updateTools();
}

async function poll() {
  try {
    const next = await api('/state');
    if (next.ready) {
      const now=performance.now();
      if(next.time!==lastEngineTime||!next.running||next.updating){lastEngineTime=next.time;lastEngineAdvance=now;}
      engineUnavailable=next.running&&!next.updating&&now-lastEngineAdvance>10000;
      if (!net || net.revision !== next.revision) { const first = !net; net = await api('/network');net.junctionById=new Map(net.junctions.map(j=>[j.id,j]));lanePreviewCache=null;if (first) fit(); previous = null; }
      else previous = state && next.time >= state.time ? state : null;
      state = next; previousReceived = received; received = performance.now();
      $('clock').textContent = [Math.floor(next.time / 3600), Math.floor(next.time / 60) % 60, Math.floor(next.time) % 60].map(v => String(v).padStart(2, '0')).join(':');
      $('status').textContent = next.updating ? 'Applying scenario edits…' : !next.running ? 'Paused' : next.warming ? 'Filling the study area…' : 'SUMO running · ' + next.speed + '× speed';
      $('dot').style.background = next.running ? '#3b9565' : '#c19a4d'; $('toggle').textContent = next.running ? 'Pause' : 'Play';
      if(next.safety_stop){$('status').textContent='Safety check stopped this run. Restart or choose another scenario.';$('toggle').disabled=true;}else $('toggle').disabled=false;
      $('restart').disabled=false;
      if (document.activeElement !== $('speed')) $('speed').value = next.speed;
      if(requestedTraffic&&((next.target_hour===requestedTraffic.hour&&next.traffic_mode===requestedTraffic.mode)||next.edit_error))requestedTraffic=null;
      $('peak-hour').disabled=$('traffic-mode').disabled=!!requestedTraffic;
      if(!requestedTraffic&&!next.updating){$('peak-hour').value=String(next.target_hour);$('traffic-mode').value=next.traffic_mode;}
      if(requestedController===next.controller_mode)requestedController=null;
      $('controller-mode').disabled=!!requestedController;
      if(!requestedController)$('controller-mode').value=next.controller_mode;
      $('active').textContent = next.vehicles.length;
      $('ramp-waiting').textContent = next.ramp_waiting ?? 0;
      $('pending').textContent=next.pending;
      $('scenario-note').textContent = next.traffic_mode!=='baseline' ? 'Assumed stress scenario: heavy mainline demand and a constrained receiving lane. Driver cooperation, demand bursts, and blockages are experimental settings. The next exit is outside this map.' : 'Count-informed baseline: collected station profiles and estimated connecting traffic.';
      for (const key of ['waiting', 'arrived']) $(key).textContent = next[key].toLocaleString();
      const c=next.controller;
      if(c){
        const label=next.controller_mode==='intelligent'?'Intelligent coordinator':next.controller_mode==='coordinated'?'Coordinated adaptive':next.controller_mode==='local'?'Local capacity-aware control':'Fixed-time control';
        $('controller-summary').textContent=`${label} · ${c.signal_count} signals · ${next.warming?'collecting warm-up history':`${c.decisions} adaptive changes`}`;
        $('controller-diagnostics').textContent=next.warming?`Common fixed-control warm-up; the selected controller starts at ${next.target_hour===17?'5 pm':'4 pm'}. Collecting arrival, queue, and discharge history.`:`${c.horizon} s lookahead · ${c.blocked_receivers} blocked receiving links · merge discharge model estimate ${c.merge_discharge_vehicles_per_hour} vehicles/hour · ${c.neighbour_messages} neighbour messages · ${c.decision_ms} ms for the last decision · stock forecast error ${c.forecast_mae_vehicles??'—'} vehicles per observed approach. A split red/green marker means only some turns can go. All figures refer to this simulation.`;
        const names={astoria_31st_north:'31st Street · north',astoria_31st_south:'31st Street · south',astoria_33rd:'33rd Street · ramp entry'};
        $('controller-detail').replaceChildren(...Object.entries(names).filter(([id])=>c.signals[id]).map(([id,name])=>{
          const s=c.signals[id],div=document.createElement('div');div.className='controller-junction';const title=document.createElement('strong');title.textContent=name;
          const text=document.createElement('span');text.textContent=next.warming?`Fixed-control warm-up · ${s.queued_vehicles} queued. Collecting traffic history before the selected start time.`:next.controller_mode==='fixed'?'Repeating protected greens at fixed times, with yellow and occupied-box clearance.':`${s.stage} · phase ${s.phase+1}. ${s.queued_vehicles} queued; ${s.expected_arrivals} approaching (detector estimate). ${s.reason}. Estimated service: ${s.predicted_service} vehicles.`;
          div.append(title,text);return div;
        }));
      }
      $('error').textContent = ''; $('error').style.display = 'none';
      if(next.edit_error){$('error').textContent=next.edit_error;$('error').style.display='inline';}
      if(engineUnavailable)showEngineFailure('No updates from the traffic engine.');
      else updateTools();
    } else if(next.error) {
      showEngineFailure('The traffic engine stopped.');
    }
  } catch (error) { $('status').textContent = 'Reconnecting to SUMO…'; $('error').style.display = 'inline'; $('error').textContent = ' ' + error.message; }
  setTimeout(poll, 100);
}
window.addEventListener('resize', resize); resize(); requestAnimationFrame(draw); poll();
