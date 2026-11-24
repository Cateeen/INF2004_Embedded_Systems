'use strict';
const $ = id => document.getElementById(id);
const LETTERS = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'.split('');
const MOVE = {forward: {icon: '↑', label: 'Go straight'}, left: {icon: '↰', label: 'Turn left'},
  right: {icon: '↱', label: 'Turn right'}, uturn: {icon: '↻', label: 'U-turn'}};
let latest = null;
let clockSkew = 0;

const num = (x, digits = 1) => x === null || x === undefined || Number.isNaN(Number(x)) ? '—' : Number(x).toFixed(digits);
const cm = mm => mm == null ? '—' : num(mm / 10, 1) + ' cm';
const clock = t => new Date(t || Date.now()).toLocaleTimeString('en-SG', {hour12: false});
function set(id, value) { const node = $(id); if (node) node.textContent = String(value); }
function el(tag, cls, text) { const n = document.createElement(tag); if (cls) n.className = cls; if (text != null) n.textContent = text; return n; }
function chip(id, text, tone) { const n = $(id); n.textContent = text; n.className = 'chip ' + (tone || ''); }

function fmtDuration(ms) {
  if (ms == null || ms < 0) return '00:00.0';
  const m = Math.floor(ms / 60000), s = (ms % 60000) / 1000;
  return String(m).padStart(2, '0') + ':' + s.toFixed(1).padStart(4, '0');
}
function sinceRunStart(s, at) {
  return s.run && s.run.startedAt ? '+' + fmtDuration(at - s.run.startedAt) : clock(at);
}

function currentState(d, s) {
  if (s.run.pickedUp) return ['Picked up · stopped', 'red'];
  if (d.vehicle === 'FAULT' || d.faultFlags > 0) return ['Fault · safe stop', 'red'];
  const phase = s.run.obstacle.phase;
  if (d.obstacle === 'DETECTED' || ['SCANNING', 'BYPASS', 'SEARCH'].includes(phase)) return ['Avoiding obstacle', 'amber'];
  if (d.terrain === 'HUMP') return ['On hump', 'amber'];
  if (d.line === 'FOUND' && d.vehicle === 'RUNNING') return ['Line following', 'green'];
  if (d.line === 'LOST' && d.vehicle === 'RUNNING') return ['Searching for line', 'amber'];
  return [d.vehicle === 'COMPLETE' ? 'Complete' : 'Idle', ''];
}

/* ---- Static parts built once ------------------------------------------- */
const letterSelect = $('mapping-letter');
LETTERS.forEach(l => letterSelect.append(new Option(l, l)));
const grid = $('letter-grid');
LETTERS.forEach(l => {
  const b = el('button', 'letter');
  b.type = 'button'; b.dataset.letter = l;
  b.append(el('span', 'l', l), el('span', 'm', '↑'));
  b.addEventListener('click', () => { letterSelect.value = l; syncMappingForm(); });
  grid.append(b);
});
let mappingShown = false;
function syncMappingForm() {
  if (latest) { $('mapping-movement').value = latest.mapping[letterSelect.value] || 'forward'; mappingShown = true; }
  highlightEditing();
}
function highlightEditing() {
  grid.querySelectorAll('.letter').forEach(b => b.classList.toggle('editing', b.dataset.letter === letterSelect.value));
}
letterSelect.addEventListener('change', syncMappingForm);

/* ---- Render ----------------------------------------------------------------- */
function renderNextJunction(s) {
  const nj = s.nextJunction;
  const mv = MOVE[nj.action] || MOVE.left;
  set('nj-icon', mv.icon);
  set('nj-action', mv.label.toUpperCase());
  set('nj-source', nj.letter ? 'From barcode ' + nj.letter + ' · decoded ' + sinceRunStart(s, nj.at)
    : 'Default — no barcode decoded yet');
  $('next-card').classList.toggle('isdefault', !nj.letter);
}

function renderMapping(s) {
  const last = s.run.barcodes[0];
  grid.querySelectorAll('.letter').forEach(b => {
    const mv = s.mapping[b.dataset.letter] || 'forward';
    b.querySelector('.m').textContent = MOVE[mv].icon;
    b.title = b.dataset.letter + ' → ' + MOVE[mv].label;
    b.classList.toggle('changed', mv !== 'forward');
    b.classList.toggle('lastread', !!last && last.letter === b.dataset.letter);
  });
  set('last-barcode-tag', last ? 'LAST DECODED: ' + last.letter : 'NO BARCODE YET');
  const list = $('barcode-list');
  list.replaceChildren();
  if (!s.run.barcodes.length) { list.append(el('div', 'emptylog', 'No barcodes read this run')); return; }
  for (const b of s.run.barcodes) {
    const row = el('div', 'barcode-row');
    row.append(el('span', 'code', b.letter), el('span', 'arrow', MOVE[b.action].icon + ' ' + MOVE[b.action].label),
      el('span', 'status ' + b.status, b.status === 'done' ? 'EXECUTED' : 'SENT'), el('time', null, sinceRunStart(s, b.at)));
    list.append(row);
  }
}

function renderCards(s, d) {
  const r = s.run;
  /* Line */
  set('line-word', !d ? '—' : d.line === 'FOUND' ? 'On line' : d.line === 'LOST' ? 'Line lost' : 'Unknown');
  set('line-losses', r.startedAt ? r.line.losses + ' time' + (r.line.losses === 1 ? '' : 's') : '—');
  set('ir-raw', d && d.ir ? `${d.ir.left ?? d.ir.barcode ?? '—'} / ${d.ir.center ?? '—'} / ${d.ir.right ?? '—'}` : 'not sent');
  const line = d ? d.line : null;
  chip('line-chip', line || '—', line === 'FOUND' ? 'green' : line === 'LOST' ? 'amber' : '');
  /* Hump */
  set('hump-peak', r.hump.peakCm == null ? '—' : num(r.hump.peakCm, 1));
  const onHump = d && d.terrain === 'HUMP';
  set('hump-now', onHump && d.humpHeightCm != null ? num(d.humpHeightCm, 1) + ' cm' : onHump ? 'on hump' : 'flat');
  chip('terrain-chip', d ? d.terrain : '—', onHump ? 'amber' : d && d.terrain === 'FLAT' ? 'green' : '');
  /* Obstacle */
  const o = r.obstacle;
  set('obs-width', o.widthMm == null ? '—' : num(o.widthMm / 10, 1));
  set('obs-dist', d && d.obstacleMm != null ? cm(d.obstacleMm) : o.distanceMm != null ? cm(o.distanceMm) + ' (at detection)' : '—');
  set('obs-clear', o.clearLeftMm == null && o.clearRightMm == null ? '—' : cm(o.clearLeftMm) + ' / ' + cm(o.clearRightMm));
  set('obs-side', o.side || '—');
  const total = (o.clearLeftMm || 0) + (o.widthMm || 0) + (o.clearRightMm || 0);
  [['seg-left', o.clearLeftMm], ['seg-mid', o.widthMm], ['seg-right', o.clearRightMm]].forEach(([id, v]) => {
    $(id).style.width = total ? ((v || 0) / total * 100) + '%' : '0%';
  });
  $('seg-left').classList.toggle('chosen', o.side === 'LEFT');
  $('seg-right').classList.toggle('chosen', o.side === 'RIGHT');
  const phase = o.phase === 'IDLE' ? (d && d.obstacle === 'CLEAR' ? 'CLEAR' : d ? d.obstacle : '—') : o.phase;
  chip('obstacle-chip', phase, phase === 'REJOINED' || phase === 'CLEAR' ? 'green' : phase === '—' || phase === 'UNKNOWN' ? '' : 'amber');
  /* MQTT */
  const now = Date.now() + clockSkew;
  set('mqtt-rate', s.carConnected ? num(s.rateHz, 1) : '—');
  set('mqtt-age', s.lastSeen ? num((now - s.lastSeen) / 1000, 1) + ' s ago' : 'never');
  set('mqtt-hb', s.lastHeartbeat ? num((now - s.lastHeartbeat) / 1000, 1) + ' s ago' : 'none received');
  set('mqtt-count', s.packets + ' / ' + s.invalid);
  chip('mqtt-chip', s.preview ? 'PREVIEW' : s.broker, s.carConnected ? 'green' : 'red');
}

function renderLog(events, s) {
  const list = $('event-list');
  list.replaceChildren();
  if (!events.length) { list.append(el('div', 'emptylog', 'No recorded events yet')); return; }
  for (const e of events) {
    const r = el('div', 'logitem');
    r.append(el('time', null, clock(e.at)), el('span', 'kind ' + e.kind.toLowerCase(), e.kind), el('span', null, e.message));
    list.append(r);
  }
}

function render(s) {
  latest = s;
  clockSkew = s.now - Date.now();
  const d = s.data;
  const connected = s.carConnected === true;
  const label = s.preview ? 'PREVIEW · SIMULATED CAR' : connected ? 'CAR CONNECTED' : s.brokerConnected ? 'MQTT CONNECTED · NO CAR' : 'NO MQTT CONNECTION';
  set('connection-text', label); set('side-state', label);
  $('connection-dot').className = 'signal ' + (connected ? 'ok' : 'bad');
  $('side-dot').className = 'signal ' + (connected ? 'ok' : 'bad');
  $('preview-banner').classList.toggle('hidden', !s.preview);
  $('stale-banner').classList.toggle('hidden', !s.stale || s.preview);
  $('pickup-banner').classList.toggle('hidden', !s.run.pickedUp);
  const [runLabel, runTone] = s.run.pickedUp ? ['Robot stopped · picked up', 'stopped']
    : s.run.endedAt ? ['Robot stopped', 'stopped']
    : s.run.startedAt ? ['Robot running', 'running']
    : ['Robot idle · not started', 'idle'];
  set('run-text', runLabel);
  $('run-pill').className = 'pill runpill ' + runTone;
  $('run-dot').className = 'signal ' + (runTone === 'running' ? 'ok' : runTone === 'stopped' ? 'bad' : '');
  if (s.run.pickedUp) set('pickup-detail', 'End of demo');
  set('link-tag', s.preview ? 'PREVIEW' : 'LIVE MQTT');
  set('footer-mode', s.preview ? 'Preview mode · simulated data' : 'Local dashboard · real MQTT data only');
  ['robot-start', 'robot-stop', 'save-mapping'].forEach(id => { $(id).disabled = !s.brokerConnected; });

  if (d) {
    const [state, tone] = currentState(d, s);
    set('vehicle-status', state);
    $('vehicle-status').className = 'kpi-value small ' + tone;
    set('vehicle-hint', d.faults && d.faults.length ? 'Faults: ' + d.faults.join(', ') : 'Vehicle ' + d.vehicle.toLowerCase() + ' · line ' + d.line.toLowerCase());
  } else {
    set('vehicle-status', '—'); $('vehicle-status').className = 'kpi-value small';
    set('vehicle-hint', 'No current telemetry');
  }
  renderNextJunction(s);
  renderMapping(s);
  renderCards(s, d);
  renderLog(s.events, s);
  if (mappingShown) highlightEditing(); else syncMappingForm();
}

/* ---- Actions ---------------------------------------------------------------- */
async function post(url, obj) {
  try {
    const r = await fetch(url, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(obj)});
    if (!r.ok) throw Error(await r.text());
  } catch (e) { alert('Dashboard request failed: ' + e.message); }
}
$('robot-start').addEventListener('click', () => post('/api/command', {type: 'start'}));
$('robot-stop').addEventListener('click', () => post('/api/command', {type: 'stop'}));
$('save-mapping').addEventListener('click', () => post('/api/mapping', {letter: letterSelect.value, movement: $('mapping-movement').value}));
$('reset-run').addEventListener('click', () => { if (confirm('Clear the run, barcode history and barcode mapping? Next junction returns to Turn left.')) post('/api/run/reset', {}); });
$('clear-log').addEventListener('click', () => { if (confirm('Clear the event timeline?')) post('/api/events/clear', {}); });
$('download-log').addEventListener('click', () => {
  const rows = [['Time', 'Type', 'Event'], ...(latest?.events || []).map(e => [new Date(e.at).toISOString(), e.kind, e.message])];
  const csv = rows.map(r => r.map(x => '"' + String(x).replaceAll('"', '""') + '"').join(',')).join('\r\n');
  const blob = new Blob([csv], {type: 'text/csv'}), url = URL.createObjectURL(blob), a = document.createElement('a');
  a.href = url; a.download = 'picocar-run.csv'; a.click(); URL.revokeObjectURL(url);
});

const feed = new EventSource('/api/events');
feed.onmessage = e => { try { render(JSON.parse(e.data)); } catch (err) { console.warn('Invalid dashboard event', err); } };
feed.onerror = () => set('connection-text', 'Dashboard reconnecting…');
