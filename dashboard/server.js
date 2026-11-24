'use strict';
const http = require('node:http');
const fs = require('node:fs');
const path = require('node:path');
let mqtt = null;
try { mqtt = require('mqtt'); } catch { /* reported as broker state */ }
const {decodeTelemetry, decodeHeartbeat, decodeFault} = require('./protocol');

const PORT = Number(process.env.PORT || 3000);
const HOST = '127.0.0.1';
const MQTT_URL = process.env.MQTT_URL || 'mqtt://127.0.0.1:1883';
const MQTT_USER = process.env.MQTT_USER || '';
const MQTT_PASSWORD = process.env.MQTT_PASSWORD || '';
const PREVIEW = process.argv.includes('--preview') || process.env.PREVIEW === '1';
const STALE_MS = 5000;
const topics = ['robot/telemetry', 'robot/telemetry/detail', 'robot/heartbeat',
  'robot/fault', 'robot/barcode', 'robot/event'];
const MOVEMENTS = ['forward', 'left', 'right', 'uturn'];
const DEFAULT_JUNCTION = 'left'; /* used until the first barcode is decoded */

/* Barcode letter -> movement. Editable live from the dashboard; cleared back
 * to forward when a run ends (Stop, car picked up, Reset run), so each run
 * starts from a fresh mapping. Only ever mutated in place: preview.js holds it. */
const LETTERS = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'.split('');
const mappings = Object.fromEntries(LETTERS.map(l => [l, 'forward']));

function resetMappings() {
  LETTERS.forEach(l => { mappings[l] = 'forward'; });
}
const clients = new Set();
let connection = null;
let commandSequence = 0;
let broker = 'DISCONNECTED';
let live = {lastSeen: null, lastHeartbeat: null, packet: null, packets: 0, invalid: 0, recent: []};
let events = [];
let run = newRun();

function newRun() {
  return {
    startedAt: null, endedAt: null, barcodes: [],
    hump: {peakCm: null, state: 'UNKNOWN'},
    obstacle: {distanceMm: null, widthMm: null, clearLeftMm: null, clearRightMm: null,
      side: null, phase: 'IDLE', seenDuringRun: false, lostLineSinceObstacle: false},
    line: {state: 'UNKNOWN', losses: 0},
    pickedUp: false
  };
}

function log(kind, message) {
  events.unshift({id: Date.now() + '-' + Math.random().toString(16).slice(2, 7), at: Date.now(), kind, message});
  events = events.slice(0, 80);
}

function startRunIfNeeded() {
  if (!run.startedAt) run.startedAt = Date.now();
}

function endRun() {
  if (run.startedAt && !run.endedAt) run.endedAt = Date.now();
  resetMappings();
  log('INFO', 'Run ended — barcode mapping cleared');
}

function cm(mm) { return mm == null ? null : Math.round(mm / 10 * 10) / 10; }

/* What the car will do at the next junction: the mapping of the most recently
 * decoded letter (live, so remapping that letter changes it), else the default. */
function nextJunction() {
  const last = run.barcodes[0];
  if (!last) return {letter: null, action: DEFAULT_JUNCTION, at: null};
  return {letter: last.letter, action: mappings[last.letter] || 'forward', at: last.at};
}

function snapshot() {
  const now = Date.now();
  const linkUp = broker === 'CONNECTED' || broker === 'PREVIEW';
  const stale = !linkUp || !live.lastSeen || now - live.lastSeen > STALE_MS;
  const carConnected = linkUp && !stale;
  live.recent = live.recent.filter(t => now - t < 10000);
  return {
    preview: PREVIEW, broker,
    mqttUrl: PREVIEW ? 'simulated' : MQTT_URL.replace(/:\/\/([^/@]+)@/, '://***@'),
    brokerConnected: linkUp, carConnected, connected: carConnected, stale,
    lastSeen: live.lastSeen, lastHeartbeat: live.lastHeartbeat,
    rateHz: Math.round(live.recent.length / 10 * 10) / 10,
    data: carConnected ? live.packet : null,
    packets: live.packets, invalid: live.invalid,
    events: events.slice(0, 40),
    mapping: mappings,
    nextJunction: nextJunction(),
    run: {startedAt: run.startedAt, endedAt: run.endedAt, pickedUp: run.pickedUp,
      barcodes: run.barcodes.slice(0, 6), hump: run.hump, obstacle: run.obstacle,
      line: run.line},
    now
  };
}

function push() {
  const message = 'data: ' + JSON.stringify(snapshot()) + '\n\n';
  for (const client of clients) {
    try { client.write(message); } catch { clients.delete(client); }
  }
}

function commandPacket(type, argument = 0) {
  const types = {start: 1, stop: 4, junction: 6};
  if (!(type in types)) throw Error('Unsupported robot command');
  return Buffer.from([(types[type] << 5) | (argument & 31), commandSequence++ & 255]);
}

function publishCommand(type) {
  const movement = {forward: 0, reverse: 1, left: 2, right: 3, uturn: 4};
  const packetType = movement[type] === undefined ? type : 'junction';
  const packetArgument = movement[type] === undefined ? 0 : movement[type];
  const packet = commandPacket(packetType, packetArgument);
  if (PREVIEW) { log('COMMAND', 'Sent ' + type + ' (preview, not transmitted)'); return; }
  if (!connection || broker !== 'CONNECTED') throw Error('MQTT broker is not connected');
  connection.publish('robot/command', packet, {qos: 0}, error => {
    if (error) log('ERROR', 'Command publish failed: ' + error.message);
  });
  log('COMMAND', 'Sent ' + type);
}

/* ---- Telemetry interpretation ------------------------------------------- */

function onTelemetry(t) {
  if (t.vehicle === 'RUNNING') startRunIfNeeded();
  if (run.startedAt && !run.endedAt) {
    const line = run.line;
    const o = run.obstacle;
    if (line.state === 'FOUND' && t.line === 'LOST') {
      line.losses++;
      if (o.seenDuringRun) o.lostLineSinceObstacle = true;
    }
    if (line.state === 'LOST' && t.line === 'FOUND' && o.lostLineSinceObstacle && o.phase !== 'REJOINED') {
      o.phase = 'REJOINED';
      log('ROBOT', 'Line reacquired after obstacle');
    }
    line.state = t.line;
    if (t.obstacle === 'DETECTED' && !o.seenDuringRun) {
      o.seenDuringRun = true;
      if (o.phase === 'IDLE') o.phase = 'DETECTED';
      log('ROBOT', 'Obstacle detected');
    }
    run.hump.state = t.terrain;
  }
  if (t.faultFlags) log('FAULT', 'Fault flags: ' + t.faults.join(', '));
}

function setHumpPeak(heightCm) {
  if (heightCm == null || !(heightCm >= 0)) return;
  if (run.hump.peakCm == null || heightCm > run.hump.peakCm) run.hump.peakCm = Math.round(heightCm * 10) / 10;
}

function setObstacleProfile(p) {
  const o = run.obstacle;
  if (p.widthMm != null) o.widthMm = Number(p.widthMm);
  if (p.clearLeftMm != null) o.clearLeftMm = Number(p.clearLeftMm);
  if (p.clearRightMm != null) o.clearRightMm = Number(p.clearRightMm);
  if (p.side) o.side = String(p.side).toUpperCase();
}

function pickup() {
  if (run.pickedUp) return;
  run.pickedUp = true;
  endRun();
}

/* Optional numeric fields merged from robot/telemetry/detail (JSON). */
function onDetail(d) {
  if (d.humpHeightCm != null) setHumpPeak(Number(d.humpHeightCm));
  if (d.obstacleMm != null) run.obstacle.distanceMm = Number(d.obstacleMm);
  if (d.obstacleWidthMm != null || d.clearLeftMm != null || d.clearRightMm != null || d.avoidSide) {
    setObstacleProfile({widthMm: d.obstacleWidthMm, clearLeftMm: d.clearLeftMm,
      clearRightMm: d.clearRightMm, side: d.avoidSide});
  }
  if (d.avoidPhase) run.obstacle.phase = String(d.avoidPhase).toUpperCase();
  if (d.pickedUp === true) { pickup(); log('ROBOT', 'Car picked up — motors stopped'); }
}

/* Discrete robot events from robot/event (JSON: {"type": "...", ...}). */
function onRobotEvent(e) {
  const type = String(e.type || '').toUpperCase();
  const o = run.obstacle;
  switch (type) {
    case 'BARCODE_DETECTED':
      log('ROBOT', 'Barcode detected');
      break;
    case 'HUMP':
      setHumpPeak(e.peakMm != null ? Number(e.peakMm) / 10 : Number(e.peakCm));
      log('ROBOT', 'Hump peak reported: ' + run.hump.peakCm + ' cm');
      break;
    case 'OBSTACLE':
      if (e.distMm != null) o.distanceMm = Number(e.distMm);
      o.seenDuringRun = true;
      o.phase = 'SCANNING';
      log('ROBOT', 'Obstacle detected' + (e.distMm != null ? ' at ' + cm(e.distMm) + ' cm' : ''));
      break;
    case 'OBSTACLE_PROFILE':
      setObstacleProfile(e);
      o.phase = 'BYPASS';
      log('ROBOT', 'Obstacle profile: width ' + cm(o.widthMm) + ' cm, bypass ' + (o.side || '?'));
      break;
    case 'AVOID':
      if (e.phase) o.phase = String(e.phase).toUpperCase();
      log('ROBOT', 'Avoidance: ' + o.phase);
      break;
    case 'LINE_REJOINED':
      o.phase = 'REJOINED';
      log('ROBOT', 'Line rejoined after obstacle');
      break;
    case 'ACTION_DONE': {
      const b = run.barcodes[0];
      if (b) b.status = 'done';
      log('ROBOT', 'Executed ' + String(e.action || (b && b.action) || 'junction action').toLowerCase());
      break;
    }
    case 'PICKUP_STOP':
      pickup();
      log('ROBOT', 'Car picked up — motors stopped');
      break;
    default:
      throw Error('Unknown robot event type');
  }
}

function onBarcode(letter) {
  const action = mappings[letter] || 'forward';
  startRunIfNeeded();
  run.barcodes.unshift({at: Date.now(), letter, action, status: 'sent'});
  run.barcodes = run.barcodes.slice(0, 12);
  log('ROBOT', 'Decoded barcode ' + letter + ' → next junction: ' + action);
  publishCommand(action);
}

function handleMessage(topic, payload, packet) {
  try {
    if ((topic === 'robot/telemetry' || topic === 'robot/telemetry/detail') && packet && packet.retain) return;
    if (topic === 'robot/telemetry') {
      const next = decodeTelemetry(payload);
      live.packet = {...(live.packet || {}), ...next, at: Date.now()};
      live.lastSeen = Date.now(); live.packets++; live.recent.push(Date.now());
      onTelemetry(next);
    } else if (topic === 'robot/telemetry/detail') {
      const detail = JSON.parse(payload.toString('utf8'));
      if (!detail || typeof detail !== 'object' || Array.isArray(detail)) throw Error('Telemetry detail must be a JSON object');
      live.packet = {...(live.packet || {}), ...detail, at: Date.now()};
      live.lastSeen = Date.now(); live.packets++; live.recent.push(Date.now());
      onDetail(detail);
    } else if (topic === 'robot/event') {
      const e = JSON.parse(payload.toString('utf8'));
      if (!e || typeof e !== 'object' || Array.isArray(e)) throw Error('Robot event must be a JSON object');
      startRunIfNeeded();
      onRobotEvent(e);
    } else if (topic === 'robot/barcode') {
      const letter = payload.toString('utf8').trim().toUpperCase();
      if (!/^[A-Z]$/.test(letter)) throw Error('Barcode must be a single letter A-Z');
      onBarcode(letter);
    } else if (topic === 'robot/heartbeat') {
      decodeHeartbeat(payload);
      live.lastHeartbeat = Date.now();
    } else if (topic === 'robot/fault') {
      const fault = decodeFault(payload);
      if (fault.faultFlags) log('FAULT', 'Fault flags: ' + fault.faults.join(', '));
    }
    push();
  } catch (error) {
    live.invalid++;
    if (live.invalid <= 5) log('ERROR', 'Rejected malformed ' + topic + ' packet: ' + error.message);
  }
}

function connectMQTT() {
  if (!mqtt) { broker = 'MQTT PACKAGE NOT INSTALLED'; return; }
  const options = {clientId: 'picocar-dashboard-' + process.pid, reconnectPeriod: 2500, connectTimeout: 6000, clean: true};
  if (MQTT_USER) options.username = MQTT_USER;
  if (MQTT_PASSWORD) options['pass' + 'word'] = MQTT_PASSWORD;
  connection = mqtt.connect(MQTT_URL, options);
  connection.on('connect', () => {
    broker = 'CONNECTED';
    connection.subscribe(topics, {qos: 0}, error => { if (error) log('ERROR', 'MQTT topic subscription failed'); });
    push();
  });
  const drop = state => () => { broker = state; live.lastSeen = null; live.packet = null; push(); };
  connection.on('reconnect', drop('RECONNECTING'));
  connection.on('close', drop('DISCONNECTED'));
  connection.on('error', error => { broker = 'ERROR'; log('ERROR', 'MQTT connection error: ' + error.message); });
  connection.on('message', handleMessage);
}

/* ---- HTTP ------------------------------------------------------------------ */

const allowed = {'/': 'index.html', '/app.js': 'app.js', '/styles.css': 'styles.css'};
const mime = {'.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8', '.css': 'text/css; charset=utf-8'};

function readBody(req, done) {
  let body = '';
  req.on('data', chunk => { body += chunk; if (body.length > 1000) req.destroy(); });
  req.on('end', () => { try { done(JSON.parse(body)); } catch { done(null); } });
}

function sendJson(res, status, obj) {
  res.writeHead(status, {'Content-Type': 'application/json'});
  res.end(JSON.stringify(obj));
}

const server = http.createServer((req, res) => {
  const pathname = new URL(req.url, 'http://localhost').pathname;
  res.setHeader('Cache-Control', 'no-store');
  res.setHeader('X-Content-Type-Options', 'nosniff');
  if (pathname === '/api/events') {
    res.writeHead(200, {'Content-Type': 'text/event-stream', 'Connection': 'keep-alive', 'Cache-Control': 'no-cache'});
    clients.add(res);
    res.write('data: ' + JSON.stringify(snapshot()) + '\n\n');
    req.on('close', () => clients.delete(res));
    return;
  }
  if (pathname === '/api/state') { sendJson(res, 200, snapshot()); return; }
  if (pathname === '/api/events/clear' && req.method === 'POST') { events = []; push(); sendJson(res, 200, {ok: true}); return; }
  if (pathname === '/api/run/reset' && req.method === 'POST') {
    run = newRun(); resetMappings();
    log('INFO', 'Run reset — checklist and barcode mapping cleared'); push(); sendJson(res, 200, {ok: true}); return;
  }
  if (pathname === '/api/command' && req.method === 'POST') {
    readBody(req, body => {
      try {
        if (!body || !['start', 'stop'].includes(body.type)) throw Error('Invalid command');
        if (body.type === 'start') { run = newRun(); run.startedAt = Date.now(); }
        else endRun();
        publishCommand(body.type);
        push(); sendJson(res, 200, {ok: true});
      } catch (error) { res.writeHead(400); res.end(error.message); }
    });
    return;
  }
  if (pathname === '/api/mapping' && req.method === 'GET') { sendJson(res, 200, mappings); return; }
  if (pathname === '/api/mapping' && req.method === 'POST') {
    readBody(req, body => {
      try {
        if (!body || !/^[A-Z]$/.test(body.letter) || !MOVEMENTS.includes(body.movement)) throw Error('Invalid mapping');
        const changed = mappings[body.letter] !== body.movement;
        mappings[body.letter] = body.movement;
        log('INFO', 'Barcode ' + body.letter + ' mapped to ' + body.movement);
        const last = run.barcodes[0];
        if (changed && last && last.letter === body.letter) {
          /* The pending junction instruction changed: resend it to the car. */
          last.action = body.movement; last.status = 'sent';
          try { publishCommand(body.movement); } catch (error) { log('ERROR', 'Resend failed: ' + error.message); }
        }
        push(); sendJson(res, 200, {ok: true});
      } catch (error) { res.writeHead(400); res.end(error.message); }
    });
    return;
  }
  const filename = allowed[pathname];
  if (!filename || req.method !== 'GET') { res.writeHead(404); res.end('Not found'); return; }
  fs.readFile(path.join(__dirname, 'public', filename), (error, bytes) => {
    if (error) { res.writeHead(500); res.end('Internal error'); return; }
    res.writeHead(200, {'Content-Type': mime[path.extname(filename)],
      'Content-Security-Policy': "default-src 'self'; connect-src 'self'; style-src 'self'; script-src 'self'; object-src 'none'; base-uri 'none'; frame-ancestors 'none'"});
    res.end(bytes);
  });
});

server.listen(PORT, HOST, () => console.log(`PicoCar Mission Control${PREVIEW ? ' (PREVIEW)' : ''}: http://${HOST}:${PORT}`));
setInterval(push, 1000); /* keeps timers and "last seen" ages fresh */

if (PREVIEW) {
  broker = 'PREVIEW';
  require('./preview').start({handleMessage, mappings, reset: () => { run = newRun(); run.startedAt = Date.now(); log('INFO', 'Preview run started'); }});
} else {
  connectMQTT();
}
