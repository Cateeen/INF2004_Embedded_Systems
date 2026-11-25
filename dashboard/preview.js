'use strict';
/* Preview mode: replays a scripted demo run through the real message handler
 * so the dashboard can be reviewed without the car. Nothing is transmitted. */

const STATE = {IDLE: 0, RUNNING: 1};
const LINE = {UNKNOWN: 0, FOUND: 1, LOST: 2};
const OBST = {CLEAR: 1, DETECTED: 2};
const TERRAIN = {FLAT: 1, HUMP: 2};
const LOOP_S = 38;

function telemetryPacket(s, seq) {
  return Buffer.from([
    (STATE[s.vehicle] << 2) | LINE[s.line],
    (OBST[s.obstacle] << 6) | (TERRAIN[s.terrain] << 4),
    (seq & 127) << 1, 0, 0
  ]);
}

const json = obj => Buffer.from(JSON.stringify(obj));

function start({handleMessage, mappings, reset}) {
  Object.assign(mappings, {A: 'left', B: 'right', C: 'forward', D: 'uturn'});
  let s, seq = 0, t0 = 0, fired = new Set();
  const send = (topic, payload) => handleMessage(topic, payload, {retain: false});

  /* [second, action] — mirrors the order the assessor will see on the course. */
  const script = [
    [4.0, () => send('robot/event', json({type: 'BARCODE_DETECTED'}))],
    [4.6, () => send('robot/barcode', Buffer.from('B'))],
    [6.2, () => send('robot/event', json({type: 'ACTION_DONE', action: 'right'}))],
    [7.5, () => send('robot/event', json({type: 'LINE', event: 'PIVOT'}))],
    [8.4, () => send('robot/event', json({type: 'LINE', event: 'SEARCH_CENTRED'}))],
    [9.0, () => { s.terrain = 'HUMP'; }],
    [9.5, () => send('robot/telemetry/detail', json({humpHeightCm: 1.4}))],
    [10.0, () => send('robot/telemetry/detail', json({humpHeightCm: 3.2}))],
    [10.5, () => send('robot/telemetry/detail', json({humpHeightCm: 2.1}))],
    [11.0, () => { s.terrain = 'FLAT'; send('robot/event', json({type: 'HUMP', peakMm: 32})); }],
    [14.0, () => { s.obstacle = 'DETECTED'; s.speed = 0; send('robot/event', json({type: 'OBSTACLE', distMm: 180})); }],
    [16.0, () => send('robot/event', json({type: 'OBSTACLE_PROFILE', widthMm: 120, clearLeftMm: 260, clearRightMm: 90, side: 'LEFT'}))],
    [17.0, () => { s.line = 'UNKNOWN'; s.speed = 0.2; }],   /* not following while it drives round */
    [19.0, () => send('robot/event', json({type: 'AVOID', phase: 'SEARCH'}))],
    [21.0, () => { s.line = 'FOUND'; s.obstacle = 'CLEAR'; s.speed = 0.32; send('robot/event', json({type: 'LINE_REJOINED'})); }],
    [22.5, () => send('robot/event', json({type: 'LINE', event: 'SEARCH_FOUND'}))],
    [24.0, () => send('robot/barcode', Buffer.from('D'))],
    [26.0, () => send('robot/event', json({type: 'ACTION_DONE', action: 'uturn'}))],
    [30.0, () => { s.vehicle = 'IDLE'; s.speed = 0; send('robot/event', json({type: 'PICKUP_STOP'})); }]
  ];

  function begin() {
    s = {vehicle: 'RUNNING', line: 'FOUND', obstacle: 'CLEAR', terrain: 'FLAT', speed: 0.32};
    fired = new Set(); t0 = Date.now();
    reset();
  }

  begin();
  setInterval(() => {
    const t = (Date.now() - t0) / 1000;
    if (t > LOOP_S) { begin(); return; }
    script.forEach(([at, fn], i) => { if (t >= at && !fired.has(i)) { fired.add(i); fn(); } });
    send('robot/telemetry', telemetryPacket(s, seq++));
    if (seq % 4 === 0) send('robot/heartbeat', Buffer.from([(1 << 5) | (STATE[s.vehicle] << 2), seq & 127]));
  }, 250);
}

module.exports = {start};
