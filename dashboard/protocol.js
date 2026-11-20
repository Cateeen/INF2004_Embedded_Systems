'use strict';
// Decodes the exact 5-byte wire protocol in src/telemetry_protocol.c.
const states = ['IDLE', 'RUNNING', 'PAUSED', 'FAULT', 'COMPLETE'];
const lines = ['UNKNOWN', 'FOUND', 'LOST'];
const obstacles = ['UNKNOWN', 'CLEAR', 'DETECTED'];
const terrain = ['UNKNOWN', 'FLAT', 'HUMP'];
const faults = [
  [1, 'MOTOR_STALL'], [2, 'ENCODER'], [4, 'IMU'],
  [8, 'NO_ECHO'], [16, 'WIFI_LOSS']
];
function decodeTelemetry(data) {
  if (!Buffer.isBuffer(data) || data.length !== 5) throw Error('Telemetry must be exactly 5 binary bytes');
  const [a, b, c, d, e] = data;
  if ((d & 128) || e !== 0) throw Error('Reserved telemetry bits are not zero');
  const messageType = a >>> 5, vehicleState = (a >>> 2) & 7, line = a & 3;
  const obstacle = b >>> 6, hump = (b >>> 4) & 3;
  const faultFlags = (b & 15) | ((c & 1) << 4);
  const seq = (c >>> 1) & 127;
  if (messageType !== 0 || vehicleState > 4 || line > 2 || obstacle > 2 || hump > 2 || d > 100) {
    throw Error('Invalid telemetry enum or battery percentage');
  }
  return {
    vehicle: states[vehicleState], line: lines[line], obstacle: obstacles[obstacle],
    terrain: terrain[hump], sequence: seq,
    batteryPct: d === 0 ? null : d, // Firmware uses zero when there is no battery sensor.
    faults: faults.filter(([bit]) => (faultFlags & bit) !== 0).map(([, label]) => label),
    faultFlags
  };
}
function decodeHeartbeat(data) {
  if (!Buffer.isBuffer(data) || data.length !== 2) throw Error('Heartbeat must be 2 bytes');
  if ((data[0] >>> 5) !== 1 || (data[0] & 3) !== 0) throw Error('Invalid heartbeat header');
  const state = (data[0] >>> 2) & 7;
  if (state > 4 || data[1] > 127) throw Error('Invalid heartbeat state or sequence');
  return { vehicle: states[state], sequence: data[1] };
}
function decodeFault(data) {
  if (!Buffer.isBuffer(data) || data.length !== 1 || data[0] > 31) throw Error('Fault message must contain one 5-bit fault byte');
  return {faultFlags: data[0], faults: faults.filter(([bit]) => data[0] & bit).map(([, label]) => label)};
}
module.exports = {decodeTelemetry, decodeHeartbeat, decodeFault};
