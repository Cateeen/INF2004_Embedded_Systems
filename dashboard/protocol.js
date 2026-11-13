'use strict';

const vehicleStates = ['IDLE', 'RUNNING', 'PAUSED', 'FAULT', 'COMPLETE'];
const lineStates = ['UNKNOWN', 'FOUND', 'LOST'];
const obstacleStates = ['UNKNOWN', 'CLEAR', 'DETECTED'];
const terrainStates = ['UNKNOWN', 'FLAT', 'HUMP'];
const faultNames = [
  [0x01, 'MOTOR_STALL'],
  [0x02, 'ENCODER'],
  [0x04, 'IMU'],
  [0x08, 'NO_ECHO'],
  [0x10, 'WIFI_LOSS']
];

function decodeTelemetry(packet) {
  if (!Buffer.isBuffer(packet) || packet.length !== 5) {
    throw new Error('Telemetry must be exactly five binary bytes');
  }

  const [header, obstacleTerrain, sequenceFaults, battery, reserved] = packet;
  const messageType = header >>> 5;
  const vehicle = (header >>> 2) & 0x07;
  const line = header & 0x03;
  const obstacle = obstacleTerrain >>> 6;
  const terrain = (obstacleTerrain >>> 4) & 0x03;
  const faultFlags = (obstacleTerrain & 0x0f) | ((sequenceFaults & 0x01) << 4);
  const sequence = (sequenceFaults >>> 1) & 0x7f;

  if (messageType !== 0 || vehicle >= vehicleStates.length ||
      line >= lineStates.length || obstacle >= obstacleStates.length ||
      terrain >= terrainStates.length || battery > 100 || reserved !== 0) {
    throw new Error('Telemetry contains invalid or reserved values');
  }

  return {
    vehicle: vehicleStates[vehicle],
    line: lineStates[line],
    obstacle: obstacleStates[obstacle],
    terrain: terrainStates[terrain],
    sequence,
    batteryPct: battery === 0 ? null : battery,
    faultFlags,
    faults: faultNames.filter(([bit]) => (faultFlags & bit) !== 0)
      .map(([, name]) => name)
  };
}

module.exports = {decodeTelemetry};
