'use strict';

const assert = require('node:assert/strict');
const {test} = require('node:test');
const {decodeTelemetry} = require('../protocol');

test('decodes the target five-byte telemetry packet', () => {
  const packet = Buffer.from([
    (1 << 2) | 1,
    (1 << 6) | (1 << 4) | 0x08,
    (12 << 1) | 1,
    76,
    0
  ]);
  assert.deepEqual(decodeTelemetry(packet), {
    vehicle: 'RUNNING',
    line: 'FOUND',
    obstacle: 'CLEAR',
    terrain: 'FLAT',
    sequence: 12,
    batteryPct: 76,
    faultFlags: 0x18,
    faults: ['NO_ECHO', 'WIFI_LOSS']
  });
});

test('reports zero battery as unavailable', () => {
  const telemetry = decodeTelemetry(Buffer.from([0, 0, 0, 0, 0]));
  assert.equal(telemetry.batteryPct, null);
});

test('rejects invalid lengths, enums, and reserved bits', () => {
  const packets = [
    Buffer.alloc(4),
    Buffer.from([0x80, 0, 0, 0, 0]),
    Buffer.from([0x14, 0, 0, 0, 0]),
    Buffer.from([0, 0xc0, 0, 0, 0]),
    Buffer.from([0, 0x30, 0, 0, 0]),
    Buffer.from([0, 0, 0, 101, 0]),
    Buffer.from([0, 0, 0, 0, 1])
  ];
  for (const packet of packets) assert.throws(() => decodeTelemetry(packet));
});
