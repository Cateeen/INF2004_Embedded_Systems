'use strict';

const assert = require('node:assert/strict');
const {after, test} = require('node:test');
const {createDashboardServer} = require('../server');

const apps = [];
after(async () => {
  await Promise.all(apps.map(app => new Promise(resolve => {
    app.closeEventClients();
    if (!app.server.listening) return resolve();
    app.server.close(resolve);
  })));
});

async function listen(app) {
  await new Promise((resolve, reject) => {
    app.server.once('error', reject);
    app.server.listen(0, '127.0.0.1', resolve);
  });
  return `http://127.0.0.1:${app.server.address().port}`;
}

test('dashboard HTTP API is read-only and exposes no movement routes', async () => {
  const app = createDashboardServer();
  apps.push(app);
  const base = await listen(app);

  const stateResponse = await fetch(`${base}/api/state`);
  assert.equal(stateResponse.status, 200);
  const state = await stateResponse.json();
  assert.equal(Object.hasOwn(state, 'controlEnabled'), false);

  const commandResponse = await fetch(`${base}/api/command`, {method: 'POST'});
  assert.equal(commandResponse.status, 405);
  const mappingResponse = await fetch(`${base}/api/mapping`);
  assert.equal(mappingResponse.status, 404);
  const pageResponse = await fetch(base);
  assert.equal(pageResponse.status, 200);
  assert.match(await pageResponse.text(), /Read-only dashboard/);
});

test('retained telemetry is ignored and fresh packets become stale', () => {
  let now = 1000;
  const app = createDashboardServer({clock: () => now, staleAfterMs: 2000});
  apps.push(app);
  app.setBrokerState('CONNECTED');
  const packet = Buffer.from([5, 0x50, 24, 0, 0]);

  app.handleTelemetryMessage('robot/telemetry', packet, {retain: true});
  assert.equal(app.snapshot().telemetry, null);
  app.handleTelemetryMessage('robot/telemetry', packet);
  assert.equal(app.snapshot().telemetry.vehicle, 'RUNNING');
  assert.equal(app.snapshot().telemetryConnected, true);

  now += 2001;
  assert.equal(app.snapshot().stale, true);
  assert.equal(app.snapshot().telemetry, null);
});

test('malformed telemetry is counted without exposing packet contents', () => {
  const app = createDashboardServer();
  app.handleTelemetryMessage('robot/telemetry', Buffer.from([1]));
  const state = app.snapshot();
  assert.equal(state.invalid, 1);
  assert.equal(state.events[0].message, 'Rejected malformed telemetry packet');
});
