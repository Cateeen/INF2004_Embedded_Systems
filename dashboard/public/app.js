'use strict';

const byId = id => document.getElementById(id);
const display = value => value === null || value === undefined || value === ''
  ? '—'
  : String(value);

function setText(id, value) {
  byId(id).textContent = display(value);
}

function renderEvents(events) {
  const list = byId('events');
  list.replaceChildren();
  if (events.length === 0) {
    const empty = document.createElement('p');
    empty.className = 'empty';
    empty.textContent = 'No dashboard events yet.';
    list.append(empty);
    return;
  }

  for (const event of events) {
    const row = document.createElement('div');
    row.className = 'event';
    const time = document.createElement('time');
    time.dateTime = new Date(event.at).toISOString();
    time.textContent = new Date(event.at).toLocaleTimeString([], {hour12: false});
    const kind = document.createElement('span');
    kind.className = `event-kind ${event.kind.toLowerCase()}`;
    kind.textContent = event.kind;
    const message = document.createElement('span');
    message.textContent = event.message;
    row.append(time, kind, message);
    list.append(row);
  }
}

function render(snapshot) {
  const telemetry = snapshot.telemetry;
  const connected = snapshot.telemetryConnected === true;
  const label = connected
    ? 'LIVE TELEMETRY'
    : snapshot.brokerConnected
      ? 'MQTT CONNECTED · WAITING FOR DATA'
      : snapshot.broker;

  setText('connection-label', label);
  byId('connection-dot').className = `dot ${connected ? 'live' : 'offline'}`;
  byId('stale-notice').hidden = !snapshot.stale;
  setText('broker-state', snapshot.broker);
  setText('packet-count', snapshot.packets);
  setText('invalid-count', snapshot.invalid);

  if (!telemetry) {
    for (const id of ['vehicle-state', 'line-state', 'obstacle-state',
      'terrain-state', 'sequence', 'battery', 'last-seen', 'fault-list']) {
      setText(id, null);
    }
  } else {
    setText('vehicle-state', telemetry.vehicle);
    setText('line-state', telemetry.line);
    setText('obstacle-state', telemetry.obstacle);
    setText('terrain-state', telemetry.terrain);
    setText('sequence', telemetry.sequence);
    setText('battery', telemetry.batteryPct === null
      ? 'Unavailable'
      : `${telemetry.batteryPct}%`);
    setText('last-seen', new Date(telemetry.receivedAt).toLocaleTimeString([], {
      hour12: false
    }));
    setText('fault-list', telemetry.faults.length
      ? telemetry.faults.join(', ')
      : 'No fault flags reported');
  }

  renderEvents(snapshot.events);
}

const feed = new EventSource('/api/events');
feed.onmessage = event => {
  try {
    render(JSON.parse(event.data));
  } catch (error) {
    console.error('Invalid dashboard update', error);
  }
};
feed.onerror = () => setText('connection-label', 'DASHBOARD RECONNECTING');
