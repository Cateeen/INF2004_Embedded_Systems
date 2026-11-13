'use strict';

const fs = require('node:fs');
const http = require('node:http');
const path = require('node:path');
const {decodeTelemetry} = require('./protocol');

const HOST = '127.0.0.1';
const TELEMETRY_TOPIC = 'robot/telemetry';
const DEFAULT_STALE_AFTER_MS = 5000;
const publicFiles = {
  '/': ['index.html', 'text/html; charset=utf-8'],
  '/app.js': ['app.js', 'text/javascript; charset=utf-8'],
  '/styles.css': ['styles.css', 'text/css; charset=utf-8']
};
const contentSecurityPolicy = [
  "default-src 'self'",
  "connect-src 'self'",
  "style-src 'self'",
  "script-src 'self'",
  "img-src 'self' data:",
  "object-src 'none'",
  "base-uri 'none'",
  "frame-ancestors 'none'"
].join('; ');

function createDashboardServer(options = {}) {
  const clock = options.clock || Date.now;
  const staleAfterMs = options.staleAfterMs === undefined
    ? DEFAULT_STALE_AFTER_MS
    : options.staleAfterMs;
  if (!Number.isInteger(staleAfterMs) || staleAfterMs <= 0) {
    throw new RangeError('staleAfterMs must be a positive integer');
  }

  const clients = new Set();
  const events = [];
  let eventSequence = 0;
  let broker = 'CONNECTING';
  let lastSeen = null;
  let telemetry = null;
  let packets = 0;
  let invalid = 0;

  function snapshot() {
    const now = clock();
    const age = lastSeen === null ? null : now - lastSeen;
    const stale = broker !== 'CONNECTED' || age === null ||
      age < 0 || age > staleAfterMs;
    return {
      broker,
      brokerConnected: broker === 'CONNECTED',
      telemetryConnected: !stale,
      stale,
      lastSeen,
      telemetry: stale ? null : telemetry,
      packets,
      invalid,
      events: events.slice(0, 30)
    };
  }

  function push() {
    const data = `data: ${JSON.stringify(snapshot())}\n\n`;
    for (const client of clients) {
      if (client.destroyed) {
        clients.delete(client);
        continue;
      }
      if (!client.write(data)) {
        client.destroy();
        clients.delete(client);
      }
    }
  }

  function record(kind, message) {
    events.unshift({id: ++eventSequence, at: clock(), kind, message});
    if (events.length > 50) events.length = 50;
    push();
  }

  function setBrokerState(next) {
    if (broker === next) return;
    broker = next;
    if (broker !== 'CONNECTED') {
      lastSeen = null;
      telemetry = null;
    }
    record('INFO', `MQTT status: ${broker}`);
  }

  function handleTelemetryMessage(topic, payload, packet = {}) {
    if (topic !== TELEMETRY_TOPIC || packet.retain) return;
    let decoded;
    try {
      decoded = decodeTelemetry(payload);
    } catch {
      invalid++;
      if (invalid <= 5) record('ERROR', 'Rejected malformed telemetry packet');
      else push();
      return;
    }
    telemetry = {...decoded, receivedAt: clock()};
    lastSeen = telemetry.receivedAt;
    packets++;
    push();
  }

  const server = http.createServer((req, res) => {
    res.setHeader('Cache-Control', 'no-store');
    res.setHeader('X-Content-Type-Options', 'nosniff');
    res.setHeader('X-Frame-Options', 'DENY');
    res.setHeader('Referrer-Policy', 'no-referrer');
    res.setHeader('Content-Security-Policy', contentSecurityPolicy);

    if (req.method !== 'GET') {
      res.writeHead(405, {'Allow': 'GET'});
      res.end('Method not allowed');
      return;
    }

    let pathname;
    try {
      pathname = new URL(req.url, `http://${HOST}`).pathname;
    } catch {
      res.writeHead(400);
      res.end('Bad request');
      return;
    }

    if (pathname === '/api/state') {
      res.writeHead(200, {'Content-Type': 'application/json; charset=utf-8'});
      res.end(JSON.stringify(snapshot()));
      return;
    }

    if (pathname === '/api/events') {
      res.writeHead(200, {
        'Content-Type': 'text/event-stream; charset=utf-8',
        'Connection': 'keep-alive',
        'Cache-Control': 'no-cache'
      });
      clients.add(res);
      res.write(`data: ${JSON.stringify(snapshot())}\n\n`);
      res.on('close', () => clients.delete(res));
      return;
    }

    const file = publicFiles[pathname];
    if (!file) {
      res.writeHead(404);
      res.end('Not found');
      return;
    }

    fs.readFile(path.join(__dirname, 'public', file[0]), (error, contents) => {
      if (error) {
        console.error('Dashboard static-file read failed');
        if (!res.headersSent) res.writeHead(500);
        res.end('Internal server error');
        return;
      }
      res.writeHead(200, {'Content-Type': file[1]});
      res.end(contents);
    });
  });

  return {
    server,
    snapshot,
    setBrokerState,
    handleTelemetryMessage,
    closeEventClients() {
      for (const client of clients) client.end();
      clients.clear();
    }
  };
}

function startDashboard() {
  const mqtt = require('mqtt');
  const mqttUrl = process.env.MQTT_URL || 'mqtt://127.0.0.1:1883';
  let parsedUrl;
  try {
    parsedUrl = new URL(mqttUrl);
  } catch {
    throw new Error('MQTT_URL must be a valid mqtt URL');
  }
  if (parsedUrl.protocol !== 'mqtt:' && parsedUrl.protocol !== 'mqtts:') {
    throw new Error('MQTT_URL must use mqtt:// or mqtts://');
  }

  const port = Number(process.env.PORT || 3000);
  if (!Number.isInteger(port) || port < 1 || port > 65535) {
    throw new RangeError('PORT must be an integer from 1 to 65535');
  }

  const app = createDashboardServer();
  const options = {
    clientId: `picocar-readonly-${process.pid}`,
    reconnectPeriod: 2500,
    connectTimeout: 6000,
    clean: true
  };
  if (process.env.MQTT_USER) options.username = process.env.MQTT_USER;
  if (process.env.MQTT_PASSWORD) options.password = process.env.MQTT_PASSWORD;

  const connection = mqtt.connect(mqttUrl, options);
  connection.on('connect', () => {
    app.setBrokerState('CONNECTED');
    connection.subscribe(TELEMETRY_TOPIC, {qos: 0}, error => {
      if (error) {
        app.setBrokerState('SUBSCRIBE_ERROR');
        console.error('Dashboard MQTT subscription failed');
      }
    });
  });
  connection.on('reconnect', () => app.setBrokerState('RECONNECTING'));
  connection.on('offline', () => app.setBrokerState('OFFLINE'));
  connection.on('close', () => app.setBrokerState('DISCONNECTED'));
  connection.on('error', () => {
    app.setBrokerState('ERROR');
    console.error('Dashboard MQTT connection error');
  });
  connection.on('message', app.handleTelemetryMessage);

  app.server.on('error', error => {
    console.error(`Dashboard HTTP server error: ${error.message}`);
    connection.end(true);
    process.exitCode = 1;
  });
  app.server.listen(port, HOST, () => {
    console.log(`Read-only PicoCar dashboard: http://${HOST}:${port}`);
  });

  function shutdown() {
    app.setBrokerState('DISCONNECTED');
    app.closeEventClients();
    connection.end(true);
    app.server.close();
  }
  process.once('SIGINT', shutdown);
  process.once('SIGTERM', shutdown);
}

if (require.main === module) startDashboard();

module.exports = {createDashboardServer};
