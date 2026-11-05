'use strict';
const http = require('node:http');
const fs = require('node:fs');
const path = require('node:path');
let mqtt = null;
try { mqtt = require('mqtt'); } catch { /* Demo mode works without npm install. */ }
const {decodeTelemetry,decodeHeartbeat,decodeFault} = require('./protocol');
const PORT = Number(process.env.PORT || 3000);
const HOST = '127.0.0.1'; // Local-only dashboard; never expose to the Internet by default.
const MQTT_URL = process.env.MQTT_URL || 'mqtt://127.0.0.1:1883';
const MQTT_USER = process.env.MQTT_USER || '';
const MQTT_PASSWORD = process.env.MQTT_PASSWORD || '';
const names = ['robot/telemetry','robot/heartbeat','robot/fault'];
const clients = new Set();
let mode='demo'; // Keep real telemetry opt-in; no commands go to MQTT under any circumstances.
let broker='DISCONNECTED';
let live={lastSeen:null,packet:null,lastHeartbeat:null,lastFault:null,packets:0,invalid:0};
let simulator={playing:true,tick:0,events:[],distance:0};
let demoData=null;
let events=[];
function log(kind,message) {
  const item={id:Date.now()+'-'+Math.random().toString(16).slice(2,7),at:Date.now(),kind,message};
  events.unshift(item); events=events.slice(0,65); push();
}
function makeDemo() {
  const t=simulator.tick;
  const cycle=t%38;
  const stage=cycle < 14?'FOLLOWING':cycle < 18?'OBSTACLE DETECTED':cycle < 24?'SCANNING':cycle < 31?'BYPASSING':'RECOVERING';
  const motorActive=simulator.playing && ['FOLLOWING','BYPASSING','RECOVERING'].includes(stage);
  const left=motorActive ? Math.round(140+17*Math.sin(t/3)):0;
  const right=motorActive ? Math.round(137+18*Math.cos(t/4)):0;
  simulator.distance += motorActive ? 0.026:0;
  return {
    at:Date.now(), vehicle:simulator.playing?'RUNNING':'PAUSED',
    line:stage==='RECOVERING'?'LOST':'FOUND',
    obstacle:['OBSTACLE DETECTED','SCANNING','BYPASSING'].includes(stage)?'DETECTED':'CLEAR',
    terrain:cycle>=26&&cycle<30?'HUMP':'FLAT', faultFlags:0,faults:[], batteryPct:85,
    seq:t%128, stage:simulator.playing?stage:'PAUSED',
    leftRpm:left,rightRpm:right, distanceM:simulator.distance,
    obstacleMm:stage==='OBSTACLE DETECTED'?220:stage==='SCANNING'?290:stage==='BYPASSING'?370:610,
    imuX:(0.08*Math.sin(t/3)).toFixed(2), imuY:(0.04*Math.cos(t/4)).toFixed(2),
    imuZ:(9.81+(cycle>=26&&cycle<30?2.5:0)).toFixed(2),
    barcode:cycle>33?'03':'—',
    speedMps:motorActive?Number((0.25+Math.sin(t/3)*0.02).toFixed(2)):0,
    pidLeft:motorActive?32:0,pidRight:motorActive?30:0
  };
}
function snapshot() {
  const active = mode==='demo' ? demoData : live.packet;
  const stale=mode==='live' && (!live.lastSeen || Date.now()-live.lastSeen>5000);
  return {mode, broker, mqttUrl:MQTT_URL.replace(/:\/\/([^/@]+)@/, '://***@'),
    connected:mode==='demo'?true:broker==='CONNECTED' && !stale,
    stale, lastSeen:mode==='demo'?(demoData?.at||null):live.lastSeen,
    data:stale?null:active, events:events.slice(0,28), packets:live.packets, invalid:live.invalid,
    simulationPlaying:simulator.playing, controlEnabled:false};
}
function push() {
  const msg='data: '+JSON.stringify(snapshot())+'\n\n';
  for(const client of clients){try{client.write(msg);}catch{clients.delete(client);}}
}
function recordDemoStage() {
  const sequence={0:'Line detected — autonomous following',14:'Obstacle ahead — motor stop requested',18:'Obstacle scan started',24:'Bypass path selected: LEFT',31:'Searching for line reacquisition',37:'Line recovered — autonomous navigation'};
  const message=sequence[simulator.tick%38]; if(message)log('DEMO',message);
}
setInterval(()=>{
  if(mode==='demo'){
    if(simulator.playing){simulator.tick++;recordDemoStage();}
    demoData=makeDemo();
  }
  push();
},1000);
demoData=makeDemo();
log('INFO','Dashboard started in SIMULATION mode — all values are illustrative');
function connectMQTT() {
  if (!mqtt) { broker='MQTT PACKAGE NOT INSTALLED'; console.log('MQTT package missing: Demo works; run npm install to enable Live MQTT.'); return; }
  const options={clientId:'picocar-dashboard-'+process.pid, reconnectPeriod:2500, connectTimeout:6000, clean:true};
  if(MQTT_USER) options.username=MQTT_USER;
  if(MQTT_PASSWORD) options.password=MQTT_PASSWORD;
  const conn=mqtt.connect(MQTT_URL, options);
  conn.on('connect',()=>{
    broker='CONNECTED'; log('NETWORK','MQTT broker connected');
    conn.subscribe(names,{qos:0},(err)=>{if(err)log('ERROR','MQTT topic subscription failed');});
    push();
  });
  conn.on('reconnect',()=>{broker='RECONNECTING';push();});
  conn.on('close',()=>{if(broker!=='DISCONNECTED'){broker='DISCONNECTED';push();}});
  conn.on('error',(e)=>{broker='ERROR'; if(mode==='live')log('ERROR','MQTT connection error: '+e.message);});
  conn.on('message',(topic,payload)=>{
    try{
      if(topic==='robot/telemetry'){
        const next=decodeTelemetry(payload);
        const old=live.packet;
        live.packet={...next,at:Date.now(),stage:null,
          leftRpm:null,rightRpm:null,distanceM:null,obstacleMm:null,
          imuX:null,imuY:null,imuZ:null,barcode:null,speedMps:null,pidLeft:null,pidRight:null};
        live.lastSeen=Date.now();live.packets++;
        if(!old || next.vehicle!==old.vehicle || next.obstacle!==old.obstacle || next.line!==old.line || next.faultFlags!==old.faultFlags)
          log('ROBOT',`State: ${next.vehicle} · Line: ${next.line} · Obstacle: ${next.obstacle}`);
      } else if(topic==='robot/heartbeat'){
        live.lastHeartbeat={...decodeHeartbeat(payload),at:Date.now()};
      }else if(topic==='robot/fault'){
        const f=decodeFault(payload);live.lastFault={...f,at:Date.now()};
        if(f.faultFlags)log('FAULT','Telemetry fault flags: '+f.faults.join(', '));
      }
      push();
    }catch(e){live.invalid++;if(live.invalid<=5)log('ERROR','Rejected malformed '+topic+' packet: '+e.message);}
  });
}
const allowed={'/':'index.html','/app.js':'app.js','/styles.css':'styles.css'};
const mime={'.html':'text/html; charset=utf-8','.js':'text/javascript; charset=utf-8','.css':'text/css; charset=utf-8'};
const server=http.createServer((req,res)=>{
  const pathname=new URL(req.url,'http://localhost').pathname;
  res.setHeader('Cache-Control','no-store');
  res.setHeader('X-Content-Type-Options','nosniff');
  if(pathname==='/api/events'){
    res.writeHead(200,{'Content-Type':'text/event-stream','Connection':'keep-alive','Cache-Control':'no-cache'});
    clients.add(res);res.write('data: '+JSON.stringify(snapshot())+'\n\n');req.on('close',()=>clients.delete(res));return;
  }
  if(pathname==='/api/state'){
    res.writeHead(200,{'Content-Type':'application/json'});res.end(JSON.stringify(snapshot()));return;
  }
  if(pathname==='/api/mode' && req.method==='POST'){
    let body='';req.on('data',c=>{body+=c; if(body.length>1000)req.destroy();});req.on('end',()=>{
      try{
        const {nextMode}=JSON.parse(body);
        if(nextMode!=='demo'&&nextMode!=='live')throw Error('Invalid mode');
        mode=nextMode;log('INFO',nextMode==='demo'?'Simulation mode selected':'Live MQTT mode selected; real values only');
        res.writeHead(200,{'Content-Type':'application/json'});res.end(JSON.stringify({ok:true}));
      }catch{res.writeHead(400);res.end('Invalid mode');}
    });return;
  }
  if(pathname==='/api/demo' && req.method==='POST'){
    let body='';req.on('data',c=>{body+=c;if(body.length>1000)req.destroy();});req.on('end',()=>{
      try{
        if(mode!=='demo')throw Error('Demo actions unavailable in live mode');
        const action=JSON.parse(body).action;
        if(action==='pause'){simulator.playing=false;log('DEMO','Simulation paused — no robot command sent');}
        else if(action==='resume'){simulator.playing=true;log('DEMO','Simulation resumed — no robot command sent');}
        else if(action==='reset'){simulator.playing=true;simulator.tick=0;simulator.distance=0;log('DEMO','Simulation reset — no robot command sent');}
        else throw Error('Unknown action');
        demoData=makeDemo();push();res.writeHead(200,{'Content-Type':'application/json'});res.end(JSON.stringify({ok:true}));
      }catch(e){res.writeHead(400);res.end(e.message);}
    });return;
  }
  const filename=allowed[pathname];
  if(!filename || req.method!=='GET'){res.writeHead(404);res.end('Not found');return;}
  const fp=path.join(__dirname,'public',filename);
  fs.readFile(fp,(err,bytes)=>{
    if(err){res.writeHead(500);res.end('Internal error');return;}
    res.writeHead(200,{'Content-Type':mime[path.extname(fp)],'Content-Security-Policy':"default-src 'self'; connect-src 'self'; style-src 'self'; script-src 'self'; img-src 'self' data:; object-src 'none'; base-uri 'none'; frame-ancestors 'none'"});res.end(bytes);
  });
});
server.listen(PORT,HOST,()=>{
  console.log(`\nPicoCar Mission Control: http://${HOST}:${PORT}`);
  console.log('Starts in DEMO mode. Switch to LIVE in the dashboard.');
  console.log('MQTT read-only subscriptions: '+names.join(', '));
  console.log('NO remote movement commands are published.\n');
});
connectMQTT();
