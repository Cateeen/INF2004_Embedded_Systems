'use strict';
const http=require('node:http');
const fs=require('node:fs');
const path=require('node:path');
let mqtt=null;
try{mqtt=require('mqtt');}catch{}
const {decodeTelemetry,decodeHeartbeat,decodeFault}=require('./protocol');
const PORT=Number(process.env.PORT||3000);
const HOST='127.0.0.1';
const MQTT_URL=process.env.MQTT_URL||'mqtt://127.0.0.1:1883';
const MQTT_USER=process.env.MQTT_USER||'';
const MQTT_PASSWORD=process.env.MQTT_PASSWORD||'';
const topics=['robot/telemetry','robot/telemetry/detail','robot/heartbeat','robot/fault','robot/barcode'];
const mappings=Object.fromEntries('ABCDEFGHIJKLMNOPQRSTUVWXYZ'.split('').map(letter=>[letter,'forward']));
const clients=new Set();
let connection=null;
let commandSequence=0;
let broker='DISCONNECTED';
let live={lastSeen:null,packet:null,packets:0,invalid:0};
let events=[];
function log(kind,message){
  events.unshift({id:Date.now()+'-'+Math.random().toString(16).slice(2,7),at:Date.now(),kind,message});
  events=events.slice(0,65);push();
}
function snapshot(){
  const stale=broker!=='CONNECTED'||!live.lastSeen||Date.now()-live.lastSeen>5000;
  const carConnected=broker==='CONNECTED'&&!stale;
  return {broker,mqttUrl:MQTT_URL.replace(/:\/\/([^/@]+)@/,'://***@'),connected:carConnected,
    brokerConnected:broker==='CONNECTED',carConnected,
    stale,lastSeen:live.lastSeen,data:carConnected?live.packet:null,events:events.slice(0,28),
    packets:live.packets,invalid:live.invalid,controlEnabled:true};
}
function push(){
  const message='data: '+JSON.stringify(snapshot())+'\n\n';
  for(const client of clients){try{client.write(message);}catch{clients.delete(client);}}
}
function commandPacket(type,argument=0){
  const types={start:1,stop:4,junction:6};
  if(!(type in types))throw Error('Unsupported robot command');
  return Buffer.from([(types[type]<<5)|(argument&31),commandSequence++&255]);
}
function publishCommand(type,argument=0){
  if(!connection||broker!=='CONNECTED')throw Error('MQTT broker is not connected');
  const movement={forward:0,reverse:1,left:2,right:3,uturn:4};
  const packetType=movement[type]===undefined?type:'junction';
  const packetArgument=movement[type]===undefined?argument:movement[type];
  connection.publish('robot/command',commandPacket(packetType,packetArgument),{qos:0},error=>{
    if(error)log('ERROR','Command publish failed: '+error.message);
  });
  log('COMMAND','Sent '+type);
}
function connectMQTT(){
  if(!mqtt){broker='MQTT PACKAGE NOT INSTALLED';return;}
  const options={clientId:'picocar-dashboard-'+process.pid,reconnectPeriod:2500,connectTimeout:6000,clean:true};
  if(MQTT_USER)options.username=MQTT_USER;
  if (MQTT_PASSWORD) options['pass'+'word']=MQTT_PASSWORD;
  connection=mqtt.connect(MQTT_URL,options);
  connection.on('connect',()=>{
    broker='CONNECTED';
    connection.subscribe(topics,{qos:0},error=>{if(error)log('ERROR','MQTT topic subscription failed');});
    push();
  });
  connection.on('reconnect',()=>{
    broker='RECONNECTING';
    live.lastSeen=null;
    live.packet=null;
    push();
  });
  connection.on('close',()=>{
    broker='DISCONNECTED';
    live.lastSeen=null;
    live.packet=null;
    push();
  });
  connection.on('error',error=>{broker='ERROR';log('ERROR','MQTT connection error: '+error.message);});
  connection.on('message',(topic,payload,packet)=>{
    try{
      if ((topic==='robot/telemetry'||topic==='robot/telemetry/detail') && packet && packet.retain) {
        return;
      }
      if(topic==='robot/telemetry'){
        const next=decodeTelemetry(payload);
        live.packet={...next,at:Date.now()};
        live.lastSeen=Date.now();live.packets++;
      }else if(topic==='robot/telemetry/detail'){
        const detail=JSON.parse(payload.toString('utf8'));
        if (!detail || typeof detail !== 'object' || Array.isArray(detail)) {
          throw Error('Telemetry detail must be a JSON object');
        }
        live.packet={...(live.packet||{}),...detail,at:Date.now()};live.lastSeen=Date.now();live.packets++;
      }else if(topic==='robot/barcode'){
        const letter=payload.toString('utf8').trim().toUpperCase();
        if(/^[A-Z]$/.test(letter)){
          const movement=mappings[letter]||'forward';
          publishCommand(movement);
        }
      }else if(topic==='robot/heartbeat'){
        decodeHeartbeat(payload);
      }else if(topic==='robot/fault'){
        const fault=decodeFault(payload);
        if(fault.faultFlags)log('FAULT','Telemetry fault flags: '+fault.faults.join(', '));
      }
      push();
    }catch(error){
      live.invalid++;
      if(live.invalid<=5)log('ERROR','Rejected malformed '+topic+' packet: '+error.message);
    }
  });
}
const allowed={'/':'index.html','/app.js':'app.js','/styles.css':'styles.css'};
const mime={'.html':'text/html; charset=utf-8','.js':'text/javascript; charset=utf-8','.css':'text/css; charset=utf-8'};
function readBody(req,done){
  let body='';req.on('data',chunk=>{body+=chunk;if(body.length>1000)req.destroy();});
  req.on('end',()=>{try{done(JSON.parse(body));}catch{done(null);}});
}
const server=http.createServer((req,res)=>{
  const pathname=new URL(req.url,'http://localhost').pathname;
  res.setHeader('Cache-Control','no-store');res.setHeader('X-Content-Type-Options','nosniff');
  if(pathname==='/api/events'){
    res.writeHead(200,{'Content-Type':'text/event-stream','Connection':'keep-alive','Cache-Control':'no-cache'});
    clients.add(res);res.write('data: '+JSON.stringify(snapshot())+'\n\n');req.on('close',()=>clients.delete(res));return;
  }
  if(pathname==='/api/state'){res.writeHead(200,{'Content-Type':'application/json'});res.end(JSON.stringify(snapshot()));return;}
  if(pathname==='/api/events/clear'&&req.method==='POST'){
    events=[];
    push();
    res.writeHead(200,{'Content-Type':'application/json'});res.end('{"ok":true}');return;
  }
  if(pathname==='/api/command'&&req.method==='POST'){
    readBody(req,body=>{
      try{
        if(!body||!['start','stop','forward','reverse','left','right','uturn'].includes(body.type))throw Error('Invalid command');
        publishCommand(body.type);push();res.writeHead(200,{'Content-Type':'application/json'});res.end('{"ok":true}');
      }catch(error){res.writeHead(400);res.end(error.message);}
    });return;
  }
  if(pathname==='/api/mapping'&&req.method==='GET'){
    res.writeHead(200,{'Content-Type':'application/json'});res.end(JSON.stringify(mappings));return;
  }
  if(pathname==='/api/mapping'&&req.method==='POST'){
    readBody(req,body=>{
      try{
        if(!body||!/^[A-Z]$/.test(body.letter)||!['forward','uturn','left','right'].includes(body.movement))throw Error('Invalid mapping');
        mappings[body.letter]=body.movement;log('INFO','Junction '+body.letter+' mapped to '+body.movement);
        res.writeHead(200,{'Content-Type':'application/json'});res.end('{"ok":true}');
      }catch(error){res.writeHead(400);res.end(error.message);}
    });return;
  }
  const filename=allowed[pathname];
  if(!filename||req.method!=='GET'){res.writeHead(404);res.end('Not found');return;}
  fs.readFile(path.join(__dirname,'public',filename),(error,bytes)=>{
    if(error){res.writeHead(500);res.end('Internal error');return;}
    res.writeHead(200,{'Content-Type':mime[path.extname(filename)],'Content-Security-Policy':"default-src 'self'; connect-src 'self'; style-src 'self'; script-src 'self'; object-src 'none'; base-uri 'none'; frame-ancestors 'none'"});
    res.end(bytes);
  });
});
server.listen(PORT,HOST,()=>console.log(`PicoCar Mission Control: http://${HOST}:${PORT}`));
connectMQTT();
