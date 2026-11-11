'use strict';
const $=id=>document.getElementById(id);
let latest=null;
const get=x=>x===undefined||x===null||x===''?'—':x;
const asNum=(x,digits=1)=>x===null||x===undefined?'—':Number(x).toFixed(digits);
const clock=t=>new Date(t||Date.now()).toLocaleTimeString('en-SG',{hour12:false});
function set(id,value){const node=$(id);if(node)node.textContent=String(value);}
function statusColor(node,value){node.style.color=value==='Safe stop'?'var(--red)':value==='Line following'?'var(--green)':'var(--ink)';}
function currentState(d){
  if(d.vehicle==='FAULT'||d.faultFlags>0)return 'Safe stop';
  if(d.obstacle==='DETECTED')return 'Obstacle avoidance';
  if(d.terrain==='HUMP')return 'Terrain event';
  if(d.line==='FOUND'&&d.vehicle==='RUNNING')return 'Line following';
  return 'Idle';
}
function render(s){
  latest=s;
  const d=s.data;
  const connected=s.carConnected===true;
  const connectionLabel=connected?'CAR CONNECTED':s.brokerConnected?'MQTT CONNECTED · NO CAR':'NO MQTT CONNECTION';
  set('connection-text',connectionLabel);
  set('side-state',connectionLabel);
  $('connection-dot').className='signal '+(connected?'ok':'bad');
  $('side-dot').className='signal '+(connected?'ok':'bad');
  $('stale-banner').classList.toggle('hidden',!s.stale);
  ['robot-start','robot-stop','forward','reverse','left','right','uturn','save-mapping']
    .forEach(id=>{const node=$(id);if(node)node.disabled=!connected;});
  if(!d){
    ['vehicle-status','speed','distance','speed-src','distance-src','safety-status','vehicle-hint','mqtt-status','mqtt-details','ir-status','ir-details','barcode-status','barcode-details','terrain-status','imu-details','obs-status','obs-details'].forEach(id=>set(id,'—'));
    set('vehicle-hint','No current telemetry');set('safety-note','Unknown — do not assume clear');
    return;
  }
  const state=currentState(d);
  set('vehicle-status',state);statusColor($('vehicle-status'),state);
  set('vehicle-hint','Live robot telemetry');
  set('speed',asNum(d.speedMps,2));set('speed-src',d.speedMps==null?'not transmitted':'live telemetry');
  set('distance',asNum(d.distanceM,2));set('distance-src',d.distanceM==null?'not transmitted':'live telemetry');
  const emergency=d.vehicle==='FAULT'||d.faultFlags>0;
  set('safety-status',emergency?'FAULT':'NO FAULT REPORTED');
  $('safety-status').style.color=emergency?'var(--red)':'var(--green)';
  set('safety-note',emergency?'Fault flags: '+(d.faults?.join(', ')||'unknown'):'Telemetry flags only · not physical safety assurance');
  set('mqtt-status',s.broker);set('mqtt-details',`Seq ${get(d.sequence)} · ${s.packets} valid packets`);
  set('ir-status',d.ir?`${d.ir.barcode}/${d.ir.center}/${d.ir.right}`:'NOT EXPOSED');
  set('ir-details','barcode / centre / right');
  set('barcode-status',d.barcode||'—');set('barcode-details','Latest decoded character');
  set('terrain-status',d.terrain);set('imu-details',d.humpHeightCm==null?'Height unavailable':`${d.humpHeightCm} cm`);
  set('obs-status',d.obstacle);set('obs-details',d.obstacleMm==null?'Distance unavailable':`${d.obstacleMm} mm`);
  renderLog(s.events);
}
function renderLog(events){
  const el=$('event-list');el.replaceChildren();
  if(!events.length){const n=document.createElement('div');n.className='emptylog';n.textContent='No recorded events yet';el.append(n);return;}
  for(const e of events){
    const r=document.createElement('div');r.className='logitem';
    const t=document.createElement('time');t.textContent=clock(e.at);
    const k=document.createElement('span');k.className='kind '+e.kind.toLowerCase();k.textContent=e.kind;
    const m=document.createElement('span');m.textContent=e.message;r.append(t,k,m);el.append(r);
  }
}
async function post(url,obj){
  try{const r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(obj)});if(!r.ok)throw Error(await r.text());}
  catch(e){alert('Dashboard request failed: '+e.message);}
}
$('robot-start').addEventListener('click',()=>post('/api/command',{type:'start'}));
$('robot-stop').addEventListener('click',()=>post('/api/command',{type:'stop'}));
document.querySelectorAll('[data-command]').forEach(button=>button.addEventListener('click',()=>post('/api/command',{type:button.dataset.command})));
const letters='ABCDEFGHIJKLMNOPQRSTUVWXYZ';
for(const letter of letters){const option=document.createElement('option');option.value=letter;option.textContent=letter;$('mapping-letter').append(option);}
async function loadMapping(){try{const r=await fetch('/api/mapping');const mappings=await r.json();$('mapping-movement').value=mappings[$('mapping-letter').value]||'forward';}catch(e){console.warn('Mapping load failed',e);}}
$('mapping-letter').addEventListener('change',loadMapping);
$('save-mapping').addEventListener('click',()=>post('/api/mapping',{letter:$('mapping-letter').value,movement:$('mapping-movement').value}));
loadMapping();
$('clear-log').addEventListener('click',async()=>{
  if(!confirm('Clear the event timeline?'))return;
  await post('/api/events/clear',{});
});
$('download-log').addEventListener('click',()=>{
  const rows=[['Time','Type','Event'],...(latest?.events||[]).map(e=>[new Date(e.at).toISOString(),e.kind,e.message])];
  const csv=rows.map(r=>r.map(s=>'"'+String(s).replaceAll('"','""')+'"').join(',')).join('\r\n');
  const blob=new Blob([csv],{type:'text/csv'}),url=URL.createObjectURL(blob),a=document.createElement('a');
  a.href=url;a.download='picocar-events.csv';a.click();URL.revokeObjectURL(url);
});
setInterval(()=>set('localtime',clock()),1000);set('localtime',clock());
const feed=new EventSource('/api/events');
feed.onmessage=e=>{try{render(JSON.parse(e.data));}catch(err){console.warn('Invalid dashboard event',err);}};
feed.onerror=()=>set('connection-text','Dashboard reconnecting…');
