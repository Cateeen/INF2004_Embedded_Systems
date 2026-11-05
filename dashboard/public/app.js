'use strict';
const $=id=>document.getElementById(id);
let latest=null, feed=null;
let history=[];
const charts=$('speed-chart');
const ctx=charts.getContext('2d');
const get=x=>x===undefined||x===null||x===''?'—':x;
const asNum=(x,digits=1)=>x===null||x===undefined?'—':Number(x).toFixed(digits);
const clock=t=>new Date(t||Date.now()).toLocaleTimeString('en-SG',{hour12:false});
function set(id,value){const node=$(id); if(node)node.textContent=String(value);}
function tagStatus(id,value){set(id,value);}
function statusColor(node,value){node.style.color=value==='FAULT'?'var(--red)':value==='RUNNING'?'var(--green)':'var(--ink)';}
function plot(value,reset=false){
  if(reset)history=[];
  history.push(value===null||value===undefined?null:Number(value));history=history.slice(-50);
  const w=charts.width,h=charts.height;
  ctx.clearRect(0,0,w,h);ctx.save();
  ctx.strokeStyle='#253649';ctx.lineWidth=1;
  for(let i=0;i<4;i++){let y=18+(h-36)*i/3;ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke();}
  const values=history.filter(v=>v!==null&&Number.isFinite(v));
  if(!values.length){ctx.fillStyle='#637d95';ctx.font='12px sans-serif';ctx.textAlign='center';ctx.fillText('No measured speed samples available',w/2,h/2);ctx.restore();return;}
  const max=Math.max(.4,...values)*1.12;
  ctx.beginPath();let started=false;
  history.forEach((v,i)=>{if(v===null){started=false;return;}const x=i*(w/(Math.max(1,history.length-1)));const y=h-14-v/max*(h-32);if(!started){ctx.moveTo(x,y);started=true;}else ctx.lineTo(x,y);});
  ctx.strokeStyle='#60d5f7';ctx.lineWidth=3;ctx.lineJoin='round';ctx.stroke();ctx.restore();
}
function render(s){
  latest=s;
  const isDemo=s.mode==='demo';
  const d=s.data;
  $('demo-mode').classList.toggle('selected',isDemo);$('live-mode').classList.toggle('selected',!isDemo);
  set('mode-title',isDemo?'Simulation mode':'Live MQTT telemetry');
  set('mode-desc',isDemo?'Illustrative readings · no physical robot connection':'Only measurements contained in the Pico W MQTT protocol are shown');
  $('stale-banner').classList.toggle('hidden',isDemo || !s.stale);
  const connected=isDemo?'SIMULATED':s.broker==='CONNECTED'?(s.stale?'NO RECENT DATA':'BROKER CONNECTED'):'BROKER '+s.broker;
  set('connection-text',connected);set('side-state',connected);
  $('connection-dot').className='signal '+(isDemo||s.connected?'ok':'bad');
  $('side-dot').className='signal '+(isDemo||s.connected?'ok':'bad');
  ['pause-demo','resume-demo','reset-demo'].forEach(id=>{$(id).disabled=!isDemo;$(id).style.opacity=isDemo?'1':'.4';});
  if(!d){
    for(const id of ['vehicle-status','speed','distance','safety-status','vehicle-hint','current-action','line-position','line-status','obstacle-position','obstacle-status','obstacle-distance','phase-name','graph-value','left-wheel','right-wheel','left-pid','right-pid','mqtt-status','mqtt-details','wheel-status','wheel-details','barcode-status','barcode-details','terrain-status','imu-details','obs-status','obs-details'])set(id,'—');
    set('vehicle-hint','No current telemetry');set('safety-note','Unknown — do not assume clear');
    set('speed-src','not transmitted');set('distance-src','not transmitted');set('nav-state','NO DATA');
    $('hazard').classList.add('inactive');$('progress-bar').style.width='0%';plot(null);renderLog(s.events);return;
  }
  const emergency=d.vehicle==='FAULT'||d.faultFlags>0;
  set('vehicle-status',d.vehicle);statusColor($('vehicle-status'),d.vehicle);
  set('vehicle-hint',isDemo?'Simulated operational state':'Decoded 5-byte telemetry packet');
  set('speed',asNum(d.speedMps,2));set('speed-src',isDemo?'simulated':'not transmitted');
  set('distance',asNum(d.distanceM,2));set('distance-src',isDemo?'simulated':'not transmitted');
  set('safety-status',emergency?'FAULT':d.vehicle==='FAULT'?'FAULT':'NO FAULT REPORTED');
  $('safety-status').style.color=emergency?'var(--red)':'var(--green)';
  set('safety-note',emergency?('Fault flags: '+(d.faults?.join(', ')||'unknown')):'Telemetry flags only · not physical safety assurance');
  set('current-action',isDemo?d.stage:({RUNNING:'Running — action detail unavailable',PAUSED:'Paused',FAULT:'Fault reported',IDLE:'Idle',COMPLETE:'Mission complete'})[d.vehicle]||'—');
  set('nav-state',isDemo?d.stage:d.vehicle);set('phase-name',isDemo?d.stage:'State telemetry');
  set('line-position',d.line);set('line-status',d.line);
  set('obstacle-position',d.obstacle);set('obstacle-status',d.obstacle);
  set('obstacle-distance',d.obstacleMm===null?'Not transmitted':d.obstacleMm+' mm');
  $('hazard').classList.toggle('inactive',d.obstacle!=='DETECTED');
  $('progress-bar').style.width=isDemo?((d.seq%38)/38*100).toFixed(1)+'%':'0%';
  set('graph-value',asNum(d.speedMps,2));set('left-wheel',get(d.leftRpm));set('right-wheel',get(d.rightRpm));
  set('left-pid',get(d.pidLeft));set('right-pid',get(d.pidRight));
  set('mqtt-status',isDemo?'SIMULATED':s.broker);set('mqtt-details',isDemo?'Sample packets generated locally':`Seq ${get(d.sequence)} · ${s.packets} valid packets`);
  set('wheel-status',isDemo?'ACTIVE (DEMO)':'NOT EXPOSED');set('wheel-details',isDemo?`${asNum(d.distanceM,2)} m simulated`:'Not in 5-byte MQTT packet');
  set('barcode-status',isDemo?'MARKER '+d.barcode:'NOT EXPOSED');set('barcode-details',isDemo?'Simulated IR/barcode':'Not in 5-byte MQTT packet');
  set('terrain-status',d.terrain);set('imu-details',isDemo?`${d.imuZ} m/s² Z (simulated)`:'Raw acceleration not transmitted');
  set('obs-status',d.obstacle);set('obs-details',isDemo?`${d.obstacleMm} mm simulated`:'Raw ultrasonic range not transmitted');
  if(!latest.prevSample || latest.prevSample!==d.at) plot(d.speedMps);
  renderLog(s.events);
}
function renderLog(events){
  const el=$('event-list');el.replaceChildren();
  if(!events.length){const n=document.createElement('div');n.className='emptylog';n.textContent='No recorded events yet';el.append(n);return;}
  for(const e of events){
    const r=document.createElement('div');r.className='logitem';
    const t=document.createElement('time');t.textContent=clock(e.at);
    const k=document.createElement('span');k.className='kind '+e.kind.toLowerCase();k.textContent=e.kind;
    const m=document.createElement('span');m.textContent=e.message;
    r.append(t,k,m);el.append(r);
  }
}
async function post(url,obj){try{const r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(obj)});if(!r.ok)throw Error(await r.text());}catch(e){alert('Dashboard request failed: '+e.message);}}
$('demo-mode').addEventListener('click',()=>post('/api/mode',{nextMode:'demo'}));
$('live-mode').addEventListener('click',()=>{history=[];post('/api/mode',{nextMode:'live'});});
$('pause-demo').addEventListener('click',()=>post('/api/demo',{action:'pause'}));
$('resume-demo').addEventListener('click',()=>post('/api/demo',{action:'resume'}));
$('reset-demo').addEventListener('click',()=>{history=[];post('/api/demo',{action:'reset'});});
$('download-log').addEventListener('click',()=>{
  const rows=[['Time','Type','Event'],...(latest?.events||[]).map(e=>[new Date(e.at).toISOString(),e.kind,e.message])];
  const csv=rows.map(r=>r.map(s=>'"'+String(s).replaceAll('"','""')+'"').join(',')).join('\r\n');
  const blob=new Blob([csv],{type:'text/csv'}),url=URL.createObjectURL(blob);
  const a=document.createElement('a');a.href=url;a.download='picocar-events.csv';a.click();URL.revokeObjectURL(url);
});
function openStream(){
  feed=new EventSource('/api/events');
  feed.onmessage=e=>{try{const s=JSON.parse(e.data);if(latest?.mode!==s.mode)history=[];render(s);}catch(err){console.warn('Invalid dashboard event',err);}};
  feed.onerror=()=>{set('connection-text','Dashboard reconnecting…');};
}
setInterval(()=>set('localtime',clock()),1000);set('localtime',clock());openStream();
