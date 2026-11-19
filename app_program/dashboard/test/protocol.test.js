const {test} = require('node:test');
const assert = require('node:assert/strict');
const {decodeTelemetry, decodeHeartbeat, decodeFault} = require('../protocol');
test('real firmware 5-byte packet fields decode correctly', () => {
  const bytes = Buffer.from([(0<<5) | (1<<2) | 1, (2<<6)|(1<<4)|8, (12<<1), 0, 0]);
  const x=decodeTelemetry(bytes);
  assert.equal(x.vehicle,'RUNNING'); assert.equal(x.line,'FOUND');
  assert.equal(x.obstacle,'DETECTED'); assert.equal(x.terrain,'FLAT');
  assert.equal(x.sequence,12); assert.deepEqual(x.faults,['NO_ECHO']);
  assert.equal(x.batteryPct,null);
});
test('fifth fault bit is unpacked from byte 2',()=>{
  const x=decodeTelemetry(Buffer.from([0,0,1,76,0]));
  assert.deepEqual(x.faults,['WIFI_LOSS']); assert.equal(x.batteryPct,76);
});
test('invalid packets do not pass validation',()=>{
  for (const input of [Buffer.alloc(4),Buffer.from([0,0,0,0,1]),Buffer.from([0,0,0,128,0]),Buffer.from([0,0,0,101,0])])
    assert.throws(()=>decodeTelemetry(input));
});
test('heartbeats and separate fault topic',()=>{
  assert.deepEqual(decodeHeartbeat(Buffer.from([(1<<5)|(2<<2),9])),{vehicle:'PAUSED',sequence:9});
  assert.deepEqual(decodeFault(Buffer.from([9])),{faultFlags:9,faults:['MOTOR_STALL','NO_ECHO']});
});
