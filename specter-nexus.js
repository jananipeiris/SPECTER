/* NEXUS browser integration. Existing UI/firmware remain authoritative. */
'use strict';
const NK=SpecterKinematics;
let homingSpanReady=false;
let configReceived=false, firmwareIdentity="unreported", repairLockSource="unreported";
let baseReferenceConfirmed=false, configRequestCount=0, nextConfigRequestAt=0;
window.__nexusNewSession=()=>{resetSwitchMapping();C.repairLock=undefined;homingSpanReady=false;configReceived=false;firmwareIdentity="unreported";repairLockSource="unreported";configRequestCount=0;nextConfigRequestAt=Date.now()+1000;lastNexusTel=0;baseReferenceConfirmed=false;nexusPreview=null;gt=null;nexusRun++};
let measured=null,nexusPreview=null,nexusController=null,nexusRun=0,lastNexusTel=0;
try{const p=LS('specterMeasuredV1',null);if(p)measured=NK.validate(p)}catch(e){log('Calibration rejected: '+e.message)}
function homedAngularScale(k){
 if(k!=='s'&&k!=='e')throw Error('Base angle scale must be configured separately; homing has no base angle sensor.');
 if(!S.hm)throw Error('Auto-home the arm first.');
 if(!homingSpanReady)throw Error('Waiting for the homed travel spans from ESP32.');
 const total=k==='s'?C.shTot:C.elbTot,range=k==='s'?T.shRange:T.elbRange;
 if(!Number.isFinite(total)||total<=40||!Number.isFinite(range)||range<=0)
  throw Error('Invalid homed step span or configured angle range.');
 return {total,range};
}
function calibratedAngle(k,pulse){
 if(measured)return NK.angle(measured,k,pulse);
 if(live){
  try{const {total,range}=homedAngularScale(k);return pulse/total*range}catch{return NaN}
 }
 return k==='b'?pulse*T.baseDeg:k==='s'?pulse/C.shTot*T.shRange:pulse/C.elbTot*T.elbRange;
}
function calibratedSteps(k,deg){
 if(!Number.isFinite(deg))throw Error('A finite target angle is required.');
 if(measured)return NK.steps(measured,k,deg);
 if(live){
  const {total,range}=homedAngularScale(k);
  // Firmware reserves 20 pulses at each travel end.
  return Math.max(20,Math.min(total-20,Math.round(deg/range*total)));
 }
 return Math.round(k==='b'?deg/T.baseDeg:k==='s'?deg/T.shRange*C.shTot:deg/T.elbRange*C.elbTot);
}
// Native firmware commands are already in pulses; they do not need angular calibration.
function nexusCheckPulsePose(q){
 if(measured)return NK.checkSteps(measured,q,C);
 if(!Number.isSafeInteger(q.b)||Math.abs(q.b)>2147483647)throw Error('Invalid base pulse target');
 for(const [k,total] of [['s',C.shTot],['e',C.elbTot]]){
  if(!Number.isSafeInteger(q[k])||!Number.isFinite(total)||total<=40||q[k]<0||q[k]>total)
   throw Error('Pulse target outside firmware travel span');
 }
 return q;
}
window.__nexusCheckSteps=nexusCheckPulsePose;
function nexusReason(command){
 if(!live||/^(GET|HOME_DIAG|MOTOR_DIAG|SW_DIAG|STOP|ESTOP|J:0:0)$/.test(command))return '';
 if(C.repairLock===1)return 'Firmware repair lock is ON. Repair the elbow upper limit switch and verify wiring/DIR. This lock cannot be cleared from the UI.';
 if(C.repairLock!==0)return configReceived?
  'ESP32 configuration received, but this firmware does not report repairLock. Upload the matching project firmware; no unlocked state will be assumed.':
  'Waiting for ESP32 configuration. Read firmware configuration / homing diagnostics; check the uploaded firmware and 460800 baud.';
 if(Date.now()-lastNexusTel>1000)return 'Motion blocked: USB telemetry is stale.';
 if(S.lf||!S.lim)return 'Motion blocked: inspect limit protection and switch fault.';
 if(command.startsWith('J:')&&measured){
  const v=command.split(':').map(Number),k=({1:'b',2:'s',3:'e'})[v[1]];
  if(k){const a=NK.angle(measured,k,S[k]+Math.sign(v[2])*20),[lo,hi]=measured.joints[k].limits;
   if(a<lo||a>hi)return 'Jog would exceed the measured joint limit.'}
 }
 if(/^(J:|HOME$|PWR$|CFG:|FLIP:|SPD:|LIM$)/.test(command))return '';
 try{
  if(command.startsWith('GOTO:')){
   const v=command.slice(5).split(',').map(Number);
   if(v.length!==6||!v.every(Number.isFinite)||!v.every(Number.isInteger))throw Error('GOTO requires six integer values');
   nexusCheckPulsePose({b:v[0],s:v[1],e:v[2]});
   if(v[3]<0||v[3]>180||v[4]<20||v[4]>160||v[5]<650||v[5]>2500)throw Error('GOTO servo or speed out of range');
  }
  if(command==='GO_HOME')nexusCheckPulsePose({b:C.hB,s:C.hS,e:C.hE});
  if(command.startsWith('WP_RUN')||command.startsWith('WP_GO:')){
   const poses=command.startsWith('WP_GO:')?[W[+command.slice(6)]]:W;
   for(const w of poses){if(!w)throw Error('Missing waypoint');nexusCheckPulsePose({b:w[0],s:w[1],e:w[2]})}
  }
 }catch(e){return e.message}
 return '';
}
function nexusApply(p){
 NK.validate(p);if(S.job||seqBusy||homingNow()||curJog)throw Error('Stop motion before changing calibration');
 measured=structuredClone(p);localStorage.setItem('specterMeasuredV1',JSON.stringify(measured));
 for(const k of ['b','s','e'])LIM[k]=[...measured.joints[k].limits];
 const sb=NK.fit(p.joints.s.points),eb=NK.fit(p.joints.e.points);
 T.baseDeg=NK.fit(p.joints.b.points).scale;T.shMin=0;T.shRange=sb.scale*C.shTot;T.elbRange=eb.scale*C.elbTot;
 arm.setGeometry(p.upperMm/40,p.foreMm/40,p.shoulderHeightMm/40);gh.setGeometry(p.upperMm/40,p.foreMm/40,p.shoulderHeightMm/40);
 T.L1=p.upperMm/40;T.L2=p.foreMm/40;
 log('Measured calibration saved. Base zero must be re-established at every ESP32 boot.');
}
window.__nexusAngle=calibratedAngle;window.__nexusSteps=calibratedSteps;window.__nexusMeasured=()=>measured;
const legacyRx=rx;
rx=function(line){
 let o;try{o=JSON.parse(String(line).replace(/^\uFEFF/,'').trim())}catch{return}
 if(!o||typeof o!=='object')return;
 if(o.t==='tel'&&!['b','s','e','w','g','en','lim','hm','job','ri','rn','sw','ph'].every(k=>Number.isFinite(o[k])))return;
 const wasHomed=!!S.hm;
 if(o.t==='cfg'){
  if(S.hm&&Number.isInteger(o.shTot)&&o.shTot>40&&Number.isInteger(o.elbTot)&&o.elbTot>40)homingSpanReady=true;
  configReceived=true;
  if(typeof o.firmware==='string')firmwareIdentity=o.firmware;
 }
 const diagnostic=o.t==='st'&&typeof o.m==='string'?o.m.match(/^HOME DIAG repairLock=([01])(?: |$)/):null;
 const lock=(o.repairLock===0||o.repairLock===false)?0:(o.repairLock===1||o.repairLock===true)?1:null;
 if(lock!==null){C.repairLock=lock;repairLockSource=o.t}
 else if(diagnostic){C.repairLock=Number(diagnostic[1]);repairLockSource='HOME_DIAG'}
 // Normalize before legacy cfg assignment; an explicit malformed lock must fail closed.
 if(o.t==='cfg'&&Object.hasOwn(o,'repairLock')){o.repairLock=lock===null?undefined:lock;C.repairLock=o.repairLock}
 legacyRx(JSON.stringify(o));
 if(o.t==='tel'){
  lastNexusTel=Date.now();S.rawSw=o.rawSw;S.lf=o.lf;S.fl=o.fl;
  if(!o.hm)homingSpanReady=false;
  else if(!wasHomed){homingSpanReady=false;if(live)cmd('GET')}
 }
 if(o.t==='sw_diag'){
  const text=switchElectricalText(o)+'. Active levels SH='+o.trig_sh+' ELB='+o.trig_el+'; repairLock='+C.repairLock+'.';
  $('swDiagResult').textContent=text;log(text);
 }
};
const legacyCmd=cmd;
cmd=function(command){
 const why=nexusReason(command);
 if(why){gt=null;log('⚠️ '+why);window.__specterNotify?.(why,'fail');return false}
 if(/^(STOP|ESTOP|PWR|HOME)$/.test(command)){nexusRun++;nexusController?.abort();nexusPreview=null}
 legacyCmd(command);return true;
};
const legacyAct=act;
act=function(o){
 if(!o||typeof o!=='object'||!OKI.has(o.i))return 'Invalid assistant command.';
 if(o.i==='joint'&&live){
  const safety=nexusReason('SPD:1400');if(safety)return safety;
  if(!measured){try{homedAngularScale(o.joint==='shoulder'?'s':o.joint==='elbow'?'e':'b')}catch(e){return e.message}}
 }
 if(['joint','grip','wrist','speed','mot','wp'].includes(o.i)&&!Number.isFinite(Number(o.v)))return 'A finite numeric value is required.';
 if(o.i==='mot'){
  const target=Number(o.v);if(target!==0&&target!==1)return 'Say motors on or motors off.';
  if(!!S.en===!!target)return target?'Motors are already on.':'Motors are already off.';
  const reason=nexusReason('PWR');if(reason)return reason;
  if(cmd('PWR')===false)return 'Motor power command was refused.';
  return live?(target?'Motor power-on requested.':'Motor power-off requested.'):(target?'Motors on.':'Motors off.');
 }
 if(o.i==='status')return (live?'USB connected. ':'Simulator. ')+(S.en?'Motors on, ':'Motors off, ')+(S.hm?'homed. ':'not homed. ')+(S.lf?'Limit switch fault.':C.repairLock===1?'Firmware repair lock on.':'Check diagnostics for details.');
 return legacyAct(o);
};
function nexusDiagnostics(){
 return JSON.stringify({mode:live?'USB':'simulation',positionSource:'open-loop firmware pulse counts; not physical feedback',
 firmwareIdentity,configReceived,repairLockSource,firmwareRepairLock:C.repairLock??'unknown',telemetryAgeMs:live?Date.now()-lastNexusTel:null,
 homed:!!S.hm,motors:!!S.en,limitGuard:!!S.lim,limitFault:S.lf??'unavailable',rawSwitches:S.rawSw??'unavailable',
 debouncedSwitches:S.sw,firmwareDriftEvents:S.dr,calibrated:!!measured,homingSpanReady,angleMapping:measured?'measured':homingSpanReady?'homed span plus configured ranges; estimated':'unavailable',
 calibration:measured?Object.fromEntries(['b','s','e'].map(k=>[k,NK.fit(measured.joints[k].points)])):null},null,2);
}
async function ollamaJSON(path,body,signal){
 const timeout=AbortSignal.timeout(body?60000:5000);
 const response=await fetch(OL+path,{method:body?'POST':'GET',headers:body?{'Content-Type':'application/json'}:{},
 body:body?JSON.stringify(body):undefined,signal:signal?AbortSignal.any([signal,timeout]):timeout});
 if(!response.ok)throw Error('Ollama HTTP '+response.status);
 return response.json();
}
let installedChatModels=[];
loadModels=async function(){
 try{
  const result=await ollamaJSON('/api/tags');
  if(!Array.isArray(result.models))throw Error('Malformed model list');
  const models=result.models.filter(m=>typeof m.name==='string'&&(!Array.isArray(m.capabilities)||m.capabilities.includes('completion')));
  installedChatModels=models;
  $('v_m').replaceChildren(...models.map(m=>new Option(m.name,m.name)));
  const saved=localStorage.getItem('specterModel');
  $('v_m').value=models.some(m=>m.name===saved)?saved:(models.find(m=>/8b/i.test(m.name)||/^8(?:\.|$)/.test(m.details?.parameter_size||''))||models[0])?.name||'';
  if(!models.length)throw Error('No locally installed models');
  warmChatModel();
  $('nexusResult').textContent='Discovered '+models.length+' installed model(s). Selected '+$('v_m').value;
  return true;
 }catch(e){$('nexusResult').textContent='Ollama discovery failed: '+e.message+'. Start Ollama and allow http://localhost:8000 as an origin.';return false}
};
$('v_m').onchange=()=>{localStorage.setItem('specterModel',$('v_m').value);nexusPreview=null;warmChatModel()};
let warmedChatModel='';
function warmChatModel(){
 const model=$('v_m').value;if(!model||warmedChatModel===model)return;
 warmedChatModel=model;
 ollamaJSON('/api/generate',{model,stream:false,keep_alive:'10m',options:{num_ctx:2048,num_predict:0}},AbortSignal.timeout(15000)).catch(()=>{if(warmedChatModel===model)warmedChatModel=''});
}
let specterChatHistory=[];
window.__specterQuestion=text=>/^(tell me\b|explain\b|describe\b|what\b|why\b|how\b|who\b|when\b|where\b|which\b|do you\b|are you\b|can you (tell|explain|describe)\b|could you (tell|explain|describe)\b)/i.test(text.trim());
function localAssistantAnswer(text){
 const q=String(text).toLowerCase();
 if(/\b(your|you|specter)\b/.test(q)&&/\b(ai|technolog\w*|language model|speech recognition|voice technolog\w*)\b/.test(q)){
  const engine=$('v_eng').value==='local'?'local Whisper':$('v_eng').value==='groq'?'Groq Whisper':'browser speech recognition';
  return 'I use '+($('v_m').value||'a local language model')+' through Ollama, '+engine+' for speech to text, and browser speech synthesis to speak. Robot commands use local control logic.';
 }
 if(/(?:which|what).*(?:model).*(?:using|use|selected)|(?:which|what).*(?:your|current).*(?:model)/.test(q))
  return 'The selected language model is '+($('v_m').value||'not configured')+'.';
 if(/(?:what is|explain|how does|tell me about).*(?:auto.?homing|homing)/.test(q))
  return 'Homing measures shoulder and elbow travel in steps using limit switches. It does not measure the base angle or physical degrees.';
 return '';
}
ask=async function(text){
 // Conversation never executes commands or routes to the hidden task planner.
 ctl?.abort();
 const factualAnswer=localAssistantAnswer(text);
 if(factualAnswer){ctl=null;$('v_st').textContent='Ready';say(factualAnswer);return}
 const controller=new AbortController();ctl=controller;
 $('v_st').textContent='Thinking…';
 try{
  if(!$('v_m').value&&!await loadModels())throw Error('Ollama is unavailable or no chat model is installed.');
  if(controller.signal.aborted)return;
  const facts={selectedLocalModel:$('v_m').value,languageModelProvider:'Ollama on localhost',
   speechRecognitionEngine:$('v_eng').value,speechOutput:'browser speech synthesis (text to speech); Whisper is speech to text, not speech synthesis',
   robotConnection:live?'ESP32 USB serial at 460800 baud':'simulator',
   positionSource:'open-loop pulse counts, not measured physical feedback',
   measuredGeometryConfigured:!!measured,cameraRequired:false,firmwareRepairLock:C.repairLock??'unknown'};
  const reply=await ollamaJSON('/api/chat',{model:$('v_m').value,stream:false,
   keep_alive:"10m",options:{temperature:0.1,num_predict:90,num_ctx:2048},messages:[
    {role:'system',content:'You are Specter, a friendly robot-arm voice assistant. Answer the user question directly in concise, natural spoken English, at most 2 short sentences and 45 words. If uncertain, say so briefly or ask one short question. Never ramble or guess. This is conversation, not task planning: return plain text, never JSON actions, serial commands or a task preview instruction. Do not claim to move hardware. For an action request that this conversation cannot execute, ask the user to use an explicit supported robot command. Describe only technologies supported by the supplied facts. Do not invent encoders, vision, physical feedback, accuracy, autonomy, cloud AI or calibration values. Robot motors are electric joint actuators. Speech recognition can confuse motors with mortars; for an unclear robot-power request ask whether the user meant motors. No firing mechanism or fire command is supported; never invent such commands. Browser speech recognition may use an online service; local Whisper may download a model on first use. Use the selected speech engine when describing the current setup, not every option as active. Current facts: '+JSON.stringify(facts)},
    ...specterChatHistory.slice(-4),{role:'user',content:text}]},controller.signal);
  if(controller.signal.aborted||ctl!==controller)return;
  const answer=reply.message?.content;
  if(typeof answer!=='string'||!answer.trim())throw Error('The local model returned an empty answer.');
  const spoken=typeof conciseSpeech==='function'?conciseSpeech(answer,45):answer.trim().slice(0,400);
  specterChatHistory=[...specterChatHistory,{role:'user',content:text},{role:'assistant',content:spoken}].slice(-8);
  $('v_st').textContent='Ready';say(spoken);
 }catch(e){
  if(!controller.signal.aborted&&ctl===controller){
   $('v_st').textContent='AI unavailable';
   log('AI request failed: '+e.message);say('AI is unavailable. Please try again.');
  }
 }finally{if(ctl===controller)ctl=null}
};
async function nexusPlan(){
 nexusPreview=null;nexusController?.abort();const controller=new AbortController();nexusController=controller;
 try{
  if(!measured)throw Error('Enter measured calibration first');
  if(!$('v_m').value&&!await loadModels())return;
  const context={calibration:measured,steps:{b:S.b,s:S.s,e:S.e,w:S.w,g:S.g},firmwareSpans:{shTot:C.shTot,elbTot:C.elbTot},waypoints:W};
  const result=await ollamaJSON('/api/chat',{model:$('v_m').value,stream:false,format:'json',options:{temperature:0,num_predict:1000},messages:[
   {role:'system',content:'Propose a robot task as JSON {actions:[...]}. No camera or physical feedback exists. Use only explicit user coordinates or known joint angles. If ambiguous return {question:"..."}. Supported actions: {type:"joint",joint:"b|s|e",deg:finite absolute degrees}, {type:"cartesian",x:mm,y:mm,z:mm}, {type:"grip",deg:20..160}, {type:"wrist",deg:0..180}, {type:"wait",ms:0..10000}. Maximum 20 actions. Never invent object locations. Base yaw around +Z, shoulder pitch from horizontal, elbow relative pitch; target is forearm endpoint, not gripper TCP. Context: '+JSON.stringify(context)},
   {role:'user',content:$('nexusTask').value}]},controller.signal);
  if(controller.signal.aborted)return;
  const plan=JSON.parse(result.message.content);if(plan.question){$('nexusResult').textContent=String(plan.question);return}
  nexusBuildPreview(plan);
 }catch(e){if(e.name!=='AbortError')$('nexusResult').textContent=e.message}
}
function nexusBuildPreview(plan){
 const start={b:S.b,s:S.s,e:S.e,w:S.w,g:S.g},moves=NK.compile(measured,plan,start,C);
 nexusPreview={moves,start,profile:JSON.stringify(measured),live,created:Date.now()};
 $('nexusResult').textContent=JSON.stringify({notice:'Joint-interpolated preview; collision clearance and object contact are NOT verified. Forearm endpoint in mm.',moves:moves.map(m=>m.wait!==undefined?m:{pose:m.pose,end:m.path.at(-1)})},null,2);
 const canvas=$('nexusCanvas'),ctx=canvas.getContext('2d'),points=moves.flatMap(m=>m.path||[]);
 ctx.clearRect(0,0,canvas.width,canvas.height);if(!points.length)return;
 const span=Math.max(measured.upperMm+measured.foreMm,1),scale=Math.min(canvas.width/(span*2.2),canvas.height/(span*2.2));
 ctx.beginPath();points.forEach((p,i)=>{const x=canvas.width/2+p.x*scale,y=canvas.height/2-(p.z-measured.shoulderHeightMm)*scale;i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.strokeStyle='#8bf0c4';ctx.lineWidth=2;ctx.stroke();
}
async function nexusExecute(){
 const p=nexusPreview;
 try{
  if(!p||Date.now()-p.created>60000)throw Error('Create a fresh preview first');
  if(p.live!==live||p.profile!==JSON.stringify(measured)||['b','s','e','w','g'].some(k=>Math.abs(S[k]-p.start[k])>1))throw Error('Pose, connection or calibration changed; preview again');
  if(!S.hm||S.job||seqBusy||homingNow()||curJog)throw Error('Home first and stop other motion');
  const reason=nexusReason('GO_HOME');if(reason)throw Error(reason);
  nexusPreview=null;gt=null;const run=++nexusRun;seqBusy='nexus';
  const wait=async(test,ms)=>{const end=Date.now()+ms;while(run===nexusRun&&!test()){if(Date.now()>end)throw Error('Motion completion timeout');if(live&&(Date.now()-lastNexusTel>1000||S.lf||C.repairLock!==0||!S.en||!S.hm))throw Error('Telemetry or hardware safety state invalid');await sleep(40)}if(run!==nexusRun)throw Error('Task cancelled')};
  for(const m of p.moves){
   if(run!==nexusRun)throw Error('Task cancelled');
   if(m.wait!==undefined){const until=Date.now()+m.wait;await wait(()=>Date.now()>=until, m.wait+1000);continue}
   const q=m.pose;if(!cmd('GOTO:'+['b','s','e','w','g'].map(k=>q[k]).concat(1400).join(',')))throw Error('Command refused');
   await wait(()=>!S.job&&['b','s','e','w','g'].every(k=>Math.abs(S[k]-q[k])<=1),60000);
  }
  $('nexusResult').textContent='Task completed according to firmware pulse/servo estimates. Physical arrival is unverified.';
 }catch(e){cmd('STOP');$('nexusResult').textContent=e.message}
 finally{seqBusy=null;modeUI()}
}
const panel=document.createElement('section');panel.className='pane';panel.id='nexusPanel';
// Removed from the visible Automation UI at the user's request.
panel.hidden=true;panel.style.display='none';
panel.innerHTML='<h4>SPECTER NEXUS · measured motion and task planning</h4><p>Uncalibrated twin = illustrative simulation. Hardware positions are pulse estimates. Kinematics supports a rigid yaw / shoulder pitch / relative elbow chain; measure geometry before use.</p><details><summary>Measured calibration JSON</summary><p>Provide two or more signed pulse/angle pairs per joint. Shoulder angles are from horizontal; elbow angles are relative to the upper arm. Base is counterclockwise from +X viewed from above. Link lengths are pivot to pivot in mm. Base reference resets every boot.</p><textarea id="nexusCalibration" rows="14" style="width:100%" aria-label="Measured calibration JSON"></textarea><button id="nexusSave">Validate and save measurements</button><button id="nexusExport">Export calibration</button></details><div style="display:flex;gap:6px;flex-wrap:wrap"><button id="nexusModels">Discover Ollama models</button><button id="nexusDiag">Diagnostics</button><button id="nexusBaseReference">Confirm physical base at calibrated startup reference</button></div><textarea id="nexusTask" rows="3" style="width:100%" aria-label="AI task request" placeholder="Describe a task with measured joint angles or known coordinates"></textarea><button id="nexusPlan">AI task preview</button><button id="nexusCartesian">Preview XYZ target</button><input id="nexusXYZ" aria-label="Target x,y,z in millimetres" placeholder="x,y,z (mm)"><button id="nexusExecute" class="wn">Execute reviewed preview</button><button id="nexusCancel" class="bd">Cancel task</button><canvas id="nexusCanvas" width="600" height="220" style="width:100%" aria-label="Task path side projection X Z"></canvas><pre id="nexusResult" style="white-space:pre-wrap;overflow-wrap:anywhere">No physical calibration loaded. No camera required.</pre>';
document.getElementById('page-automation').append(panel);
const blank={version:1,geometry:'yaw-pitch-relative-elbow',measurementNotes:'',upperMm:null,foreMm:null,shoulderHeightMm:null,joints:Object.fromEntries(['b','s','e'].map(k=>[k,{points:[{steps:null,deg:null},{steps:null,deg:null}],limits:[null,null],maxResidualDeg:null}]))};
$('nexusCalibration').value=JSON.stringify(measured||blank,null,2);
$('nexusSave').onclick=()=>{try{nexusApply(JSON.parse($('nexusCalibration').value));nexusPreview=null;$('nexusResult').textContent='Calibration validated and saved.'}catch(e){$('nexusResult').textContent=e.message}};
$('nexusExport').onclick=()=>{if(!measured)return;const a=document.createElement('a');a.href='data:application/json,'+encodeURIComponent(JSON.stringify(measured,null,2));a.download='specter-measured-calibration.json';a.click()};
$('nexusBaseReference').onclick=()=>{if(!live||S.b!==0){$('nexusResult').textContent='Connect with the base physically aligned to the measured zero reference and firmware base count zero.';return}baseReferenceConfirmed=true;$('nexusResult').textContent='Operator confirmed base startup reference for this session.'};
$('nexusModels').onclick=loadModels;$('nexusDiag').onclick=()=>{$('nexusResult').textContent=nexusDiagnostics()};
$('nexusPlan').onclick=nexusPlan;$('nexusExecute').onclick=nexusExecute;
$('nexusCancel').onclick=()=>{nexusRun++;nexusController?.abort();nexusPreview=null;cmd('STOP')};
$('nexusCartesian').onclick=()=>{try{const v=$('nexusXYZ').value.split(',').map(Number);if(v.length!==3||!v.every(Number.isFinite))throw Error('Enter x,y,z in mm');nexusBuildPreview({actions:[{type:'cartesian',x:v[0],y:v[1],z:v[2]}]})}catch(e){$('nexusResult').textContent=e.message}};
if(measured)nexusApply(measured);
loadModels();
setInterval(()=>{if(live&&lastNexusTel&&Date.now()-lastNexusTel>1000&&(gt||curJog||seqBusy)){gt=null;jogStop();seqCancel();nexusRun++;cmd('STOP');log('⚠️ Stale telemetry: task stopped; physical state unknown')}},200);

// Distinguish a confirmed hardware repair lock from missing serial configuration.
const lockPanel=document.createElement('section');
lockPanel.id='nexusHardwareStatus';lockPanel.className='pane';
lockPanel.style.cssText='margin-bottom:18px;border-color:var(--wn);';
lockPanel.setAttribute('role','status');lockPanel.setAttribute('aria-live','polite');
const lockTitle=document.createElement('h4'),lockMessage=document.createElement('p');
lockTitle.textContent='Hardware readiness';
lockMessage.style.cssText='line-height:1.6;margin-bottom:12px;overflow-wrap:anywhere';
const requestConfig=document.createElement('button'),requestDiag=document.createElement('button');
requestConfig.textContent='Read firmware configuration';requestDiag.textContent='Read homing diagnostics';
requestDiag.style.marginLeft='8px';
requestConfig.onclick=()=>{if(!live)return;configRequestCount=0;nextConfigRequestAt=Date.now()+1000;cmd('GET');cmd('HOME_DIAG')};
requestDiag.onclick=()=>{if(live)cmd('HOME_DIAG')};
lockPanel.append(lockTitle,lockMessage,requestConfig,requestDiag);
document.getElementById('page-manual').prepend(lockPanel);
let previousLockState='';
function updateHardwareReadiness(){
 const subtitle=document.querySelector('#slotManual h4 small');
 if(subtitle)subtitle.textContent=live&&!measured?
  'Auto-home maps shoulder/elbow to configured angle ranges · physical degrees unverified':
  'Hold to move · Enter to go to angle';
 const state=!live?'simulation':C.repairLock===1?'locked':C.repairLock===0?'available':configReceived?'unsupported':'unknown';
 if(state!==previousLockState){
  previousLockState=state;
  lockPanel.hidden=state==='simulation';
  lockTitle.textContent=state==='locked'?'Hardware repair required':state==='unknown'?'Waiting for firmware configuration':state==='unsupported'?'Uploaded firmware protocol does not match':'Firmware repair lock: OFF';
  lockMessage.textContent=state==='locked'?
   'ESP32 reports repairLock=1. The supplied firmware disables motors, servos and homing for a damaged elbow upper limit switch. Replace/repair that switch and verify DIR and wiring before any firmware change that would enable motion. No browser button can clear this compiled lock. STOP and E-STOP remain available.':
   state==='unsupported'?'Configuration arrived, but repairLock is missing. The firmware uploaded on the ESP32 does not match the current project protocol. Upload specter_arm_smooth.ino from this project; this does not clear its hardware repair lock.':
   state==='unknown'?'USB is connected, but a valid repairLock field has not arrived. The dashboard retries GET up to five times. If this remains, read homing diagnostics and reconnect. Firmware without the repairLock field needs compatibility review; its state will not be guessed.':
   'ESP32 reports repairLock=0. Other protections, homing, calibration and base-reference checks still apply.';
  lockPanel.style.borderColor=state==='locked'?'var(--bd)':state==='unknown'?'var(--wn)':'var(--ok)';
 }
 requestConfig.disabled=requestDiag.disabled=!live;
 if(live&&(state==='unknown'||!verifiedSwitchConfig)&&configRequestCount<5&&Date.now()>=nextConfigRequestAt){
  configRequestCount++;nextConfigRequestAt=Date.now()+1000;cmd('GET');if(configRequestCount===1||configRequestCount===5)cmd('HOME_DIAG');
 }
}
setInterval(updateHardwareReadiness,250);
updateHardwareReadiness();
