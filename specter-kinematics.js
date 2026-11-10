/* SPECTER NEXUS: pure, dependency-free calibrated kinematics. Units: mm, degrees, pulses. */
(function(root){
'use strict';
const axes=['b','s','e'], rad=Math.PI/180;
const finite=n=>typeof n==='number'&&Number.isFinite(n);
function requireNumber(n,label){if(!finite(n))throw Error(label+' must be a finite number');return n}
function fit(points){
 if(!Array.isArray(points)||points.length<2)throw Error('At least two measured step/angle pairs required');
 points.forEach(p=>{requireNumber(p.steps,'steps');requireNumber(p.deg,'angle')});
 const x=points.reduce((s,p)=>s+p.steps,0)/points.length,y=points.reduce((s,p)=>s+p.deg,0)/points.length;
 const den=points.reduce((s,p)=>s+(p.steps-x)**2,0);
 if(!den)throw Error('Measurements need different step counts');
 const scale=points.reduce((s,p)=>s+(p.steps-x)*(p.deg-y),0)/den;
 if(Math.abs(scale)<1e-9)throw Error('Measured angular travel is zero');
 const offset=y-scale*x, residual=Math.max(...points.map(p=>Math.abs(offset+scale*p.steps-p.deg)));
 return {scale,offset,residual};
}
function validate(p){
 if(!p||p.version!==1||p.geometry!=='yaw-pitch-relative-elbow')throw Error('Profile must declare version 1 and yaw-pitch-relative-elbow geometry');
 if(typeof p.measurementNotes!=='string'||!p.measurementNotes.trim())throw Error('Record measurement method and reference in measurementNotes');
 for(const k of axes){
  const j=p.joints?.[k];if(!j)throw Error('Missing joint '+k);
  fit(j.points);
  if(!Array.isArray(j.limits)||j.limits.length!==2||!j.limits.every(finite)||j.limits[0]>=j.limits[1])throw Error('Invalid measured limits for '+k);
  if(!finite(j.maxResidualDeg)||j.maxResidualDeg<0||fit(j.points).residual>j.maxResidualDeg)throw Error('Calibration residual exceeds tolerance for '+k);
 }
 for(const k of ['upperMm','foreMm','shoulderHeightMm'])requireNumber(p[k],k);
 if(p.upperMm<=0||p.foreMm<=0||p.shoulderHeightMm<0)throw Error('Invalid measured link geometry');
 return p;
}
function angle(p,k,steps){requireNumber(steps,'steps');const f=fit(p.joints[k].points);return f.offset+steps*f.scale}
function steps(p,k,deg){requireNumber(deg,'angle');const f=fit(p.joints[k].points);return Math.round((deg-f.offset)/f.scale)}
function angles(p,q){return Object.fromEntries(axes.map(k=>[k,angle(p,k,q[k])]))}
function checkAngles(p,q){for(const k of axes){requireNumber(q[k],k);const [lo,hi]=p.joints[k].limits;if(q[k]<lo-1e-8||q[k]>hi+1e-8)throw Error(k+' outside measured limits')}return q}
function checkSteps(p,q,cfg){
 checkAngles(p,angles(p,q));
 for(const [k,total] of [['s',cfg?.shTot],['e',cfg?.elbTot]]){
  if(!Number.isInteger(q[k]))throw Error(k+' target must be integer pulses');
  if(!finite(total)||total<=40)throw Error('Firmware travel span unavailable');
  if(q[k]<20||q[k]>total-20)throw Error(k+' target violates firmware 20-pulse soft margin');
 }
 if(!Number.isSafeInteger(q.b)||Math.abs(q.b)>2147483647)throw Error('Invalid base pulse count');
 return q;
}
function fk(p,q){
 validate(p);checkAngles(p,q);
 const b=q.b*rad,s=q.s*rad,e=q.e*rad;
 const r=p.upperMm*Math.cos(s)+p.foreMm*Math.cos(s+e);
 return {x:r*Math.cos(b),y:r*Math.sin(b),z:p.shoulderHeightMm+p.upperMm*Math.sin(s)+p.foreMm*Math.sin(s+e)};
}
function ik(p,target,seed={b:0,s:0,e:0}){
 validate(p);for(const k of ['x','y','z'])requireNumber(target[k],k);
 const r=Math.hypot(target.x,target.y),z=target.z-p.shoulderHeightMm,L=p.upperMm,M=p.foreMm;
 const c=(r*r+z*z-L*L-M*M)/(2*L*M);
 if(c>1+1e-10||c< -1-1e-10)throw Error('Unreachable Cartesian target');
 const out=[];
 for(const signedR of r<1e-9?[0]:[r,-r]){
  const yaw=r<1e-9?seed.b:Math.atan2(target.y,target.x)/rad+(signedR<0?180:0);
  for(const e of [Math.acos(Math.max(-1,Math.min(1,c))),-Math.acos(Math.max(-1,Math.min(1,c)))]){
   const s=(Math.atan2(z,signedR)-Math.atan2(M*Math.sin(e),L+M*Math.cos(e)))/rad;
   // Enumerate all equivalent angles permitted by finite measured joint limits.
   const equivalents=(a,k)=>{const [lo,hi]=p.joints[k].limits;const n0=Math.ceil((lo-a)/360),n1=Math.floor((hi-a)/360);
    if(n1-n0>100)throw Error('Joint limit range too large');return Array.from({length:Math.max(0,n1-n0+1)},(_,i)=>a+360*(n0+i))};
   for(const b of equivalents(yaw,'b'))for(const sh of equivalents(s,'s'))for(const el of equivalents(e/rad,'e')){
    const q={b,s:sh,e:el};if(!out.some(a=>axes.every(k=>Math.abs(a[k]-q[k])<1e-7)))out.push(q);
   }
  }
 }
 if(!out.length)throw Error('Target has no solution inside measured joint limits');
 return out.sort((a,b)=>axes.reduce((s,k)=>s+(a[k]-seed[k])**2,0)-axes.reduce((s,k)=>s+(b[k]-seed[k])**2,0));
}
function preview(p,start,end,cfg,count=60){
 checkSteps(p,start,cfg);checkSteps(p,end,cfg);
 return Array.from({length:count+1},(_,i)=>{
  const q=Object.fromEntries(axes.map(k=>[k,Math.round(start[k]+(end[k]-start[k])*i/count)]));
  checkSteps(p,q,cfg);return {steps:q,...fk(p,angles(p,q))};
 });
}
function compile(p,plan,start,cfg){
 validate(p);
 if(!plan||!Array.isArray(plan.actions)||!plan.actions.length||plan.actions.length>20)throw Error('Plan needs 1–20 actions');
 let pose={...start};const moves=[];
 for(const a of plan.actions){
  if(!a||!['joint','cartesian','grip','wrist','wait'].includes(a.type))throw Error('Unsupported plan action');
  if(a.type==='wait'){if(!finite(a.ms)||a.ms<0||a.ms>10000)throw Error('Wait must be 0–10000 ms');moves.push({wait:a.ms});continue}
  let q={b:pose.b,s:pose.s,e:pose.e};
  if(a.type==='joint'){if(!axes.includes(a.joint)||!finite(a.deg))throw Error('Invalid joint action');q[a.joint]=steps(p,a.joint,a.deg)}
  if(a.type==='cartesian'){
   const solutions=ik(p,a,angles(p,pose));
   let found;for(const solution of solutions){const candidate=Object.fromEntries(axes.map(k=>[k,steps(p,k,solution[k])]));
    try{checkSteps(p,candidate,cfg);found=candidate;break}catch{}}
   if(!found)throw Error('IK solutions violate firmware pulse limits');q=found;
  }
  const w=a.type==='wrist'?a.deg:pose.w,g=a.type==='grip'?a.deg:pose.g;
  if(!finite(w)||w<0||w>180||!finite(g)||g<20||g>160)throw Error('Invalid servo range');
  const path=preview(p,pose,q,cfg);
  pose={...q,w:Math.round(w),g:Math.round(g)};moves.push({pose:{...pose},path});
 }
 return moves;
}
const api={fit,validate,angle,steps,angles,checkAngles,checkSteps,fk,ik,preview,compile};
if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.SpecterKinematics=api;
})(typeof globalThis!=='undefined'?globalThis:this);
