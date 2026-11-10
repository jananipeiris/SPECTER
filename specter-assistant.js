/* Persistent cross-page assistant and original manual controls. */
let lastAssistantReply='';
const previousSay=say;
function conciseSpeech(text,maxWords=55){
 const cleaned=String(text||'').replace(/[*#`]/g,'').replace(/\s+/g,' ').trim();
 const sentences=cleaned.match(/[^.!?]+[.!?]+|[^.!?]+$/g)||[];
 const brief=sentences.slice(0,2).join(' ').trim(),words=brief.split(/\s+/);
 return words.length>maxWords?words.slice(0,maxWords).join(' ').replace(/[,:;]$/,'')+'.':brief;
}
say=function(text){if(!text)return;lastAssistantReply=conciseSpeech(text);return previousSay(lastAssistantReply)};
const assistantStyle=document.createElement('style');
assistantStyle.textContent=`
:root{--assistant-dock-height:0px}
/* Keep assistant behavior available; remove only its floating UI. */
#assistantDock{display:none!important}
#scontent{padding-bottom:calc(var(--assistant-dock-height) + 32px)!important}
#assistantDock{position:fixed;right:18px;bottom:14px;width:min(650px,calc(100vw - 36px));z-index:1000;background:#111316;border:1px solid var(--stroke2);border-radius:14px;box-shadow:0 12px 50px #0009;padding:12px;color:var(--tx)}
#assistantDock .assistTop{display:flex;align-items:center;gap:8px;flex-wrap:wrap}
#assistantDock button,#manualDrawer button{border-radius:8px;background:#181b1f;border:1px solid var(--stroke2);padding:9px 12px;color:var(--tx)}
#assistantDock button.on{background:var(--accent);color:var(--accent-ink)}
#assistantDock .assistStop{color:#ff8a80;border-color:#ff5a4e88}
#assistantDock .assistState{font:11px var(--mono);color:var(--dim);flex:1;min-width:100px}
#assistantDock .assistText{font-size:12px;line-height:1.4;margin:7px 0;color:var(--dim);white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
#assistantDock form{display:flex;gap:8px}
#assistantDock input{flex:1;min-width:0;background:#0c0d0f;border:1px solid var(--stroke2);border-radius:8px;padding:9px 10px;color:var(--tx);background-image:none;cursor:text;min-height:38px}
#manualDrawer{position:fixed;right:18px;bottom:calc(var(--assistant-dock-height) + 26px);width:min(440px,calc(100vw - 36px));max-height:calc(100dvh - var(--assistant-dock-height) - 50px);overflow:auto;z-index:1001;background:#111316;border:1px solid var(--stroke2);border-radius:14px;box-shadow:0 12px 60px #000b;padding:14px}
#manualDrawer[hidden]{display:none!important}
#manualDrawer .drawerHead{display:flex;justify-content:space-between;align-items:center;margin-bottom:12px}
#manualDrawer .pane{background:#0c0d0f;border:1px solid var(--stroke);border-radius:12px;padding:14px}
#manualDrawer h4{display:flex;flex-direction:column;gap:5px}
#manualDrawer #mcs>*{padding:10px!important}
#manualDrawer .mc{grid-template-columns:44px minmax(0,1fr) 44px}
#manualDrawer input[type=number]{min-width:0;background:#111316;color:var(--tx);border:1px solid var(--stroke2);border-radius:8px;background-image:none;padding:9px;min-height:42px}
#specterHealth{position:static!important;max-width:100%!important;margin-top:16px}
#specterToast{bottom:calc(var(--assistant-dock-height) + 28px)!important}
@media(max-width:650px){#assistantDock{right:10px;bottom:10px;width:calc(100vw - 20px)}#assistantDock button{padding:8px 10px;font-size:12px}#manualDrawer{right:10px;width:calc(100vw - 20px)}#assistantDock .assistState{min-width:80px}}
`;
document.head.append(assistantStyle);
const assistantDock=document.createElement('aside');assistantDock.id='assistantDock';assistantDock.setAttribute('aria-label','Specter assistant available on every page');
assistantDock.innerHTML='<div class="assistTop"><button id="globalMic" type="button" aria-pressed="false">Start mic</button><span id="globalVoiceState" class="assistState" role="status">Mic off</span><button id="globalManual" type="button" aria-expanded="false" aria-controls="manualDrawer">Controls</button><button id="globalQuiet" type="button">Quiet</button><button id="globalStop" type="button" class="assistStop">Stop motion</button><button id="globalFastModel" type="button" hidden>Faster AI</button><button id="globalVoiceSettings" type="button" aria-label="Open voice settings">Settings</button></div><div id="globalHeard" class="assistText">Say “Specter” + command, or type below.</div><div id="globalReply" class="assistText" aria-live="polite"></div><form id="globalCommandForm"><input id="globalCommand" type="text" autocomplete="off" aria-label="Ask Specter or enter a robot command" placeholder="Ask Specter or type a command…"><button type="submit">Send</button></form>';
document.body.append(assistantDock);
const manualDrawer=document.createElement('aside');manualDrawer.id='manualDrawer';manualDrawer.hidden=true;manualDrawer.setAttribute('aria-label','Quick manual controls');
manualDrawer.innerHTML='<div class="drawerHead"><strong>Manual controls</strong><button id="closeGlobalManual" type="button" aria-label="Close manual controls">Close</button></div><div style="display:flex;gap:8px;margin-bottom:12px;flex-wrap:wrap"><button id="drawerHome" type="button">Auto-home</button><button id="drawerGoHome" type="button">Go home</button><button id="drawerEstop" type="button" style="color:#ff8a80">E-STOP</button></div><div id="globalManualContent"></div>';
document.body.append(manualDrawer);
const manualHomeSlot=document.getElementById('slotManual'),originalManualPane=manualHomeSlot.querySelector('.pane');
function toggleGlobalManual(open=manualDrawer.hidden){
 jogStop();
 if(open){$('globalManualContent').append(originalManualPane);manualDrawer.hidden=false}
 else{manualHomeSlot.append(originalManualPane);manualDrawer.hidden=true}
 $('globalManual').setAttribute('aria-expanded',String(open));
}
$('globalManual').onclick=()=>toggleGlobalManual();$('closeGlobalManual').onclick=()=>toggleGlobalManual(false);
$('globalMic').onclick=()=>{$('v_mic').click()};
$('drawerHome').onclick=()=>{gt=null;pending=null;cmd('HOME')};
$('drawerGoHome').onclick=()=>{gt=null;pending=null;cmd('GO_HOME')};
$('drawerEstop').onclick=()=>{quietAssistant();gt=null;pending=null;cmd('ESTOP')};
function fastChatModel(){
 return installedChatModels.find(m=>/^(?:[123](?:\.\d+)?|4(?:\.0)?)B$/i.test(m.details?.parameter_size||'')||/:(?:1b|2b|3b|4b)\b/i.test(m.name));
}
$('globalFastModel').onclick=()=>{
 const model=fastChatModel();if(!model)return;
 ctl?.abort();specterChatHistory=[];$('v_m').value=model.name;$('v_m').onchange();
 lastAssistantReply='AI model selected: '+model.name;
};
function quietAssistant(){
 const current=ctl;ctl=null;current?.abort();nexusController?.abort();speechSynthesis.cancel();sayId++;speaking=false;
 lastAssistantReply='';$('v_st').textContent=vOn?'Listening':'Off';
}
$('globalQuiet').onclick=quietAssistant;
$('globalStop').onclick=()=>{quietAssistant();gt=null;pending=null;cmd('STOP');lastAssistantReply='Motion stop requested.'};
$('globalVoiceSettings').onclick=()=>{toggleGlobalManual(false);document.querySelector('.snav[data-page="voice"]').click();$('voiceSettings')?.scrollIntoView({block:'nearest',behavior:'smooth'})};
$('globalCommandForm').onsubmit=event=>{
 event.preventDefault();const input=$('globalCommand'),text=input.value.trim();if(!text)return;input.value='';
 quietAssistant();const {c}=wakeSplit(norm(text));$('v_tx').textContent='“'+text+'”';awakeUntil=Date.now()+UNLOCK_MS;
 if(EM_ONLY.test(c))hardStop(true);else if(STOP_ONLY.test(c))hardStop(false);else handle(c);
};
new ResizeObserver(()=>document.documentElement.style.setProperty('--assistant-dock-height',assistantDock.offsetHeight+'px')).observe(assistantDock);
document.getElementById('page-diagnostics').append(document.getElementById('specterHealth'));
let lastPage=document.querySelector('.spage.active')?.id;
const pageObserver=new MutationObserver(()=>{
 const page=document.querySelector('.spage.active')?.id;
 if(page!==lastPage){lastPage=page;jogStop();kd=null}
});
document.querySelectorAll('.spage').forEach(page=>pageObserver.observe(page,{attributes:true,attributeFilter:['class']}));
function refreshGlobalAssistant(){
 const fast=fastChatModel(),fastButton=$('globalFastModel');fastButton.hidden=!fast;fastButton.disabled=!!fast&&$('v_m').value===fast.name;fastButton.title=fast?'Switch to '+fast.name+'; speed depends on hardware':'';
 const mic=$('globalMic');mic.textContent=vOn?'Mic on · stop':'Start mic';mic.classList.toggle('on',vOn);mic.setAttribute('aria-pressed',String(vOn));
 $('globalVoiceState').textContent=ctl?'Thinking…':transcribing?'Transcribing…':speaking?'Speaking…':!vOn?'Mic off':Date.now()<awakeUntil?'Listening · unlocked':'Listening · say Specter';
 $('globalHeard').textContent=$('v_tx').textContent;$('globalReply').textContent=lastAssistantReply;
}
setInterval(refreshGlobalAssistant,200);refreshGlobalAssistant();
