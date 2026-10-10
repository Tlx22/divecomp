function FW(HAL){
const {analogMv,u8g2,Serial,prefs,beep,chime}=HAL;
const P_SURF=1.013,PH2O=0.0627,FN2_AIR=0.7902,MPB=10,ASC=9;
const HT=[5,8,12.5,18.5,27,38.3,54.3,77,109,146,187,239,305,390,498,635];
const A=[1.1696,1,0.8618,0.7562,0.62,0.5043,0.441,0.4,0.375,0.35,0.3295,0.3065,0.2835,0.261,0.248,0.2327];
const B=[0.5578,0.6514,0.7222,0.7825,0.8126,0.8434,0.8693,0.891,0.9092,0.9222,0.9319,0.9403,0.9477,0.9544,0.9602,0.9653];
const K=HT.map(h=>Math.LN2/(h*60)),PAIR=(P_SURF-PH2O)*FN2_AIR;let P=HT.map(()=>PAIR);
// Helium: same 16-compartment structure, standard ZHL-16C He half-times/coefficients.
const HT_HE=[1.88,3.02,4.72,6.99,10.21,14.48,20.53,29.11,41.20,55.19,70.69,90.34,115.29,147.42,188.24,240.03];
const A_HE=[1.6189,1.3830,1.1919,1.0458,0.9220,0.8205,0.7305,0.6502,0.5950,0.5545,0.5333,0.5189,0.5181,0.5176,0.5172,0.5119];
const B_HE=[0.4770,0.5747,0.6527,0.7223,0.7582,0.7957,0.8279,0.8553,0.8757,0.8903,0.8997,0.9073,0.9122,0.9171,0.9217,0.9267];
const K_HE=HT_HE.map(h=>Math.LN2/(h*60));
let PHE=HT.map(()=>0); // helium loading; starts at 0 (no He in air)
// Combined-gas M-value: per compartment, weight A/B by how much of that compartment's
// loading is N2 vs He, then apply the same M-value formula to the combined total. With
// no helium ever present (PHE all zero) this reduces exactly to the original N2-only math.
function maxTolC(Qn,Qh,g){let m=0;for(let i=0;i<16;i++){const pn=Qn[i],ph=Qh[i],tot=pn+ph;const a=tot>1e-9?(A[i]*pn+A_HE[i]*ph)/tot:A[i];const b=tot>1e-9?(B[i]*pn+B_HE[i]*ph)/tot:B[i];const t=(tot-a*g)/(g/b-g+1);if(t>m)m=t}return Math.max(0,m)}
// Surface GF%: the inverse of maxTolC -- instead of solving for the ceiling depth at a given
// gradient factor, solve for the gradient factor whose ceiling would put the diver exactly at
// the surface right now. A high value right after surfacing (whether or not it technically broke
// the ceiling) is the signature of a rapid/rescue-type ascent -- the computer never had time to
// off-gas before the diver was already up, unlike a normal, paced ascent that lands well under 100%.
function surfaceGF(){
 let m=0;
 for(let i=0;i<16;i++){
  const pn=P[i],ph=PHE[i],tot=pn+ph;
  const a=tot>1e-9?(A[i]*pn+A_HE[i]*ph)/tot:A[i];
  const b=tot>1e-9?(B[i]*pn+B_HE[i]*ph)/tot:B[i];
  const m0=a+P_SURF/b; // M-value at the surface (gf=100%)
  const g=(tot-P_SURF)/(m0-P_SURF)*100;
  if(g>m)m=g;
 }
 return Math.max(0,m);
}
const TD=[10,12,14,16,18,20,22,25,30,35,40,42],TN=[219,147,98,72,56,45,37,29,20,14,9,8];let tadj=TN.slice();
const ALG=["Buhlmann","RGBM-sim","PADI-tbl"];
const S={mode:'SURFACE',view:0,algo:0,fo2:21,ppo2:140,deep:false,dirty:false,zero:250,zeroValid:false,gauge:0,depth:0,maxD:0,rate:0,pabs:P_SURF,prev:P_SURF,ppo2n:.21,mod:0,fault:false,bad:0,dstart:0,dur:0,dcount:0,viol:0,nofly:0,lock:0,ndl:99,deco:false,ceil:0,tts:0,nstop:0,nmin:0,texc:false,dDeco:false,dViol:false,breach:0,gf0:.3,gf1:.7,ss:false,ssDone:false,ssEl:0,ds:false,dsDone:false,dsEl:0,dsD:0,cur:0,desc:0,surf:0,sync:0,wc:false,wa:false,wp:false,hdg:0,card:'N',showLog:false,logCur:0,logPage:0,ugf0:.3,ugf1:.7,batt:100,cns:0,atDepth:0,atAnchor:0,stops:[],lockout:0,compLocked:false,refHdg:null,bookmarks:[],surfPage:0,toast:'',toastUntil:0,alarmActive:null,alarmStep:0,alarmNextAt:0,alarmCycle:0,popupMsg:'',popupUntil:0,units:'met',avgAccD:0,avgAccT:0,avgAccTemp:0,profile:[],profileIv:10,lastDiveEndMs:null,gasList:[{o2:21,he:0,enabled:true,labelE:false},{o2:21,he:0,enabled:false,labelE:false},{o2:21,he:0,enabled:false,labelE:false}],activeGas:0,gasesUsedThisDive:[],gasSetup:false,gasCur:0,gasSwitchUI:false,gasSwitchSel:0,diveMode:'OC'};
// Dive modes, selectable only on the surface (menu isn't reachable mid-dive at all):
// OC=open circuit -- NOT itself A/NX/OCM; the gas setup you already configured decides which
// of those it actually is (see ocLabel()), so there's nothing to pick separately and nothing
// to get out of sync. F=freedive, G=gauge (bottom timer, diver plans their own deco).
// CC=closed circuit -- deliberately left OUT of DIVE_MODES for now (not selectable at all),
// until real CCR support (setpoint control, diluent, bailout) is built; add it back as a 4th
// entry then. The active mode is shown on every dive screen (see diveBody header).
const DIVE_MODES=['OC','G','F'];
let lastSim=0,lastS=0,saveAcc=0,lastR=0;const hist=[],pw={};
const blinkOn=(now,periodMs)=>Math.floor(now/periodMs)%2===0;
// Unit helpers. All physics/state stay metric; these only format what is drawn.
const M2FT=3.28084;
const isImp=()=>S.units==='imp';
const dU=()=>isImp()?'ft':'m';                                   // depth unit label
const dS=m=>isImp()?String(Math.round(m*M2FT)):m.toFixed(1);     // depth string (imperial = whole feet)
const dI=m=>isImp()?String(Math.round(m*M2FT)):m.toFixed(0);     // whole-number depth string
const cS=m=>isImp()?String(Math.round(m/3)*10):m.toFixed(0);     // ceiling/stop depth: 3 m steps shown as 10 ft steps (always slightly deeper than the true value)
const rS=r=>isImp()?(r*M2FT).toFixed(0):r.toFixed(1);            // vertical rate string
const tS=c=>isImp()?(c*9/5+32).toFixed(1)+'F':c.toFixed(1)+'C';  // temperature string
const pAt=d=>P_SURF+d/MPB;
function leg(Qn,Qh,p0,p1,s,fN2,fHe){
 if(s<=0)return;
 const a=(p0-PH2O)*fN2,b=(p1-PH2O)*fN2,R=(b-a)/s;
 for(let i=0;i<16;i++){const k=K[i],e=Math.exp(-k*s);Qn[i]=a+R*(s-1/k)-(a-Qn[i]-R/k)*e}
 const a2=(p0-PH2O)*fHe,b2=(p1-PH2O)*fHe,R2=(b2-a2)/s;
 for(let i=0;i<16;i++){const k=K_HE[i],e=Math.exp(-k*s);Qh[i]=a2+R2*(s-1/k)-(a2-Qh[i]-R2/k)*e}
}
const maxTol=(Q,g)=>Math.max(0,...Q.map((p,i)=>(p-A[i]*g)/(g/B[i]-g+1)));
function gfNow(){if(S.algo==1){const hi=Math.min(S.ugf1,Math.max(.45,S.ugf1+.05-.05*Math.min(S.dcount,3)-.05*Math.min(S.viol,3)));return[S.ugf0,hi]}return[S.ugf0,S.ugf1]}
const CNS_TBL=[[.5,1e9],[.6,720],[.7,570],[.8,450],[.9,360],[1.0,300],[1.1,240],[1.2,210],[1.3,180],[1.4,150],[1.5,120],[1.6,45]];
function cnsRatePerMin(ppo2){if(ppo2<=.5)return 0;let lim=45;for(let i=1;i<CNS_TBL.length;i++){if(ppo2<=CNS_TBL[i][0]){const[a,la]=CNS_TBL[i-1],[b,lb]=CNS_TBL[i];const t=(ppo2-a)/(b-a);lim=la+(lb-la)*t;break}if(i==CNS_TBL.length-1)lim=45}return 100/lim}
function ndlCalc(Qn,Qh,pa,fN2,fHe,gh){
 if(!(fHe>0)&&!Qh.some(x=>x>1e-6)){ // no helium involved: identical to the original analytic formula
  const pi=(pa-PH2O)*fN2;let best=9999*60;
  for(let i=0;i<16;i++){const M=P_SURF+gh*(A[i]+P_SURF/B[i]-P_SURF);if(Qn[i]>=M)return 0;if(pi<=M)continue;best=Math.min(best,Math.log((pi-Qn[i])/(pi-M))/K[i])}
  return best/60;
 }
 // combined N2+He loads at different rates, so there's no closed form: step forward a
 // minute at a time (at the current, constant depth) until a ceiling would appear.
 const cn=Qn.slice(),ch=Qh.slice();
 for(let m=0;m<99;m++){
  if(maxTolC(cn,ch,gh)>P_SURF+1e-4)return m;
  leg(cn,ch,pa,pa,60,fN2,fHe);
 }
 return 99;
}
function ceilCalc(Qn,Qh,g){if(maxTolC(Qn,Qh,g[1])<=P_SURF+1e-4)return 0;const pl=maxTolC(Qn,Qh,g[0]),fm=Math.max(3,Math.ceil((pl-P_SURF)*MPB/3-1e-9)*3),pf=pAt(fm);for(let d=3;d<=fm+.01;d+=3){const pd=pAt(d),gg=g[1]+(g[0]-g[1])*(pd-P_SURF)/(pf-P_SURF);if(maxTolC(Qn,Qh,gg)<=pd+1e-4)return d}return fm}
function simDeco(g,fN2,fHe){const Qn=P.slice(),Qh=PHE.slice();let d=S.depth,t=0,fc=-1,fmn=0,fin=false;const sched=[];let curD=-1,curMin=0;
for(let it=0;it<600;it++){const c=ceilCalc(Qn,Qh,g);if(fc<0)fc=c;if(c<=0){t+=d/ASC*60;fin=true;if(curD>0)sched.push([curD,curMin]);break}
 if(d>c+.01){const s=(d-c)/ASC*60;leg(Qn,Qh,pAt(d),pAt(c),s,fN2,fHe);t+=s;d=c;if(curD>0)sched.push([curD,curMin]);curD=c;curMin=0}
 else{d=c;leg(Qn,Qh,pAt(c),pAt(c),60,fN2,fHe);t+=60;curMin++;if(Math.abs(c-fc)<.01)fmn++}}
S.tts=fin?Math.ceil(t/60):999;S.nstop=Math.max(fc,0);S.nmin=Math.max(1,fmn);S.stops=sched.slice(0,4)}
const tIdx=d=>TD.findIndex(x=>d<=x);
function buildTbl(){const fN2=1-S.fo2/100,PA=HT.map(()=>PAIR),PAHe=HT.map(()=>0);for(let j=0;j<12;j++){const pa=pAt(TD[j]),fr=ndlCalc(PA,PAHe,pa,fN2,0,.7),nw=ndlCalc(P,PHE,pa,fN2,0,.7),r=fr<.5?1:Math.min(1,nw/fr);tadj[j]=Math.floor(TN[j]*Math.max(0,r))}}
function saveState(){prefs.put('st',{P:P.slice(),PHE:PHE.slice(),nofly:S.nofly,lock:S.lock,dc:S.dcount,viol:S.viol,zero:S.zero,lde:S.lastDiveEndMs})}
function loadState(){const s=prefs.get('st',null);if(!s||!s.P||s.P.length!=16||s.P.some(x=>!(x>.3&&x<8)))return false;P=s.P.slice();if(s.PHE&&s.PHE.length==16)PHE=s.PHE.slice();S.nofly=s.nofly;S.lock=s.lock;S.dcount=s.dc;S.viol=s.viol;if(s.zero>50&&s.zero<2450){S.zero=s.zero;S.zeroValid=true}if(typeof s.lde==='number')S.lastDiveEndMs=s.lde;return true}
function saveSet(){prefs.put('set',{fo2:S.fo2,ppo2:S.ppo2,deep:S.deep,algo:S.algo,units:S.units,gasList:S.gasList,activeGas:S.activeGas,heFrac:S.heFrac,diveMode:S.diveMode});S.dirty=false}
function sensors(){let mv=0;for(let i=0;i<10;i++)mv+=analogMv(0);mv/=10;
 if(mv<50||mv>2450){if(S.bad<255)S.bad++;if(S.bad>=10)S.fault=true}else{S.bad=0;S.fault=false;let g=(mv-S.zero)/1000/2*12;if(S.mode!='DIVE'&&Math.abs(g)<.03)S.zero+=.002*(mv-S.zero);if(g<0)g=0;S.gauge+=.3*(g-S.gauge);S.depth=S.gauge*MPB;S.pabs=P_SURF+S.gauge}
 const h=HAL.heading();S.hdg=Math.round(h)%360;S.card=["N","NE","E","SE","S","SW","W","NW"][Math.floor((h+22.5)/45)%8]}
function rate(now){if(now-lastR>=1000){lastR=now;hist.push(S.depth);if(hist.length>6)hist.shift();S.rate=hist.length==6?(hist[0]-S.depth)*12:0}}
function startDive(t){if(S.dirty)saveSet();S.mode='DIVE';S.view=0;S.dstart=t;S.dur=0;S.maxD=S.depth;S.surf=0;S.ss=S.ssDone=S.ds=S.dsDone=false;S.ssEl=S.dsEl=0;S.dDeco=S.dViol=false;S.breach=0;S.breachTot=0;S.severe=false;S.deco=false;S.ceil=0;S.texc=false;S.lock=86400;S.gfUsed=[S.ugf0,S.ugf1];S.avgAccD=0;S.avgAccT=0;S.avgAccTemp=0;S.profile=[];S.profileIv=10;S.tissueStart=P.slice();S.surfIntervalSec=S.lastDiveEndMs!=null?Math.round((t-S.lastDiveEndMs)/1000):null;S.gasesUsedThisDive=[{fo2:S.fo2,he:Math.round(S.heFrac*100),label:gasLabel(S.gasList[S.activeGas])}];if(S.algo==2)buildTbl();saveState();beep('DIVE START',880,150)}
function endDive(){S.dur=Math.floor((S.surf-S.dstart)/1000);S.dcount++;if(S.dViol)S.viol=Math.min(255,S.viol+1);const missedSS=S.maxD>=10&&!S.ssDone;const tg=(S.dDeco||S.dViol||missedSS)?86400:(S.dcount>1?64800:43200);S.nofly=Math.max(S.nofly,tg);S.lock=86400;if(S.severe){S.lockout=172800;S.compLocked=true;beep('COMPUTER LOCKED',300,700)}const l=prefs.get('logs',[]);S.profile=S.profile.filter(p=>p.t<=S.dur);
 l.push({
  date:Date.now(), // sim uses the browser clock; real firmware needs the planned RTC for this
  durSec:S.dur, maxD:+S.maxD.toFixed(1), avgD:+(S.avgAccT>0?S.avgAccD/S.avgAccT:S.maxD).toFixed(1),
  avgTemp:+(S.avgAccT>0?S.avgAccTemp/S.avgAccT:HAL.temp()).toFixed(1),
  gases:S.gasesUsedThisDive.length?S.gasesUsedThisDive:[{fo2:S.fo2,he:Math.round(S.heFrac*100),label:gasLabel(S.gasList[S.activeGas])}],
  algo:S.algo, deco:S.dDeco?1:0, gfUsed:S.gfUsed||[S.ugf0,S.ugf1],
  surfIntervalSec:S.surfIntervalSec, surfPressure:+P_SURF.toFixed(3),
  tissueStart:{n2:S.tissueStart||P.slice(),he:null}, tissueEnd:{n2:P.slice(),he:null},
  profile:S.profile
 });
 prefs.put('logs',l.slice(-20)); // richer records now (profile+tissue snapshots) -> smaller cap than before
 S.lastDiveEndMs=S.surf;saveState();S.mode='SURFACE';S.surf=0;beep('DIVE END',660,300)}
function upd(dt,now){
 if(S.mode!='DIVE'){S.ndl=99;S.deco=false;S.ceil=0;S.tts=0;S.texc=false;S.wc=S.wa=S.wp=false;return}
 const g=gfNow();S.gf0=g[0];S.gf1=g[1];const f=1-S.fo2/100;S.texc=false;
 // F (freedive) and G (gauge) are bottom-timer modes: no NDL/deco/table guidance is computed
 // or shown at all -- the diver manages their own decompression, same convention as a real
 // gauge-mode dive computer. Tissue loading (leg(), called elsewhere in tick()) still runs
 // underneath regardless of mode, so later dives the same day stay accurate.
 if(S.diveMode=='F'||S.diveMode=='G'){S.ndl=99;S.deco=false;S.ceil=0;S.tts=0}
 else{
  if(S.algo==2){const ix=tIdx(S.maxD),el=Math.ceil(S.dur/60);if(ix<0||el>=tadj[ix]){S.texc=true;S.dViol=true}else{S.ndl=Math.min(99,tadj[ix]-el);S.deco=false;S.ceil=0;S.tts=Math.ceil(S.depth/ASC)}}
  if(S.algo!=2||S.texc){S.ceil=ceilCalc(P,PHE,g);S.deco=S.ceil>0;if(S.deco){S.ndl=0;S.dDeco=true;if(!lastSim||now-lastSim>=2000){simDeco(g,f,S.heFrac||0);lastSim=now||1}}else{S.ndl=Math.min(99,Math.floor(ndlCalc(P,PHE,S.pabs,f,S.heFrac||0,g[1])));S.tts=Math.ceil(S.depth/ASC)}}
 }
 S.wc=S.deco&&S.depth<S.ceil-.5;if(S.wc){S.breach+=dt;if(S.breach>10)S.dViol=true}else S.breach=0;
 S.wa=S.rate>9;if(S.rate>12)S.dViol=true;S.wp=S.ppo2n>=S.ppo2/100;S.wpLow=S.mode=='DIVE'&&S.ppo2n<0.18;S.wcns=S.cns>=80;S.wbat=S.batt<20;S.wndl2=!S.deco&&S.ndl<=2;if(S.wc){S.breachTot=(S.breachTot||0)+dt;if(S.breachTot>60)S.severe=true}}
function stops(dt){if(S.deco){S.ss=S.ds=false;return}
 if(S.deep&&S.maxD>=18&&!S.dsDone){S.dsD=S.maxD/2;S.ds=S.depth<S.maxD-3&&Math.abs(S.depth-S.dsD)<=1.5;if(S.ds){S.dsEl+=dt;if(S.dsEl>=120){S.dsDone=true;S.ds=false;beep('DEEP STOP DONE',1000,200)}}}
 if(S.maxD>=10&&!S.ssDone){if(S.depth>=3&&S.depth<=6)S.ss=true;else if(S.depth<2.5||S.depth>7){S.ss=false;S.ssEl=0}if(S.ss){S.ssEl+=dt;if(S.ssEl>=180){S.ssDone=true;S.ss=false;beep('SAFETY STOP DONE',1000,200)}}}}
function menuAdjust(dir){ // dir=+1 (DOWN/increase) or -1 (UP/decrease); fires the item at S.cur
 switch(S.cur){
  case 0: S.ppo2=Math.max(100,Math.min(160,S.ppo2+5*dir));S.dirty=true;break;
  case 1: S.deep=!S.deep;S.dirty=true;break;                              // toggle: either direction
  case 2: if(!S.lock){S.algo=(S.algo+dir+3)%3;S.dirty=true}break;         // cycle: UP=prev, DOWN=next
  case 3: S.diveMode=DIVE_MODES[(DIVE_MODES.indexOf(S.diveMode)+dir+DIVE_MODES.length)%DIVE_MODES.length];S.dirty=true;break; // cycle: UP=prev, DOWN=next
  case 4: S.mode='SYNC';S.sync=HAL.millis()+3000;Serial.println('LOGS '+JSON.stringify(prefs.get('logs',[])));break; // action: either direction fires it
  case 5: if(S.dirty)saveSet();S.mode='SURFACE';S.cur=0;S.showLog='list';S.logCur=0;break;                                        // action
  case 6: S.ugf0=Math.max(.1,Math.min(S.ugf1-.05,+(S.ugf0+.05*dir).toFixed(2)));S.dirty=true;break;
  case 7: S.ugf1=Math.max(S.ugf0+.05,Math.min(1.0,+(S.ugf1+.05*dir).toFixed(2)));S.dirty=true;break;
  case 8: S.units=S.units==='imp'?'met':'imp';S.dirty=true;break;   // toggle: either direction
  case 9: if(S.dirty)saveSet();S.gasSetup=true;S.gasCur=0;break;          // action: opens the gas-setup sub-screen
 }}
function gasSetupAdjust(dir){ // same dir convention, operating on the gas-setup sub-screen's 11 items
 switch(S.gasCur){
  case 0:{let o2=Math.max(1,Math.min(100-S.gasList[0].he,S.fo2+dir));S.fo2=o2;S.gasList[0].o2=o2;S.dirty=true;break;}
  case 1:{let he=Math.max(0,Math.min(100-S.fo2,S.gasList[0].he+dir));S.gasList[0].he=he;S.heFrac=he/100;S.dirty=true;break;}
  case 2:{let o2=Math.max(1,Math.min(100-S.gasList[1].he,S.gasList[1].o2+dir));S.gasList[1].o2=o2;S.dirty=true;break;}
  case 3:{let he=Math.max(0,Math.min(100-S.gasList[1].o2,S.gasList[1].he+dir));S.gasList[1].he=he;S.dirty=true;break;}
  case 4: S.gasList[1].enabled=!S.gasList[1].enabled;S.dirty=true;break;  // toggle: either direction
  case 5:{let o2=Math.max(1,Math.min(100-S.gasList[2].he,S.gasList[2].o2+dir));S.gasList[2].o2=o2;S.dirty=true;break;}
  case 6:{let he=Math.max(0,Math.min(100-S.gasList[2].o2,S.gasList[2].he+dir));S.gasList[2].he=he;S.dirty=true;break;}
  case 7: S.gasList[2].enabled=!S.gasList[2].enabled;S.dirty=true;break;
  // 8-10: per-gas A/E record label (see gasLabel()) -- appended after the original 8 items rather
  // than interleaved, so existing gasCur positions above are untouched. Toggle: either direction.
  case 8: S.gasList[0].labelE=!S.gasList[0].labelE;S.dirty=true;break;
  case 9: S.gasList[1].labelE=!S.gasList[1].labelE;S.dirty=true;break;
  case 10:S.gasList[2].labelE=!S.gasList[2].labelE;S.dirty=true;break;
 }}
function buttons(){
 const d3=HAL.edge(3),d2=HAL.edge(2),d0=HAL.edge(0),d1=HAL.edge(1);
 if((d3||d2||d0||d1)&&HAL.millis()<S.popupUntil)S.popupUntil=0; // any press dismisses the forced popup early; ongoing alarm sound/border/icon are unaffected
 if(S.gasSwitchUI){ // top priority: the gas-switch picker owns all 4 buttons while open
  if(d3)cycleGasSwitchSel(1);
  if(d0)cycleGasSwitchSel(-1);
  if(d1)confirmGasSwitch();
  if(d2)S.gasSwitchUI=false; // cancel, no switch made
  return;
 }
 if(d3){ // DOWN: menu=increase highlighted item; surface=next preview page; dive=bookmark/set-bearing (unchanged)
  if(S.mode=='MENU'&&S.gasSetup)gasSetupAdjust(1);
  else if(S.mode=='MENU')menuAdjust(1);
  else handleDown();
 }
 if(d2){if(S.showLog)S.showLog=false;else if(S.mode=='MENU'&&S.gasSetup)S.gasSetup=false;else if(S.mode=='SURFACE'&&!S.compLocked)S.mode='MENU';else if(S.mode=='MENU'){if(S.dirty)saveSet();S.mode='SURFACE';S.cur=0}else if(S.mode=='DIVE')S.view=(S.view+1)%5}
 if(d0){ // UP: menu=decrease highlighted item; surface=prev preview page; log browser=prev entry/page
  if(S.mode=='MENU'&&S.gasSetup)gasSetupAdjust(-1);
  else if(S.mode=='MENU')menuAdjust(-1);
  else if(S.showLog=='list')S.logCur=Math.max(0,S.logCur-1);
  else if(S.showLog=='detail')S.logPage=(S.logPage+2)%3;
  else if(S.mode=='SURFACE'&&!S.compLocked)S.surfPage=(S.surfPage+7)%8;
 }
 if(d1){
  if(S.mode=='MENU'&&S.gasSetup)S.gasCur=(S.gasCur+1)%11; // SELECT: move cursor within the gas-setup sub-screen
  else if(S.mode=='MENU')S.cur=(S.cur+1)%10; // SELECT: move cursor to next main-menu item
  else if(S.showLog=='list'){if(prefs.get('logs',[]).length){S.showLog='detail';S.logPage=0}}
  else if(S.showLog=='detail')S.showLog='list';
  else if(!S.showLog&&(S.mode=='DIVE'||S.mode=='SURFACE'))openGasSwitch();
 }
}
function enabledGasIndices(){const a=[];for(let i=0;i<3;i++)if(S.gasList[i].enabled)a.push(i);return a}
// A single gas's A/E label: deco math is identical for air and EANx21 (same 21%O2/79%N2 either
// way -- compressor fill vs nitrox blend doesn't change tissue loading), so this is a *record*
// distinction only. At exactly 21% O2 / 0% He the diver's own labelE flag (set in gas setup)
// decides A vs E, so a deliberately-blended EANx21 fill can be logged as E, not lumped in as A.
// Any other composition always shows E regardless of the flag -- there's no ambiguity to tag there.
function gasLabel(g){return(g.o2===21&&g.he===0)?(g.labelE?'E':'A'):'E'}
// OC's displayed sub-label is derived, not stored: >1 enabled gas = OCM (open circuit multigas),
// otherwise it's just that one gas's label (see gasLabel above). gasList[0] can never be disabled
// (see gasSetupAdjust), so en.length is always >=1.
function ocLabel(){const en=enabledGasIndices();if(en.length>1)return'OCM';return gasLabel(S.gasList[en[0]||0])}
// Best-mix recommendation: among the enabled gases, pick the richest O2 whose PPO2
// stays within the diver's set limit at the given depth (richer O2 = shorter deco/less
// N2 loading, so "richest that's still safe" is the right pick). If every enabled gas
// would exceed the PPO2 limit at that depth (too deep for all of them), fall back to the
// leanest O2 (deepest MOD) instead, since that's the safest of the available options.
function bestGasIdx(depth){
 const en=enabledGasIndices();if(!en.length)return -1;
 const lim=S.ppo2/100,pa=pAt(depth);
 let best=-1,bestO2=-1;
 for(const i of en){const g=S.gasList[i];if(g.o2/100*pa<=lim+1e-6&&g.o2>bestO2){bestO2=g.o2;best=i}}
 if(best<0){let leanO2=101;for(const i of en){const g=S.gasList[i];if(g.o2<leanO2){leanO2=g.o2;best=i}}}
 return best;
}
// Gas switching is now a deliberate two-step action (open the picker, then confirm) rather
// than an instant switch on a single SELECT press, specifically so an accidental press
// mid-dive can't change gas. Only offered at all when 2+ gases are actually enabled.
function openGasSwitch(){
 const en=enabledGasIndices();
 if(en.length<2){S.toast='ONLY 1 GAS SET';S.toastUntil=HAL.millis()+1500;return}
 S.gasSwitchUI=true;
 const idx=en.indexOf(S.activeGas);
 S.gasSwitchSel=en[(idx+1)%en.length]; // default highlight: the "next" gas, matching the old one-press behavior
}
function cycleGasSwitchSel(dir){
 const en=enabledGasIndices();
 let idx=en.indexOf(S.gasSwitchSel);
 idx=(idx+dir+en.length)%en.length;
 S.gasSwitchSel=en[idx];
}
function confirmGasSwitch(){
 const g=S.gasList[S.gasSwitchSel];
 S.activeGas=S.gasSwitchSel;S.fo2=g.o2;S.heFrac=g.he/100;S.dirty=true;
 if(S.mode=='DIVE')S.gasesUsedThisDive.push({fo2:g.o2,he:g.he,label:gasLabel(g)});
 S.toast=`GAS ${g.o2}${g.he?'/'+g.he:''}%`;S.toastUntil=HAL.millis()+1500;
 beep('GAS SWITCH',1100,120);
 S.gasSwitchUI=false;
}
function edge(k,v,n,f,ms){if(v&&!pw[k])beep(n,f,ms);pw[k]=v}
const ALARM_PATTERNS={
 lock:  [[400,300],[0,150],[400,300],[0,600]],
 ceil:  [[1800,100],[0,60],[1800,100],[0,60],[1800,100],[0,500]],
 ppo2:  [[2000,150],[0,80],[1200,150],[0,600]],
 ppo2lo:[[700,200],[0,120],[700,200],[0,120],[700,200],[0,600]], // slower/lower-pitched than high-PPO2, distinguishable by ear
 ascent:[[1000,90],[0,50],[1300,90],[0,50],[1600,90],[0,500]],
 cns:   [[900,180],[0,140],[1400,180],[0,700]],
 table: [[1400,150],[0,100],[1400,150],[0,600]],
 fault: [[500,80],[0,80],[500,80],[0,80],[500,80],[0,600]],
};
const CRIT_LABELS={lock:'COMPUTER LOCKED',ceil:'CEILING BREACH!',ppo2:'HIGH PPO2!',ppo2lo:'LOW PPO2!',ascent:'ASCENT TOO FAST!',cns:'CNS HIGH!',table:'TABLE LIMIT EXCEEDED',fault:'SENSOR FAULT'};
// All mandatory/critical alarms share one color (red) and all advisory warnings share
// another (amber, see WM/'minor' below) -- two consistent tiers rather than a different
// hue per alarm type, so the diver reads "critical" vs "advisory" at a glance underwater
// without having to learn/recall a color-per-alarm mapping.
const ALARM_COL={lock:'crit',ceil:'crit',ppo2:'crit',ppo2lo:'crit',ascent:'crit',cns:'crit',table:'crit',fault:'crit'};
const GAS_COL=['gas1','gas2','gas3']; // per-gas-slot accent color, keyed by gas index
function themeFor(){if(S.mode=='DIVE')return'bgDive';if(S.mode=='MENU'||S.mode=='SYNC')return'bgMenu';return'bg'} // subtle background shift by context (surface/dive/menu)
function activeAlarmKey(){ // priority order: most safety-critical first
 if(S.compLocked)return'lock';
 if(S.wpLow)return'ppo2lo'; // hypoxia risk ranks above high-PPO2/ceiling: unconsciousness can happen with no other warning signs
 if(S.wc)return'ceil';
 if(S.wp)return'ppo2';
 if(S.wa)return'ascent';
 if(S.wcns)return'cns';
 if(S.texc)return'table';
 if(S.fault)return'fault';
 return null;
}
function driveAlarmSound(now){
 const key=activeAlarmKey();
 if(key!==S.alarmActive){ // (re)start the pattern from step 0 whenever the active alarm changes
  S.alarmActive=key;S.alarmStep=0;S.alarmNextAt=0;S.alarmCycle=0;
  if(key){
   beep(key,ALARM_PATTERNS[key][0][0]||0,ALARM_PATTERNS[key][0][1]); // log the first note as the alert-log entry
   S.popupMsg=CRIT_LABELS[key];S.popupUntil=now+4000; // one-time forced popup; ongoing sound/border/icon are separate and unaffected by this
  }
 }
 if(!key||S.mode!=='DIVE')return; // repeating chimes only while diving; on the surface (e.g. a 48h lockout) only the one-time beep + popup fire
 if(now>=S.alarmNextAt){
  const pat=ALARM_PATTERNS[key],[freq,dur]=pat[S.alarmStep];
  if(freq>0&&!(S.alarmStep===0&&S.alarmCycle===0))chime(freq,dur); // only the very first note was already played by the transition beep
  S.alarmNextAt=now+dur;
  S.alarmStep=(S.alarmStep+1)%pat.length;
  if(S.alarmStep===0)S.alarmCycle++;
 }
}
function bookmark(){S.bookmarks.push({d:+S.depth.toFixed(1),t:S.dur,h:S.hdg});if(S.bookmarks.length>6)S.bookmarks.shift();S.toast='BOOKMARK SAVED';S.toastUntil=HAL.millis()+1500;beep('BOOKMARK',1600,120)}
function handleDown(){
 if(S.showLog=='list'){const n=prefs.get('logs',[]).length;S.logCur=Math.min(Math.max(0,n-1),S.logCur+1);return}
 if(S.showLog=='detail'){S.logPage=(S.logPage+1)%3;return}
 // (S.mode=='MENU' is unreachable here -- buttons() handles DOWN in MENU via menuAdjust/gasSetupAdjust before ever calling handleDown)
 if(S.mode=='SURFACE'&&!S.compLocked)S.surfPage=(S.surfPage+1)%8;
 else if(S.mode=='DIVE'){
  if(S.view==4){if(S.refHdg==null){S.refHdg=S.hdg;beep('BEARING SET',900,150)}else{S.refHdg=null;beep('BEARING CLEAR',500,150)}}
  else bookmark();
 }}
function alerts(){edge('ss',S.ss,'SAFETY STOP',700,150);edge('ds',S.ds,'DEEP STOP',700,150);edge('wbat',S.wbat,'BATTERY LOW',600,350);edge('wndl2',S.wndl2,'NDL 2 MIN',1000,250)}
const T=(f,x,y,s,col)=>{u8g2.setFont(f);u8g2.drawStr(x,y,s,col)},mmss=s=>String(Math.floor(s/60)).padStart(2,'0')+':'+String(Math.floor(s%60)).padStart(2,'0');
function dots(view){for(let i=0;i<5;i++){const y=44+i*4;if(view==i)u8g2.drawBox(125,y,3,3);else u8g2.drawBox(126,y+1,1,1)}}
/* Tissue chart: bar = loading vs surface M-value (0% = surface pressure, 100% = raw M-value); dotted line = GF hi (NDL limit) */
function vsi(now){const cx0=1,yc=31,half=11;const col=S.wa?(blinkOn(now,180)?'crit':'bg'):'primary';u8g2.drawBox(cx0,yc,4,1,col);
 const v=Math.max(-1,Math.min(1,S.rate/18)),h=Math.round(Math.abs(v)*half);
 if(h>0){if(v>0)u8g2.drawBox(cx0,yc-h,4,h,col);else u8g2.drawBox(cx0,yc+1,4,h,col)}}
// Shared by diveBody() and the compass page so both show the ongoing ambient
// indicator (ring flash) for as long as a critical condition lasts. Distinct
// from the one-time forced popup, which fires once per new alarm and can be dismissed.
function gasHdr(){return S.heFrac>0?`${S.fo2}/${Math.round(S.heFrac*100)}%`:`O2:${S.fo2}%`}
function menuLabel(i,fo,pp,lk){
 switch(i){
  case 0:return `PPO2:${pp}`;
  case 1:return `DeepStop:${S.deep?'ON':'OFF'}`;
  case 2:return S.lock?`${ALG[S.algo]} LK${lk}h`:`Algo:${ALG[S.algo]}`;
  case 3:return `Mode:${S.diveMode==='OC'?ocLabel():S.diveMode}`;
  case 4:return 'PC Sync';
  case 5:return 'Dive Log';
  case 6:return `GFlo:${Math.round(S.ugf0*100)}`;
  case 7:return `GFhi:${Math.round(S.ugf1*100)}`;
  case 8:return `Units:${isImp()?'Imperial':'Metric'}`;
  case 9:return 'Gas Setup >';
 }
}
function critList(){
 const WC=[];if(S.compLocked)WC.push('COMPUTER LOCKED');if(S.fault)WC.push('SENSOR FAULT');if(S.wpLow)WC.push('LOW PPO2!');if(S.wc)WC.push('CEILING BREACH!');if(S.wp)WC.push('HIGH PPO2!');if(S.wa)WC.push('ASCENT TOO FAST!');if(S.texc)WC.push('TABLE LIMIT EXCEEDED');if(S.wcns)WC.push('CNS HIGH!');
 return WC;
}
function drawCritBorder(now,WC){
 if(WC.length&&blinkOn(now,180)){const col=ALARM_COL[activeAlarmKey()]||'crit';u8g2.drawHLine(0,0,128,col);u8g2.drawHLine(0,63,128,col);u8g2.drawBox(0,0,2,64,col);u8g2.drawBox(126,0,2,64,col)}
}
// One-time forced popup: appears in front of whatever page is showing when a
// NEW critical alarm starts, auto-clears after 4s or on any button press.
// Does not recur while the same alarm stays active, and never blocks paging.
function drawAlertPopup(now){
 if(now>=S.popupUntil)return;
 const col=ALARM_COL[S.alarmActive]||'crit';
 u8g2.drawBox(2,18,124,28,col);
 T('6x12',6,32,S.popupMsg,'bg');
 T('5x7',6,44,'press any button','bg');
}
function chart(){const y0=57,H=11;for(let x=0;x<120;x+=3){u8g2.drawBox(x,y0,1,1,'dim');u8g2.drawBox(x,y0-Math.round(S.gf1*H),1,1,'dim')}
 for(let i=0;i<16;i++){const m=A[i]+P_SURF/B[i]-P_SURF,g=Math.max(-.4,Math.min(1.1,(P[i]-P_SURF)/m)),x=i*7+4;
  const col=g>=1.0?'crit':g>=0.7?'minor':'ok';
  if(g>=0){const h=Math.max(1,Math.round(g*H));u8g2.drawBox(x,y0-h,5,h,col)}else{const h=Math.max(1,Math.round(-g*H));u8g2.drawBox(x,y0+1,5,h,'dim')}}}
// Rotating-card compass: N/E/S/W and the reference bearing move around the
// ring as you turn; a single fixed pointer at the top ("lubber line") always
// means "straight ahead" (the direction you're currently facing). This is
// the standard handheld-compass convention and avoids reading a moving
// needle against fixed labels.
function compassScreen(now,surfacePreview){
 const F='6x10';
 if(surfacePreview){T(F,0,9,'COMPASS PREVIEW','dim');}
 else{T(F,0,9,`DEPTH ${dS(S.depth)}${dU()}`,'dim');const cs=`CEIL ${cS(S.ceil)}${dU()}`;T(F,70,9,S.deco?(cs.length>9?`CEIL ${cS(S.ceil)}`:cs):`NDL ${S.ndl}`,S.ndl<=2&&!S.deco?'minor':'dim');}
 u8g2.drawHLine(0,11,120,'dim');
 const cx0=55,cy0=39,r=18;
 u8g2.drawCircle(cx0,cy0,r,'dim');
 const rot=trueBearing=>((trueBearing-S.hdg-90)*Math.PI/180); // screen angle for a fixed compass bearing, given current heading
 [[0,'N'],[90,'E'],[180,'S'],[270,'W']].forEach(([ang,lbl])=>{const a=rot(ang);T('5x7',cx0+Math.cos(a)*(r+3)-3,cy0+Math.sin(a)*(r+3)+3,lbl,'dim')});
 [45,135,225,315].forEach(ang=>{const a=rot(ang),x0=cx0+Math.cos(a)*(r-4),y0=cy0+Math.sin(a)*(r-4),x1=cx0+Math.cos(a)*r,y1=cy0+Math.sin(a)*r;u8g2.drawLine(x0,y0,x1,y1,'dim')});
 u8g2.drawLine(cx0,cy0,cx0,cy0-r,'primary'); // fixed lubber line: always "ahead"
 if(S.refHdg!=null){
  const ar=rot(S.refHdg);
  u8g2.drawLine(cx0+Math.cos(ar)*(r-6),cy0+Math.sin(ar)*(r-6),cx0+Math.cos(ar)*r,cy0+Math.sin(ar)*r,'minor'); // bearing marker on the ring
  let delta=((S.hdg-S.refHdg+540)%360)-180;
  T('5x7',0,60,`REF ${S.refHdg}* ${delta>=0?'+':''}${delta.toFixed(0)}* HDG ${String(S.hdg).padStart(3,'0')}*`,'minor');
 } else T('5x7',0,60,`HDG ${String(S.hdg).padStart(3,'0')}*`,'dim');
}
function render(now){u8g2.clearBuffer(themeFor());const F='6x10',fo=S.fo2,pp=(S.ppo2/100).toFixed(2),lk=Math.ceil(S.lock/3600);
 if(S.gasSwitchUI){
  drawCritBorder(now,critList());
  T(F,0,9,'SWITCH GAS');u8g2.drawHLine(0,11,128);
  const best=bestGasIdx(S.depth);
  enabledGasIndices().forEach((gi,row)=>{
   const g=S.gasList[gi];
   T('5x7',0,22+row*9,`${S.gasSwitchSel==gi?'>':' '}Gas${gi+1}: ${g.o2}${g.he?'/'+g.he:''}%${gi===S.activeGas?' (current)':''}`,GAS_COL[gi]);
   if(gi===best)T('5x7',118,22+row*9,'B','ok'); // "B" = best mix for the current depth
  });
  T('5x7',0,63,'SEL:confirm MODE:cancel');
 }
 else if(S.mode=='SURFACE'&&S.showLog=='list'){
  const all=prefs.get('logs',[]);const l=all.slice().reverse();
  T(F,0,9,`DIVE LOG (${all.length})`);u8g2.drawHLine(0,11,128);
  if(!l.length)T('5x7',0,22,'No dives logged yet');
  l.slice(0,6).forEach((e,i)=>T('5x7',0,20+i*8,`${S.logCur==i?'>':' '}${mmss(e.durSec)} ${dS(e.maxD)}${dU()} O2${e.gases[0].fo2}% ${ALG[e.algo].slice(0,4)}${e.deco?' DECO':''}`));
  T('5x7',0,62,'SEL: open  MODE: back');
 }
 else if(S.mode=='SURFACE'&&S.showLog=='detail'){
  const all=prefs.get('logs',[]);const l=all.slice().reverse();const e=l[Math.min(S.logCur,l.length-1)];
  if(!e){S.showLog='list'}
  else{
   T(F,0,9,`DIVE #${all.length-S.logCur} (${S.logPage+1}/3)`);u8g2.drawHLine(0,11,128);
   if(S.logPage==0){
    T(F,0,23,`Time: ${mmss(e.durSec)}`);
    T('5x7',0,33,`Max: ${dS(e.maxD)}${dU()}  Avg: ${dS(e.avgD)}${dU()}`);
    {const parts=e.gases.slice(0,2).map(g=>`${g.fo2}${g.he?'/'+g.he:''}%`);const extra=e.gases.length>2?`+${e.gases.length-2}`:'';T('5x7',0,43,`Gas: ${parts.join(',')}${extra}`);}
    T('5x7',0,53,`Algo: ${ALG[e.algo]} T:${tS(e.avgTemp)}`);
   } else if(S.logPage==1){
    T(F,0,23,`GF used: ${Math.round(e.gfUsed[0]*100)}/${Math.round(e.gfUsed[1]*100)}`);
    T('5x7',0,33,e.surfIntervalSec==null?'Surface int: first dive':`Surface int: ${mmss(e.surfIntervalSec)}`);
    T('5x7',0,43,`Surface pres: ${e.surfPressure} bar`);
    T('5x7',0,53,'Tissue N2 (start->end):');
    for(let i=0;i<16;i++){const x=i*7+4,sH=Math.max(1,Math.round(Math.min(1,(e.tissueStart.n2[i]-P_SURF)/1.5)*8)),eH=Math.max(1,Math.round(Math.min(1,(e.tissueEnd.n2[i]-P_SURF)/1.5)*8));u8g2.drawBox(x,63-sH,2,sH,'dim');u8g2.drawBox(x+3,63-eH,2,eH,'primary')}
   } else {
    if(e.profile.length<2)T(F,0,35,'No profile recorded');
    else{
     const maxD=Math.max(0.1,e.maxD),n=e.profile.length;
     T('5x7',0,17,`0`,'dim');T('5x7',0,60,`${dS(maxD)}${dU()}`,'dim');
     for(let i=1;i<n;i++){
      const p0=e.profile[i-1],p1=e.profile[i];
      const x0=4+Math.round(p0.t/e.durSec*118),y0=16+Math.round(p0.d/maxD*44);
      const x1=4+Math.round(p1.t/e.durSec*118),y1=16+Math.round(p1.d/maxD*44);
      u8g2.drawLine(x0,y0,x1,y1,'primary');
     }
    }
   }
   T('5x7',0,62,'UP/DN: page  SEL: back');
  }
 }
 else if(S.mode=='SURFACE'&&S.surfPage==1){T(F,0,10,'SURFACE - STATUS');u8g2.drawHLine(0,12,128);
  T(F,0,24,`Battery: ${S.batt.toFixed(1)}%`);T(F,0,34,`CNS O2: ${S.cns.toFixed(1)}%`);
  T(F,0,44,`Comp lock: ${S.compLocked?Math.ceil(S.lockout/3600)+'h':'none'}`);
  const l=prefs.get('logs',[]);const last=l[l.length-1];
  T(F,0,54,last?`Last dive: ${mmss(last.durSec)} ${dS(last.maxD)}${dU()}`:'Last dive: none');
  T('5x7',0,63,'UP/DOWN: page  MODE: menu')}
 else if(S.mode=='SURFACE'&&S.surfPage>=2&&S.surfPage<=6){const pv=S.surfPage-2;if(pv==4)compassScreen(now,true);else diveBody(now,pv)}
 else if(S.mode=='SURFACE'&&S.surfPage==7){
  T(F,0,10,'SURFACE GF');u8g2.drawHLine(0,12,128);
  const sgf=surfaceGF(),col=sgf>=100?'crit':sgf>=80?'minor':'primary';
  T('L22',30,40,`${Math.round(sgf)}%`,col);
  T('5x7',0,52,sgf>=100?'CEILING VIOLATION':(sgf>=80?'HIGH - check ascent rate':'Normal'),col);
  T('5x7',0,63,'UP/DOWN: page  MODE: menu');
 }
 else if(S.mode=='SURFACE'){T(F,0,10,'SURFACE / PLANNER');u8g2.drawHLine(0,12,128);T(F,0,24,`NO FLY: ${Math.floor(S.nofly/3600)}h${String(Math.floor(S.nofly%3600/60)).padStart(2,'0')}m`);
  {const gh=gasHdr(),lbl=` PPO2:`;T(F,0,34,gh,GAS_COL[S.activeGas]);T(F,gh.length*6,34,lbl);const ppCol=(S.wpLow||S.wp)&&blinkOn(now,180)?'crit':'primary';T(F,(gh.length+lbl.length)*6,34,pp,ppCol);T(F,(gh.length+lbl.length)*6+pp.length*6,34,` G${S.activeGas+1}`);}
  T(F,0,44,`MOD:${dS(S.mod)}${dU()} Dives:${S.dcount}`);T(F,0,54,'ALG:'+ALG[S.algo]+(S.lock?` LK${lk}h`:''));if(S.compLocked)T('6x12',0,63,`LOCKED ${Math.ceil(S.lockout/3600)}h`,'crit');else if(S.fault)T(F,0,63,'SENSOR FAULT','crit');else{const bt=`BAT:${S.batt.toFixed(0)}% `;T(F,0,63,bt);T(F,bt.length*6,63,`CNS:${S.cns.toFixed(0)}%`,S.wcns&&blinkOn(now,180)?'crit':'primary')}}
 else if(S.mode=='MENU'&&S.gasSetup){
  T(F,0,9,'GAS SETUP');u8g2.drawHLine(0,11,128);
  const labels=[
   `Gas1 O2:${S.gasList[0].o2}%`,`Gas1 He:${S.gasList[0].he}%`,
   `Gas2 O2:${S.gasList[1].o2}%`,`Gas2 He:${S.gasList[1].he}%`,`Gas2:${S.gasList[1].enabled?'ON':'OFF'}`,
   `Gas3 O2:${S.gasList[2].o2}%`,`Gas3 He:${S.gasList[2].he}%`,`Gas3:${S.gasList[2].enabled?'ON':'OFF'}`,
   `Gas1 Label:${gasLabel(S.gasList[0])==='E'?'EANx21':'Air'}`,
   `Gas2 Label:${gasLabel(S.gasList[1])==='E'?'EANx21':'Air'}`,
   `Gas3 Label:${gasLabel(S.gasList[2])==='E'?'EANx21':'Air'}`,
  ];
  const rowGas=[0,0,1,1,1,2,2,2,0,1,2]; // which gas slot each row belongs to, for per-gas coloring
  const total=11,winSize=6,start=Math.max(0,Math.min(S.gasCur-2,total-winSize));
  for(let row=0;row<winSize;row++){
   const i=start+row;if(i>=total)break;
   T('5x7',0,18+row*7,`${S.gasCur==i?'>':' '}${labels[i]}`,GAS_COL[rowGas[i]]);
  }
  T('5x7',0,63,'MODE: back to menu');
 }
 else if(S.mode=='MENU'){
  T(F,0,9,'SETTINGS MENU');u8g2.drawHLine(0,11,128);
  const total=10,winSize=6,start=Math.max(0,Math.min(S.cur-2,total-winSize));
  for(let row=0;row<winSize;row++){
   const i=start+row;if(i>=total)break;
   T('5x7',0,18+row*7,`${S.cur==i?'>':' '}${i+1} ${menuLabel(i,fo,pp,lk)}`);
  }
  if(S.compLocked)T('6x12',0,63,'** LOCKED **');
  else T('5x7',96,63,`${S.cur+1}/${total}`);
 }
 else if(S.mode=='SYNC'){T('6x12',10,25,'PC DEBUGGER');T('6x12',10,45,'SYNCING LOGS...')}
 else{
  if(S.view==4){compassScreen(now);dots(4);drawCritBorder(now,critList());drawAlertPopup(now);u8g2.sendBuffer();return}
  diveBody(now,S.view);
 }
 if(HAL.millis()<S.toastUntil){u8g2.drawBox(20,26,88,10,'ok');T('5x7',24,33,S.toast,'bg')}
 drawAlertPopup(now);u8g2.sendBuffer()}

// Shared by live DIVE mode and the surface preview pages. `view` is passed
// explicitly (rather than read from S.view) so the surface preview can show
// any of the 4 non-compass dive screens without S actually being in DIVE mode.
function diveBody(now,view){
  const F='6x10',fo=S.fo2,pp=(S.ppo2/100).toFixed(2);
  const WC=critList();
  const WM=[];if(S.wndl2&&!S.deco)WM.push('NDL 2 MIN');if(S.wbat)WM.push('BATTERY LOW');
  const crit=WC.length>0;
  drawCritBorder(now,WC);
  if(!crit&&WM.length&&blinkOn(now,500)){u8g2.drawHLine(0,0,128,'minor');u8g2.drawHLine(0,63,128,'minor')}
  T(F,0,9,`${String(S.hdg).padStart(3,'0')}* ${S.card}`,'dim');T('5x7',44,9,S.diveMode==='OC'?ocLabel():S.diveMode,'white');T(F,62,9,gasHdr(),GAS_COL[S.activeGas]);const tm=mmss(S.dur);T(F,128-6*tm.length,9,tm,'dim');u8g2.drawHLine(0,11,128,'dim');
  // VSI now sits left of the depth digits (not between depth and NDL); depth
  // shifts right (x=9) to clear it, on every page.
  vsi(now);
  // On the alerts/stops page (view0) the header is always the original full
  // layout, unchanged. On every other page, an active stop condition
  // compacts the NDL/CEIL slot so the per-view info below still has room:
  // DECO/deep ceiling -> ceiling shrinks to the label row, countdown grows;
  // safety/advisory stop -> NDL shrinks to the label row, small "stop mm:ss".
  const nearCeiling = S.deco && S.depth<=S.ceil+1.5; // ceiling only shrinks once genuinely within 1.5m of it
  const compact = view!=0 && (nearCeiling||S.ss||S.ds);
  if(!compact){
   T('5x7',9,19,`DEPTH ${dU()}`,'dim');T('5x7',80,19,S.deco?`CEIL ${dU()}`:'NDL min','dim');T('L22',9,41,dS(S.depth),'white');
   if(crit&&blinkOn(now,180))T('6x12',68,19,'!','crit');
   if(S.deco)T('L22',80,41,cS(S.ceil),'deco');else if(S.ndl>=99){T('L22',80,41,'99');T(F,108,41,'+')}else T('L22',80,41,''+S.ndl,S.ndl<=2?'minor':'primary');
  } else if(nearCeiling){
   T('5x7',9,19,`DEPTH ${dU()}`,'dim');T('5x7',68,19,'CEIL','dim');T('7x14',92,24,cS(S.ceil),'deco');
   T('L22',9,41,dS(S.depth),'white');
   const rem=Math.max(0,S.nmin*60-S.atDepth);
   T('5x7',64,37,'Stop','minor');T('7x14',88,37,mmss(rem),'deco');
  } else {
   T('5x7',9,19,`DEPTH ${dU()}`,'dim');T('5x7',68,19,'NDL','dim');T('7x14',88,24,''+S.ndl,'primary');
   T('L22',9,41,dS(S.depth),'white');
   if(S.ds){const rem=Math.max(0,120-S.dsEl);T('5x7',70,37,`deep ${mmss(rem)}`,'minor')} // was wrongly using the safety-stop timer even during a deep stop
   else{const rem=Math.max(0,180-S.ssEl);T('5x7',70,37,`stop ${mmss(rem)}`,'minor')}
  }
  u8g2.drawHLine(0,43,128,'dim');dots(view);
  const banner=crit?WC[Math.floor(now/1500)%WC.length]:(WM.length?WM[Math.floor(now/1500)%WM.length]:null);
  const bannerCol=crit?(blinkOn(now,180)?(ALARM_COL[activeAlarmKey()]||'crit'):'bg'):(WM.length?(blinkOn(now,500)?'minor':'bg'):null);
  if(view!=0){}
  else if(banner)T('6x12',0,58,banner,bannerCol);
  else if(S.deco){const a=`STOP ${cS(S.nstop)}${dU()} ${S.nmin}min TTS${S.tts}`;T(F,0,58,a.length<=21?a:`STOP ${cS(S.nstop)}${dU()} ${S.nmin}' TTS${S.tts}`,'deco')}
  else if(S.ss)T(F,0,58,`STOP ${isImp()?'15ft':'5m'} | ${mmss(Math.max(0,180-S.ssEl))}`,'minor');
  else if(S.ds)T(F,0,58,`DEEP ${dI(S.dsD)}${dU()} | ${mmss(Math.max(0,120-S.dsEl))}`,'minor');
  else chart();
  if(view==1){const gh=gasHdr(),lbl=' PPO2:';T(F,0,53,gh,GAS_COL[S.activeGas]);T(F,gh.length*6,53,lbl);
   const ppCol=(S.wpLow||S.wp)&&blinkOn(now,180)?'crit':'primary';
   T(F,(gh.length+lbl.length)*6,53,S.ppo2n.toFixed(2),ppCol);
   const best=bestGasIdx(S.depth),line=`MOD:${dS(S.mod)}${dU()} G${S.activeGas+1}/3`;T(F,0,63,line,GAS_COL[S.activeGas]);
   if(best>=0&&best!==S.activeGas)T(F,line.length*6,63,` B${best+1}`,'ok'); // a richer/leaner gas than the active one is better for this depth
  }
  else if(view==2){
   if(S.deco){let sc='';for(const [d,m] of S.stops){const q=`${cS(d)}${dU()}/${m}'`;if((sc+(sc?' ':'')+q).length>25)break;sc+=(sc?' ':'')+q}T('5x7',0,53,sc||'calc...');const pre=`TTS:${S.tts}m CNS:`;T(F,0,63,pre);T(F,pre.length*6,63,`${S.cns.toFixed(0)}%`,S.wcns&&blinkOn(now,180)?'crit':'primary')}
   else{T(F,0,53,`NDL:${S.ndl} @depth:${mmss(S.atDepth)}`);const pre=`${S.algo==2&&!S.texc?ALG[2]:`GF${Math.round(S.gf0*100)}/${Math.round(S.gf1*100)}`} CNS:`;T(F,0,63,pre);T(F,pre.length*6,63,`${S.cns.toFixed(0)}%`,S.wcns&&blinkOn(now,180)?'crit':'primary')}}
  else if(view==3){const rl=S.rate>0.05?'ASCN':(S.rate<-0.05?'DESC':'STAT'),rv=Math.abs(S.rate);T(F,0,53,`MAX:${dS(S.maxD)}${dU()} ${rl}:${rS(rv)}`);T(F,0,63,`TEMP:${tS(HAL.temp())}`)}
}
function sampleProfile(now){
 const elMs=now-S.dstart;
 if(S.profile.length===0||elMs-S.profile[S.profile.length-1].t*1000>=S.profileIv*1000){
  S.profile.push({t:Math.round(elMs/1000),d:+S.depth.toFixed(1)});
  if(S.profile.length>=120){const half=[];for(let i=0;i<S.profile.length;i+=2)half.push(S.profile[i]);S.profile=half;S.profileIv*=2}
 }
}
function tick(now,dt){sensors();const fo=S.fo2/100;S.ppo2n=S.pabs*fo;S.mod=Math.max(0,Math.floor((S.ppo2/100/fo-P_SURF)*MPB*10)/10);
 S.batt=Math.max(0,S.batt-dt/86400);
 if(S.ppo2n>0.5)S.cns=Math.min(150,S.cns+cnsRatePerMin(S.ppo2n)*dt/60);else S.cns=Math.max(0,S.cns-dt*100/5400);
 if(S.mode=='DIVE'){if(S.atAnchor==0||Math.abs(S.depth-S.atAnchor)>1.5){S.atAnchor=S.depth;S.atDepth=0}else S.atDepth+=dt}else{S.atAnchor=0;S.atDepth=0}
 if(!S.fault){const heFr=S.mode=='DIVE'?(S.heFrac||0):0;leg(P,PHE,S.prev,S.pabs,dt,S.mode=='DIVE'?1-fo-heFr:FN2_AIR,heFr);S.prev=S.pabs;rate(now);if(S.mode=='DIVE'){S.maxD=Math.max(S.maxD,S.depth);S.dur=Math.floor((now-S.dstart)/1000);S.avgAccD+=S.depth*dt;S.avgAccT+=dt;S.avgAccTemp+=HAL.temp()*dt;sampleProfile(now)}upd(dt,now);if(S.mode=='DIVE')stops(dt)}
 if(S.mode!='DIVE'){if(!S.fault&&S.depth>=.8){if(!S.desc)S.desc=now;if(now-S.desc>=3000){startDive(S.desc);S.desc=0}}else S.desc=0}else{if(S.depth<.3){if(!S.surf)S.surf=now;if(now-S.surf>=60000)endDive()}else S.surf=0}
 if(S.mode=='SYNC'&&now>=S.sync)S.mode='SURFACE';
 if(now-lastS>=1000){lastS=now;saveAcc++;if(S.mode!='DIVE'){if(S.nofly>0)S.nofly--;if(S.lock>0)S.lock--;if(S.lockout>0){S.lockout--;if(S.lockout==0)S.compLocked=false}if(!S.nofly&&!S.lock&&(S.dcount||S.viol)){S.dcount=0;S.viol=0;saveState()}}if(saveAcc>=(S.mode=='DIVE'?120:300)){saveAcc=0;if(S.mode=='DIVE'||S.nofly>0||S.lockout>0)saveState()}}
 alerts();driveAlarmSound(now);render(now)}
function setup(){const s=prefs.get('set',{});S.fo2=s.fo2||21;S.ppo2=s.ppo2||140;S.deep=!!s.deep;S.algo=s.algo||0;S.units=s.units||'met';
 S.diveMode=DIVE_MODES.includes(s.diveMode)?s.diveMode:'OC';
 if(s.gasList&&s.gasList.length===3)S.gasList=s.gasList;
 S.activeGas=s.activeGas||0;S.heFrac=typeof s.heFrac==='number'?s.heFrac:0;
 S.fo2=S.gasList[S.activeGas].o2;S.heFrac=S.gasList[S.activeGas].he/100; // keep live fo2/heFrac in sync with the stored active slot
 if(!loadState())P=P.map(()=>PAIR);
 let mv=0;for(let i=0;i<30;i++)mv+=analogMv(0);mv/=30;if(mv<50||mv>2450)S.fault=true;else if(!(S.zeroValid&&Math.abs(mv-S.zero)>16.7)){S.zero=mv;S.zeroValid=true}S.prev=P_SURF}
function loop(now,dt){buttons();tick(now,dt)}
return{setup,loop,vars:()=>({...S,P:P.slice(),PHE:PHE.slice(),tadj}),A,B,A_HE,B_HE,
 _set:(k,v)=>{S[k]=v}} // test-only hook; stage 2 (gas list UI) will set S.heFrac for real instead
}