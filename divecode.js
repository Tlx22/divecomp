function FW(HAL){
const {analogMv,u8g2,Serial,prefs,beep}=HAL;
const P_SURF=1.013,PH2O=0.0627,FN2_AIR=0.7902,MPB=10,ASC=9;
const HT=[5,8,12.5,18.5,27,38.3,54.3,77,109,146,187,239,305,390,498,635];
const A=[1.1696,1,0.8618,0.7562,0.62,0.5043,0.441,0.4,0.375,0.35,0.3295,0.3065,0.2835,0.261,0.248,0.2327];
const B=[0.5578,0.6514,0.7222,0.7825,0.8126,0.8434,0.8693,0.891,0.9092,0.9222,0.9319,0.9403,0.9477,0.9544,0.9602,0.9653];
const K=HT.map(h=>Math.LN2/(h*60)),PAIR=(P_SURF-PH2O)*FN2_AIR;let P=HT.map(()=>PAIR);
const TD=[10,12,14,16,18,20,22,25,30,35,40,42],TN=[219,147,98,72,56,45,37,29,20,14,9,8];let tadj=TN.slice();
const ALG=["Buhlmann","RGBM-sim","PADI-tbl"];
const S={mode:'SURFACE',view:0,algo:0,fo2:21,ppo2:140,deep:false,dirty:false,zero:250,zeroValid:false,gauge:0,depth:0,maxD:0,rate:0,pabs:P_SURF,prev:P_SURF,ppo2n:.21,mod:0,fault:false,bad:0,dstart:0,dur:0,dcount:0,viol:0,nofly:0,lock:0,ndl:99,deco:false,ceil:0,tts:0,nstop:0,nmin:0,texc:false,dDeco:false,dViol:false,breach:0,gf0:.3,gf1:.7,ss:false,ssDone:false,ssEl:0,ds:false,dsDone:false,dsEl:0,dsD:0,cur:0,desc:0,surf:0,sync:0,wc:false,wa:false,wp:false,hdg:0,card:'N'};
let lastSim=0,lastS=0,saveAcc=0,lastR=0;const hist=[],pw={};
const pAt=d=>P_SURF+d/MPB;
function leg(Q,p0,p1,s,f){if(s<=0)return;const a=(p0-PH2O)*f,b=(p1-PH2O)*f,R=(b-a)/s;for(let i=0;i<16;i++){const k=K[i],e=Math.exp(-k*s);Q[i]=a+R*(s-1/k)-(a-Q[i]-R/k)*e}}
const maxTol=(Q,g)=>Math.max(0,...Q.map((p,i)=>(p-A[i]*g)/(g/B[i]-g+1)));
function gfNow(){if(S.algo==1)return[.3,Math.max(.45,.75-.05*Math.min(S.dcount,3)-.05*Math.min(S.viol,3))];return[.3,.7]}
function ndlCalc(Q,pa,f,gh){const pi=(pa-PH2O)*f;let best=9999*60;for(let i=0;i<16;i++){const M=P_SURF+gh*(A[i]+P_SURF/B[i]-P_SURF);if(Q[i]>=M)return 0;if(pi<=M)continue;best=Math.min(best,Math.log((pi-Q[i])/(pi-M))/K[i])}return best/60}
function ceilCalc(Q,g){if(maxTol(Q,g[1])<=P_SURF+1e-4)return 0;const pl=maxTol(Q,g[0]),fm=Math.max(3,Math.ceil((pl-P_SURF)*MPB/3-1e-9)*3),pf=pAt(fm);for(let d=3;d<=fm+.01;d+=3){const pd=pAt(d),gg=g[1]+(g[0]-g[1])*(pd-P_SURF)/(pf-P_SURF);if(maxTol(Q,gg)<=pd+1e-4)return d}return fm}
function simDeco(g,f){const Q=P.slice();let d=S.depth,t=0,fc=-1,fmn=0,fin=false;for(let it=0;it<600;it++){const c=ceilCalc(Q,g);if(fc<0)fc=c;if(c<=0){t+=d/ASC*60;fin=true;break}if(d>c+.01){const s=(d-c)/ASC*60;leg(Q,pAt(d),pAt(c),s,f);t+=s;d=c}else{d=c;leg(Q,pAt(c),pAt(c),60,f);t+=60;if(Math.abs(c-fc)<.01)fmn++}}S.tts=fin?Math.ceil(t/60):999;S.nstop=Math.max(fc,0);S.nmin=Math.max(1,fmn)}
const tIdx=d=>TD.findIndex(x=>d<=x);
function buildTbl(){const f=1-S.fo2/100,PA=HT.map(()=>PAIR);for(let j=0;j<12;j++){const pa=pAt(TD[j]),fr=ndlCalc(PA,pa,f,.7),nw=ndlCalc(P,pa,f,.7),r=fr<.5?1:Math.min(1,nw/fr);tadj[j]=Math.floor(TN[j]*Math.max(0,r))}}
function saveState(){prefs.put('st',{P:P.slice(),nofly:S.nofly,lock:S.lock,dc:S.dcount,viol:S.viol,zero:S.zero})}
function loadState(){const s=prefs.get('st',null);if(!s||!s.P||s.P.length!=16||s.P.some(x=>!(x>.3&&x<8)))return false;P=s.P.slice();S.nofly=s.nofly;S.lock=s.lock;S.dcount=s.dc;S.viol=s.viol;if(s.zero>50&&s.zero<2450){S.zero=s.zero;S.zeroValid=true}return true}
function saveSet(){prefs.put('set',{fo2:S.fo2,ppo2:S.ppo2,deep:S.deep,algo:S.algo});S.dirty=false}
function sensors(){let mv=0;for(let i=0;i<10;i++)mv+=analogMv(0);mv/=10;
 if(mv<50||mv>2450){if(S.bad<255)S.bad++;if(S.bad>=10)S.fault=true}else{S.bad=0;S.fault=false;let g=(mv-S.zero)/1000/2*12;if(S.mode!='DIVE'&&Math.abs(g)<.03)S.zero+=.002*(mv-S.zero);if(g<0)g=0;S.gauge+=.3*(g-S.gauge);S.depth=S.gauge*MPB;S.pabs=P_SURF+S.gauge}
 const h=HAL.heading();S.hdg=Math.round(h)%360;S.card=["N","NE","E","SE","S","SW","W","NW"][Math.floor((h+22.5)/45)%8]}
function rate(now){if(now-lastR>=1000){lastR=now;hist.push(S.depth);if(hist.length>6)hist.shift();S.rate=hist.length==6?(hist[0]-S.depth)*12:0}}
function startDive(t){if(S.dirty)saveSet();S.mode='DIVE';S.view=0;S.dstart=t;S.dur=0;S.maxD=S.depth;S.surf=0;S.ss=S.ssDone=S.ds=S.dsDone=false;S.ssEl=S.dsEl=0;S.dDeco=S.dViol=false;S.breach=0;S.deco=false;S.ceil=0;S.texc=false;S.lock=86400;if(S.algo==2)buildTbl();saveState();beep('DIVE START',880,150)}
function endDive(){S.dur=Math.floor((S.surf-S.dstart)/1000);S.dcount++;if(S.dViol)S.viol=Math.min(255,S.viol+1);const tg=(S.dDeco||S.dViol)?86400:(S.dcount>1?64800:43200);S.nofly=Math.max(S.nofly,tg);S.lock=86400;const l=prefs.get('logs',[]);l.push([S.dur,+S.maxD.toFixed(1),S.fo2,S.algo,S.dDeco?1:0]);prefs.put('logs',l.slice(-100));saveState();S.mode='SURFACE';S.surf=0;beep('DIVE END',660,300)}
function upd(dt,now){
 if(S.mode!='DIVE'){S.ndl=99;S.deco=false;S.ceil=0;S.tts=0;S.texc=false;S.wc=S.wa=S.wp=false;return}
 const g=gfNow();S.gf0=g[0];S.gf1=g[1];const f=1-S.fo2/100;S.texc=false;
 if(S.algo==2){const ix=tIdx(S.maxD),el=Math.ceil(S.dur/60);if(ix<0||el>=tadj[ix]){S.texc=true;S.dViol=true}else{S.ndl=Math.min(99,tadj[ix]-el);S.deco=false;S.ceil=0;S.tts=Math.ceil(S.depth/ASC)}}
 if(S.algo!=2||S.texc){S.ceil=ceilCalc(P,g);S.deco=S.ceil>0;if(S.deco){S.ndl=0;S.dDeco=true;if(!lastSim||now-lastSim>=2000){simDeco(g,f);lastSim=now||1}}else{S.ndl=Math.min(99,Math.floor(ndlCalc(P,S.pabs,f,g[1])));S.tts=Math.ceil(S.depth/ASC)}}
 S.wc=S.deco&&S.depth<S.ceil-.5;if(S.wc){S.breach+=dt;if(S.breach>10)S.dViol=true}else S.breach=0;
 S.wa=S.rate>9;if(S.rate>12)S.dViol=true;S.wp=S.ppo2n>=S.ppo2/100}
function stops(dt){if(S.deco){S.ss=S.ds=false;return}
 if(S.deep&&S.maxD>=18&&!S.dsDone){S.dsD=S.maxD/2;S.ds=S.depth<S.maxD-3&&Math.abs(S.depth-S.dsD)<=1.5;if(S.ds){S.dsEl+=dt;if(S.dsEl>=120){S.dsDone=true;S.ds=false;beep('DEEP STOP DONE',1000,200)}}}
 if(S.maxD>=10&&!S.ssDone){if(S.depth>=3&&S.depth<=6)S.ss=true;else if(S.depth<2.5||S.depth>7){S.ss=false;S.ssEl=0}if(S.ss){S.ssEl+=dt;if(S.ssEl>=180){S.ssDone=true;S.ss=false;beep('SAFETY STOP DONE',1000,200)}}}}
function buttons(){
 if(HAL.edge(2)){if(S.mode=='SURFACE')S.mode='MENU';else if(S.mode=='MENU'){if(S.dirty)saveSet();S.mode='SURFACE'}else if(S.mode=='DIVE')S.view=(S.view+1)%4}
 if(HAL.edge(0)&&S.mode=='MENU')S.cur=(S.cur+1)%5;
 if(HAL.edge(1)&&S.mode=='MENU'){switch(S.cur){case 0:S.fo2=S.fo2>=40?21:S.fo2+1;S.dirty=true;break;case 1:S.ppo2=S.ppo2>=160?100:S.ppo2+5;S.dirty=true;break;case 2:S.deep=!S.deep;S.dirty=true;break;case 3:if(!S.lock){S.algo=(S.algo+1)%3;S.dirty=true}break;case 4:S.mode='SYNC';S.sync=HAL.millis()+3000;Serial.println('LOGS '+JSON.stringify(prefs.get('logs',[])))}}}
function edge(k,v,n,f,ms){if(v&&!pw[k])beep(n,f,ms);pw[k]=v}
function alerts(){edge('wc',S.wc,'CEILING BREACH',1500,500);edge('wa',S.wa,'ASCENT TOO FAST',1200,400);edge('wp',S.wp,'HIGH PPO2',1800,500);edge('te',S.texc,'TABLE EXCEEDED',1400,500);edge('sf',S.fault,'SENSOR FAULT',400,600);edge('ss',S.ss,'SAFETY STOP',700,150);edge('ds',S.ds,'DEEP STOP',700,150)}
const T=(f,x,y,s)=>{u8g2.setFont(f);u8g2.drawStr(x,y,s)},mmss=s=>String(Math.floor(s/60)).padStart(2,'0')+':'+String(Math.floor(s%60)).padStart(2,'0');
function dots(){for(let i=0;i<4;i++){const y=46+i*5;if(S.view==i)u8g2.drawBox(125,y,3,3);else u8g2.drawBox(126,y+1,1,1)}}
/* Tissue chart: bar = loading vs surface M-value (0% = surface pressure, 100% = raw M-value); dotted line = GF hi (NDL limit) */
function chart(){const y0=57,H=11;for(let x=0;x<120;x+=3){u8g2.drawBox(x,y0,1,1);u8g2.drawBox(x,y0-Math.round(S.gf1*H),1,1)}
 for(let i=0;i<16;i++){const m=A[i]+P_SURF/B[i]-P_SURF,g=Math.max(-.4,Math.min(1.1,(P[i]-P_SURF)/m)),x=i*7+4;
  if(g>=0){const h=Math.max(1,Math.round(g*H));u8g2.drawBox(x,y0-h,5,h)}else{const h=Math.max(1,Math.round(-g*H));u8g2.drawBox(x,y0+1,5,h)}}}
function render(now){u8g2.clearBuffer();const F='6x10',fo=S.fo2,pp=(S.ppo2/100).toFixed(2),lk=Math.ceil(S.lock/3600);
 if(S.mode=='SURFACE'){T(F,0,10,'SURFACE / PLANNER');u8g2.drawHLine(0,12,128);T(F,0,24,`NO FLY: ${Math.floor(S.nofly/3600)}h${String(Math.floor(S.nofly%3600/60)).padStart(2,'0')}m`);T(F,0,34,`O2:${fo}% PPO2:${pp}`);T(F,0,44,`MOD:${S.mod.toFixed(1)}m Dives:${S.dcount}`);T(F,0,54,'ALG:'+ALG[S.algo]+(S.lock?` LK${lk}h`:''));if(S.fault)T(F,0,63,'SENSOR FAULT')}
 else if(S.mode=='MENU'){const c=i=>S.cur==i?'>':' ';T(F,0,10,'SETTINGS MENU');u8g2.drawHLine(0,12,128);T(F,0,22,`${c(0)}1 FO2 Nitrox: ${fo}%`);T(F,0,32,`${c(1)}2 PPO2 Lim: ${pp}`);T(F,0,42,`${c(2)}3 Deep Stop: ${S.deep?'ON':'OFF'}`);T(F,0,52,S.lock?`${c(3)}4 ${ALG[S.algo]} LK ${lk}h`:`${c(3)}4 Algo: ${ALG[S.algo]}`);T(F,0,62,`${c(4)}5 PC Debugger Sync`)}
 else if(S.mode=='SYNC'){T('6x12',10,25,'PC DEBUGGER');T('6x12',10,45,'SYNCING LOGS...')}
 else{const W=[];if(S.fault)W.push('SENSOR FAULT');if(S.wc)W.push('CEILING BREACH!');if(S.wp)W.push('HIGH PPO2!');if(S.wa)W.push('ASCENT TOO FAST!');if(S.texc)W.push('TABLE LIMIT EXCEEDED');
  T(F,0,9,`${String(S.hdg).padStart(3,'0')}* ${S.card}`);T(F,52,9,`O2:${fo}%`);const tm=mmss(S.dur);T(F,128-6*tm.length,9,tm);u8g2.drawHLine(0,11,128);
  T('5x7',0,19,'DEPTH m');T('5x7',80,19,S.deco?'CEIL m':'NDL min');T('L22',0,41,S.depth.toFixed(1));
  if(S.deco)T('L22',80,41,S.ceil.toFixed(0));else if(S.ndl>=99){T('L22',80,41,'99');T(F,108,41,'+')}else T('L22',80,41,''+S.ndl);
  u8g2.drawHLine(0,43,128);dots();
  if(W.length)T('6x12',0,58,W[Math.floor(now/1500)%W.length]);
  else if(S.deco)T(F,0,58,`STOP ${S.nstop.toFixed(0)}m ${S.nmin}min TTS${S.tts}`);
  else if(S.ss)T(F,0,58,`STOP 5m | ${mmss(Math.max(0,180-S.ssEl))}`);
  else if(S.ds)T(F,0,58,`DEEP ${S.dsD.toFixed(0)}m | ${mmss(Math.max(0,120-S.dsEl))}`);
  else if(S.view==0)chart();
  else if(S.view==1){T(F,0,53,`O2:${fo}% PPO2:${S.ppo2n.toFixed(2)}`);T(F,0,63,`MOD:${S.mod.toFixed(1)}m LIM:${pp}`)}
  else if(S.view==2){T(F,0,53,S.deco?`CEIL:${S.ceil}m TTS:${S.tts}m`:`NO DECO TTS:${S.tts}m`);T(F,0,63,S.algo==2&&!S.texc?ALG[2]:`${ALG[S.algo]} GF${Math.round(S.gf0*100)}/${Math.round(S.gf1*100)}`)}
  else{T(F,0,53,`MAX:${S.maxD.toFixed(1)}m ASC:${S.rate.toFixed(1)}`);T(F,0,63,`TEMP:${HAL.temp().toFixed(1)}C`)}}
 u8g2.sendBuffer()}
function tick(now,dt){sensors();const fo=S.fo2/100;S.ppo2n=S.pabs*fo;S.mod=Math.max(0,Math.floor((S.ppo2/100/fo-P_SURF)*MPB*10)/10);
 if(!S.fault){leg(P,S.prev,S.pabs,dt,S.mode=='DIVE'?1-fo:FN2_AIR);S.prev=S.pabs;rate(now);if(S.mode=='DIVE'){S.maxD=Math.max(S.maxD,S.depth);S.dur=Math.floor((now-S.dstart)/1000)}upd(dt,now);if(S.mode=='DIVE')stops(dt)}
 if(S.mode!='DIVE'){if(!S.fault&&S.depth>=.8){if(!S.desc)S.desc=now;if(now-S.desc>=3000){startDive(S.desc);S.desc=0}}else S.desc=0}else{if(S.depth<.3){if(!S.surf)S.surf=now;if(now-S.surf>=60000)endDive()}else S.surf=0}
 if(S.mode=='SYNC'&&now>=S.sync)S.mode='SURFACE';
 if(now-lastS>=1000){lastS=now;saveAcc++;if(S.mode!='DIVE'){if(S.nofly>0)S.nofly--;if(S.lock>0)S.lock--;if(!S.nofly&&!S.lock&&(S.dcount||S.viol)){S.dcount=0;S.viol=0;saveState()}}if(saveAcc>=(S.mode=='DIVE'?120:300)){saveAcc=0;if(S.mode=='DIVE'||S.nofly>0)saveState()}}
 alerts();render(now)}
function setup(){const s=prefs.get('set',{});S.fo2=s.fo2||21;S.ppo2=s.ppo2||140;S.deep=!!s.deep;S.algo=s.algo||0;if(!loadState())P=P.map(()=>PAIR);
 let mv=0;for(let i=0;i<30;i++)mv+=analogMv(0);mv/=30;if(mv<50||mv>2450)S.fault=true;else if(!(S.zeroValid&&Math.abs(mv-S.zero)>16.7)){S.zero=mv;S.zeroValid=true}S.prev=P_SURF}
function loop(now,dt){buttons();tick(now,dt)}
return{setup,loop,vars:()=>({...S,P:P.slice(),tadj}),A,B}}