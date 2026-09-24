<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>Dive Computer Simulator</title>
<style>
/* ... (your original style) ... */
</style></head><body>
<main>
<section class="card"><h1>Dive Computer Simulator</h1>
<canvas id="oled" width="512" height="256"></canvas>
<div class="row"><button class="hw" id="b0">UP / NEXT <small>[1]</small></button><button class="hw" id="b1">SELECT <small>[2]</small></button><button class="hw" id="b2">MODE / MENU <small>[3]</small></button></div>
<div class="row"><span id="buz"></span><span id="clk"></span></div>
<div id="badges"></div><h2 style="margin-top:8px">Alert log</h2><pre id="alog" style="max-height:110px"></pre>
</section>
<section class="card"><h2>Environment</h2>
<label>Target depth<input id="dep" type="range" min="0" max="60" step="0.1" value="0"><span id="depv"></span></label>
<label>Move rate<input id="rate" type="range" min="1" max="60" step="1" value="15"><span id="ratev"></span></label>
<label>Water temp<input id="tmp" type="range" min="2" max="32" step="0.5" value="26"><span id="tmpv"></span></label>
<label>Heading<input id="hdg" type="range" min="0" max="359" value="0"><span id="hdgv"></span></label>
<div class="row"><label style="display:flex;gap:6px"><input type="checkbox" id="surf"> On surface (lock depth 0)</label></div>
<div class="row" id="live"></div>
<h2>Time</h2>
<div class="row"><button id="s0">Pause</button><button id="s1" class="on">1x</button><button id="s2">2x</button><button id="s3">3x</button></div>
<label>Skip ahead<input id="skm" type="range" min="1" max="720" value="30"><span id="skv"></span></label>
<div class="row"><button id="skip">Skip ahead at current depth</button></div>
<h2>Sensor / sound</h2>
<label>Noise (mV)<input id="noi" type="range" min="0" max="20" value="1"><span id="noiv"></span></label>
<div class="row"><label style="display:flex;gap:6px"><input type="checkbox" id="flt"> Sensor fault</label><label style="display:flex;gap:6px"><input type="checkbox" id="snd" checked> Buzzer audio</label><label style="display:flex;gap:6px"><input type="checkbox" id="dbg"> Debug</label></div>
<div class="row"><button id="pc">Power cycle (keep flash)</button><button id="fr">Factory reset</button></div>
</section>
<section class="card wide" id="dbgp" hidden><h2>Debug: firmware variables</h2><div id="dv"></div>
<h2 style="margin-top:8px">Tissues (bar = N2 tension 0-4 bar; white = ambient, orange = surface M-value at GF hi)</h2><div class="ov"><canvas id="tis" width="620" height="200" style="width:620px"></canvas></div>
<h2 style="margin-top:8px">HAL / Serial</h2><pre id="hal"></pre>
</section>
<section class="card wide"><h2>Firmware (JavaScript)</h2>
<textarea id="src" spellcheck="false"></textarea>
<div class="row"><button id="ld">Load firmware</button><button id="rs">Restore default</button><span id="err"></span></div>
<small style="color:var(--mu)">Firmware = <code>function FW(HAL){...}</code>. HAL has: millis, analogMv, edge, heading, temp, u8g2, Serial, prefs, beep.</small>
</section>
</main>

<script>
// ================= FIRMWARE (fixed version) =================
function FW(HAL){
  const P_SURF=1.013,PH2O=0.0627,FN2_AIR=0.7902,MPB=10,ASC=9;
  const ASC_WARN=9, ASC_VIOL=12;

  const HT=[5,8,12.5,18.5,27,38.3,54.3,77,109,146,187,239,305,390,498,635];
  const A=[1.1696,1,0.8618,0.7562,0.62,0.5043,0.441,0.4,0.375,0.35,0.3295,0.3065,0.2835,0.261,0.248,0.2327];
  const B=[0.5578,0.6514,0.7222,0.7825,0.8126,0.8434,0.8693,0.891,0.9092,0.9222,0.9319,0.9403,0.9477,0.9544,0.9602,0.9653];

  const K=HT.map(h=>Math.LN2/(h*60));
  const PAIR=(P_SURF-PH2O)*FN2_AIR;
  let P=HT.map(()=>PAIR);

  const TD=[10,12,14,16,18,20,22,25,30,35,40,42],TN=[219,147,98,72,56,45,37,29,20,14,9,8];
  let tadj=TN.slice();

  const ALG=["Buhlmann","RGBM-sim","PADI-tbl"];

  let S={
    mode:'SURFACE', view:0, algo:0, fo2:21, ppo2:140, deep:false, dirty:false,
    zero:250, zeroValid:false, gauge:0, depth:0, maxD:0, rate:0, pabs:P_SURF,
    prev:P_SURF, ppo2n:.21, mod:0, fault:false, bad:0, dstart:0, dur:0,
    dcount:0, viol:0, nofly:0, lock:0, ndl:99, deco:false, ceil:0, tts:0,
    nstop:0, nmin:0, texc:false, dDeco:false, dViol:false, breach:0,
    gf0:.3, gf1:.7, ss:false, ssDone:false, ssEl:0, ds:false, dsDone:false, dsEl:0, dsD:0,
    cur:0, desc:0, surf:0, sync:0, wc:false, wa:false, wp:false, hdg:0, card:'N',
    ratio:0.7, hist:[]
  };

  let lastSim=0, lastS=0, saveAcc=0, lastR=0;
  const pw={};

  const pAt = d => P_SURF + d / MPB;
  const leg = (Q,p0,p1,s,f)=>{if(s<=0)return; const a=(p0-PH2O)*f,b=(p1-PH2O)*f,R=(b-a)/s;
    for(let i=0;i<16;i++){const k=K[i],e=Math.exp(-k*s); Q[i]=a+R*(s-1/k)-(a-Q[i]-R/k)*e}}
  const maxTol = (Q,g)=>Math.max(0,...Q.map((p,i)=>(p-A[i]*g)/(g/B[i]-g+1)));
  const gfNow = ()=>S.algo===1?[0.3,Math.max(0.45,0.75-0.05*Math.min(S.dcount,3)-0.05*Math.min(S.viol,3))]:[0.3,0.7];
  const ndlCalc = (Q,pa,f,gh)=>{const pi=(pa-PH2O)*f; let best=9999*60;
    for(let i=0;i<16;i++){const M=P_SURF+gh*(A[i]+P_SURF/B[i]-P_SURF); if(Q[i]>=M)return 0; if(pi<=M)continue; best=Math.min(best,Math.log((pi-Q[i])/(pi-M))/K[i])} return best/60}
  const ceilCalc = ()=> {if(maxTol(P,S.gf1)<=P_SURF+1e-4)return 0; const pl=maxTol(P,S.gf0); let fm=Math.max(3,Math.ceil((pl-P_SURF)*MPB/3-1e-9)*3); const pf=pAt(fm); for(let d=3;d<=fm+.01;d+=3){const pd=pAt(d); const gg=S.gf1+(S.gf0-S.gf1)*(pd-P_SURF)/(pf-P_SURF); if(maxTol(P,gg)<=pd+1e-4)return d} return fm}
  const simDeco = (g,f)=>{const Q=P.slice(); let d=S.depth,t=0,fc=-1,fmn=0,fin=false;
    for(let it=0;it<600;it++){const c=ceilCalc(); if(fc<0)fc=c; if(c<=0){t+=d/ASC*60;fin=true;break} if(d>c+.01){const s=(d-c)/ASC*60;leg(Q,pAt(d),pAt(c),s,f);t+=s;d=c}else{d=c;leg(Q,pAt(c),pAt(c),60,f);t+=60; if(Math.abs(c-fc)<.01)fmn++}}
    S.tts=fin?Math.ceil(t/60):999; S.nstop=Math.max(fc,0); S.nmin=Math.max(1,fmn)}
  const tIdx = d=>TD.findIndex(x=>d<=x);
  const buildTbl = ()=>{const f=1-S.fo2/100; for(let j=0;j<12;j++){const pa=pAt(TD[j]); const fr=ndlCalc([PAIR].map(()=>PAIR),pa,f,0.7); const nw=ndlCalc(P,pa,f,0.7); const r=fr<.5?1:Math.min(1,nw/fr); tadj[j]=Math.floor(TN[j]*Math.max(0,r))}}

  const saveState = ()=>HAL.prefs.put('st',{P:P.slice(),nofly:S.nofly,lock:S.lock,dc:S.dcount,viol:S.viol,zero:S.zero});
  const loadState = ()=>{const s=HAL.prefs.get('st',null); if(!s||!s.P||s.P.length!=16||s.P.some(x=>(!(x>0.3&&x<8))))return false; P=s.P.slice(); S.nofly=s.nofly; S.lock=s.lock; S.dcount=s.dc; S.viol=s.viol; if(s.zero>50&&s.zero<2450){S.zero=s.zero;S.zeroValid=true} return true}
  const saveSet = ()=> {HAL.prefs.put('set',{fo2:S.fo2,ppo2:S.ppo2,deep:S.deep,algo:S.algo}); S.dirty=false}

  const sensors = ()=>{let mv=0; for(let i=0;i<10;i++)mv+=HAL.analogMv(0); mv/=10;
    if(mv<50||mv>2450){if(S.bad<255)S.bad++;if(S.bad>=10)S.fault=true}else{S.bad=0;S.fault=false; let g=(mv-S.zero)/1000/2*12; if(S.mode!='DIVE'&&Math.abs(g)<.03)S.zero+=.002*(mv-S.zero); if(g<0)g=0; S.gauge+=.3*(g-S.gauge); S.depth=S.gauge*MPB; S.pabs=P_SURF+S.gauge}
    const h=HAL.heading(); S.hdg=Math.round(h)%360; S.card=["N","NE","E","SE","S","SW","W","NW"][Math.floor((h+22.5)/45)%8]}

  const rate = now=>{if(now-lastR<1000)return; lastR=now; S.hist.push(S.depth); if(S.hist.length>6)S.hist.shift(); S.rate=S.hist.length===6?(S.hist[0]-S.depth)*12:0}

  const startDive = t=>{if(S.dirty)saveSet(); S.mode='DIVE'; S.view=0; S.dstart=t; S.dur=0; S.maxD=S.depth; S.surf=0; S.ss=S.ssDone=S.ds=S.dsDone=false; S.ssEl=S.dsEl=0; S.dDeco=S.dViol=false; S.breach=0; S.deco=S.ceil=S.texc=false; S.lock=86400; if(S.algo==2)buildTbl(); saveState(); HAL.beep('DIVE START',880,150)}
  const endDive = ()=>{S.dur=Math.floor((S.surf-S.dstart)/1000); S.dcount++; if(S.dViol)S.viol=Math.min(255,S.viol+1); const tg=(S.dDeco||S.dViol)?86400:(S.dcount>1?64800:43200); S.nofly=Math.max(S.nofly,tg); S.lock=86400; const l=HAL.prefs.get('logs',[]); l.push([S.dur,+S.maxD.toFixed(1),S.fo2,S.algo,S.dDeco?1:0]); HAL.prefs.put('logs',l.slice(-100)); saveState(); S.mode='SURFACE'; S.surf=0; HAL.beep('DIVE END',660,300)}

  const upd = (dt,now)=>{if(S.mode!='DIVE'){S.ndl=99;S.deco=false;S.ceil=0;S.tts=0;S.texc=false;S.wc=S.wa=S.wp=false;return}
    const g=gfNow(); S.gf0=g[0]; S.gf1=g[1]; const f=1-S.fo2/100; S.texc=false;
    if(S.algo==2){const ix=tIdx(S.maxD),el=Math.ceil(S.dur/60); if(ix<0||el>=tadj[ix]){S.texc=true;S.dViol=true}else{S.ndl=Math.min(99,tadj[ix]-el);S.deco=false;S.ceil=0;S.tts=Math.ceil(S.depth/ASC)}}
    if(S.algo!=2||S.texc){S.ceil=ceilCalc(); S.deco=S.ceil>0; if(S.deco){S.ndl=0;S.dDeco=true; if(!lastSim||now-lastSim>=2000){simDeco(g,f); lastSim=now||1}}else{S.ndl=Math.min(99,Math.floor(ndlCalc(P,S.pabs,f,S.gf1))); S.tts=Math.ceil(S.depth/ASC)}}
    S.wc=S.deco&&S.depth<S.ceil-.5; if(S.wc){S.breach+=dt; if(S.breach>10)S.dViol=true}else S.breach=0;
    S.wa=S.rate>ASC_WARN; if(S.rate>ASC_VIOL)S.dViol=true; S.wp=S.ppo2n>=S.ppo2/100}

  const buttons = ()=>{if(HAL.edge(2)){if(S.mode=='SURFACE')S.mode='MENU';else if(S.mode=='MENU'){if(S.dirty)saveSet();S.mode='SURFACE'}else if(S.mode=='DIVE')S.view=(S.view+1)%4}
    if(HAL.edge(0)&&S.mode=='MENU')S.cur=(S.cur+1)%5;
    if(HAL.edge(1)&&S.mode=='MENU'){switch(S.cur){case 0:S.fo2=S.fo2>=40?21:S.fo2+1;S.dirty=true;break; case 1:S.ppo2=S.ppo2>=160?100:S.ppo2+5;S.dirty=true;break; case 2:S.deep=!S.deep;S.dirty=true;break; case 3:if(!S.lock){S.algo=(S.algo+1)%3;S.dirty=true}break; case 4:S.mode='SYNC';S.sync=HAL.millis()+3000;HAL.Serial.println('LOGS '+JSON.stringify(HAL.prefs.get('logs',[])))}}}

  const render = now=>{u8g2.clearBuffer(); const F='6x10',fo=S.fo2,pp=(S.ppo2/100).toFixed(2),lk=Math.ceil(S.lock/3600);

    if(S.mode==='SURFACE'){u8g2.setFont(F); u8g2.drawStr(0,10,'SURFACE / PLANNER'); u8g2.drawHLine(0,12,128);
      u8g2.drawStr(0,24,`NO FLY: ${Math.floor(S.nofly/3600)}h${String(Math.floor(S.nofly%3600/60)).padStart(2,'0')}m`);
      u8g2.drawStr(0,34,`O2:${fo}% PPO2:${pp}`); u8g2.drawStr(0,44,`MOD:${S.mod.toFixed(1)}m Dives:${S.dcount}`);
      u8g2.drawStr(0,54,'ALG:'+ALG[S.algo]+(S.lock?` LK${lk}h`:''));
      if(S.fault)u8g2.drawStr(0,63,'SENSOR FAULT')}
    else if(S.mode==='MENU'){const c=i=>S.cur===i?'>':' '; u8g2.setFont(F); u8g2.drawStr(0,10,'SETTINGS MENU'); u8g2.drawHLine(0,12,128);
      u8g2.drawStr(0,22,`${c(0)}1 FO2 Nitrox: ${fo}%`); u8g2.drawStr(0,32,`${c(1)}2 PPO2 Lim: ${pp}`);
      u8g2.drawStr(0,42,`${c(2)}3 Deep Stop: ${S.deep?'ON':'OFF'}`);
      u8g2.drawStr(0,52,S.lock?`${c(3)}4 ${ALG[S.algo]} LK ${lk}h`:`${c(3)}4 Algo: ${ALG[S.algo]}`);
      u8g2.drawStr(0,62,`${c(4)}5 PC Debugger Sync`)}
    else if(S.mode==='SYNC'){u8g2.setFont('6x12'); u8g2.drawStr(10,25,'PC DEBUGGER'); u8g2.drawStr(10,45,'SYNCING LOGS...')}
    else { // DIVE
      const W=[]; if(S.fault)W.push('SENSOR FAULT'); if(S.wc)W.push('CEILING BREACH!'); if(S.wp)W.push('HIGH PPO2!'); if(S.wa)W.push('ASCENT TOO FAST!'); if(S.texc)W.push('TABLE LIMIT EXCEEDED');

      u8g2.setFont(F); u8g2.drawStr(0,9,`${String(S.hdg).padStart(3,'0')}* ${S.card}`); u8g2.drawStr(52,9,`O2:${fo}%`);
      const tm=String(Math.floor(S.dur/60)).padStart(2,'0')+':'+String(Math.floor(S.dur%60)).padStart(2,'0');
      u8g2.drawStr(128-6*tm.length,9,tm); u8g2.drawHLine(0,11,128);

      // === ASCENT / DESCENT SYMBOLS ===
      const rateStr = S.rate>0.5 ? `ASCN:${S.rate.toFixed(1)} m/min` : (S.rate<-0.5 ? `DESC:${Math.abs(S.rate).toFixed(1)} m/min` : 'STAT: --.- m/min');
      u8g2.drawStr(0,20,rateStr);

      u8g2.setFont('5x7'); u8g2.drawStr(0,33,'DEPTH m'); u8g2.drawStr(80,33,S.deco?'CEIL m':'NDL min');

      u8g2.setFont('L22'); u8g2.drawStr(0,44,S.depth.toFixed(1));
      if(S.deco)u8g2.drawStr(80,44,S.ceil.toFixed(0)); else if(S.ndl>=99){u8g2.drawStr(80,44,'99'); u8g2.drawStr(112,44,'+')} else u8g2.drawStr(80,44,''+S.ndl);

      u8g2.drawHLine(0,47,128);
      if(W.length){u8g2.setFont('6x12'); u8g2.drawStr(0,58,W[Math.floor(now/1500)%W.length]);}
      else if(S.deco)u8g2.drawStr(0,60,`STOP ${S.nstop.toFixed(0)}m ${S.nmin}min TTS${S.tts}`);
      else if(S.ss)u8g2.drawStr(0,60,`STOP 5m | ${String(Math.floor((180-S.ssEl)/60)).padStart(2,'0')}:${String(Math.floor((180-S.ssEl)%60)).padStart(2,'0')}`);
      else if(S.ds)u8g2.drawStr(0,60,`DEEP ${S.dsD.toFixed(0)}m | ${String(Math.floor((120-S.dsEl)/60)).padStart(2,'0')}:${String(Math.floor((120-S.dsEl)%60)).padStart(2,'0')}`);
      else {
        if(S.view===0){ // Tissue chart
          const y0=57,H=11; for(let x=0;x<120;x+=3){u8g2.drawBox(x,y0,1,1); u8g2.drawBox(x,y0-Math.round(S.gf1*H),1,1)}
          for(let i=0;i<16;i++){const m=A[i]+P_SURF/B[i]-P_SURF; let g=Math.max(-0.4,Math.min(1.1,(P[i]-P_SURF)/m)); const x=i*7+4;
            if(g>=0){const h=Math.max(1,Math.round(g*H)); u8g2.drawBox(x,y0-h,5,h)} else {const h=Math.max(1,Math.round(-g*H)); u8g2.drawBox(x,y0+1,5,h)}}
        } else if(S.view===1){
          u8g2.drawStr(0,53,`O2:${fo}% PPO2:${S.ppo2n.toFixed(2)}`); u8g2.drawStr(0,63,`MOD:${S.mod.toFixed(1)}m LIM:${pp}`);
        } else if(S.view===2){
          u8g2.drawStr(0,53,S.deco?`CEIL:${S.ceil}m TTS:${S.tts}m`:`NO DECO  NDL:${S.ndl}`);
          u8g2.drawStr(0,63,S.algo==2&&!S.texc?ALG[2]:`ALG:${ALG[S.algo]} GF${Math.round(S.gf0*100)}/${Math.round(S.gf1*100)}`);
        } else {
          u8g2.drawStr(0,53,`MAX:${S.maxD.toFixed(1)}m ASC:${S.rate.toFixed(1)}`);
          u8g2.drawStr(0,63,`TEMP:${HAL.temp().toFixed(1)} C`);
        }
      }
    }
    u8g2.sendBuffer()
  }

  const tick = (now,dt)=>{sensors(); const fo=S.fo2/100; S.ppo2n=S.pabs*fo; S.mod=Math.max(0,Math.floor((S.ppo2/100/fo-P_SURF)*MPB*10)/10);
    if(!S.fault){leg(P,S.prev,S.pabs,dt,S.mode==='DIVE'?1-fo:FN2_AIR); S.prev=S.pabs; rate(now); if(S.mode==='DIVE'){S.maxD=Math.max(S.maxD,S.depth); S.dur=Math.floor((now-S.dstart)/1000)} upd(dt,now); if(S.mode==='DIVE'){
      // advisory stops
      if(!S.deco && S.deep && S.maxD>=18 && !S.dsDone){ S.dsD=S.maxD/2; if(S.depth<S.maxD-3 && Math.abs(S.depth-S.dsD)<=1.5){ S.ds=true; if(S.dsEl>=120){S.dsDone=true; S.ds=false; HAL.beep('DEEP STOP DONE',1000,200)}}}
      if(S.maxD>=10 && !S.ssDone){ if(S.depth>=3&&S.depth<=6)S.ss=true; else if(S.depth<2.5||S.depth>7){S.ss=false;S.ssEl=0} if(S.ss){S.ssEl+=dt; if(S.ssEl>=180){S.ssDone=true;S.ss=false; HAL.beep('SAFETY STOP DONE',700,150)}}}
    }}
    if(S.mode!='DIVE'){if(!S.fault&&S.depth>=0.8){if(!S.desc)S.desc=now;if(now-S.desc>=3000){startDive(S.desc);S.desc=0}}else S.desc=0}else{if(S.depth<.3){if(!S.surf)S.surf=now;if(now-S.surf>=60000)endDive()}else S.surf=0}
    if(S.mode==='SYNC'&&now>=S.sync)S.mode='SURFACE';
    if(now-lastS>=1000){lastS=now; saveAcc++; if(S.mode!='DIVE'){if(S.nofly>0)S.nofly--; if(S.lock>0)S.lock--; if(!S.nofly&&!S.lock&&(S.dcount||S.viol)){S.dcount=0;S.viol=0; saveState()}} if(saveAcc>=(S.mode==='DIVE'?120:300)){saveAcc=0; if(S.mode==='DIVE'||S.nofly>0)saveState()}}
    render(now)}
  }

  const setup = ()=>{const s=HAL.prefs.get('set',{}); S.fo2=s.fo2||21; S.ppo2=s.ppo2||140; S.deep=!!s.deep; S.algo=s.algo||0; if(!loadState())P=P.map(()=>PAIR); let mv=0; for(let i=0;i<30;i++)mv+=HAL.analogMv(0); mv/=30; if(mv<50||mv>2450)S.fault=true; else if(!(S.zeroValid&&Math.abs(mv-S.zero)>16.7)){S.zero=mv; S.zeroValid=true} S.prev=P_SURF}

  return {setup, loop:tick}
}

/* ================= SIMULATOR / HAL ================= */
const cv=$('oled'),cx=cv.getContext('2d'),tc=$('tis').getContext('2d');
const FS={'5x7':5,'6x10':6,'6x12':6,'7x14':7,'L22':13}; let fw=6,skipR=false,quiet=false;
const u8g2={clearBuffer(){if(skipR)return; cx.setTransform(1,0,0,1,0,0); cx.fillStyle='#000'; cx.fillRect(0,0,512,256); cx.scale(4,4); cx.fillStyle='#7df9ff'}, setFont(f){fw=FS[f]||6}, drawStr(x,y,s){if(skipR)return; cx.font=(fw/.6).toFixed(1)+'px monospace'; cx.fillText(s,x,y)}, drawHLine(x,y,w){if(!skipR)cx.fillRect(x,y,w,1)}, sendBuffer(){}};

let FWi,simMs=0,depth=0,tgt=0,speed=1,flash=new Map(),pend=[0,0,0],ac,buzz=0,lastMv=0,ticks=0,alog=[],ser=[],ptr=0;
const st={temp:26,hdg:0,noise:1,fault:false,rate:15,surf:false,sound:true};
const fmt=ms=>{const s=Math.floor(ms/1000),h=Math.floor(s/3600); return (h?h+':':'')+String(Math.floor(s%3600/60)).padStart(2,'0')+':'+String(s%60).padStart(2,'0')};
function tone(f,ms){try{ac=ac||new AudioContext(); const o=ac.createOscillator(),g=ac.createGain(); o.type='square'; o.frequency.value=f; g.gain.value=.04; o.connect(g); g.connect(ac.destination); o.start(); o.stop(ac.currentTime+ms/1000)}catch(e){}}
const HAL={millis:()=>simMs, analogMv:()=>{lastMv=st.fault?0:250+depth/10/12*2000+(Math.random()-.5)*st.noise; return lastMv}, edge:i=>{const v=pend[i]; pend[i]=0; return v}, heading:()=>st.hdg, temp:()=>st.temp, u8g2, Serial:{println:s=>{if(!quiet){ser.push(fmt(simMs)+' '+s); ser=ser.slice(-40)}}}, prefs:{get:(k,d)=>flash.has(k)?JSON.parse(flash.get(k)):d, put:(k,v)=>flash.set(k,JSON.stringify(v))}, beep:(n,f,ms)=>{if(quiet)return; alog.unshift(fmt(simMs)+'  '+n); alog=alog.slice(0,30); buzz=performance.now()+ms; if(st.sound)tone(f,ms)}};

const DEF=FW.toString();
function boot(src){try{FWi=(src?new Function('return ('+src+')')():FW)(HAL); FWi.setup(); $('err').textContent=''}catch(e){$('err').textContent='Load failed: '+e; FWi=FW(HAL); FWi.setup()}}
function step(){const m=st.rate/600, t=st.surf?0:tgt; depth+=Math.max(-m,Math.min(m,t-depth)); simMs+=100; ticks++; FWi.loop(simMs,.1)}
function power(clear){if(clear){flash.clear(); depth=0; tgt=0; $('dep').value=0; simMs=0; alog=[]; ser=[]} boot($('src').value===DEF?null:$('src').value); ui()}
$('src').value=DEF;

const bind=(id,k,fn)=>{const e=$(id),o=()=>{const v=e.type=='checkbox'?e.checked:+e.value; fn(v); ui()}; e.oninput=o; o()};
bind('dep','',v=>{tgt=v; $('depv').textContent=v.toFixed(1)+' m'});
bind('rate','',v=>{st.rate=v; $('ratev').textContent=v+' m/min'});
bind('tmp','',v=>{st.temp=v; $('tmpv').textContent=v+' C'});
bind('hdg','',v=>{st.hdg=v; $('hdgv').textContent=v+'\u00b0'});
bind('surf','',v=>st.surf=v); bind('noi','',v=>{st.noise=v; $('noiv').textContent=v}); bind('flt','',v=>st.fault=v); bind('snd','',v=>st.sound=v);
bind('skm','',v=>$('skv').textContent=v+' min');
$('dbg').onchange=()=>{$('dbgp').hidden=!$('dbg').checked};

[0,1,2].forEach(i=>{$('b'+i).onclick=()=>{pend[i]=1}});
addEventListener('keydown',e=>{const i='123'.indexOf(e.key); if(i>=0&&!/INPUT|TEXTAREA/.test(e.target.tagName))pend[i]=1});

[0,1,2,3].forEach(s=>$('s'+s).onclick=()=>{speed=s; [0,1,2,3].forEach(k=>$('s'+k).className=k==s?'on':'')});
$('skip').onclick=()=>{const n=+$('skm').value*600; quiet=skipR=true; for(let i=0;i<n;i++)step(); quiet=skipR=false; step(); ui()};
$('pc').onclick=()=>power(false); $('fr').onclick=()=>power(true);
$('ld').onclick=()=>{boot($('src').value); ui()}; $('rs').onclick=()=>{$('src').value=DEF; boot(null); ui()};

function badge(t,c){return `<span class="bd ${c||''}">${t}</span>`}
function ui(){if(!FWi)return; const v=FWi.vars(); $('clk').textContent=`Sim time ${fmt(simMs)} | depth ${depth.toFixed(1)} m | dive ${fmt(v.dur*1000)}`; $('buz').className=performance.now()<buzz?'on':'';
  $('badges').innerHTML=[v.fault&&badge('SENSOR FAULT','w'),v.wc&&badge('CEILING BREACH','w'),v.wp&&badge('HIGH PPO2','w'),v.wa&&badge('ASCENT FAST','w'),v.texc&&badge('TABLE EXCEEDED','w'),v.deco&&badge('DECO','i'),v.ss&&badge('SAFETY STOP','i'),v.ds&&badge('DEEP STOP','i'),v.lock>0&&badge('ALGO LOCKED'),v.nofly>0&&badge('NO FLY')].filter(Boolean).join('')||badge('no active alerts');
  $('alog').textContent=alog.join('\n')||'-'; $('live').innerHTML=badge('mode '+v.mode)+badge('NDL '+v.ndl)+badge('ceil '+v.ceil)+badge('TTS '+v.tts)}
function dbgDraw(){if($('dbgp').hidden)return; const v=FWi.vars();
  $('dv').innerHTML=Object.entries(v).filter(([k,x])=>typeof x!='object').map(([k,x])=>`<div><b>${k}</b> ${typeof x=='number'?(+x.toFixed(4)):x}</div>`).join('');
  tc.clearRect(0,0,620,200); tc.font='10px monospace'; const gh=v.gf1||.7;
  v.P.forEach((p,i)=>{const y=i*12+2; tc.fillStyle='#8884'; tc.fillRect(30,y,540,9); tc.fillStyle=p>=1.013+gh*((FWi.A?FWi.A[i]+1.013/FWi.B[i]:2)-1.013)?'#d92d20':'#3b82f6'; tc.fillRect(30,y,p/4*540,9);
    tc.fillStyle='#fff'; tc.fillRect(30+v.pabs/4*540,y-1,2,11); if(FWi.A){tc.fillStyle='#f79009'; tc.fillRect(30+(1.013+gh*(FWi.A[i]+1.013/FWi.B[i]-1.013))/4*540,y-1,2,11)}
    tc.fillStyle='#888'; tc.fillText('C'+(i+1),0,y+8); tc.fillText(p.toFixed(3),574,y+8)});
  $('hal').textContent=`analogMv=${lastMv.toFixed(1)}  ticks=${ticks}  buttons pending=${pend}  flash keys=${[...flash.keys()]}\n`+ser.join('\n')}

boot(null); ui();
let dc=0; setInterval(()=>{if(!speed)return; try{for(let i=0;i<speed;i++){skipR=i<speed-1; step()} skipR=false}catch(e){$('err').textContent='Runtime error: '+e; speed=0} ui(); if(++dc%3==0)dbgDraw()},100);
</script></body></html>