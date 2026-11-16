'use strict';
(() => {
  const $ = id => document.getElementById(id);
  try {
    const DATA = JSON.parse($('corridor-data').textContent);
    if (DATA.schema !== 1 || DATA.datasets.length !== 4) throw Error('Unsupported replay data');
    const models = ['disabled', 'independent', 'correlated'];
    const names = {disabled:'Raw IMU/GNSS EKF', independent:'Independent map noise', correlated:'Correlated map error'};
    const colors = {disabled:'#6ba9ff', independent:'#70d5ed', correlated:'#61e5b3', gnss:'#edb465'};
    const roads = DATA.roads.map(r => ({id:r[0], group:r[1], width:r[2], points:r[3], next:r[4], layer:r[5],
      box:[Math.min(...r[3].map(p=>p[0])),Math.min(...r[3].map(p=>p[1])),Math.max(...r[3].map(p=>p[0])),Math.max(...r[3].map(p=>p[1]))]}));
    const roadById = new Map(roads.map(r => [r.id,r]));
    const canvases = [0,1,2].map(i=>$('map'+i));
    const backgrounds = canvases.map(()=>document.createElement('canvas'));
    const lanes = [
      ['Road selected', '#bd99ef', (r,n)=>r.original_edge!==null, false],
      ['Geometric support', '#d0bb8b', (r,n)=>r.proposed_cell!==null, false],
      ['Independent update', colors.independent, (r,n)=>n.independent.applied, true],
      ['Correlated update', colors.correlated, (r,n)=>n.correlated.applied, true],
      ['Continuous output', colors.correlated, (r,n)=>r.outputs.correlated.ready, false]
    ];
    lanes.forEach(([label],i)=>{
      const l=document.createElement('label');l.textContent=label;
      const c=document.createElement('canvas');c.id='lane'+i;c.setAttribute('aria-label',label+' timeline');
      $('timeline').append(l,c);
      c.addEventListener('click',e=>{const rect=c.getBoundingClientRect();seek((e.clientX-rect.left)/rect.width*duration());});
    });
    let di=0, d=DATA.datasets[0], time=0, index=0, mode='overlay', playing=false, lastFrame=null;
    let dirty=true, bgDirty=true, inspected=null, gapCursor=-1;
    let view={x:0,y:0,s:1}, drag=null;
    const pointerPositions=new Map();
    let pinch=null;
    const fmt=(x,n=3)=>x===null||x===undefined?'—':Number(x).toFixed(n);
    function stamp(t){const m=Math.floor(t/60), s=t-m*60;return String(m).padStart(2,'0')+':'+s.toFixed(1).padStart(4,'0');}
    function duration(){return d.rows[d.rows.length-1].elapsed_s;}
    function floorIndex(array,t,get=r=>r.elapsed_s){let a=0,b=array.length;while(a<b){const m=(a+b)>>1;if(get(array[m])<=t)a=m+1;else b=m;}return a-1;}
    function current(){return d.rows[index];}
    function seek(t){time=Math.max(0,Math.min(duration(),Number(t)));index=Math.max(0,floorIndex(d.rows,time));$('seek').value=time;dirty=true;}
    function mark(){dirty=true;}
    function canvasSize(c,height=null){const r=c.getBoundingClientRect(),w=r.width,h=height===null?r.height:height;
      const dpi=Math.min(2,window.devicePixelRatio||1);if(c.width!==Math.round(w*dpi)||c.height!==Math.round(h*dpi)){c.width=Math.round(w*dpi);c.height=Math.round(h*dpi);}
      const ctx=c.getContext('2d');ctx.setTransform(dpi,0,0,dpi,0,0);return [ctx,w,h];}
    function screen(p,w,h){return [w/2+(p[0]-view.x)*view.s,h/2-(p[1]-view.y)*view.s];}
    function world(x,y,w,h){return [view.x+(x-w/2)/view.s,view.y-(y-h/2)/view.s];}
    function resize(){bgDirty=true;dirty=true;}
    function fit(){const ps=d.rows.flatMap(r=>models.map(m=>r.outputs[m].position).filter(Boolean));
      const xs=ps.map(p=>p[0]),ys=ps.map(p=>p[1]);const lo=[Math.min(...xs),Math.min(...ys)],hi=[Math.max(...xs),Math.max(...ys)];
      const r=canvases[0].getBoundingClientRect();view={x:(lo[0]+hi[0])/2,y:(lo[1]+hi[1])/2,s:Math.min((r.width-70)/Math.max(20,hi[0]-lo[0]),(r.height-90)/Math.max(20,hi[1]-lo[1]))};
      $('follow').checked=false;resize();}
    function focus(){const r=current(),p=r.outputs.correlated.position||r.raw_position;if(!p)return;view={x:p[0],y:p[1],s:Math.min(canvases[0].clientWidth,canvases[0].clientHeight)/65};resize();}
    function jumpMax(){let best=0;for(let i=1;i<d.rows.length;i++)if(d.diagnostics[i].correlated.native_position_step_m>d.diagnostics[best].correlated.native_position_step_m)best=i;seek(d.rows[best].elapsed_s);focus();}
    function modelDifference(r){const a=r.outputs.correlated.position,b=r.outputs.independent.position;return a&&b?Math.hypot(a[0]-b[0],a[1]-b[1]):0;}
    function jumpDifference(){let best=0;for(let i=1;i<d.rows.length;i++)if(modelDifference(d.rows[i])>modelDifference(d.rows[best]))best=i;seek(d.rows[best].elapsed_s);focus();}
    function gapIndices(){return d.rows.map((r,i)=>r.original_edge===null&&r.outputs.correlated.ready&&r.outputs.correlated.map_updates>0?i:-1).filter(i=>i>=0);}
    function jumpGap(){const gaps=gapIndices();if(!gaps.length)return;let i=gaps.find(i=>i>Math.max(index,gapCursor));if(i===undefined)i=gaps[0];gapCursor=i;seek(d.rows[i].elapsed_s);focus();}
    function setDataset(i){if(!Number.isInteger(i)||i<0||i>=DATA.datasets.length)throw Error('Invalid recording');di=i;d=DATA.datasets[i];time=0;index=0;playing=false;lastFrame=null;inspected=null;gapCursor=-1;
      $('dataset').value=i;$('seek').max=duration();$('play').textContent='▶ Play';resize();}
    function setMode(m){mode=m;const split=m==='split';$('maps').className='maps'+(split?' split':'');for(let i=1;i<3;i++)$('mapbox'+i).hidden=!split;
      $('split').setAttribute('aria-pressed',String(split));$('overlay').setAttribute('aria-pressed',String(!split));resize();}
    function background(k,w,h){const c=backgrounds[k];c.width=Math.round(w);c.height=Math.round(h);const x=c.getContext('2d');x.fillStyle='#0b1621';x.fillRect(0,0,w,h);
      const box=[view.x-w/2/view.s,view.y-h/2/view.s,view.x+w/2/view.s,view.y+h/2/view.s];
      const gridStep=10**Math.floor(Math.log10(90/view.s));x.strokeStyle='#142636';x.lineWidth=1;x.beginPath();
      for(let v=Math.ceil(box[0]/gridStep)*gridStep;v<box[2];v+=gridStep){const p=screen([v,0],w,h);x.moveTo(p[0],0);x.lineTo(p[0],h);}
      for(let v=Math.ceil(box[1]/gridStep)*gridStep;v<box[3];v+=gridStep){const p=screen([0,v],w,h);x.moveTo(0,p[1]);x.lineTo(w,p[1]);}x.stroke();
      if($('roads').checked){x.strokeStyle='#2c4356';x.lineWidth=1.2;x.beginPath();for(const road of roads){const b=road.box;if(b[2]<box[0]||b[0]>box[2]||b[3]<box[1]||b[1]>box[3])continue;road.points.forEach((p,i)=>{const q=screen(p,w,h);i?x.lineTo(...q):x.moveTo(...q);});}x.stroke();}
    }
    function trace(ctx,positions,w,h,color,width,dash=[]){ctx.strokeStyle=color;ctx.lineWidth=width;ctx.setLineDash(dash);ctx.beginPath();let active=false;
      for(const p of positions){if(!p){active=false;continue;}const q=screen(p,w,h);if(!active){ctx.moveTo(...q);active=true;}else ctx.lineTo(...q);}ctx.stroke();ctx.setLineDash([]);}
    function marker(ctx,p,model,radius=6){ctx.save();ctx.translate(...p);ctx.fillStyle=colors[model];ctx.strokeStyle='#08111b';ctx.lineWidth=1.5;ctx.beginPath();
      if(model==='correlated'){ctx.moveTo(0,-radius-2);ctx.lineTo(radius+1,radius);ctx.lineTo(-radius-1,radius);ctx.closePath();}
      else if(model==='independent'){ctx.rect(-radius,-radius,2*radius,2*radius);}else ctx.arc(0,0,radius,0,Math.PI*2);ctx.fill();ctx.stroke();ctx.restore();}
    function ellipse(ctx,p,cov,w,h,color){if(!cov)return;const a=cov[0][0],b=cov[0][1],c=cov[1][1],v=Math.hypot(a-c,2*b),l1=(a+c+v)/2,l2=(a+c-v)/2;
      if(l1<0||l2<0)return;ctx.save();ctx.translate(...screen(p,w,h));ctx.rotate(-.5*Math.atan2(2*b,a-c));ctx.strokeStyle=color;ctx.globalAlpha=.7;ctx.lineWidth=1;ctx.setLineDash([3,3]);ctx.beginPath();ctx.ellipse(0,0,Math.sqrt(l1)*view.s,Math.sqrt(l2)*view.s,0,0,Math.PI*2);ctx.stroke();ctx.restore();}
    function modelVisible(m){return $(m==='disabled'?'raw':m).checked;}
    function drawMap(k){const [x,w,h]=canvasSize(canvases[k]);if(!w||!h)return;
      if(bgDirty)background(k,w,h);x.drawImage(backgrounds[k],0,0,w,h);
      const r=current(),last=$('full').checked?d.rows.length:index+1;
      if($('selected').checked&&r.original_edge){const road=roadById.get(r.original_edge);if(road)trace(x,road.points,w,h,'#bd99ef',3,[6,5]);}
      if(inspected&&inspected!==r.original_edge){const road=roadById.get(inspected);if(road)trace(x,road.points,w,h,'#f2dca8',2,[4,4]);}
      if($('gnss').checked){const gs=d.gnss.filter(g=>$('full').checked||g[0]<=r.elapsed_s);trace(x,gs.map(g=>g.slice(1,3)),w,h,'#edb46580',1,[2,4]);
        x.fillStyle='#edb465';for(const g of gs){const q=screen(g.slice(1,3),w,h);x.fillRect(q[0]-1,q[1]-1,2,2);}const gi=floorIndex(d.gnss,r.elapsed_s,g=>g[0]);if(gi>=0)marker(x,screen(d.gnss[gi].slice(1,3),w,h),'gnss',3);}
      const shown=mode==='split'?[models[k]]:models;
      for(const model of shown){if(!modelVisible(model))continue;trace(x,d.rows.slice(0,last).map(r=>r.outputs[model].position),w,h,colors[model],model==='correlated'?2.1:1.5,model==='independent'?[6,3]:[]);
        // Draw every exported position, including single-sample intervals.
        x.fillStyle=colors[model];for(let i=0;i<last;i++){const o=d.rows[i].outputs[model];if(!o.position)continue;const p=screen(o.position,w,h);x.globalAlpha=o.usable?.7:.3;x.fillRect(p[0]-1.2,p[1]-1.2,2.4,2.4);}x.globalAlpha=1;
        const out=r.outputs[model];if(out.ready){if($('covariance').checked)ellipse(x,out.position,out.covariance,w,h,colors[model]);marker(x,screen(out.position,w,h),model);}}
      const metres=10**Math.floor(Math.log10(100/view.s)),pixels=metres*view.s;x.strokeStyle='#a6bece';x.lineWidth=2;x.beginPath();x.moveTo(18,h-22);x.lineTo(18+pixels,h-22);x.stroke();x.fillStyle='#bfd0df';x.font='10px system-ui';x.fillText(metres>=1000?(metres/1000)+' km':metres+' m',18,h-31);x.fillText('N ↑',w-40,23);
      $('maptitle'+k).textContent=mode==='split'?names[models[k]]:'Synchronized position overlay';
      const sub=document.createElement('small');sub.textContent=r.outputs.correlated.ready?'Exact exported positions · t = '+fmt(r.elapsed_s,3)+' s':'EKF not initialized at this sample';$('maptitle'+k).append(sub);
    }
    function drawTimeline(){lanes.forEach(([label,color,fn,ticks],j)=>{const [x,w,h]=canvasSize($('lane'+j));x.fillStyle='#07101b';x.fillRect(0,0,w,h);x.fillStyle=color;
      d.rows.forEach((r,i)=>{if(!fn(r,d.diagnostics[i]))return;const a=r.elapsed_s/duration()*w,b=(d.rows[i+1]?.elapsed_s??duration())/duration()*w;
        x.fillRect(a,0,ticks?Math.max(1,window.devicePixelRatio?1:1):Math.max(1,b-a),h);});x.fillStyle='#f1f4f7';x.fillRect(time/duration()*w-1,0,2,h);});}
    function kv(parent,label,value,color=null){const e=document.createElement('div');e.className='kv';const a=document.createElement('span'),b=document.createElement('b');a.textContent=label;b.textContent=value;if(color)b.style.color=color;e.append(a,b);parent.append(e);}
    function drawOffset(){const [x,w,h]=canvasSize($('offset')),r=current(),p=r.raw_position;x.fillStyle='#0a1520';x.fillRect(0,0,w,h);
      if(!p){x.fillStyle='#a8bdcd';x.font='11px system-ui';x.fillText('Waiting for native initialization',16,h/2);$('offsetScale').textContent='No position is fabricated before initialization.';return;}
      const delta=models.map(m=>r.outputs[m].position.map((v,i)=>v-p[i]));const extent=Math.max(.05,...delta.flat().map(v=>Math.abs(v)*1.4));const scale=Math.min(w,h)*.38/extent;
      x.strokeStyle='#31485b';x.lineWidth=1;x.beginPath();x.moveTo(w/2,10);x.lineTo(w/2,h-10);x.moveTo(10,h/2);x.lineTo(w-10,h/2);x.stroke();
      models.forEach((m,i)=>{if(!modelVisible(m))return;const q=[w/2+delta[i][0]*scale,h/2-delta[i][1]*scale];x.strokeStyle=colors[m];x.beginPath();x.moveTo(w/2,h/2);x.lineTo(...q);x.stroke();marker(x,q,m,5);});
      x.fillStyle='#b9ccdc';x.font='10px system-ui';x.fillText('N',w/2+6,13);x.fillText('E',w-15,h/2-7);$('offsetScale').textContent='Displacement from raw EKF · ±'+fmt(extent,3)+' m reference range. Magnified view; map positions are unaltered.';
    }
    function sidebar(){const r=current(),n=d.diagnostics[index],p=r.outputs.correlated,ready=p.ready;
      $('clock').textContent=stamp(time)+' / '+stamp(duration());$('sampleClock').textContent='Displayed sample '+(index+1)+' / '+d.rows.length+' · '+fmt(r.elapsed_s,6)+' s';
      $('outputState').textContent=ready?'Both corrected outputs continue':'Not initialized yet';
      $('updateState').className='status'+(ready?'':' wait');
      $('updateState').textContent=ready?(n.correlated.applied?'New correlated road measurement applied. ':'No new correlated map measurement. ')+(p.usable?'Persistent native state is usable.':'Native output is marked unusable.'):'No vehicle position is displayed before the native EKF initializes.';
      const values=$('values');values.replaceChildren();
      kv(values,'Independent − raw',fmt(r.outputs.independent.displacement_from_raw_m)+' m',colors.independent);
      kv(values,'Correlated − raw',fmt(p.displacement_from_raw_m)+' m',colors.correlated);
      kv(values,'Difference between map models',ready?fmt(modelDifference(r),4)+' m':'—');
      kv(values,'Independent update step',fmt(n.independent.native_position_step_m,4)+' m');
      kv(values,'Correlated update step',fmt(n.correlated.native_position_step_m,4)+' m');
      kv(values,'Correlated factor',n.correlated.reason);
      $('coordinates').textContent=models.map(m=>{const o=r.outputs[m];return names[m]+'\n'+JSON.stringify({position_EN_m:o.position,covariance_EN_m2:o.covariance,ready:o.ready,usable:o.usable},null,2);}).join('\n\n');
      $('roadStatus').textContent='Support: '+r.support_reason+'. Purple is the entire original selected edge, not a localization trajectory. Input geometry is not a surveyed lane boundary.';
      $('edge').textContent='Selected edge: '+(r.original_edge||'none')+'\nCell: '+(r.proposed_cell||'none');$('edge').style.whiteSpace='pre-line';
      const road=roadById.get(inspected);$('inspected').textContent=road?'Inspected '+road.id+' · half-width '+road.width+' m · layer '+road.layer+' · '+road.next.length+' explicit successors.':'Click an input road to inspect its identity and model width.';
      const summary=$('summary');summary.replaceChildren();const s=d.summary;
      kv(summary,'Sensor events',s.sensor_events.toLocaleString('en-US'));kv(summary,'Initialized outputs',s.models.correlated.counts.initialized_outputs+' / '+d.rows.length);
      kv(summary,'Original selected / supported',s.original_selected_epochs+' / '+s.supported_epochs);
      kv(summary,'Independent / correlated updates',s.models.independent.map_updates+' / '+s.models.correlated.map_updates);
      kv(summary,'Independent p95 map step',fmt(s.models.independent.p95_map_step_m,4)+' m');kv(summary,'Correlated p95 map step',fmt(s.models.correlated.p95_map_step_m,4)+' m');
    }
    function draw(){if($('app').hidden)return;const p=current().outputs.correlated.position;if($('follow').checked&&p){if(view.x!==p[0]||view.y!==p[1]){view.x=p[0];view.y=p[1];bgDirty=true;}}
      for(let i=0;i<(mode==='split'?3:1);i++)drawMap(i);bgDirty=false;drawTimeline();drawOffset();sidebar();dirty=false;}
    function inspectRoad(e,k){const c=canvases[k],rect=c.getBoundingClientRect(),px=e.clientX-rect.left,py=e.clientY-rect.top;let best=8,selected=null;
      for(const r of roads)for(let i=1;i<r.points.length;i++){const a=screen(r.points[i-1],rect.width,rect.height),b=screen(r.points[i],rect.width,rect.height),dx=b[0]-a[0],dy=b[1]-a[1],u=Math.max(0,Math.min(1,((px-a[0])*dx+(py-a[1])*dy)/(dx*dx+dy*dy||1))),dist=Math.hypot(px-a[0]-u*dx,py-a[1]-u*dy);if(dist<best){best=dist;selected=r.id;}}
      inspected=selected;mark();}
    function zoom(factor){view.s=Math.max(.005,Math.min(150,view.s*factor));resize();}
    function csvText(){const header=['elapsed_s','raw_e_m','raw_n_m','independent_e_m','independent_n_m','correlated_e_m','correlated_n_m','raw_ready','raw_usable','independent_ready','independent_usable','correlated_ready','correlated_usable','original_selected_edge','support_reason','proposed_cell','independent_update','correlated_update','independent_step_m','correlated_step_m','correction_sigma_m','shared_map_sigma_m'];
      const rows=d.rows.map((r,i)=>{const o=r.outputs,n=d.diagnostics[i];return [r.elapsed_s,...(r.raw_position||[null,null]),...(o.independent.position||[null,null]),...(o.correlated.position||[null,null]),...models.flatMap(m=>[o[m].ready,o[m].usable]),r.original_edge,r.support_reason,r.proposed_cell,n.independent.applied,n.correlated.applied,n.independent.native_position_step_m,n.correlated.native_position_step_m,DATA.provenance.sigma_m,DATA.provenance.map_sigma_m];});
      const esc=v=>v===null?'':String(v).includes(',')||String(v).includes('"')?'"'+String(v).replaceAll('"','""')+'"':String(v);return [header,...rows].map(row=>row.map(esc).join(',')).join('\n')+'\n';}
    DATA.datasets.forEach((d,i)=>{const o=document.createElement('option');o.value=i;o.textContent=d.label;$('dataset').append(o);});
    $('dataset').onchange=()=>{setDataset(Number($('dataset').value));fit();jumpMax();};
    $('seek').oninput=()=>seek($('seek').value);$('play').onclick=()=>{if(time>=duration())seek(0);playing=!playing;lastFrame=null;$('play').textContent=playing?'Ⅱ Pause':'▶ Play';};
    $('prev').onclick=()=>seek(d.rows[Math.max(0,index-1)].elapsed_s);$('next').onclick=()=>seek(d.rows[Math.min(d.rows.length-1,index+1)].elapsed_s);
    $('max').onclick=jumpMax;$('difference').onclick=jumpDifference;$('gap').onclick=jumpGap;$('fit').onclick=fit;$('focus').onclick=focus;$('overlay').onclick=()=>setMode('overlay');$('split').onclick=()=>setMode('split');
    ['raw','independent','correlated','gnss','roads','selected','covariance','full','follow'].forEach(id=>$(id).onchange=resize);
    canvases.forEach((c,k)=>{
      c.addEventListener('pointerdown',e=>{c.setPointerCapture(e.pointerId);pointerPositions.set(e.pointerId,[e.clientX,e.clientY]);$('follow').checked=false;
        if(pointerPositions.size===2){const ps=[...pointerPositions.values()];pinch={distance:Math.hypot(ps[0][0]-ps[1][0],ps[0][1]-ps[1][1]),scale:view.s};drag=null;}
        else drag={id:e.pointerId,x:e.clientX,y:e.clientY,vx:view.x,vy:view.y,moved:false};});
      c.addEventListener('pointermove',e=>{if(!pointerPositions.has(e.pointerId))return;pointerPositions.set(e.pointerId,[e.clientX,e.clientY]);
        if(pinch&&pointerPositions.size===2){const ps=[...pointerPositions.values()];view.s=Math.max(.005,Math.min(150,pinch.scale*Math.hypot(ps[0][0]-ps[1][0],ps[0][1]-ps[1][1])/Math.max(1,pinch.distance)));resize();}
        else if(drag){drag.moved ||= Math.hypot(e.clientX-drag.x,e.clientY-drag.y)>4;view.x=drag.vx-(e.clientX-drag.x)/view.s;view.y=drag.vy+(e.clientY-drag.y)/view.s;resize();}});
      const end=e=>{if(drag&&!drag.moved&&!pinch)inspectRoad(e,k);pointerPositions.delete(e.pointerId);drag=null;pinch=null;};c.addEventListener('pointerup',end);c.addEventListener('pointercancel',()=>{pointerPositions.clear();drag=null;pinch=null;});
      c.addEventListener('wheel',e=>{e.preventDefault();const r=c.getBoundingClientRect(),x=e.clientX-r.left,y=e.clientY-r.top,before=world(x,y,r.width,r.height);zoom(Math.exp(-Math.sign(e.deltaY)*.18));const after=world(x,y,r.width,r.height);view.x+=before[0]-after[0];view.y+=before[1]-after[1];$('follow').checked=false;resize();},{passive:false});
    });
    $('export').onclick=()=>{const url=URL.createObjectURL(new Blob([csvText()],{type:'text/csv'})),a=document.createElement('a');a.href=url;a.download=d.id+'-qualified-corridor.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
    $('provenance').textContent=JSON.stringify({...DATA.provenance,member_sha256:'27 artifact member hashes verified by builder; available in source artifact'},null,2);
    [['CI run','https://github.com/yongkyuns/navigation_research/actions/runs/'+DATA.provenance.run_id],['Merged estimator','https://github.com/yongkyuns/navigation_research/commit/'+DATA.provenance.merge_commit]].forEach(([label,url])=>{const a=document.createElement('a');a.textContent=label;a.href=url;a.target='_blank';a.rel='noopener noreferrer';$('evidenceLinks').append(a,document.createTextNode(' · '));});
    if(location.protocol==='file:'){$('legacyLink').href='https://yongkyuns.github.io/navigation_research/';$('legacyLink').textContent='Original hosted experiment ↗';}
    window.addEventListener('resize',resize);
    setDataset(0);$('app').hidden=false;fit();jumpDifference();draw();$('fallback').hidden=true;
    function frame(t){if(playing){if(lastFrame!==null)seek(time+Math.min(.25,(t-lastFrame)/1000)*Number($('speed').value));if(time>=duration()){playing=false;$('play').textContent='▶ Play';}}lastFrame=t;if(dirty)draw();requestAnimationFrame(frame);}requestAnimationFrame(frame);
    // Read-only evidence and explicit UI operations used by browser regression tests.
    window.__corridorTest={data:DATA,get:()=>({di,time,index,mode,view:{...view},playing,row:current(),note:d.diagnostics[index]}),
      set:(i,t)=>{setDataset(i);seek(t);draw();},seek:t=>{seek(t);draw();},mode:m=>{setMode(m);draw();},fit,focus,draw,csv:csvText,gaps:gapIndices};
  } catch(error) {
    $('app').hidden=true;$('fallback').hidden=false;$('fallbackMessage').textContent='Interactive viewer could not start: '+error.message+'. The static overview remains available.';console.error(error);
  }
})();
