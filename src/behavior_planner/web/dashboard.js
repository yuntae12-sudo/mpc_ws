(() => {
  const canvas = document.getElementById('map'), ctx = canvas.getContext('2d');
  const $ = id => document.getElementById(id);
  const behaviorNames = ['KEEP','FOLLOW','LEFT CHANGE','RIGHT CHANGE','STOP','YIELD','EMERGENCY STOP'];
  const stateNames = ['KEEP LANE','FOLLOW','LEFT PREPARE','LEFT EXECUTE','RIGHT PREPARE','RIGHT EXECUTE','STOP','EMERGENCY STOP'];
  const lightNames = ['UNKNOWN','RED','YELLOW','GREEN'];
  const lightColors = ['#87939b','#ff5263','#ffd447','#39e58c'];
  const data = {links:[], linkById:new Map(), nodes:[], routeIds:[], route:[], lights:{}, context:null, feature:null, ego:null, objects:null};
  const seen = {context:0, feature:0, ego:0, objects:0};
  const view = {scale:1, x:0, y:0, dragging:false, lastX:0, lastY:0, follow:false};
  let bounds = null, selected = null;

  const finite = v => Number.isFinite(Number(v));
  const fmt = (v, digits=1) => finite(v) && Math.abs(Number(v)) < 1e8 ? Number(v).toFixed(digits) : '∞';
  const esc = s => String(s ?? '').replace(/[&<>]/g, c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]));
  const worldToScreen = p => ({x:p[0]*view.scale+view.x,y:-p[1]*view.scale+view.y});
  const screenToWorld = (x,y) => [(x-view.x)/view.scale,-(y-view.y)/view.scale];
  const now = () => performance.now();

  function resize(){const d=devicePixelRatio||1,r=canvas.getBoundingClientRect();canvas.width=r.width*d;canvas.height=r.height*d;ctx.setTransform(d,0,0,d,0,0);draw();}
  function fitMap(){if(!bounds)return;const r=canvas.getBoundingClientRect(),pad=45;view.scale=Math.min((r.width-pad*2)/(bounds.maxX-bounds.minX),(r.height-pad*2)/(bounds.maxY-bounds.minY));view.x=r.width/2-(bounds.minX+bounds.maxX)/2*view.scale;view.y=r.height/2+(bounds.minY+bounds.maxY)/2*view.scale;view.follow=false;draw();}
  function centerEgo(){if(!data.ego)return;const r=canvas.getBoundingClientRect();view.x=r.width/2-data.ego.position.x*view.scale;view.y=r.height/2+data.ego.position.y*view.scale;draw();}
  function path(points,color,width=1,alpha=1){if(!points||points.length<2)return;ctx.beginPath();let p=worldToScreen(points[0]);ctx.moveTo(p.x,p.y);for(let i=1;i<points.length;i++){p=worldToScreen(points[i]);ctx.lineTo(p.x,p.y)}ctx.globalAlpha=alpha;ctx.strokeStyle=color;ctx.lineWidth=width;ctx.stroke();ctx.globalAlpha=1}
  function circle(x,y,r,color,stroke=null){const p=worldToScreen([x,y]);ctx.beginPath();ctx.arc(p.x,p.y,r,0,Math.PI*2);ctx.fillStyle=color;ctx.fill();if(stroke){ctx.strokeStyle=stroke;ctx.lineWidth=1;ctx.stroke()}}
  function drawSignalNode(n,active=false){const st=data.lights[n.traffic_light_id]??0,r=active?7.2:4.8;circle(n.point[0],n.point[1],r,lightColors[st],active?'#ffffff':'#061018')}

  function draw(){const r=canvas.getBoundingClientRect();ctx.clearRect(0,0,r.width,r.height);ctx.fillStyle='#071018';ctx.fillRect(0,0,r.width,r.height);if(!data.links.length)return;
    for(const l of data.links) path(l.points,'#243744',.65,.65);
    if($('showNodes').checked&&view.scale>1.2) for(const n of data.nodes) circle(n.point[0],n.point[1],1.2,'#526875');
    for(const l of data.route) path(l.points,'#20c8f5',2.4,.95);
    const c=data.context||{}, highlights=[[c.route_required_link_id,'#458cff',5],[c.current_link_id,'#39e58c',4],[c.target_link_id,'#ff4fd8',4]];
    for(const [id,color,w] of highlights){const l=data.linkById.get(id);if(l)path(l.points,color,w,.95)}
    if($('showSignals').checked){const activeId=data.feature?.traffic_light_id||'';for(const n of data.nodes){if(!n.on_stop_line||!n.traffic_light_id||n.traffic_light_id===activeId)continue;drawSignalNode(n)}for(const n of data.nodes){if(!n.on_stop_line||n.traffic_light_id!==activeId)continue;drawSignalNode(n,true)}}
    if($('showObjects').checked&&data.objects){for(const o of data.objects.npc_list||[])circle(o.position.x,o.position.y,3.4,'#ff9b4a');for(const o of data.objects.pedestrian_list||[])circle(o.position.x,o.position.y,3,'#ff75cc');for(const o of data.objects.obstacle_list||[])circle(o.position.x,o.position.y,3.2,'#ff5263')}
    if(data.ego){const p=worldToScreen([data.ego.position.x,data.ego.position.y]),a=-Number(data.ego.heading||0)*Math.PI/180;ctx.save();ctx.translate(p.x,p.y);ctx.rotate(a);ctx.beginPath();ctx.moveTo(11,0);ctx.lineTo(-7,-7);ctx.lineTo(-4,0);ctx.lineTo(-7,7);ctx.closePath();ctx.fillStyle='#fff';ctx.fill();ctx.strokeStyle='#39e58c';ctx.lineWidth=2;ctx.stroke();ctx.restore()}
    if(selected){const l=data.linkById.get(selected);if(l)path(l.points,'#fff',2)}
  }

  function kv(items){return items.map(([k,v])=>`<div><span>${esc(k)}</span><strong title="${esc(v)}">${esc(v)}</strong></div>`).join('')}
  function updatePanel(){const c=data.context||{},f=data.feature||{},b=Number(c.selected_behavior),s=Number(c.behavior_state);if(f.traffic_light_id&&finite(f.traffic_light_state))data.lights[f.traffic_light_id]=Number(f.traffic_light_state);$('behaviorName').textContent=behaviorNames[b]||'WAITING';$('behaviorCode').textContent=finite(b)?b:'–';$('stateName').textContent=stateNames[s]||'–';$('stateCode').textContent=finite(s)?`STATE ${s}`:'–';
    $('linkGrid').innerHTML=kv([['Current',c.current_link_id||f.current_link_id||'–'],['Route Required',c.route_required_link_id||f.route_required_link_id||'–'],['Target',c.target_link_id||'–'],['Candidates',(c.candidate_link_ids||[]).join(', ')||'–'],['Ego speed',`${fmt(f.ego_speed)} m/s`],['Desired',`${fmt(c.desired_speed)} m/s`],['Route s',fmt(f.ego_s)],['Route d',fmt(f.ego_d)]]);
    $('riskGrid').innerHTML=kv([['Front vehicle',f.has_front_vehicle?'YES':'NO'],['Front gap',`${fmt(f.front_gap)} m`],['Relative speed',`${fmt(f.front_relative_speed)} m/s`],['Front TTC',`${fmt(f.front_ttc)} s`],['Predicted TTC',`${fmt(f.predicted_front_ttc)} s`],['Required decel',`${fmt(f.required_deceleration)} m/s²`],['Signal',`${f.traffic_light_id||'–'} / ${lightNames[f.traffic_light_state]||'UNKNOWN'}`],['Stop distance',`${fmt(f.distance_to_stop_line)} m`],['Min TTC',`${fmt(f.min_ttc)} s`],['Emergency',f.emergency_risk?'YES':'NO']]);
    const flags=[['KEEP',c.enable_keep],['FOLLOW',c.enable_follow],['LEFT',c.enable_left_change],['RIGHT',c.enable_right_change],['STOP',c.enable_stop],['E-STOP',c.enable_emergency_stop],['NO LEFT',c.forbid_left_change,'blocked'],['NO RIGHT',c.forbid_right_change,'blocked'],['FORCE STOP',c.force_stop,'blocked']];$('enableGrid').innerHTML=flags.map(([n,on,kind])=>`<div class="flag ${on?(kind||'on'):''}">${n}</div>`).join('');
    const scores=[['KEEP',c.keep_score],['FOLLOW',c.follow_score],['LEFT',c.left_change_score],['RIGHT',c.right_change_score],['STOP',c.stop_score],['E-STOP',c.emergency_stop_score]],vals=scores.map(x=>Number(x[1])).filter(Number.isFinite),min=Math.min(0,...vals),max=Math.max(1,...vals);$('scores').innerHTML=scores.map(([n,v])=>{const num=Number(v),pct=Number.isFinite(num)?Math.max(0,Math.min(100,(num-min)/(max-min)*100)):0;return `<div class="score"><span>${n}</span><div class="bar"><i style="width:${pct}%"></i></div><b>${fmt(v,0)}</b></div>`}).join('');
    $('reasons').innerHTML=`<div>${esc(c.decision_reason||'–')}</div><div class="hard">${esc(c.hard_rule_reason||'–')}</div>`;updateHealth();}
  function updateHealth(){const t=now(),status=k=>seen[k]?(t-seen[k]<2500?'LIVE':'STALE'):'WAIT';$('health').innerHTML=kv([['BehaviorContext',status('context')],['FeatureDebug',status('feature')],['Ego',status('ego')],['Objects',status('objects')]]);}

  function nearestLink(wx,wy){let best=null,bd=Infinity;for(const l of data.links)for(let i=1;i<l.points.length;i++){const a=l.points[i-1],b=l.points[i],vx=b[0]-a[0],vy=b[1]-a[1],d=vx*vx+vy*vy;if(!d)continue;const t=Math.max(0,Math.min(1,((wx-a[0])*vx+(wy-a[1])*vy)/d)),dx=wx-(a[0]+t*vx),dy=wy-(a[1]+t*vy),q=dx*dx+dy*dy;if(q<bd){bd=q;best=l}}return Math.sqrt(bd)*view.scale<14?best:null}
  function selectLink(x,y){const [wx,wy]=screenToWorld(x,y),l=nearestLink(wx,wy);selected=l?.idx||null;$('selectedLink').textContent=l?`id: ${l.idx}\nroad: ${l.road_id||'–'}  lane: ${l.ego_lane??'–'}\nspeed: ${l.max_speed??'–'}\nleft: ${l.left_lane_change_dst_link_idx||'–'}\nright: ${l.right_lane_change_dst_link_idx||'–'}`:'근처 링크가 없습니다.';draw()}

  canvas.addEventListener('wheel',e=>{e.preventDefault();const r=canvas.getBoundingClientRect(),mx=e.clientX-r.left,my=e.clientY-r.top,[wx,wy]=screenToWorld(mx,my),factor=Math.exp(-e.deltaY*.001);view.scale=Math.max(.05,Math.min(80,view.scale*factor));view.x=mx-wx*view.scale;view.y=my+wy*view.scale;view.follow=false;draw()},{passive:false});
  canvas.addEventListener('mousedown',e=>{view.dragging=true;view.lastX=e.clientX;view.lastY=e.clientY});window.addEventListener('mouseup',()=>view.dragging=false);window.addEventListener('mousemove',e=>{const r=canvas.getBoundingClientRect(),[wx,wy]=screenToWorld(e.clientX-r.left,e.clientY-r.top);$('cursor').textContent=`x ${wx.toFixed(1)}  y ${wy.toFixed(1)}`;if(view.dragging){view.x+=e.clientX-view.lastX;view.y+=e.clientY-view.lastY;view.lastX=e.clientX;view.lastY=e.clientY;view.follow=false;draw()}});canvas.addEventListener('click',e=>{if(Math.abs(e.movementX||0)<2){const r=canvas.getBoundingClientRect();selectLink(e.clientX-r.left,e.clientY-r.top)}});
  $('fit').onclick=fitMap;$('follow').onclick=()=>{view.follow=!view.follow;if(view.follow)centerEgo();$('follow').textContent=view.follow?'추적 중':'Ego 추적'};for(const id of ['showNodes','showObjects','showSignals'])$(id).onchange=draw;window.addEventListener('resize',resize);

  async function loadMap(){try{const noCache={cache:'no-store'};const [links,nodes,routeText,lightText]=await Promise.all([fetch('../src/config/link_set.json',noCache).then(r=>r.json()),fetch('../src/config/node_set.json',noCache).then(r=>r.json()),fetch('../src/config/route_links_1.yaml',noCache).then(r=>r.text()),fetch('../src/config/traffic_light_mock.yaml',noCache).then(r=>r.text())]);data.links=Array.isArray(links)?links:links.links;data.nodes=Array.isArray(nodes)?nodes:nodes.nodes;for(const l of data.links)data.linkById.set(l.idx,l);data.routeIds=[...routeText.matchAll(/-\s*["']?([A-Za-z0-9_]+)["']?/g)].map(x=>x[1]);data.route=data.routeIds.map(x=>data.linkById.get(x)).filter(Boolean);for(const m of lightText.matchAll(/^\s*([A-Za-z0-9_]+):\s*(RED|YELLOW|GREEN|UNKNOWN)\s*$/gm))data.lights[m[1]]={UNKNOWN:0,RED:1,YELLOW:2,GREEN:3}[m[2]];bounds={minX:Infinity,maxX:-Infinity,minY:Infinity,maxY:-Infinity};for(const l of data.links)for(const p of l.points){bounds.minX=Math.min(bounds.minX,p[0]);bounds.maxX=Math.max(bounds.maxX,p[0]);bounds.minY=Math.min(bounds.minY,p[1]);bounds.maxY=Math.max(bounds.maxY,p[1])}$('mapStatus').textContent=`MAP ${data.links.length} LINKS`;$('mapStatus').classList.add('ok');resize();fitMap()}catch(e){$('mapStatus').textContent='MAP ERROR';$('mapStatus').classList.add('bad');console.error(e)}}

  function connectRos(){const ws=new WebSocket(`ws://${location.hostname}:9090`);ws.onopen=()=>{ $('rosStatus').textContent='ROS CONNECTED';$('rosStatus').className='chip ok';[['/Ego_topic','morai_msgs/EgoVehicleStatus'],['/Object_topic','morai_msgs/ObjectStatusList']].forEach(([topic,type])=>ws.send(JSON.stringify({op:'subscribe',topic,type,throttle_rate:100,queue_length:1}))) };ws.onmessage=e=>{const m=JSON.parse(e.data);if(m.op!=='publish')return;if(m.topic==='/Ego_topic'){data.ego=m.msg;seen.ego=now();if(view.follow)centerEgo()}else if(m.topic==='/Object_topic'){data.objects=m.msg;seen.objects=now()}updatePanel();draw()};ws.onclose=()=>{$('rosStatus').textContent='ROS OFFLINE';$('rosStatus').className='chip bad';setTimeout(connectRos,1500)};ws.onerror=()=>ws.close()}
  async function pollBehavior(){try{const snapshot=await fetch('../api/snapshot',{cache:'no-store'}).then(r=>r.json());if(snapshot.context){data.context=snapshot.context;seen.context=now()}if(snapshot.feature){data.feature=snapshot.feature;seen.feature=now()}updatePanel();draw()}catch(e){console.warn('behavior snapshot unavailable',e)}}
  setInterval(updateHealth,1000);setInterval(pollBehavior,100);loadMap();connectRos();pollBehavior();updatePanel();
})();
