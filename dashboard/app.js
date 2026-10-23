"use strict";

const $=id=>document.getElementById(id);
const palette=["#0067b1","#d14b16","#00844a","#9635a8","#af7800","#008b9b","#cb365b","#444444"];
const storageKey="simlap.telemetry.layout.v1";
let run=null, comparison=null, cursor=0, range=[0,1], selected=new Set(), activePlot="", views=[];
let loadingVersion=0, compareVersion=0, playing=false, lastFrame=0, drawPending=false;

function element(tag,text,className) {
  const node=document.createElement(tag);
  if (text!==undefined) node.textContent=text;
  if (className) node.className=className;
  return node;
}
function button(text,action,title=text) {const node=element("button",text);node.title=title;node.setAttribute("aria-label",title);node.onclick=action;return node;}
function select(options,value,action,label) {
  const node=element("select");
  options.forEach(([key,text])=>node.add(new Option(text,key)));
  node.value=value;node.onchange=()=>action(node.value);if(label)node.setAttribute("aria-label",label);return node;
}
function check(checked,action,label) {const node=element("input");node.type="checkbox";node.checked=checked;node.onchange=()=>action(node.checked);node.setAttribute("aria-label",label);return node;}
function label(text,node) {const result=element("label",text);result.append(node);return result;}
function newPlot(title,channels=[],x="time_s") {
  return {id:`p${Date.now()}${Math.random().toString(16).slice(2,7)}`,title,x,wide:false,equal:false,
    series:channels.map((channel,i)=>({channel,color:palette[i%palette.length],axis:"left",visible:true}))};
}
function defaults() {
  const battery=newPlot("Battery",["battery_current_a","battery_voltage_v"]);
  battery.series[1].axis="right";
  return {columns:2,plots:[newPlot("Speed",["speed_mps"]),newPlot("Acceleration",["ax_mps2","ay_mps2"]),
    newPlot("Power",["battery_terminal_power_w","motor_shaft_power_w","wheel_drive_power_w"]),
    battery]};
}
function readLayout() {
  try {
    const saved=JSON.parse(localStorage.getItem(storageKey));
    if (!saved || ![1,2,3].includes(saved.columns) || !Array.isArray(saved.plots) || !saved.plots.length) return defaults();
    if (new Set(saved.plots.map(p=>p.id)).size!==saved.plots.length) return defaults();
    if (!saved.plots.every(p=>typeof p.id==="string"&&typeof p.title==="string"&&typeof p.x==="string"&&Array.isArray(p.series)&&
      p.series.every(s=>typeof s.channel==="string"&&/^#[0-9a-f]{6}$/i.test(s.color)&&["left","right"].includes(s.axis)&&typeof s.visible==="boolean"))) return defaults();
    return saved;
  } catch {return defaults();}
}
let layout=readLayout();
function saveLayout() {
  try {localStorage.setItem(storageKey,JSON.stringify(layout));$("layout-status").textContent="Layout saved in this browser";}
  catch {$("layout-status").textContent="Browser storage unavailable; layout is temporary";}
}
function message(text) {$("notice").textContent=text;$("notice").hidden=!text;}
async function api(path) {const response=await fetch(path);if(!response.ok)throw new Error(`Request failed (${response.status}). Check the server and refresh.`);return response.json();}
function duration() {return Math.max(run?.summary.duration_s || 0,comparison?.summary.duration_s || 0,.001);}
function allChannels() {return [...new Set([...(run?.columns || []),...(comparison?.columns || [])])];}
function filteredChannels() {const query=$("search").value.toLowerCase();return allChannels().filter(key=>key.toLowerCase().includes(query));}

async function refresh() {
  try {
    const runs=await api("/api/runs"), old=run?.summary.run_id;
    $("run-select").replaceChildren();$("compare").replaceChildren(new Option("None",""));
    runs.forEach(r=>{const name=`${r.maneuver} | ${r.run_id}`;$("run-select").add(new Option(name,r.run_id));$("compare").add(new Option(name,r.run_id));});
    if (!runs.length) {run=null;playing=false;$("workspace").hidden=true;message("No completed runs. Execute python -m simlap.run all, then Refresh.");return;}
    await selectRun(runs.some(r=>r.run_id===old)?old:runs[0].run_id);
  } catch(error) {message(error.message);}
}
async function selectRun(id) {
  const version=++loadingVersion;++compareVersion;playing=false;$("play").textContent="Play";
  message("Loading full-resolution telemetry…");
  try {
    const data=await api(`/api/run?id=${encodeURIComponent(id)}`);
    if(version!==loadingVersion)return;
    run=data;comparison=null;cursor=0;$("compare").value="";$("run-select").value=id;
    $("workspace").hidden=false;selected.clear();message("");
    const s=run.summary,c=run.config;
    $("run-info").textContent=`${s.maneuver} | ${s.reason} | ${run.rows.length.toLocaleString()} samples, full resolution | ${s.duration_s.toFixed(3)} s | motor ${c.motor.rating}, ${c.motor.peak_power_kw} kW peak | mass ${c.vehicle.mass_kg} kg`;
    $("config").textContent=JSON.stringify(c,null,2);$("checks").textContent=JSON.stringify(s,null,2);
    $("downloads").replaceChildren();
    ["telemetry.csv","engine.log","config.json","metadata.json","summary.json","channels.json"].forEach(name=>{
      const link=element("a",name);link.href=`/api/file?id=${encodeURIComponent(id)}&name=${name}`;link.download=name;$("downloads").append(link);
    });
    fitRange();renderChannels();renderPlots();
  }catch(error){if(version===loadingVersion)message(error.message);}
}
function renderChannels() {
  $("channel-count").textContent=`${allChannels().length} logged`;
  $("channels").replaceChildren();
  filteredChannels().forEach(key=>{
    const populated=[run,comparison].filter(Boolean).some(data=>data.rows.some(row=>Number.isFinite(row[key])));
    const row=element("label",undefined,`channel${populated?"":" channel-empty"}`);
    row.title=run?.channel_descriptions?.[key] || key;
    const box=check(selected.has(key),value=>{value?selected.add(key):selected.delete(key);updateSelection();},`Select ${key}`);
    row.append(box,element("span",key,"channel-name"),element("span",populated?channelUnit(key):"no data","channel-unit"));
    row.ondblclick=event=>{event.preventDefault();addChannels([key]);};$("channels").append(row);
  });updateSelection();
}
function updateSelection() {$("add-channels").textContent=`Add selected (${selected.size})`;$("add-channels").disabled=!selected.size;}
function setActive(id) {
  activePlot=id;$("target-plot").value=id;
  views.forEach(v=>v.root.classList.toggle("active",v.plot.id===id));
}
function addChannels(channels) {
  const plot=layout.plots.find(p=>p.id===$("target-plot").value) || layout.plots[0];
  channels.forEach(channel=>{if(!plot.series.some(s=>s.channel===channel))plot.series.push({channel,color:palette[plot.series.length%palette.length],axis:"left",visible:true});});
  activePlot=plot.id;saveLayout();renderPlots();
}
function renderPlots() {
  views=[];$("plots").replaceChildren();$("plots").style.setProperty("--columns",layout.columns);$("columns").value=layout.columns;
  $("target-plot").replaceChildren(...layout.plots.map((p,i)=>new Option(`${i+1}: ${p.title}`,p.id)));
  layout.plots.forEach((plot,index)=>{
    const root=element("section",undefined,`plot${plot.wide?" wide":""}`),head=element("div",undefined,"plot-head");
    root.dataset.plotId=plot.id;root.onpointerdown=()=>setActive(plot.id);
    const title=element("input",undefined,"plot-title");title.value=plot.title;title.setAttribute("aria-label",`Plot ${index+1} title`);
    title.onchange=()=>{plot.title=title.value;saveLayout();renderPlots();};
    const move=offset=>{const other=index+offset;[layout.plots[index],layout.plots[other]]=[layout.plots[other],layout.plots[index]];saveLayout();renderPlots();};
    const up=button("↑",()=>move(-1),"Move plot earlier"),down=button("↓",()=>move(1),"Move plot later");up.disabled=index===0;down.disabled=index===layout.plots.length-1;
    const remove=button("×",()=>{layout.plots.splice(index,1);if(!layout.plots.length)layout.plots.push(newPlot("Plot"));saveLayout();renderPlots();},"Remove plot");
    head.append(element("span",`P${index+1}`,"plot-number"),title,up,down,remove);
    const settings=element("div",undefined,"plot-settings");
    const channels=[...new Set([...allChannels(),plot.x])];
    settings.append(label("X",select(channels.map(key=>[key,`${key} [${channelUnit(key)}]`]),plot.x,value=>{plot.x=value;saveLayout();requestDraw();},"X-axis channel")),
      label("Full width",check(plot.wide,value=>{plot.wide=value;saveLayout();renderPlots();},"Full-width plot")),
      label("Equal XY",check(plot.equal,value=>{plot.equal=value;saveLayout();requestDraw();},"Equal X and Y scale")));
    const canvas=element("canvas");canvas.setAttribute("role","img");canvas.setAttribute("aria-label",`${plot.title} telemetry plot`);
    const legend=element("div",undefined,"legend"),table=element("table"),header=element("tr");
    ["Show","Color","Channel [unit]","Axis","Selected ━",...(comparison?["Compare ┄"]:[]),"Move to",""].forEach(text=>header.append(element("th",text)));table.append(header);
    const values=[];
    plot.series.forEach((s,seriesIndex)=>{
      const row=element("tr");
      const cell=node=>{const td=element("td");td.append(node);row.append(td);return td;};
      cell(check(s.visible,value=>{s.visible=value;saveLayout();requestDraw();},`Show ${s.channel}`));
      const color=element("input");color.type="color";color.value=s.color;color.setAttribute("aria-label",`Color for ${s.channel}`);
      color.oninput=()=>{s.color=color.value;saveLayout();requestDraw();};cell(color);
      const name=element("span",`${s.channel} [${channelUnit(s.channel)}]`);name.title=name.textContent;cell(name).className="trace-name";
      cell(select([["left","L"],["right","R"]],s.axis,value=>{s.axis=value;saveLayout();requestDraw();},`Axis for ${s.channel}`));
      const current=cell(element("span"));current.className="value";
      const compare=comparison?cell(element("span")):null;if(compare)compare.className="value";
      values.push({series:s,current,compare,name});
      cell(select(layout.plots.map((p,i)=>[p.id,`P${i+1}: ${p.title}`]),plot.id,id=>{
        if(id===plot.id)return;const destination=layout.plots.find(p=>p.id===id);
        if(!destination.series.some(item=>item.channel===s.channel))destination.series.push({...s});
        plot.series.splice(seriesIndex,1);activePlot=id;saveLayout();renderPlots();
      },`Move ${s.channel} to plot`));
      cell(button("×",()=>{plot.series.splice(seriesIndex,1);saveLayout();renderPlots();},`Remove ${s.channel}`));table.append(row);
    });
    legend.append(table);const note=element("div",undefined,"plot-note");root.append(head,settings,canvas,legend,note);$("plots").append(root);
    views.push({root,canvas,plot,values,note});attachPlotEvents(canvas,plot);
  });
  setActive(layout.plots.some(p=>p.id===activePlot)?activePlot:layout.plots[0].id);requestDraw();
}
function setRange(start,end) {
  if(!Number.isFinite(start)||!Number.isFinite(end)||start<0||end<=start||end>duration()+1e-8){message("Enter a valid time range within the loaded runs.");return;}
  range=[start,end];cursor=Math.max(start,Math.min(end,cursor));$("time-start").value=Number(start.toFixed(6));$("time-end").value=Number(end.toFixed(6));message("");requestDraw();
}
function fitRange() {setRange(0,duration());}
function attachPlotEvents(canvas,plot) {
  let dragStart=null;
  const xValue=event=>{const g=canvas.plotGeometry;if(!g)return 0;const px=event.clientX-canvas.getBoundingClientRect().left;return g.xr[0]+Math.max(0,Math.min(1,(px-g.area.left)/(g.area.right-g.area.left)))*(g.xr[1]-g.xr[0]);};
  canvas.onpointerdown=event=>{dragStart={x:event.clientX,value:xValue(event)};canvas.setPointerCapture(event.pointerId);};
  canvas.onpointerup=event=>{
    if(!dragStart)return;
    const end=xValue(event);
    if(plot.x==="time_s"&&Math.abs(event.clientX-dragStart.x)>6)setRange(Math.min(dragStart.value,end),Math.max(dragStart.value,end));
    else if(plot.x==="time_s"){cursor=Math.max(range[0],Math.min(range[1],end));requestDraw();}
    else {
      const rows=canvas.plotGeometry.samples.filter(row=>Number.isFinite(row[plot.x]));
      const nearest=rows.reduce((best,row)=>!best||Math.abs(row[plot.x]-end)<Math.abs(best[plot.x]-end)?row:best,null);
      if(nearest){cursor=nearest.time_s;requestDraw();}
    }
    dragStart=null;
  };
  canvas.onpointercancel=()=>{dragStart=null;};canvas.ondblclick=fitRange;
}
function requestDraw() {if(!drawPending){drawPending=true;requestAnimationFrame(()=>{drawPending=false;draw();});}}
function draw() {
  if(!run)return;
  $("cursor-time").textContent=`${cursor.toFixed(4)} s`;$("scrub").value=Math.round((cursor-range[0])/(range[1]-range[0])*1000);
  const current=nearestSample(run.rows,cursor),other=comparison?nearestSample(comparison.rows,cursor):null;
  views.forEach(view=>{
    view.note.textContent=drawTelemetryPlot(view.canvas,view.plot,[run,comparison].filter(Boolean),range,cursor);
    view.values.forEach(({series,current:cell,compare,name})=>{
      cell.textContent=formatNumber(current?.[series.channel]);if(compare)compare.textContent=formatNumber(other?.[series.channel]);
      name.style.color=series.color;name.style.opacity=series.visible?"1":"0.45";
    });
  });
}
$("refresh").onclick=refresh;$("run-select").onchange=()=>selectRun($("run-select").value);
$("compare").onchange=async()=>{
  const id=$("compare").value,version=++compareVersion,runVersion=loadingVersion;
  try {const data=id?await api(`/api/run?id=${encodeURIComponent(id)}`):null;if(version!==compareVersion||runVersion!==loadingVersion)return;
    comparison=data;fitRange();renderChannels();renderPlots();
  }catch(error){if(version===compareVersion)message(error.message);}
};
$("search").oninput=renderChannels;
$("select-filtered").onclick=()=>{filteredChannels().forEach(key=>selected.add(key));renderChannels();};
$("clear-selection").onclick=()=>{selected.clear();renderChannels();};
$("add-channels").onclick=()=>addChannels([...selected]);$("target-plot").onchange=()=>setActive($("target-plot").value);
$("add-plot").onclick=()=>{const plot=newPlot(`Plot ${layout.plots.length+1}`);layout.plots.push(plot);activePlot=plot.id;saveLayout();renderPlots();};
$("columns").onchange=()=>{layout.columns=Number($("columns").value);saveLayout();renderPlots();};
$("reset-layout").onclick=()=>{layout=defaults();saveLayout();renderPlots();};
$("apply-range").onclick=()=>setRange(Number($("time-start").value),Number($("time-end").value));$("fit-range").onclick=fitRange;
$("scrub").oninput=()=>{cursor=range[0]+Number($("scrub").value)/1000*(range[1]-range[0]);requestDraw();};
$("play").onclick=()=>{playing=!playing;if(playing&&cursor>=range[1])cursor=range[0];lastFrame=0;$("play").textContent=playing?"Pause":"Play";};
window.addEventListener("resize",requestDraw);
function animate(now) {
  if(playing&&run){if(lastFrame)cursor=Math.min(range[1],cursor+(now-lastFrame)/1000);lastFrame=now;requestDraw();if(cursor>=range[1]){playing=false;$("play").textContent="Play";}}
  requestAnimationFrame(animate);
}
requestAnimationFrame(animate);refresh();
