"use strict";

const $ = id => document.getElementById(id);
const colors = ["#28765a", "#db713b", "#537d9e", "#9574aa"];
const names = {acceleration: "Acceleration", braking: "Braking", cornering: "Steady-state cornering"};
let runs = [], run = null, comparison = null, fraction = 0, playing = false, lastFrame = 0, selectionVersion = 0;
const views = {
  motion: [
    ["Speed · km/h", [["speed_mps", "Ground speed", 3.6]]],
    ["Acceleration · m/s²", [["ax_mps2", "Longitudinal"], ["ay_mps2", "Lateral"]]],
    ["Yaw rate · rad/s", [["yaw_rate_radps", "Yaw rate"]]],
    ["Lean estimate · degrees", [["lean_estimate_deg", "Equilibrium lean"]]]
  ],
  power: [
    ["Power · kW", [["battery_terminal_power_w", "Pack", .001], ["motor_shaft_power_w", "Shaft", .001], ["wheel_drive_power_w", "Wheel", .001]]],
    ["Heat generation · kW", [["battery_loss_w", "Battery", .001], ["motor_loss_w", "Motor", .001], ["inverter_loss_w", "Inverter", .001], ["gearbox_loss_w", "Gearbox", .001]]],
    ["Pack current · A", [["battery_current_a", "Current"]]],
    ["State of charge · %", [["soc", "SOC", 100]]]
  ],
  grip: [
    ["Grip utilization · fraction", [["front_grip_usage", "Front"], ["rear_grip_usage", "Rear"]]],
    ["Normal load · N", [["front_normal_n", "Front"], ["rear_normal_n", "Rear"]]],
    ["Lateral force · N", [["front_fy_n", "Front"], ["rear_fy_n", "Rear"]]],
    ["Slip angle · degrees", [["front_slip_rad", "Front", 180/Math.PI], ["rear_slip_rad", "Rear", 180/Math.PI]]]
  ],
  controls: [
    ["Throttle · fraction", [["command_throttle", "Command"], ["throttle", "Actual"]]],
    ["Brake · fraction", [["command_brake", "Command"], ["brake", "Actual"]]],
    ["Steering · degrees", [["steer_target_rad", "Target", 180/Math.PI], ["steer_rad", "Actual", 180/Math.PI]]],
    ["Motor torque · Nm", [["requested_drive_torque_nm", "Drive request"], ["motor_torque_nm", "Delivered"]]]
  ]
};

function fmt(value, digits = 1) { return Number.isFinite(value) ? value.toFixed(digits) : "—"; }
function el(tag, text, className) {
  const node = document.createElement(tag);
  if (text !== undefined) node.textContent = text;
  if (className) node.className = className;
  return node;
}
async function api(path) {
  const response = await fetch(path);
  if (!response.ok) throw new Error(`Could not load results (${response.status}). Check the dashboard server and refresh.`);
  return response.json();
}
function error(message) { $("notice").textContent = message; $("notice").hidden = false; }

async function refresh() {
  try {
    runs = await api("/api/runs");
    $("runs").replaceChildren();
    $("compare").replaceChildren(new Option("No comparison", ""));
    runs.forEach(item => {
      const button = el("button");
      button.dataset.id = item.run_id;
      button.append(el("strong", names[item.maneuver]), el("small", `${fmt(item.duration_s)} s · ${fmt(item.max_speed_kph)} km/h · ${item.run_id.slice(9,15)}`));
      button.onclick = () => selectRun(item.run_id);
      $("runs").append(button);
      $("compare").add(new Option(`${names[item.maneuver]} · ${item.run_id.slice(0,15)}`, item.run_id));
    });
    if (!runs.length) {
      run = null; playing = false; $("workspace").hidden = true;
      error("No completed simulations yet. From the repository run: python -m simlap.run all, then refresh here.");
    } else await selectRun(run && runs.some(r => r.run_id === run.summary.run_id) ? run.summary.run_id : runs[0].run_id);
  } catch (err) { error(err.message); }
}

async function selectRun(id) {
  const version = ++selectionVersion;
  try {
    const data = await api(`/api/run?id=${encodeURIComponent(id)}`);
    if (version !== selectionVersion) return;
    run = data; comparison = null; $("compare").value = "";
    fraction = 0; playing = false; $("play").textContent = "▶ Play";
    $("notice").hidden = true; $("workspace").hidden = false;
    document.querySelectorAll("nav button").forEach(b => b.classList.toggle("active", b.dataset.id === id));
    const s = run.summary, c = run.config;
    $("title").textContent = names[s.maneuver];
    $("run-date").textContent = new Date(run.metadata.created_utc).toLocaleString();
    $("subtitle").textContent = `${c.motor.peak_power_kw} kW peak motor · ${c.motor.rating} map · ${c.vehicle.mass_kg} kg including rider`;
    $("status").textContent = s.reason.replaceAll("_", " ").toUpperCase();
    $("status").classList.toggle("warning", !["stopped", "duration_completed"].includes(s.reason));
    const special = s.maneuver === "acceleration" ? ["0–100 km/h", s.zero_to_100_s, "s"] :
      s.maneuver === "braking" ? ["Stopping distance", s.stopping_distance_m, "m"] : ["Max radius error", s.max_radius_error_m, "m"];
    $("summary").replaceChildren();
    [["Maximum speed", s.max_speed_kph, "km/h"], special, ["Net pack energy", s.terminal_energy_wh, "Wh"], ["Duration", s.duration_s, "s"]].forEach(([label, value, unit]) => {
      const card = el("div", undefined, "metric"), number = el("div", fmt(value, label.includes("radius") ? 4 : 2), "value");
      number.append(el("small", unit)); card.append(el("div", label, "label"), number); $("summary").append(card);
    });
    $("config-strip").textContent = `${c.battery.capacity_kwh} kWh pack · ${c.battery.nominal_voltage_v} V nominal · μ ${c.tires.friction_coefficient} · Cd ${c.vehicle.drag_coefficient} · Δt ${c.simulation.dt_s * 1000} ms`;
    $("config").textContent = JSON.stringify(c, null, 2);
    $("checks").textContent = JSON.stringify(s, null, 2);
    $("signal").replaceChildren();
    run.columns.filter(key => run.rows.some(row => Number.isFinite(row[key]))).forEach(key => $("signal").add(new Option(key, key)));
    $("signal").value = "battery_voltage_v";
    $("downloads").replaceChildren();
    ["telemetry.csv", "engine.log", "config.json", "metadata.json", "summary.json", "channels.json"].forEach(name => {
      const link = el("a", `↓ ${name}`); link.href = `/api/file?id=${encodeURIComponent(id)}&name=${name}`; link.download = name; $("downloads").append(link);
    });
    buildCharts(); draw();
  } catch (err) { error(err.message); }
}

function canvasContext(canvas) {
  const rect = canvas.getBoundingClientRect(), ratio = window.devicePixelRatio || 1;
  canvas.width = Math.round(rect.width * ratio); canvas.height = Math.round(rect.height * ratio);
  const ctx = canvas.getContext("2d"); ctx.scale(ratio, ratio);
  ctx.font = "10px -apple-system, sans-serif";
  return [ctx, rect.width, rect.height];
}
function buildCharts() {
  $("charts").replaceChildren();
  views[$("view").value].forEach(([title, series]) => {
    const box = el("div"), canvas = el("canvas", undefined, "chart"), legend = el("div", undefined, "legend");
    canvas.setAttribute("role", "img"); canvas.setAttribute("aria-label", title);
    box.append(el("div", title, "chart-title"), canvas, legend);
    series.forEach(([,label], i) => { const item = el("span", label), line = el("i"); line.style.background = colors[i]; item.prepend(line); legend.append(item); });
    canvas.onclick = event => scrubChart(event, canvas);
    $("charts").append(box);
  });
}
function scrubChart(event, canvas) {
  if (!run) return;
  const duration = Math.max(run.summary.duration_s, comparison?.summary.duration_s || 0, .001);
  fraction = Math.max(0, Math.min(1, (event.clientX - canvas.getBoundingClientRect().left - 48) / (canvas.clientWidth - 62) * duration / Math.max(run.summary.duration_s, .001)));
  draw();
}
function chart(canvas, series) {
  const [ctx, width, height] = canvasContext(canvas), left = 48, right = width - 14, top = 14, bottom = height - 28;
  const duration = Math.max(run.summary.duration_s, comparison?.summary.duration_s || 0, .001);
  let min = Infinity, max = -Infinity;
  [run, comparison].filter(Boolean).forEach(data => data.rows.forEach(row => series.forEach(([key,,scale=1]) => {
    if (Number.isFinite(row[key])) { min = Math.min(min, row[key] * scale); max = Math.max(max, row[key] * scale); }
  })));
  if (!Number.isFinite(min)) return;
  const pad = Math.max((max - min) * .13, Math.abs(max) * .01, 1e-4); min -= pad; max += pad;
  const x = t => left + t / duration * (right - left), y = v => bottom - (v - min) / (max - min) * (bottom - top);
  ctx.lineWidth = 1; ctx.fillStyle = "#748079";
  for (let i=0;i<=4;i++) {
    const value = min + i * (max-min)/4, py = y(value);
    ctx.strokeStyle = "#e9eae4"; ctx.beginPath(); ctx.moveTo(left,py); ctx.lineTo(right,py); ctx.stroke();
    ctx.textAlign = "right"; ctx.fillText(Math.abs(value) >= 10000 ? value.toExponential(1) : fmt(value, Math.abs(max-min)<1 ? 3 : 1), left-7, py+3);
    ctx.textAlign = "center"; ctx.fillText(fmt(duration*i/4,1), x(duration*i/4),height-9);
  }
  [run, comparison].filter(Boolean).forEach((data, compareIndex) => series.forEach(([key,,scale=1], index) => {
    ctx.strokeStyle = colors[index]; ctx.lineWidth = compareIndex ? 1.5 : 2; ctx.globalAlpha = compareIndex ? .6 : 1; ctx.setLineDash(compareIndex ? [5,4] : []); ctx.beginPath();
    let started = false;
    data.rows.forEach(row => { if (!Number.isFinite(row[key])) { started = false; return; } const px=x(row.time_s),py=y(row[key]*scale); if(started) ctx.lineTo(px,py); else ctx.moveTo(px,py); started=true; });
    ctx.stroke();
  }));
  ctx.globalAlpha=1; ctx.setLineDash([3,4]); ctx.strokeStyle="#9aa59d"; ctx.lineWidth=1;
  ctx.beginPath(); ctx.moveTo(x(fraction*run.summary.duration_s),top);ctx.lineTo(x(fraction*run.summary.duration_s),bottom);ctx.stroke(); ctx.setLineDash([]);
}
function currentRow() {
  const t = fraction * run.summary.duration_s;
  let lo=0, hi=run.rows.length-1;
  while(lo<hi) { const mid=Math.floor((lo+hi)/2); if(run.rows[mid].time_s<t) lo=mid+1;else hi=mid; }
  return run.rows[lo];
}
function drawPath(row) {
  const [ctx,w,h] = canvasContext($("path")), rows=run.rows;
  let minX=Infinity,maxX=-Infinity,minY=Infinity,maxY=-Infinity;
  rows.forEach(p=>{minX=Math.min(minX,p.x_m);maxX=Math.max(maxX,p.x_m);minY=Math.min(minY,p.y_m);maxY=Math.max(maxY,p.y_m);});
  const radius = run.metadata.parameters.radius_m;
  if(radius) { minX=Math.min(minX,-Math.abs(radius));maxX=Math.max(maxX,Math.abs(radius));minY=Math.min(minY,radius-Math.abs(radius));maxY=Math.max(maxY,radius+Math.abs(radius)); }
  const scale=Math.min((w-60)/Math.max(maxX-minX,5),(h-55)/Math.max(maxY-minY,5));
  const x=v=>w/2+(v-(minX+maxX)/2)*scale,y=v=>h/2-(v-(minY+maxY)/2)*scale;
  ctx.strokeStyle="#e5e8df";ctx.lineWidth=1;
  for(let i=1;i<8;i++){ctx.beginPath();ctx.moveTo(w*i/8,0);ctx.lineTo(w*i/8,h);ctx.stroke();ctx.beginPath();ctx.moveTo(0,h*i/8);ctx.lineTo(w,h*i/8);ctx.stroke();}
  if(radius){ctx.strokeStyle="#a4afa1";ctx.setLineDash([5,5]);ctx.beginPath();ctx.arc(x(0),y(radius),Math.abs(radius)*scale,0,2*Math.PI);ctx.stroke();ctx.setLineDash([]);}
  ctx.strokeStyle="#c7d7c8";ctx.lineWidth=3;ctx.beginPath();rows.forEach((p,i)=>i?ctx.lineTo(x(p.x_m),y(p.y_m)):ctx.moveTo(x(p.x_m),y(p.y_m)));ctx.stroke();
  ctx.strokeStyle=colors[0];ctx.beginPath();rows.filter(p=>p.time_s<=row.time_s).forEach((p,i)=>i?ctx.lineTo(x(p.x_m),y(p.y_m)):ctx.moveTo(x(p.x_m),y(p.y_m)));ctx.stroke();
  ctx.save();ctx.translate(x(row.x_m),y(row.y_m));ctx.rotate(-row.heading_rad);ctx.fillStyle=colors[1];ctx.beginPath();ctx.moveTo(10,0);ctx.lineTo(-6,-5);ctx.lineTo(-3,0);ctx.lineTo(-6,5);ctx.closePath();ctx.fill();ctx.restore();
  const bar=Math.max(1,10**Math.floor(Math.log10((w/5)/scale)));
  ctx.strokeStyle="#748079";ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(16,h-20);ctx.lineTo(16+bar*scale,h-20);ctx.stroke();ctx.fillStyle="#748079";ctx.fillText(`${bar} m`,16,h-29);
  ctx.textAlign="right";ctx.fillText(`x ${fmt(row.x_m)} m  /  y ${fmt(row.y_m)} m`,w-14,h-13);
}
function draw() {
  if(!run) return;
  const row=currentRow();$("scrub").value=Math.round(fraction*1000);$("time").textContent=`${fmt(row.time_s,2)} s`;
  drawPath(row);
  $("instant").replaceChildren();
  [["SPEED",`${fmt(row.speed_mps*3.6)} km/h`],["LEAN ESTIMATE",`${fmt(row.lean_estimate_deg)}°`],["PACK SOC",`${fmt(row.soc*100,2)}%`]].forEach(([label,value])=>{const item=el("div");item.append(el("span",label),el("b",value));$("instant").append(item);});
  $("energy").replaceChildren();
  [["Battery terminal",row.battery_terminal_power_w],["Motor shaft",row.motor_shaft_power_w],["At rear wheel",row.wheel_drive_power_w],["Battery heat",row.battery_loss_w],["Motor + inverter heat",row.motor_loss_w+row.inverter_loss_w],["Friction brake heat",row.friction_brake_loss_w]].forEach(([label,value],i)=>{const item=el("div",undefined,`energy-row${i===0?" main":""}`);item.append(el("span",label),el("b",fmt(value/1000,2)));$("energy").append(item);});
  $("charts").querySelectorAll("canvas").forEach((canvas,i)=>chart(canvas,views[$("view").value][i][1]));
  chart($("raw-chart"),[[$("signal").value,$("signal").value]]);
}
$("refresh").onclick=refresh;
$("view").onchange=()=>{buildCharts();draw();};
$("signal").onchange=draw;
$("raw-chart").onclick=event=>scrubChart(event,$("raw-chart"));
$("scrub").oninput=()=>{fraction=Number($("scrub").value)/1000;draw();};
$("compare").onchange=async()=>{const id=$("compare").value,version=selectionVersion;try{const data=id?await api(`/api/run?id=${encodeURIComponent(id)}`):null;if(version===selectionVersion&&$("compare").value===id){comparison=data;draw();}}catch(err){error(err.message);}};
$("play").onclick=()=>{playing=!playing;if(playing&&fraction>=1)fraction=0;lastFrame=0;$("play").textContent=playing?"Ⅱ Pause":"▶ Play";};
window.addEventListener("resize",draw);
function animate(now){if(playing&&run){if(lastFrame)fraction=Math.min(1,fraction+(now-lastFrame)/1000/Math.max(run.summary.duration_s,.001));lastFrame=now;draw();if(fraction>=1){playing=false;$("play").textContent="▶ Play";}}requestAnimationFrame(animate);}
requestAnimationFrame(animate);
refresh();
