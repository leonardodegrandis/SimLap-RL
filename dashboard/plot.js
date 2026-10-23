"use strict";

// Canvas renderer: no channel-specific knowledge or data resampling.
function channelUnit(key) {
  const suffixes = {mps2:"m/s²", radps2:"rad/s²", radps:"rad/s", mps:"m/s", rpm:"rpm", nm:"N·m",
    rad:"rad", deg:"°", ms:"ms", s:"s", m:"m", n:"N", a:"A", v:"V", w:"W", j:"J"};
  if (/^(command_)?(throttle|brake)$/.test(key) || key === "soc" || key.endsWith("_usage")) return "1";
  return suffixes[key.split("_").pop()] || "—";
}

function formatNumber(value) {
  if (!Number.isFinite(value)) return "—";
  if (value !== 0 && (Math.abs(value) < .001 || Math.abs(value) >= 100000)) return value.toExponential(3);
  return Number(value.toPrecision(6)).toString();
}

function axisTick(value, span) {
  const step=span/4;
  if (Math.abs(value)<step*1e-8) return "0";
  const precision=Math.max(0,Math.min(10,Math.ceil(-Math.log10(step))));
  if (Math.abs(value)>=1e5 || Math.abs(value)<1e-4) {
    const digits=Math.max(1,Math.min(10,Math.ceil(Math.log10(Math.abs(value)/step))+1));
    return value.toExponential(digits);
  }
  return value.toFixed(precision);
}

function nearestSample(rows, time) {
  if (!rows.length || time < rows[0].time_s || time > rows[rows.length-1].time_s) return null;
  let lo=0, hi=rows.length-1;
  while (lo<hi) {
    const mid=Math.floor((lo+hi)/2);
    if (rows[mid].time_s<time) lo=mid+1; else hi=mid;
  }
  return lo && time-rows[lo-1].time_s < rows[lo].time_s-time ? rows[lo-1] : rows[lo];
}

function drawTelemetryPlot(canvas, plot, datasets, range, cursor) {
  const rect=canvas.getBoundingClientRect(), dpr=window.devicePixelRatio || 1;
  canvas.width=Math.round(rect.width*dpr); canvas.height=Math.round(rect.height*dpr);
  const ctx=canvas.getContext("2d"); ctx.scale(dpr,dpr);
  ctx.font="10px Consolas, monospace";
  const series=plot.series.filter(s=>s.visible), hasRight=series.some(s=>s.axis==="right");
  const area={left:64, right:rect.width-(hasRight?68:18), top:27, bottom:rect.height-42};
  const samples=datasets.map(data=>data.rows.filter(row=>row.time_s>=range[0] && row.time_s<=range[1]));
  let xr=plot.x==="time_s" ? [...range] : [Infinity,-Infinity];
  const yr={left:[Infinity,-Infinity],right:[Infinity,-Infinity]};
  samples.forEach(rows=>rows.forEach(row=>{
    if (!Number.isFinite(row[plot.x])) return;
    if (plot.x!=="time_s") { xr[0]=Math.min(xr[0],row[plot.x]); xr[1]=Math.max(xr[1],row[plot.x]); }
    series.forEach(s=>{
      if (Number.isFinite(row[s.channel])) {
        yr[s.axis][0]=Math.min(yr[s.axis][0],row[s.channel]);
        yr[s.axis][1]=Math.max(yr[s.axis][1],row[s.channel]);
      }
    });
  }));
  const hasData=Object.values(yr).some(bounds=>Number.isFinite(bounds[0]));
  function padded(bounds) {
    if (!Number.isFinite(bounds[0])) return [0,1];
    const pad=Math.max((bounds[1]-bounds[0])*.06, Math.abs(bounds[0])*1e-4, 1e-9);
    return [bounds[0]-pad,bounds[1]+pad];
  }
  if (plot.x!=="time_s") xr=padded(xr);
  yr.left=padded(yr.left); yr.right=padded(yr.right);
  const units=axis=>[...new Set(series.filter(s=>s.axis===axis).map(s=>channelUnit(s.channel)))];
  const equal=plot.equal && !hasRight && units("left").length===1 && units("left")[0]===channelUnit(plot.x);
  if (equal) {
    const scale=Math.max((xr[1]-xr[0])/(area.right-area.left),(yr.left[1]-yr.left[0])/(area.bottom-area.top));
    const mx=(xr[0]+xr[1])/2, my=(yr.left[0]+yr.left[1])/2;
    xr=[mx-scale*(area.right-area.left)/2,mx+scale*(area.right-area.left)/2];
    yr.left=[my-scale*(area.bottom-area.top)/2,my+scale*(area.bottom-area.top)/2];
  }
  const x=value=>area.left+(value-xr[0])/(xr[1]-xr[0])*(area.right-area.left);
  const y=(value,axis)=>area.bottom-(value-yr[axis][0])/(yr[axis][1]-yr[axis][0])*(area.bottom-area.top);
  ctx.lineWidth=1; ctx.fillStyle="#4d5965";
  for (let i=0;i<=4;i++) {
    const fraction=i/4, px=area.left+fraction*(area.right-area.left), py=area.bottom-fraction*(area.bottom-area.top);
    ctx.strokeStyle="#dde2e8";ctx.beginPath();ctx.moveTo(area.left,py);ctx.lineTo(area.right,py);ctx.moveTo(px,area.top);ctx.lineTo(px,area.bottom);ctx.stroke();
    ctx.textAlign="right";ctx.fillText(axisTick(yr.left[0]+fraction*(yr.left[1]-yr.left[0]),yr.left[1]-yr.left[0]),area.left-7,py+3);
    if (hasRight) {ctx.textAlign="left";ctx.fillText(axisTick(yr.right[0]+fraction*(yr.right[1]-yr.right[0]),yr.right[1]-yr.right[0]),area.right+7,py+3);}
    ctx.textAlign="center";ctx.fillText(axisTick(xr[0]+fraction*(xr[1]-xr[0]),xr[1]-xr[0]),px,area.bottom+16);
  }
  ctx.textAlign="left";ctx.fillText(`L [${units("left").join(", ") || "—"}]`,area.left,14);
  if (hasRight) {ctx.textAlign="right";ctx.fillText(`R [${units("right").join(", ")}]`,area.right,14);}
  ctx.textAlign="center";ctx.fillText(`${plot.x} [${channelUnit(plot.x)}]`,(area.left+area.right)/2,rect.height-7);
  ctx.save();ctx.beginPath();ctx.rect(area.left,area.top,area.right-area.left,area.bottom-area.top);ctx.clip();
  datasets.forEach((data,index)=>series.forEach(s=>{
    ctx.strokeStyle=s.color;ctx.lineWidth=1.5;ctx.setLineDash(index?[6,4]:[]);ctx.beginPath();
    let started=false;
    samples[index].forEach(row=>{
      if (!Number.isFinite(row[plot.x]) || !Number.isFinite(row[s.channel])) {started=false;return;}
      const px=x(row[plot.x]),py=y(row[s.channel],s.axis);
      if (started) ctx.lineTo(px,py); else ctx.moveTo(px,py);
      started=true;
    });ctx.stroke();
    const row=nearestSample(data.rows,cursor);
    if (row && Number.isFinite(row[plot.x]) && Number.isFinite(row[s.channel])) {
      ctx.setLineDash([]);ctx.fillStyle=s.color;ctx.beginPath();ctx.arc(x(row[plot.x]),y(row[s.channel],s.axis),3,0,2*Math.PI);ctx.fill();
    }
  }));
  ctx.setLineDash([3,3]);ctx.strokeStyle="#536270";
  if (plot.x==="time_s") {ctx.beginPath();ctx.moveTo(x(cursor),area.top);ctx.lineTo(x(cursor),area.bottom);ctx.stroke();}
  ctx.restore();ctx.setLineDash([]);
  if (!hasData) {ctx.fillStyle="#596573";ctx.textAlign="center";ctx.fillText(series.length?"No numeric samples in this range":"Select channels from the left panel",(area.left+area.right)/2,(area.top+area.bottom)/2);}
  canvas.plotGeometry={area,xr,samples:samples[0] || []};
  return ["left","right"].filter(axis=>units(axis).length>1).map(axis=>`${axis}: mixed units share a numeric scale`).join("; ") ||
    (plot.equal&&!equal?"Equal scale requires matching X/Y units and no right-axis traces.":"");
}
