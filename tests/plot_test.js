// Run with a JavaScript shell: jsc dashboard/plot.js tests/plot_test.js
function assert(condition, message) {if (!condition) throw new Error(message);}
for (const [start, span] of [[14.9985,.003], [131.43,.04], [1e-7,1e-10], [1e8,4]]) {
  const labels=Array.from({length:5},(_,i)=>axisTick(start+i*span/4,span));
  assert(new Set(labels).size===5, `Axis labels must remain distinct: ${labels}`);
}
const rows=[{time_s:0,value:1},{time_s:.5,value:2},{time_s:1,value:3}];
assert(nearestSample(rows,.6).value===2,"Cursor must use nearest sample");
assert(nearestSample(rows,2)===null,"Do not extrapolate cursor values");
assert(nearestSample([],0)===null,"Empty channels must be supported");
assert(channelUnit("battery_current_a")==="A","Current unit");
assert(channelUnit("custom_unknown")==="—","Do not invent unknown units");
print("Plot numeric checks passed");
