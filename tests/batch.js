// Command-line Monte Carlo batch, identical to the site's batch with first seed 1
// and no boxes ticked except "use the SC parameters and λ above" (defaults).
// Usage: node tests/batch.js [K]
const S = require('../docs/_static/dronelift/sim-core.js');
const K = +process.argv[2] || 60;
const sys = ['hp', 'sc', 'fs', 'simplex'];
const agg = Object.fromEntries(sys.map(s => [s, { viol: 0, kept: 0 }]));
const t0 = Date.now(); let inScope = 0;
for (let k = 1; k <= K; k++) {
  const r = S.compareSystems(S.sampleScenario(k), { fast: true });
  if (r.cap.inScope) inScope++;
  for (const s of sys) { if (!r[s].success) agg[s].viol++; else if (r[s].finalMode !== 'FS') agg[s].kept++; }
}
console.log(`K=${K}, in scope ${inScope}, ${((Date.now() - t0) / 1000).toFixed(1)} s`);
for (const s of sys) console.log(s.padEnd(8), 'violation', (100 * agg[s].viol / K).toFixed(1) + '%', ' mission kept', (100 * agg[s].kept / K).toFixed(1) + '%');
