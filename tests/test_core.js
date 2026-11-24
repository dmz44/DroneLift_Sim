// Minimal regression tests for the simulation core (run: node tests/test_core.js)
const assert = require('assert');
const S = require('../docs/_static/dronelift/sim-core.js');

// 1. Nominal flight with no failure tracks the reference.
let p = S.defaultParams(); p.F = []; p.selector = 'hp';
let r = S.runScenario(p);
assert.ok(r.outcome.success, 'nominal flight should be safe');
assert.ok(r.outcome.maxErr < 0.1, 'nominal tracking error < 0.1 m');

// 2. No NaNs across team sizes and selectors.
for (let N = 3; N <= 7; N++) for (const sel of ['hp', 'sc', 'fs', 'simplex']) {
  p = S.defaultParams(); p.N = N; p.F = [0]; p.selector = sel;
  r = S.runScenario(p);
  for (const f of r.sim.frames) assert.ok(isFinite(f.L[0] + f.L[1] + f.L[2] + f.err), `NaN at N=${N} ${sel}`);
}

// 3. Reform yields a regular polygon.
const rf = S.reform([{ i: 0, psi: 0 }, { i: 1, psi: Math.PI / 2 }, { i: 2, psi: Math.PI }]);
const tg = [0, 1, 2].map(i => rf.targets[i].to).sort((a, b) => a - b);
assert.ok(Math.abs((tg[1] - tg[0]) - 2 * Math.PI / 3) < 1e-9 && Math.abs((tg[2] - tg[1]) - 2 * Math.PI / 3) < 1e-9, 'reform spacing');

// 4. Determinism: the same scenario gives the same outcome.
const a = S.compareSystems(S.sampleScenario(7), { fast: true }), b = S.compareSystems(S.sampleScenario(7), { fast: true });
assert.deepStrictEqual(JSON.stringify(a.simplex), JSON.stringify(b.simplex));

// 5. Dihedral strata of 2 failures in a hexagon: adjacent, skip-one, opposite.
assert.strictEqual(S.strata(6, 2).length, 3);

// 6. Load ratio round-trip.
p = S.defaultParams(); p.rho = 0.55;
assert.ok(Math.abs(S.capacity(p).rho - 0.55) < 1e-12);
console.log('all core tests passed');
