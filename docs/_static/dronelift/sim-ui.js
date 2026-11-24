/*
 * DroneLift browser simulator — user interface.
 * Builds the controls, 3D viewer, plots, comparison and batch panels into
 * #dronelift-app and #dronelift-batch. Depends on sim-core.js.
 */
(function () {
  'use strict';
  var S = window.DroneLiftSim, C = S.CONST, DEG = Math.PI / 180;
  // categorical palette, slot i = drone i (fixed by entity, never by rank)
  var COLORS = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300', '#6250d6'];
  var MODE_FILL = { HP: '#e6eef8', SC: '#fbecd0', FS: '#d9d9de' };
  var INK = '#404040', INK2 = '#6b6b6b', GRID = '#e4e4e4';

  /* ---------------------------------------------------------------- */
  /* DOM helpers                                                       */
  /* ---------------------------------------------------------------- */
  function el(tag, attrs, kids) {
    var e = document.createElement(tag);
    if (attrs) for (var k in attrs) {
      if (k === 'class') e.className = attrs[k];
      else if (k === 'html') e.innerHTML = attrs[k];
      else if (k === 'text') e.textContent = attrs[k];
      else if (k.slice(0, 2) === 'on') e.addEventListener(k.slice(2), attrs[k]);
      else e.setAttribute(k, attrs[k]);
    }
    (kids || []).forEach(function (c) { if (c) e.appendChild(typeof c === 'string' ? document.createTextNode(c) : c); });
    return e;
  }
  function fmt(x, d) { return (x === null || x === undefined || !isFinite(x)) ? '—' : x.toFixed(d === undefined ? 2 : d); }
  function fmtBudget(b) { return b < 0 ? '< ' + S.BUDGET_GRID[0].toFixed(2) + ' s' : (b >= S.BUDGET_GRID[S.BUDGET_GRID.length - 1] ? '≥ ' + b.toFixed(2) + ' s' : b.toFixed(2) + ' s'); }

  /* ---------------------------------------------------------------- */
  /* State                                                             */
  /* ---------------------------------------------------------------- */
  var st = {
    p: S.defaultParams(), res: null, stale: true,
    playing: false, speed: 1, ti: 0, tAcc: 0,
    cam: { yaw: -2.2, pitch: 0.38, dist: 2.3 },
    refPath: null, plots: [], batch: null
  };
  var ui = {};

  var SYS_NAMES = { hp: 'High-level only (HP)', sc: 'Always SC', fs: 'Always FS', simplex: 'Simplex (oracle budgets)' };

  /* Field specs ------------------------------------------------------ */
  var GROUPS = [
    { title: 'Team and failure', fields: [
      { k: 'N', label: 'Team size N', type: 'range', min: 3, max: 7, step: 1, f: 0 },
      { k: 'F', label: 'Failure pattern F (click drones)', type: 'picker' },
      { k: 'psi0_deg', label: 'Formation yaw ψ₀', type: 'range', min: 0, max: 359, step: 1, unit: '°', f: 0 }
    ] },
    { title: 'Payload configuration', fields: [
      { k: 'loadMode', label: 'Specify load by', type: 'select', options: [['ratio', 'load ratio ρ_L'], ['mass', 'payload mass m_L']] },
      { k: 'rho', label: 'Load ratio ρ_L', type: 'range', min: 0.2, max: 1.0, step: 0.05, f: 2, show: function (p) { return p.loadMode === 'ratio'; } },
      { k: 'm_L', label: 'Payload mass m_L', type: 'range', min: 0.010, max: 0.150, step: 0.001, unit: ' kg', f: 3, show: function (p) { return p.loadMode === 'mass'; } },
      { k: 'ell', label: 'Cable length ℓ', type: 'range', min: 0.3, max: 1.0, step: 0.05, unit: ' m', f: 2 },
      { k: 'theta0_deg', label: 'Nominal cable angle θ₀', type: 'range', min: 15, max: 50, step: 1, unit: '°', f: 0 }
    ] },
    { title: 'Mission and disturbance', fields: [
      { k: 'phase', label: 'Flight phase at failure', type: 'select', options: [['cruise', 'cruise'], ['accel', 'acceleration'], ['turn', 'turn']] },
      { k: 't_f', label: 'Failure time t_f', type: 'range', min: 1.6, max: 5.0, step: 0.05, unit: ' s', f: 2 },
      { k: 'v0', label: 'Cruise speed v₀', type: 'range', min: 0.2, max: 0.8, step: 0.01, unit: ' m/s', f: 2 },
      { k: 'a0', label: 'Acceleration a₀', type: 'range', min: 0.2, max: 0.8, step: 0.01, unit: ' m/s²', f: 2, show: function (p) { return p.phase === 'accel'; } },
      { k: 'r_turn', label: 'Turn radius', type: 'range', min: 1.0, max: 3.0, step: 0.05, unit: ' m', f: 2, show: function (p) { return p.phase === 'turn'; } },
      { k: 'turn_dir', label: 'Turn direction', type: 'select', options: [['1', 'left (+1)'], ['-1', 'right (−1)']], num: true, show: function (p) { return p.phase === 'turn'; } },
      { k: 'z0', label: 'Payload altitude z₀', type: 'range', min: 0.8, max: 2.0, step: 0.05, unit: ' m', f: 2 },
      { k: 'wind', label: 'Gusts (mean 0.5 m/s, σ 0.5 m/s, 1 s corr.)', type: 'check' },
      { k: 'seed', label: 'Random seed', type: 'number', min: 1, max: 99999 }
    ] },
    { title: 'Switching runtime', fields: [
      { k: 'selector', label: 'System', type: 'select', options: [['simplex', SYS_NAMES.simplex], ['hp', SYS_NAMES.hp], ['sc', SYS_NAMES.sc], ['fs', SYS_NAMES.fs]] },
      { k: 's_lat', label: 'Latency scale s_Λ (Δ = 80 ms × s_Λ × j)', type: 'select', options: [['0.5', '0.5'], ['1', '1'], ['2', '2'], ['4', '4'], ['8', '8'], ['16', '16']], num: true },
      { k: 'jitter', label: 'Latency jitter j', type: 'range', min: 0.6, max: 1.6, step: 0.01, f: 2 },
      { k: 'lambda', label: 'Decision margin λ', type: 'range', min: 0, max: 0.5, step: 0.01, unit: ' s', f: 2, show: function (p) { return p.selector === 'simplex'; } }
    ] },
    { title: 'Safety-controller parameters (stand-in for ML-2)', fields: [
      { k: 'Tb', label: 'Reform / rejoin time T_b', type: 'range', min: 1.0, max: 4.0, step: 0.1, unit: ' s', f: 1 },
      { k: 'theta_sc_deg', label: 'SC cable angle θ_SC', type: 'range', min: 15, max: 50, step: 1, unit: '°', f: 0 },
      { k: 'k_s', label: 'Swing feedback k_s', type: 'range', min: 0, max: 1.5, step: 0.05, f: 2 },
      { k: 'sc_stop', label: 'Smooth stop T_stop (0 = reference frozen)', type: 'range', min: 0, max: 3, step: 0.1, unit: ' s', f: 1 }
    ] }
  ];

  /* ---------------------------------------------------------------- */
  /* URL hash <-> params (shareable scenario links)                    */
  /* ---------------------------------------------------------------- */
  function toHash(p) {
    var d = S.defaultParams(), q = [];
    for (var k in d) {
      var v = p[k];
      if (k === 'F') { q.push('F=' + v.map(function (i) { return i + 1; }).join('.')); continue; }
      q.push(k + '=' + (typeof v === 'number' ? +v.toFixed(4) : v));
    }
    return '#' + q.join('&');
  }
  function fromHash(h) {
    var p = S.defaultParams(), d = S.defaultParams();
    if (!h || h.length < 2) return p;
    h.slice(1).split('&').forEach(function (kv) {
      var i = kv.indexOf('='); if (i < 0) return;
      var k = decodeURIComponent(kv.slice(0, i)), v = decodeURIComponent(kv.slice(i + 1));
      if (!(k in d)) return;
      if (k === 'F') { p.F = v ? v.split('.').map(function (x) { return parseInt(x, 10) - 1; }).filter(function (x) { return x >= 0; }) : []; return; }
      if (typeof d[k] === 'number') { var n = parseFloat(v); if (isFinite(n)) p[k] = n; }
      else if (typeof d[k] === 'boolean') p[k] = v === 'true' || v === '1';
      else p[k] = v;
    });
    p.N = Math.max(3, Math.min(7, Math.round(p.N)));
    p.F = p.F.filter(function (i) { return i < p.N; });
    return p;
  }

  /* ---------------------------------------------------------------- */
  /* Controls                                                          */
  /* ---------------------------------------------------------------- */
  function buildControls(root) {
    var wrap = el('div', { class: 'dl-groups' });
    ui.fields = {};
    GROUPS.forEach(function (g) {
      var fs = el('fieldset', { class: 'dl-group' }, [el('legend', { text: g.title })]);
      g.fields.forEach(function (f) {
        var row = el('div', { class: 'dl-row' });
        var lab = el('label', { class: 'dl-label' }, [f.label]);
        var val = el('span', { class: 'dl-val' });
        var input;
        if (f.type === 'picker') {
          input = el('div', { class: 'dl-picker' });
          row.appendChild(el('div', { class: 'dl-label' }, [f.label]));
          row.appendChild(input);
          ui.picker = input;
          ui.pickerNote = el('div', { class: 'dl-note' });
          row.appendChild(ui.pickerNote);
        } else {
          if (f.type === 'range') input = el('input', { type: 'range', min: f.min, max: f.max, step: f.step });
          else if (f.type === 'number') input = el('input', { type: 'number', min: f.min, max: f.max, step: 1, class: 'dl-num' });
          else if (f.type === 'check') input = el('input', { type: 'checkbox' });
          else {
            input = el('select');
            f.options.forEach(function (o) { input.appendChild(el('option', { value: o[0], text: o[1] })); });
          }
          var id = 'dl-' + f.k; input.id = id; lab.setAttribute('for', id);
          input.addEventListener(f.type === 'range' ? 'input' : 'change', function () {
            var v;
            if (f.type === 'check') v = input.checked;
            else if (f.type === 'range' || f.type === 'number' || f.num) v = parseFloat(input.value);
            else v = input.value;
            st.p[f.k] = v;
            if (f.k === 'N') st.p.F = st.p.F.filter(function (i) { return i < v; });
            if (f.k === 'phase') {
              var w = { cruise: [2.0, 3.0], accel: [3.1, 3.6], turn: [3.4, 4.4] }[v];
              if (st.p.t_f < w[0] || st.p.t_f > w[1]) st.p.t_f = +(0.5 * (w[0] + w[1])).toFixed(2);
            }
            changed();
          });
          if (f.type === 'check') { row.className += ' dl-row-check'; row.appendChild(input); row.appendChild(lab); }
          else { row.appendChild(lab); row.appendChild(val); row.appendChild(input); }
        }
        ui.fields[f.k] = { spec: f, input: input, val: val, row: row };
        fs.appendChild(row);
      });
      wrap.appendChild(fs);
    });
    root.appendChild(wrap);
  }

  function syncControls() {
    var p = st.p;
    for (var k in ui.fields) {
      var F = ui.fields[k], f = F.spec;
      if (f.type === 'picker') continue;
      if (f.type === 'check') F.input.checked = !!p[k];
      else F.input.value = String(p[k]);
      if (f.type === 'range') F.val.textContent = (+p[k]).toFixed(f.f) + (f.unit || '');
      F.row.style.display = f.show && !f.show(p) ? 'none' : '';
    }
    var win = { cruise: '2.0–3.0 s', accel: '3.1–3.6 s', turn: '3.4–4.4 s' }[p.phase];
    ui.fields.t_f.val.textContent = (+p.t_f).toFixed(2) + ' s (paper window ' + win + ')';
    drawPicker();
  }

  function drawPicker() {
    var p = st.p, N = p.N, box = ui.picker;
    var W = 150, R = 52, c = W / 2;
    var ns = 'http://www.w3.org/2000/svg';
    var svg = document.createElementNS(ns, 'svg');
    svg.setAttribute('viewBox', '0 0 ' + W + ' ' + W); svg.setAttribute('width', W); svg.setAttribute('height', W);
    svg.setAttribute('role', 'group'); svg.setAttribute('aria-label', 'Failure pattern picker');
    var poly = [];
    for (var i = 0; i < N; i++) {
      var a = p.psi0_deg * DEG + 2 * Math.PI * i / N;
      poly.push([c + R * Math.cos(a), c - R * Math.sin(a)]);
    }
    var pl = document.createElementNS(ns, 'polygon');
    pl.setAttribute('points', poly.map(function (q) { return q.join(','); }).join(' '));
    pl.setAttribute('fill', 'none'); pl.setAttribute('stroke', GRID); pl.setAttribute('stroke-width', '1.5');
    svg.appendChild(pl);
    var pay = document.createElementNS(ns, 'circle');
    pay.setAttribute('cx', c); pay.setAttribute('cy', c); pay.setAttribute('r', 7); pay.setAttribute('fill', '#52514e');
    svg.appendChild(pay);
    for (i = 0; i < N; i++) {
      var failed = p.F.indexOf(i) >= 0;
      var cable = document.createElementNS(ns, 'line');
      cable.setAttribute('x1', c); cable.setAttribute('y1', c); cable.setAttribute('x2', poly[i][0]); cable.setAttribute('y2', poly[i][1]);
      cable.setAttribute('stroke', failed ? '#c9c9c9' : '#8a8a8a'); cable.setAttribute('stroke-width', '1');
      if (failed) cable.setAttribute('stroke-dasharray', '3 3');
      svg.insertBefore(cable, pay);
      var g = document.createElementNS(ns, 'g');
      g.setAttribute('class', 'dl-pick' + (failed ? ' is-failed' : ''));
      g.setAttribute('tabindex', '0'); g.setAttribute('role', 'button');
      g.setAttribute('aria-pressed', failed ? 'true' : 'false');
      g.setAttribute('aria-label', 'Drone ' + (i + 1) + (failed ? ' (fails)' : ' (healthy)'));
      var hit = document.createElementNS(ns, 'circle');
      hit.setAttribute('cx', poly[i][0]); hit.setAttribute('cy', poly[i][1]); hit.setAttribute('r', 15);
      hit.setAttribute('fill', failed ? '#ffffff' : COLORS[i]); hit.setAttribute('stroke', failed ? '#c0392b' : '#ffffff'); hit.setAttribute('stroke-width', '2');
      g.appendChild(hit);
      var tx = document.createElementNS(ns, 'text');
      tx.setAttribute('x', poly[i][0]); tx.setAttribute('y', poly[i][1] + 4); tx.setAttribute('text-anchor', 'middle');
      tx.setAttribute('font-size', '12'); tx.setAttribute('font-weight', '700'); tx.setAttribute('fill', failed ? '#c0392b' : '#ffffff');
      tx.textContent = failed ? '✕' + (i + 1) : String(i + 1);
      g.appendChild(tx);
      (function (idx) {
        function toggle() {
          var j = st.p.F.indexOf(idx);
          if (j >= 0) st.p.F.splice(j, 1); else st.p.F.push(idx);
          st.p.F.sort(function (a, b) { return a - b; });
          changed();
        }
        g.addEventListener('click', toggle);
        g.addEventListener('keydown', function (e) { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); toggle(); } });
      })(i);
      svg.appendChild(g);
    }
    box.innerHTML = ''; box.appendChild(svg);
    var n = N - p.F.length;
    var msg = p.F.length ? 'Fails: ' + p.F.map(function (i) { return i + 1; }).join(', ') + ' → ' + n + ' survivor' + (n === 1 ? '' : 's') : 'No failure selected (nominal flight).';
    if (p.F.length > N - 2) msg += ' Outside the paper’s range |F| ≤ N−2.';
    ui.pickerNote.textContent = msg;
  }

  function changed() {
    st.stale = true;
    syncControls();
    renderCapacity();
    ui.staleNote.style.visibility = st.res ? 'visible' : 'hidden';
    try { history.replaceState(null, '', toHash(st.p)); } catch (e) { /* file:// or sandbox */ }
  }

  /* ---------------------------------------------------------------- */
  /* Capacity panel                                                    */
  /* ---------------------------------------------------------------- */
  function renderCapacity() {
    var c = S.capacity(st.p), p = st.p;
    function ok(b) { return b ? '<span class="dl-ok">✓ yes</span>' : '<span class="dl-bad">✕ no</span>'; }
    ui.cap.innerHTML =
      '<table class="dl-table"><tbody>' +
      '<tr><th>Payload mass m<sub>L</sub></th><td>' + fmt(c.m_L * 1000, 1) + ' g</td><th>Load ratio before / after failure</th><td>' + fmt(c.rho, 2) + ' / ' + fmt(c.rho_after, 2) + '</td></tr>' +
      '<tr><th>Hover thrust per member, N = ' + p.N + '</th><td>' + fmt(c.f_hover_N, 3) + ' N</td><th>Hover thrust per member, n = ' + c.n + '</th><td>' + (c.n > 0 ? fmt(c.f_hover_n, 3) + ' N' : '—') + '</td></tr>' +
      '<tr><th>Safety factor σ<sub>N</sub> = f̄ / f*<sub>N</sub></th><td>' + fmt(c.sigma_N, 2) + '</td><th>Safety factor σ<sub>n</sub> after failure</th><td>' + fmt(c.sigma_n, 2) + '</td></tr>' +
      '<tr><th>Descent capacity Cap<sub>D</sub></th><td>' + ok(c.capD) + '</td><th>Recovery capacity Cap<sub>R</sub> (10% margin)</th><td>' + ok(c.capR) + '</td></tr>' +
      '<tr><th>Min. survivors n<sub>D</sub> / n<sub>R</sub></th><td>' + c.n_D + ' / ' + c.n_R + '</td><th>Scenario in scope</th><td>' + ok(c.inScope) + '</td></tr>' +
      '</tbody></table>' +
      '<p class="dl-note">f̄ = ' + C.f_bar + ' N usable thrust, m<sub>d</sub> = ' + (C.m_d * 1000) + ' g. σ<sub>n</sub> ≥ 1 is necessary but not sufficient: it certifies the end state, not the transient while the team switches.</p>';
  }

  /* ---------------------------------------------------------------- */
  /* Running                                                           */
  /* ---------------------------------------------------------------- */
  function busy(on, text) {
    ui.busy.textContent = on ? text : '';
    ui.runBtn.disabled = on; ui.cmpBtn.disabled = on;
  }

  function runSingle() {
    busy(true, st.p.selector === 'simplex' ? 'Simulating (sweeping switching budgets)…' : 'Simulating…');
    setTimeout(function () {
      var t0 = performance.now();
      var r = S.runScenario(st.p, { alwaysBudgets: true });
      r.ms = performance.now() - t0;
      st.res = r; st.stale = false; ui.staleNote.style.visibility = 'hidden';
      try { history.replaceState(null, '', toHash(st.p)); } catch (e) { /* ignore */ }
      // reference path for drawing
      var pts = [], ref = r.sim.ref, o = { p: [0, 0, 0], v: [0, 0, 0], a: [0, 0, 0] };
      for (var tau = 0; tau <= r.sim.t_end; tau += 0.05) { ref.at(tau, o); pts.push(o.p.slice()); }
      st.refPath = pts;
      st.ti = 0; st.tAcc = 0;
      var fi = r.sim.frames.findIndex(function (f) { return f.t >= st.p.t_f - 1.0; });
      st.ti = Math.max(fi, 0);
      ui.slider.max = r.sim.frames.length - 1;
      renderResult(); buildPlots(); drawAll();
      busy(false);
      setPlaying(true);
    }, 30);
  }

  function runCompare() {
    busy(true, 'Running all four systems on this scenario…');
    setTimeout(function () {
      var r = S.compareSystems(st.p);
      var rows = ['hp', 'sc', 'fs', 'simplex'].map(function (k) {
        var o = r[k];
        return '<tr><td>' + SYS_NAMES[k] + '</td><td>' + (o.success ? '<span class="dl-ok">✓ safe</span>' : '<span class="dl-bad">✕ violation</span>') +
          '</td><td>' + o.finalMode + '</td><td>' + fmt(o.maxErr, 2) + ' m</td><td>' + fmt(o.minSep, 2) + ' m</td><td class="dl-why">' + (o.reason || (o.finalMode === 'FS' ? 'landed (mission abandoned safely)' : 'mission continued')) + '</td></tr>';
      }).join('');
      ui.cmp.innerHTML = '<table class="dl-table dl-cmp"><thead><tr><th>System</th><th>Outcome</th><th>Final mode</th><th>Max tracking error</th><th>Min separation</th><th>Detail</th></tr></thead><tbody>' + rows + '</tbody></table>' +
        '<p class="dl-note">Switching budgets on this scenario: τ<sub>HP</sub> ' + fmtBudget(r.budgets.HP) + ', τ<sub>SC</sub> ' + fmtBudget(r.budgets.SC) + ', τ<sub>FS</sub> ' + fmtBudget(r.budgets.FS) +
        '. Switching delay Δ = ' + fmt(r.simplex.Delta * 1000, 0) + ' ms. All four systems share the pre-failure trajectory and the post-failure random numbers.</p>';
      busy(false);
    }, 30);
  }

  function renderResult() {
    var r = st.res, o = r.outcome, b = r.budgets, sim = r.sim;
    var badge = o.success ? '<span class="dl-badge dl-badge-ok">✓ Safe run</span>' : '<span class="dl-badge dl-badge-bad">✕ Violation</span>';
    var sweep = '';
    if (b) {
      sweep = '<table class="dl-table dl-sweep"><thead><tr><th>Mode \\ delay [s]</th>' + S.BUDGET_GRID.map(function (d) { return '<th>' + d + '</th>'; }).join('') + '<th>budget τ</th></tr></thead><tbody>' +
        ['HP', 'SC', 'FS'].map(function (m) {
          var row = b.sweep[m] || [];
          return '<tr><th>' + m + '</th>' + S.BUDGET_GRID.map(function (d, i) {
            var v = row[i];
            var mark = v === undefined ? '' : (v ? '✓' : '✕');
            var cls = v === undefined ? 'dl-na' : (v ? 'dl-ok' : 'dl-bad');
            var here = (sim.Delta > (i ? S.BUDGET_GRID[i - 1] : 0) && sim.Delta <= d) ? ' dl-delta' : '';
            return '<td class="' + cls + here + '">' + mark + '</td>';
          }).join('') + '<td>' + fmtBudget(b[m]) + '</td></tr>';
        }).join('') + '</tbody></table>' +
        '<p class="dl-note">Each cell re-runs the scenario from the pre-failure state with the given switching delay (stale HP until the switch, no watchdogs). τ is the largest delay before the first failure. The outlined column brackets this run’s Δ. ' +
        (st.p.selector === 'simplex' ? 'Rule: HP if τ<sub>HP</sub> ≥ Δ + λ, else SC if τ<sub>SC</sub> ≥ Δ + λ, else FS.' : 'The selected system ignores these budgets; they are shown for reference.') + '</p>';
    }
    ui.result.innerHTML =
      '<div class="dl-result-head">' + badge + '<span>' + (o.reason ? o.reason : (o.finalMode === 'FS' ? 'Payload landed under the failsafe (mission abandoned safely).' : 'All constraints held and the final mode kept its promise.')) + '</span></div>' +
      '<table class="dl-table"><tbody>' +
      '<tr><th>Switching delay Δ</th><td>' + fmt(o.Delta * 1000, 0) + ' ms</td><th>Initial decision → final mode</th><td>' + (o.decision || '—') + ' → ' + o.finalMode + '</td></tr>' +
      '<tr><th>Max tracking error</th><td>' + fmt(o.maxErr, 3) + ' m</td><th>Max reference delay</th><td>' + fmt(o.maxDelay, 2) + ' s</td></tr>' +
      '<tr><th>Min separation (S1)</th><td>' + fmt(o.minSep, 3) + ' m</td><th>Min drone height (S2)</th><td>' + fmt(o.minH, 3) + ' m</td></tr>' +
      '<tr><th>Payload touchdown</th><td>' + (o.t_td !== null ? 't_f + ' + fmt(o.t_td - st.p.t_f, 2) + ' s at ' + fmt(o.v_td, 2) + ' m/s' : 'none') + '</td><th>Compute time</th><td>' + fmt(r.ms, 0) + ' ms</td></tr>' +
      '</tbody></table>' + sweep +
      '<details class="dl-events"><summary>Event log (' + sim.events.length + ')</summary><ol>' +
      sim.events.map(function (e) { return '<li><code>t_f ' + ((e.t - st.p.t_f) >= 0 ? '+' : '−') + fmt(Math.abs(e.t - st.p.t_f), 3) + ' s</code> <b>' + e.kind + '</b> ' + e.text + '</li>'; }).join('') + '</ol></details>';
  }

  /* ---------------------------------------------------------------- */
  /* 3D viewer                                                         */
  /* ---------------------------------------------------------------- */
  function buildViewer(root) {
    var box = el('div', { class: 'dl-viewer' });
    ui.canvas = el('canvas', { class: 'dl-canvas', 'aria-label': '3D view of the team and payload; drag to orbit, scroll to zoom' });
    ui.hud = el('div', { class: 'dl-hud' });
    var legend = el('div', { class: 'dl-legend3d', html:
      '<span><i class="dl-sw" style="background:#2a78d6"></i>member (colored by drone)</span>' +
      '<span><i class="dl-sw dl-sw-x">✕</i>failed, cable attached</span>' +
      '<span><i class="dl-sw" style="background:#c8c8c8"></i>released</span>' +
      '<span><i class="dl-sw" style="background:#52514e;border-radius:50%"></i>payload</span>' +
      '<span><i class="dl-sw dl-sw-ring"></i>payload reference</span>' });
    box.appendChild(ui.canvas); box.appendChild(ui.hud); box.appendChild(legend);
    var ctl = el('div', { class: 'dl-play' });
    ui.playBtn = el('button', { class: 'dl-btn dl-btn-sm', text: '▶ Play', onclick: function () { setPlaying(!st.playing); } });
    var toFail = el('button', { class: 'dl-btn dl-btn-sm', text: '⤒ 1 s before failure', onclick: function () {
      if (!st.res) return;
      var fi = st.res.sim.frames.findIndex(function (f) { return f.t >= st.p.t_f - 1.0; });
      st.ti = Math.max(fi, 0); drawAll();
    } });
    ui.slider = el('input', { type: 'range', min: 0, max: 0, step: 1, value: 0, class: 'dl-scrub', 'aria-label': 'Playback time' });
    ui.slider.addEventListener('input', function () { st.ti = +ui.slider.value; setPlaying(false); drawAll(); });
    ui.speedSel = el('select', { 'aria-label': 'Playback speed' });
    [['0.25', '0.25×'], ['0.5', '0.5×'], ['1', '1×'], ['2', '2×']].forEach(function (o) { ui.speedSel.appendChild(el('option', { value: o[0], text: o[1] })); });
    ui.speedSel.value = '1';
    ui.speedSel.addEventListener('change', function () { st.speed = +ui.speedSel.value; });
    ui.timeLbl = el('span', { class: 'dl-time' });
    ctl.appendChild(ui.playBtn); ctl.appendChild(toFail); ctl.appendChild(ui.slider); ctl.appendChild(ui.speedSel); ctl.appendChild(ui.timeLbl);
    root.appendChild(box); root.appendChild(ctl);

    // orbit interaction
    var drag = null;
    function down(x, y) { drag = { x: x, y: y, yaw: st.cam.yaw, pitch: st.cam.pitch }; }
    function move(x, y) {
      if (!drag) return;
      st.cam.yaw = drag.yaw - (x - drag.x) * 0.008;
      st.cam.pitch = Math.max(0.05, Math.min(1.45, drag.pitch + (y - drag.y) * 0.006));
      draw3D();
    }
    ui.canvas.addEventListener('mousedown', function (e) { down(e.clientX, e.clientY); });
    window.addEventListener('mousemove', function (e) { move(e.clientX, e.clientY); });
    window.addEventListener('mouseup', function () { drag = null; });
    ui.canvas.addEventListener('touchstart', function (e) { var t = e.touches[0]; down(t.clientX, t.clientY); }, { passive: true });
    ui.canvas.addEventListener('touchmove', function (e) { var t = e.touches[0]; move(t.clientX, t.clientY); e.preventDefault(); }, { passive: false });
    ui.canvas.addEventListener('touchend', function () { drag = null; });
    ui.canvas.addEventListener('wheel', function (e) {
      e.preventDefault();
      st.cam.dist = Math.max(1.2, Math.min(9, st.cam.dist * Math.exp(e.deltaY * 0.001)));
      draw3D();
    }, { passive: false });
  }

  function fitCanvas(cv, aspect) {
    var dpr = window.devicePixelRatio || 1;
    var w = cv.clientWidth || 600, h = aspect ? Math.round(w * aspect) : (cv.clientHeight || 200);
    if (aspect) cv.style.height = h + 'px';
    if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); }
    var ctx = cv.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    return { ctx: ctx, w: w, h: h };
  }

  function draw3D() {
    var cv = ui.canvas, F = fitCanvas(cv, 0.6), ctx = F.ctx, W = F.w, H = F.h;
    ctx.fillStyle = '#fbfbfa'; ctx.fillRect(0, 0, W, H);
    if (!st.res) {
      ctx.fillStyle = INK2; ctx.font = '14px sans-serif'; ctx.textAlign = 'center';
      ctx.fillText('Press “Run simulation” to simulate this scenario.', W / 2, H / 2);
      ui.hud.textContent = ''; ui.timeLbl.textContent = '';
      return;
    }
    var sim = st.res.sim, frames = sim.frames, fr = frames[Math.min(st.ti, frames.length - 1)];
    var N = st.p.N;
    var T = [fr.L[0], fr.L[1], fr.L[2] + 0.3];
    var cam = st.cam;
    var eye = [T[0] + cam.dist * Math.cos(cam.pitch) * Math.cos(cam.yaw), T[1] + cam.dist * Math.cos(cam.pitch) * Math.sin(cam.yaw), T[2] + cam.dist * Math.sin(cam.pitch)];
    var fw = [T[0] - eye[0], T[1] - eye[1], T[2] - eye[2]], fl = Math.hypot(fw[0], fw[1], fw[2]);
    fw = [fw[0] / fl, fw[1] / fl, fw[2] / fl];
    var rt = [fw[1], -fw[0], 0], rl = Math.hypot(rt[0], rt[1]); rt = [rt[0] / rl, rt[1] / rl, 0];
    var up = [rt[1] * fw[2] - rt[2] * fw[1], rt[2] * fw[0] - rt[0] * fw[2], rt[0] * fw[1] - rt[1] * fw[0]];
    var foc = H / (2 * Math.tan(22.5 * DEG));
    function P(x, y, z) {
      var d0 = x - eye[0], d1 = y - eye[1], d2 = z - eye[2];
      var zc = d0 * fw[0] + d1 * fw[1] + d2 * fw[2];
      if (zc < 0.05) zc = 0.05;
      return { x: W / 2 + foc * (d0 * rt[0] + d1 * rt[1] + d2 * rt[2]) / zc, y: H / 2 - foc * (d0 * up[0] + d1 * up[1] + d2 * up[2]) / zc, z: zc, s: foc / zc };
    }
    // ground grid
    ctx.lineWidth = 1; ctx.strokeStyle = GRID;
    var gx = Math.round(T[0] / 0.5) * 0.5, gy = Math.round(T[1] / 0.5) * 0.5, ext = 3;
    for (var k = -ext; k <= ext + 1e-9; k += 0.5) {
      var a = P(gx + k, gy - ext, 0), b = P(gx + k, gy + ext, 0);
      ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
      a = P(gx - ext, gy + k, 0); b = P(gx + ext, gy + k, 0);
      ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
    }
    // reference path (ground trace and at altitude)
    if (st.refPath) {
      ctx.setLineDash([5, 5]); ctx.strokeStyle = '#b5b5b5'; ctx.lineWidth = 1.2;
      ctx.beginPath();
      st.refPath.forEach(function (q, i) { var s = P(q[0], q[1], q[2]); if (i) ctx.lineTo(s.x, s.y); else ctx.moveTo(s.x, s.y); });
      ctx.stroke(); ctx.setLineDash([]);
    }
    // payload trail (last 3 s)
    ctx.strokeStyle = 'rgba(82,81,78,0.45)'; ctx.lineWidth = 1.5; ctx.beginPath();
    for (var i = Math.max(0, st.ti - 300), first = true; i <= st.ti; i++) {
      var q = frames[i].L, s = P(q[0], q[1], q[2]);
      if (first) { ctx.moveTo(s.x, s.y); first = false; } else ctx.lineTo(s.x, s.y);
    }
    ctx.stroke();
    // shadows
    ctx.fillStyle = 'rgba(0,0,0,0.10)';
    function shadow(x, y, r) { var s = P(x, y, 0); ctx.beginPath(); ctx.ellipse(s.x, s.y, r * s.s, r * s.s * 0.45, 0, 0, 2 * Math.PI); ctx.fill(); }
    for (i = 0; i < N; i++) shadow(fr.d[3 * i], fr.d[3 * i + 1], 0.05);
    shadow(fr.L[0], fr.L[1], 0.04);
    // reference marker
    var rp = P(fr.R[0], fr.R[1], fr.R[2]);
    ctx.strokeStyle = INK2; ctx.lineWidth = 1.2; ctx.setLineDash([3, 3]);
    ctx.beginPath(); ctx.arc(rp.x, rp.y, Math.max(6, 0.05 * rp.s), 0, 2 * Math.PI); ctx.stroke(); ctx.setLineDash([]);
    // depth-sorted items
    var items = [];
    var Tn = sim.mL * C.g / (N * Math.cos(sim.theta0));
    for (i = 0; i < N; i++) {
      var dp = [fr.d[3 * i], fr.d[3 * i + 1], fr.d[3 * i + 2]];
      if (fr.st[i] !== 2) {
        var c0 = P(dp[0], dp[1], dp[2]), c1 = P(fr.L[0], fr.L[1], fr.L[2]);
        items.push({ z: (c0.z + c1.z) / 2 + 0.01, draw: (function (c0, c1, Ti) { return function () {
          var taut = Ti > 1e-4;
          ctx.strokeStyle = taut ? '#555555' : '#bbbbbb';
          ctx.lineWidth = taut ? Math.min(4, 1 + 1.6 * Ti / Tn) : 1;
          if (!taut) ctx.setLineDash([3, 3]);
          ctx.beginPath(); ctx.moveTo(c0.x, c0.y); ctx.lineTo(c1.x, c1.y); ctx.stroke(); ctx.setLineDash([]);
        }; })(c0, c1, fr.T[i]) });
      }
      var cp = P(dp[0], dp[1], dp[2]);
      items.push({ z: cp.z, draw: (function (cp, i, stt) { return function () { drawDrone(ctx, cp, i, stt); }; })(cp, i, fr.st[i]) });
    }
    var lp = P(fr.L[0], fr.L[1], fr.L[2]);
    items.push({ z: lp.z, draw: function () {
      var r = Math.max(4, 0.035 * lp.s);
      var g = ctx.createRadialGradient(lp.x - r * 0.35, lp.y - r * 0.35, r * 0.1, lp.x, lp.y, r);
      g.addColorStop(0, '#8d8c88'); g.addColorStop(1, '#3d3c3a');
      ctx.fillStyle = g; ctx.beginPath(); ctx.arc(lp.x, lp.y, r, 0, 2 * Math.PI); ctx.fill();
    } });
    items.sort(function (a, b) { return b.z - a.z; });
    items.forEach(function (it) { it.draw(); });

    // HUD
    var dt = fr.t - st.p.t_f;
    var mode = fr.t < st.p.t_f ? 'HP (nominal)' : fr.mode + (fr.pending ? ' → ' + fr.pending + ' pending' : '');
    if (fr.t >= st.p.t_f && sim.decision === null) mode += '';
    ui.hud.innerHTML = '<b>t − t<sub>f</sub> = ' + (dt >= 0 ? '+' : '−') + Math.abs(dt).toFixed(2) + ' s</b><br>mode: ' + mode +
      '<br>tracking error: ' + fmt(fr.err, 3) + ' m' + (fr.delay > 0.01 ? '<br>reference delay: ' + fmt(fr.delay, 2) + ' s' : '');
    ui.timeLbl.textContent = 't = ' + fr.t.toFixed(2) + ' s';
    ui.slider.value = st.ti;
  }

  function drawDrone(ctx, cp, i, stt) {
    var r = Math.max(8, 0.075 * cp.s);
    var col = stt === 0 ? COLORS[i] : (stt === 1 ? '#ffffff' : '#c8c8c8');
    var edge = stt === 1 ? '#c0392b' : '#ffffff';
    ctx.lineWidth = 2;
    [[-1, -1], [1, -1], [1, 1], [-1, 1]].forEach(function (o) {
      ctx.beginPath(); ctx.arc(cp.x + o[0] * r * 0.62, cp.y + o[1] * r * 0.38, r * 0.42, 0, 2 * Math.PI);
      ctx.fillStyle = col; ctx.fill(); ctx.strokeStyle = edge; ctx.stroke();
    });
    ctx.beginPath(); ctx.arc(cp.x, cp.y, r * 0.42, 0, 2 * Math.PI); ctx.fillStyle = stt === 0 ? col : '#ffffff'; ctx.fill();
    ctx.strokeStyle = stt === 1 ? '#c0392b' : (stt === 2 ? '#c8c8c8' : '#ffffff'); ctx.stroke();
    ctx.fillStyle = stt === 0 ? '#ffffff' : (stt === 1 ? '#c0392b' : '#8a8a8a');
    ctx.font = '700 ' + Math.round(Math.max(10, r * 0.7)) + 'px sans-serif'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(stt === 1 ? '✕' : String(i + 1), cp.x, cp.y + 0.5);
    if (stt !== 0) { ctx.fillStyle = '#8a8a8a'; ctx.font = '11px sans-serif'; ctx.fillText(String(i + 1), cp.x + r * 1.3, cp.y - r * 0.9); }
    ctx.textBaseline = 'alphabetic';
  }

  /* ---------------------------------------------------------------- */
  /* Plots                                                             */
  /* ---------------------------------------------------------------- */
  function buildPlots() {
    var box = ui.plotBox; box.innerHTML = '';
    st.plots = [];
    var sim = st.res.sim, frames = sim.frames, N = st.p.N, tf = st.p.t_f;
    var t0 = Math.max(0, tf - 1.0);
    var i0 = frames.findIndex(function (f) { return f.t >= t0; });
    var fr = frames.slice(i0);
    var xs = fr.map(function (f) { return f.t - tf; });
    var surv = sim.survivors;
    var specs = [
      { title: 'Payload tracking error', unit: 'm', series: [{ label: 'error', color: '#52514e', y: fr.map(function (f) { return f.err; }) }],
        hl: [{ y: C.delta_HP, label: 'δ_HP 0.30 m' }, { y: C.delta_SC, label: 'δ_SC 0.60 m' }], yMin: 0 },
      { title: 'Thrust use f / f̄ (survivors)', unit: '', series: surv.map(function (i) { return { label: 'drone ' + (i + 1), color: COLORS[i], y: fr.map(function (f) { return f.f[i] / C.f_bar; }) }; }),
        hl: [{ y: 1, label: 'saturation' }], yMin: 0, yMax: 1.1 },
      { title: 'Cable tension (all drones; failed drones end at release)', unit: 'N', series: range(N).map(function (i) { return { label: 'drone ' + (i + 1) + (sim.failSet[i] ? ' (failed)' : ''), color: COLORS[i], dash: !!sim.failSet[i], y: fr.map(function (f) { return f.T[i]; }) }; }),
        hl: [], yMin: 0 },
      { title: 'Payload altitude', unit: 'm', series: [{ label: 'payload z', color: '#52514e', y: fr.map(function (f) { return f.L[2]; }) }], hl: [], yMin: 0 }
    ];
    specs.forEach(function (sp) {
      var card = el('div', { class: 'dl-plot' });
      card.appendChild(el('div', { class: 'dl-plot-title', text: sp.title + (sp.unit ? ' [' + sp.unit + ']' : '') }));
      if (sp.series.length > 1) {
        var lg = el('div', { class: 'dl-plot-legend' });
        sp.series.forEach(function (s) { lg.appendChild(el('span', { html: '<i class="dl-sw' + (s.dash ? ' dl-sw-dash' : '') + '" style="background:' + s.color + '"></i>' + s.label })); });
        card.appendChild(lg);
      }
      var cv = el('canvas', { class: 'dl-plot-canvas' });
      var tip = el('div', { class: 'dl-tip' });
      card.appendChild(cv); card.appendChild(tip);
      box.appendChild(card);
      var plot = { sp: sp, cv: cv, tip: tip, xs: xs, frames: fr, i0: i0, hover: null };
      cv.addEventListener('mousemove', function (e) {
        var rc = cv.getBoundingClientRect();
        plot.hover = e.clientX - rc.left; drawPlot(plot);
      });
      cv.addEventListener('mouseleave', function () { plot.hover = null; tip.style.display = 'none'; drawPlot(plot); });
      cv.addEventListener('click', function () {
        if (plot.hIdx !== undefined) { st.ti = plot.i0 + plot.hIdx; setPlaying(false); drawAll(); }
      });
      st.plots.push(plot);
    });
  }
  function range(n) { var a = []; for (var i = 0; i < n; i++) a.push(i); return a; }

  function drawPlot(pl) {
    var F = fitCanvas(pl.cv, 0), ctx = F.ctx, W = F.w, H = F.h, sp = pl.sp;
    var m = { l: 44, r: 10, t: 16, b: 22 };
    ctx.clearRect(0, 0, W, H);
    var xs = pl.xs, x0 = xs[0], x1 = xs[xs.length - 1];
    var ymax = sp.yMax !== undefined ? sp.yMax : 0, ymin = sp.yMin !== undefined ? sp.yMin : Infinity;
    sp.series.forEach(function (s) { for (var i = 0; i < s.y.length; i++) { if (sp.yMax === undefined) ymax = Math.max(ymax, s.y[i]); if (sp.yMin === undefined) ymin = Math.min(ymin, s.y[i]); } });
    sp.hl.forEach(function (h) { if (sp.yMax === undefined) ymax = Math.max(ymax, h.y * 1.08); });
    if (!(ymax > ymin)) ymax = ymin + 1;
    ymax *= 1.05;
    function X(x) { return m.l + (x - x0) / (x1 - x0) * (W - m.l - m.r); }
    function Y(y) { return H - m.b - (y - ymin) / (ymax - ymin) * (H - m.t - m.b); }
    // mode bands (top strip) with text labels
    var fr = pl.frames, prev = null, start = 0;
    function band(a, b, mode) {
      ctx.fillStyle = MODE_FILL[mode] || '#f3f3f3';
      ctx.fillRect(X(xs[a]), m.t - 12, X(xs[b]) - X(xs[a]), 10);
      if (X(xs[b]) - X(xs[a]) > 18) { ctx.fillStyle = INK2; ctx.font = '9px sans-serif'; ctx.textAlign = 'left'; ctx.fillText(mode, X(xs[a]) + 3, m.t - 4); }
    }
    for (var i = 0; i < fr.length; i++) {
      var md = fr[i].mode;
      if (md !== prev) { if (prev !== null) band(start, i, prev); prev = md; start = i; }
    }
    band(start, fr.length - 1, prev);
    // grid + axes
    ctx.strokeStyle = GRID; ctx.lineWidth = 1; ctx.fillStyle = INK2; ctx.font = '10px sans-serif';
    var ny = 4;
    for (var k = 0; k <= ny; k++) {
      var yv = ymin + (ymax - ymin) * k / ny, yy = Y(yv);
      ctx.beginPath(); ctx.moveTo(m.l, yy); ctx.lineTo(W - m.r, yy); ctx.stroke();
      ctx.textAlign = 'right'; ctx.fillText(yv.toFixed(ymax < 1 ? 2 : 1), m.l - 4, yy + 3);
    }
    ctx.textAlign = 'center';
    for (var xt = Math.ceil(x0); xt <= x1; xt += 1) ctx.fillText((xt > 0 ? '+' : '') + xt + ' s', X(xt), H - 6);
    // failure line
    ctx.strokeStyle = '#c0392b'; ctx.setLineDash([2, 3]);
    ctx.beginPath(); ctx.moveTo(X(0), m.t); ctx.lineTo(X(0), H - m.b); ctx.stroke(); ctx.setLineDash([]);
    // threshold lines
    sp.hl.forEach(function (h) {
      ctx.strokeStyle = '#9a9a9a'; ctx.setLineDash([6, 4]);
      ctx.beginPath(); ctx.moveTo(m.l, Y(h.y)); ctx.lineTo(W - m.r, Y(h.y)); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = INK2; ctx.textAlign = 'right'; ctx.fillText(h.label, W - m.r - 2, Y(h.y) - 3);
    });
    // series
    sp.series.forEach(function (s) {
      ctx.strokeStyle = s.color; ctx.lineWidth = 2; if (s.dash) ctx.setLineDash([4, 3]);
      ctx.beginPath();
      for (var i = 0; i < s.y.length; i++) { var px = X(xs[i]), py = Y(s.y[i]); if (i) ctx.lineTo(px, py); else ctx.moveTo(px, py); }
      ctx.stroke(); ctx.setLineDash([]);
    });
    // playback cursor
    var ci = st.ti - pl.i0;
    if (ci >= 0 && ci < xs.length) {
      ctx.strokeStyle = INK; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(X(xs[ci]), m.t); ctx.lineTo(X(xs[ci]), H - m.b); ctx.stroke();
    }
    // hover crosshair + tooltip
    if (pl.hover !== null && pl.hover >= m.l && pl.hover <= W - m.r) {
      var xv = x0 + (pl.hover - m.l) / (W - m.l - m.r) * (x1 - x0);
      var hi = Math.max(0, Math.min(xs.length - 1, Math.round((xv - x0) / (x1 - x0) * (xs.length - 1))));
      pl.hIdx = hi;
      ctx.strokeStyle = '#9a9a9a'; ctx.beginPath(); ctx.moveTo(X(xs[hi]), m.t); ctx.lineTo(X(xs[hi]), H - m.b); ctx.stroke();
      sp.series.forEach(function (s) { ctx.fillStyle = '#fff'; ctx.strokeStyle = s.color; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(X(xs[hi]), Y(s.y[hi]), 4, 0, 2 * Math.PI); ctx.fill(); ctx.stroke(); });
      pl.tip.style.display = 'block';
      pl.tip.innerHTML = '<b>t − t<sub>f</sub> = ' + (xs[hi] >= 0 ? '+' : '') + xs[hi].toFixed(2) + ' s</b> · ' + fr[hi].mode + '<br>' +
        sp.series.map(function (s) { return '<i class="dl-sw" style="background:' + s.color + '"></i>' + s.label + ': ' + s.y[hi].toFixed(3); }).join('<br>') + '<br><span class="dl-muted">click to seek</span>';
      var left = X(xs[hi]) + 12; if (left > W - 170) left = X(xs[hi]) - 172;
      pl.tip.style.left = left + 'px'; pl.tip.style.top = (m.t + 26) + 'px';
    } else pl.hIdx = undefined;
  }

  function drawAll() { draw3D(); st.plots.forEach(drawPlot); }

  function setPlaying(on) {
    st.playing = on && !!st.res;
    ui.playBtn.textContent = st.playing ? '❚❚ Pause' : '▶ Play';
    if (st.playing) {
      if (st.res && st.ti >= st.res.sim.frames.length - 1) st.ti = 0;
      st.last = performance.now();
      requestAnimationFrame(tick);
    }
  }
  function tick(now) {
    if (!st.playing) return;
    var dtw = Math.min(0.1, (now - st.last) / 1000); st.last = now;
    st.tAcc += dtw * st.speed;
    var step = C.dt * C.rec_every;
    var adv = Math.floor(st.tAcc / step);
    if (adv > 0) { st.tAcc -= adv * step; st.ti += adv; }
    var n = st.res.sim.frames.length;
    if (st.ti >= n - 1) { st.ti = n - 1; drawAll(); setPlaying(false); return; }
    drawAll();
    requestAnimationFrame(tick);
  }

  /* ---------------------------------------------------------------- */
  /* Batch (Monte Carlo)                                               */
  /* ---------------------------------------------------------------- */
  function buildBatch(root) {
    var form = el('div', { class: 'dl-batch-form' });
    ui.bK = el('input', { type: 'number', min: 5, max: 500, step: 5, value: 40, class: 'dl-num', id: 'dl-bK' });
    ui.bSeed = el('input', { type: 'number', min: 1, max: 99999, step: 1, value: 1, class: 'dl-num', id: 'dl-bSeed' });
    function chk(id, text, on) { var c = el('input', { type: 'checkbox', id: id }); c.checked = !!on; return { c: c, row: el('label', { class: 'dl-chk', for: id }, [c, ' ' + text]) }; }
    ui.bFixN = chk('dl-bFixN', 'fix team size N to the value above', false);
    ui.bFixF = chk('dl-bFixF', 'fix failure count |F| to the value above (pattern still random)', false);
    ui.bFixL = chk('dl-bFixL', 'fix payload configuration (ρ_L or m_L, ℓ, θ₀) to the values above', false);
    ui.bFixS = chk('dl-bFixS', 'fix latency scale s_Λ', false);
    ui.bFixC = chk('dl-bFixC', 'use the SC parameters and λ above', true);
    form.appendChild(el('div', { class: 'dl-row-inline' }, [el('label', { for: 'dl-bK', text: 'Scenarios ' }), ui.bK, el('label', { for: 'dl-bSeed', text: ' first seed ' }), ui.bSeed]));
    [ui.bFixN, ui.bFixF, ui.bFixL, ui.bFixS, ui.bFixC].forEach(function (x) { form.appendChild(x.row); });
    ui.bRun = el('button', { class: 'dl-btn dl-btn-primary', text: 'Run batch', onclick: runBatch });
    ui.bStop = el('button', { class: 'dl-btn', text: 'Stop', onclick: function () { if (st.batch) st.batch.stop = true; } });
    ui.bStop.disabled = true;
    ui.bCsv = el('button', { class: 'dl-btn', text: 'Download CSV', onclick: downloadCsv });
    ui.bCsv.disabled = true;
    ui.bProg = el('progress', { max: 1, value: 0, class: 'dl-progress' });
    ui.bStatus = el('span', { class: 'dl-note' });
    form.appendChild(el('div', { class: 'dl-actions' }, [ui.bRun, ui.bStop, ui.bCsv, ui.bProg, ui.bStatus]));
    ui.bOut = el('div', { class: 'dl-batch-out' });
    root.appendChild(form); root.appendChild(ui.bOut);
  }

  function runBatch() {
    var K = Math.max(1, Math.min(500, parseInt(ui.bK.value, 10) || 40)), s0 = parseInt(ui.bSeed.value, 10) || 1;
    var p = st.p, fixed = {};
    if (ui.bFixN.c.checked) fixed.N = p.N;
    if (ui.bFixF.c.checked) fixed.nF = Math.max(1, p.F.length);
    if (ui.bFixL.c.checked) {
      if (p.loadMode === 'mass') { fixed.loadMode = 'mass'; fixed.m_L = p.m_L; } else fixed.rho = p.rho;
      fixed.ell = p.ell; fixed.theta0_deg = p.theta0_deg;
    }
    if (ui.bFixS.c.checked) fixed.s_lat = p.s_lat;
    if (ui.bFixC.c.checked) { fixed.Tb = p.Tb; fixed.theta_sc_deg = p.theta_sc_deg; fixed.k_s = p.k_s; fixed.sc_stop = p.sc_stop; fixed.lambda = p.lambda; }
    var B = st.batch = { K: K, s0: s0, fixed: fixed, k: 0, rows: [], stop: false, t0: performance.now() };
    ui.bRun.disabled = true; ui.bStop.disabled = false; ui.bCsv.disabled = true;
    ui.bProg.max = K; ui.bProg.value = 0;
    function chunk() {
      var tEnd = performance.now() + 120;
      while (B.k < K && performance.now() < tEnd && !B.stop) {
        var sc = S.sampleScenario(s0 + B.k, fixed);
        var r = S.compareSystems(sc, { fast: true });
        B.rows.push({ seed: s0 + B.k, p: sc, r: r });
        B.k++;
      }
      ui.bProg.value = B.k;
      ui.bStatus.textContent = B.k + ' / ' + K + ' scenarios · ' + ((performance.now() - B.t0) / 1000).toFixed(1) + ' s';
      renderBatch();
      if (B.k < K && !B.stop) setTimeout(chunk, 0);
      else { ui.bRun.disabled = false; ui.bStop.disabled = true; ui.bCsv.disabled = !B.rows.length; }
    }
    setTimeout(chunk, 0);
  }

  function renderBatch() {
    var B = st.batch, n = B.rows.length; if (!n) return;
    var sys = ['hp', 'sc', 'fs', 'simplex'];
    var agg = {}; sys.forEach(function (s) { agg[s] = { viol: 0, kept: 0, HP: 0, SC: 0, FS: 0 }; });
    var inScope = 0;
    B.rows.forEach(function (row) {
      if (row.r.cap.inScope) inScope++;
      sys.forEach(function (s) {
        var o = row.r[s];
        if (!o.success) agg[s].viol++;
        if (o.success && o.finalMode !== 'FS') agg[s].kept++;
        agg[s][o.finalMode]++;
      });
    });
    var maxV = 0; sys.forEach(function (s) { maxV = Math.max(maxV, agg[s].viol / n); });
    var scale = Math.max(0.1, Math.ceil(maxV * 10 + 0.5) / 10);
    var html = '<table class="dl-table dl-batch-table"><thead><tr><th>System</th><th>Violation rate</th><th style="width:38%"></th><th>Mission kept (safe, ends in HP/SC)</th><th>Final mode HP / SC / FS</th></tr></thead><tbody>' +
      sys.map(function (s) {
        var a = agg[s], v = a.viol / n;
        return '<tr><td>' + SYS_NAMES[s] + '</td><td>' + (100 * v).toFixed(1) + ' % <span class="dl-muted">(' + a.viol + '/' + n + ')</span></td>' +
          '<td><div class="dl-bar" title="' + (100 * v).toFixed(1) + ' %"><span style="width:' + (100 * v / scale) + '%"></span></div></td>' +
          '<td>' + (100 * a.kept / n).toFixed(1) + ' %</td><td>' + a.HP + ' / ' + a.SC + ' / ' + a.FS + '</td></tr>';
      }).join('') + '</tbody></table>' +
      '<p class="dl-note">Bar scale: 0 to ' + (100 * scale).toFixed(0) + ' %. ' + inScope + ' of ' + n + ' scenarios are in scope (descent capacity holds for the survivors). With ' + n + ' scenarios, one run changes a rate by ' + (100 / n).toFixed(1) + ' points.</p>';
    ui.bOut.innerHTML = html;
  }

  function downloadCsv() {
    var B = st.batch; if (!B || !B.rows.length) return;
    var head = ['seed', 'N', 'F', 'rho', 'm_L', 'ell', 'theta0_deg', 'phase', 't_f', 's_lat', 'jitter', 'wind', 'Delta_ms', 'tau_HP', 'tau_SC', 'in_scope'];
    ['hp', 'sc', 'fs', 'simplex'].forEach(function (s) { head.push(s + '_success', s + '_final', s + '_maxErr', s + '_reason'); });
    var lines = [head.join(',')];
    B.rows.forEach(function (row) {
      var p = row.p, r = row.r;
      var v = [row.seed, p.N, '"' + p.F.map(function (i) { return i + 1; }).join(' ') + '"', p.rho.toFixed(3), S.payloadMass(p).toFixed(4), p.ell, p.theta0_deg, p.phase, p.t_f.toFixed(3), p.s_lat, p.jitter.toFixed(3), p.wind ? 1 : 0,
        (r.simplex.Delta * 1000).toFixed(1), r.budgets.HP, r.budgets.SC, r.cap.inScope ? 1 : 0];
      ['hp', 'sc', 'fs', 'simplex'].forEach(function (s) { var o = r[s]; v.push(o.success ? 1 : 0, o.finalMode, o.maxErr.toFixed(4), '"' + (o.reason || '').replace(/"/g, "'") + '"'); });
      lines.push(v.join(','));
    });
    var blob = new Blob([lines.join('\n')], { type: 'text/csv' });
    var a = el('a', { href: URL.createObjectURL(blob), download: 'dronelift_batch_seed' + B.s0 + '_K' + B.rows.length + '.csv' });
    document.body.appendChild(a); a.click(); a.remove();
  }

  /* ---------------------------------------------------------------- */
  /* Init                                                              */
  /* ---------------------------------------------------------------- */
  function init() {
    var root = document.getElementById('dronelift-app');
    if (!root) return;
    root.classList.add('dl-app');
    st.p = fromHash(location.hash);

    var head = el('div', { class: 'dl-section' });
    buildControls(head);
    root.appendChild(head);

    ui.runBtn = el('button', { class: 'dl-btn dl-btn-primary', text: '▶ Run simulation', onclick: runSingle });
    ui.cmpBtn = el('button', { class: 'dl-btn', text: 'Compare all four systems', onclick: runCompare });
    var rnd = el('button', { class: 'dl-btn', text: 'Random scenario', onclick: function () {
      var seed = 1 + Math.floor(Math.random() * 99999);
      st.p = S.sampleScenario(seed); st.p.seed = seed; changed(); runSingle();
    } });
    var reset = el('button', { class: 'dl-btn', text: 'Reset', onclick: function () { st.p = S.defaultParams(); changed(); } });
    var link = el('button', { class: 'dl-btn', text: 'Copy scenario link', onclick: function () {
      var url = location.href.split('#')[0] + toHash(st.p);
      var done = function () { link.textContent = 'Link copied'; setTimeout(function () { link.textContent = 'Copy scenario link'; }, 1500); };
      if (navigator.clipboard) navigator.clipboard.writeText(url).then(done, function () { prompt('Scenario link:', url); });
      else prompt('Scenario link:', url);
    } });
    ui.busy = el('span', { class: 'dl-note dl-busy', 'aria-live': 'polite' });
    ui.staleNote = el('span', { class: 'dl-stale', text: 'Parameters changed: run again to update.' });
    ui.staleNote.style.visibility = 'hidden';
    root.appendChild(el('div', { class: 'dl-actions' }, [ui.runBtn, ui.cmpBtn, rnd, reset, link, ui.busy, ui.staleNote]));

    root.appendChild(el('h3', { class: 'dl-h', text: 'Static capacity check' }));
    ui.cap = el('div', { class: 'dl-cap' }); root.appendChild(ui.cap);

    root.appendChild(el('h3', { class: 'dl-h', text: 'Simulation' }));
    buildViewer(root);
    ui.result = el('div', { class: 'dl-result' }); root.appendChild(ui.result);
    ui.plotBox = el('div', { class: 'dl-plots' }); root.appendChild(ui.plotBox);

    root.appendChild(el('h3', { class: 'dl-h', text: 'Same scenario, four systems' }));
    ui.cmp = el('div', { class: 'dl-cmp-box', html: '<p class="dl-note">Press “Compare all four systems” to run high-level-only, always-SC, always-FS and Simplex on this exact scenario.</p>' });
    root.appendChild(ui.cmp);

    var broot = document.getElementById('dronelift-batch');
    if (broot) { broot.classList.add('dl-app'); buildBatch(broot); }

    syncControls(); renderCapacity(); draw3D();
    window.addEventListener('resize', function () { drawAll(); });
    // auto-run once so reviewers see motion immediately
    runSingle();
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
  else init();
})();
