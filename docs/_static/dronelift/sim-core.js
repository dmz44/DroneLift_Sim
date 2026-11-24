/*
 * DroneLift browser simulator — core (no DOM).
 *
 * A reduced-order re-implementation of the closed loop studied in the paper:
 * N point-mass multirotors carry a point-mass payload on elastic, slack-able
 * cables; a crash failure removes drones, and a Simplex runtime switches among
 * a high-performance controller (HP), a safety controller (SC) and a
 * failsafe controller (FS) after a switching delay.
 *
 * Runs in browsers (window.DroneLiftSim) and in Node (module.exports).
 */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.DroneLiftSim = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  var G = 9.81;
  var DEG = Math.PI / 180;

  /* ------------------------------------------------------------------ */
  /* Constants (fixed system and specification parameters)              */
  /* ------------------------------------------------------------------ */
  var CONST = {
    g: G,
    m_d: 0.027,          // drone mass [kg] (Crazyflie 2.x)
    f_bar: 0.506,        // usable thrust per drone [N] (85% of motor max)
    tilt_max: 60 * DEG,  // (S3) maximum tilt
    d_sep: 0.10,         // (S1) minimum separation [m]
    h_min: 0.12,         // (S2) minimum drone height [m]
    a_bar: 0.6 * G,      // (S6) max payload downward acceleration under FS
    delta_HP: 0.30,      // HP promise tube [m]
    delta_SC: 0.60,      // SC promise tube [m]
    D_max: 4.0,          // max delay under SC [s]
    mu_pos: 0.05,        // watchdog position margin [m]
    mu_del: 0.3,         // watchdog delay margin [s]
    T_h: 9.0,            // evaluation horizon after failure [s]
    a_ref: 0.6,          // reference acceleration bound used in the load ratio [m/s^2]
    // nominal latencies at latency scale 1 (multiplied by s_lat * jitter)
    lat_det: 0.050, lat_dec: 0.010, lat_ho: 0.020, lat_rel: 0.030,
    T_bl: 0.200,         // handover blend (not scaled)
    // simulation-only parameters
    dt: 0.001,
    rec_every: 10,
    k_cable: 60.0,       // cable axial stiffness [N/m]
    c_cable: 0.30,       // cable axial damping [N s/m]
    tau_thrust: 0.04,    // thrust-vector lag (attitude loop) [s]
    Kp: [15, 15, 40],    // drone position gains [1/s^2]
    Kd: [8, 8, 16],      // drone velocity gains [1/s]
    k_pL: 0.5,           // payload position feedback in drone setpoints
    drag_d: 0.02,        // linear drag drone [N s/m]
    drag_L: 0.03,        // linear drag payload [N s/m]
    // failsafe (non-learned) constants
    fs_Tb: 1.5, fs_ks: 0.5, fs_acc: 0.3, fs_vdesc: 0.3,
    sc_hold: 1.5         // SC: time the reference stays stopped before rejoining [s]
  };

  var BUDGET_GRID = [0.02, 0.04, 0.08, 0.12, 0.16, 0.24, 0.32, 0.48, 0.64, 0.96, 1.28, 1.92];
  var MODE_RANK = { HP: 2, SC: 1, FS: 0 };

  function defaultParams() {
    return {
      N: 5, F: [0, 1], psi0_deg: 0,
      loadMode: 'ratio', rho: 0.55, m_L: 0.040,
      ell: 0.6, theta0_deg: 35,
      phase: 'turn', t_f: 3.8, v0: 0.45, a0: 0.45, r_turn: 2.0, turn_dir: 1,
      z0: 1.45, chi0_deg: 0, wind: false,
      s_lat: 4, jitter: 1,
      selector: 'simplex', lambda: 0.0,
      Tb: 2.5, theta_sc_deg: 35, k_s: 0.8, sc_stop: 0,
      seed: 1
    };
  }

  /* ------------------------------------------------------------------ */
  /* Small helpers                                                       */
  /* ------------------------------------------------------------------ */
  function mulberry32(a) {
    return { s: a >>> 0 };
  }
  function rnd(r) {
    var t = (r.s = (r.s + 0x6D2B79F5) >>> 0);
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  }
  function randn(r) {
    var u = Math.max(rnd(r), 1e-12), v = rnd(r);
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
  }
  function smooth(x) { x = x < 0 ? 0 : x > 1 ? 1 : x; return x * x * (3 - 2 * x); }
  function dsmooth(x) { return (x <= 0 || x >= 1) ? 0 : 6 * x * (1 - x); }
  function clamp(x, a, b) { return x < a ? a : x > b ? b : x; }
  function norm3(a) { return Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); }
  function wrap2pi(a) { a %= 2 * Math.PI; return a < 0 ? a + 2 * Math.PI : a; }

  function payloadMass(p) {
    if (p.loadMode === 'mass') return p.m_L;
    var th = p.theta0_deg * DEG;
    return p.rho * p.N * Math.cos(th) * (CONST.f_bar - CONST.m_d * G) / (G + CONST.a_ref);
  }

  /* Capacity / safety-factor analysis (static, closed form). */
  function capacity(p) {
    var mL = payloadMass(p), th = p.theta0_deg * DEG;
    var N = p.N, n = N - p.F.length;
    var md = CONST.m_d, fb = CONST.f_bar;
    function hover(k) {
      if (k <= 0) return Infinity;
      var T = mL * G / (k * Math.cos(th));
      var fz = md * G + T * Math.cos(th), fh = T * Math.sin(th);
      return Math.sqrt(fz * fz + fh * fh);
    }
    var u = fb - md * G;                 // spare lift per drone
    var cosT = Math.cos(th);
    var nD = Math.ceil(mL * (G - CONST.a_bar) / (u * cosT) - 1e-12);
    var nR = Math.ceil(1.1 * mL * (G + CONST.a_ref) / (u * cosT) - 1e-12);
    return {
      m_L: mL, n: n,
      rho: mL * (G + CONST.a_ref) / (N * cosT * u),
      rho_after: n > 0 ? mL * (G + CONST.a_ref) / (n * cosT * u) : Infinity,
      f_hover_N: hover(N), f_hover_n: hover(n),
      sigma_N: fb / hover(N), sigma_n: fb / hover(n),
      n_D: Math.max(nD, 2), n_R: Math.max(nR, 2),
      capD: n >= 2 && n * u * cosT >= mL * (G - CONST.a_bar),
      capR: n >= 2 && n * u * cosT >= 1.1 * mL * (G + CONST.a_ref),
      inScope: n >= 2 && n * u * cosT >= mL * (G - CONST.a_bar)
    };
  }

  /* ------------------------------------------------------------------ */
  /* Reference trajectory (payload), tabulated in reference time tau     */
  /* ------------------------------------------------------------------ */
  function buildReference(p, Tend) {
    var dt = 0.005, n = Math.ceil(Tend / dt) + 2;
    var P = new Float64Array(3 * n), V = new Float64Array(3 * n), A = new Float64Array(3 * n);
    var x = 0, y = 0, chi = p.chi0_deg * DEG;
    for (var k = 0; k < n; k++) {
      var tau = k * dt;
      var u = p.v0 * smooth(tau / 1.5);
      if (p.phase === 'accel' && tau >= 3.0) u += p.a0 * Math.min(tau - 3.0, 1.0);
      var chidot = (p.phase === 'turn' && tau >= 3.2) ? p.turn_dir * u / p.r_turn : 0;
      P[3 * k] = x; P[3 * k + 1] = y; P[3 * k + 2] = p.z0;
      V[3 * k] = u * Math.cos(chi); V[3 * k + 1] = u * Math.sin(chi); V[3 * k + 2] = 0;
      x += V[3 * k] * dt; y += V[3 * k + 1] * dt; chi += chidot * dt;
    }
    for (k = 0; k < n; k++) {
      var k0 = Math.max(k - 1, 0), k1 = Math.min(k + 1, n - 1);
      for (var j = 0; j < 3; j++) A[3 * k + j] = (V[3 * k1 + j] - V[3 * k0 + j]) / ((k1 - k0) * dt);
    }
    return {
      dt: dt, n: n, P: P, V: V, A: A,
      at: function (tau, out) {
        var s = clamp(tau / dt, 0, n - 1.000001), i = Math.floor(s), f = s - i;
        for (var j = 0; j < 3; j++) {
          out.p[j] = P[3 * i + j] * (1 - f) + P[3 * i + 3 + j] * f;
          out.v[j] = V[3 * i + j] * (1 - f) + V[3 * i + 3 + j] * f;
          out.a[j] = A[3 * i + j] * (1 - f) + A[3 * i + 3 + j] * f;
        }
        return out;
      }
    };
  }

  /* Reform: target geometry for the reconfigured team (Alg. Reform). */
  function reform(azimuths) {
    // azimuths: array of {i, psi}. Returns map i -> target, and max move.
    var n = azimuths.length;
    var s = azimuths.map(function (a) { return { i: a.i, psi: wrap2pi(a.psi) }; })
      .sort(function (a, b) { return a.psi - b.psi; });
    // choose the cyclic start that minimizes the max move
    var best = null;
    for (var st = 0; st < n; st++) {
      var c = [], ps = [];
      for (var r = 0; r < n; r++) {
        var e = s[(st + r) % n];
        var psi = e.psi + ((st + r) >= n ? 2 * Math.PI : 0);
        ps.push(psi);
        c.push(2 * Math.PI * r / n - psi);
      }
      var mx = Math.max.apply(null, c), mn = Math.min.apply(null, c);
      var alpha = -0.5 * (mx + mn), move = 0.5 * (mx - mn);
      if (!best || move < best.move - 1e-12) best = { st: st, alpha: alpha, move: move, c: c, ps: ps };
    }
    var targets = {};
    for (r = 0; r < n; r++) {
      var ent = s[(best.st + r) % n];
      targets[ent.i] = { from: best.ps[r], to: best.ps[r] + best.alpha + best.c[r] };
    }
    return { targets: targets, move: best.move };
  }

  /* ------------------------------------------------------------------ */
  /* Simulator                                                           */
  /* ------------------------------------------------------------------ */
  function Sim(params, opts) {
    opts = opts || {};
    this.p = Object.assign(defaultParams(), params);
    var p = this.p;
    this.record = opts.record !== false;
    this.earlyStop = !!opts.earlyStop;
    this.forced = opts.forced || null;      // {mode, delay} for oracle branches
    this.watchdogs = opts.watchdogs !== undefined ? opts.watchdogs : (p.selector === 'simplex');
    this.budgets = opts.budgets || null;

    this.mL = payloadMass(p);
    this.theta0 = p.theta0_deg * DEG;
    this.t_end = p.t_f + CONST.T_h;
    this.ref = opts.ref || buildReference(p, this.t_end + 1);
    var j = p.jitter;
    this.lat = {
      det: CONST.lat_det * p.s_lat * j, dec: CONST.lat_dec * p.s_lat * j,
      ho: CONST.lat_ho * p.s_lat * j, rel: CONST.lat_rel * p.s_lat * j
    };
    this.Delta = this.lat.det + this.lat.dec + this.lat.ho;
    this.failSet = {};
    for (var q = 0; q < p.F.length; q++) this.failSet[p.F[q]] = true;
    this.survivors = [];
    for (q = 0; q < p.N; q++) if (!this.failSet[q]) this.survivors.push(q);

    this.rng = mulberry32((p.seed * 2654435761) >>> 0 || 1);
    this._init();
  }

  Sim.prototype._init = function () {
    var p = this.p, N = p.N, th = this.theta0, ell = p.ell;
    var R = ell * Math.sin(th), h = ell * Math.cos(th);
    var r0 = this.ref.at(0, { p: [0, 0, 0], v: [0, 0, 0], a: [0, 0, 0] });
    var T0 = this.mL * G / (N * Math.cos(th));
    var stretch = 1 + T0 / CONST.k_cable / ell;
    this.slots = [];
    this.drones = [];
    for (var i = 0; i < N; i++) {
      var psi = p.psi0_deg * DEG + 2 * Math.PI * i / N;
      this.slots.push(psi);
      var pos = [r0.p[0] + R * Math.cos(psi) * stretch, r0.p[1] + R * Math.sin(psi) * stretch, r0.p[2] + h * stretch];
      var f = [T0 * Math.sin(th) * Math.cos(psi), T0 * Math.sin(th) * Math.sin(psi), CONST.m_d * G + T0 * Math.cos(th)];
      this.drones.push({ p: pos, v: [0, 0, 0], f: f, T: T0, alive: true, attached: true, grounded: false });
    }
    this.L = { p: r0.p.slice(), v: [0, 0, 0], grounded: false, vzPrev: 0, az: 0 };
    // wind (OU process), direction drawn from the seed
    var wdir = rnd(this.rng) * 2 * Math.PI;
    this.wind = { on: !!p.wind, dir: [Math.cos(wdir), Math.sin(wdir)], w: [0, 0], next: 0 };
    if (this.wind.on) { this.wind.w = [0.5 * this.wind.dir[0], 0.5 * this.wind.dir[1]]; }
    this.t = 0;
    this.k = 0;
    // controllers
    this.cur = { type: 'HP', members: range(N), phi: this.slots.slice(), from: this.slots.slice(), to: this.slots.slice(), n: N, t_act: 0, stale: false, sigma: 0, sdot: 1, theta: this.theta0, ks: 0 };
    this.prev = null; this.t_blend = -1;
    this.mode = 'HP';                 // nominal mode before failure
    this.modeLabel = 'HP';
    this.pending = null;              // {mode, t_act}
    this.floor = 2;                   // watchdog floor (rank)
    this.events = [];
    this.failed = false; this.decided = false;
    this.viol = null;                 // first safety violation
    this.maxErr = 0; this.maxErrPreFS = 0; this.maxDelay = 0; this.maxAzDown = 0;
    this.t_FS = null; this.t_td = null; this.v_td = null;
    this.frames = [];
    this.minSep = Infinity; this.minH = Infinity;
    this.decision = null;
    this.done = false;
    this._tmp = { p: [0, 0, 0], v: [0, 0, 0], a: [0, 0, 0] };
  };

  function range(n) { var a = []; for (var i = 0; i < n; i++) a.push(i); return a; }

  /* Deep copy of the dynamic state (for counterfactual branches). */
  Sim.prototype.clone = function (opts) {
    var s = Object.create(Sim.prototype);
    for (var key in this) if (Object.prototype.hasOwnProperty.call(this, key)) s[key] = this[key];
    s.drones = this.drones.map(function (d) {
      return { p: d.p.slice(), v: d.v.slice(), f: d.f.slice(), T: d.T, alive: d.alive, attached: d.attached, grounded: d.grounded };
    });
    s.L = { p: this.L.p.slice(), v: this.L.v.slice(), grounded: this.L.grounded, vzPrev: this.L.vzPrev, az: this.L.az };
    s.wind = { on: this.wind.on, dir: this.wind.dir.slice(), w: this.wind.w.slice(), next: this.wind.next };
    s.rng = { s: this.rng.s };
    s.cur = cloneCtrl(this.cur); s.prev = this.prev ? cloneCtrl(this.prev) : null;
    s.events = this.events.slice();
    s.frames = [];
    s.pending = this.pending ? Object.assign({}, this.pending) : null;
    s.slots = this.slots.slice();
    s._tmp = { p: [0, 0, 0], v: [0, 0, 0], a: [0, 0, 0] };
    s._out = null; s._out2 = null;
    opts = opts || {};
    s.record = !!opts.record; s.earlyStop = !!opts.earlyStop;
    s.forced = opts.forced || null;
    s.watchdogs = opts.watchdogs !== undefined ? opts.watchdogs : false;
    return s;
  };
  function cloneCtrl(c) {
    var o = Object.assign({}, c);
    o.members = c.members.slice(); o.phi = c.phi.slice();
    if (c.from) o.from = c.from.slice();
    if (c.to) o.to = c.to.slice();
    if (c.holdXY) o.holdXY = c.holdXY.slice();
    return o;
  }

  /* Azimuth of a drone around the payload. */
  Sim.prototype._azimuth = function (i) {
    var d = this.drones[i];
    return Math.atan2(d.p[1] - this.L.p[1], d.p[0] - this.L.p[0]);
  };

  /* Build a controller for mode m configured for the current survivors. */
  Sim.prototype._makeCtrl = function (m) {
    var mem = this.survivors.slice(), self = this;
    // start from the slot azimuths of the stale configuration (nearest branch to current geometry)
    var az = mem.map(function (i) {
      var a = self._azimuth(i), s0 = self.cur.phi[self.cur.members.indexOf(i)];
      if (s0 === undefined) s0 = self.slots[i];
      // pick the slot value but follow the actual geometry if it drifted
      var d = Math.atan2(Math.sin(a - s0), Math.cos(a - s0));
      return { i: i, psi: s0 + d };
    });
    var rf = reform(az);
    var from = mem.map(function (i) { return rf.targets[i].from; });
    var to = mem.map(function (i) { return rf.targets[i].to; });
    var sigma = this.cur.sigma !== undefined ? this.cur.sigma : this.t;
    var c = { type: m, members: mem, n: mem.length, from: from, to: to, phi: to.slice(), t_act: this.t, stale: false, sigma: sigma, sdot: 1, move: rf.move };
    if (m === 'SC') {
      c.Tb = this.p.Tb; c.theta = this.p.theta_sc_deg * DEG; c.ks = this.p.k_s; c.Tstop = this.p.sc_stop;
    } else if (m === 'FS') {
      c.Tb = CONST.fs_Tb; c.theta = this.theta0; c.ks = CONST.fs_ks;
      c.z0 = this.L.p[2]; c.vz = 0;
      var r = this.ref.at(sigma, this._tmp);
      c.holdXY = [r.p[0], r.p[1]];
      // stop horizontally from the current reference velocity
      c.vxy0 = [r.v[0] * this.cur.sdot, r.v[1] * this.cur.sdot];
    } else {
      c.Tb = 0; c.theta = this.theta0; c.ks = 0;
    }
    return c;
  };

  /* Controller output: commanded thrust vector for each member of ctrl. */
  Sim.prototype._ctrlOut = function (c, out) {
    var t = this.t, dt = CONST.dt, L = this.L, mL = this.mL, ell = this.p.ell;
    var refp = [0, 0, 0], refv = [0, 0, 0], refa = [0, 0, 0];
    var theta = c.theta !== undefined ? c.theta : this.theta0;
    var s = 1, sw = [0, 0];
    var rr = this._tmp;

    if (c.type === 'HP') {
      c.sigma = t; c.sdot = 1;
      this.ref.at(t, rr);
      refp = rr.p.slice(); refv = rr.v.slice(); refa = rr.a.slice();
      s = 1;  // HP reconfigured: immediate step to reform targets
    } else if (c.type === 'SC') {
      var x = (t - c.t_act);
      // time warp: stop (frozen if Tstop = 0), hold, then rejoin over Tb
      var Ts = c.Tstop, Th = CONST.sc_hold, Tb = c.Tb;
      var sd;
      if (x < Ts) sd = 1 - smooth(x / Ts);
      else if (x < Ts + Th) sd = 0;
      else sd = smooth((x - Ts - Th) / Tb);
      if (Ts <= 0 && x < Th) sd = 0;
      c.sdot = sd; c.sigma += sd * dt;
      this.ref.at(c.sigma, rr);
      refp = rr.p.slice();
      for (var j = 0; j < 3; j++) { refv[j] = rr.v[j] * sd; refa[j] = rr.a[j] * sd * sd; }
      s = smooth(x / Tb);
      theta = this.theta0 + (c.theta - this.theta0) * s;
    } else { // FS
      var y = t - c.t_act;
      var Tst = 1.5;
      // horizontal: decelerate from the reference velocity to rest
      var sd2 = 1 - smooth(y / Tst);
      if (c._hx === undefined) { c._hx = c.holdXY[0]; c._hy = c.holdXY[1]; }
      c._hx += c.vxy0[0] * sd2 * dt; c._hy += c.vxy0[1] * sd2 * dt;
      refp[0] = c._hx; refp[1] = c._hy;
      refv[0] = c.vxy0[0] * sd2; refv[1] = c.vxy0[1] * sd2;
      // vertical: bounded-acceleration descent
      if (!L.grounded || !c.landed) {
        c.vz = Math.max(c.vz - CONST.fs_acc * dt, -CONST.fs_vdesc);
        c.z0 += c.vz * dt;
        refp[2] = Math.max(c.z0, -0.05);
        refv[2] = c.z0 > -0.05 ? c.vz : 0;
        refa[2] = c.vz > -CONST.fs_vdesc ? -CONST.fs_acc : 0;
      }
      if (L.grounded && !c.landed) { c.landed = true; c.zLand = 0; }
      s = smooth(y / c.Tb);
      c.sdot = 0;
    }

    var R = ell * Math.sin(theta), h = ell * Math.cos(theta);
    // payload position feedback (horizontal)
    var ex = clamp(CONST.k_pL * (refp[0] - L.p[0]), -0.15, 0.15);
    var ey = clamp(CONST.k_pL * (refp[1] - L.p[1]), -0.15, 0.15);
    // swing feedback: payload velocity relative to member centroid
    if (c.ks > 0) {
      var cx = 0, cy = 0, nm = 0;
      for (var m = 0; m < c.members.length; m++) {
        var dd = this.drones[c.members[m]];
        if (!dd.alive) continue;
        cx += dd.v[0]; cy += dd.v[1]; nm++;
      }
      if (nm) { sw[0] = c.ks * (L.v[0] - cx / nm); sw[1] = c.ks * (L.v[1] - cy / nm); }
    }
    var landed = c.type === 'FS' && c.landed;
    var nCfg = c.n;
    for (m = 0; m < c.members.length; m++) {
      var i = c.members[m], d = this.drones[i];
      if (!d.alive) { out[i] = null; continue; }
      var psi = c.stale ? c.phi[m] : c.from[m] + (c.to[m] - c.from[m]) * s;
      if (c.type === 'HP' && !c.stale) psi = c.to[m];
      var cp = Math.cos(psi), sp = Math.sin(psi);
      var psp = [refp[0] + ex + R * cp, refp[1] + ey + R * sp, refp[2] + h];
      var vsp = [refv[0] + sw[0], refv[1] + sw[1], refv[2]];
      if (landed) { psp[2] = 0.85 * h; vsp[2] = 0; }
      // cable feedforward for a team of nCfg members
      var share = landed ? 0 : mL / nCfg;
      var ff = [share * refa[0] + share * G * Math.tan(theta) * cp,
                share * refa[1] + share * G * Math.tan(theta) * sp,
                share * (refa[2] + G)];
      var f = out[i] || [0, 0, 0];
      for (j = 0; j < 3; j++) {
        var a = refa[j] + CONST.Kp[j] * (psp[j] - d.p[j]) + CONST.Kd[j] * (vsp[j] - d.v[j]);
        f[j] = CONST.m_d * (a + (j === 2 ? G : 0)) + ff[j];
      }
      if (landed) { f[0] -= ff[0]; f[1] -= ff[1]; }
      out[i] = f;
    }
    c._refp = refp;
    return out;
  };

  /* Thrust constraints: tilt cone, then magnitude (direction preserved, as an
     attitude loop that holds attitude while the collective thrust saturates). */
  function constrain(f) {
    var fz = Math.max(f[2], 0), fh = Math.sqrt(f[0] * f[0] + f[1] * f[1]);
    var hmax = fz * Math.tan(CONST.tilt_max);
    if (fh > hmax) { var sc = fh > 0 ? hmax / fh : 0; f[0] *= sc; f[1] *= sc; fh = hmax; }
    f[2] = fz;
    var m = Math.sqrt(fh * fh + fz * fz);
    if (m > CONST.f_bar) { var k = CONST.f_bar / m; f[0] *= k; f[1] *= k; f[2] *= k; }
    return f;
  }

  Sim.prototype._request = function (m, why) {
    // downgrade request (watchdog). Modes only move down HP > SC > FS.
    var rank = MODE_RANK[m];
    if (!this.decided) { this.floor = Math.min(this.floor, rank); this._event('watchdog', why + ' (before decision) → floor ' + m); return; }
    var target = this.pending ? this.pending.mode : this.mode;
    if (rank >= MODE_RANK[target]) return;
    this.pending = { mode: m, t_act: this.t + this.lat.ho };
    this._event('watchdog', why + ' → request ' + m);
  };

  Sim.prototype._event = function (kind, text) {
    if (this.record) this.events.push({ t: this.t, kind: kind, text: text });
  };

  Sim.prototype._activate = function (m) {
    this.prev = this.cur;
    this.cur = this._makeCtrl(m);
    this.t_blend = this.t;
    this.mode = m;
    if (m === 'FS' && this.t_FS === null) { this.t_FS = this.t; this.maxErrPreFS = this.maxErr; }
    this._event('switch', 'activate ' + m + (m === 'HP' ? ' (reconfigured for ' + this.survivors.length + ' members)' : ''));
  };

  Sim.prototype._violate = function (why) {
    if (!this.viol) { this.viol = { t: this.t, why: why }; this._event('violation', why); }
    if (this.earlyStop) this.done = true;
  };

  Sim.prototype._decide = function () {
    var p = this.p, m;
    if (this.forced) m = this.forced.mode;
    else if (p.selector === 'hp') m = 'HP';
    else if (p.selector === 'sc') m = 'SC';
    else if (p.selector === 'fs') m = 'FS';
    else {
      var b = this.budgets || { HP: -1, SC: -1 };
      var need = this.Delta + p.lambda;
      m = b.HP >= need ? 'HP' : (b.SC >= need ? 'SC' : 'FS');
    }
    if (MODE_RANK[m] > this.floor) m = ['FS', 'SC', 'HP'][this.floor];
    this.decision = m;
    return m;
  };

  Sim.prototype.step = function () {
    var p = this.p, t = this.t, dt = CONST.dt, i, j, d;
    var N = p.N, L = this.L, mL = this.mL;

    /* ---- events ---- */
    if (!this.failed && t >= p.t_f) {
      this.failed = true;
      for (var q = 0; q < p.F.length; q++) this.drones[p.F[q]].alive = false;
      this.cur.stale = true;
      this._event('failure', 'crash of drone(s) ' + p.F.map(function (x) { return x + 1; }).join(', '));
      if (this.forced) { this.pending = { mode: this.forced.mode, t_act: p.t_f + this.forced.delay, initial: true }; this.decided = true; }
    }
    if (this.failed) {
      if (t >= p.t_f + this.lat.rel) for (q = 0; q < p.F.length; q++) {
        d = this.drones[p.F[q]];
        if (d.attached) { d.attached = false; d.T = 0; this._event('release', 'cable of drone ' + (p.F[q] + 1) + ' released'); }
      }
      if (!this.forced && !this.decided && t >= p.t_f + this.lat.det + this.lat.dec) {
        this.decided = true;
        var m = this._decide();
        this._event('decision', 'selector → ' + m);
        this.pending = { mode: m, t_act: t + this.lat.ho, initial: true };
      }
      if (this.pending && t >= this.pending.t_act) {
        var pm = this.pending.mode; this.pending = null;
        this._activate(pm);
      }
    }

    /* ---- controller ---- */
    var out = this._out || (this._out = []);
    for (i = 0; i < N; i++) out[i] = null;
    this._ctrlOut(this.cur, out);
    if (this.prev && t - this.t_blend < CONST.T_bl) {
      var o2 = this._out2 || (this._out2 = []);
      for (i = 0; i < N; i++) o2[i] = null;
      this._ctrlOut(this.prev, o2);
      var beta = (t - this.t_blend) / CONST.T_bl;
      for (i = 0; i < N; i++) if (out[i] && o2[i]) for (j = 0; j < 3; j++) out[i][j] = (1 - beta) * o2[i][j] + beta * out[i][j];
    } else this.prev = null;

    /* ---- wind ---- */
    var W = this.wind;
    if (W.on && t >= W.next) {
      var h = 0.01, tc = 1.0, sg = 0.5;
      for (j = 0; j < 2; j++) {
        var mu = 0.5 * W.dir[j];
        W.w[j] += (mu - W.w[j]) * h / tc + sg * Math.sqrt(2 * h / tc) * randn(this.rng);
      }
      W.next = t + h;
    }
    var wx = W.on ? W.w[0] : 0, wy = W.on ? W.w[1] : 0;

    /* ---- cables ---- */
    var FL = [0, 0, -mL * G];
    var k = CONST.k_cable, cc = CONST.c_cable, ell = p.ell;
    for (i = 0; i < N; i++) {
      d = this.drones[i];
      d.T = 0;
      if (!d.attached) continue;
      var dx = d.p[0] - L.p[0], dy = d.p[1] - L.p[1], dz = d.p[2] - L.p[2];
      var dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
      if (dist > ell) {
        var qx = dx / dist, qy = dy / dist, qz = dz / dist;
        var rate = (d.v[0] - L.v[0]) * qx + (d.v[1] - L.v[1]) * qy + (d.v[2] - L.v[2]) * qz;
        var T = k * (dist - ell) + cc * rate;
        if (T > 0) {
          d.T = T; d.q0 = qx; d.q1 = qy; d.q2 = qz;
          FL[0] += T * qx; FL[1] += T * qy; FL[2] += T * qz;
        }
      }
    }
    FL[0] += CONST.drag_L * (wx - L.v[0]); FL[1] += CONST.drag_L * (wy - L.v[1]); FL[2] += -CONST.drag_L * L.v[2];

    /* ---- drones ---- */
    var a = dt / CONST.tau_thrust;
    for (i = 0; i < N; i++) {
      d = this.drones[i];
      if (d.grounded) continue;
      if (d.alive && out[i]) {
        constrain(out[i]);
        for (j = 0; j < 3; j++) d.f[j] += (out[i][j] - d.f[j]) * a;
      } else { d.f[0] = d.f[1] = d.f[2] = 0; }
      var fx = d.f[0] + CONST.drag_d * (wx - d.v[0]), fy = d.f[1] + CONST.drag_d * (wy - d.v[1]), fz = d.f[2] - CONST.m_d * G - CONST.drag_d * d.v[2];
      if (d.T > 0) { fx -= d.T * d.q0; fy -= d.T * d.q1; fz -= d.T * d.q2; }
      d.v[0] += fx / CONST.m_d * dt; d.v[1] += fy / CONST.m_d * dt; d.v[2] += fz / CONST.m_d * dt;
      d.p[0] += d.v[0] * dt; d.p[1] += d.v[1] * dt; d.p[2] += d.v[2] * dt;
      if (d.p[2] <= 0) {
        d.p[2] = 0; d.v = [0, 0, 0]; d.grounded = true;
        if (!this.failSet[i] && this.failed) this._violate('surviving drone ' + (i + 1) + ' hit the ground');
      }
    }

    /* ---- payload ---- */
    var vzOld = L.v[2];
    for (j = 0; j < 3; j++) L.v[j] += FL[j] / mL * dt;
    for (j = 0; j < 3; j++) L.p[j] += L.v[j] * dt;
    if (L.p[2] <= 0) {
      if (!L.grounded) {
        L.grounded = true;
        if (this.t_td === null && this.failed) { this.t_td = t; this.v_td = -L.v[2]; }
        if (this.mode !== 'FS' && this.failed) this._violate('payload hit the ground outside failsafe');
        this._event('touchdown', 'payload touchdown at ' + (-L.v[2]).toFixed(2) + ' m/s');
      }
      L.p[2] = 0; L.v[2] = Math.max(L.v[2], 0); L.v[0] *= 0.9; L.v[1] *= 0.9;
    } else if (L.p[2] > 0.005) L.grounded = false;
    // filtered vertical acceleration for (S6)
    var azRaw = (L.v[2] - vzOld) / dt;
    L.az += (azRaw - L.az) * (dt / 0.02);

    this.t += dt; this.k++;
    t = this.t;

    /* ---- monitoring (from failure until end of horizon) ---- */
    if (this.failed) {
      var err = this._trackErr();
      var delay = t - (this.cur.sigma !== undefined ? this.cur.sigma : t);
      if (this.mode === 'FS') delay = 0;
      this.lastErr = err; this.lastDelay = delay;
      if (this.mode !== 'FS' || this.t_FS === null) { this.maxErr = Math.max(this.maxErr, err); }
      this.maxDelay = Math.max(this.maxDelay, delay);
      var sv = this.survivors, minSep = Infinity, minH = Infinity;
      for (var u = 0; u < sv.length; u++) {
        var du = this.drones[sv[u]];
        minH = Math.min(minH, du.p[2]);
        for (var w = u + 1; w < sv.length; w++) {
          var dw = this.drones[sv[w]];
          var ds = Math.hypot(du.p[0] - dw.p[0], du.p[1] - dw.p[1], du.p[2] - dw.p[2]);
          if (ds < minSep) minSep = ds;
        }
      }
      this.minSep = Math.min(this.minSep, minSep); this.minH = Math.min(this.minH, minH);
      if (minSep < CONST.d_sep) this._violate('(S1) separation ' + minSep.toFixed(3) + ' m < ' + CONST.d_sep + ' m');
      if (minH < CONST.h_min) this._violate('(S2) drone height ' + minH.toFixed(3) + ' m < ' + CONST.h_min + ' m');
      if (this.mode === 'FS' && this.t_td === null && -L.az > CONST.a_bar) this._violate('(S6) payload downward acceleration ' + (-L.az / G).toFixed(2) + ' g > 0.6 g');
      // early termination for oracle branches: promise of the forced mode
      if (this.earlyStop && this.forced) {
        var fm = this.forced.mode;
        if (fm === 'HP' && err > CONST.delta_HP) this.done = true;
        if (fm === 'SC' && (err > CONST.delta_SC || delay > CONST.D_max)) this.done = true;
        if (fm === 'FS' && this.mode !== 'FS' && err > CONST.delta_SC) this.done = true;
      }
      // watchdogs
      if (this.watchdogs && !this.forced) {
        var tgt = this.pending ? this.pending.mode : this.mode;
        if (this.mode === 'HP' && tgt === 'HP') {
          if (err > CONST.delta_HP - CONST.mu_pos) this._request('SC', 'W1: tracking error ' + err.toFixed(2) + ' m');
          else if (minSep < CONST.d_sep + 0.03) this._request('SC', 'W1: separation margin');
          else if (minH < CONST.h_min + 0.05) this._request('SC', 'W1: height margin');
        } else if (this.mode === 'SC' && tgt === 'SC') {
          if (err > CONST.delta_SC - CONST.mu_pos) this._request('FS', 'W2: tracking error ' + err.toFixed(2) + ' m');
          else if (delay > CONST.D_max - CONST.mu_del) this._request('FS', 'W2: delay ' + delay.toFixed(2) + ' s');
          else if (minSep < CONST.d_sep + 0.03) this._request('FS', 'W2: separation margin');
          else if (minH < CONST.h_min + 0.05) this._request('FS', 'W2: height margin');
        }
      }
    }
    if (this.record && this.k % CONST.rec_every === 0) this._recordFrame();
    if (t >= this.t_end) this.done = true;
  };

  Sim.prototype._trackErr = function () {
    var c = this.cur, rr = this._tmp;
    if (this.mode === 'FS' && this.t_FS !== null) return 0;
    var sig = c.type === 'HP' ? this.t : c.sigma;
    this.ref.at(sig, rr);
    var L = this.L.p;
    return Math.hypot(L[0] - rr.p[0], L[1] - rr.p[1], L[2] - rr.p[2]);
  };

  Sim.prototype._recordFrame = function () {
    var N = this.p.N, fr = { t: this.t, mode: this.failed ? this.mode : 'HP', pending: this.pending ? this.pending.mode : null };
    fr.d = new Float32Array(3 * N); fr.T = new Float32Array(N); fr.f = new Float32Array(N); fr.tilt = new Float32Array(N);
    fr.st = new Uint8Array(N);
    for (var i = 0; i < N; i++) {
      var d = this.drones[i];
      fr.d[3 * i] = d.p[0]; fr.d[3 * i + 1] = d.p[1]; fr.d[3 * i + 2] = d.p[2];
      fr.T[i] = d.T;
      var fm = norm3(d.f);
      fr.f[i] = fm;
      fr.tilt[i] = fm > 0 ? Math.acos(clamp(d.f[2] / fm, -1, 1)) : 0;
      fr.st[i] = d.alive ? 0 : (d.attached ? 1 : 2);
    }
    fr.L = new Float32Array(this.L.p);
    var rp = this.cur._refp || this.L.p;
    fr.R = new Float32Array(rp);
    fr.err = this.failed ? this.lastErr : this._preErr();
    fr.delay = this.failed ? this.lastDelay : 0;
    fr.az = this.L.az;
    this.frames.push(fr);
  };
  Sim.prototype._preErr = function () {
    var rr = this.ref.at(this.t, this._tmp), L = this.L.p;
    return Math.hypot(L[0] - rr.p[0], L[1] - rr.p[1], L[2] - rr.p[2]);
  };

  Sim.prototype.runTo = function (tStop) { while (!this.done && this.t < tStop - 1e-9) this.step(); return this; };
  Sim.prototype.run = function () { while (!this.done) this.step(); return this; };

  /* Outcome according to the success predicate. */
  Sim.prototype.outcome = function () {
    var fin = this.mode, ok = !this.viol, why = this.viol ? this.viol.why : null;
    if (ok) {
      if (fin === 'HP' && this.maxErr > CONST.delta_HP) { ok = false; why = 'HP promise broken: max tracking error ' + this.maxErr.toFixed(2) + ' m > ' + CONST.delta_HP + ' m'; }
      if (fin === 'SC' && this.maxErr > CONST.delta_SC) { ok = false; why = 'SC promise broken: max error ' + this.maxErr.toFixed(2) + ' m > ' + CONST.delta_SC + ' m'; }
      if (fin === 'SC' && this.maxDelay > CONST.D_max) { ok = false; why = 'SC promise broken: delay ' + this.maxDelay.toFixed(2) + ' s > ' + CONST.D_max + ' s'; }
      if (fin === 'FS' && this.maxErrPreFS > CONST.delta_SC) { ok = false; why = 'FS promise broken: error before failsafe ' + this.maxErrPreFS.toFixed(2) + ' m > ' + CONST.delta_SC + ' m'; }
    }
    return { success: ok, reason: why, finalMode: fin, decision: this.decision, maxErr: this.maxErr, maxDelay: this.maxDelay,
      minSep: this.minSep, minH: this.minH, t_td: this.t_td, v_td: this.v_td, Delta: this.Delta, violation: this.viol };
  };

  /* ------------------------------------------------------------------ */
  /* Oracle switching budgets: sweep the switching delay per mode         */
  /* ------------------------------------------------------------------ */
  function budgetsFrom(snap, modes) {
    var res = { HP: -1, SC: -1, FS: -1, sweep: {} };
    (modes || ['HP', 'SC', 'FS']).forEach(function (m) {
      var tau = -1, row = [];
      for (var g = 0; g < BUDGET_GRID.length; g++) {
        var D = BUDGET_GRID[g];
        var br = snap.clone({ forced: { mode: m, delay: D }, earlyStop: true, record: false, watchdogs: false });
        br.run();
        var ok = br.outcome().success;
        row.push(ok);
        if (!ok) break;
        tau = D;
      }
      res[m] = tau; res.sweep[m] = row;
    });
    return res;
  }

  /* Run one scenario end to end. Returns {sim, outcome, budgets, cap}. */
  function runScenario(params, opts) {
    opts = opts || {};
    var base = new Sim(params, { record: opts.record !== false });
    var budgets = null;
    if (base.p.selector === 'simplex' || opts.alwaysBudgets) {
      var pre = new Sim(params, { record: false, ref: base.ref });
      pre.runTo(base.p.t_f - 1e-9);
      budgets = budgetsFrom(pre);
      base.budgets = budgets;
    }
    base.run();
    return { sim: base, outcome: base.outcome(), budgets: budgets, cap: capacity(base.p) };
  }

  /* Evaluate the four systems of the paper on the same scenario. */
  function compareSystems(params, opts) {
    opts = opts || {};
    var p = Object.assign(defaultParams(), params);
    var ref = buildReference(p, p.t_f + CONST.T_h + 1);
    var pre = new Sim(Object.assign({}, p, { selector: 'simplex' }), { record: false, ref: ref });
    pre.runTo(p.t_f - 1e-9);
    var budgets = budgetsFrom(pre, opts.fast ? ['HP', 'SC'] : null);
    var out = {};
    ['hp', 'sc', 'fs', 'simplex'].forEach(function (sel) {
      var s = pre.clone({ record: false, watchdogs: sel === 'simplex' });
      s.p = Object.assign({}, p, { selector: sel });
      s.budgets = budgets;
      s.run();
      out[sel] = s.outcome();
    });
    out.budgets = budgets;
    out.cap = capacity(p);
    return out;
  }

  /* ------------------------------------------------------------------ */
  /* Scenario sampling (paper's domain-randomized distribution)          */
  /* ------------------------------------------------------------------ */
  function canonicalPattern(N, set) {
    // canonical representative of a subset of Z_N under the dihedral group D_N
    var best = null;
    for (var refl = 0; refl < 2; refl++) for (var r = 0; r < N; r++) {
      var img = set.map(function (i) { var x = refl ? (N - i) % N : i; return (x + r) % N; }).sort(function (a, b) { return a - b; });
      var key = img.join(',');
      if (best === null || key < best) best = key;
    }
    return best;
  }
  function strata(N, nF) {
    var groups = {};
    (function rec(start, acc) {
      if (acc.length === nF) { var k = canonicalPattern(N, acc); (groups[k] = groups[k] || []).push(acc.slice()); return; }
      for (var i = start; i < N; i++) { acc.push(i); rec(i + 1, acc); acc.pop(); }
    })(0, []);
    return Object.keys(groups).map(function (k) { return groups[k]; });
  }

  function sampleScenario(seed, fixed) {
    fixed = fixed || {};
    var r = mulberry32((seed * 747796405 + 2891336453) >>> 0);
    function U(a, b) { return a + (b - a) * rnd(r); }
    function pick(arr) { return arr[Math.floor(rnd(r) * arr.length)]; }
    var p = defaultParams();
    p.N = fixed.N !== undefined ? fixed.N : pick([3, 4, 5, 6, 7]);
    var nF = fixed.nF !== undefined ? Math.min(fixed.nF, p.N - 1) : 1 + Math.floor(rnd(r) * Math.max(p.N - 2, 1));
    var st = strata(p.N, nF);
    p.F = pick(pick(st));
    p.psi0_deg = U(0, 360);
    if (fixed.loadMode === 'mass') { p.loadMode = 'mass'; p.m_L = fixed.m_L; }
    else p.rho = fixed.rho !== undefined ? fixed.rho : pick([0.40, 0.55, 0.70]);
    if (fixed.ell !== undefined) p.ell = fixed.ell;
    if (fixed.theta0_deg !== undefined) p.theta0_deg = fixed.theta0_deg;
    p.s_lat = fixed.s_lat !== undefined ? fixed.s_lat : pick([0.5, 1, 2, 4, 8, 16]);
    p.jitter = Math.exp(0.2 * randn(r));
    p.phase = pick(['cruise', 'accel', 'turn']);
    p.v0 = U(0.3, 0.6); p.a0 = U(0.3, 0.6); p.r_turn = U(1.5, 3.0); p.turn_dir = rnd(r) < 0.5 ? -1 : 1;
    p.z0 = U(1.3, 1.6); p.chi0_deg = U(0, 360);
    p.wind = fixed.wind !== undefined ? fixed.wind : rnd(r) < 0.5;
    var win = { cruise: [2.0, 3.0], accel: [3.1, 3.6], turn: [3.4, 4.4] }[p.phase];
    p.t_f = U(win[0], win[1]);
    p.seed = (seed % 100000) + 1;
    if (fixed.Tb !== undefined) p.Tb = fixed.Tb;
    if (fixed.theta_sc_deg !== undefined) p.theta_sc_deg = fixed.theta_sc_deg;
    if (fixed.k_s !== undefined) p.k_s = fixed.k_s;
    if (fixed.sc_stop !== undefined) p.sc_stop = fixed.sc_stop;
    if (fixed.lambda !== undefined) p.lambda = fixed.lambda;
    return p;
  }

  return {
    CONST: CONST, BUDGET_GRID: BUDGET_GRID, defaultParams: defaultParams,
    payloadMass: payloadMass, capacity: capacity, reform: reform,
    budgetsFrom: budgetsFrom, Sim: Sim, runScenario: runScenario, compareSystems: compareSystems,
    sampleScenario: sampleScenario, strata: strata, buildReference: buildReference
  };
});
