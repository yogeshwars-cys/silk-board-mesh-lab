/*
 * Silk Board Mesh Lab: simulation core. No DOM; index.html and check.mjs both drive it.
 *
 * Traffic: a 1.2 km stretch of Hosur Road into the Silk Board signal, keep-left, lane-less mixed traffic.
 * Car-following is IDM+ with per-driver reaction time and occasional distraction; lateral movement is gap seeking.
 * Mesh: the RESILANE hazard relay rules from RoadMesh-iqoo core/mesh/MeshEngine.kt (ROAD profile), road queries from
 * core/road/RoadQueries.kt and the slowdown forecast from core/road/RoadPicture.kt, over a simple BLE reception model.
 */
(function (root) {
  'use strict';

  const L = 1200, W = 12, STOP_X = 1100, DT = 0.1, CELL = 10, CELLS = L / CELL, HIST = 480;
  const LANE_EDGES = [1.5, 5.0, 8.5];
  const BUS_STOP_X = 340;
  const POT = { x0: 640, x1: 651, y0: 4.6, y1: 7.9 };
  const SIGNAL = { green: 40, amber: 3, red: 77 };
  const CYCLE = SIGNAL.green + SIGNAL.amber + SIGNAL.red;
  const START_CLOCK = 18 * 3600 + 20 * 60;

  // Driver and vehicle types. Speeds in m/s, lengths in m. sight is how far ahead a driver notices a hazard.
  const TYPES = {
    bike:  { label: 'Two-wheeler', len: 1.9, wid: 0.75, v0: 46 / 3.6, a: 2.4, b: 3.5, T: 0.8, s0: 0.9, tau: [0.3, 0.6], lat: 1.3, latEvery: 0.4, sight: 35, heavy: false },
    car:   { label: 'Car', len: 4.3, wid: 1.75, v0: 46 / 3.6, a: 1.7, b: 3.0, T: 1.3, s0: 1.6, tau: [0.5, 0.9], lat: 0.7, latEvery: 0.8, sight: 28, heavy: false },
    auto:  { label: 'Auto', len: 2.7, wid: 1.35, v0: 38 / 3.6, a: 1.4, b: 2.8, T: 1.0, s0: 1.1, tau: [0.4, 0.8], lat: 0.9, latEvery: 0.6, sight: 30, heavy: false },
    bus:   { label: 'BMTC bus', len: 12, wid: 2.55, v0: 36 / 3.6, a: 0.9, b: 2.5, T: 1.6, s0: 2.2, tau: [0.6, 1.0], lat: 0.45, latEvery: 1.2, sight: 55, heavy: true },
    truck: { label: 'Truck', len: 7.5, wid: 2.4, v0: 34 / 3.6, a: 0.8, b: 2.5, T: 1.7, s0: 2.2, tau: [0.6, 1.0], lat: 0.4, latEvery: 1.2, sight: 50, heavy: true },
  };
  const MIX = [['bike', 0.56], ['car', 0.24], ['auto', 0.10], ['bus', 0.06], ['truck', 0.04]];

  // ROAD profile of MeshProfile plus the BLE burst sizes the app uses (origin 10 events, relay 4, 100 ms apart).
  const MESH = {
    ttl: 4, maxAgeMs: 5000, futureMs: 2000, maxRelayDelayMs: 200, jitterMs: 20, suppression: 2, rangeM: 150, relevanceM: 400,
    relayPerSecond: 3, originPerSecond: 2, neighborPerSecond: 8, dedupMs: 30000,
    advMs: 100, originEvents: 10, relayEvents: 4, ctrlMs: 200, ctrlEvents: 3, queryWindowMs: 2500,
    refreshMs: 20000, refreshWithinM: 250, mergeM: 25,
  };
  const KINDS = {
    BRAKING:       { label: 'Braking', use: 'caution', life: 90 },
    SLOWDOWN:      { label: 'Slowdown', use: 'caution', life: 120 },
    POTHOLE:       { label: 'Pothole', use: 'caution', life: 1800 },
    ACCIDENT:      { label: 'Accident', use: 'caution', life: 1200 },
    PEDESTRIAN:    { label: 'Pedestrian', use: 'caution', life: 30 },
    NO_HELMET:     { label: 'No helmet', use: 'report', life: 600 },
    TRIPLE_RIDING: { label: 'Triple riding', use: 'report', life: 600 },
    RED_LIGHT:     { label: 'Red light jumped', use: 'report', life: 600 },
  };

  /** Settings shared by both worlds, so the twin sees the same demand and incidents. */
  const cfg = { demand: 4200, appShare: 0.25, reach: 80, jaywalk: 1, pothole: true, busStops: true, heedSlow: true, heedHazard: true, heedBraking: true };

  const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));
  const kmh = (mps) => Math.round(mps * 3.6);
  const overlap = (y, wid, y0, y1) => y + wid / 2 > y0 && y - wid / 2 < y1;

  // Mulberry32; state is a plain object so a world can be cloned mid-run.
  function rng(seed) { return { a: seed >>> 0 }; }
  function rnd(s) {
    s.a = (s.a + 0x6D2B79F5) | 0; let t = s.a;
    t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  }

  function signalPhase(t) {
    const c = (t + 25) % CYCLE;
    if (c < SIGNAL.green) return { state: 'green', left: SIGNAL.green - c };
    if (c < SIGNAL.green + SIGNAL.amber) return { state: 'amber', left: SIGNAL.green + SIGNAL.amber - c };
    return { state: 'red', left: CYCLE - c };
  }

  function freshStats(t) { return { t0: t, hb: 0, nm: 0, exits: 0, ttSum: 0, ttN: 0, stopsSum: 0, spdSum: 0, spdN: 0, delay: 0, contacts: 0 }; }
  function freshMeshStats() {
    return { orig: 0, refresh: 0, relays: 0, tx: 0, rx: 0, dup: 0, suppressed: 0, stale: 0, implausible: 0, rate: 0, expired: 0,
      warnings: 0, leadSum: 0, leadN: 0, reports: 0, queries: 0, qFrames: 0 };
  }

  function createWorld(seed) {
    const w = {
      t: 0, tm: 0, seed, ents: [], byId: new Map(), nextId: 1, crashes: [], crashSeq: 0,
      spawnRng: rng(seed ^ 0x9e3779b9), envRng: rng(seed ^ 0x85ebca6b), meshRng: rng(seed ^ 0xc2b2ae35),
      nextArrival: 0, nextPed: 20, entry: [], mesh: false, heap: [], recentTx: [], rings: [], links: [], app: [], heavy: [],
      st: freshStats(0), ms: freshMeshStats(), msgs: new Map(), reports: [], jumps: [], reported: new Set(),
      hist: [], queueHist: [], lastSample: 0, meanKmh: 0, queueM: 0, count: 0, query: null, qseq: 0,
    };
    scheduleArrival(w);
    return w;
  }

  /** An exact copy that evolves on its own from here; both copies draw the same arrivals and driver noise. */
  function fork(w) {
    const c = structuredClone({ ...w, heap: [], rings: [], links: [], recentTx: [] });
    c.mesh = false; c.query = null;
    return c;
  }
  function resetStats(w) { w.st = freshStats(w.t); w.ms = freshMeshStats(); w.msgs.clear(); w.reports = []; }

  function setMesh(w, on) {
    w.mesh = on;
    if (!on) {
      w.heap = []; w.rings = []; w.links = []; w.recentTx = []; w.query = null;
      for (const e of w.ents) if (e.kind === 'veh') { e.m = null; e.know = []; e.heeding = null; e.advice = null; }
    }
  }
  const isApp = (e) => e.kind === 'veh' && e.uApp < cfg.appShare;

  // ---------- arrivals ----------
  function scheduleArrival(w) { w.nextArrival += -Math.log(1 - rnd(w.spawnRng)) * 3600 / Math.max(300, cfg.demand); }
  function makeSpec(w) {
    const s = w.spawnRng;
    const r1 = rnd(s), seed = (rnd(s) * 4294967296) >>> 0, uApp = rnd(s);
    const u = [rnd(s), rnd(s), rnd(s), rnd(s), rnd(s), rnd(s)];
    let acc = 0, type = 'car';
    for (const [t, p] of MIX) { acc += p; if (r1 < acc) { type = t; break; } }
    return { type, seed, uApp, u };
  }
  function makeVehicle(w, sp, x, y, v) {
    const T = TYPES[sp.type], rs = rng(sp.seed), u = sp.u;
    const f = 0.88 + 0.24 * rnd(rs);
    return {
      kind: 'veh', id: w.nextId++, type: sp.type, len: T.len, wid: T.wid, x, y, ty: y, v, acc: 0, held: 0,
      v0: T.v0 * f, a: T.a * (0.9 + 0.2 * rnd(rs)), b: T.b, T: T.T * (0.85 + 0.3 * rnd(rs)), s0: T.s0,
      nextAct: w.t + rnd(rs) * 0.5, nextLat: w.t + rnd(rs) * T.latEvery, rs, uApp: sp.uApp, t0: w.t,
      stops: 0, moving: true, hb: false, nm: false, idx: 0, dwell: 0,
      noHelmet: sp.type === 'bike' && u[0] < 0.12, triple: sp.type === 'bike' && u[1] < 0.04,
      jumper: sp.type === 'bike' && u[2] < 0.06, jumping: false,
      busStop: sp.type === 'bus' && cfg.busStops && u[3] < 0.9 ? BUS_STOP_X : null,
      pickupX: sp.type === 'auto' && u[3] < 0.12 ? 140 + u[4] * 860 : null,
      shade: sp.seed % 5, know: [], m: null, heeding: null, advice: null,
    };
  }
  function tryPlace(w, sp) {
    const T = TYPES[sp.type], half = T.wid / 2 + 0.15;
    const pref = half + sp.u[5] * (W - 2 * half);
    const cands = [pref];
    for (let k = 1; k <= 14; k++) cands.push(pref + k * 0.8, pref - k * 0.8);
    for (const y of cands) {
      if (y < half || y > W - half) continue;
      let gap = Infinity, vl = 0;
      for (const o of w.ents) {
        if (o.x > 170 || o.kind === 'ped') continue;
        if (Math.abs(o.y - y) < (o.wid + T.wid) / 2 + 0.2) { const g = o.x - o.len - T.len; if (g < gap) { gap = g; vl = o.v; } }
      }
      if (gap < T.s0 + 1.5) continue;
      const v = gap === Infinity ? T.v0 * 0.8 : Math.min(T.v0 * 0.8, vl + Math.max(0, (gap - T.s0) / (T.T * 1.5)));
      const e = makeVehicle(w, sp, T.len, y, v);
      w.ents.push(e); w.byId.set(e.id, e);
      return true;
    }
    return false;
  }
  function arrivals(w) {
    while (w.nextArrival <= w.t) { w.entry.push(makeSpec(w)); scheduleArrival(w); }
    let placed = 0;
    while (w.entry.length && placed < 6 && tryPlace(w, w.entry[0])) { w.entry.shift(); placed++; }
  }

  // ---------- pedestrians ----------
  function pedestrians(w) {
    if (cfg.jaywalk <= 0) { w.nextPed = w.t + 5; return; }
    while (w.nextPed <= w.t) {
      const r = w.envRng;
      let x = 100 + rnd(r) * 950; const fromKerb = rnd(r) < 0.6; const sp = 1.0 + rnd(r) * 0.5;
      if (x > POT.x0 - 15 && x < POT.x1 + 10) x -= 40;
      const e = { kind: 'ped', id: w.nextId++, x, len: 0.5, wid: 0.6, y: fromKerb ? -0.5 : W + 0.5, ty: fromKerb ? W + 0.6 : -0.6, v: 0, sp, waited: 0, idx: 0, gone: false };
      w.ents.push(e); w.byId.set(e.id, e);
      w.nextPed += -Math.log(1 - rnd(r)) * 60 / cfg.jaywalk;
    }
  }
  function movePed(w, p) {
    const dir = Math.sign(p.ty - p.y), ny = p.y + dir * p.sp * DT;
    const ents = w.ents;
    let blocked = false;
    for (let j = p.idx - 1; j >= 0 && !blocked; j--) {
      const o = ents[j]; if (o.x < p.x - 30) break;
      if (o.kind !== 'veh' && o.kind !== 'wreck') continue;
      const lat = Math.abs(ny - o.y) < o.wid / 2 + 0.4;
      if (!lat) continue;
      if (o.x - o.len - 0.3 < p.x && o.x + 0.3 > p.x) blocked = true;                // standing on the spot
      else if (o.x < p.x && p.x - o.x < Math.max(3.5, o.v * 1.2) && o.v > 1) blocked = true; // too close to step out
    }
    for (let j = p.idx + 1; j < ents.length && !blocked; j++) {
      const o = ents[j]; if (o.x - 13 > p.x) break;
      if (o.kind !== 'veh' && o.kind !== 'wreck') continue;
      if (Math.abs(ny - o.y) < o.wid / 2 + 0.4 && o.x - o.len - 0.3 < p.x && o.x + 0.3 > p.x) blocked = true;
    }
    if (blocked) p.waited += DT; else p.y = ny;
    if ((dir > 0 ? p.y >= p.ty : p.y <= p.ty) || p.waited > 45) p.gone = true;
  }

  // ---------- perception and car-following ----------
  function leaderIn(w, v, y, wid, lim) {
    const ents = w.ents; let best = null, bestGap = Infinity;
    const margin = v.type === 'bike' ? 0.15 : 0.3;
    for (let j = v.idx + 1; j < ents.length; j++) {
      const o = ents[j];
      if (o.x - v.x > lim + 12.5) break;
      const g = o.x - o.len - v.x;
      if (g < -1.5 || g > lim || g >= bestGap) continue;
      // Alongside (g < 0.5) only a true overlap counts; a rider squeezed past with a hand's width to spare is not ahead.
      if (Math.abs(o.y - y) < (o.wid + wid) / 2 + (g < 0.5 ? 0.02 : margin)) { best = o; bestGap = Math.max(g, 0.05); }
    }
    return best ? { e: best, gap: bestGap } : null;
  }
  function idm(v, gap, vl) {
    const s = Math.max(gap, 0.05), dv = v.v - vl;
    const sStar = v.s0 + Math.max(0, v.v * v.T + v.v * dv / (2 * Math.sqrt(v.a * v.b)));
    return v.a * (1 - (sStar / s) ** 2);
  }
  /** Improved IDM with the constant-acceleration heuristic (Kesting, Treiber, Helbing 2010): no panic braking at cut-ins. */
  function follow(v, gap, vl, al) {
    const aIdm = idm(v, gap, vl), s = Math.max(gap, 0.05), at = Math.min(al, v.a);
    let aCah;
    const den = vl * vl - 2 * s * at;
    if (vl * (v.v - vl) <= -2 * s * at && den > 1e-6) aCah = v.v * v.v * at / den;
    else aCah = at - (v.v > vl ? (v.v - vl) ** 2 : 0) / (2 * s);
    if (aIdm >= aCah) return aIdm;
    return 0.01 * aIdm + 0.99 * (aCah + v.b * Math.tanh((aIdm - aCah) / v.b));
  }
  function freeAcc(v, v0) {
    v0 = Math.max(v0, 0.5);
    // Above the wanted speed a driver lifts off and coasts down (about 1 m/s2); following distance is handled separately.
    return v.v <= v0 ? v.a * (1 - (v.v / v0) ** 4) : Math.max(-1.2, (v0 - v.v) / 3);
  }
  function sightOf(v, lead) {
    const T = TYPES[v.type];
    if (!lead || lead.gap > 15 || lead.e.kind !== 'veh' || lead.e.type === 'bike') return T.sight;
    if (T.heavy && !TYPES[lead.e.type].heavy) return T.sight; // a bus driver sits above the cars
    return Math.min(T.sight, lead.gap + 5);
  }
  function knows(v, kind, x, within) { for (const k of v.know) if (k.kind === kind && Math.abs(k.x - x) < within) return true; return false; }
  function potholeAcc(w, v, sight) {
    if (!cfg.pothole || !overlap(v.y, v.wid, POT.y0, POT.y1)) return Infinity;
    const d = POT.x0 - v.x;
    if (d < -(POT.x1 - POT.x0) - v.len) return Infinity;
    const range = w.mesh && isApp(v) && cfg.heedHazard && knows(v, 'POTHOLE', POT.x0, 30) ? 200 : sight;
    if (d > range) return Infinity;
    const vcap = v.type === 'bike' ? 3.2 : v.type === 'auto' ? 3.5 : v.type === 'car' ? 4.2 : 5;
    if (v.v <= vcap) return Infinity;
    if (d > 0) return Math.max(-8, (vcap * vcap - v.v * v.v) / (2 * Math.max(d, 0.5)));
    return Math.max(-8, -(v.v - vcap) / 0.6);
  }

  /** What a driver with the app does with what it has heard (see the notes in the page). */
  function heed(w, v, v0) {
    v.heeding = null;
    if (!w.mesh || !isApp(v)) return v0;
    const tm = w.tm;
    for (const k of v.know) {
      if (!k.heard && !k.own) continue;
      const D = k.x - v.x;
      if (k.kind === 'SLOWDOWN' || k.kind === 'BRAKING') {
        if (cfg.heedSlow && (cfg.heedBraking || k.kind === 'SLOWDOWN') && k.target != null && D > 0 && tm < k.until && k.target < v0) { v0 = k.target; v.heeding = k; }
      } else if (!cfg.heedHazard) continue;
      else if (k.kind === 'POTHOLE') {
        if (D > -2 && D < 80 && overlap(v.y, v.wid, POT.y0 - 0.5, POT.y1 + 0.5) && v0 > 5.5) { v0 = 5.5; v.heeding = k; }
        else if (D > 0 && D < 200 && !v.heeding) v.heeding = k;
      } else if (k.kind === 'ACCIDENT') {
        if (D > -5 && D < 120 && v0 > 6.9) { v0 = 6.9; v.heeding = k; }
        else if (D > 0 && D < 250 && !v.heeding) v.heeding = k;
      } else if (k.kind === 'PEDESTRIAN') {
        if (D > 0 && D < 60 && tm - k.last < 15000 && v0 > 6.9) { v0 = 6.9; v.heeding = k; }
      }
    }
    if (v.advice && tm < v.advice.until && v.advice.target < v0) v0 = v.advice.target;
    return v0;
  }

  function hazardPenalty(w, v, yc, sight) {
    let p = 0;
    const app = w.mesh && isApp(v) && cfg.heedHazard;
    if (cfg.pothole) {
      const d = POT.x0 - v.x;
      if (d > -1 && overlap(yc, v.wid, POT.y0, POT.y1)) {
        const range = app && knows(v, 'POTHOLE', POT.x0, 30) ? 200 : sight;
        if (d < range) p += 30;
      }
    }
    for (const c of w.crashes) {
      const d = c.x - v.x;
      if (d > -1 && overlap(yc, v.wid, c.y0, c.y1)) {
        const range = app && knows(v, 'ACCIDENT', c.x, 40) ? 250 : sight + 10;
        if (d < range) p += 45;
      }
    }
    return p;
  }
  function kerbBonus(v, yc) {
    const target = v.dwell > 0 ? null : v.busStop ?? v.pickupX;
    if (target == null) return 0;
    const d = target - v.x;
    if (d < 0 || d > (v.type === 'bus' ? 150 : 80)) return 0;
    return 25 * (1 - Math.min(1, (yc - v.wid / 2 - 0.2) / 3));
  }
  function sideFree(w, v, ny) {
    const ents = w.ents, half = v.wid / 2;
    for (let j = v.idx + 1; j < ents.length; j++) {
      const o = ents[j]; if (o.x > v.x + 13) break;
      if (o.x - o.len < v.x + 0.3 && Math.abs(ny - o.y) < half + o.wid / 2 + 0.1 && Math.abs(v.y - o.y) >= Math.abs(ny - o.y) - 1e-9) return false;
    }
    for (let j = v.idx - 1; j >= 0; j--) {
      const o = ents[j]; if (o.x < v.x - v.len - 0.3) break;
      if (Math.abs(ny - o.y) < half + o.wid / 2 + 0.1 && Math.abs(v.y - o.y) >= Math.abs(ny - o.y) - 1e-9) return false;
    }
    return true;
  }
  function followerOk(w, v, yc) {
    const ents = w.ents;
    for (let j = v.idx - 1; j >= 0; j--) {
      const f = ents[j]; const g = v.x - v.len - f.x;
      if (g > 40) break;
      if (f.kind !== 'veh' || g < -0.5) continue;
      if (Math.abs(f.y - yc) < (f.wid + v.wid) / 2 + 0.2) {
        if (g < 0.6) return false;
        return follow(f, g, v.v, v.acc) > (v.type === 'bike' ? -2.5 : -2);
      }
    }
    return true;
  }
  function chooseLateral(w, v, sight) {
    const half = v.wid / 2 + 0.15, lo = half, hi = W - half;
    const steps = v.type === 'bike' ? [-2.4, -1.4, -0.8, 0, 0.8, 1.4, 2.4] : v.type === 'auto' ? [-2.6, -1.3, 0, 1.3, 2.6] : [-3.5, -1.75, 0, 1.75, 3.5];
    const cost = v.type === 'bike' ? 1.2 : v.type === 'auto' ? 2.2 : 4;
    const look = 30 + v.v * 3;
    let best = v.y, bestScore = -Infinity, cur = -Infinity;
    for (const dy of steps) {
      const yc = clamp(v.y + dy, lo, hi);
      if (dy !== 0 && Math.abs(yc - v.y) < 0.3) continue;
      if (dy !== 0 && (!sideFree(w, v, v.y + Math.sign(dy) * 0.3) || !followerOk(w, v, yc))) continue;
      const ld = leaderIn(w, v, yc, v.wid, look);
      if (dy !== 0 && ld && follow(v, ld.gap, ld.e.v, ld.e.kind === 'veh' ? ld.e.acc : 0) < -1.5) continue;
      const gap = ld ? Math.min(ld.gap, look) : look;
      const vl = ld ? Math.min(ld.e.v, v.v0) : v.v0;
      const s = gap + vl * 1.5 - Math.abs(yc - v.y) * cost - hazardPenalty(w, v, yc, sight) + kerbBonus(v, yc) + rnd(v.rs) * 0.5;
      if (dy === 0) cur = s;
      if (s > bestScore) { bestScore = s; best = yc; }
    }
    v.ty = bestScore > cur + (v.type === 'bike' ? 1.5 : 4) ? best : v.y;
  }

  function decide(w, v) {
    const T = TYPES[v.type];
    if (v.dwell > 0) {
      v.dwell -= DT; v.acc = v.v > 0 ? -v.b : 0; v.held = 0;
      if (v.dwell <= 0) { v.dwell = 0; if (v.busStop != null) v.busStop = null; else v.pickupX = null; v.nextLat = w.t; }
      return;
    }
    const lead = leaderIn(w, v, v.y, v.wid, 25 + v.v * 4);
    const sight = sightOf(v, lead);
    let v0 = v.v0;
    for (const c of w.crashes) if (v.x > c.x - 60 && v.x < c.x + 25) v0 = Math.min(v0, 5); // everyone slows to look
    v0 = heed(w, v, v0);
    let acc = freeAcc(v, v0);
    if (lead) acc = Math.min(acc, follow(v, lead.gap, lead.e.v, lead.e.kind === 'veh' ? lead.e.acc : 0));
    if (v.x < STOP_X - 0.2 && !v.jumping) {
      const ph = signalPhase(w.t);
      if (ph.state !== 'green') {
        const gap = STOP_X - v.x - 0.5;
        if (ph.state === 'red' && v.jumper && gap < 20 && v.v > 1.5) v.jumping = true;
        else if (ph.state === 'red' || v.v * v.v / (2 * Math.max(gap, 0.1)) < 2.5) acc = Math.min(acc, idm(v, Math.max(gap, 0.1), 0));
      }
    }
    const stopAt = v.busStop ?? v.pickupX;
    if (stopAt != null && v.x < stopAt + 0.5 && stopAt - v.x < 200) {
      const gap = stopAt - v.x;
      // Pulls in only if it reached the kerb side; otherwise drives past the stop.
      if (gap < 25 && v.y - v.wid / 2 > 3.2) { if (v.busStop != null) v.busStop = null; else v.pickupX = null; }
      else if (gap < 1 && v.v < 0.4) { v.dwell = v.type === 'bus' ? 15 + rnd(v.rs) * 25 : 5 + rnd(v.rs) * 7; v.acc = 0; v.held = 0; return; }
      else acc = Math.min(acc, idm(v, Math.max(gap, 0) + v.s0, 0));
    }
    acc = Math.min(acc, potholeAcc(w, v, sight));
    // Reaction: a driver acts at their own action points, except when the situation suddenly needs much more braking.
    if (w.t >= v.nextAct) {
      v.nextAct = w.t + T.tau[0] + rnd(v.rs) * (T.tau[1] - T.tau[0]);
      if (rnd(v.rs) < 0.012) acc -= 0.3 + rnd(v.rs) * 1.0; // a glance at the phone, a honk, a pothole guess
      v.held = acc;
    } else if (acc < v.held - 2) v.held = acc;
    v.acc = Math.max(-9, v.held);
    if (w.t >= v.nextLat || (lead && lead.gap < 4 && v.v < 1.5 && w.t >= v.nextLat - T.latEvery * 0.6)) {
      chooseLateral(w, v, sight);
      v.nextLat = w.t + T.latEvery * (0.7 + 0.6 * rnd(v.rs));
    }
    const hb = v.acc < -3.5 && v.v > 1.5;
    if (hb && !v.hb) w.st.hb++;
    v.hb = hb;
    if (lead) {
      const closing = v.v - lead.e.v, nm = closing > 0.5 && v.v > 2 && lead.gap / closing < 1.5;
      if (nm && !v.nm) w.st.nm++;
      v.nm = nm;
    } else v.nm = false;
    v.lead = lead ? lead.e.id : 0; v.leadGap = lead ? lead.gap : Infinity; v.sight = sight;
  }

  function integrate(w, v) {
    const T = TYPES[v.type];
    const vn = Math.max(0, v.v + v.acc * DT);
    v.x += (v.v + vn) / 2 * DT; v.v = vn;
    if (v.v < 0.3 && v.moving) { v.stops++; v.moving = false; } else if (v.v > 3) v.moving = true;
    if (Math.abs(v.ty - v.y) > 0.02) {
      const sp = T.lat * Math.min(1, 0.25 + v.v / 4);
      const ny = v.y + Math.sign(v.ty - v.y) * Math.min(sp * DT, Math.abs(v.ty - v.y));
      if (sideFree(w, v, ny)) v.y = ny; else v.ty = v.y;
    }
  }

  function sortEnts(w) {
    const a = w.ents;
    for (let i = 1; i < a.length; i++) {
      const e = a[i]; let j = i - 1;
      while (j >= 0 && a[j].x > e.x) { a[j + 1] = a[j]; j--; }
      a[j + 1] = e;
    }
    for (let i = 0; i < a.length; i++) a[i].idx = i;
  }
  function resolveOverlaps(w) {
    const a = w.ents;
    for (let i = a.length - 1; i >= 0; i--) {
      const v = a[i]; if (v.kind !== 'veh') continue;
      for (let j = i + 1; j < a.length; j++) {
        const o = a[j]; if (o.x - v.x > 13) break;
        if (o.kind === 'ped') continue;
        if (o.x - o.len < v.x + 0.05 && Math.abs(o.y - v.y) < (o.wid + v.wid) / 2 - 0.05) {
          v.x = o.x - o.len - 0.1; v.v = Math.min(v.v, o.v); w.st.contacts++;
        }
      }
    }
  }

  // ---------- one step ----------
  function step(w) {
    w.t = Math.round((w.t + DT) * 10) / 10; w.tm = Math.round(w.t * 1000);
    arrivals(w); pedestrians(w);
    sortEnts(w);
    for (const e of w.ents) if (e.kind === 'veh') decide(w, e);
    for (const e of w.ents) if (e.kind === 'veh') integrate(w, e); else if (e.kind === 'ped') movePed(w, e);
    sortEnts(w);
    resolveOverlaps(w);
    let k = 0;
    for (const e of w.ents) {
      if (e.kind === 'veh' && e.jumping && e.x > STOP_X && !e.jumpLogged) { e.jumpLogged = true; w.jumps.push({ id: e.id, x: STOP_X, t: w.tm }); }
      if ((e.kind === 'veh' && e.x > L + 1) || (e.kind === 'ped' && e.gone)) {
        w.byId.delete(e.id);
        if (e.kind === 'veh') {
          w.st.exits++;
          if (e.t0 >= w.st.t0) { w.st.ttSum += w.t - e.t0; w.st.ttN++; w.st.stopsSum += e.stops; }
        }
        continue;
      }
      w.ents[k++] = e;
    }
    w.ents.length = k;
    for (let i = 0; i < k; i++) w.ents[i].idx = i;
    w.jumps = w.jumps.filter((j) => w.tm - j.t < 3000);
    if (w.mesh) meshStep(w);
    if (w.t - w.lastSample >= 1 - 1e-6) { w.lastSample = w.t; sample(w); }
  }

  function sample(w) {
    const sum = new Float32Array(CELLS), cnt = new Uint16Array(CELLS);
    let s = 0, n = 0, delay = w.entry.length, count = 0;
    for (const e of w.ents) {
      if (e.kind !== 'veh') continue;
      count++;
      const c = clamp(Math.floor((e.x - e.len / 2) / CELL), 0, CELLS - 1);
      sum[c] += e.v; cnt[c]++;
      if (e.x < STOP_X) { s += e.v; n++; }
      delay += Math.max(0, 1 - e.v / e.v0);
    }
    const col = new Float32Array(CELLS);
    for (let c = 0; c < CELLS; c++) col[c] = cnt[c] ? sum[c] / cnt[c] : -1;
    w.hist.push(col); if (w.hist.length > HIST) w.hist.shift();
    w.meanKmh = n ? s / n * 3.6 : 0; w.count = count;
    w.st.spdSum += w.meanKmh; w.st.spdN++; w.st.delay += delay / 3600;
    let q = 0;
    for (let c = Math.floor(STOP_X / CELL) - 1; c >= 0; c--) {
      if (cnt[c] && sum[c] / cnt[c] < 2) q = STOP_X - c * CELL; else break;
    }
    // Longest standing queue behind the stop line over the last signal cycle.
    w.queueHist.push(q); if (w.queueHist.length > CYCLE) w.queueHist.shift();
    w.queueM = Math.max(...w.queueHist);
  }

  // ---------- incidents (applied to both worlds by the page) ----------
  function placeCrash(w, x, y, id) {
    x = clamp(x, 40, STOP_X - 30); y = clamp(y, 1.3, W - 2.6);
    const parts = [
      { kind: 'wreck', type: 'car', id: w.nextId++, x, y, len: 4.3, wid: 1.8, v: 0, idx: 0 },
      { kind: 'wreck', type: 'bike', id: w.nextId++, x: x - 1.2, y: y + 1.55, len: 1.9, wid: 0.8, v: 0, idx: 0 },
    ];
    for (const e of w.ents) {
      if (e.kind === 'veh' && parts.some((p) => e.x > p.x - p.len - 0.3 && e.x - e.len < p.x + 0.3 && Math.abs(e.y - p.y) < (e.wid + p.wid) / 2 + 0.1)) {
        e.kind = 'wreck'; e.v = 0; e.acc = 0;
      }
    }
    for (const p of parts) { w.ents.push(p); w.byId.set(p.id, p); }
    w.ents = w.ents.filter((e) => !(e.kind === 'ped' && Math.abs(e.x - x) < 6));
    w.crashes.push({ id, x: x - 1, y0: y - 1.3, y1: y + 2.3, y: y + 0.5, t: w.tm });
    sortEnts(w);
  }
  function clearCrashes(w) {
    for (const e of w.ents) if (e.kind === 'wreck') w.byId.delete(e.id);
    w.ents = w.ents.filter((e) => e.kind !== 'wreck');
    w.crashes = [];
    for (const e of w.ents) if (e.kind === 'veh') e.know = e.know.filter((k) => k.kind !== 'ACCIDENT');
    sortEnts(w);
  }

  // ---------- RESILANE mesh ----------
  function hpush(h, e) {
    h.push(e); let i = h.length - 1;
    while (i > 0) { const p = (i - 1) >> 1; if (h[p].t <= e.t) break; h[i] = h[p]; i = p; }
    h[i] = e;
  }
  function hpop(h) {
    const top = h[0], last = h.pop(), n = h.length;
    if (n) {
      let i = 0;
      for (;;) {
        const l = 2 * i + 1; if (l >= n) break;
        let c = l; if (l + 1 < n && h[l + 1].t < h[l].t) c = l + 1;
        if (h[c].t >= last.t) break;
        h[i] = h[c]; i = c;
      }
      h[i] = last;
    }
    return top;
  }
  function mstate(v) {
    return v.m || (v.m = {
      seen: new Map(), pending: new Map(), relayWin: -1e12, relayCount: 0, ownWin: -1e12, ownCount: 0,
      oRates: new Map(), nRates: new Map(), seq: 0, qseen: new Map(), saw: new Set(),
      lastBrake: -1e12, lastSlow: -1e12, fastAt: -1e12, housekeep: 0,
    });
  }
  function lowerBound(arr, x) { let lo = 0, hi = arr.length; while (lo < hi) { const m = (lo + hi) >> 1; if (arr[m].x < x) lo = m + 1; else hi = m; } return lo; }

  /** Per-advertising-event reception: falls off around cfg.reach, buses and trucks in the path cut it, busy air cuts it. */
  function eventChance(w, s, r, d, busy) {
    let p = 1 / (1 + Math.exp((d - cfg.reach) / 10));
    const x0 = Math.min(s.x, r.x), x1 = Math.max(s.x, r.x);
    for (let i = lowerBound(w.heavy, x0); i < w.heavy.length; i++) {
      const h = w.heavy[i]; if (h.x - h.len > x1) break;
      if (h === s || h === r) continue;
      const xm = h.x - h.len / 2;
      const yl = s.y + (r.y - s.y) * ((xm - s.x) / ((r.x - s.x) || 1));
      if (Math.abs(yl - h.y) < h.wid / 2 + 0.5) p *= 0.7;
    }
    return p / (1 + 0.04 * busy);
  }
  function transmit(w, s, f, events, spacing, cls) {
    const tm = w.tm;
    w.ms.tx++;
    if (cls !== 'h') w.ms.qFrames++;
    while (w.recentTx.length && tm - w.recentTx[0].t > 100) w.recentTx.shift();
    let busy = 0; for (const r of w.recentTx) if (Math.abs(r.x - s.x) < 150) busy++;
    w.recentTx.push({ t: tm, x: s.x });
    w.rings.push({ x: s.x - s.len / 2, y: s.y, t: tm, cls }); if (w.rings.length > 500) w.rings.splice(0, w.rings.length - 500);
    const app = w.app;
    for (let i = lowerBound(app, s.x - 230); i < app.length; i++) {
      const r = app[i]; if (r.x > s.x + 230) break;
      if (r === s) continue;
      if (cls === 'q' && r.m && r.m.qseen.has(f.id + '|' + f.origin)) continue;
      if (cls === 'r' && r.m && r.m.qseen.has(f.qid + '|' + f.responder)) continue;
      const d = Math.hypot(r.x - s.x, r.y - s.y);
      const p = eventChance(w, s, r, d, busy);
      if (p < 0.002) continue;
      for (let k = 0; k < events; k++) {
        if (rnd(w.meshRng) < p) {
          hpush(w.heap, { t: tm + k * spacing + 2 + Math.floor(rnd(w.meshRng) * 20), type: 'rx', to: r.id, f, sx: s.x - s.len / 2, sy: s.y, cls });
          break;
        }
      }
    }
  }
  function remember(w, v, kind, x, y, reporter, own) {
    const tm = w.tm;
    for (const k of v.know) {
      if (k.kind === kind && Math.abs(k.x - x) <= MESH.mergeM) {
        k.last = tm; k.reporters.add(reporter); if (own) k.own = true;
        return { k, isNew: false };
      }
    }
    const k = { kind, x, y, first: tm, last: tm, reporters: new Set([reporter]), own, sent: own ? tm : -1e12, heard: false };
    v.know.push(k); if (v.know.length > 64) v.know.shift();
    return { k, isNew: true };
  }
  function publish(w, v, kind, range, refresh) {
    const m = mstate(v), tm = w.tm;
    if (tm - m.ownWin >= 1000) { m.ownWin = tm; m.ownCount = 0; }
    if (m.ownCount >= MESH.originPerSecond) { w.ms.rate++; return null; }
    m.ownCount++;
    const f = { id: v.id + ':' + (++m.seq), origin: v.id, kind, ox: v.x, oy: v.y, range: Math.max(0, Math.round(range)), tGen: tm, ttl: MESH.ttl, hop: 0, fwd: v.id };
    m.seen.set(f.id, tm + MESH.dedupMs);
    const { k } = remember(w, v, kind, v.x + f.range, v.y, v.id, true);
    k.sent = tm;
    if (refresh) w.ms.refresh++; else w.ms.orig++;
    if (KINDS[kind].use === 'report') { w.ms.reports++; if (!refresh) w.reports.push({ t: tm, kind, x: Math.round(v.x + f.range), by: v.id, byType: v.type }); }
    w.msgs.set(f.id, { id: f.id, kind, origin: v.id, otype: v.type, t0: tm, hx: v.x + f.range, range: f.range, n: 0, hops: 0, back: 0, last: tm, refresh: !!refresh });
    if (w.msgs.size > 120) w.msgs.delete(w.msgs.keys().next().value);
    transmit(w, v, f, MESH.originEvents, MESH.advMs, 'h');
    return f;
  }

  function receiveHazard(w, r, f, ev) {
    const m = mstate(r), tm = w.tm;
    if (f.origin === r.id) return;
    const age = tm - f.tGen;
    if (age > MESH.maxAgeMs || age < -MESH.futureMs) { w.ms.stale++; return; }
    const d = Math.hypot(f.ox - r.x, f.oy - r.y);
    if (d > (f.hop + 1) * MESH.rangeM * 1.5) { w.ms.implausible++; return; }
    const q = m.pending.get(f.id);
    if (q) {
      if (f.hop > q.f.hop && f.fwd !== r.id) q.heard.add(f.fwd);
      if (q.heard.size >= MESH.suppression) { m.pending.delete(f.id); w.ms.suppressed++; return; }
    }
    if (m.seen.has(f.id)) { w.ms.dup++; return; }
    const o = m.oRates.get(f.origin) || { win: -1e12, n: 0 }; m.oRates.set(f.origin, o);
    if (tm - o.win >= 1000) { o.win = tm; o.n = 0; }
    if (++o.n > MESH.originPerSecond) { w.ms.rate++; return; }
    const nb = m.nRates.get(f.fwd) || { win: -1e12, n: 0 }; m.nRates.set(f.fwd, nb);
    if (tm - nb.win >= 1000) { nb.win = tm; nb.n = 0; }
    if (++nb.n > MESH.neighborPerSecond) { w.ms.rate++; return; }
    m.seen.set(f.id, tm + MESH.dedupMs);
    w.ms.rx++;
    const hx = f.ox + f.range;
    const t = w.msgs.get(f.id);
    if (t) { t.n++; t.hops = Math.max(t.hops, f.hop + 1); t.back = Math.max(t.back, t.hx - r.x); t.last = tm; }
    w.links.push({ x1: ev.sx, y1: ev.sy, x2: r.x - r.len / 2, y2: r.y, t: tm, cls: 'h' }); if (w.links.length > 600) w.links.splice(0, w.links.length - 600);
    const { k, isNew } = remember(w, r, f.kind, hx, f.oy, f.origin, false);
    const relevant = d <= MESH.relevanceM && (d < 5 || Math.abs(Math.atan2(f.oy - r.y, f.ox - r.x)) * 180 / Math.PI <= 30);
    if (KINDS[f.kind].use === 'caution' && relevant) {
      if (!k.heard) {
        w.ms.warnings++;
        if ((f.kind === 'POTHOLE' || f.kind === 'ACCIDENT') && hx > r.x && isNew) { w.ms.leadSum += hx - r.x; w.ms.leadN++; }
      }
      k.heard = true; k.last = tm;
      // The spoken advice (RoadPicture.forecast): ease off to v*D/(D + 4.5 m/s * 20 s), never below 15 km/h, held for 30 s.
      if ((f.kind === 'SLOWDOWN' || f.kind === 'BRAKING') && hx > r.x && r.v > 4.2 + 0.8) {
        const D = hx - r.x;
        k.target = clamp(r.v * D / (D + 90), 4.2, r.v); k.until = tm + 30000;
      }
    }
    if (f.ttl > 0) {
      const delay = Math.floor(MESH.maxRelayDelayMs * (1 - Math.min(1, d / MESH.rangeM))) + Math.floor(rnd(w.meshRng) * (MESH.jitterMs + 1));
      m.pending.set(f.id, { f, due: tm + delay, expires: tm + (MESH.maxAgeMs - Math.max(0, age)), heard: new Set() });
      hpush(w.heap, { t: tm + delay, type: 'relay', node: r.id, id: f.id });
    }
  }
  function relay(w, ev) {
    const r = w.byId.get(ev.node); if (!r || !isApp(r) || !r.m) return;
    const m = r.m, q = m.pending.get(ev.id);
    if (!q || q.due !== ev.t) return;
    m.pending.delete(ev.id);
    if (w.tm > q.expires) { w.ms.expired++; return; }
    if (w.tm - m.relayWin >= 1000) { m.relayWin = w.tm; m.relayCount = 0; }
    if (m.relayCount >= MESH.relayPerSecond) { w.ms.rate++; return; }
    m.relayCount++;
    w.ms.relays++;
    transmit(w, r, { ...q.f, fwd: r.id, ttl: q.f.ttl - 1, hop: q.f.hop + 1 }, MESH.relayEvents, MESH.advMs, 'h');
  }

  // Road queries: flood both ways, everyone answers once, answers flood back (RoadQueries.kt).
  function records(w, r) {
    return r.know.filter((k) => Math.abs(k.x - r.x) <= 2000)
      .sort((a, b) => (KINDS[a.kind].use !== 'caution') - (KINDS[b.kind].use !== 'caution') || Math.abs(a.x - r.x) - Math.abs(b.x - r.x))
      .slice(0, 8).map((k) => ({ kind: k.kind, dx: Math.round(k.x - r.x), age: Math.round((w.tm - k.last) / 1000), rep: k.reporters.size }));
  }
  function ask(w, id) {
    const v = w.byId.get(id);
    if (!w.mesh || !v || !isApp(v)) return false;
    const m = mstate(v);
    const f = { id: 'q' + v.id + ':' + (++w.qseq), origin: v.id, fwd: v.id, ttl: MESH.ttl, hop: 0, x: v.x, y: v.y };
    m.qseen.set(f.id + '|' + v.id, w.tm);
    w.query = { id: f.id, asker: v.id, sent: w.tm, replies: new Map(), done: false, frames0: w.ms.qFrames, picture: null };
    w.ms.queries++;
    transmit(w, v, f, MESH.ctrlEvents, MESH.ctrlMs, 'q');
    return true;
  }
  function receiveControl(w, r, f, ev) {
    const m = mstate(r), tm = w.tm;
    if (ev.cls === 'q') {
      const key = f.id + '|' + f.origin;
      if (f.origin === r.id || m.qseen.has(key)) return;
      m.qseen.set(key, tm);
      w.links.push({ x1: ev.sx, y1: ev.sy, x2: r.x - r.len / 2, y2: r.y, t: tm, cls: 'q' });
      const reply = { qid: f.id, responder: r.id, fwd: r.id, ttl: MESH.ttl, hop: 0, x: r.x, y: r.y, v: r.v, recs: records(w, r) };
      m.qseen.set(f.id + '|' + r.id, tm);
      transmit(w, r, reply, MESH.ctrlEvents, MESH.ctrlMs, 'r');
      if (f.ttl > 0) transmit(w, r, { ...f, fwd: r.id, ttl: f.ttl - 1, hop: f.hop + 1 }, MESH.ctrlEvents, MESH.ctrlMs, 'q');
    } else {
      const key = f.qid + '|' + f.responder;
      if (f.responder === r.id || m.qseen.has(key)) return;
      m.qseen.set(key, tm);
      const q = w.query;
      if (q && q.id === f.qid && q.asker === r.id) {
        if (!q.done) { q.replies.set(f.responder, f); w.links.push({ x1: ev.sx, y1: ev.sy, x2: r.x - r.len / 2, y2: r.y, t: tm, cls: 'r' }); }
        return;
      }
      if (f.ttl > 0) transmit(w, r, { ...f, fwd: r.id, ttl: f.ttl - 1, hop: f.hop + 1 }, MESH.ctrlEvents, MESH.ctrlMs, 'r');
    }
  }

  /** RoadPicture.forecast, ported. Speeds in m/s, distances in m. */
  function forecast(speed, peers, items) {
    const v = speed > 1 ? speed : null;
    const slowPeers = peers.filter((p) => p.aheadM > 0 && p.speed != null && ((v != null && p.speed < v * 0.5) || p.speed < 3));
    const slowReports = items.filter((i) => i.aheadM > 0 && (i.kind === 'SLOWDOWN' || i.kind === 'BRAKING'));
    const all = slowPeers.map((p) => p.aheadM).concat(slowReports.map((i) => i.aheadM));
    const slowAt = all.length ? Math.min(...all) : null;
    const advice = []; let meet = null, rec = null;
    if (slowAt != null) {
      if (v != null) {
        meet = slowAt / (v + 4.5);
        rec = kmh(clamp(v * slowAt / (slowAt + 4.5 * 20), 4.2, v));
        advice.push(`Traffic is slowing about ${slowAt} m ahead; at your speed you would meet it in about ${Math.round(meet)} s.`);
        if (rec < kmh(v) - 3) advice.push(`Easing off to about ${rec} km/h now lets it clear before you arrive, so you avoid braking hard and don't pass the jam back.`);
      } else advice.push(`Traffic is slowing about ${slowAt} m ahead.`);
    }
    const next = items.filter((i) => i.aheadM >= 0 && i.aheadM <= 1500 && KINDS[i.kind].use === 'caution' && i.kind !== 'SLOWDOWN').sort((a, b) => a.aheadM - b.aheadM)[0];
    if (next) {
      const eta = v ? next.aheadM / v : null;
      const when = eta != null ? ` (about ${Math.round(eta)} s away)` : '';
      advice.push({
        POTHOLE: `Pothole ${next.aheadM} m ahead${when}: slow to about 20 km/h and watch the road surface.`,
        ACCIDENT: `Accident ${next.aheadM} m ahead${when}: slow down early, expect a blocked lane and people on the road.`,
        PEDESTRIAN: `Someone is on the road ${next.aheadM} m ahead${when}: slow down.`,
        BRAKING: `Traffic is braking ${next.aheadM} m ahead${when}: ease off early.`,
      }[next.kind] || `${KINDS[next.kind].label} ${next.aheadM} m ahead${when}.`);
    }
    if (slowAt == null && !next) advice.push('Nothing reported on the road ahead within range.');
    return { slowAt, meet, rec, next: next || null, advice };
  }
  function combine(w, asker, q) {
    const hits = [];
    for (const k of asker.know) hits.push({ kind: k.kind, x: k.x, rep: k.reporters.size, age: Math.round((w.tm - k.last) / 1000), self: true, src: null });
    for (const r of q.replies.values()) for (const rec of r.recs) hits.push({ kind: rec.kind, x: r.x + rec.dx, rep: rec.rep, age: rec.age, self: false, src: r.responder });
    const groups = [];
    for (const h of hits) { const g = groups.find((g) => g[0].kind === h.kind && Math.abs(g[0].x - h.x) <= 30); if (g) g.push(h); else groups.push([h]); }
    const items = groups.map((g) => ({
      kind: g[0].kind, aheadM: Math.round(g[0].x - asker.x), self: g.some((h) => h.self), age: Math.min(...g.map((h) => h.age)),
      reporters: Math.max(Math.max(...g.map((h) => h.rep)), new Set(g.filter((h) => h.src != null).map((h) => h.src)).size + (g.some((h) => h.self) ? 1 : 0)),
    })).sort((a, b) => a.aheadM - b.aheadM);
    const peers = [...q.replies.values()].map((r) => ({ id: r.responder, aheadM: Math.round(r.x - asker.x), speed: r.v, hops: r.hop, recs: r.recs.length }))
      .sort((a, b) => b.aheadM - a.aheadM);
    return { items, peers, fc: forecast(asker.v, peers, items), frames: w.ms.qFrames - q.frames0, at: w.tm, speed: asker.v };
  }

  /** What an app phone's own camera and sensors originate this step. */
  function observe(w, v) {
    const m = mstate(v), tm = w.tm;
    const lead = v.lead ? w.byId.get(v.lead) : null;
    const sight = v.sight || TYPES[v.type].sight;
    if (cfg.pothole && !m.saw.has('pot')) {
      const d = POT.x0 - v.x;
      if (d > 4 && d < Math.min(30, sight) && overlap(v.y, 6, POT.y0, POT.y1)) {
        m.saw.add('pot');
        if (rnd(w.meshRng) < 0.8) publish(w, v, 'POTHOLE', d);
      }
    }
    for (const c of w.crashes) {
      const key = 'c' + c.id; if (m.saw.has(key)) continue;
      const d = c.x - v.x;
      if (d > 2 && d < Math.min(60, sight + 15)) { m.saw.add(key); publish(w, v, 'ACCIDENT', d); }
    }
    if (v.acc < -4 && v.v > 2 && tm - m.lastBrake > 8000) { m.lastBrake = tm; publish(w, v, 'BRAKING', 0); }
    if (v.v > 7) m.fastAt = tm;
    if (v.v < 2.2 && tm - m.fastAt < 10000 && tm - m.lastSlow > 20000) { m.lastSlow = tm; publish(w, v, 'SLOWDOWN', 0); }
    if (lead && lead.kind === 'ped' && v.leadGap < 25 && !m.saw.has('p' + lead.id)) { m.saw.add('p' + lead.id); publish(w, v, 'PEDESTRIAN', v.leadGap); }
    // Violations seen by the dashcam: riders just ahead, and red-light jumps at the junction.
    if (rnd(w.meshRng) < 0.05) {
      for (let j = v.idx + 1; j < w.ents.length; j++) {
        const o = w.ents[j]; const d = o.x - o.len - v.x;
        if (d > 20) break;
        if (o.kind !== 'veh' || d < 3 || Math.abs(o.y - v.y) > 3) continue;
        for (const [flag, kind] of [['noHelmet', 'NO_HELMET'], ['triple', 'TRIPLE_RIDING']]) {
          const key = kind + o.id;
          if (o[flag] && !w.reported.has(key)) { w.reported.add(key); publish(w, v, kind, d); }
        }
      }
    }
    for (const j of w.jumps) {
      const key = 'RED_LIGHT' + j.id;
      if (!w.reported.has(key) && STOP_X - v.x > 0 && STOP_X - v.x < 50) { w.reported.add(key); publish(w, v, 'RED_LIGHT', STOP_X - v.x); }
    }
    // MeshSession refresh: own long-lived reports within 250 m, every 20 s. Only while the hazard is still ahead here;
    // the app also refreshes after passing it, where an unsigned range puts the hazard in front of the sender.
    for (const k of v.know) {
      if (!k.own || KINDS[k.kind].life < 600) continue;
      const d = k.x - v.x;
      if (d > 0 && d < MESH.refreshWithinM && tm - k.sent >= MESH.refreshMs) { k.sent = tm; publish(w, v, k.kind, d, true); }
    }
    if (tm - m.housekeep > 1000) {
      m.housekeep = tm;
      for (const [id, until] of m.seen) if (until <= tm) m.seen.delete(id);
      for (const [id, at] of m.qseen) if (tm - at > 30000) m.qseen.delete(id);
      v.know = v.know.filter((k) => tm - k.last < KINDS[k.kind].life * 1000);
    }
  }

  function meshStep(w) {
    w.app = w.ents.filter(isApp);
    w.heavy = w.ents.filter((e) => e.kind === 'veh' && TYPES[e.type].heavy);
    for (const v of w.app) observe(w, v);
    while (w.heap.length && w.heap[0].t <= w.tm) {
      const ev = hpop(w.heap);
      if (ev.type === 'relay') { relay(w, ev); continue; }
      const r = w.byId.get(ev.to);
      if (!r || !isApp(r)) continue;
      if (ev.cls === 'h') receiveHazard(w, r, ev.f, ev); else receiveControl(w, r, ev.f, ev);
    }
    const q = w.query;
    if (q && !q.done && w.tm - q.sent >= MESH.queryWindowMs) {
      q.done = true;
      const asker = w.byId.get(q.asker);
      if (asker) {
        q.picture = combine(w, asker, q);
        const rec = q.picture.fc.rec;
        asker.advice = rec != null && rec < kmh(asker.v) - 3 ? { target: rec / 3.6, until: w.tm + 30000 } : null;
      }
    }
    const cut = w.tm - 1500;
    if (w.rings.length && w.rings[0].t < cut) w.rings = w.rings.filter((r) => r.t >= cut);
    if (w.links.length && w.links[0].t < cut) w.links = w.links.filter((l) => l.t >= cut);
  }

  function clockText(t) {
    const s = Math.floor(START_CLOCK + t);
    const h = Math.floor(s / 3600) % 24, m = Math.floor(s / 60) % 60, sec = s % 60;
    return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}:${String(sec).padStart(2, '0')}`;
  }

  const api = {
    L, W, STOP_X, DT, CELL, CELLS, HIST, LANE_EDGES, BUS_STOP_X, POT, SIGNAL, TYPES, KINDS, MESH, cfg,
    createWorld, fork, resetStats, setMesh, step, signalPhase, isApp, placeCrash, clearCrashes, ask, forecast, clockText, kmh,
  };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  root.SilkSim = api;
})(typeof window !== 'undefined' ? window : globalThis);
