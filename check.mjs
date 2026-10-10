// Headless run of the Silk Board simulation: node check.mjs [seed] [minutes]
// Checks that a fork without the mesh stays identical, then compares the mesh against its twin.
import { createRequire } from 'node:module';
const require = createRequire(import.meta.url);
const S = require('./sim-core.js');

if (process.env.DEMAND) S.cfg.demand = Number(process.env.DEMAND);
if (process.env.SHARE) S.cfg.appShare = Number(process.env.SHARE);
if (process.env.HEED) { const [a, b] = process.env.HEED.split(','); S.cfg.heedSlow = a !== '0'; S.cfg.heedBraking = a === '1'; S.cfg.heedHazard = b === '1'; }
const seed = Number(process.argv[2] ?? 7), minutes = Number(process.argv[3] ?? 10);
const run = (w, s) => { for (let i = 0; i < s / S.DT; i++) S.step(w); };
const sig = (w) => w.ents.map((e) => `${e.id}:${e.x.toFixed(3)}:${e.y.toFixed(3)}`).join('|');

const started = performance.now();
const main = S.createWorld(seed);
run(main, 240);
console.log(`warm-up 240 s: ${main.count} vehicles, ${main.entry.length} waiting to enter, ${main.meanKmh.toFixed(1)} km/h, queue ${main.queueM} m, ${((performance.now() - started) / 1000).toFixed(1)} s cpu`);

const a = S.fork(main), b = S.fork(main);
run(a, 60); run(b, 60);
if (sig(a) !== sig(b)) { console.error('FAIL: two forks without the mesh diverged'); process.exit(1); }
console.log('ok: forks without the mesh stay identical');

const twin = S.fork(main);
S.resetStats(main); S.resetStats(twin);
S.setMesh(main, true);
const t1 = performance.now();
run(main, minutes * 60); run(twin, minutes * 60);
const cpu = (performance.now() - t1) / 1000;
const row = (w) => {
  const st = w.st, mins = (w.t - st.t0) / 60;
  return {
    'avg km/h': (st.spdSum / st.spdN).toFixed(1), 'queue m': w.queueM, 'delay veh-h': st.delay.toFixed(1),
    'hard brakes/min': (st.hb / mins).toFixed(1), 'near misses/min': (st.nm / mins).toFixed(1), 'exits/h': Math.round(st.exits / mins * 60),
    'travel min': st.ttN ? (st.ttSum / st.ttN / 60).toFixed(2) : '-', 'stops/veh': st.ttN ? (st.stopsSum / st.ttN).toFixed(2) : '-',
    'waiting': w.entry.length, contacts: st.contacts,
  };
};
console.table({ 'without mesh': row(twin), 'with mesh': row(main) });
const ms = main.ms;
console.log(`mesh: ${ms.orig} originated, ${ms.refresh} refreshes, ${ms.relays} relays, ${ms.rx} delivered, ${ms.dup} duplicates, ${ms.suppressed} suppressed, ${ms.rate} rate drops, ${ms.stale} stale`);
console.log(`warnings ${ms.warnings}, mean warning distance ${ms.leadN ? Math.round(ms.leadSum / ms.leadN) : '-'} m over ${ms.leadN}, reports ${ms.reports}`);
const app = main.ents.filter(S.isApp);
const q = app.find((v) => v.x > 300 && v.x < 600);
if (q) {
  S.ask(main, q.id); run(main, 3);
  const p = main.query.picture;
  console.log(`query from ${q.type} at ${Math.round(q.x)} m: ${p.peers.length} phones answered, ${p.frames} frames on air`);
  p.fc.advice.forEach((s) => console.log('  ' + s));
}
console.log(`cpu for ${minutes} min x2 worlds: ${cpu.toFixed(1)} s`);
for (const w of [main, twin]) for (const e of w.ents) if (!Number.isFinite(e.x) || !Number.isFinite(e.y) || !Number.isFinite(e.v)) { console.error('FAIL: non-finite state', e); process.exit(1); }
