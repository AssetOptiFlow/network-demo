// main.js — pipeline orchestration + UI wiring + headless selftest mode.

import { RNG } from "./rng.js";
import { Terrain } from "./terrain.js";
import { seedTowns, buildDensityGrid } from "./density.js";
import { sampleCustomers } from "./customers.js";
import { buildRoads, roadDistanceGrid } from "./roads.js";
import { buildNetwork } from "./network.js";
import { buildSubtx } from "./subtx.js";
import {
  computeSaidi, greedyPlace, allCandidates, debugRateExperiment,
  faultScenario, roadVsLine, emptyDevices, setFaultRates, faultRates,
  SWITCH_MIN, REPAIR_MIN,
} from "./reliability.js";
import { runChecks, checkFaultConservation, checkMonotone } from "./checks.js";
import { FEEDER_MIN_CUST } from "./membership.js";
import { ASSUMPTIONS } from "./assumptions.js";
import { Renderer, feederColour, STATUS } from "./render.js";

// ------------------------------------------------------------ generation

// ---- Validation thresholds (named for tuning). A world failing any rule
// is regenerated on a deterministic retry seed; after MAX_ATTEMPTS the
// BEST attempt (fewest failures) is used and the unresolved reasons are
// reported — never a hard fail.
export const VALIDATION = {
  MAX_SUB_CENTROID_KM: 8,    // X: sub farther than this from its load centroid
  MAX_SUBTX_STRAIGHT_KM: 15, // Y: subtx line straight for longer than this
                             //    (scaled with the 100 km map — lines can
                             //    legitimately run long across the plains)
  MIN_GRID_SPREAD_DEG: 15,   // all town grids within this spread = suspicious
  MIN_ZIPF_RATIO: 2.2,       // realised largest/median town size below this = too
                             // uniform (the gradual peri-urban density kernel
                             // spreads big-town mass outward, so realised ratios
                             // sit lower than the pure-Gaussian era's 3.0)
  MAX_ATTEMPTS: 4,
};

// STRICT LAYER ORDER — each layer consumes only earlier layers:
//   terrain → settlements (+ provisional corridors) → roads → load →
//   MEMBERSHIP (customers → feeders → subs, before any routing) →
//   sub siting → feeder routing → subtransmission (visual only).
function generateOnce(params) {
  const t0 = performance.now();
  const rng = new RNG(params.seed);
  const timings = {};
  const mark = (name, since) => { timings[name] = Math.round(performance.now() - since); };

  let t = performance.now();
  const terrain = new Terrain(rng.fork("terrain"));
  mark("terrain", t);

  t = performance.now();
  const { towns, corridors } = seedTowns(terrain, rng.fork("towns"), params.nTowns, params.inlandWeight ?? 0.25, params.nCust);
  mark("settlements", t);

  t = performance.now();
  const { graph, repair } = buildRoads(terrain, towns, corridors, rng.fork("roads"));
  const roadDistM = roadDistanceGrid(terrain, graph);
  mark("roads", t);

  t = performance.now();
  const density = buildDensityGrid(terrain, towns, roadDistM, rng.fork("density"), params.nCust);
  const sampled = sampleCustomers(terrain, density, graph, params.nCust, rng.fork("cust"));
  let customers = sampled.customers;
  const snapStats = sampled.snapStats;
  mark("load", t);

  // MEMBERSHIP-FIRST: buildNetwork decides customers→feeders→subs BEFORE
  // routing, sites the subs from feeder groups, then routes honouring the
  // membership tables (validate/repair loop inside, outcomes reported).
  // Then PRUNE: a feeder carrying fewer than FEEDER_MIN_CUST customers is
  // uneconomic to reticulate — the feeder, its transformers AND its
  // customers are removed and the network is rebuilt from the survivors
  // (repeated: a rebuild can surface new sub-minimum feeders as membership
  // resettles). Rebuilding, rather than surgery on the net structures,
  // keeps every derived table consistent by construction.
  t = performance.now();
  let net = buildNetwork(terrain, graph, customers, towns, density, rng.fork("net"));
  const prune = { customers: 0, txs: 0, feeders: 0, rounds: 0 };
  for (let round = 0; round < 5; round++) {
    const small = new Set(net.feeders.filter(f => f.customers < FEEDER_MIN_CUST).map(f => f.id));
    if (!small.size) break;
    const keep = customers.filter(c => {
      const tx = c.tx >= 0 ? net.txs[c.tx] : null;
      return !tx || !small.has(tx.feeder);
    });
    prune.feeders += small.size;
    prune.txs += net.txs.filter(tx => small.has(tx.feeder)).length;
    prune.customers += customers.length - keep.length;
    prune.rounds++;
    customers = keep;
    net = buildNetwork(terrain, graph, customers, towns, density, rng.fork("net" + round));
  }
  const residualSmall = net.feeders.filter(f => f.customers < FEEDER_MIN_CUST).length;
  const subs = net.subs;
  mark("membership+feeders", t);

  t = performance.now();
  let lx = 0, ly = 0;
  for (const c of customers) { lx += c.x; ly += c.y; }
  const loadCentroid = { x: lx / Math.max(1, customers.length), y: ly / Math.max(1, customers.length) };
  const subtx = buildSubtx(terrain, subs, loadCentroid, roadDistM);
  mark("subtx", t);

  const world = {
    params, terrain, towns, corridors, density, customers, graph, net, subtx,
    roadRepair: repair, snapStats, roadDistM,
  };
  t = performance.now();
  world.prune = prune;
  world.checks = runChecks(world);
  world.checks.push(...(net.extraChecks ?? []));
  world.checks.push({
    name: `Feeder minimum (≥ ${FEEDER_MIN_CUST} customers — smaller feeders pruned)`,
    pass: residualSmall === 0,
    tunable: true,
    detail: `${prune.feeders} feeder(s), ${prune.txs} TX(s), ${prune.customers} customer(s) ` +
      `pruned over ${prune.rounds} rebuild round(s); ${residualSmall} sub-minimum feeder(s) remain; ` +
      `${customers.length.toLocaleString("en-NZ")} of ${params.nCust.toLocaleString("en-NZ")} sampled customers kept`,
  });
  world.roadVsLine = roadVsLine(net, graph);

  // Subtransmission is VISUAL ONLY — asserted, not assumed: rebuilding it
  // must leave SAIDI, the road graph and the feeder structure untouched.
  const saidiBefore = computeSaidi(net, emptyDevices()).overall;
  const edgesBefore = net.treeEdges.length, roadEdgesBefore = graph.edges.length;
  const subtx2 = buildSubtx(terrain, subs, loadCentroid, roadDistM);
  const overlayOk = computeSaidi(net, emptyDevices()).overall === saidiBefore &&
    net.treeEdges.length === edgesBefore && graph.edges.length === roadEdgesBefore &&
    subtx2.lines.length === subtx.lines.length;
  console.assert(overlayOk, "subtransmission changed the numbers");
  world.checks.push({
    name: "Subtransmission is visual-only",
    pass: overlayOk,
    detail: `SAIDI ${saidiBefore.toFixed(2)} min/yr, ${edgesBefore} sections, ` +
      `${roadEdgesBefore} road edges — all unchanged by (re)building ${subtx.lines.length} subtx lines`,
  });
  // Calibration: the device-free-but-fused baseline should sit in a
  // realistic planning band. Real NZ networks report 60–300 min/yr WITH
  // their automation; the baseline (breakers + standard lateral fusing,
  // no automation) belongs somewhat above that. A rule change that blows
  // this band should fail visibly here, not be discovered by feel.
  const CAL_BAND = [150, 500];
  world.checks.push({
    name: `Baseline SAIDI calibration (${CAL_BAND[0]}–${CAL_BAND[1]} min/yr band)`,
    tunable: true,
    pass: saidiBefore >= CAL_BAND[0] && saidiBefore <= CAL_BAND[1],
    detail: `baseline ${saidiBefore.toFixed(0)} min/yr — breakers + standard lateral ` +
      `fusing, no automation; devices placed in the UI cut it from here`,
  });
  world.validation = validateWorld(world);
  mark("checks", t);
  timings.total = Math.round(performance.now() - t0);
  world.timings = timings;
  return world;
}

// ---- validation metrics + rules ------------------------------------------

function validateWorld(world) {
  const V = VALIDATION;
  const failures = [];
  // X: every sub near its own load centroid
  const subCentroidKm = Math.max(0, ...world.net.subs.map(s =>
    Math.hypot(s.x - s.centroidX, s.y - s.centroidY) / 1000));
  if (subCentroidKm > V.MAX_SUB_CENTROID_KM) {
    failures.push(`sub ${subCentroidKm.toFixed(1)} km from its load centroid (max ${V.MAX_SUB_CENTROID_KM})`);
  }
  // Y: no long-ruler subtransmission
  const subtxStraightKm = world.subtx.maxStraightKm;
  if (subtxStraightKm > V.MAX_SUBTX_STRAIGHT_KM) {
    failures.push(`subtx straight for ${subtxStraightKm} km (max ${V.MAX_SUBTX_STRAIGHT_KM})`);
  }
  // grid orientations must not all agree (mod 90°, needs ≥3 towns)
  let gridSpreadDeg = 90;
  if (world.towns.length >= 3) {
    gridSpreadDeg = 0;
    for (let i = 0; i < world.towns.length; i++) {
      for (let j = i + 1; j < world.towns.length; j++) {
        const a = (world.towns[i].theta * 180 / Math.PI) % 90;
        const b = (world.towns[j].theta * 180 / Math.PI) % 90;
        const d = Math.abs(a - b);
        gridSpreadDeg = Math.max(gridSpreadDeg, Math.min(d, 90 - d));
      }
    }
    if (gridSpreadDeg < V.MIN_GRID_SPREAD_DEG) {
      failures.push(`all town grids within ${gridSpreadDeg.toFixed(0)}° (min spread ${V.MIN_GRID_SPREAD_DEG}°)`);
    }
  }
  // realised town sizes must be genuinely Zipf-ish, not near-uniform
  const counts = world.towns.map(t => 0);
  for (const c of world.customers) {
    let bi = -1, bd = Infinity;
    for (let i = 0; i < world.towns.length; i++) {
      const t = world.towns[i];
      const d = Math.hypot(c.x - t.x, c.y - t.y);
      if (d < bd) { bd = d; bi = i; }
    }
    if (bi >= 0 && bd <= world.towns[bi].sigma * 3.2) counts[bi]++;
  }
  const sorted = counts.slice().sort((a, b) => b - a);
  const median = sorted[Math.floor(sorted.length / 2)] || 1;
  const zipfRatio = sorted[0] / Math.max(1, median);
  const zipfTopRatio = sorted[0] / Math.max(1, sorted[1] ?? 1);
  if (world.towns.length >= 3 && zipfRatio < V.MIN_ZIPF_RATIO) {
    failures.push(`town sizes near-uniform: largest/median ${zipfRatio.toFixed(2)} (min ${V.MIN_ZIPF_RATIO})`);
  }
  return {
    pass: failures.length === 0,
    failures,
    metrics: {
      subCentroidKm: +subCentroidKm.toFixed(2),
      subtxStraightKm,
      gridSpreadDeg: +gridSpreadDeg.toFixed(1),
      zipfRatio: +zipfRatio.toFixed(2),
      zipfTopRatio: +zipfTopRatio.toFixed(2),
      townCounts: sorted,
    },
  };
}

// Best-of-N wrapper: deterministic retry seeds; on exhaustion the attempt
// with the FEWEST failures wins and the reasons are reported in Checks.
export function generate(params) {
  const attempts = [];
  let best = null;
  for (let a = 0; a < VALIDATION.MAX_ATTEMPTS; a++) {
    const seed = a === 0 ? params.seed : `${params.seed}#retry${a}`;
    const w = generateOnce({ ...params, seed });
    attempts.push({ attempt: a + 1, seed, failures: w.validation.failures });
    if (!best || w.validation.failures.length < best.validation.failures.length) best = w;
    if (w.validation.failures.length === 0) break;
  }
  best.validationHistory = attempts;
  const last = attempts[attempts.length - 1];
  best.checks.push({
    name: "Validation (regenerate-on-fail, best-of-N)",
    pass: best.validation.pass,
    detail: best.validation.pass
      ? `passed on attempt ${attempts.length}/${VALIDATION.MAX_ATTEMPTS}` +
        (attempts.length > 1 ? ` (earlier: ${attempts.slice(0, -1).map(x => x.failures.join("; ")).join(" | ")})` : "")
      : `best of ${attempts.length} attempts — unresolved: ${best.validation.failures.join("; ")}`,
  });
  return best;
}

// ---------------------------------- devices-for-improvement table

const DR_TARGETS = [0.10, 0.25, 0.50];
const DR_CAP = { switch: 120, recloser: 60 }; // match the UI input limits

// Fewest greedily-placed devices of one kind (from a device-free network)
// to reach each fractional SAIDI improvement. Placed in batches so the
// search stops as soon as the deepest target is met; a null count means
// the target was not reached (exhausted tells the two reasons apart).
export function countsForTargets(net, kind, rateMul = null) {
  const cap = DR_CAP[kind];
  const counts = DR_TARGETS.map(() => null);
  let devices = null, placed = 0, base = null, exhausted = false;
  while (placed < cap && !exhausted && counts[counts.length - 1] === null) {
    const res = greedyPlace(net, Math.min(6, cap - placed), kind, devices, rateMul);
    if (base === null) base = res.baseline;
    for (const l of res.log) {
      if (l.stopped) { exhausted = true; break; }
      placed++;
      const imp = (base - l.saidiAfter) / base;
      for (let i = 0; i < DR_TARGETS.length; i++) {
        if (counts[i] === null && imp >= DR_TARGETS[i]) counts[i] = placed;
      }
    }
    devices = res.devices;
  }
  return { counts, exhausted, cap };
}

// ------------------------------------------------------------- selftest

function selftest() {
  const results = [];
  for (const seed of ["aotearoa-1", "kahikatea-2", "rimu-3"]) {
    const world = generate({ seed, nCust: 50000, nTowns: 5 }); // the fixed UI size
    const { net } = world;
    const greedy = greedyPlace(net, 12, "switch");
    const mono = checkMonotone(greedy.log);
    const greedyRc = greedyPlace(net, 6, "recloser", greedy.devices);
    const monoRc = checkMonotone(greedyRc.log);
    const dbg = debugRateExperiment(net);
    // fault conservation on three deterministic edges, with devices placed
    const ids = [0, Math.floor(net.treeEdges.length / 2), net.treeEdges.length - 1];
    const conservation = ids.map(i => {
      const sc = faultScenario(net, greedyRc.devices, i);
      return checkFaultConservation(sc).pass;
    });
    const rvl = world.roadVsLine;
    // --- diagnostics: river geometry + customer split across the river
    const rp = world.terrain.riverPath;
    let riverLen = 0;
    for (let i = 1; i < rp.length; i++) riverLen += Math.hypot(rp[i].x - rp[i - 1].x, rp[i].y - rp[i - 1].y);
    let left = 0;
    for (const c of world.customers) {
      let bi = 0, bd = Infinity;
      for (let i = 0; i < rp.length - 1; i += 4) {
        const d = (rp[i].x - c.x) ** 2 + (rp[i].y - c.y) ** 2;
        if (d < bd) { bd = d; bi = i; }
      }
      const j = Math.min(rp.length - 1, bi + 4);
      const cross = (rp[j].x - rp[bi].x) * (c.y - rp[bi].y) - (rp[j].y - rp[bi].y) * (c.x - rp[bi].x);
      if (cross > 0) left++;
    }
    const ranked = allCandidates(net, emptyDevices(), "switch");
    // --- diagnostics: how far customers sit from their sub (road metres)
    const dists = world.customers
      .map(c => net.dist[net.txs[c.tx].node]).filter(isFinite).sort((a, b) => a - b);
    const q = (p) => +(dists[Math.floor(p * (dists.length - 1))] / 1000).toFixed(1);
    let minSep = Infinity;
    for (let i = 0; i < net.subs.length; i++) {
      for (let j = i + 1; j < net.subs.length; j++) {
        minSep = Math.min(minSep, Math.hypot(
          net.subs[i].x - net.subs[j].x, net.subs[i].y - net.subs[j].y));
      }
    }
    results.push({
      custDistKm: {
        mean: +(dists.reduce((s, d) => s + d, 0) / dists.length / 1000).toFixed(1),
        p50: q(0.5), p95: q(0.95), max: q(1),
        beyond10kmPct: +(100 * dists.filter(d => d > 10000).length / dists.length).toFixed(1),
      },
      subSepKm: isFinite(minSep) ? +(minSep / 1000).toFixed(1) : null,
      subPos: net.subs.map(s => [Math.round(s.x / 100) / 10, Math.round(s.y / 100) / 10]),
      subCust: net.subs.map(s => Math.round(
        net.feeders.filter(f => f.sub === s.id).reduce((t, f) => t + f.customers, 0))),
      river: {
        lenKm: +(riverLen / 1000).toFixed(1),
        start: [Math.round(rp[0].x), Math.round(rp[0].y)],
        end: [Math.round(rp[rp.length - 1].x), Math.round(rp[rp.length - 1].y)],
        custLeftShare: +(left / world.customers.length).toFixed(3),
      },
      dbgGains: ranked.slice(0, 6).map(c => ({ f: c.feeder, n: c.node, g: Math.round(c.gain) })),
      townSides: world.towns.map(t => t.side),
      dbgBoostGain: dbg.supported ? Math.round(dbg.boostGain ?? -1) : null,
      seed,
      totalMs: world.timings.total,
      timings: world.timings,
      custFinal: world.customers.length,
      prune: world.prune,
      checks: world.checks.map(c => ({ name: c.name, pass: c.pass, tunable: !!c.tunable, detail: c.detail })),
      feeders: net.feeders.length,
      meanCustPerFeeder: Math.round(world.customers.length / net.feeders.length),
      urbanMeanCust: feederKindMean(net, true),
      ruralMeanCust: feederKindMean(net, false),
      feederSizes: net.feeders.map(f => f.customers).sort((a, b) => b - a),
      subs: net.subs.length,
      membership: {
        urbanSharePct: Math.round(net.membership.urbanShare * 100),
        loadNodes: net.membership.loadNodes,
        worstTxSubKm: net.membership.worstTxSubKm,
        worstCircuitKm: net.membership.worstCircuitKm,
        reassigned: net.membership.reassigned,
        maxFeedersPerSub: Math.max(0, ...net.subs.map(s => net.feeders.filter(f => f.sub === s.id).length)),
        easementSpans: net.membership.easementSpans,
        expressFeeders: net.membership.expressFeeders,
        loudExpress: net.membership.loudExpress,
        parsimony: net.membership.parsimony,
        ruleViolations: net.membership.ruleViolations,
      },
      txs: net.txs.length,
      ties: net.ties.length,
      roadNodes: world.graph.nNodes,
      bridges: world.terrain.bridges.length,
      snapMeanM: Math.round(world.snapStats.mean),
      snapMaxM: Math.round(world.snapStats.max),
      baselineSaidi: +greedy.baseline.toFixed(1),
      afterSwitchesSaidi: +greedy.final.toFixed(1),
      finalSaidi: +greedyRc.final.toFixed(1),
      greedyMonotone: mono.pass,
      recloserMonotone: monoRc.pass,
      greedyPlacements: greedy.log.filter(l => !l.stopped).length,
      recloserPlacements: greedyRc.log.filter(l => !l.stopped).length,
      debugSupported: dbg.supported,
      debugMoved: dbg.supported ? dbg.moved : null,
      debugRateWeighted: dbg.supported ? dbg.rateWeighted : null,
      faultConservation: conservation.every(Boolean),
      roadVsLineMin: +Math.min(...rvl.map(r => r.ratio)).toFixed(3),
      roadVsLineMax: +Math.max(...rvl.map(r => r.ratio)).toFixed(3),
      gxp: world.subtx.gxp ? [Math.round(world.subtx.gxp.x), Math.round(world.subtx.gxp.y)] : null,
      validationPass: world.validation.pass,
      validationAttempts: world.validationHistory.length,
      validationMetrics: world.validation.metrics,
      unresolved: world.validation.failures,
      drTargets: {
        sw: countsForTargets(net, "switch"),
        rc: countsForTargets(net, "recloser"),
      },
    });
  }
  // Inland-weighting slider path: full-inland towns must still generate a
  // clean network, and must actually move the towns.
  const wCoast = generate({ seed: "aotearoa-1", nCust: 50000, nTowns: 5, inlandWeight: 0 });
  const wInland = generate({ seed: "aotearoa-1", nCust: 50000, nTowns: 5, inlandWeight: 1 });
  // Correctness checks must pass; an unresolved best-of-N VALIDATION on
  // this deliberately extreme world is a reported outcome, not a failure.
  const inlandTest = {
    checksPass: wInland.checks
      .filter(c => !c.name.startsWith("Validation")).every(c => c.pass || c.tunable),
    validationUnresolved: wInland.validation.failures,
    failedChecks: wInland.checks.filter(c => !c.pass).map(c => `${c.name}: ${c.detail}`),
    townsMoved: JSON.stringify(wCoast.towns.map(t => [t.x | 0, t.y | 0])) !==
      JSON.stringify(wInland.towns.map(t => [t.x | 0, t.y | 0])),
    meanCoastDistCoastal: townCoastDist(wCoast),
    meanCoastDistInland: townCoastDist(wInland),
  };
  // Correctness checks gate allPass; tunable-rule residuals (caps, trunk,
  // transit — marked tunable) are reported but do not gate: they are the
  // user's dials, and some seeds legitimately leave residuals to tune.
  const allPass = results.every(r =>
    r.checks.every(c => c.pass || c.tunable) && r.greedyMonotone && r.recloserMonotone &&
    r.faultConservation && r.validationPass &&
    // degeneracy bands: sub and feeder counts EMERGE from the rule caps
    // (100 km map, 50k sampled, busbar-branch feeders with no express
    // runs: measured ≈ 35 subs, 88–105 feeders, ≈ 475–570 cust/feeder)
    r.subs >= 12 && r.subs <= 70 &&
    r.feeders >= 50 && r.feeders <= 400 &&
    r.meanCustPerFeeder >= 60 && r.meanCustPerFeeder <= 900 &&
    r.gxp !== null &&
    // reclosers must reach the 10% target; switch-only 10% is NOT
    // guaranteed on a 50k-customer network (restoration-only gains are
    // smaller), so for switches just assert greedy finds beneficial sites
    r.drTargets.rc.counts[0] !== null && r.greedyPlacements >= 6 &&
    r.totalMs < 30000 && (!r.debugSupported || r.debugRateWeighted === true)) &&
    inlandTest.checksPass && inlandTest.townsMoved;
  const out = { allPass, inlandTest, results };
  const pre = document.getElementById("selftest-out");
  pre.textContent = JSON.stringify(out, null, 1);
  pre.style.display = "block";
  document.title = "SELFTEST-DONE";
  return out;
}

// Mean straight-line distance (km) from towns to the coast edge — used by
// the selftest to show the inland slider actually pulls towns inland.
function townCoastDist(world) {
  const t = world.terrain;
  const d = world.towns.map(tn => {
    const [cx, cy] = t.cellOf(tn.x, tn.y);
    return t._coastDist(cx, cy) * t.coastAxisM() / 1000; // normalised → km inland
  });
  return +(d.reduce((a, b) => a + b, 0) / d.length).toFixed(1);
}

// Mean customers per feeder, split urban vs rural by underground share
// (UG sections exist exactly where density is high).
function feederKindMean(net, urban) {
  const sel = net.feeders.filter(f => {
    const ugShare = f.ugLenM / Math.max(1, f.lengthM);
    return urban ? ugShare >= 0.4 : ugShare < 0.4;
  });
  if (!sel.length) return null;
  return Math.round(sel.reduce((s, f) => s + f.customers, 0) / sel.length);
}

// ------------------------------------------------------------- scaletest
// ?scaletest=1 — one seed at increasing customer counts, reporting timing
// and all correctness checks, to find where the model gets slow or messy.

function scaletest() {
  const sizes = [8000, 12000, 16000, 24000, 32000, 48000, 64000];
  const results = [];
  for (const nCust of sizes) {
    const t0 = performance.now();
    const world = generate({ seed: "aotearoa-1", nCust, nTowns: 5 });
    const genMs = Math.round(performance.now() - t0);
    const t1 = performance.now();
    const greedy = greedyPlace(world.net, 8, "switch");
    const greedyRc = greedyPlace(world.net, 4, "recloser", greedy.devices);
    const greedyMs = Math.round(performance.now() - t1);
    results.push({
      nCust, genMs, greedyMs,
      timings: world.timings,
      checksPass: world.checks.every(c => c.pass || c.tunable),
      failedChecks: world.checks.filter(c => !c.pass && !c.tunable).map(c => c.name),
      tunableResiduals: world.checks.filter(c => !c.pass && c.tunable).map(c => c.name),
      txs: world.net.txs.length,
      feeders: world.net.feeders.length,
      treeEdges: world.net.treeEdges.length,
      roadNodes: world.graph.nNodes,
      snapMeanM: Math.round(world.snapStats.mean),
      monotone: checkMonotone(greedy.log).pass && checkMonotone(greedyRc.log).pass,
      maxTxLoadShare: +(Math.max(...world.net.txs.map(t => t.customers.length)) / 100).toFixed(2),
    });
  }
  const pre = document.getElementById("selftest-out");
  pre.textContent = JSON.stringify({ results }, null, 1);
  pre.style.display = "block";
  document.title = "SELFTEST-DONE";
}

// ------------------------------------------------------------- UI state

const $ = (id) => document.getElementById(id);
const state = {
  world: null,
  renderer: null,
  greedyLog: [],
  debug: null,       // debugRateExperiment result when debug mode on
  rateMul: null,
  playbackAnim: null,
  faultMode: false,
};

function fmt(x, dp = 1) { return x.toLocaleString("en-NZ", { maximumFractionDigits: dp, minimumFractionDigits: dp }); }

// The UI world SAMPLES a fixed 50 000 customers; those on feeders under
// FEEDER_MIN_CUST are pruned, so the served count emerges a little lower.
// Other sizes remain available programmatically.
const N_CUST_START = 50000;

function regenerate() {
  const params = {
    seed: $("seed").value || "aotearoa",
    nCust: N_CUST_START,
    nTowns: +$("nTowns").value,
    inlandWeight: +$("inland").value / 100,
  };
  const world = generate(params);
  state.world = world;
  state.greedyLog = [];
  state.debug = null;
  state.rateMul = null;
  state.playbackAnim = null;
  $("debugRate").checked = false;
  state.renderer.setWorld(world);
  const ohKm = world.net.feeders.reduce((s, f) => s + f.ohLenM, 0) / 1000;
  const ugKm = world.net.feeders.reduce((s, f) => s + f.ugLenM, 0) / 1000;
  const meanCust = world.customers.length / world.net.feeders.length;
  $("genTime").textContent =
    `generated in ${world.timings.total} ms — ` +
    `${world.customers.length.toLocaleString()} of ${N_CUST_START.toLocaleString()} customers kept ` +
    `(${world.prune.customers.toLocaleString()} pruned on sub-20 feeders), ` +
    `${world.graph.nNodes.toLocaleString()} road nodes, ${world.net.txs.length} TX, ` +
    `${world.net.subs.length} subs, ${world.net.feeders.length} feeders ` +
    `(mean ${Math.round(meanCust)} cust/feeder), ` +
    `${ohKm.toFixed(0)} km OH / ${ugKm.toFixed(0)} km UG, ` +
    `${world.terrain.bridges.length} bridge(s), ` +
    `${world.net.membership.easementSpans} off-road span(s), ` +
    `${world.net.ties.length} normally-open tie(s)`;
  renderChecks();
  computeDrTable();
  refreshDerived();
  renderFeederStats(-1);
  $("greedyLog").innerHTML = "";
  $("playbackInfo").innerHTML = "<em>Toggle fault mode, then click a line section on the map.</em>";
  draw();
}

function draw() { state.renderer.draw(); }

function currentDevices() {
  return {
    switches: new Set(state.renderer.switchList),
    reclosers: new Set(state.renderer.recloserList),
  };
}

function renderChecks(extra = []) {
  const el = $("checks");
  const items = [...state.world.checks, ...extra];
  el.innerHTML = items.map(c =>
    `<div class="check ${c.pass ? "pass" : "fail"}">` +
    `<span class="mark">${c.pass ? "✓" : "✗"}</span>` +
    `<div><strong>${c.name}</strong><br><span class="detail">${c.detail}</span></div></div>`
  ).join("");
}

function renderSaidi() {
  const { net } = state.world;
  const dev = currentDevices();
  const base = computeSaidi(net, emptyDevices(), state.rateMul);
  const now = computeSaidi(net, dev, state.rateMul);
  $("saidiBase").textContent = fmt(base.overall);
  $("saidiNow").textContent = fmt(now.overall);
  $("saidiSwitches").textContent = dev.switches.size;
  $("saidiReclosers").textContent = dev.reclosers.size;
  const delta = base.overall - now.overall;
  $("saidiDelta").textContent = fmt(delta);
}

// ------------------------------- devices-for-improvement table (UI)

function computeDrTable() {
  const { net } = state.world;
  state.drTable = {
    sw: countsForTargets(net, "switch", state.rateMul),
    rc: countsForTargets(net, "recloser", state.rateMul),
  };
}

function renderDrTable() {
  const t = state.drTable;
  if (!t) return;
  const cell = (r, i) => r.counts[i] !== null ? `<strong>${r.counts[i]}</strong>`
    : r.exhausted ? `<span class="detail">not reachable</span>`
    : `<span class="detail">&gt;${r.cap}</span>`;
  $("drTable").innerHTML =
    `<tr><th>SAIDI cut</th><th>Sectionalisers</th><th>Reclosers</th></tr>` +
    DR_TARGETS.map((tgt, i) =>
      `<tr><td>≥ ${Math.round(tgt * 100)}%</td><td>${cell(t.sw, i)}</td><td>${cell(t.rc, i)}</td></tr>`
    ).join("");
  // the actual placed mix (may combine both device kinds), for comparison
  const dev = currentDevices();
  if (dev.switches.size + dev.reclosers.size === 0) { $("drNow").textContent = ""; return; }
  const base = computeSaidi(state.world.net, emptyDevices(), state.rateMul).overall;
  const now = computeSaidi(state.world.net, dev, state.rateMul).overall;
  $("drNow").textContent =
    `Placed mix (${dev.switches.size} SW + ${dev.reclosers.size} RC): ` +
    `${fmt(100 * (base - now) / base)}% improvement`;
}

// ------------------------------------------------ feeder league + heat

function renderLeague() {
  const { net } = state.world;
  const per = computeSaidi(net, currentDevices(), state.rateMul).perFeeder
    .slice().sort((a, b) => b.custMin - a.custMin);
  $("leagueTable").innerHTML =
    `<tr><th>#</th><th>Feeder</th><th>Cust</th><th>SAIDI</th><th>Cust·min</th></tr>` +
    per.map((p, i) => {
      const f = net.feeders.find(f => f.id === p.feeder);
      return `<tr data-fid="${p.feeder}" class="${state.renderer.selectedFeeder === p.feeder ? "sel" : ""}">` +
        `<td>${i + 1}</td>` +
        `<td><span class="chip" style="background:${feederColour(p.feeder)}"></span>F${p.feeder}</td>` +
        `<td>${f.customers}</td><td>${fmt(p.saidi)}</td><td>${(p.custMin / 1000).toFixed(0)}k</td></tr>`;
    }).join("");
}

function updateHeat() {
  const { net } = state.world;
  const per = computeSaidi(net, currentDevices(), state.rateMul).perFeeder;
  const maxCM = Math.max(1, ...per.map(p => p.custMin));
  const heat = new Map(per.map(p => [p.feeder, p.custMin / maxCM]));
  state.renderer.updateHeat(state.world, heat);
}

// One call to refresh everything that depends on devices/rates.
function refreshDerived() {
  renderSaidi();
  renderLeague();
  updateHeat();
  renderDrTable();
  renderSubTable();
  renderFeederStats(state.renderer.selectedFeeder);
}

// Zone sub summary — customers, total HV feeder length, customer-weighted
// SAIDI under the current devices/rates. Worst first.
function renderSubTable() {
  const { net } = state.world;
  const per = computeSaidi(net, currentDevices(), state.rateMul).perFeeder;
  const cmOf = new Map(per.map(p => [p.feeder, p.custMin]));
  const rows = net.subs.map(s => {
    const fs = net.feeders.filter(f => f.sub === s.id);
    const cust = fs.reduce((t, f) => t + f.customers, 0);
    const lenKm = fs.reduce((t, f) => t + f.lengthM, 0) / 1000;
    const cm = fs.reduce((t, f) => t + (cmOf.get(f.id) ?? 0), 0);
    return { s, nF: fs.length, cust, lenKm, saidi: cust > 0 ? cm / cust : 0 };
  }).sort((a, b) => b.saidi - a.saidi);
  $("subTable").innerHTML =
    `<tr><th>Zone sub</th><th>Feeders</th><th>Cust</th><th>HV km</th><th>SAIDI</th></tr>` +
    rows.map(r =>
      `<tr><td>${r.s.name}</td><td>${r.nF}</td><td>${Math.round(r.cust).toLocaleString("en-NZ")}</td>` +
      `<td>${fmt(r.lenKm, 0)}</td><td><strong>${fmt(r.saidi)}</strong></td></tr>`).join("");
}

function renderFeederStats(fid) {
  const el = $("feederStats");
  if (fid < 0) { el.innerHTML = "<em>Click a coloured feeder line on the map.</em>"; return; }
  const { net } = state.world;
  const f = net.feeders.find(f => f.id === fid);
  const dev = currentDevices();
  const base = computeSaidi(net, emptyDevices(), state.rateMul).perFeeder.find(p => p.feeder === fid);
  const now = computeSaidi(net, dev, state.rateMul).perFeeder.find(p => p.feeder === fid);
  const rvl = state.world.roadVsLine.find(r => r.feeder === fid);
  const nSw = [...dev.switches].filter(id => net.treeEdges[id].feeder === fid).length;
  const nRc = [...dev.reclosers].filter(id => net.treeEdges[id].feeder === fid).length;
  const ugShare = f.ugLenM / Math.max(1, f.lengthM);
  const kind = ugShare >= 0.4 ? "urban" : ugShare >= 0.15 ? "mixed" : "rural";
  const perKm = f.customers / Math.max(0.3, f.lengthM / 1000);
  el.innerHTML = `
    <div><span class="chip" style="background:${feederColour(fid)}"></span>
      <strong>Feeder F${fid}</strong> — ${net.subs[f.sub].name} zone sub</div>
    <table>
      <tr><td>Type</td><td>${kind} (${fmt(perKm, 0)} cust/km)</td></tr>
      <tr><td>Customers</td><td>${f.customers}</td></tr>
      <tr><td>Transformers</td><td>${f.txCount}</td></tr>
      <tr><td>HV length</td><td>${fmt(f.lengthM / 1000, 1)} km ` +
        `(${fmt(f.ohLenM / 1000, 1)} OH / ${fmt(f.ugLenM / 1000, 1)} UG)</td></tr>
      <tr><td>Sectionalisers</td><td>${nSw}</td></tr>
      <tr><td>Reclosers</td><td>${nRc}</td></tr>
      <tr><td>SAIDI baseline</td><td>${fmt(base.saidi)} min/yr</td></tr>
      <tr><td>SAIDI now</td><td>${fmt(now.saidi)} min/yr</td></tr>
      <tr><td>Road ÷ line distance</td><td>${fmt(rvl.ratio, 2)}×</td></tr>
    </table>`;
}

// ------------------------------------------ greedy device placement

function placeDevices(kind) {
  const n = +(kind === "switch" ? $("nSwitches") : $("nReclosers")).value;
  const { net } = state.world;
  const existing = currentDevices();
  const nExisting = existing.switches.size + existing.reclosers.size;
  const result = greedyPlace(net, n, kind, existing, state.rateMul);
  const newOnes = result.log.filter(l => !l.stopped);
  if (result.log.some(l => l.stopped)) {
    const div = document.createElement("div");
    div.className = "logentry";
    div.innerHTML = `<em>stopped early — no candidate with positive benefit</em>`;
    $("greedyLog").prepend(div);
  }
  // animate placements landing one by one
  let i = 0;
  const logEl = $("greedyLog");
  const stepIn = () => {
    if (i >= newOnes.length) {
      const mono = checkMonotone(result.log);
      renderChecks([mono]);
      return;
    }
    const l = newOnes[i++];
    (kind === "switch" ? state.renderer.switchList : state.renderer.recloserList).push(l.teId);
    const div = document.createElement("div");
    div.className = "logentry";
    div.innerHTML =
      `<span class="chip" style="background:${feederColour(l.feeder)}"></span>` +
      `#${nExisting + l.step} <strong>${kind === "switch" ? "SW" : "RC"}</strong> ` +
      `node ${l.node} (F${l.feeder}) — ` +
      `<strong>${l.custRestored}</strong> customers ${kind === "switch" ? "restorable" : "protected"}, ` +
      `SAIDI ${fmt(l.saidiBefore)} → <strong>${fmt(l.saidiAfter)}</strong> ` +
      `(−${fmt(l.benefitMin, 2)} min) ${l.monotone ? "" : "⚠ NON-MONOTONE"}`;
    logEl.prepend(div);
    refreshDerived();
    draw();
    setTimeout(stepIn, 240);
  };
  stepIn();
}

function resetDevices() {
  state.renderer.switchList = [];
  state.renderer.recloserList = [];
  $("greedyLog").innerHTML = "";
  refreshDerived();
  renderChecks();
  draw();
}

function toggleDebugRate(on) {
  const { net } = state.world;
  resetDevices();
  if (!on) {
    state.debug = null;
    state.rateMul = null;
    state.renderer.debugBranch = null;
    $("debugInfo").innerHTML = "";
  } else {
    const dbg = debugRateExperiment(net);
    state.debug = dbg;
    if (!dbg.supported) {
      $("debugInfo").innerHTML = `<em>Not supported: ${dbg.reason ?? "no candidates"}.</em>`;
      state.rateMul = null;
      state.renderer.debugBranch = null;
    } else {
      state.rateMul = dbg.rateMul;
      state.renderer.debugBranch = dbg.branch;
      $("debugInfo").innerHTML =
        `λ doubled on the branch below node ${dbg.boostNode} ` +
        `(<span class="chip" style="background:${feederColour(dbg.boostFeeder)}"></span>F${dbg.boostFeeder}, ` +
        `${dbg.branch.size} sections, shown dashed).<br>` +
        `First greedy pick: node ${dbg.basePick.node} (F${dbg.basePick.feeder}) → ` +
        `node ${dbg.debugPick.node} (F${dbg.debugPick.feeder}) — ` +
        (dbg.moved
          ? `<strong class="ok">pick moved ✓ (benefit is rate-weighted)</strong>`
          : dbg.rateWeighted
            ? `<strong class="ok">pick held — the baseline pick dominates every ` +
              `single doubling on this seed, but the boosted branch's benefit ` +
              `rose ×${(dbg.boostRatio ?? 0).toFixed(3)} (in-branch part doubles; ` +
              `the backfeed part from upstream faults is rate-independent) ✓ rate-weighted</strong>`
            : `<strong class="bad">boosted benefit did NOT rise with its fault rate ✗</strong>`);
    }
  }
  computeDrTable();
  refreshDerived();
  draw();
}

// ------------------------------------------------------------ fault playback

function startFault(teId) {
  const { net } = state.world;
  const sc = faultScenario(net, currentDevices(), teId);
  const cons = checkFaultConservation(sc);
  renderChecks([cons]);
  const tEnd = sc.tRepairDone + 15;
  state.playbackAnim = { sc, tEnd, playing: true, tMin: 0 };
  state.renderer.playback = { scenario: sc, tMin: 0 };
  const swText = sc.tSwitch === null
    ? `<span class="chip" style="background:${STATUS.serious}"></span> no isolating device helps here — everyone tripped waits for repair`
    : `<span class="chip" style="background:${STATUS.warning}"></span> ${sc.custIso} customers isolatable upstream — restored at ${SWITCH_MIN} min`;
  const tieText = sc.custTie > 0
    ? `<div><span class="chip" style="background:${STATUS.warning}"></span> ${sc.custTie} customers BACKFED from the neighbouring feeder via a normally-open tie — restored at ${SWITCH_MIN} min</div>`
    : "";
  const rcText = sc.recloserEdge !== -1
    ? `<div><span class="chip" style="background:${feederColour(sc.feeder)}"></span> ${sc.custUnaffected} customers upstream of the recloser — never interrupted</div>`
    : "";
  $("playbackInfo").innerHTML = `
    <div><strong>Fault on F${sc.feeder}</strong>, section ${sc.teId} — ${sc.custAffected} customers affected` +
    `${sc.recloserEdge !== -1 ? " (recloser zone)" : ""}</div>
    <div><span class="chip" style="background:${STATUS.critical}"></span> faulted section</div>
    <div><span class="chip" style="background:${STATUS.serious}"></span> ${sc.custOut} customers out until repair (travel ${fmt(sc.tTravel)} + ${REPAIR_MIN} min)</div>
    <div>${swText}</div>
    ${tieText}
    ${rcText}
    <div class="detail ${cons.pass ? "ok" : "bad"}">${cons.pass ? "✓" : "✗"} ${cons.detail}</div>
    <div class="timeline"><input type="range" id="tScrub" min="0" max="${Math.ceil(tEnd)}" value="0" step="1">
    <span id="tLabel">t = 0 min</span></div>`;
  $("tScrub").addEventListener("input", (e) => {
    state.playbackAnim.playing = false;
    state.playbackAnim.tMin = +e.target.value;
    updatePlayback();
  });
  const t0 = performance.now();
  const realMsPerSimMin = 7000 / tEnd; // whole timeline ≈ 7 s
  const tick = () => {
    if (!state.playbackAnim || state.playbackAnim.sc !== sc) return;
    if (state.playbackAnim.playing) {
      state.playbackAnim.tMin = Math.min(tEnd, (performance.now() - t0) / realMsPerSimMin);
      updatePlayback();
      if (state.playbackAnim.tMin < tEnd) requestAnimationFrame(tick);
    }
  };
  requestAnimationFrame(tick);
}

function updatePlayback() {
  const pa = state.playbackAnim;
  state.renderer.playback = { scenario: pa.sc, tMin: pa.tMin };
  const scrub = $("tScrub");
  if (scrub) { scrub.value = Math.round(pa.tMin); }
  const lbl = $("tLabel");
  if (lbl) {
    const zone = pa.sc.recloserEdge !== -1 ? "recloser zone out" : "whole feeder out";
    let phase = `fault — protection open, ${zone}`;
    if (pa.sc.tSwitch !== null && pa.tMin >= pa.sc.tSwitch) phase = "switched — upstream of switch restored";
    if (pa.tMin >= pa.sc.tRepairDone) phase = "repaired — all restored";
    lbl.textContent = `t = ${Math.round(pa.tMin)} min — ${phase}`;
  }
  draw();
}

function clearFault() {
  state.playbackAnim = null;
  state.renderer.playback = null;
  $("playbackInfo").innerHTML = "<em>Toggle fault mode, then click a line section on the map.</em>";
  renderChecks();
  draw();
}

// --------------------------------------------------------------- wiring

function initUI() {
  const canvas = $("map");
  const resize = () => {
    const r = canvas.parentElement.getBoundingClientRect();
    canvas.width = r.width; canvas.height = r.height;
    if (state.renderer) draw();
  };
  state.renderer = new Renderer(canvas);
  window.addEventListener("resize", resize);
  resize();

  // pan/zoom
  let dragging = false, lx = 0, ly = 0, moved = 0;
  canvas.addEventListener("mousedown", (e) => { dragging = true; moved = 0; lx = e.offsetX; ly = e.offsetY; });
  window.addEventListener("mouseup", () => { dragging = false; });
  canvas.addEventListener("mousemove", (e) => {
    if (!dragging) return;
    state.renderer.pan(e.offsetX - lx, e.offsetY - ly);
    moved += Math.abs(e.offsetX - lx) + Math.abs(e.offsetY - ly);
    lx = e.offsetX; ly = e.offsetY;
    draw();
  });
  canvas.addEventListener("wheel", (e) => {
    e.preventDefault();
    state.renderer.zoomAt(e.offsetX, e.offsetY, e.deltaY < 0 ? 1.15 : 1 / 1.15);
    draw();
  }, { passive: false });
  canvas.addEventListener("click", (e) => {
    if (moved > 4) return; // was a drag
    const teId = state.renderer.hitTestEdge(e.offsetX, e.offsetY);
    if (teId < 0) return;
    const fid = state.world.net.treeEdges[teId].feeder;
    state.renderer.selectedFeeder = fid;
    state.renderer.selectedEdge = teId;
    renderFeederStats(fid);
    if (state.faultMode) startFault(teId);
    draw();
  });

  // controls
  $("regen").addEventListener("click", regenerate);
  $("seed").addEventListener("keydown", (e) => { if (e.key === "Enter") regenerate(); });
  $("randomSeed").addEventListener("click", () => {
    $("seed").value = "nz-" + Math.floor(performance.now() * 997 % 100000);
    regenerate();
  });
  for (const id of ["nTowns", "inland"]) {
    $(id).addEventListener("input", () => { $(id + "Val").textContent = $(id).value; });
  }
  for (const layer of ["terrain", "density", "roads", "customers", "network", "txs", "switches", "bridges", "roadVsLine", "heat", "subtx", "ties"]) {
    const box = $("layer-" + layer);
    if (!box) continue;
    box.checked = state.renderer.layers[layer];
    box.addEventListener("change", () => {
      state.renderer.layers[layer] = box.checked;
      draw();
    });
  }
  $("placeSwitches").addEventListener("click", () => placeDevices("switch"));
  $("placeReclosers").addEventListener("click", () => placeDevices("recloser"));
  $("resetSwitches").addEventListener("click", resetDevices);
  for (const id of ["rateOh", "rateUg"]) {
    $(id).addEventListener("change", () => {
      setFaultRates(+$("rateOh").value, +$("rateUg").value);
      // placed devices stay; all figures recompute under the new rates
      state.world.roadVsLine = roadVsLine(state.world.net, state.world.graph);
      computeDrTable();
      refreshDerived();
      renderChecks();
      draw();
    });
  }
  $("leagueTable").addEventListener("click", (e) => {
    const tr = e.target.closest("tr[data-fid]");
    if (!tr) return;
    const fid = +tr.dataset.fid;
    state.renderer.selectedFeeder = fid;
    state.renderer.selectedEdge = -1;
    state.renderer.zoomToFeeder(state.world, fid);
    renderFeederStats(fid);
    renderLeague();
    draw();
  });
  $("debugRate").addEventListener("change", (e) => toggleDebugRate(e.target.checked));
  $("faultMode").addEventListener("change", (e) => {
    state.faultMode = e.target.checked;
    if (!state.faultMode) clearFault();
  });
  $("clearFault").addEventListener("click", () => { $("faultMode").checked = false; state.faultMode = false; clearFault(); });

  // assumptions panel
  $("assumptions").innerHTML = ASSUMPTIONS.map(a =>
    `<div class="assumption"><strong>${a.area}</strong> — ${a.text}</div>`).join("");

  // URL params: ?seed=x&cust=8000&towns=5&inland=100 preset the generation
  // controls; ?on=heat,density&off=roads preset layers (handy for sharing
  // a specific view and for headless screenshots)
  const qs = new URLSearchParams(location.search);
  for (const [param, id] of [["seed", "seed"], ["towns", "nTowns"], ["inland", "inland"]]) {
    const v = qs.get(param);
    if (v !== null) {
      $(id).value = v;
      const lbl = $(id + "Val");
      if (lbl) lbl.textContent = v;
    }
  }
  for (const [param, val] of [["on", true], ["off", false]]) {
    for (const name of (qs.get(param) ?? "").split(",").filter(Boolean)) {
      if (name in state.renderer.layers) {
        state.renderer.layers[name] = val;
        const box = $("layer-" + name);
        if (box) box.checked = val;
      }
    }
  }

  regenerate();

  // ?demo — place switches and freeze a fault mid-timeline (used for
  // headless visual verification; harmless to use by hand too).
  if (location.search.includes("demo")) {
    const { net } = state.world;
    const rSw = greedyPlace(net, 8, "switch");
    for (const l of rSw.log.filter(l => !l.stopped)) state.renderer.switchList.push(l.teId);
    const rRc = greedyPlace(net, 4, "recloser", currentDevices());
    for (const l of rRc.log.filter(l => !l.stopped)) state.renderer.recloserList.push(l.teId);
    refreshDerived();
    const mid = net.treeEdges[Math.floor(net.treeEdges.length * 0.55)];
    startFault(mid.id);
    state.playbackAnim.playing = false;
    state.playbackAnim.tMin = Math.max(SWITCH_MIN + 5,
      Math.min(state.playbackAnim.sc.tRepairDone - 10, 60));
    updatePlayback();
  }
}

// --------------------------------------------------------------- boot

if (location.search.includes("scaletest")) {
  try {
    scaletest();
  } catch (err) {
    const pre = document.getElementById("selftest-out");
    pre.textContent = JSON.stringify({ error: String(err), stack: String(err.stack) });
    pre.style.display = "block";
    document.title = "SELFTEST-DONE";
  }
} else if (location.search.includes("selftest")) {
  // Synchronous at module top level so headless --dump-dom captures it.
  try {
    selftest();
  } catch (err) {
    const pre = document.getElementById("selftest-out");
    pre.textContent = JSON.stringify({ allPass: false, error: String(err), stack: String(err.stack) });
    pre.style.display = "block";
    document.title = "SELFTEST-DONE";
  }
} else {
  initUI();
}
