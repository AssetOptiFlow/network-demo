// checks.js — correctness guards. Every guard is ASSERTED and REPORTED in
// the UI, never assumed.

import { components, CLS_EASEMENT } from "./roads.js";
import { computeSaidi, emptyDevices } from "./reliability.js";

export function runChecks(world) {
  const { graph, customers, net } = world;
  const checks = [];
  const add = (name, pass, detail) => {
    checks.push({ name, pass: !!pass, detail });
    console.assert(pass, `CHECK FAILED: ${name} — ${detail}`);
  };

  // 1. road graph fully connected
  const { nComp } = components(graph);
  add("Road graph fully connected", nComp === 1,
    `${nComp} component(s), ${graph.nNodes} nodes, ${graph.edges.length} edges` +
    (world.roadStats ? `, ${world.roadStats.webLinks} rural web link(s), ` +
      `${world.roadStats.crossLinks} cross-country connector(s)` : "") +
    (world.roadRepair ? `, ${world.roadRepair.merges} repair merge(s)` : ""));

  // 2. every customer served by exactly one feeder
  let servedOnce = 0, unserved = 0;
  const feederSet = new Set(net.feeders.map(f => f.id));
  for (const c of customers) {
    const tx = c.tx >= 0 ? net.txs[c.tx] : null;
    if (tx && tx.feeder >= 0 && feederSet.has(tx.feeder)) servedOnce++;
    else unserved++;
  }
  add("Every customer served by exactly one feeder",
    servedOnce === customers.length && unserved === 0,
    `${servedOnce}/${customers.length} served, ${unserved} unserved, ${net.orphanTx} orphan TX`);

  // 3. feeder customer totals conserve the population
  const feederSum = net.feeders.reduce((s, f) => s + f.customers, 0);
  add("Feeder customer totals conserve population",
    Math.abs(feederSum - customers.length) < 1e-6,
    `sum over feeders = ${feederSum}, customers = ${customers.length}`);

  // 4. no customer sits in water (should be impossible by construction)
  const wet = customers.filter(c => world.terrain.waterAt(c.x, c.y) !== 0).length;
  add("No customers in water", wet === 0, `${wet} wet customer(s)`);

  // 5. roads cross the river only at generated bridges (line easements are
  // power spans, not roads — towers may cross the river, so they're exempt)
  let badCrossings = 0;
  let badInfo = "";
  let easements = 0;
  const t = world.terrain;
  for (const e of graph.edges) {
    if (e.cls === CLS_EASEMENT) { easements++; continue; }
    if (e.bridge) continue;
    // exact traversal — same test the road builder uses, so no sampling gaps
    const hit = t.segmentHits(graph.nx[e.a], graph.ny[e.a], graph.nx[e.b], graph.ny[e.b],
      (i) => t.water[i] === 2);
    if (hit) {
      badCrossings++;
      if (!badInfo) {
        badInfo = ` [first: cls=${e.cls} ` +
          `a=(${Math.round(graph.nx[e.a])},${Math.round(graph.ny[e.a])}) ` +
          `b=(${Math.round(graph.nx[e.b])},${Math.round(graph.ny[e.b])})]`;
      }
    }
  }
  add("Roads cross river only at bridges", badCrossings === 0,
    `${badCrossings} non-bridge river crossing edge(s), ${world.terrain.bridges.length} bridge(s), ` +
    `${easements} line easement(s) exempt` + badInfo);

  // 6. SAIDI is finite and positive
  const s0 = computeSaidi(net, emptyDevices());
  add("Baseline SAIDI finite & positive",
    isFinite(s0.overall) && s0.overall > 0,
    `${s0.overall.toFixed(1)} min/yr over ${net.feeders.length} feeders`);

  return checks;
}

// Conservation check for a fault scenario (run per playback, reported live).
export function checkFaultConservation(scenario) {
  const un = scenario.custUnaffected ?? 0;
  const tie = scenario.custTie ?? 0;
  return {
    name: "Fault classes conserve customers",
    pass: scenario.conservationOk,
    detail: `${scenario.custOut} out + ${scenario.custIso} isolatable + ${tie} backfed = ` +
      `${scenario.custOut + scenario.custIso + tie} of ${scenario.custAffected} affected` +
      (un > 0 ? ` (+ ${un} unaffected upstream of recloser = feeder total)` : ""),
  };
}

// Monotonicity check over a greedy log.
export function checkMonotone(log) {
  const placed = log.filter(l => !l.stopped);
  const ok = placed.every(l => l.monotone);
  return {
    name: "Greedy SAIDI monotone non-increasing",
    pass: ok,
    detail: placed.length
      ? `${placed.length} placements, ${placed[0].saidiBefore.toFixed(1)} → ${placed[placed.length - 1].saidiAfter.toFixed(1)} min/yr`
      : "no placements",
  };
}
