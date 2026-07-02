// reliability.js — SAIDI model, greedy device placement (sectionalisers +
// reclosers), fault playback classification, road-vs-line comparison.
//
// ASSUMPTIONS (all also listed in the UI):
//  - Uniform fault rate λ = 0.10 faults / km / year on every HV segment
//    (no weather, vegetation or asset-age variation). Debug mode can
//    double λ on one branch to show the greedy is rate-weighted.
//  - Fault duration for customers NOT restorable by switching:
//    crew travel from the zone sub (the crew depot) along roads at
//    50 km/h to the faulted segment midpoint, + flat 120 min repair.
//  - SECTIONALISERS (load-break switches, no protection role): opening the
//    deepest switch upstream of the fault and reclosing the tripped device
//    restores everyone else in the tripped zone after a flat 45 min.
//    (Flat time keeps greedy placement provably monotone.)
//  - RECLOSERS (protection devices): a fault downstream of a recloser is
//    cleared BY the recloser — customers upstream of it see no sustained
//    interruption at all (0 min). Momentary interruptions (SAIFI/MAIFI)
//    are NOT modelled. Sectionalising still works inside the tripped zone.
//  - One device per section; radial only — no backfeed, so customers
//    downstream of the opened device wait for the full repair regardless.
//  - Device-free baseline: one breaker at the sub, no fuses.
//
// Device model per fault on edge e:
//   zone  = subtree of the deepest recloser on path(root → e), else feeder
//   wait  = subtree of the deepest sectionaliser BELOW that recloser on the
//           path (else = zone): these customers wait travel + 120 min
//   zone − wait      → restored at 45 min
//   feeder − zone    → never interrupted

export const REPAIR_MIN = 120;
export const TRAVEL_KMH = 50;
export const SWITCH_MIN = 45;

// ASSUMPTION: separate uniform fault rates for overhead line and
// underground cable (cables fault far less often; the flat 120 min repair
// is kept for both, which flatters cable repairs — labelled in UI).
// Adjustable at runtime via the UI.
export const faultRates = { oh: 0.10, ug: 0.03 }; // faults / km / yr

export function setFaultRates(oh, ug) {
  if (isFinite(oh) && oh >= 0) faultRates.oh = oh;
  if (isFinite(ug) && ug >= 0) faultRates.ug = ug;
}

const travelMin = (distM) => (distM / 1000) / TRAVEL_KMH * 60;
export const faultDurMin = (te) => travelMin(te.midDistM) + REPAIR_MIN;

export const emptyDevices = () => ({ switches: new Set(), reclosers: new Set() });

// Fault "mass" of a tree edge: expected faults/yr, with optional rate
// multiplier (debug mode).
function edgeMass(te, rateMul) {
  const base = te.underground ? faultRates.ug : faultRates.oh;
  return base * (te.lenM / 1000) * (rateMul ? rateMul(te) : 1);
}

// Express run (sub → feeder head, a parallel circuit on the same roads):
// faults there black out the WHOLE feeder and no in-feeder device helps,
// so it contributes a constant, un-switchable customer-minute floor.
function expressCustMin(f) {
  const m = faultRates.oh * f.expressOhKm + faultRates.ug * f.expressUgKm;
  if (m <= 0) return 0;
  return m * (travelMin(f.expressMidM) + REPAIR_MIN) * f.customers;
}

// Build per-feeder child lists once.
export function feederStructure(net) {
  const children = new Map(); // node -> [treeEdge ids of child edges]
  for (const te of net.treeEdges) {
    if (!children.has(te.parentNode)) children.set(te.parentNode, []);
    children.get(te.parentNode).push(te.id);
  }
  return children;
}

// SAIDI with given devices {switches, reclosers} (Sets of treeEdge ids).
// Returns { perFeeder: [{saidi, custMin}], overall } in minutes/year.
export function computeSaidi(net, devices, rateMul = null) {
  const children = net._children ?? (net._children = feederStructure(net));
  const { switches, reclosers } = devices;
  const perFeeder = [];
  let totalCustMin = 0, totalCust = 0;
  for (const f of net.feeders) {
    const Nf = f.customers;
    let custMin = expressCustMin(f);
    // DFS carrying (NZ = tripped-zone population, NW = full-wait population).
    const stack = [[net.treeEdgeOfNode[f.rootNode], Nf, Nf]];
    while (stack.length) {
      const [teId, NZin, NWin] = stack.pop();
      const te = net.treeEdges[teId];
      let NZ = NZin, NW = NWin;
      if (reclosers.has(teId)) NZ = NW = net.subtreeCust[te.node];
      else if (switches.has(teId)) NW = net.subtreeCust[te.node];
      const m = edgeMass(te, rateMul);
      const D = faultDurMin(te);
      custMin += m * (D * NW + SWITCH_MIN * (NZ - NW));
      const kids = children.get(te.node);
      if (kids) for (const k of kids) {
        if (net.treeEdges[k].feeder === f.id) stack.push([k, NZ, NW]);
      }
    }
    perFeeder.push({ feeder: f.id, custMin, saidi: Nf > 0 ? custMin / Nf : 0 });
    totalCustMin += custMin;
    totalCust += Nf;
  }
  return { perFeeder, overall: totalCust > 0 ? totalCustMin / totalCust : 0, totalCust };
}

// All positive-benefit candidate placements for one device kind, sorted
// descending. Closed-form marginal customer-minute savings:
//   switch at c:   (NW − N_sub(c)) · Σ_A m·(D − 45)
//   recloser at c: (NW − N_sub(c)) · Σ_A m·D
//                  + 45·(NZ − NW) · Σ_A m  +  45·(NZ − N_sub(c)) · Σ_B m
// where A = edges of subtree(c) with no device between c and e (they share
// c's wait context NW), and B = edges below a deeper SWITCH but no deeper
// recloser (their wait population is unchanged; only the zone shrinks).
// Subtrees below a deeper RECLOSER are unaffected entirely.
export function allCandidates(net, devices, kind, rateMul = null) {
  const children = net._children ?? (net._children = feederStructure(net));
  const { switches, reclosers } = devices;
  const out = [];
  for (const f of net.feeders) {
    const rootTe = net.treeEdgeOfNode[f.rootNode];
    const order = [];
    const NZof = new Map(), NWof = new Map(); // context just ABOVE each edge
    const stack = [[rootTe, f.customers, f.customers]];
    while (stack.length) {
      const [teId, NZin, NWin] = stack.pop();
      const te = net.treeEdges[teId];
      NZof.set(teId, NZin); NWof.set(teId, NWin);
      let NZ = NZin, NW = NWin;
      if (reclosers.has(teId)) NZ = NW = net.subtreeCust[te.node];
      else if (switches.has(teId)) NW = net.subtreeCust[te.node];
      order.push(teId);
      const kids = children.get(te.node);
      if (kids) for (const k of kids) {
        if (net.treeEdges[k].feeder === f.id) stack.push([k, NZ, NW]);
      }
    }
    // Post-order truncated sums (order[] is parent-before-child).
    const SAm = new Map(), SAmD = new Map(), SRm = new Map();
    for (let i = order.length - 1; i >= 0; i--) {
      const teId = order[i];
      const te = net.treeEdges[teId];
      const m = edgeMass(te, rateMul), D = faultDurMin(te);
      let sam = m, samd = m * D, srm = m;
      const kids = children.get(te.node);
      if (kids) for (const k of kids) {
        if (net.treeEdges[k].feeder !== f.id) continue;
        if (!reclosers.has(k)) {
          srm += SRm.get(k);
          if (!switches.has(k)) { sam += SAm.get(k); samd += SAmD.get(k); }
        }
      }
      SAm.set(teId, sam); SAmD.set(teId, samd); SRm.set(teId, srm);
    }
    for (const teId of order) {
      if (switches.has(teId) || reclosers.has(teId)) continue;
      const te = net.treeEdges[teId];
      const Nsub = net.subtreeCust[te.node];
      const NW = NWof.get(teId), NZ = NZof.get(teId);
      const gain = kind === "switch"
        ? (NW - Nsub) * (SAmD.get(teId) - SWITCH_MIN * SAm.get(teId))
        : (NW - Nsub) * SAmD.get(teId)
          + SWITCH_MIN * (NZ - NW) * SAm.get(teId)
          + SWITCH_MIN * (NZ - Nsub) * (SRm.get(teId) - SAm.get(teId));
      if (gain > 1e-9) {
        out.push({ teId, gain, kind, feeder: f.id, node: te.node,
          custRestored: (kind === "switch" ? NW : NZ) - Nsub });
      }
    }
  }
  return out.sort((a, b) => b.gain - a.gain);
}

export function bestCandidate(net, devices, kind, rateMul = null) {
  const all = allCandidates(net, devices, kind, rateMul);
  return all.length ? all[0] : null;
}

// Greedy placement of `count` devices of one kind. Returns log entries;
// asserts the running SAIDI is monotone non-increasing.
export function greedyPlace(net, count, kind, existing = null, rateMul = null) {
  const devices = {
    switches: new Set(existing?.switches ?? []),
    reclosers: new Set(existing?.reclosers ?? []),
  };
  const log = [];
  let prev = computeSaidi(net, devices, rateMul).overall;
  const baseline = prev;
  for (let k = 0; k < count; k++) {
    const best = bestCandidate(net, devices, kind, rateMul);
    if (!best) { log.push({ stopped: true, reason: "no candidate with positive benefit" }); break; }
    (kind === "switch" ? devices.switches : devices.reclosers).add(best.teId);
    const now = computeSaidi(net, devices, rateMul).overall;
    const entry = {
      step: k + 1, kind, teId: best.teId, node: best.node, feeder: best.feeder,
      custRestored: Math.round(best.custRestored),
      saidiBefore: prev, saidiAfter: now, benefitMin: prev - now,
      monotone: now <= prev + 1e-6,
    };
    log.push(entry);
    if (!entry.monotone) {
      console.assert(false, "SAIDI increased at greedy step", entry);
    }
    prev = now;
  }
  return { devices, log, baseline, final: prev };
}

// Edge ids in the subtree hanging off a tree edge (the edge itself
// included), staying WITHIN that edge's feeder — deeper cut feeders are
// separate circuits and not part of this subtree.
export function subtreeEdges(net, teId) {
  const children = net._children ?? (net._children = feederStructure(net));
  const fid = net.treeEdges[teId].feeder;
  const set = new Set();
  const stack = [teId];
  while (stack.length) {
    const cur = stack.pop();
    set.add(cur);
    const kids = children.get(net.treeEdges[cur].node);
    if (kids) for (const k of kids) {
      if (net.treeEdges[k].feeder === fid) stack.push(k);
    }
  }
  return set;
}

// ---- debug mode: double one branch's fault rate; the greedy's first pick
// should move — proving the score is rate-weighted. The boosted branch is
// the subtree of the strongest candidate whose doubling actually moves the
// global first pick; doubling a branch exactly doubles that candidate's own
// benefit, so scanning the top candidates finds a mover unless the baseline
// pick dominates everything by more than 2x (then reported honestly).
export function debugRateExperiment(net) {
  const ranked = allCandidates(net, emptyDevices(), "switch");
  if (!ranked.length) return { supported: false, reason: "no candidates" };
  const base = ranked[0];
  let chosen = null, tried = 0;
  for (const cand of ranked.slice(1)) {
    if (tried >= 120) break;
    const branch = subtreeEdges(net, cand.teId);
    // An ANCESTOR of the base pick can never move it (its boost doubles the
    // base's benefit too); descendants and disjoint branches can.
    if (branch.has(base.teId)) continue;
    tried++;
    const rateMul = (te) => (branch.has(te.id) ? 2 : 1);
    const withDebug = bestCandidate(net, emptyDevices(), "switch", rateMul);
    const moved = withDebug.teId !== base.teId;
    if (!chosen || (moved && !chosen.moved)) {
      chosen = { boostEdge: cand.teId, boostNode: cand.node, boostGain: cand.gain,
        boostFeeder: cand.feeder, branch, rateMul, debugPick: withDebug, moved };
    }
    if (moved) break;
  }
  if (!chosen) return { supported: false, reason: "single-candidate network" };
  return {
    supported: true,
    boostEdge: chosen.boostEdge,
    boostNode: chosen.boostNode,
    boostGain: chosen.boostGain,
    boostFeeder: chosen.boostFeeder,
    branch: chosen.branch,
    basePick: base,
    debugPick: chosen.debugPick,
    moved: chosen.moved,
    rateMul: chosen.rateMul,
  };
}

// ---- fault playback classification -------------------------------------
// Walk up from the faulted edge: the first recloser found bounds the
// tripped zone; the first switch found BEFORE that recloser bounds the
// full-wait set. Conservation (out + isolatable = affected = zone;
// zone + unaffected = feeder) is checked, not assumed.
export function faultScenario(net, devices, teId) {
  const { switches, reclosers } = devices;
  const te = net.treeEdges[teId];
  const f = net.feeders.find(f => f.id === te.feeder);
  let switchTe = -1, recloserTe = -1;
  for (let cur = teId; cur !== -1; cur = parentTreeEdge(net, cur)) {
    if (net.treeEdges[cur].feeder !== f.id) break; // crossed onto the trunk circuit
    if (reclosers.has(cur)) { recloserTe = cur; break; }
    if (switchTe === -1 && switches.has(cur)) switchTe = cur;
  }
  const rootTe = net.treeEdgeOfNode[f.rootNode];
  const zoneEdges = subtreeEdges(net, recloserTe !== -1 ? recloserTe : rootTe);
  const waitTe = switchTe !== -1 ? switchTe : recloserTe;
  const outEdges = waitTe !== -1 ? subtreeEdges(net, waitTe) : zoneEdges;
  const custZone = recloserTe !== -1
    ? net.subtreeCust[net.treeEdges[recloserTe].node] : f.customers;
  const custOut = waitTe !== -1
    ? net.subtreeCust[net.treeEdges[waitTe].node] : f.customers;
  const custIso = custZone - custOut;
  const custUnaffected = f.customers - custZone;
  const tTravel = travelMin(te.midDistM);
  return {
    teId, feeder: f.id, faultEdge: te,
    switchEdge: switchTe, recloserEdge: recloserTe,
    zoneEdges, outEdges,
    custOut, custIso, custUnaffected,
    custAffected: custZone,
    conservationOk:
      Math.abs(custOut + custIso - custZone) < 1e-6 &&
      Math.abs(custZone + custUnaffected - f.customers) < 1e-6,
    tSwitch: switchTe !== -1 && custIso > 0 ? SWITCH_MIN : null,
    tTravel,
    tRepairDone: tTravel + REPAIR_MIN,
  };
}

export function parentTreeEdge(net, teId) {
  const te = net.treeEdges[teId];
  return net.treeEdgeOfNode[te.parentNode] ?? -1;
}

// ---- road vs straight-line ----------------------------------------------
// Per feeder: rate-weighted mean road distance sub→fault vs straight-line
// distance. Bridges + terrain detours should push this above 1.
export function roadVsLine(net, graph) {
  const out = [];
  for (const f of net.feeders) {
    const sub = net.subs[f.sub];
    let wRoad = 0, wLine = 0, w = 0;
    for (const teId of f.edges) {
      const te = net.treeEdges[teId];
      const m = edgeMass(te, null);
      const mx = (graph.nx[te.node] + graph.nx[te.parentNode]) / 2;
      const my = (graph.ny[te.node] + graph.ny[te.parentNode]) / 2;
      const line = Math.hypot(mx - sub.x, my - sub.y);
      wRoad += m * te.midDistM;
      wLine += m * Math.max(1, line);
      w += m;
    }
    out.push({
      feeder: f.id, sub: f.sub,
      roadKm: wRoad / w / 1000, lineKm: wLine / w / 1000,
      ratio: wRoad / Math.max(1, wLine),
    });
  }
  return out;
}
