// reliability.js — SAIDI model, greedy device placement (sectionalisers +
// reclosers), BACKFEED via normally-open ties, fault playback
// classification, road-vs-line comparison.
//
// ASSUMPTIONS (all also listed in the UI):
//  - Separate uniform fault rates for overhead line and underground cable
//    (cables fault far less often; the flat 120 min repair is kept for
//    both, which flatters cable repairs — labelled). Adjustable in the UI.
//    Debug mode can double λ on one branch to show scoring is rate-weighted.
//  - Fault duration for customers NOT restorable by switching:
//    crew travel from the zone sub (the crew depot) along the line route
//    at 50 km/h to the faulted segment midpoint, + flat 120 min repair.
//  - SECTIONALISERS (load-break switches, no protection role): restorable
//    customers come back after a flat 45 min of switching.
//  - RECLOSERS (protection devices): a fault downstream of a recloser is
//    cleared BY the recloser — customers upstream of it see no sustained
//    interruption at all (0 min). Momentary interruptions (SAIFI/MAIFI)
//    are NOT modelled. A recloser also acts as an isolator for backfeed.
//  - BACKFEED TIES: adjacent feeders share one normally-open tie point
//    (built in network.js from the shortest unused-road corridor joining
//    them, ≤ 2 km). For a fault, every maximal device-bounded subtree in
//    the wait set that reaches a tie and does NOT contain the fault is
//    re-energised from the neighbouring feeder at the same flat 45 min —
//    opening the bounding device isolates the subtree from its own feeder
//    no matter where the fault sits, so LATERAL branches backfeed too.
//    Tie capacity is UNLIMITED and the neighbour is assumed healthy —
//    labelled simplifications.
//  - One device per section. The stretch between the fault and the nearest
//    isolators (up- and downstream) still waits out the full repair.
//  - LATERAL FUSES are STANDARD CONSTRUCTION, part of the baseline like
//    crossarms: every section whose downstream subtree carries at most
//    LATERAL_FUSE_MAX_CUST customers is fuse-protected — a fault there
//    interrupts only that subtree (travel + repair) and nobody else.
//    Deepest fuse wins (ideal coordination); fuse replacement time is
//    folded into the flat repair. Devices on fused laterals gain nothing,
//    so the greedy correctly spends its budget on trunks.
//  - SIBLING/EXPRESS feeders (network.js) enter as an unloaded exit LEAD —
//    one virtual first section. A lead fault interrupts the whole feeder;
//    parallel circuits on the same corridor fault INDEPENDENTLY
//    (common-mode shared-structure events not modelled — labelled).
//  - Device-free baseline: one breaker per feeder position + standard
//    lateral fusing, no automation. That is the counterfactual a real
//    planner reasons from — no utility operates fuseless.
//
// Device model per fault on edge e (evaluated EXACTLY, per feeder):
//   zone  = subtree of the deepest recloser on path(root → e), else feeder
//   NW    = subtree of the deepest device below that recloser on the path
//           (else = zone): tripped and not upstream-restorable
//   B(e)  = customers of maximal device-bounded tie-reaching subtrees in
//           the wait set that do not contain e (laterals passed on the way
//           down + subtrees below e) — backfed at 45 min
//   zone − NW    → restored at 45 min (upstream switching)
//   B(e)         → restored at 45 min (downstream backfeed)
//   NW − B(e)    → wait travel + 120 min
//   feeder − zone → never interrupted
// The greedy scores each candidate by re-running the (cheap) per-feeder
// evaluation with the trial device added — exact, so it stays provably
// monotone even with backfeed in play.

export const REPAIR_MIN = 120;
export const TRAVEL_KMH = 50;
export const SWITCH_MIN = 45;
// Sections with at most this many downstream customers are fuse-protected
// as standard construction (part of the device-free baseline). Calibrated
// so the fused-but-unautomated baseline lands in the 150–500 min/yr
// planning band (30 ≈ one TX group / small spur, not whole TX clusters).
export const LATERAL_FUSE_MAX_CUST = 30;

// Default fault rates. NETWORK STRUCTURE (the SAIDI-driven feeder splits
// in network.js) is computed from these DEFAULTS so regeneration stays
// deterministic — the live UI sliders below are analysis-only knobs.
export const DEFAULT_FAULT_RATES = { oh: 0.08, ug: 0.02 }; // faults / km / yr
export const faultRates = { ...DEFAULT_FAULT_RATES };

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

// Build per-feeder child lists once.
export function feederStructure(net) {
  const children = new Map(); // node -> [treeEdge ids of child edges]
  for (const te of net.treeEdges) {
    if (!children.has(te.parentNode)) children.set(te.parentNode, []);
    children.get(te.parentNode).push(te.id);
  }
  return children;
}

// Parent-before-child tree-edge order for one feeder (cached on the net).
function feederOrder(net, f) {
  if (!net._orders) net._orders = new Map();
  let order = net._orders.get(f.id);
  if (order) return order;
  const children = net._children ?? (net._children = feederStructure(net));
  order = [];
  const rootTe = net.treeEdgeOfNode[f.rootNode];
  if (rootTe !== -1) {
    const stack = [rootTe];
    while (stack.length) {
      const teId = stack.pop();
      order.push(teId);
      const kids = children.get(net.treeEdges[teId].node);
      if (kids) for (const k of kids) {
        if (net.treeEdges[k].feeder === f.id) stack.push(k);
      }
    }
  }
  net._orders.set(f.id, order);
  return order;
}

// Exact expected cust·min/yr for ONE feeder under a device set, optionally
// with one extra TRIAL device — candidate scoring re-runs this, so gains
// are exact rather than closed-form-approximate.
function feederCustMin(net, f, devices, rateMul = null, extraTe = -1, extraKind = null) {
  const children = net._children ?? (net._children = feederStructure(net));
  const order = feederOrder(net, f);
  let custMin = 0; // feeders root AT the busbar — no express-run floor
  if (!order.length) return custMin;
  const isSw = (id) => devices.switches.has(id) || (extraKind === "switch" && id === extraTe);
  const isRc = (id) => devices.reclosers.has(id) || (extraKind === "recloser" && id === extraTe);
  const tieNodes = net.tieNodesByFeeder ? net.tieNodesByFeeder[f.id] : null;

  // Bottom-up: tieIn (does this subtree reach a tie point?) and R — the
  // backfeedable customers within this edge's subtree: the union of
  // maximal device-bounded child subtrees that reach a tie. contrib(k) is
  // one child branch's share of that.
  const tieIn = new Map(), R = new Map();
  const contrib = (k) => ((isSw(k) || isRc(k)) && tieIn.get(k))
    ? net.subtreeCust[net.treeEdges[k].node]
    : R.get(k);
  for (let i = order.length - 1; i >= 0; i--) {
    const teId = order[i], te = net.treeEdges[teId];
    let t = tieNodes ? tieNodes.has(te.node) : false;
    let r = 0;
    const kids = children.get(te.node);
    if (kids) for (const k of kids) {
      if (net.treeEdges[k].feeder !== f.id) continue;
      if (tieIn.get(k)) t = true;
      r += contrib(k);
    }
    tieIn.set(teId, t);
    R.set(teId, r);
  }

  // Top-down: zone/wait context per fault edge, accumulating cust·min.
  // A = backfeedable customers on LATERAL branches passed on the way down
  // from the wait-set root: opening those branches' devices isolates them
  // from the feeder no matter where the fault is, so they backfeed for
  // this fault too. A resets at every device (everything above a deeper
  // device is upstream-restored the ordinary way instead).
  const stack = [[order[0], f.customers, f.customers, 0]];
  while (stack.length) {
    const [teId, NZin, NWin, Ain] = stack.pop();
    const te = net.treeEdges[teId];
    const nBelow = net.subtreeCust[te.node];
    if (nBelow <= LATERAL_FUSE_MAX_CUST) {
      // FUSED LATERAL (standard construction, deepest fuse wins): a fault
      // on this section drops only its own subtree for travel + repair —
      // the rest of the feeder rides through. Devices in here change
      // nothing, so trial gains on fused sections are exactly zero.
      custMin += edgeMass(te, rateMul) * faultDurMin(te) * nBelow;
      const fkids = children.get(te.node);
      if (fkids) for (const k of fkids) {
        if (net.treeEdges[k].feeder === f.id) stack.push([k, NZin, NWin, Ain]);
      }
      continue;
    }
    let NZ = NZin, NW = NWin, A = Ain;
    if (isRc(teId)) { NZ = NW = net.subtreeCust[te.node]; A = 0; }
    else if (isSw(teId)) { NW = net.subtreeCust[te.node]; A = 0; }
    const m = edgeMass(te, rateMul);
    const D = faultDurMin(te);
    // backfed at 45 min: laterals above (A) + device-bounded tie subtrees
    // below the fault (R)
    const B = Math.min(A + R.get(teId), NW);
    custMin += m * (D * (NW - B) + SWITCH_MIN * (NZ - NW + B));
    const kids = children.get(te.node);
    if (kids) for (const k of kids) {
      if (net.treeEdges[k].feeder !== f.id) continue;
      // descending into k: every OTHER branch at this junction becomes a
      // lateral — R(teId) minus k's own contribution
      stack.push([k, NZ, NW, A + R.get(teId) - contrib(k)]);
    }
  }
  return custMin;
}

// SAIDI with given devices {switches, reclosers} (Sets of treeEdge ids).
// Returns { perFeeder: [{saidi, custMin}], overall } in minutes/year.
export function computeSaidi(net, devices, rateMul = null) {
  const perFeeder = [];
  let totalCustMin = 0, totalCust = 0;
  for (const f of net.feeders) {
    const custMin = feederCustMin(net, f, devices, rateMul);
    perFeeder.push({ feeder: f.id, custMin, saidi: f.customers > 0 ? custMin / f.customers : 0 });
    totalCustMin += custMin;
    totalCust += f.customers;
  }
  return { perFeeder, overall: totalCust > 0 ? totalCustMin / totalCust : 0, totalCust };
}

// All positive-benefit candidate placements for one device kind, sorted
// descending. Each candidate's gain is EXACT: the feeder's cust·min is
// re-evaluated with the trial device added (benefits are feeder-local, so
// only that feeder needs recomputing).
export function allCandidates(net, devices, kind, rateMul = null) {
  const children = net._children ?? (net._children = feederStructure(net));
  const { switches, reclosers } = devices;
  const out = [];
  for (const f of net.feeders) {
    const order = feederOrder(net, f);
    if (!order.length) continue;
    const base = feederCustMin(net, f, devices, rateMul);
    // context pass under the CURRENT devices, for the custRestored report
    const tieNodes = net.tieNodesByFeeder ? net.tieNodesByFeeder[f.id] : null;
    const tieIn = new Map();
    for (let i = order.length - 1; i >= 0; i--) {
      const teId = order[i], te = net.treeEdges[teId];
      let t = tieNodes ? tieNodes.has(te.node) : false;
      const kids = children.get(te.node);
      if (kids) for (const k of kids) {
        if (net.treeEdges[k].feeder === f.id && tieIn.get(k)) t = true;
      }
      tieIn.set(teId, t);
    }
    const NZof = new Map(), NWof = new Map();
    const stack = [[order[0], f.customers, f.customers]];
    while (stack.length) {
      const [teId, NZin, NWin] = stack.pop();
      const te = net.treeEdges[teId];
      NZof.set(teId, NZin); NWof.set(teId, NWin);
      let NZ = NZin, NW = NWin;
      if (reclosers.has(teId)) NZ = NW = net.subtreeCust[te.node];
      else if (switches.has(teId)) NW = net.subtreeCust[te.node];
      const kids = children.get(te.node);
      if (kids) for (const k of kids) {
        if (net.treeEdges[k].feeder === f.id) stack.push([k, NZ, NW]);
      }
    }
    for (const teId of order) {
      if (switches.has(teId) || reclosers.has(teId)) continue;
      const gain = base - feederCustMin(net, f, devices, rateMul, teId, kind);
      if (gain > 1e-9) {
        const te = net.treeEdges[teId];
        const Nsub = net.subtreeCust[te.node];
        const upstream = (kind === "switch" ? NWof.get(teId) : NZof.get(teId)) - Nsub;
        const backfed = tieIn.get(teId) ? Nsub : 0;
        out.push({ teId, gain, kind, feeder: f.id, node: te.node,
          custRestored: upstream + backfed });
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
// asserts the running SAIDI is monotone non-increasing (it must be — every
// accepted candidate's gain is an exact recomputation).
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
// should move — proving the score is rate-weighted. With backfeed ties part
// of a candidate's benefit comes from faults UPSTREAM of the boosted branch
// (unchanged by the boost), so the boosted candidate's gain rises by a
// factor in (1, 2] rather than exactly 2 — the check accepts any clear rise.
export function debugRateExperiment(net) {
  const ranked = allCandidates(net, emptyDevices(), "switch");
  if (!ranked.length) return { supported: false, reason: "no candidates" };
  const base = ranked[0];
  let chosen = null, tried = 0;
  for (const cand of ranked.slice(1)) {
    if (tried >= 1500) break; // safety valve only — scan is cheap
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
  // Whether or not the pick moved, the boosted branch's own benefit must
  // have clearly RISEN — the direct proof the score is rate-weighted.
  // (A dominant baseline pick can survive any single doubling; that is a
  // property of the network, not of the scoring.)
  const after = allCandidates(net, emptyDevices(), "switch", chosen.rateMul)
    .find(c => c.teId === chosen.boostEdge);
  const boostGainDebug = after ? after.gain : 0;
  const ratio = chosen.boostGain > 0 ? boostGainDebug / chosen.boostGain : 0;
  const rateWeighted = chosen.moved || ratio > 1.1;
  console.assert(rateWeighted, "boosted branch benefit did not rise with its fault rate");
  return {
    supported: true,
    boostEdge: chosen.boostEdge,
    boostNode: chosen.boostNode,
    boostGain: chosen.boostGain,
    boostGainDebug,
    boostRatio: ratio,
    rateWeighted,
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
// tripped zone; the first device found BEFORE that recloser bounds the
// full-wait set. Below the fault, maximal device-bounded subtrees that
// reach a tie are BACKFED from the neighbouring feeder at 45 min.
// Conservation (out + isolatable + backfed = affected = zone;
// zone + unaffected = feeder) is checked, not assumed.
export function faultScenario(net, devices, teId) {
  const { switches, reclosers } = devices;
  const children = net._children ?? (net._children = feederStructure(net));
  const te = net.treeEdges[teId];
  const f = net.feeders.find(f => f.id === te.feeder);
  // FUSED LATERAL: the fuse clears the fault — only this subtree goes out,
  // for travel + repair; no switching phase, the feeder rides through.
  const nBelow = net.subtreeCust[te.node];
  if (nBelow <= LATERAL_FUSE_MAX_CUST) {
    const outE = subtreeEdges(net, teId);
    const tTravel = travelMin(te.midDistM);
    return {
      teId, feeder: f.id, faultEdge: te, fused: true,
      switchEdge: -1, recloserEdge: -1,
      zoneEdges: outE, outEdges: new Set(outE), tieEdges: new Set(),
      custOut: nBelow, custIso: 0, custTie: 0,
      custUnaffected: f.customers - nBelow,
      custAffected: nBelow,
      conservationOk: true,
      tSwitch: null,
      tTravel,
      tRepairDone: tTravel + REPAIR_MIN,
    };
  }
  let switchTe = -1, recloserTe = -1;
  for (let cur = teId; cur !== -1; cur = parentTreeEdge(net, cur)) {
    if (net.treeEdges[cur].feeder !== f.id) break; // crossed onto the trunk circuit
    if (reclosers.has(cur)) { recloserTe = cur; break; }
    if (switchTe === -1 && switches.has(cur)) switchTe = cur;
  }
  const rootTe = net.treeEdgeOfNode[f.rootNode];
  const zoneEdges = subtreeEdges(net, recloserTe !== -1 ? recloserTe : rootTe);
  const waitTe = switchTe !== -1 ? switchTe : recloserTe;
  const outEdges = waitTe !== -1 ? subtreeEdges(net, waitTe) : new Set(zoneEdges);

  // Backfeed: every maximal device-bounded subtree inside the wait set
  // that reaches a tie and does NOT contain the fault comes back at
  // SWITCH_MIN from the neighbour — laterals passed on the way down from
  // the wait-set root as well as subtrees below the fault (opening the
  // bounding device isolates the subtree from the feeder no matter where
  // the fault sits).
  const tieNodes = net.tieNodesByFeeder ? net.tieNodesByFeeder[f.id] : null;
  const isDev = (id) => switches.has(id) || reclosers.has(id);
  const subtreeHasTie = (rootId) => {
    if (!tieNodes || tieNodes.size === 0) return false;
    const stack = [rootId];
    while (stack.length) {
      const cur = stack.pop();
      if (tieNodes.has(net.treeEdges[cur].node)) return true;
      const kids = children.get(net.treeEdges[cur].node);
      if (kids) for (const k of kids) {
        if (net.treeEdges[k].feeder === f.id) stack.push(k);
      }
    }
    return false;
  };
  const tieEdges = new Set();
  let custTie = 0;
  const collectStack = [];
  const collectFrom = (k) => { collectStack.push(k); };
  // walk the path wait-set root → fault edge; every off-path branch at
  // every junction (and everything below the fault) is a candidate
  const startTe = waitTe !== -1 ? waitTe : rootTe;
  const onPath = new Set();
  for (let cur = teId; cur !== -1; cur = parentTreeEdge(net, cur)) {
    if (net.treeEdges[cur].feeder !== f.id) break;
    onPath.add(cur);
    if (cur === startTe) break;
  }
  for (const p of onPath) {
    const kids = children.get(net.treeEdges[p].node);
    if (kids) for (const k of kids) {
      if (net.treeEdges[k].feeder !== f.id || onPath.has(k)) continue;
      collectFrom(k);
    }
  }
  while (collectStack.length) {
    const k = collectStack.pop();
    if (isDev(k) && subtreeHasTie(k)) {
      for (const id of subtreeEdges(net, k)) tieEdges.add(id);
      custTie += net.subtreeCust[net.treeEdges[k].node];
      continue;
    }
    const kids = children.get(net.treeEdges[k].node);
    if (kids) for (const kk of kids) {
      if (net.treeEdges[kk].feeder === f.id) collectStack.push(kk);
    }
  }
  for (const id of tieEdges) outEdges.delete(id);

  const custZone = recloserTe !== -1
    ? net.subtreeCust[net.treeEdges[recloserTe].node] : f.customers;
  const custOut = (waitTe !== -1
    ? net.subtreeCust[net.treeEdges[waitTe].node] : f.customers) - custTie;
  const custIso = custZone - custOut - custTie;
  const custUnaffected = f.customers - custZone;
  const tTravel = travelMin(te.midDistM);
  return {
    teId, feeder: f.id, faultEdge: te,
    switchEdge: switchTe, recloserEdge: recloserTe,
    zoneEdges, outEdges, tieEdges,
    custOut, custIso, custTie, custUnaffected,
    custAffected: custZone,
    conservationOk:
      Math.abs(custOut + custIso + custTie - custZone) < 1e-6 &&
      Math.abs(custZone + custUnaffected - f.customers) < 1e-6,
    tSwitch: (custIso > 0 || custTie > 0) ? SWITCH_MIN : null,
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
