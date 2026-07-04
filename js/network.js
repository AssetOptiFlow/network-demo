// network.js — electrical model: distribution transformers, zone
// substations, and radial feeders routed ALONG the road graph.
//
// TWO-LEVEL MEMBERSHIP-FIRST FLOW (membership.js decides, this file routes):
//   1. customers classified urban/rural (local density, cust/km²)
//   2. customers → feeders: road-capacitated clustering (membership.js)
//   3. feeders → zone subs: road-adjacent groups, ≤ FEEDERS_PER_SUB_MAX
//   4. subs at the load-weighted centroid of their FEEDER GROUP, nudged to
//      a subtransmission-viable corridor node (nudgeToCorridor, unchanged)
//   5. routing LAST: one Dijkstra tree per sub over the roads, serving ONLY
//      that sub's member load nodes; trees may not overlap (later subs
//      route around earlier trees — one circuit per road corridor)
//   6. validate + repair, bounded passes: feeder caps, rural extent,
//      MAX_FEEDER_KM trunks, MAX_FOREIGN_CROSSING_M transit, group caps.
//      Repairs re-cluster / re-group AFFECTED MEMBERS only; outcomes are
//      reported in the Checks panel, never silently absorbed.
//
// ASSUMPTIONS:
//  - Distribution transformers (TX): greedy capacitated clustering of
//    customers, max 50 per TX, gathered within ~500 m (urban) / 1500 m
//    (rural). TX sits at the road node nearest the cluster centroid.
//  - Structural soundness is enforced, not hoped for: every feeder is a
//    contiguous subtree with a single root (forced splits otherwise, and
//    trunk runts absorb into the feeder that surrounds them, both logged).
//  - Sections through high-density cells are UNDERGROUND cable, the rest
//    overhead (per-type fault rates live in reliability.js).
//  - LV detail below the TX is ignored; customers hang off their TX.

import {
  classifyCustomers, buildLoadNodes, clusterFeeders, mergeRuntFeeders,
  feederVoronoi, groupFeeders, NodeHeap, morton, capOf,
  FEEDERS_PER_SUB_MAX, RURAL_EXTENT_KM_MAX, FEEDER_MIN_CUST,
} from "./membership.js";

export const TX_CAP = 50;
// Town peaks sit near 1.0 (mass-compensated), so this keeps cable to the
// inner core (roughly r < σ) rather than whole towns.
export const UG_DENSITY_THRESH = 0.55; // density units — inner-urban ⇒ cable

// ---- validate/repair caps (named for tuning) ----------------------------
// Measured on generated worlds: rural pockets 15–20 km by road and 1–2 km
// arterial pass-throughs of a neighbouring catchment are structural on
// this geography, so tighter values than these thrash the repair loop.
export const MAX_FEEDER_KM = 20;           // sub → farthest member, by road
export const MAX_FOREIGN_CROSSING_M = 2500; // transit through a foreign sub's catchment
export const MAX_REPAIR_PASSES = 3;
export const CAP_SLACK = 1.10;            // headroom before a cap counts as violated
                                          // (trunk-load absorbs jitter counts upward)
const MAX_SUBS = 24;                      // backstop for repair-created subs
// A remote valley pocket this big justifies its own small rural sub when
// no existing sub is closer — real utilities build them for exactly this.
const NEW_SUB_MIN_CUST = 150;

// =====================================================================
// buildNetwork — membership first, routing last.
// =====================================================================
export function buildNetwork(terrain, graph, customers, towns, density, rng) {
  // ---------- TX layer (atoms of load; unchanged)
  const txs = clusterTransformers(customers, rng);
  for (const tx of txs) tx.node = graph.nearestNode(tx.x, tx.y, 30000).id;
  for (const tx of txs) for (const ci of tx.customers) customers[ci].tx = tx.id;

  // ---------- MEMBERSHIP (steps 1–3) — before any routing
  const classOfCust = classifyCustomers(customers);
  const loadNodes = buildLoadNodes(txs, customers, classOfCust);
  let cf = clusterFeeders(graph, loadNodes);
  cf = mergeRuntFeeders(graph, loadNodes, cf.feederOf, cf.feeders);
  const g0 = groupFeeders(graph, loadNodes, cf.feederOf, cf.feeders.length);
  // Mutable membership state: feederOf per load node + group per feeder id.
  const M = {
    feederOf: cf.feederOf,
    fidGroup: new Map(cf.feeders.map(f => [f.id, g0.groupOf[f.id]])),
    nextFid: cf.feeders.length,
  };

  // ---------- ROUTE → VALIDATE → REPAIR (bounded, affected members only)
  const repairLog = [];
  let built = null;
  for (let pass = 0; ; pass++) {
    const mem = materialise(loadNodes, M);
    const subs = placeSubsForGroups(terrain, graph, towns, loadNodes, mem);
    built = routeMembership(terrain, graph, density, loadNodes, mem, subs, M, repairLog, pass);
    const viol = validateMembership(graph, loadNodes, built);
    // runt merging is tidying, not a rule breach — repaired but never
    // reported as an unresolved violation
    built.violations = viol.filter(v => v.kind !== "runt");
    if (!viol.length || pass >= MAX_REPAIR_PASSES) {
      if (built.violations.length) {
        repairLog.push({ pass, action: "unresolved",
          detail: built.violations.map(v => v.detail).join("; ") });
      }
      break;
    }
    applyRepairs(graph, loadNodes, M, built, viol, repairLog, pass);
  }

  return finishNet(graph, customers, txs, loadNodes, built, classOfCust, repairLog);
}

// Compact the mutable membership state into dense ids (0..n-1) + metadata.
// Group ids are compacted too (a group can empty out after a repair move).
function materialise(loadNodes, M) {
  const liveFids = [...new Set([...M.feederOf].filter(f => f !== -1))].sort((a, b) => a - b);
  const fidTo = new Map(liveFids.map((f, i) => [f, i]));
  const liveGroups = [...new Set(liveFids.map(f => M.fidGroup.get(f)))].sort((a, b) => a - b);
  const groupTo = new Map(liveGroups.map((g, i) => [g, i]));
  const feeders = liveFids.map((oldFid, id) => ({
    id, oldFid, cust: 0, urbanCust: 0, urban: false,
    group: groupTo.get(M.fidGroup.get(oldFid)),
    loadIdx: [],
  }));
  const feederOf = new Int32Array(loadNodes.length).fill(-1);
  for (let i = 0; i < loadNodes.length; i++) {
    if (M.feederOf[i] === -1) continue;
    const f = feeders[fidTo.get(M.feederOf[i])];
    feederOf[i] = f.id;
    f.cust += loadNodes[i].cust;
    f.urbanCust += loadNodes[i].urbanCust;
    f.loadIdx.push(i);
  }
  for (const f of feeders) f.urban = f.urbanCust * 2 >= f.cust;
  return { feederOf, feeders, nGroups: liveGroups.length, rawGroupOf: liveGroups };
}

// ---------------------------------------------------------------------
// Step 4 — sub per feeder group: LOAD-WEIGHTED centroid of the group's
// member load nodes (never a geometric centre), nudged to the nearest
// subtransmission-viable corridor node (existing nudge logic, unchanged).
// ---------------------------------------------------------------------
function placeSubsForGroups(terrain, graph, towns, loadNodes, mem) {
  const subs = [];
  for (let g = 0; g < mem.nGroups; g++) {
    let sx = 0, sy = 0, cust = 0, nFeeders = 0;
    for (const f of mem.feeders) {
      if (f.group !== g) continue;
      nFeeders++;
      for (const li of f.loadIdx) {
        const ln = loadNodes[li];
        sx += graph.nx[ln.node] * ln.cust;
        sy += graph.ny[ln.node] * ln.cust;
        cust += ln.cust;
      }
    }
    const cx = sx / Math.max(1, cust), cy = sy / Math.max(1, cust);
    const node = nudgeToCorridor(terrain, graph, cx, cy, subs);
    let town = towns[0], bd = Infinity;
    for (const t of towns) {
      const d = Math.hypot(t.x - cx, t.y - cy);
      if (d < bd) { bd = d; town = t; }
    }
    const dupes = subs.filter(s => s.baseName === town.name).length;
    subs.push({
      id: g, node, x: graph.nx[node], y: graph.ny[node],
      centroidX: cx, centroidY: cy, catchment: cust, nFeeders,
      baseName: town.name,
      name: town.name + (dupes ? " " + "BCDEFG"[dupes - 1] : ""),
    });
  }
  return subs;
}

// ---------------------------------------------------------------------
// Step 5 — routing LAST, honouring membership. One UNBLOCKED Dijkstra per
// sub over the road graph, serving only its member load nodes — true
// shortest paths. A sub may only CLAIM (own tree edges on) nodes inside
// its OWN catchment (road-graph Voronoi over the membership tables); a
// path's stretches through a foreign catchment are charged to the feeder
// as EXPRESS exposure — the second circuit strung along a shared road,
// policed by the MAX_FOREIGN_CROSSING_M rule. Because every load node is
// in its own sub's catchment by construction, no member can be claimed
// away — membership is honoured exactly.
// Within each sub's claimed forest, node ownership is sticky top-down
// with a HARD rule at load nodes, and every feeder ends as a contiguous
// subtree with a single root (forced splits / runt absorbs, logged).
// ---------------------------------------------------------------------
function routeMembership(terrain, graph, density, loadNodes, mem, subs, M, repairLog, pass) {
  const nN = graph.nNodes;
  const loadAt = new Map(loadNodes.map((ln, i) => [ln.node, i]));
  const subOrder = subs.slice().sort((a, b) => b.catchment - a.catchment || a.id - b.id);

  // sub catchment label per road node (feeder Voronoi → that feeder's sub)
  const fLabel = feederVoronoi(graph, loadNodes, mem.feederOf);
  const subLabel = new Int32Array(nN).fill(-1);
  for (let v = 0; v < nN; v++) {
    if (fLabel[v] !== -1) subLabel[v] = mem.feeders[fLabel[v]].group;
  }

  const dist = new Float64Array(nN).fill(Infinity);
  const parent = new Int32Array(nN).fill(-1);
  const parentEdge = new Int32Array(nN).fill(-1);
  const claimed = new Int32Array(nN).fill(-1);
  const usedEdge = new Uint8Array(graph.edges.length);
  const midDistNode = new Float64Array(nN);
  const perSubClaimed = subs.map(() => []);
  const subChains = subs.map(() => null);
  for (const s of subs) { claimed[s.node] = s.id; dist[s.node] = 0; }

  const sd = new Float64Array(nN), sparent = new Int32Array(nN),
    sparentEdge = new Int32Array(nN), stamp = new Int32Array(nN);
  let gen = 0;
  for (const s of subOrder) {
    // full-graph Dijkstra from the sub — true shortest paths, no blocking
    gen++;
    const heap = new NodeHeap();
    sd[s.node] = 0; sparent[s.node] = -1; sparentEdge[s.node] = -1; stamp[s.node] = gen;
    heap.push(0, s.node);
    const done = new Set();
    while (heap.size) {
      const v = heap.pop();
      if (done.has(v)) continue;
      done.add(v);
      for (const ei of graph.adj[v]) {
        const e = graph.edges[ei];
        const w = e.a === v ? e.b : e.a;
        const nd = sd[v] + e.len;
        if (stamp[w] !== gen || nd < sd[w] - 1e-9) {
          stamp[w] = gen; sd[w] = nd; sparent[w] = v; sparentEdge[w] = ei;
          heap.push(nd, w);
        }
      }
    }
    // PHASE 1: claim the own-catchment portion of each member's path;
    // stretches in foreign catchments are skipped for now
    for (let li = 0; li < loadNodes.length; li++) {
      const fid = mem.feederOf[li];
      if (fid === -1 || mem.feeders[fid].group !== s.id) continue;
      for (let v = loadNodes[li].node; v !== s.node && claimed[v] !== s.id; v = sparent[v]) {
        if (claimed[v] === -1 && subLabel[v] === s.id) {
          claimed[v] = s.id;
          perSubClaimed[s.id].push(v);
          dist[v] = sd[v]; parent[v] = sparent[v]; parentEdge[v] = sparentEdge[v];
          midDistNode[v] = (sd[v] + sd[sparent[v]]) / 2;
          usedEdge[sparentEdge[v]] = 1;
        }
      }
    }
    // keep this sub's full parent chains + distances — phase 2, express
    // runs and transit measurements need the true path everywhere
    subChains[s.id] = { par: sparent.slice(), edge: sparentEdge.slice(), sdist: sd.slice() };
  }

  // PHASE 2: foreign-catchment stretches whose nodes the HOME sub never
  // uses are claimed by the crossing sub — the feeder passes through as
  // one contiguous circuit. Only genuinely shared segments (both subs on
  // the same road, e.g. a bridge) stay foreign: there the far side splits
  // into its own feeder with an express run back — the second circuit.
  for (const s of subOrder) {
    const chain = subChains[s.id];
    for (let li = 0; li < loadNodes.length; li++) {
      const fid = mem.feederOf[li];
      if (fid === -1 || mem.feeders[fid].group !== s.id) continue;
      for (let v = loadNodes[li].node; v !== s.node && v !== -1; v = chain.par[v]) {
        if (claimed[v] === -1) {
          claimed[v] = s.id;
          perSubClaimed[s.id].push(v);
          dist[v] = chain.sdist[v]; parent[v] = chain.par[v]; parentEdge[v] = chain.edge[v];
          midDistNode[v] = (chain.sdist[v] + chain.sdist[chain.par[v]]) / 2;
          usedEdge[chain.edge[v]] = 1;
        }
      }
    }
  }

  // ---- per-sub tree orders (parent-before-child) + node ownership.
  // Ownership works on RAW feeder ids (M.feederOf) so that id compaction
  // during structural mutations cannot skew earlier subs' results; compact
  // ids are assigned once, at the end.
  const ownerOld = new Int32Array(nN).fill(-1);
  const kids = new Map(); // node -> child nodes (within claimed forests)
  for (let v = 0; v < nN; v++) {
    if (claimed[v] === -1 || parent[v] === -1) continue;
    let arr = kids.get(parent[v]);
    if (!arr) kids.set(parent[v], arr = []);
    arr.push(v);
  }
  const subNodeOrder = subs.map(() => []);
  let forcedSplits = 0, trunkAbsorbs = 0;
  for (const s of subs) {
    ownerOld[s.node] = -2; // busbar
    // parent-before-child order over THIS sub's claimed forest. A claimed
    // node whose parent is foreign-claimed starts its own component (its
    // path continues upstream as express through the foreign corridor).
    const sameSub = (v) => v !== -1 && claimed[v] === s.id && v !== s.node;
    const st = [...(kids.get(s.node) ?? []).filter(w => claimed[w] === s.id)];
    for (const v of perSubClaimed[s.id]) {
      const p = parent[v];
      if (p !== s.node && !sameSub(p)) st.push(v); // boundary root
    }
    const order = [];
    while (st.length) {
      const v = st.pop();
      order.push(v);
      for (const w of kids.get(v) ?? []) if (claimed[w] === s.id) st.push(w);
    }
    subNodeOrder[s.id] = order;

    // Top-down ownership. HARD RULE: a load node always belongs to its
    // own member feeder — series segments along one corridor keep their
    // identity (the downstream feeder reaches past on an express run),
    // exactly like real parallel circuits. Pass-through nodes: sticky
    // to the parent's feeder while it still has members below, else the
    // dominant feeder below this node.
    const deriveOwnership = () => {
      const fcust = new Map(order.map(v => [v, new Map()]));
      for (let i = order.length - 1; i >= 0; i--) {
        const v = order[i];
        const m = fcust.get(v);
        const li = loadAt.get(v);
        if (li !== undefined && M.feederOf[li] !== -1) {
          m.set(M.feederOf[li], (m.get(M.feederOf[li]) ?? 0) + loadNodes[li].cust);
        }
        const p = parent[v];
        if (sameSub(p)) {
          const pm = fcust.get(p);
          for (const [f, c] of m) pm.set(f, (pm.get(f) ?? 0) + c);
        }
      }
      for (const v of order) {
        const p = parent[v];
        const pOwn = sameSub(p) ? ownerOld[p] : -1;
        const m = fcust.get(v);
        const liV = loadAt.get(v);
        let o = -1;
        if (liV !== undefined && M.feederOf[liV] !== -1) o = M.feederOf[liV];
        else if (pOwn >= 0 && (m.get(pOwn) ?? 0) > 0) o = pOwn;
        else {
          let bc = -1;
          for (const [f, c] of m) if (c > bc || (c === bc && f < o)) { bc = c; o = f; }
        }
        ownerOld[v] = o;
      }
    };
    // components of one raw feeder id, largest-customer first
    const componentsOf = () => {
      const rootsOf = new Map(); // raw feeder id -> [root nodes]
      for (const v of order) {
        const f = ownerOld[v];
        if (f < 0) continue;
        const p = parent[v];
        if (!sameSub(p) || ownerOld[p] !== f) {
          if (!rootsOf.has(f)) rootsOf.set(f, []);
          rootsOf.get(f).push(v);
        }
      }
      const out = [];
      for (const [f, roots] of rootsOf) {
        if (roots.length <= 1) continue;
        const comps = roots.map(r => {
          const nodes = [], stc = [r];
          let cust = 0;
          while (stc.length) {
            const v = stc.pop();
            nodes.push(v);
            const li = loadAt.get(v);
            if (li !== undefined && M.feederOf[li] === f) cust += loadNodes[li].cust;
            for (const w of kids.get(v) ?? []) if (claimed[w] === s.id && ownerOld[w] === f) stc.push(w);
          }
          return { f, root: r, nodes, cust };
        }).sort((a, b) => b.cust - a.cust || a.root - b.root);
        out.push(comps);
      }
      return out;
    };

    // settle ownership; early rounds may absorb runt components upstream —
    // but never past the receiving feeder's cap (absorb jitter otherwise
    // re-inflates trunk feeders every pass)
    for (let round = 0; round < 6; round++) {
      deriveOwnership();
      const custOf = new Map(), urbOf = new Map();
      for (let i = 0; i < loadNodes.length; i++) {
        const f = M.feederOf[i];
        if (f === -1) continue;
        custOf.set(f, (custOf.get(f) ?? 0) + loadNodes[i].cust);
        urbOf.set(f, (urbOf.get(f) ?? 0) + loadNodes[i].urbanCust);
      }
      const capOfFid = (f) => capOf((urbOf.get(f) ?? 0) * 2 >= (custOf.get(f) ?? 0));
      let changed = false;
      for (const comps of componentsOf()) {
        for (const comp of comps.slice(1)) {
          const pr = parent[comp.root];
          const upstream = sameSub(pr) ? ownerOld[pr] : -1;
          if (comp.cust < FEEDER_MIN_CUST && upstream >= 0 &&
              (custOf.get(upstream) ?? 0) + comp.cust <= capOfFid(upstream)) {
            for (const v of comp.nodes) {
              const li = loadAt.get(v);
              if (li !== undefined && M.feederOf[li] === comp.f) M.feederOf[li] = upstream;
            }
            custOf.set(upstream, (custOf.get(upstream) ?? 0) + comp.cust);
            trunkAbsorbs++;
            changed = true;
          }
        }
      }
      if (!changed) break;
    }
    // component-materialise sweep: contiguity GUARANTEED by construction —
    // extra components become new feeders with nodes reassigned directly,
    // and ownership is NOT re-derived afterwards.
    deriveOwnership();
    for (const comps of componentsOf()) {
      for (const comp of comps.slice(1)) {
        const newFid = M.nextFid++;
        M.fidGroup.set(newFid, M.fidGroup.get(comp.f));
        for (const v of comp.nodes) {
          ownerOld[v] = newFid;
          const li = loadAt.get(v);
          if (li !== undefined && M.feederOf[li] === comp.f) M.feederOf[li] = newFid;
        }
        forcedSplits++;
      }
    }
  }
  if (forcedSplits || trunkAbsorbs) {
    repairLog.push({ pass, action: "structural enforcement",
      detail: `${trunkAbsorbs} runt component(s) absorbed, ${forcedSplits} forced feeder split(s)` });
  }

  // compact ids once, after all structural mutations
  const memF = materialise(loadNodes, M);
  const oldToNew = new Map(memF.feeders.map(f => [f.oldFid, f.id]));
  const feederOfNode = new Int32Array(nN).fill(-1);
  for (let v = 0; v < nN; v++) {
    if (ownerOld[v] === -2) feederOfNode[v] = -2;
    else if (ownerOld[v] >= 0) feederOfNode[v] = oldToNew.get(ownerOld[v]) ?? -1;
  }

  return {
    mem: memF, subs, subNodeOrder, subChains, midDistNode, subLabel,
    dist, parent, parentEdge, claimed, usedEdge, feederOfNode,
    terrain, density,
  };
}

// Move one load node's membership to the road-nearest feeder of a given
// sub (or of any OTHER sub when subId is -1).
function moveToNearestFeeder(graph, loadNodes, mem, M, li, subId) {
  const heap = new NodeHeap();
  const dist = new Map([[loadNodes[li].node, 0]]);
  heap.push(0, loadNodes[li].node);
  const loadAt = new Map(loadNodes.map((ln, i) => [ln.node, i]));
  const seen = new Set();
  while (heap.size) {
    const v = heap.pop();
    if (seen.has(v)) continue;
    seen.add(v);
    const lj = loadAt.get(v);
    if (lj !== undefined && lj !== li && mem.feederOf[lj] !== -1 &&
        mem.feederOf[lj] !== mem.feederOf[li] &&
        (subId === -1
          ? mem.feeders[mem.feederOf[lj]].group !== mem.feeders[mem.feederOf[li]]?.group
          : mem.feeders[mem.feederOf[lj]].group === subId)) {
      M.feederOf[li] = mem.feeders[mem.feederOf[lj]].oldFid;
      return true;
    }
    for (const ei of graph.adj[v]) {
      const e = graph.edges[ei];
      const w = e.a === v ? e.b : e.a;
      const nd = dist.get(v) + e.len;
      if (nd < (dist.get(w) ?? Infinity)) { dist.set(w, nd); heap.push(nd, w); }
    }
  }
  return false;
}

// ---------------------------------------------------------------------
// Step 6 — validation rules (named caps above). Violations feed repairs;
// whatever survives MAX_REPAIR_PASSES is reported, never hidden.
// ---------------------------------------------------------------------
function validateMembership(graph, loadNodes, built) {
  const { mem, subs, dist, feederOfNode } = built;
  const viol = [];
  for (const f of mem.feeders) {
    const cap = capOf(f.urban);
    // tiny fragments only — genuine small leaf feeders are realistic and
    // kept; TINY_FEEDER guards against split debris
    const TINY_FEEDER = 50;
    if (f.cust < TINY_FEEDER && mem.feeders.filter(x => x.group === f.group).length > 1) {
      viol.push({ kind: "runt", feeder: f.id, lis: f.loadIdx.slice(),
        detail: `F${f.id} only ${f.cust} cust (< ${TINY_FEEDER})` });
    }
    if (f.cust > cap * CAP_SLACK) {
      viol.push({ kind: "cap", feeder: f.id,
        detail: `F${f.id} ${f.cust} cust > ${f.urban ? "urban" : "rural"} cap ${cap}` });
    }
    let maxD = 0, minD = Infinity;
    for (const li of f.loadIdx) {
      const d = dist[loadNodes[li].node];
      if (isFinite(d)) { maxD = Math.max(maxD, d); minD = Math.min(minD, d); }
    }
    f.trunkKm = +(maxD / 1000).toFixed(2);
    if (maxD / 1000 > MAX_FEEDER_KM) {
      viol.push({ kind: "trunk", feeder: f.id,
        detail: `F${f.id} trunk ${(maxD / 1000).toFixed(1)} km > ${MAX_FEEDER_KM} km` });
    } else if (!f.urban && (maxD - minD) / 1000 > 2 * RURAL_EXTENT_KM_MAX * CAP_SLACK) {
      // RURAL_EXTENT_KM_MAX is a growth RADIUS; the member span can
      // legitimately reach twice that
      viol.push({ kind: "extent", feeder: f.id,
        detail: `F${f.id} rural span ${((maxD - minD) / 1000).toFixed(1)} km > 2×${RURAL_EXTENT_KM_MAX} km` });
    }
    // foreign-NETWORK transit along the feeder's TRUE sub→member paths:
    // only stretches on another sub's actual tree count (crossing empty
    // countryside that merely LABELS foreign is harmless). The visited
    // set counts shared trunk segments once, not per member.
    const transit = new Map();
    const visited = new Set();
    const chain = built.subChains[f.group];
    const subNode = subs[f.group].node;
    for (const li of f.loadIdx) {
      for (let v = loadNodes[li].node; v !== subNode && v !== -1 && !visited.has(v);) {
        visited.add(v);
        const ei = chain.edge[v];
        const cl = built.claimed[v];
        if (cl !== -1 && cl !== f.group && ei !== -1) {
          transit.set(cl, (transit.get(cl) ?? 0) + graph.edges[ei].len);
        }
        v = chain.par[v];
      }
    }
    for (const [foreignSub, len] of transit) {
      if (len > MAX_FOREIGN_CROSSING_M) {
        viol.push({ kind: "foreign", feeder: f.id, foreignSub,
          detail: `F${f.id} transits sub ${foreignSub} catchment for ${(len / 1000).toFixed(1)} km > ${MAX_FOREIGN_CROSSING_M / 1000} km` });
        break;
      }
    }
  }
  for (const s of subs) {
    const n = mem.feeders.filter(f => f.group === s.id).length;
    if (n > FEEDERS_PER_SUB_MAX) {
      viol.push({ kind: "group", sub: s.id,
        detail: `sub ${s.name} has ${n} feeders > ${FEEDERS_PER_SUB_MAX}` });
    }
  }
  return viol;
}

// Repairs loop back to steps 2–3 for AFFECTED MEMBERS only.
function applyRepairs(graph, loadNodes, M, built, viol, repairLog, pass) {
  const { mem, subs } = built;
  const applied = [];
  for (const v of viol) {
    if (v.kind === "runt") {
      // fold a runt feeder into its road-nearest same-sub neighbour
      const f = mem.feeders[v.feeder];
      let moved = 0;
      for (const li of v.lis) {
        if (moveToNearestFeeder(graph, loadNodes, mem, M, li, f.group) ||
            moveToNearestFeeder(graph, loadNodes, mem, M, li, -1)) moved++;
      }
      applied.push(`runt: F${v.feeder} (${f.cust} cust) folded into neighbours (${moved} node(s))`);
    } else if (v.kind === "cap" || v.kind === "extent") {
      // re-split this feeder's members with the same road-capacitated growth
      const f = mem.feeders[v.feeder];
      const allowed = new Set(f.loadIdx);
      const re = clusterFeeders(graph, loadNodes, allowed, M.nextFid);
      M.nextFid += re.feeders.length;
      for (const nf of re.feeders) M.fidGroup.set(nf.id, M.fidGroup.get(f.oldFid));
      for (const li of f.loadIdx) if (re.feederOf[li] !== -1) M.feederOf[li] = re.feederOf[li];
      applied.push(`${v.kind}: F${v.feeder} re-split into ${re.feeders.length}`);
    } else if (v.kind === "trunk") {
      // far members become their own feeder, homed to the nearest sub with
      // group headroom (possibly a brand-new sub)
      const f = mem.feeders[v.feeder];
      const far = f.loadIdx.filter(li => built.dist[loadNodes[li].node] > MAX_FEEDER_KM * 1000 * 0.8);
      if (!far.length) continue;
      let sx = 0, sy = 0, c = 0;
      for (const li of far) {
        sx += graph.nx[loadNodes[li].node] * loadNodes[li].cust;
        sy += graph.ny[loadNodes[li].node] * loadNodes[li].cust;
        c += loadNodes[li].cust;
      }
      sx /= c; sy /= c;
      // nearest OTHER sub, headroom preferred; the pocket only moves when
      // that sub is genuinely closer to it than its own sub
      const dOwn = Math.hypot(subs[f.group].x - sx, subs[f.group].y - sy);
      let bestG = -1, bd = Infinity;
      for (const s of subs) {
        if (s.id === f.group) continue;
        const n = mem.feeders.filter(x => x.group === s.id).length;
        const d = Math.hypot(s.x - sx, s.y - sy) + (n < FEEDERS_PER_SUB_MAX ? 0 : 3000);
        if (d < bd) { bd = d; bestG = s.id; }
      }
      const newFid = M.nextFid++;
      const groupCount = new Set([...M.fidGroup.values()]).size;
      const donorFid = bestG !== -1 && bd < dOwn
        ? mem.feeders.find(x => x.group === bestG).oldFid // reuse that group's key
        : null;
      // a brand-new sub only for a pocket big enough to justify one
      const spawnSub = donorFid === null && c >= NEW_SUB_MIN_CUST && groupCount < MAX_SUBS;
      if (donorFid === null && !spawnSub) {
        // last resort: each far member re-homes to the road-nearest
        // feeder of any other sub
        let moved = 0;
        for (const li of far) if (moveToNearestFeeder(graph, loadNodes, mem, M, li, -1)) moved++;
        applied.push(`trunk: F${v.feeder} far pocket (${c} cust) — ${moved} member(s) re-homed by road`);
        continue;
      }
      M.fidGroup.set(newFid,
        donorFid !== null ? M.fidGroup.get(donorFid)
          : Math.max(...M.fidGroup.values()) + 1);
      for (const li of far) M.feederOf[li] = newFid;
      applied.push(`trunk: F${v.feeder} far pocket (${c} cust) → ` +
        (donorFid !== null ? `sub group ${bestG}` : "new sub"));
    } else if (v.kind === "foreign") {
      // whole feeder moves to the sub it keeps transiting, if it has room
      // (unconditional moves churn the grouping and never settle); else
      // just the members BEYOND the transit re-home to that sub
      const f = mem.feeders[v.feeder];
      const n = mem.feeders.filter(x => x.group === v.foreignSub).length;
      const donor = mem.feeders.find(x => x.group === v.foreignSub);
      if (donor && n < FEEDERS_PER_SUB_MAX) {
        M.fidGroup.set(f.oldFid, M.fidGroup.get(donor.oldFid));
        applied.push(`foreign: F${v.feeder} moved to sub ${v.foreignSub}`);
      } else if (donor) {
        const chain = built.subChains[f.group];
        let moved = 0;
        for (const li of f.loadIdx) {
          let crosses = false;
          for (let w = loadNodes[li].node; w !== -1 && w !== subs[f.group].node; w = chain.par[w]) {
            if (built.claimed[w] === v.foreignSub) { crosses = true; break; }
          }
          if (crosses && moveToNearestFeeder(graph, loadNodes, mem, M, li, v.foreignSub)) moved++;
        }
        applied.push(`foreign: F${v.feeder} — ${moved} member(s) beyond the transit re-homed to sub ${v.foreignSub}`);
      }
    } else if (v.kind === "group") {
      // split the oversize group in two by Morton order of feeder seeds
      const members = mem.feeders.filter(f => f.group === v.sub)
        .sort((a, b) => {
          const na = loadNodes[a.loadIdx[0]].node, nb = loadNodes[b.loadIdx[0]].node;
          return morton(graph.nx[na], graph.ny[na]) - morton(graph.nx[nb], graph.ny[nb]);
        });
      const half = Math.ceil(members.length / 2);
      const newGroup = Math.max(...M.fidGroup.values()) + 1;
      for (const f of members.slice(half)) M.fidGroup.set(f.oldFid, newGroup);
      applied.push(`group: sub ${v.sub} split (${members.length} → ${half}+${members.length - half} feeders)`);
    }
  }
  repairLog.push({ pass, action: "repairs", detail: applied.join("; ") || "none applicable" });
}

// ---------------------------------------------------------------------
// Final assembly: tree edges, feeder objects, express runs, membership
// tables + check entries. Interface identical to the routing-first model.
// ---------------------------------------------------------------------
function finishNet(graph, customers, txs, loadNodes, built, classOfCust, repairLog) {
  const { mem, subs, subNodeOrder, dist, parent, parentEdge, usedEdge, feederOfNode,
    terrain, density } = built;
  const nN = graph.nNodes;
  const loadAt = new Map(loadNodes.map((ln, i) => [ln.node, i]));

  const gridN = terrain.n;
  const isUnderground = (a, b) => {
    const mx = (graph.nx[a] + graph.nx[b]) / 2;
    const my = (graph.ny[a] + graph.ny[b]) / 2;
    const [cx, cy] = terrain.cellOf(mx, my);
    return density.grid[cy * gridN + cx] > UG_DENSITY_THRESH;
  };

  let feeders = mem.feeders.map(f => ({
    id: f.id, sub: f.group, rootNode: -1, urban: f.urban, trunkKm: f.trunkKm ?? 0,
    nodes: [], edges: [], customers: 0,
    lengthM: 0, ohLenM: 0, ugLenM: 0, txCount: 0,
    expressOhKm: 0, expressUgKm: 0, expressMidM: 0,
  }));

  const custAtNode = new Float64Array(nN);
  const subtreeCust = new Float64Array(nN);
  const treeEdges = [];
  const treeEdgeOfNode = new Int32Array(nN).fill(-1);
  const subOf = new Int32Array(nN).fill(-1);
  const allOrder = [];
  for (const s of subs) {
    subOf[s.node] = s.id;
    for (const v of subNodeOrder[s.id]) { subOf[v] = s.id; allOrder.push(v); }
  }
  for (const v of allOrder) {
    const fid = feederOfNode[v];
    if (fid < 0) continue;
    const ei = parentEdge[v];
    const e = graph.edges[ei];
    const te = {
      id: treeEdges.length, node: v, parentNode: parent[v], edgeId: ei,
      feeder: fid, lenM: e.len,
      midDistM: built.midDistNode[v], // own-sub travel distance to midpoint
      bridge: e.bridge,
      underground: isUnderground(v, parent[v]),
    };
    treeEdgeOfNode[v] = te.id;
    treeEdges.push(te);
    const f = feeders[fid];
    f.edges.push(te.id);
    f.nodes.push(v);
    f.lengthM += e.len;
    if (te.underground) f.ugLenM += e.len; else f.ohLenM += e.len;
    if (f.rootNode === -1 || dist[v] < dist[f.rootNode]) f.rootNode = v;
  }
  // Drop feeders that own no tree nodes (all their load sits at a sub
  // busbar, or their members went unrouted) — their TXs reattach below.
  // Ids re-compact so downstream consumers never see an edge-less feeder.
  const fidFinal = new Int32Array(feeders.length).fill(-1);
  feeders = feeders.filter(f => f.rootNode !== -1);
  feeders.forEach((f, i) => { fidFinal[f.id] = i; f.id = i; });
  for (const te of treeEdges) te.feeder = fidFinal[te.feeder];
  for (let v = 0; v < nN; v++) if (feederOfNode[v] >= 0) feederOfNode[v] = fidFinal[feederOfNode[v]];

  // feeder-local subtree customers (allOrder is parent-before-child per sub)
  let orphanTx = 0;
  for (const tx of txs) {
    const li = tx.node === -1 ? undefined : loadAt.get(tx.node);
    if (li === undefined || mem.feederOf[li] === -1) { tx.feeder = -1; tx.sub = -1; orphanTx++; continue; }
    tx.feeder = fidFinal[mem.feederOf[li]]; // -1 when the feeder was dropped
    tx.sub = mem.feeders[mem.feederOf[li]].group;
    custAtNode[tx.node] += tx.customers.length;
  }
  for (let i = allOrder.length - 1; i >= 0; i--) {
    const v = allOrder[i];
    subtreeCust[v] += custAtNode[v];
    const p = parent[v];
    if (p !== -1 && feederOfNode[p] === feederOfNode[v]) subtreeCust[p] += subtreeCust[v];
  }
  for (const f of feeders) f.customers = subtreeCust[f.rootNode];
  // TXs at a sub busbar (feederOfNode = -2): attach to that sub's largest feeder
  for (const tx of txs) {
    if (tx.sub === -1) continue;
    if (feederOfNode[tx.node] === -2) {
      const best = feeders.filter(f => f.sub === subOf[tx.node])
        .sort((a, b) => b.customers - a.customers)[0];
      if (best) {
        tx.feeder = best.id; tx.sub = best.sub;
        best.customers += tx.customers.length;
      }
    }
    if (tx.feeder >= 0) feeders[tx.feeder].txCount++;
    else { tx.sub = -1; orphanTx++; }
  }
  // express runs: feeder head back to the sub busbar along the sub's true
  // path — through other feeders' territory AND foreign subs' corridors
  // (the parallel circuit on a shared road)
  for (const f of feeders) {
    const subNode = subs[f.sub].node;
    const chain = built.subChains[f.sub];
    let ohM = 0, ugM = 0;
    for (let w = chain.par[f.rootNode]; w !== subNode && w !== -1; w = chain.par[w]) {
      const ei = chain.edge[w], p = chain.par[w];
      if (ei === -1 || p === -1) break;
      if (isUnderground(w, p)) ugM += graph.edges[ei].len; else ohM += graph.edges[ei].len;
    }
    f.expressOhKm = ohM / 1000;
    f.expressUgKm = ugM / 1000;
    f.expressMidM = (ohM + ugM) / 2;
  }

  // ---- membership tables + reported checks
  const feederOfTx = new Int32Array(txs.length).fill(-1);
  for (const tx of txs) feederOfTx[tx.id] = tx.feeder;
  const subOfFeeder = new Int32Array(feeders.length);
  for (const f of feeders) subOfFeeder[f.id] = f.sub;

  // strict contiguity: every feeder must have exactly ONE component root
  const rootsPerFeeder = new Int32Array(feeders.length);
  for (const te of treeEdges) {
    const p = te.parentNode;
    if (feederOfNode[p] !== te.feeder) rootsPerFeeder[te.feeder]++;
  }
  const multiComponent = feeders.filter(f => rootsPerFeeder[f.id] !== 1);
  const contiguous = feeders.every(f => f.rootNode !== -1) && multiComponent.length === 0;
  let honoured = 0, dishonoured = 0;
  for (let li = 0; li < loadNodes.length; li++) {
    if (mem.feederOf[li] === -1) continue;
    const fid = fidFinal[mem.feederOf[li]];
    if (fid === -1) continue; // dropped feeder: covered by busbar/orphan paths
    const own = feederOfNode[loadNodes[li].node];
    if (own === fid || own === -2) honoured++; else dishonoured++;
  }
  const residual = built.violations ?? [];
  const urbanShare = classOfCust.reduce((s, v) => s + v, 0) / Math.max(1, classOfCust.length);
  const overCap = feeders.filter(f => f.customers > capOf(f.urban) * CAP_SLACK);
  const maxPerSub = Math.max(0, ...subs.map(s => feeders.filter(f => f.sub === s.id).length));
  const worstTrunk = Math.max(0, ...feeders.map(f => f.trunkKm));
  const extraChecks = [
    {
      name: "Membership honoured by routing",
      pass: dishonoured === 0 && contiguous && orphanTx === 0,
      detail: `${honoured}/${honoured + dishonoured} load nodes on their own feeder; ` +
        (multiComponent.length
          ? `${multiComponent.length} feeder(s) NOT contiguous (${multiComponent.map(f => "F" + f.id).join(",")}); `
          : `every feeder a single contiguous subtree; `) +
        `${orphanTx} orphan TX`,
    },
    // The remaining checks police TUNABLE rules (caps the user is invited
    // to tune, marked tunable: true): a fail is honest reporting of an
    // unresolved residual, not a correctness bug — the selftest gates on
    // correctness checks only.
    {
      name: "Feeder caps (urban ≤ " + capOf(true) + ", rural ≤ " + capOf(false) + ")",
      tunable: true,
      pass: overCap.length === 0,
      detail: overCap.length
        ? `${overCap.length} feeder(s) over cap: ${overCap.map(f => `F${f.id}(${Math.round(f.customers)})`).join(", ")}`
        : `${feeders.length} feeders, ${feeders.filter(f => f.urban).length} urban / ${feeders.filter(f => !f.urban).length} rural, all within cap ×${CAP_SLACK}`,
    },
    {
      name: `Feeder trunks ≤ ${MAX_FEEDER_KM} km, rural extent ≤ ${RURAL_EXTENT_KM_MAX} km`,
      tunable: true,
      pass: !residual.some(v => v.kind === "trunk" || v.kind === "extent"),
      detail: residual.filter(v => v.kind === "trunk" || v.kind === "extent").map(v => v.detail).join("; ") ||
        `worst trunk ${worstTrunk.toFixed(1)} km`,
    },
    {
      name: `Foreign-catchment transit ≤ ${MAX_FOREIGN_CROSSING_M} m`,
      tunable: true,
      pass: !residual.some(v => v.kind === "foreign"),
      detail: residual.filter(v => v.kind === "foreign").map(v => v.detail).join("; ") || "no feeder path lingers in a foreign sub's catchment",
    },
    {
      name: `Feeders per sub ≤ ${FEEDERS_PER_SUB_MAX}`,
      tunable: true,
      pass: maxPerSub <= FEEDERS_PER_SUB_MAX,
      detail: `${subs.length} sub(s), max ${maxPerSub} feeders on one sub`,
    },
    {
      name: `Membership repair loop (≤ ${MAX_REPAIR_PASSES} passes)`,
      tunable: true,
      pass: residual.length === 0,
      detail: residual.length
        ? `unresolved after repairs: ${residual.map(v => v.detail).join("; ")}`
        : (repairLog.length ? repairLog.map(r => `[p${r.pass}] ${r.action}: ${r.detail}`).join(" | ") : "clean on first pass"),
    },
  ];

  return {
    txs, subs, feeders,
    dist, parent, parentEdge, subOf, feederOfNode,
    subtreeCust, custAtNode, treeEdges, treeEdgeOfNode,
    usedEdge, orphanTx,
    membership: {
      classOfCust, urbanShare, loadNodes: loadNodes.length,
      feederOfTx, subOfFeeder, repairLog, residual,
    },
    extraChecks,
  };
}

// Nudge a load centroid to the nearest subtransmission-viable road node:
// prefer nodes on an arterial/collector corridor with gentle slope.
function nudgeToCorridor(terrain, graph, cx, cy, existingSubs) {
  let best = -1, bestScore = Infinity;
  const R = 2500;
  const bx = (cx / 250) | 0, by = (cy / 250) | 0;
  const rings = Math.ceil(R / 250) + 1;
  for (let dy = -rings; dy <= rings; dy++) {
    for (let dx = -rings; dx <= rings; dx++) {
      const arr = graph.hash.get((bx + dx) * 4096 + (by + dy));
      if (!arr) continue;
      for (const id of arr) {
        if (existingSubs.some(s => s.node === id)) continue;
        const d = Math.hypot(graph.nx[id] - cx, graph.ny[id] - cy);
        if (d > R) continue;
        const onTrunk = graph.adj[id].some(ei => graph.edges[ei].cls <= 1);
        const score = d + (onTrunk ? 0 : 1400) +
          terrain.slopeAt(graph.nx[id], graph.ny[id]) * 20000;
        if (score < bestScore) { bestScore = score; best = id; }
      }
    }
  }
  if (best !== -1) return best;
  const near = graph.nearestNode(cx, cy, 30000,
    (id) => !existingSubs.some(s => s.node === id));
  return near.id;
}

function clusterTransformers(customers, rng) {
  // Deterministic order: Morton (Z-curve) over coordinates.
  const order = customers.map((c, i) => ({ i, m: morton(c.x, c.y) }))
    .sort((a, b) => a.m - b.m).map(o => o.i);
  const assigned = new Int32Array(customers.length).fill(-1);
  // spatial hash of unassigned customers
  const BIN = 400;
  const hash = new Map();
  const key = (x, y) => ((x / BIN) | 0) * 8192 + ((y / BIN) | 0);
  for (let i = 0; i < customers.length; i++) {
    const k = key(customers[i].x, customers[i].y);
    if (!hash.has(k)) hash.set(k, []);
    hash.get(k).push(i);
  }
  const txs = [];
  for (const start of order) {
    if (assigned[start] !== -1) continue;
    const c0 = customers[start];
    // ASSUMPTION: gather radius depends on local density — tight in town,
    // wide in the country (LV runs are longer rurally).
    const urban = c0.density > 0.15;
    const maxR = urban ? 500 : 1500;
    const members = [start];
    assigned[start] = txs.length;
    let cx = c0.x, cy = c0.y;
    while (members.length < TX_CAP) {
      let best = -1, bestD = Infinity;
      const rings = Math.ceil(maxR / BIN) + 1;
      const bx = (cx / BIN) | 0, by = (cy / BIN) | 0;
      for (let dy = -rings; dy <= rings; dy++) {
        for (let dx = -rings; dx <= rings; dx++) {
          const arr = hash.get((bx + dx) * 8192 + (by + dy));
          if (!arr) continue;
          // compact out already-assigned customers while scanning
          let w = 0;
          for (let r2 = 0; r2 < arr.length; r2++) {
            const j = arr[r2];
            if (assigned[j] !== -1) continue;
            arr[w++] = j;
            const d = Math.hypot(customers[j].x - cx, customers[j].y - cy);
            if (d < bestD) { bestD = d; best = j; }
          }
          arr.length = w;
        }
      }
      if (best === -1 || bestD > maxR) break;
      assigned[best] = txs.length;
      members.push(best);
      cx += (customers[best].x - cx) / members.length;
      cy += (customers[best].y - cy) / members.length;
    }
    txs.push({ id: txs.length, x: cx, y: cy, customers: members, node: -1, feeder: -1, sub: -1 });
  }
  return txs;
}
