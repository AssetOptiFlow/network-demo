// membership.js — the ENGINEERED REQUIREMENTS that map customers →
// transformers → feeders → zone subs. Construction resolves every load
// block down a LADDER (network.js): split an over-cap branch into SIBLING
// feeders sharing the exit corridor → spawn a zone sub on the corridor
// when a stranded block is sub-worthy → EXPRESS feeder for stranded blocks
// too small for a station (labelled exception, never the rule) → clip
// only below FEEDER_MIN_CUST. Deletion never touches a block big enough
// to matter.
//
//   customers → TX:   ≤ TX_MAX_M from the transformer OR ≤ TX_MAX_CUST
//                     (rural maxes out on distance, urban on count)
//   TX → zone sub:    ≤ SUB_MAX_KM by road, ≤ SUB_MAX_CUST per sub,
//                     ≥ SUB_MIN_CUST per sub (smaller stations are
//                     trial-dissolved by the parsimony pass), and at most
//                     SUB_MAX_COUNT stations on the map
//   feeders:          ≤ FEEDER_MAX_CUST and ≤ FEEDER_MAX_KM of CONDUCTOR;
//                     ≤ FEEDERS_MAX_PER_SUB breaker positions per sub; a
//                     sibling feeder's unloaded exit lead ≤ LEAD_MAX_M
//                     (longer unloaded runs are EXPRESS feeders — allowed
//                     as a labelled exception for 20–200 customer blocks);
//                     a feeder still under FEEDER_MIN_CUST after folding
//                     is PRUNED with its transformers and customers
//                     (uneconomic to reticulate)
//
// The membership atom is the LOAD NODE — a road node with ≥ 1 TX snapped to
// it. Routing (network.js) assigns every load node to its road-NEAREST
// sited sub (graph Voronoi), which makes each sub's tree a disjoint
// subforest of one multi-source Dijkstra — no overlap machinery needed.
//
// ASSUMPTION: all caps below are tuning knobs, not engineering limits.

export const TX_MAX_CUST = 100;      // customers per distribution transformer
export const TX_MAX_M = 500;         // customer → transformer distance (m)
export const SUB_MAX_CUST = 5000;    // customers (ICPs) per zone sub
export const SUB_MIN_CUST = 200;     // stations below this are trial-dissolved;
                                     // doubles as the sub-worthiness threshold
                                     // for spawning on a stranded block
export const SUB_MAX_COUNT = 60;     // station budget for the whole map
export const SUB_MAX_KM = 50;        // transformer → zone sub, by road
export const FEEDER_MAX_CUST = 1000; // customers per feeder (uniform)
export const FEEDER_MAX_KM = 200;    // total CONDUCTOR per feeder, km
                                     // (reach is map-bounded and uncapped)
export const FEEDERS_MAX_PER_SUB = 12; // breaker positions per station
export const FEEDER_MIN_CUST = 20;   // feeders under this are pruned entirely
                                     // (with their customers and transformers)
export const LEAD_MAX_M = 2000;      // max unloaded exit lead for a SIBLING
                                     // feeder sharing its sub's corridor;
                                     // beyond this a circuit is EXPRESS
export const MERGE_HEADROOM = 0.9;   // parsimony may only absorb load into a
                                     // station up to this share of its caps
                                     // (spawn fires at >100% — the gap is the
                                     // hysteresis that prevents oscillation)

// Urban/rural is REPORTING-ONLY now (the caps are uniform): a customer is
// urban when local density ≥ URBAN_CUST_PER_KM2 within DENSITY_RADIUS_M.
export const URBAN_CUST_PER_KM2 = 60;
export const DENSITY_RADIUS_M = 500;

// ------------------------------------------------------------ primitives

export class NodeHeap {
  constructor() { this.d = []; this.v = []; }
  get size() { return this.d.length; }
  push(d, v) {
    const D = this.d, V = this.v; let i = D.length;
    D.push(d); V.push(v);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (D[p] <= D[i]) break;
      [D[p], D[i]] = [D[i], D[p]]; [V[p], V[i]] = [V[i], V[p]]; i = p;
    }
  }
  pop() {
    const D = this.d, V = this.v; const top = V[0];
    const ld = D.pop(), lv = V.pop();
    if (D.length) {
      D[0] = ld; V[0] = lv; let i = 0;
      for (;;) {
        const l = 2 * i + 1, r = l + 1; let m = i;
        if (l < D.length && D[l] < D[m]) m = l;
        if (r < D.length && D[r] < D[m]) m = r;
        if (m === i) break;
        [D[m], D[i]] = [D[i], D[m]]; [V[m], V[i]] = [V[i], V[m]]; i = m;
      }
    }
    return top;
  }
}

export function morton(x, y) {
  let a = Math.min(65535, Math.max(0, Math.round(x / 2)));
  let b = Math.min(65535, Math.max(0, Math.round(y / 2)));
  let m = 0;
  for (let i = 0; i < 16; i++) {
    m += ((a >> i) & 1) * Math.pow(2, 2 * i) + ((b >> i) & 1) * Math.pow(2, 2 * i + 1);
  }
  return m;
}

// -------------------------------------------- urban/rural (reporting only)

export function classifyCustomers(customers) {
  const R = DENSITY_RADIUS_M, AREA_KM2 = Math.PI * R * R / 1e6;
  const BIN = R, hash = new Map();
  const key = (x, y) => ((x / BIN) | 0) * 8192 + ((y / BIN) | 0);
  for (let i = 0; i < customers.length; i++) {
    const k = key(customers[i].x, customers[i].y);
    let arr = hash.get(k);
    if (!arr) hash.set(k, arr = []);
    arr.push(i);
  }
  const cls = new Uint8Array(customers.length);
  const thresh = URBAN_CUST_PER_KM2 * AREA_KM2;
  for (let i = 0; i < customers.length; i++) {
    const c = customers[i];
    let n = 0;
    const bx = (c.x / BIN) | 0, by = (c.y / BIN) | 0;
    for (let dy = -1; dy <= 1; dy++) {
      for (let dx = -1; dx <= 1; dx++) {
        const arr = hash.get((bx + dx) * 8192 + (by + dy));
        if (!arr) continue;
        for (const j of arr) {
          if (Math.hypot(customers[j].x - c.x, customers[j].y - c.y) <= R) n++;
        }
      }
    }
    if (n >= thresh) cls[i] = 1;
  }
  return cls;
}

// Aggregate TXs by road node into load nodes — the membership atoms.
export function buildLoadNodes(txs, customers, classOfCust) {
  const byNode = new Map();
  for (const tx of txs) {
    if (tx.node === -1) continue;
    let ln = byNode.get(tx.node);
    if (!ln) byNode.set(tx.node, ln = { node: tx.node, cust: 0, urbanCust: 0, txIds: [] });
    ln.txIds.push(tx.id);
    for (const ci of tx.customers) {
      ln.cust++;
      if (classOfCust[ci]) ln.urbanCust++;
    }
  }
  const loadNodes = [...byNode.values()];
  for (const ln of loadNodes) ln.urban = ln.urbanCust * 2 >= ln.cust;
  return loadNodes;
}

// ------------------------------ TX → zone subs (≤ SUB_MAX_KM / SUB_MAX_CUST)

// Greedy road-Dijkstra growth, largest load node seeds first: absorb
// unassigned load nodes in road-distance order until the customer cap
// would overflow (oversize nodes are skipped so subs fill to the cap) or
// the radius runs out. Every load node eventually seeds or joins a sub —
// nothing can go unmapped while the road graph is connected.
export function growSubs(graph, loadNodes) {
  const nN = graph.nNodes;
  const loadAt = new Map(loadNodes.map((ln, i) => [ln.node, i]));
  const of = new Int32Array(loadNodes.length).fill(-1);
  const seedOrder = loadNodes.map((ln, i) => ({ i, c: ln.cust }))
    .sort((a, b) => b.c - a.c || a.i - b.i).map(o => o.i);
  const dist = new Float64Array(nN), stamp = new Int32Array(nN);
  let gen = 0;
  const clusters = [];
  for (const si of seedOrder) {
    if (of[si] !== -1) continue;
    const cid = clusters.length;
    let cust = 0;
    gen++;
    const heap = new NodeHeap();
    dist[loadNodes[si].node] = 0; stamp[loadNodes[si].node] = gen;
    heap.push(0, loadNodes[si].node);
    const seen = new Set();
    while (heap.size) {
      const v = heap.pop();
      if (seen.has(v)) continue;
      seen.add(v);
      if (dist[v] > SUB_MAX_KM * 1000) break;
      const li = loadAt.get(v);
      if (li !== undefined && of[li] === -1 &&
          (cust + loadNodes[li].cust <= SUB_MAX_CUST || li === si)) {
        of[li] = cid;
        cust += loadNodes[li].cust;
        if (cust >= SUB_MAX_CUST - 25) break; // effectively full
      }
      for (const ei of graph.adj[v]) {
        const e = graph.edges[ei];
        const w = e.a === v ? e.b : e.a;
        const nd = dist[v] + e.len;
        if (stamp[w] !== gen || nd < dist[w]) {
          stamp[w] = gen; dist[w] = nd;
          heap.push(nd, w);
        }
      }
    }
    clusters.push({ id: cid, cust });
  }
  return { of, clusters };
}
