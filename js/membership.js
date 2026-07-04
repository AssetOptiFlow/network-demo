// membership.js — MEMBERSHIP-FIRST layer. Who belongs to what is decided
// BEFORE any routing; pathing (network.js) comes last and must honour
// these tables.
//
//   1. customers classified urban/rural by LOCAL DENSITY (cust/km²)
//   2. customers → feeders: capacitated clustering by ROAD-GRAPH distance
//      (urban: count cap; rural: lower count cap + geographic extent cap)
//   3. feeders → zone subs: road-adjacent feeders grouped, capped per sub
//      (adjacency = shared road-graph Voronoi boundary, not straight-line)
//
// The atom of membership is the LOAD NODE — a road node with ≥1 TX snapped
// to it (all TXs at one node always share a feeder, so routing can honour
// membership exactly). Tables live on net.membership.
//
// ASSUMPTION: all caps below are tuning knobs, not engineering limits.

// Urban ⇔ ≥ this many customers within DENSITY_RADIUS_M. Measured on the
// generated density field: at the default 6 000 customers this sits at the
// town-grid boundary (≈ normalised density 0.15–0.25; ~45% of customers).
// NOTE: absolute cust/km² scales with the customer slider — a denser world
// classifies more urban, deliberately.
export const URBAN_CUST_PER_KM2 = 60;
export const DENSITY_RADIUS_M = 500;

export const N_URBAN_MAX = 700;        // customer cap, urban feeder
export const N_RURAL_MAX = 300;        // customer cap, rural feeder
export const RURAL_EXTENT_KM_MAX = 10; // rural growth radius by road
export const FEEDER_MIN_CUST = 150;    // runts below this merge if caps allow
export const FEEDERS_PER_SUB_MAX = 8;  // feeder-count cap per zone sub
export const GROUP_SPREAD_KM_MAX = 16; // max bbox diagonal of one sub's feeder group

export const capOf = (urban) => (urban ? N_URBAN_MAX : N_RURAL_MAX);

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

// -------------------------------------------- 1. urban/rural classification

// Local density = customers within DENSITY_RADIUS_M, as cust/km².
// Returns Uint8Array (1 = urban) aligned with the customers array.
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
// A load node is urban when most of its customers are.
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

// -------------------------------- 2. customers → feeders (road-capacitated)

// Grow one feeder per seed: Dijkstra over the ROAD GRAPH from the seed load
// node, absorbing unassigned load nodes in road-distance order until the
// class customer cap would overflow (stop) or, for rural feeders, the
// extent radius runs out. Seeds sweep in Morton order → deterministic.
// `allowed` (optional Set of load-node indices) restricts a re-split.
export function clusterFeeders(graph, loadNodes, allowed = null, idBase = 0) {
  const nN = graph.nNodes;
  const loadAt = new Map(loadNodes.map((ln, i) => [ln.node, i]));
  const feederOf = new Int32Array(loadNodes.length).fill(-1);
  const feeders = [];
  // Seed at load peaks first (dense town cores), Morton as tie-break —
  // feeders grow as compact balls around centres instead of stringy
  // frontier sweeps from the map corner.
  const order = loadNodes.map((ln, i) => ({ i, c: ln.cust, m: morton(graph.nx[ln.node], graph.ny[ln.node]) }))
    .sort((a, b) => b.c - a.c || a.m - b.m).map(o => o.i);
  const dist = new Float64Array(nN), stamp = new Int32Array(nN);
  let gen = 0;
  const inScope = (i) => (allowed ? allowed.has(i) : true);
  for (const si of order) {
    if (!inScope(si) || feederOf[si] !== -1) continue;
    const urban = loadNodes[si].urban;
    const cap = capOf(urban);
    const fid = idBase + feeders.length;
    let cust = 0;
    gen++;
    const heap = new NodeHeap();
    dist[loadNodes[si].node] = 0; stamp[loadNodes[si].node] = gen;
    heap.push(0, loadNodes[si].node);
    const seen = new Set();
    grow: while (heap.size) {
      const v = heap.pop();
      if (seen.has(v)) continue;
      seen.add(v);
      const d = dist[v];
      if (!urban && d > RURAL_EXTENT_KM_MAX * 1000) break;
      const li = loadAt.get(v);
      if (li !== undefined && inScope(li) && feederOf[li] === -1) {
        if (cust + loadNodes[li].cust > cap && li !== si) break grow;
        feederOf[li] = fid;
        cust += loadNodes[li].cust;
      }
      for (const ei of graph.adj[v]) {
        const e = graph.edges[ei];
        const w = e.a === v ? e.b : e.a;
        const nd = d + e.len;
        if (stamp[w] !== gen || nd < dist[w]) {
          stamp[w] = gen; dist[w] = nd;
          heap.push(nd, w);
        }
      }
    }
    feeders.push({ id: fid, seedLoad: si, urban, cust });
  }
  return { feederOf, feeders };
}

// Merge runt feeders (< FEEDER_MIN_CUST) into their road-nearest neighbour
// feeder when its cap allows. Returns compacted {feederOf, feeders}.
export function mergeRuntFeeders(graph, loadNodes, feederOf, feeders) {
  const nN = graph.nNodes;
  const loadAt = new Map(loadNodes.map((ln, i) => [ln.node, i]));
  const mergedInto = new Map();
  const resolve = (f) => { while (mergedInto.has(f)) f = mergedInto.get(f); return f; };
  const custOf = new Map(feeders.map(f => [f.id, f.cust]));
  const byId = new Map(feeders.map(f => [f.id, f]));
  const runts = feeders.filter(f => f.cust < FEEDER_MIN_CUST)
    .sort((a, b) => a.cust - b.cust || a.id - b.id);
  const dist = new Float64Array(nN), stamp = new Int32Array(nN);
  let gen = 0;
  for (const runt of runts) {
    if (resolve(runt.id) !== runt.id) continue;
    // multi-source Dijkstra out of the runt until another live feeder is hit
    gen++;
    const heap = new NodeHeap();
    for (let i = 0; i < loadNodes.length; i++) {
      if (resolve(feederOf[i]) === runt.id) {
        dist[loadNodes[i].node] = 0; stamp[loadNodes[i].node] = gen;
        heap.push(0, loadNodes[i].node);
      }
    }
    const seen = new Set();
    let target = -1;
    while (heap.size) {
      const v = heap.pop();
      if (seen.has(v)) continue;
      seen.add(v);
      const li = loadAt.get(v);
      if (li !== undefined) {
        const g = resolve(feederOf[li]);
        if (g !== runt.id &&
            custOf.get(g) + custOf.get(runt.id) <= capOf(byId.get(g).urban)) {
          target = g;
          break;
        }
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
    if (target === -1) continue; // genuinely isolated pocket — keep it
    mergedInto.set(runt.id, target);
    custOf.set(target, custOf.get(target) + custOf.get(runt.id));
  }
  return compactFeeders(loadNodes, feederOf, feeders, resolve);
}

// Renumber feeders 0..n-1 after merges; recompute size + majority class.
export function compactFeeders(loadNodes, feederOf, feeders, resolve = (f) => f) {
  const live = new Map(); // old id -> new id
  for (const f of feeders.sort((a, b) => a.id - b.id)) {
    const r = resolve(f.id);
    if (!live.has(r)) live.set(r, live.size);
  }
  const out = [...live.entries()].map(([oldId, id]) =>
    ({ id, seedLoad: feeders.find(f => f.id === oldId)?.seedLoad ?? -1, urban: false, cust: 0, urbanCust: 0 }));
  const newOf = new Int32Array(loadNodes.length).fill(-1);
  for (let i = 0; i < loadNodes.length; i++) {
    if (feederOf[i] === -1) continue;
    const id = live.get(resolve(feederOf[i]));
    newOf[i] = id;
    out[id].cust += loadNodes[i].cust;
    out[id].urbanCust += loadNodes[i].urbanCust;
  }
  for (const f of out) f.urban = f.urbanCust * 2 >= f.cust;
  return { feederOf: newOf, feeders: out };
}

// -------------------------------------- 3. feeders → zone subs (adjacency)

// Road-graph Voronoi: label every road node with the feeder of its nearest
// load node BY ROAD (multi-source Dijkstra, all load nodes at dist 0).
export function feederVoronoi(graph, loadNodes, feederOf) {
  const nN = graph.nNodes;
  const label = new Int32Array(nN).fill(-1);
  const dist = new Float64Array(nN).fill(Infinity);
  const heap = new NodeHeap();
  for (let i = 0; i < loadNodes.length; i++) {
    if (feederOf[i] === -1) continue;
    const v = loadNodes[i].node;
    if (0 < dist[v]) { dist[v] = 0; label[v] = feederOf[i]; heap.push(0, v); }
  }
  while (heap.size) {
    const v = heap.pop();
    for (const ei of graph.adj[v]) {
      const e = graph.edges[ei];
      const w = e.a === v ? e.b : e.a;
      const nd = dist[v] + e.len;
      if (nd < dist[w] - 1e-9) {
        dist[w] = nd; label[w] = label[v];
        heap.push(nd, w);
      }
    }
  }
  return label;
}

// Group road-adjacent feeders into zone-sub groups of ≤ FEEDERS_PER_SUB_MAX.
// Adjacency strength = shared Voronoi boundary length on the road graph —
// a shared road-graph neighbourhood, NOT straight-line proximity. Greedy
// agglomeration, strongest boundary first; a merge is refused when the
// combined group's bounding box would exceed GROUP_SPREAD_KM_MAX (stringy
// groups put subs far from their feeders). Deterministic tie-breaks.
export function groupFeeders(graph, loadNodes, feederOf, nFeeders) {
  const label = feederVoronoi(graph, loadNodes, feederOf);
  const strength = new Map(); // a*4096+b (a<b) -> shared boundary length
  for (const e of graph.edges) {
    const la = label[e.a], lb = label[e.b];
    if (la === -1 || lb === -1 || la === lb) continue;
    const k = Math.min(la, lb) * 4096 + Math.max(la, lb);
    strength.set(k, (strength.get(k) ?? 0) + e.len);
  }
  const parent = Array.from({ length: nFeeders }, (_, i) => i);
  const find = (x) => { while (parent[x] !== x) x = parent[x] = parent[parent[x]]; return x; };
  const size = new Int32Array(nFeeders).fill(1);
  // per-group bounding box over member load nodes
  const bbox = Array.from({ length: nFeeders }, () =>
    ({ x0: Infinity, y0: Infinity, x1: -Infinity, y1: -Infinity }));
  for (let i = 0; i < loadNodes.length; i++) {
    if (feederOf[i] === -1) continue;
    const b = bbox[feederOf[i]];
    const x = graph.nx[loadNodes[i].node], y = graph.ny[loadNodes[i].node];
    b.x0 = Math.min(b.x0, x); b.y0 = Math.min(b.y0, y);
    b.x1 = Math.max(b.x1, x); b.y1 = Math.max(b.y1, y);
  }
  const merged = (a, b) => ({
    x0: Math.min(a.x0, b.x0), y0: Math.min(a.y0, b.y0),
    x1: Math.max(a.x1, b.x1), y1: Math.max(a.y1, b.y1),
  });
  const diagKm = (b) => Math.hypot(b.x1 - b.x0, b.y1 - b.y0) / 1000;
  // compactness-first: prefer the merge with the smallest combined spread
  // (adjacency still required — every pair here shares a road boundary);
  // boundary strength breaks ties
  const pairs = [...strength.entries()]
    .map(([k, s]) => ({ a: (k / 4096) | 0, b: k % 4096, s }))
    .map(p => ({ ...p, d: diagKm(merged(bbox[p.a], bbox[p.b])) }))
    .sort((p, q) => p.d - q.d || q.s - p.s || p.a - q.a || p.b - q.b);
  for (let pass = 0, changed = true; changed && pass < nFeeders; pass++) {
    changed = false;
    for (const { a, b } of pairs) {
      const A = find(a), B = find(b);
      if (A === B || size[A] + size[B] > FEEDERS_PER_SUB_MAX) continue;
      const box = merged(bbox[A], bbox[B]);
      if (diagKm(box) > GROUP_SPREAD_KM_MAX) continue;
      parent[Math.max(A, B)] = Math.min(A, B);
      size[Math.min(A, B)] = size[A] + size[B];
      bbox[Math.min(A, B)] = box;
      changed = true;
    }
  }
  const groupIds = new Map();
  const groupOf = new Int32Array(nFeeders);
  for (let f = 0; f < nFeeders; f++) {
    const r = find(f);
    if (!groupIds.has(r)) groupIds.set(r, groupIds.size);
    groupOf[f] = groupIds.get(r);
  }
  return { groupOf, nGroups: groupIds.size, subLabelOfNode: label };
}
