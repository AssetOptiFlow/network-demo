// network.js — electrical model: distribution transformers, zone
// substations, and radial feeders routed ALONG the road graph.
//
// ASSUMPTIONS:
//  - Distribution transformers (TX): greedy capacitated clustering of
//    customers, max 50 per TX, gathered within ~500 m (urban) / 1500 m
//    (rural). TX sits at the road node nearest the cluster centroid.
//  - Zone substations: one per ~4000 customers (1–6), sited at road nodes
//    near customer-load centres (≥ 2.5 km apart), named for the nearest
//    town. Multi-source Dijkstra over roads assigns every TX to its
//    road-nearest sub.
//  - Feeders: each sub's shortest-path tree is PARTITIONED into feeders
//    sized by local linear customer density — target ~250 customers rural
//    up to ~700 urban (≈400 average), matching the untidy way real
//    networks split load. Each feeder head connects back to the sub by an
//    "express" run of parallel circuit along the same roads; its fault
//    exposure is charged to the feeder as un-switchable base SAIDI.
//  - Sections through high-density cells are UNDERGROUND cable, the rest
//    overhead line (per-type fault rates live in reliability.js).
//  - LV detail below the TX is ignored; customers hang off their TX.

export const TX_CAP = 50;
// Town peaks sit near 1.0 (mass-compensated), so this keeps cable to the
// inner core (roughly r < σ) rather than whole towns.
export const UG_DENSITY_THRESH = 0.55; // density units — inner-urban ⇒ cable
// Zone-sub sizing rules (k selection for the catchment k-means):
//  - an urban sub serves at most ~4000 customers (oversize catchments split)
//  - a rural sub can serve as few as 500 (smaller catchments merge)
const SUB_MAX_CUST = 4000;
const SUB_MIN_CUST = 500;
const MAX_SUBS = 10;
// Cut targets are set above the desired averages (rural ~250, urban ~700,
// overall ~400) because overshoot-at-cut and small branch-root remainders
// pull realised feeder sizes below the cut threshold.
const FEEDER_TARGET_RURAL = 300, FEEDER_TARGET_URBAN = 780;
const MIN_MERGE = 200;   // feeders below this merge into their largest child
const MAX_MERGED = 1000; // …unless the result would exceed this
// A feeder head more than this far (by road) from its sub would mean a
// long parallel "express" circuit down one rural road — implausible, so
// such feeders merge into the feeder that owns their trunk instead (long
// single rural feeders are the realistic outcome, even over-size).
const MAX_EXPRESS_M = 4000;

// Density-scaled feeder size target: rural long skinny feeders carry fewer
// customers, tight urban feeders carry more. Urban-ness = the stronger of
// linear customer density (30→150 cust/km) and underground share (UG cable
// exists exactly where density is high).
function feederTarget(cust, lenM, ugLenM) {
  const perKm = cust / Math.max(0.3, lenM / 1000);
  const uDensity = Math.max(0, Math.min(1, (perKm - 30) / 120));
  const uUg = Math.max(0, Math.min(1, (ugLenM / Math.max(1, lenM) - 0.15) / 0.5));
  const u = Math.max(uDensity, uUg);
  return FEEDER_TARGET_RURAL + u * (FEEDER_TARGET_URBAN - FEEDER_TARGET_RURAL);
}

// STRICT LAYER ORDER: subs are placed FIRST (k-means catchments over the
// load, see placeSubs) and passed in; feeders are the LAST layer, grown
// from the subs over the road graph.
export function buildNetwork(terrain, graph, customers, towns, density, subs, rng) {
  // ---------- 1. capacitated TX clustering (greedy, deterministic)
  const txs = clusterTransformers(customers, rng);
  for (const tx of txs) {
    tx.node = graph.nearestNode(tx.x, tx.y, 30000).id;
  }

  // ---------- 3. multi-source Dijkstra over road graph
  const nN = graph.nNodes;
  const dist = new Float64Array(nN).fill(Infinity);
  const parent = new Int32Array(nN).fill(-1);
  const parentEdge = new Int32Array(nN).fill(-1);
  const subOf = new Int32Array(nN).fill(-1);
  const heap = new NodeHeap();
  for (const s of subs) {
    dist[s.node] = 0; subOf[s.node] = s.id;
    heap.push(0, s.node);
  }
  while (heap.size) {
    const v = heap.pop();
    for (const ei of graph.adj[v]) {
      const e = graph.edges[ei];
      const w = e.a === v ? e.b : e.a;
      const nd = dist[v] + e.len;
      if (nd < dist[w] - 1e-9) {
        dist[w] = nd; parent[w] = v; parentEdge[w] = ei; subOf[w] = subOf[v];
        heap.push(nd, w);
      }
    }
  }

  // ---------- 4. prune to the union of TX→sub paths
  const usedEdge = new Uint8Array(graph.edges.length);
  const custAtNode = new Float64Array(nN);
  let orphanTx = 0;
  for (const tx of txs) {
    if (tx.node === -1 || !isFinite(dist[tx.node])) { orphanTx++; continue; }
    tx.sub = subOf[tx.node];
    custAtNode[tx.node] += tx.customers.length;
    for (let v = tx.node; parent[v] !== -1; v = parent[v]) {
      if (usedEdge[parentEdge[v]]) break; // rest of path already marked
      usedEdge[parentEdge[v]] = 1;
    }
  }

  // ---------- 5. partition each sub tree into feeders (~400 cust target)
  const feederOfNode = new Int32Array(nN).fill(-1);
  const subtreeCust = new Float64Array(nN); // FEEDER-LOCAL subtree customers
  const accCust = new Float64Array(nN);
  const accLen = new Float64Array(nN);
  const accUg = new Float64Array(nN);
  const allOrder = []; // parent-before-child, contiguous per sub
  const rawFeeders = []; // {sub, rootNode, cust} before runt merging

  const gridN = terrain.n;
  const isUnderground = (a, b) => {
    const mx = (graph.nx[a] + graph.nx[b]) / 2;
    const my = (graph.ny[a] + graph.ny[b]) / 2;
    const [cx, cy] = terrain.cellOf(mx, my);
    return density.grid[cy * gridN + cx] > UG_DENSITY_THRESH;
  };

  for (const s of subs) {
    feederOfNode[s.node] = -2; // sub busbar: not on any single feeder
    const order = [];
    const stack = [];
    for (const ei of graph.adj[s.node]) {
      if (!usedEdge[ei]) continue;
      const e = graph.edges[ei];
      const child = e.a === s.node ? e.b : e.a;
      if (parent[child] === s.node) stack.push(child);
    }
    while (stack.length) {
      const v = stack.pop();
      order.push(v);
      for (const ej of graph.adj[v]) {
        if (!usedEdge[ej]) continue;
        const e2 = graph.edges[ej];
        const w = e2.a === v ? e2.b : e2.a;
        if (parent[w] === v) stack.push(w);
      }
    }
    // Reverse (post-order) accumulate + cut into feeders. At junctions the
    // combined total can leap far past the target ("overshoot"), so large
    // child subtrees are split off as their own feeders first.
    const cutFeeder = new Map(); // node -> raw feeder id
    for (let i = order.length - 1; i >= 0; i--) {
      const v = order[i];
      const ownLen = graph.edges[parentEdge[v]].len;
      let cust = custAtNode[v], len = ownLen;
      let ug = isUnderground(v, parent[v]) ? ownLen : 0;
      const kids = [];
      for (const ej of graph.adj[v]) {
        if (!usedEdge[ej]) continue;
        const e2 = graph.edges[ej];
        const w = e2.a === v ? e2.b : e2.a;
        if (parent[w] === v && !cutFeeder.has(w)) kids.push(w);
      }
      for (const w of kids) { cust += accCust[w]; len += accLen[w]; ug += accUg[w]; }
      let guard = kids.length;
      while (guard-- > 0 && cust >= 1.3 * feederTarget(cust, len, ug)) {
        let big = -1;
        for (const w of kids) {
          if (cutFeeder.has(w)) continue;
          if (big === -1 || accCust[w] > accCust[big]) big = w;
        }
        if (big === -1 || accCust[big] < MIN_MERGE) break;
        cutFeeder.set(big, rawFeeders.length);
        rawFeeders.push({ sub: s.id, rootNode: big, cust: accCust[big] });
        cust -= accCust[big]; len -= accLen[big]; ug -= accUg[big];
      }
      accCust[v] = cust; accLen[v] = len; accUg[v] = ug;
      const isBranchRoot = parent[v] === s.node;
      if (isBranchRoot || cust >= feederTarget(cust, len, ug)) {
        cutFeeder.set(v, rawFeeders.length);
        rawFeeders.push({ sub: s.id, rootNode: v, cust });
      }
    }
    // Top-down feeder assignment (order is parent-before-child).
    for (const v of order) {
      feederOfNode[v] = cutFeeder.has(v) ? cutFeeder.get(v) : feederOfNode[parent[v]];
    }
    allOrder.push(...order);
  }

  // Merge runt feeders (usually the trunk remainder above the last cut,
  // occasionally left with 0 customers) into their largest child feeder —
  // the child's root moves up to absorb the trunk.
  const mergedInto = new Int32Array(rawFeeders.length).fill(-1);
  const resolve = (fid) => { while (mergedInto[fid] !== -1) fid = mergedInto[fid]; return fid; };
  for (let pass = 0, changed = true; changed && pass < rawFeeders.length; pass++) {
    changed = false;
    const cust = new Float64Array(rawFeeders.length);
    for (let i = 0; i < rawFeeders.length; i++) cust[resolve(i)] += rawFeeders[i].cust;
    for (let i = 0; i < rawFeeders.length; i++) {
      if (mergedInto[i] !== -1 || cust[i] >= MIN_MERGE) continue;
      let best = -1;
      for (let g = 0; g < rawFeeders.length; g++) {
        if (g === i || mergedInto[g] !== -1) continue;
        const pn = parent[rawFeeders[g].rootNode];
        if (pn === -1) continue;
        const above = feederOfNode[pn];
        if (above < 0 || resolve(above) !== i) continue; // g hangs off i
        if (best === -1 || cust[g] > cust[best]) best = g;
      }
      if (best === -1) continue; // genuine small leaf feeder — keep it
      // A near-empty trunk is not a feeder at all — always absorb it;
      // otherwise respect the size cap.
      if (cust[i] >= 50 && cust[i] + cust[best] > MAX_MERGED) continue;
      mergedInto[i] = best;
      cust[best] += cust[i]; cust[i] = 0; // keep in-pass sizes current
      rawFeeders[best].rootNode = rawFeeders[i].rootNode; // root moves up
      changed = true;
    }
    // Express merges: a feeder head too far from the sub joins the feeder
    // that owns its trunk — but only up to the size cap. A full corridor
    // keeps its long express honestly: that IS the second circuit a
    // utility would string when one rural feeder can't carry the area.
    for (let i = 0; i < rawFeeders.length; i++) {
      if (mergedInto[i] !== -1) continue;
      const subNode = subs[rawFeeders[i].sub].node;
      let exLen = 0;
      for (let w = parent[rawFeeders[i].rootNode]; w !== subNode && w !== -1; w = parent[w]) {
        exLen += graph.edges[parentEdge[w]].len;
      }
      if (exLen <= MAX_EXPRESS_M) continue;
      const pn = parent[rawFeeders[i].rootNode];
      if (pn === -1) continue;
      const above = feederOfNode[pn];
      if (above < 0) continue;
      const target = resolve(above);
      if (target === i) continue;
      if (cust[i] + cust[target] > MAX_MERGED) continue; // corridor is full
      mergedInto[i] = target;
      cust[target] += cust[i]; cust[i] = 0;
      changed = true;
    }
  }
  const finalId = new Int32Array(rawFeeders.length).fill(-1);
  const feeders = [];
  for (let i = 0; i < rawFeeders.length; i++) {
    if (mergedInto[i] !== -1) continue;
    finalId[i] = feeders.length;
    feeders.push({
      id: feeders.length, sub: rawFeeders[i].sub, rootNode: rawFeeders[i].rootNode,
      nodes: [], edges: [], customers: 0,
      lengthM: 0, ohLenM: 0, ugLenM: 0, txCount: 0,
      expressOhKm: 0, expressUgKm: 0, expressMidM: 0,
    });
  }
  for (const v of allOrder) {
    if (feederOfNode[v] >= 0) feederOfNode[v] = finalId[resolve(feederOfNode[v])];
  }
  // Feeder-local subtree customer counts (final assignment). allOrder is
  // contiguous parent-before-child per sub, so a reverse sweep works.
  for (let i = allOrder.length - 1; i >= 0; i--) {
    const v = allOrder[i];
    subtreeCust[v] += custAtNode[v];
    const p = parent[v];
    if (p !== -1 && feederOfNode[p] === feederOfNode[v]) subtreeCust[p] += subtreeCust[v];
  }
  for (const f of feeders) f.customers = subtreeCust[f.rootNode];

  // ---------- 6. tree edges (one per used node), OH/UG classification
  const treeEdges = [];
  const treeEdgeOfNode = new Int32Array(nN).fill(-1);
  for (const v of allOrder) {
    const fid = feederOfNode[v];
    if (fid < 0) continue;
    const ei = parentEdge[v];
    const e = graph.edges[ei];
    const te = {
      id: treeEdges.length, node: v, parentNode: parent[v], edgeId: ei,
      feeder: fid, lenM: e.len,
      midDistM: (dist[v] + dist[parent[v]]) / 2,
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
  }

  // ---------- 7. express runs: feeder head back to the sub busbar
  for (const f of feeders) {
    const subNode = subs[f.sub].node;
    let ohM = 0, ugM = 0;
    for (let w = parent[f.rootNode]; w !== subNode && w !== -1; w = parent[w]) {
      const teId = treeEdgeOfNode[w];
      if (teId === -1) break;
      const te = treeEdges[teId];
      if (te.underground) ugM += te.lenM; else ohM += te.lenM;
    }
    f.expressOhKm = ohM / 1000;
    f.expressUgKm = ugM / 1000;
    f.expressMidM = (ohM + ugM) / 2;
  }

  // ---------- 8. attach TXs / customers to feeders
  for (const tx of txs) {
    if (tx.node !== -1 && feederOfNode[tx.node] >= 0) {
      tx.feeder = feederOfNode[tx.node];
      feeders[tx.feeder].txCount++;
    } else if (tx.node !== -1 && feederOfNode[tx.node] === -2) {
      // TX exactly at a sub busbar: attach to that sub's largest feeder
      const f = feeders.filter(f => f.sub === subOf[tx.node])
        .sort((a, b) => b.customers - a.customers)[0];
      tx.feeder = f ? f.id : -1;
      if (f) { f.customers += tx.customers.length; f.txCount++; }
    } else {
      tx.feeder = -1;
    }
  }
  for (const tx of txs) for (const ci of tx.customers) customers[ci].tx = tx.id;

  return {
    txs, subs, feeders,
    dist, parent, parentEdge, subOf, feederOfNode,
    subtreeCust, custAtNode, treeEdges, treeEdgeOfNode,
    usedEdge, orphanTx,
  };
}

// Zone subs = LOAD CATCHMENTS. STRICT LAYER ORDER: k-means over the
// customer points (Lloyd, deterministic farthest-point seeding); k adapts
// until no catchment exceeds SUB_MAX_CUST (urban split) and none that can
// be merged sits below SUB_MIN_CUST. Each sub is placed at its catchment's
// LOAD-WEIGHTED CENTROID, then nudged to the nearest subtransmission-
// viable road node (on an arterial/collector corridor, low slope) —
// never a geometric centre, never a town marker.
export function placeSubs(terrain, graph, customers, towns) {
  const N = customers.length;
  const k = Math.max(1, Math.min(MAX_SUBS, Math.round(N / 2500)));
  let clusters = lloydClusters(customers, k);
  // Targeted fix-ups (global k bumps can ping-pong when one catchment is
  // oversize and another undersize at the same time): split any catchment
  // over SUB_MAX_CUST in two, merge any under SUB_MIN_CUST into its
  // nearest neighbour. Deterministic, bounded passes.
  for (let pass = 0; pass < 8; pass++) {
    clusters.sort((a, b) => b.members.length - a.members.length);
    const over = clusters.find(c => c.members.length > SUB_MAX_CUST);
    if (over && clusters.length < MAX_SUBS) {
      clusters = clusters.filter(c => c !== over)
        .concat(splitCluster(customers, over));
      continue;
    }
    const under = clusters.length > 1
      ? clusters.slice().sort((a, b) => a.members.length - b.members.length)
        .find(c => c.members.length < SUB_MIN_CUST)
      : null;
    if (under) {
      let near = null, nd = Infinity;
      for (const c of clusters) {
        if (c === under) continue;
        const d = Math.hypot(c.cx - under.cx, c.cy - under.cy);
        if (d < nd) { nd = d; near = c; }
      }
      near.members = near.members.concat(under.members);
      recentre(customers, near);
      clusters = clusters.filter(c => c !== under);
      continue;
    }
    break;
  }
  const subs = [];
  for (const cl of clusters) {
    cl.count = cl.members.length;
    if (!cl.count) continue;
    const node = nudgeToCorridor(terrain, graph, cl.cx, cl.cy, subs);
    if (node === -1) continue;
    let town = towns[0], bd = Infinity;
    for (const t of towns) {
      const d = Math.hypot(t.x - cl.cx, t.y - cl.cy);
      if (d < bd) { bd = d; town = t; }
    }
    const dupes = subs.filter(s => s.baseName === town.name).length;
    subs.push({
      id: subs.length, node,
      x: graph.nx[node], y: graph.ny[node],
      centroidX: cl.cx, centroidY: cl.cy, catchment: cl.count,
      baseName: town.name,
      name: town.name + (dupes ? " " + "BCDEFG"[dupes - 1] : ""),
    });
  }
  return subs;
}

// Deterministic Lloyd k-means over customer points (weight 1 per ICP).
// Seeding: the overall load centroid's nearest customer, then repeated
// farthest-point picks (no randomness → same seed, same catchments).
function lloydClusters(customers, k) {
  const N = customers.length;
  let mx = 0, my = 0;
  for (const c of customers) { mx += c.x; my += c.y; }
  mx /= N; my /= N;
  const cent = [];
  let best = 0, bd = Infinity;
  for (let i = 0; i < N; i++) {
    const d = (customers[i].x - mx) ** 2 + (customers[i].y - my) ** 2;
    if (d < bd) { bd = d; best = i; }
  }
  cent.push([customers[best].x, customers[best].y]);
  const minD = new Float64Array(N).fill(Infinity);
  while (cent.length < k) {
    let far = 0, fd = -1;
    const [lx, ly] = cent[cent.length - 1];
    for (let i = 0; i < N; i++) {
      const d = (customers[i].x - lx) ** 2 + (customers[i].y - ly) ** 2;
      if (d < minD[i]) minD[i] = d;
      if (minD[i] > fd) { fd = minD[i]; far = i; }
    }
    cent.push([customers[far].x, customers[far].y]);
  }
  const assign = new Int32Array(N);
  for (let iter = 0; iter < 10; iter++) {
    for (let i = 0; i < N; i++) {
      let a = 0, ad = Infinity;
      for (let c = 0; c < cent.length; c++) {
        const d = (customers[i].x - cent[c][0]) ** 2 + (customers[i].y - cent[c][1]) ** 2;
        if (d < ad) { ad = d; a = c; }
      }
      assign[i] = a;
    }
    const sx = new Float64Array(cent.length), sy = new Float64Array(cent.length);
    const cnt = new Int32Array(cent.length);
    for (let i = 0; i < N; i++) {
      sx[assign[i]] += customers[i].x; sy[assign[i]] += customers[i].y; cnt[assign[i]]++;
    }
    for (let c = 0; c < cent.length; c++) {
      if (cnt[c]) { cent[c][0] = sx[c] / cnt[c]; cent[c][1] = sy[c] / cnt[c]; }
    }
  }
  const out = cent.map(([cx, cy]) => ({ cx, cy, members: [] }));
  for (let i = 0; i < N; i++) out[assign[i]].members.push(i);
  return out.filter(c => c.members.length > 0);
}

function recentre(customers, cl) {
  let sx = 0, sy = 0;
  for (const i of cl.members) { sx += customers[i].x; sy += customers[i].y; }
  cl.cx = sx / cl.members.length;
  cl.cy = sy / cl.members.length;
}

// Deterministic 2-means split of an oversize catchment.
function splitCluster(customers, cl) {
  const ms = cl.members;
  // seeds: member nearest the centroid, member farthest from it
  let a = ms[0], ad = Infinity, b = ms[0], bd = -1;
  for (const i of ms) {
    const d = (customers[i].x - cl.cx) ** 2 + (customers[i].y - cl.cy) ** 2;
    if (d < ad) { ad = d; a = i; }
    if (d > bd) { bd = d; b = i; }
  }
  let ca = [customers[a].x, customers[a].y], cb = [customers[b].x, customers[b].y];
  let ga = [], gb = [];
  for (let iter = 0; iter < 8; iter++) {
    ga = []; gb = [];
    for (const i of ms) {
      const da = (customers[i].x - ca[0]) ** 2 + (customers[i].y - ca[1]) ** 2;
      const db = (customers[i].x - cb[0]) ** 2 + (customers[i].y - cb[1]) ** 2;
      (da <= db ? ga : gb).push(i);
    }
    for (const [g, c] of [[ga, ca], [gb, cb]]) {
      if (!g.length) continue;
      let sx = 0, sy = 0;
      for (const i of g) { sx += customers[i].x; sy += customers[i].y; }
      c[0] = sx / g.length; c[1] = sy / g.length;
    }
  }
  const mk = (g, c) => ({ cx: c[0], cy: c[1], members: g });
  return [mk(ga, ca), mk(gb, cb)].filter(c => c.members.length > 0);
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

class NodeHeap {
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

function morton(x, y) {
  let a = Math.min(65535, Math.max(0, Math.round(x / 2)));
  let b = Math.min(65535, Math.max(0, Math.round(y / 2)));
  let m = 0;
  for (let i = 0; i < 16; i++) {
    m += ((a >> i) & 1) * Math.pow(2, 2 * i) + ((b >> i) & 1) * Math.pow(2, 2 * i + 1);
  }
  return m;
}
