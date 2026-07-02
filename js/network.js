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
export const UG_DENSITY_THRESH = 0.30; // density units — inner-urban ⇒ cable
// Zone-sub siting rules (greedy facility location over road distance):
//  - an urban sub serves at most ~4000 customers (overloaded regions split)
//  - a rural sub is justified by as few as 500 customers IF it saves them
//    ≥ 2 km of road distance each on average
const SUB_MAX_CUST = 4000;
const SUB_MIN_CUST = 500;
const SUB_MIN_SAVING_M = 2000;
const MAX_SUBS = 10;
const SUB_MIN_SEP_M = 2000;
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

export function buildNetwork(terrain, graph, customers, towns, density, rng) {
  // ---------- 1. capacitated TX clustering (greedy, deterministic)
  const txs = clusterTransformers(customers, rng);
  for (const tx of txs) {
    tx.node = graph.nearestNode(tx.x, tx.y, 30000).id;
  }

  // ---------- 2. zone substations: greedy facility location
  const subs = placeSubs(graph, customers, towns);

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

// Zone subs by greedy facility location (weighted 1-median over ROAD
// distance). Candidate sites = road nodes of the top load bins. The first
// sub minimises total customer·distance; each further sub is the candidate
// saving the most customer·distance, ACCEPTED only if it captures ≥500
// customers and either saves them ≥2 km each on average (a worthwhile
// rural sub) or relieves a sub serving >4000 (an urban split).
function placeSubs(graph, customers, towns) {
  const nN = graph.nNodes;
  const custAtNode = new Float64Array(nN);
  for (const c of customers) if (c.node >= 0) custAtNode[c.node] += 1;
  const loaded = [];
  for (let v = 0; v < nN; v++) if (custAtNode[v] > 0) loaded.push(v);

  // candidate sites from 1.2 km load bins
  const BIN = 1200;
  const bins = new Map();
  for (const c of customers) {
    const k = ((c.x / BIN) | 0) * 8192 + ((c.y / BIN) | 0);
    let b = bins.get(k);
    if (!b) bins.set(k, b = { x: 0, y: 0, n: 0 });
    b.x += c.x; b.y += c.y; b.n++;
  }
  const binList = [...bins.values()]
    .map(b => ({ x: b.x / b.n, y: b.y / b.n, n: b.n }))
    .sort((a, b) => b.n - a.n || a.x - b.x || a.y - b.y);
  // Coverage candidates: the top bins are all in towns, which would leave
  // sparse districts with no candidate site at all — so also nominate the
  // densest bin of every 6 km super-cell holding ≥200 customers.
  const SUPER = 6000;
  const superCells = new Map(); // key -> {total, best}
  for (const b of binList) {
    const sk = ((b.x / SUPER) | 0) * 128 + ((b.y / SUPER) | 0);
    let sc = superCells.get(sk);
    if (!sc) superCells.set(sk, sc = { total: 0, best: null });
    sc.total += b.n;
    if (!sc.best || b.n > sc.best.n) sc.best = b;
  }
  const candBins = binList.slice(0, 60);
  for (const sc of superCells.values()) {
    if (sc.total >= 200) candBins.push(sc.best);
  }
  const candNodes = [];
  const seen = new Set();
  for (const b of candBins) {
    const near = graph.nearestNode(b.x, b.y, 30000);
    if (near.id !== -1 && !seen.has(near.id)) { seen.add(near.id); candNodes.push(near.id); }
  }

  // road-distance field from every candidate (reused across greedy steps)
  const fields = candNodes.map(node => dijkstraField(graph, node));

  const chosenIdx = [];
  const curDist = new Float64Array(nN).fill(Infinity); // to nearest chosen sub
  const regionOf = new Int32Array(nN).fill(-1);
  const regionCust = [];
  while (chosenIdx.length < MAX_SUBS) {
    let best = -1, bestSaving = -Infinity, bestCaptured = 0;
    for (let ci = 0; ci < candNodes.length; ci++) {
      if (chosenIdx.includes(ci)) continue;
      if (chosenIdx.some(j => Math.hypot(
        graph.nx[candNodes[j]] - graph.nx[candNodes[ci]],
        graph.ny[candNodes[j]] - graph.ny[candNodes[ci]]) < SUB_MIN_SEP_M)) continue;
      let saving = 0, captured = 0;
      for (const v of loaded) {
        const d = fields[ci][v];
        if (d < curDist[v]) {
          captured += custAtNode[v];
          saving += custAtNode[v] * ((curDist[v] === Infinity ? 60000 : curDist[v]) - d);
        }
      }
      if (saving > bestSaving) { bestSaving = saving; best = ci; bestCaptured = captured; }
    }
    if (best === -1) break;
    if (chosenIdx.length > 0) {
      const meanSaving = bestSaving / Math.max(1, bestCaptured);
      const overloaded = regionCust.some(n => n > SUB_MAX_CUST);
      const worthIt = bestCaptured >= SUB_MIN_CUST &&
        (meanSaving >= SUB_MIN_SAVING_M || overloaded);
      if (!worthIt) break;
    }
    // accept: update assignment
    const si = chosenIdx.length;
    chosenIdx.push(best);
    regionCust.push(0);
    for (const v of loaded) {
      const d = fields[best][v];
      if (d < curDist[v]) {
        if (regionOf[v] >= 0) regionCust[regionOf[v]] -= custAtNode[v];
        curDist[v] = d;
        regionOf[v] = si;
        regionCust[si] += custAtNode[v];
      }
    }
  }

  // name subs for the nearest town
  const subs = [];
  for (const ci of chosenIdx) {
    const node = candNodes[ci];
    const x = graph.nx[node], y = graph.ny[node];
    let town = towns[0], bd = Infinity;
    for (const t of towns) {
      const d = Math.hypot(t.x - x, t.y - y);
      if (d < bd) { bd = d; town = t; }
    }
    const dupes = subs.filter(s => s.baseName === town.name).length;
    subs.push({
      id: subs.length, node, x, y,
      baseName: town.name,
      name: town.name + (dupes ? " " + "BCDEFG"[dupes - 1] : ""),
    });
  }
  return subs;
}

function dijkstraField(graph, source) {
  const dist = new Float64Array(graph.nNodes).fill(Infinity);
  const heap = new NodeHeap();
  dist[source] = 0;
  heap.push(0, source);
  while (heap.size) {
    const v = heap.pop();
    for (const ei of graph.adj[v]) {
      const e = graph.edges[ei];
      const w = e.a === v ? e.b : e.a;
      const nd = dist[v] + e.len;
      if (nd < dist[w] - 1e-9) { dist[w] = nd; heap.push(nd, w); }
    }
  }
  return dist;
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
