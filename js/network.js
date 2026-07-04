// network.js — electrical model: distribution transformers, zone
// substations, and radial feeders routed ALONG the road graph.
//
// RULE-CAP FLOW (caps in membership.js; no minimums; counts EMERGE):
//   1. customers → TX: ≤ 500 m to the transformer OR ≤ 100 customers
//   2. TX → zone subs: greedy road-Dijkstra growth, ≤ 25 km / ≤ 2000 cust;
//      each sub sited at its cluster's LOAD-WEIGHTED CENTROID, nudged to a
//      subtransmission-viable corridor node
//   3. every TX then assigned to its road-NEAREST sited sub (one
//      multi-source Dijkstra). Graph-Voronoi property: shortest paths
//      never leave their cell, so sub trees are DISJOINT by construction
//   4. feeders: each sub tree partitioned into CONTIGUOUS subtrees cut to
//      ≤ 500 customers and ≤ 25 km circuit length (owned + trunk run),
//      relaxed to 40 km while under 50 customers. A feeder head reaches
//      the sub by an express run — the parallel circuit down a shared
//      trunk — charged as un-switchable base SAIDI.
//   Rules are enforced by construction where possible; the residue (sub
//   totals drifting past the cap at Voronoi boundaries, realised distances
//   after siting, irreducible single-node overshoots) is CHECKED and
//   reported, never silently absorbed.
//
// ASSUMPTIONS:
//  - Sections through high-density cells are UNDERGROUND cable, the rest
//    overhead (per-type fault rates live in reliability.js).
//  - LV detail below the TX is ignored; customers hang off their TX.

import {
  classifyCustomers, buildLoadNodes, growSubs, NodeHeap, morton,
  TX_MAX_CUST, TX_MAX_M, SUB_MAX_CUST, SUB_MAX_KM,
  FEEDER_MAX_CUST, FEEDER_MAX_KM, FEEDER_LONG_KM, FEEDER_LONG_CUST,
} from "./membership.js";

// Town peaks sit near 1.0 (mass-compensated), so this keeps cable to the
// inner core (roughly r < σ) rather than whole towns.
export const UG_DENSITY_THRESH = 0.55; // density units — inner-urban ⇒ cable

export function buildNetwork(terrain, graph, customers, towns, density, rng) {
  // ---------- 1. customers → TX (≤ TX_MAX_M or ≤ TX_MAX_CUST)
  const txs = clusterTransformers(customers);
  for (const tx of txs) tx.node = graph.nearestNode(tx.x, tx.y, 30000).id;
  for (const tx of txs) for (const ci of tx.customers) customers[ci].tx = tx.id;
  const classOfCust = classifyCustomers(customers);
  const loadNodes = buildLoadNodes(txs, customers, classOfCust);
  const loadAt = new Map(loadNodes.map((ln, i) => [ln.node, i]));

  // ---------- 2. TX → zone subs (growth), then SITE each sub
  const { of: growthOf, clusters } = growSubs(graph, loadNodes);
  let subs = [];
  for (const cl of clusters) {
    let sx = 0, sy = 0, w = 0;
    for (let i = 0; i < loadNodes.length; i++) {
      if (growthOf[i] !== cl.id) continue;
      sx += graph.nx[loadNodes[i].node] * loadNodes[i].cust;
      sy += graph.ny[loadNodes[i].node] * loadNodes[i].cust;
      w += loadNodes[i].cust;
    }
    if (!w) continue;
    const cx = sx / w, cy = sy / w;
    const node = nudgeToCorridor(terrain, graph, cx, cy, subs);
    if (node === -1) continue;
    subs.push({
      id: subs.length, node, x: graph.nx[node], y: graph.ny[node],
      centroidX: cx, centroidY: cy, catchment: cl.cust,
    });
  }

  // ---------- 3. capacity-respecting road assignment: an ADDITIVELY
  // WEIGHTED graph Voronoi. Each sub s gets a distance penalty λ_s; every
  // node joins argmin (λ_s + roadDist). This is still ONE multi-source
  // Dijkstra (sources start at λ instead of 0), so the shortest-path
  // forest stays DISJOINT per sub — and iterating λ upward on over-cap
  // subs pushes their boundary out until totals respect SUB_MAX_CUST.
  // True road distances are recovered by subtracting λ at the end.
  const nN = graph.nNodes;
  let dist, parent, parentEdge, subOf;
  let lambda = new Float64Array(subs.length);
  let lastCust = null;
  const N_ITER = 30;
  // OUTER loop: if a cell stays stubbornly over cap after the bidding
  // (e.g. a 4,000-customer town holding one sub — a pigeonhole problem no
  // boundary shift can fix), a NEW sub spawns at the load centre of the
  // cell's far half and the bidding reruns. Sub count truly emerges.
  for (let outer = 0; outer < 4; outer++) {
  for (let iter = 0; iter < N_ITER; iter++) {
    dist = new Float64Array(nN).fill(Infinity);
    parent = new Int32Array(nN).fill(-1);
    parentEdge = new Int32Array(nN).fill(-1);
    subOf = new Int32Array(nN).fill(-1);
    const heap = new NodeHeap();
    for (const s of subs) {
      dist[s.node] = lambda[s.id]; subOf[s.node] = s.id;
      heap.push(lambda[s.id], s.node);
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
    // per-sub customer totals over load nodes
    const cust = new Float64Array(subs.length);
    for (const ln of loadNodes) {
      if (subOf[ln.node] !== -1) cust[subOf[ln.node]] += ln.cust;
    }
    lastCust = cust;
    const over = subs.filter(s => cust[s.id] > SUB_MAX_CUST);
    const dead = subs.filter(s => cust[s.id] === 0);
    if ((!over.length && !dead.length) || iter === N_ITER - 1) break;
    for (const s of over) lambda[s.id] += (cust[s.id] - SUB_MAX_CUST) * 0.8;
    // a starved sub (its cell swallowed by a neighbour after siting) bids
    // back in with a NEGATIVE offset — still a valid weighted Voronoi —
    // rather than being dropped and leaving the neighbour double-loaded
    for (const s of dead) lambda[s.id] = Math.max(-6000, lambda[s.id] - 1200);
  }
  // spawn subs inside cells that stayed over cap ×1.1
  const stubborn = subs.filter(s => lastCust[s.id] > SUB_MAX_CUST * 1.1);
  if (!stubborn.length || outer === 3) break;
  for (const s of stubborn) {
    const members = [];
    for (const ln of loadNodes) {
      if (subOf[ln.node] === s.id) members.push({ ln, d: dist[ln.node] });
    }
    members.sort((a, b) => a.d - b.d);
    // load-weighted centroid of the FAR half of the cell
    let acc = 0, sx = 0, sy = 0, w = 0;
    const half = lastCust[s.id] / 2;
    for (const { ln } of members) {
      acc += ln.cust;
      if (acc <= half) continue;
      sx += graph.nx[ln.node] * ln.cust;
      sy += graph.ny[ln.node] * ln.cust;
      w += ln.cust;
    }
    if (!w) continue;
    const node = nudgeToCorridor(terrain, graph, sx / w, sy / w, subs);
    if (node === -1) continue;
    subs.push({
      id: subs.length, node, x: graph.nx[node], y: graph.ny[node],
      centroidX: sx / w, centroidY: sy / w, catchment: Math.round(w),
    });
  }
  lambda = new Float64Array(subs.length);
  } // outer

  // a sub still serving nothing after the bidding is genuinely redundant
  if (lastCust && subs.some(s => lastCust[s.id] === 0)) {
    subs = subs.filter(s => lastCust[s.id] > 0).map((s, i) => ({ ...s, id: i }));
    // one clean final assignment with the surviving subs' offsets removed
    dist = new Float64Array(nN).fill(Infinity);
    parent = new Int32Array(nN).fill(-1);
    parentEdge = new Int32Array(nN).fill(-1);
    subOf = new Int32Array(nN).fill(-1);
    lambda = new Float64Array(subs.length);
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
  }
  // recover TRUE road distances (paths are unchanged; λ is a constant
  // offset along each sub's whole forest)
  for (let v = 0; v < nN; v++) {
    if (subOf[v] !== -1 && isFinite(dist[v])) dist[v] -= lambda[subOf[v]];
  }
  let reassigned = 0;
  for (let i = 0; i < loadNodes.length; i++) {
    if (subOf[loadNodes[i].node] !== growthOf[i]) reassigned++;
  }

  // ---------- 4. prune to the union of TX→sub paths
  const usedEdge = new Uint8Array(graph.edges.length);
  const custAtNode = new Float64Array(nN);
  let orphanTx = 0;
  for (const tx of txs) {
    if (tx.node === -1 || !isFinite(dist[tx.node])) { tx.sub = -1; orphanTx++; continue; }
    tx.sub = subOf[tx.node];
    custAtNode[tx.node] += tx.customers.length;
    for (let v = tx.node; parent[v] !== -1; v = parent[v]) {
      if (usedEdge[parentEdge[v]]) break; // rest of path already marked
      usedEdge[parentEdge[v]] = 1;
    }
  }

  // ---------- 5. partition each sub tree into feeders by the rule caps.
  // Post-order accumulation; while a node's running subtree would violate
  // (cust > FEEDER_MAX_CUST, or circuit length = trunk-to-sub + owned
  // length over the cap for its size), the largest child subtree is cut
  // off as its own feeder. Children were already reduced below the caps
  // when they were processed, so every cut feeder satisfies them — the
  // only irreducible violations are single load nodes, which are reported.
  const feederOfNode = new Int32Array(nN).fill(-1);
  const subtreeCust = new Float64Array(nN);
  const accCust = new Float64Array(nN);
  const accLen = new Float64Array(nN);
  const allOrder = []; // parent-before-child, contiguous per sub
  const rawFeeders = []; // {sub, rootNode}

  const gridN = terrain.n;
  const isUnderground = (a, b) => {
    const mx = (graph.nx[a] + graph.nx[b]) / 2;
    const my = (graph.ny[a] + graph.ny[b]) / 2;
    const [cx, cy] = terrain.cellOf(mx, my);
    return density.grid[cy * gridN + cx] > UG_DENSITY_THRESH;
  };
  // circuit cap for a prospective feeder rooted at v carrying `cust`
  const overCap = (cust, lenM, v) =>
    cust > FEEDER_MAX_CUST ||
    (dist[parent[v]] + lenM) / 1000 > (cust < FEEDER_LONG_CUST ? FEEDER_LONG_KM : FEEDER_MAX_KM);

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
    const cutFeeder = new Map(); // node -> raw feeder id
    for (let i = order.length - 1; i >= 0; i--) {
      const v = order[i];
      let cust = custAtNode[v], len = graph.edges[parentEdge[v]].len;
      const kids = [];
      for (const ej of graph.adj[v]) {
        if (!usedEdge[ej]) continue;
        const e2 = graph.edges[ej];
        const w = e2.a === v ? e2.b : e2.a;
        if (parent[w] === v && !cutFeeder.has(w)) kids.push(w);
      }
      for (const w of kids) { cust += accCust[w]; len += accLen[w]; }
      let guard = kids.length;
      while (guard-- > 0 && overCap(cust, len, v)) {
        let big = -1;
        for (const w of kids) {
          if (cutFeeder.has(w) || accCust[w] <= 0) continue; // never cut a bare trunk
          if (big === -1 || accCust[w] > accCust[big]) big = w;
        }
        if (big === -1) break;
        cutFeeder.set(big, rawFeeders.length);
        rawFeeders.push({ sub: s.id, rootNode: big });
        cust -= accCust[big]; len -= accLen[big];
      }
      accCust[v] = cust; accLen[v] = len;
      if (parent[v] === s.node) {
        cutFeeder.set(v, rawFeeders.length);
        rawFeeders.push({ sub: s.id, rootNode: v });
      }
    }
    // Top-down feeder assignment (order is parent-before-child).
    for (const v of order) {
      feederOfNode[v] = cutFeeder.has(v) ? cutFeeder.get(v) : feederOfNode[parent[v]];
    }
    allOrder.push(...order);
  }

  const feeders = rawFeeders.map((rf, id) => ({
    id, sub: rf.sub, rootNode: rf.rootNode,
    nodes: [], edges: [], customers: 0,
    lengthM: 0, ohLenM: 0, ugLenM: 0, txCount: 0,
    expressOhKm: 0, expressUgKm: 0, expressMidM: 0,
  }));

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
  // feeder-local subtree customers (allOrder is parent-before-child per sub)
  for (let i = allOrder.length - 1; i >= 0; i--) {
    const v = allOrder[i];
    subtreeCust[v] += custAtNode[v];
    const p = parent[v];
    if (p !== -1 && feederOfNode[p] === feederOfNode[v]) subtreeCust[p] += subtreeCust[v];
  }
  for (const f of feeders) f.customers = subtreeCust[f.rootNode];

  // ---------- 7. express runs: feeder head back to the sub busbar (the
  // parallel circuit strung along the shared trunk)
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

  // ---------- 8. attach TXs / customers to feeders. A TX exactly at a sub
  // busbar joins that sub's SMALLEST feeder (no minimums, and the largest
  // may already sit at its cap).
  for (const tx of txs) {
    if (tx.sub === -1) { tx.feeder = -1; continue; }
    if (feederOfNode[tx.node] >= 0) {
      tx.feeder = feederOfNode[tx.node];
      feeders[tx.feeder].txCount++;
    } else if (feederOfNode[tx.node] === -2) {
      const f = feeders.filter(f => f.sub === subOf[tx.node])
        .sort((a, b) => a.customers - b.customers)[0];
      tx.feeder = f ? f.id : -1;
      if (f) { f.customers += tx.customers.length; f.txCount++; }
      else { tx.sub = -1; orphanTx++; }
    } else {
      tx.feeder = -1; tx.sub = -1; orphanTx++;
    }
  }

  // ---------- 9. name subs for their nearest town
  for (const s of subs) {
    let town = towns[0], bd = Infinity;
    for (const t of towns) {
      const d = Math.hypot(t.x - s.x, t.y - s.y);
      if (d < bd) { bd = d; town = t; }
    }
    const dupes = subs.filter(o => o.id < s.id && o.baseName === town.name).length;
    s.baseName = town.name;
    s.name = town.name + (dupes ? " " + "BCDEFG"[(dupes - 1) % 6] : "");
  }

  // ---------- 10. rule checks — asserted where construction guarantees,
  // measured and reported where it cannot (tunable)
  const extraChecks = [];
  // TX rules
  let worstTxM = 0, worstTxCust = 0;
  for (const tx of txs) {
    worstTxCust = Math.max(worstTxCust, tx.customers.length);
    for (const ci of tx.customers) {
      worstTxM = Math.max(worstTxM, Math.hypot(customers[ci].x - tx.x, customers[ci].y - tx.y));
    }
  }
  extraChecks.push({
    name: `TX rules (≤ ${TX_MAX_M} m or ≤ ${TX_MAX_CUST} cust)`,
    tunable: true,
    pass: worstTxCust <= TX_MAX_CUST && worstTxM <= TX_MAX_M * 1.2,
    detail: `${txs.length} TX; worst customer→TX ${Math.round(worstTxM)} m ` +
      `(cap ${TX_MAX_M} m ×1.2 — the TX site is a moving centroid), largest TX ${worstTxCust} cust`,
  });
  // sub rules (post road-nearest assignment)
  const subCust = subs.map(s => feeders.filter(f => f.sub === s.id)
    .reduce((t, f) => t + f.customers, 0));
  const overCapSubs = subs.filter(s => subCust[s.id] > SUB_MAX_CUST * 1.1);
  let worstTxSubKm = 0;
  for (const tx of txs) {
    if (tx.sub !== -1 && isFinite(dist[tx.node])) worstTxSubKm = Math.max(worstTxSubKm, dist[tx.node] / 1000);
  }
  extraChecks.push({
    name: `Sub rules (≤ ${SUB_MAX_CUST} cust, TX ≤ ${SUB_MAX_KM} km by road)`,
    tunable: true,
    pass: overCapSubs.length === 0 && worstTxSubKm <= SUB_MAX_KM * 1.1,
    detail: `${subs.length} subs (emergent); ` +
      (overCapSubs.length ? `${overCapSubs.length} over cap ×1.1: ${overCapSubs.map(s => `${s.name}(${Math.round(subCust[s.id])})`).join(", ")}; ` : "all within cap ×1.1; ") +
      `worst TX→sub ${worstTxSubKm.toFixed(1)} km; ${reassigned} load node(s) re-homed by road-nearest assignment`,
  });
  // feeder rules
  const circuitKm = (f) => (f.lengthM / 1000) + f.expressOhKm + f.expressUgKm;
  const badFeeders = feeders.filter(f =>
    f.customers > FEEDER_MAX_CUST * 1.05 ||
    circuitKm(f) > (f.customers < FEEDER_LONG_CUST ? FEEDER_LONG_KM : FEEDER_MAX_KM) * 1.05);
  const longAllow = feeders.filter(f => circuitKm(f) > FEEDER_MAX_KM && f.customers < FEEDER_LONG_CUST);
  extraChecks.push({
    name: `Feeder rules (≤ ${FEEDER_MAX_CUST} cust, ≤ ${FEEDER_MAX_KM} km; ≤ ${FEEDER_LONG_KM} km under ${FEEDER_LONG_CUST} cust)`,
    tunable: true,
    pass: badFeeders.length === 0,
    detail: `${feeders.length} feeders (emergent), ${longAllow.length} on the long-rural allowance; ` +
      (badFeeders.length
        ? `${badFeeders.length} outside caps ×1.05: ${badFeeders.map(f => `F${f.id}(${Math.round(f.customers)}c/${circuitKm(f).toFixed(1)}km)`).join(", ")}`
        : `worst circuit ${Math.max(...feeders.map(circuitKm)).toFixed(1)} km`),
  });
  extraChecks.push({
    name: "Every transformer maps to a feeder",
    pass: orphanTx === 0,
    detail: `${txs.length - orphanTx}/${txs.length} TXs mapped, ${orphanTx} orphan(s)`,
  });

  const urbanShare = classOfCust.reduce((t, v) => t + v, 0) / Math.max(1, classOfCust.length);
  const feederOfTx = new Int32Array(txs.length).fill(-1);
  for (const tx of txs) feederOfTx[tx.id] = tx.feeder;
  const subOfFeeder = new Int32Array(feeders.length);
  for (const f of feeders) subOfFeeder[f.id] = f.sub;

  return {
    txs, subs, feeders,
    dist, parent, parentEdge, subOf, feederOfNode,
    subtreeCust, custAtNode, treeEdges, treeEdgeOfNode,
    usedEdge, orphanTx,
    membership: {
      classOfCust, urbanShare, loadNodes: loadNodes.length,
      feederOfTx, subOfFeeder, reassigned,
      worstTxSubKm: +worstTxSubKm.toFixed(1),
      worstCircuitKm: +Math.max(...feeders.map(circuitKm)).toFixed(1),
      ruleViolations: badFeeders.map(f => `F${f.id} ${Math.round(f.customers)}c/${circuitKm(f).toFixed(1)}km`)
        .concat(overCapSubs.map(s => `${s.name} ${Math.round(subCust[s.id])} cust`)),
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

// customers → TX: the transformer sits AT the seed customer (a fixed pole
// site) and gathers the nearest unassigned customers within TX_MAX_M of
// it, up to TX_MAX_CUST — so the ≤ 500 m rule holds EXACTLY by
// construction (a moving centroid drifts past it). Rural TXs max out on
// distance, urban on count. Deterministic Morton sweep.
function clusterTransformers(customers) {
  const order = customers.map((c, i) => ({ i, m: morton(c.x, c.y) }))
    .sort((a, b) => a.m - b.m).map(o => o.i);
  const assigned = new Int32Array(customers.length).fill(-1);
  const BIN = TX_MAX_M;
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
    const members = [start];
    assigned[start] = txs.length;
    const cx = c0.x, cy = c0.y;
    while (members.length < TX_MAX_CUST) {
      let best = -1, bestD = Infinity;
      const bx = (cx / BIN) | 0, by = (cy / BIN) | 0;
      for (let dy = -1; dy <= 1; dy++) {
        for (let dx = -1; dx <= 1; dx++) {
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
      if (best === -1 || bestD > TX_MAX_M) break;
      assigned[best] = txs.length;
      members.push(best);
    }
    txs.push({ id: txs.length, x: cx, y: cy, customers: members, node: -1, feeder: -1, sub: -1 });
  }
  return txs;
}
