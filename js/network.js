// network.js — electrical model: distribution transformers, zone
// substations, and radial feeders routed ALONG the road graph.
//
// RULE-CAP FLOW (caps in membership.js; counts EMERGE from the caps):
//   1. customers → TX: ≤ 500 m to the transformer OR ≤ 100 customers
//   2. TX → zone subs: greedy road-Dijkstra growth, ≤ SUB_MAX_KM /
//      ≤ SUB_MAX_CUST; each sub sited at its cluster's LOAD-WEIGHTED
//      CENTROID, nudged to a subtransmission-viable corridor node
//   3. every TX then assigned to its road-NEAREST sited sub (one
//      multi-source Dijkstra). Graph-Voronoi property: shortest paths
//      never leave their cell, so sub trees are DISJOINT by construction
//   4. THE LADDER — every busbar branch is cut into connected clusters of
//      ≤ FEEDER_MAX_CUST customers / ≤ FEEDER_MAX_KM conductor, then each
//      cluster resolves to exactly one rung:
//        a. SIBLING feeder — its own breaker at the busbar, exit lead
//           running in parallel along the shared corridor (lead ≤
//           LEAD_MAX_M). Urban multi-feeder subs come from this rung.
//        b. SPAWN — a stranded cluster (lead > LEAD_MAX_M) of at least
//           SUB_MIN_CUST customers earns a new zone sub, sited AT the
//           cluster's load-weighted median node (ON the corridor, so the
//           re-bid is guaranteed to hand it its own neighbourhood).
//           Budgeted by SUB_MAX_COUNT.
//        c. EXPRESS feeder — a stranded cluster too small for a station
//           (20–200 customers) keeps its long unloaded lead, labelled as
//           the sanctioned exception. A sub-worthy cluster left stranded
//           by an exhausted budget becomes a LOUD express.
//        d. CLIP — below FEEDER_MIN_CUST (the splitter folds rather than
//           cuts dust, so this is rare); the prune loop in main.js
//           removes it with its customers.
//   5. PARSIMONY — stations under 2 × SUB_MIN_CUST are trial-dissolved:
//      remove, re-bid, re-split, and COMMIT only if every receiving
//      station stays within MERGE_HEADROOM of its caps and the express /
//      clip / stranded counts do not grow; otherwise roll back. This
//      replaces the old distance-based consolidation, whose unverified
//      merges were the super-feeder factory.
//
// ASSUMPTIONS:
//  - Sections through high-density cells are UNDERGROUND cable, the rest
//    overhead (per-type fault rates live in reliability.js).
//  - SIBLING circuits share road corridors on shared structures but fault
//    INDEPENDENTLY in the model — common-mode events (car vs pole line)
//    are not modelled. Labelled in the Assumptions panel.
//  - A feeder's unloaded exit lead is modelled as one virtual first
//    section (length = road distance busbar → cluster root): faults on it
//    interrupt the whole feeder; devices on it are worthless, so the
//    greedy ignores it naturally.
//  - LV detail below the TX is ignored; customers hang off their TX.
//  - LINE EASEMENTS: a transformer more than OFFROAD_SNAP_M from any road
//    gets a straight cross-country span (to the nearest road node, or a
//    nearer easement node — spans daisy-chain up remote valleys). Rural
//    feeders may therefore leave the road corridor. Easements never span
//    the sea or lakes; river spans are allowed (towers, not bridges).
//    Crew travel is charged along the LINE route — an approximation,
//    labelled in the Assumptions panel.

import {
  classifyCustomers, buildLoadNodes, growSubs, NodeHeap, morton,
  TX_MAX_CUST, TX_MAX_M, SUB_MAX_CUST, SUB_MIN_CUST, SUB_MAX_COUNT,
  SUB_MAX_KM, FEEDER_MAX_CUST, FEEDER_MAX_KM, FEEDERS_MAX_PER_SUB,
  FEEDER_MIN_CUST, LEAD_MAX_M, MERGE_HEADROOM,
} from "./membership.js";
import { CLS_EASEMENT } from "./roads.js";
import { MAP_MAX } from "./terrain.js";

// TX farther than this from a road connects by cross-country line easement.
export const OFFROAD_SNAP_M = 400;

// Town peaks sit near 1.0 (mass-compensated), so this keeps cable to the
// inner core (roughly r < σ) rather than whole towns.
export const UG_DENSITY_THRESH = 0.4; // density units — inner-urban ⇒ cable

export function buildNetwork(terrain, graph, customers, towns, density, rng) {
  // ---------- 1. customers → TX (≤ TX_MAX_M or ≤ TX_MAX_CUST)
  const txs = clusterTransformers(customers);
  // TX → graph: nearby TXs snap to the road; remote TXs get a LINE
  // EASEMENT — a straight cross-country span to the nearest road node or
  // a nearer easement node (so spans daisy-chain up a remote valley).
  // Never across the sea or a lake; a river span is fine (towers).
  const easeNodes = [];
  for (const tx of txs) {
    const near = graph.nearestNode(tx.x, tx.y, MAP_MAX);
    if (near.id === -1) { tx.node = -1; continue; }
    if (near.dist <= OFFROAD_SNAP_M) { tx.node = near.id; continue; }
    let aid = near.id, ad = near.dist;
    for (const id of easeNodes) {
      const d = Math.hypot(graph.nx[id] - tx.x, graph.ny[id] - tx.y);
      if (d < ad) { ad = d; aid = id; }
    }
    if (terrain.segmentHits(tx.x, tx.y, graph.nx[aid], graph.ny[aid],
        (i) => terrain.water[i] === 1)) { tx.node = near.id; continue; }
    const nid = graph.node(tx.x, tx.y);
    graph.addEdge(nid, aid, CLS_EASEMENT);
    easeNodes.push(nid);
    tx.node = nid;
  }
  for (const tx of txs) for (const ci of tx.customers) customers[ci].tx = tx.id;
  const classOfCust = classifyCustomers(customers);
  const loadNodes = buildLoadNodes(txs, customers, classOfCust);
  const loadAt = new Map(loadNodes.map((ln, i) => [ln.node, i]));
  const custAt = new Map(loadNodes.map(ln => [ln.node, ln.cust]));

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
    const node = nudgeToCorridor(terrain, graph, cx, cy, subs, loadAt);
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
  const bid = () => {
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
  };

  // -------- feeder splitter: cut each busbar branch's subtree into
  // connected clusters respecting the feeder caps (customers AND
  // conductor). Deterministic. The splitter FOLDS rather than cuts
  // dust — a child subtree under FEEDER_MIN_CUST is never cut off on its
  // own — so sub-minimum clusters only arise irreducibly.
  // Returns { clusters, nodeCluster, perSubCount, residualOver } where a
  // cluster is { sid, root, base, cust, condM, leadM, loads }.
  const splitFeeders = (lambdaArr) => {
    const nodeBranch = new Map(); // node → branch object
    const branches = [];
    for (const ln of loadNodes) {
      const sid = subOf[ln.node];
      if (sid === -1) continue;
      const subNode = subs[sid].node;
      if (ln.node === subNode) continue; // busbar load — attached at step 9
      const trail = [];
      let v = ln.node, br = null;
      while (v !== subNode) {
        const known = nodeBranch.get(v);
        if (known) { br = known; break; }
        trail.push(v);
        const p = parent[v];
        if (p === -1) break;
        if (p === subNode) { br = { sid, root: v, nodes: [] }; branches.push(br); break; }
        v = p;
      }
      if (!br) continue; // disconnected load — reported as orphan later
      for (const t of trail) { nodeBranch.set(t, br); br.nodes.push(t); }
    }
    const clustersOut = [];
    const nodeCluster = new Map();
    const perSubCount = new Map();
    const residualOver = [];
    for (const br of branches) {
      const kids = new Map();
      for (const v of br.nodes) {
        if (v === br.root) continue;
        const p = parent[v];
        let arr = kids.get(p);
        if (!arr) kids.set(p, arr = []);
        arr.push(v);
      }
      // children strictly farther than parents along the tree, so a
      // dist-descending sweep processes children first
      const order = br.nodes.slice().sort((a, b) => dist[b] - dist[a] || a - b);
      const subC = new Map(), subL = new Map(), cut = new Set();
      for (const v of order) {
        let c = custAt.get(v) ?? 0;
        let L = graph.edges[parentEdge[v]].len;
        const ks = kids.get(v);
        if (ks) {
          ks.sort((a, b) => (subC.get(b) - subC.get(a)) || a - b);
          for (const k of ks) { c += subC.get(k); L += subL.get(k); }
          if (c > FEEDER_MAX_CUST || L > FEEDER_MAX_KM * 1000) {
            for (const k of ks) { // largest first
              if (c <= FEEDER_MAX_CUST && L <= FEEDER_MAX_KM * 1000) break;
              if (subC.get(k) < FEEDER_MIN_CUST) continue; // fold, don't cut dust
              cut.add(k); c -= subC.get(k); L -= subL.get(k);
            }
          }
        }
        if (c > FEEDER_MAX_CUST) residualOver.push({ sid: br.sid, node: v, cust: c });
        subC.set(v, c); subL.set(v, L);
      }
      const roots = [br.root, ...[...cut].sort((a, b) => dist[a] - dist[b] || a - b)];
      for (const r of roots) {
        const base = r === br.root;
        const cl = {
          sid: br.sid, root: r, base,
          cust: subC.get(r),
          // a cut root's parent edge becomes the open boundary span, not
          // its conductor; the base cluster's first section is real line
          condM: subL.get(r) - (base ? 0 : graph.edges[parentEdge[r]].len),
          leadM: base ? 0 : dist[r] - lambdaArr[br.sid],
          loads: [],
        };
        const ci = clustersOut.length;
        clustersOut.push(cl);
        perSubCount.set(br.sid, (perSubCount.get(br.sid) ?? 0) + 1);
        const stack = [r];
        while (stack.length) {
          const v = stack.pop();
          nodeCluster.set(v, ci);
          const c = custAt.get(v);
          if (c) cl.loads.push({ node: v, d: dist[v], cust: c });
          const ks = kids.get(v);
          if (ks) for (const k of ks) if (!cut.has(k)) stack.push(k);
        }
      }
    }
    return { clusters: clustersOut, nodeCluster, perSubCount, residualOver };
  };
  // stranded = lead too long for a sibling circuit
  const stranded = (c) => !c.base && c.leadM > LEAD_MAX_M;
  const subWorthy = (c) => stranded(c) && c.cust >= SUB_MIN_CUST;
  // cluster's load-weighted MEDIAN load node — always ON the corridor, so
  // a sub spawned there is guaranteed its own neighbourhood in the re-bid
  const medianLoadNode = (cl) => {
    const loads = cl.loads.slice().sort((a, b) => a.d - b.d || a.node - b.node);
    let acc = 0;
    for (const l of loads) {
      acc += l.cust;
      if (acc * 2 >= cl.cust && !subs.some(s => s.node === l.node)) return l.node;
    }
    for (const l of loads) if (!subs.some(s => s.node === l.node)) return l.node;
    return -1;
  };
  const spawnAt = (node, catchment) => {
    subs.push({
      id: subs.length, node, x: graph.nx[node], y: graph.ny[node],
      centroidX: graph.nx[node], centroidY: graph.ny[node],
      catchment: Math.round(catchment),
    });
  };

  // ---------- 4. THE LADDER's spawn loop: bid, split, then give every
  // sub-worthy stranded cluster its own station ON its corridor. Repeats
  // until nothing sub-worthy is stranded, the budget is spent, or the
  // round cap trips (leftovers become LOUD express feeders, reported).
  let ladderRounds = 0;
  for (let outer = 0; outer < 14; outer++) {
    bid();
    ladderRounds = outer + 1;
    const split = splitFeeders(lambda);
    const spawnable = split.clusters.filter(subWorthy);
    // a whole CELL stuck over the sub cap (pigeonhole no boundary shift
    // can fix) also spawns, at the far half's median load node
    const stubborn = subs.filter(s => lastCust[s.id] > SUB_MAX_CUST * 1.1);
    if ((!spawnable.length && !stubborn.length) ||
        subs.length >= SUB_MAX_COUNT || outer === 13) break;
    spawnable.sort((a, b) => b.cust - a.cust || a.root - b.root);
    for (const cl of spawnable) {
      if (subs.length >= SUB_MAX_COUNT) break;
      const node = medianLoadNode(cl);
      if (node !== -1) spawnAt(node, cl.cust);
    }
    for (const s of stubborn) {
      if (subs.length >= SUB_MAX_COUNT) break;
      const members = [];
      for (const ln of loadNodes) {
        if (subOf[ln.node] === s.id) members.push({ node: ln.node, d: dist[ln.node], cust: ln.cust });
      }
      members.sort((a, b) => a.d - b.d || a.node - b.node);
      let acc = 0;
      const half = lastCust[s.id] / 2, farLoads = [];
      let farCust = 0;
      for (const m of members) {
        acc += m.cust;
        if (acc <= half) continue;
        farLoads.push(m); farCust += m.cust;
      }
      const fake = { cust: farCust, loads: farLoads };
      const node = medianLoadNode(fake);
      if (node !== -1) spawnAt(node, farCust);
    }
    lambda = new Float64Array(subs.length);
  }

  // ---------- 5. PARSIMONY: stations under 2 × SUB_MIN_CUST are
  // trial-dissolved smallest-first — remove, re-bid, re-split, verify,
  // commit or roll back. Receivers may only fill to MERGE_HEADROOM of
  // their caps (spawn fires at >100%: the gap is the anti-oscillation
  // hysteresis), and the stranded/express/clip counts must not grow — a
  // merge is never allowed to manufacture the exceptions the ladder just
  // resolved. Stations under SUB_MIN_CUST are MANDATORY candidates; one
  // that cannot be lawfully dissolved is kept and reported.
  const parsimony = { dissolved: 0, kept: 0, trials: 0 };
  {
    const strandedCount = (split) =>
      split.clusters.filter(c => stranded(c) && c.cust >= FEEDER_MIN_CUST).length;
    const tinyCount = (split) =>
      split.clusters.filter(c => c.cust < FEEDER_MIN_CUST).length;
    let split = splitFeeders(lambda);
    let before = {
      stranded: strandedCount(split), tiny: tinyCount(split),
      totalByNode: new Map(subs.map(s => [s.node, lastCust[s.id]])),
      countByNode: new Map(subs.map(s => [s.node, split.perSubCount.get(s.id) ?? 0])),
    };
    const candidates = subs
      .filter(s => lastCust[s.id] < SUB_MIN_CUST * 2)
      .sort((a, b) => lastCust[a.id] - lastCust[b.id] || a.node - b.node)
      .map(s => s.node); // identify by node — ids shift as subs are removed
    for (const candNode of candidates) {
      if (parsimony.trials >= 12) break;
      const idx = subs.findIndex(s => s.node === candNode);
      if (idx === -1) continue;
      const candTotal = before.totalByNode.get(candNode) ?? 0;
      if (candTotal >= SUB_MIN_CUST * 2) continue;
      parsimony.trials++;
      const save = {
        subs, lambda: lambda.slice(), dist, parent, parentEdge, subOf,
        lastCust, split, before,
      };
      subs = subs.filter((s, i) => i !== idx).map((s, i) => ({ ...s, id: i }));
      lambda = new Float64Array(subs.length);
      bid();
      const trialSplit = splitFeeders(lambda);
      const totalOk = subs.every(s => {
        const now = lastCust[s.id];
        const was = before.totalByNode.get(s.node) ?? 0;
        return now <= Math.max(was, MERGE_HEADROOM * SUB_MAX_CUST);
      });
      const countOk = subs.every(s => {
        const now = trialSplit.perSubCount.get(s.id) ?? 0;
        const was = before.countByNode.get(s.node) ?? 0;
        return now <= Math.max(was, FEEDERS_MAX_PER_SUB);
      });
      const exceptOk = strandedCount(trialSplit) <= before.stranded &&
        tinyCount(trialSplit) <= before.tiny;
      if (totalOk && countOk && exceptOk) {
        parsimony.dissolved++;
        split = trialSplit;
        before = {
          stranded: strandedCount(trialSplit), tiny: tinyCount(trialSplit),
          totalByNode: new Map(subs.map(s => [s.node, lastCust[s.id]])),
          countByNode: new Map(subs.map(s => [s.node, trialSplit.perSubCount.get(s.id) ?? 0])),
        };
      } else {
        ({ subs, dist, parent, parentEdge, subOf, lastCust, split, before } = save);
        lambda = save.lambda;
        if (candTotal < SUB_MIN_CUST) parsimony.kept++;
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
  // final cluster picture on the recovered distances
  const zeroLambda = new Float64Array(subs.length);
  const finalSplit = splitFeeders(zeroLambda);

  // ---------- 6. prune to the union of TX→sub paths
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

  // ---------- 7. feeders = one per CLUSTER. Base clusters root at the
  // busbar as before; cut clusters get their own breaker position and an
  // unloaded exit LEAD along the shared corridor, modelled as a virtual
  // first section. The boundary span between a cluster and its parent
  // cluster carries no circuit — it is released to become a normally-open
  // SIBLING TIE, which the tie search below discovers on its own.
  const feederOfNode = new Int32Array(nN).fill(-1);
  const subtreeCust = new Float64Array(nN);
  const allOrder = []; // parent-before-child, contiguous per sub

  const gridN = terrain.nx; // linear cell index stride
  const isUnderground = (a, b) => {
    const mx = (graph.nx[a] + graph.nx[b]) / 2;
    const my = (graph.ny[a] + graph.ny[b]) / 2;
    const [cx, cy] = terrain.cellOf(mx, my);
    return density.grid[cy * gridN + cx] > UG_DENSITY_THRESH;
  };

  // deterministic feeder ids: clusters grouped by sub, base first, then by
  // lead length
  const clusterOrder = finalSplit.clusters
    .map((c, i) => ({ c, i }))
    .sort((p, q) => p.c.sid - q.c.sid || (q.c.base ? 1 : 0) - (p.c.base ? 1 : 0) ||
      p.c.leadM - q.c.leadM || p.c.root - q.c.root);
  const feederIdOfCluster = new Int32Array(finalSplit.clusters.length).fill(-1);
  const feeders = clusterOrder.map(({ c, i }, fid) => {
    feederIdOfCluster[i] = fid;
    return {
      id: fid, sub: c.sid, rootNode: c.root,
      express: stranded(c), loud: subWorthy(c),
      nodes: [], edges: [], customers: 0,
      lengthM: 0, ohLenM: 0, ugLenM: 0, leadM: 0, txCount: 0,
    };
  });
  for (const [node, ci] of finalSplit.nodeCluster) {
    feederOfNode[node] = feederIdOfCluster[ci];
  }
  const leadFeederAt = new Map(); // cluster root node → feeder id, cut clusters only
  for (let i = 0; i < finalSplit.clusters.length; i++) {
    const c = finalSplit.clusters[i];
    if (!c.base) leadFeederAt.set(c.root, feederIdOfCluster[i]);
  }

  for (const s of subs) {
    feederOfNode[s.node] = -2; // sub busbar: not on any single feeder
    const stack = [];
    for (const ei of graph.adj[s.node]) {
      if (!usedEdge[ei]) continue;
      const e = graph.edges[ei];
      const child = e.a === s.node ? e.b : e.a;
      if (parent[child] === s.node) stack.push(child);
    }
    stack.sort((a, b) => b - a);
    while (stack.length) {
      const v = stack.pop();
      allOrder.push(v);
      for (const ej of graph.adj[v]) {
        if (!usedEdge[ej]) continue;
        const e2 = graph.edges[ej];
        const w = e2.a === v ? e2.b : e2.a;
        if (parent[w] === v) stack.push(w);
      }
    }
  }

  // ---------- 8. tree edges: one per used node. A cut cluster's root gets
  // its LEAD as the virtual section (path recorded for drawing); the
  // physical boundary span is released from the tree so the tie search
  // finds it as a sibling tie.
  const treeEdges = [];
  const treeEdgeOfNode = new Int32Array(nN).fill(-1);
  for (const v of allOrder) {
    const fid = feederOfNode[v];
    if (fid < 0) continue;
    const f = feeders[fid];
    if (leadFeederAt.get(v) === fid) {
      // virtual lead section: busbar → cluster root along the corridor
      const path = [v];
      let ohM = 0, ugM = 0;
      const subNode = subs[f.sub].node;
      for (let p = v; parent[p] !== -1;) {
        const q = parent[p];
        const len = graph.edges[parentEdge[p]].len;
        if (isUnderground(p, q)) ugM += len; else ohM += len;
        path.push(q);
        if (q === subNode) break;
        p = q;
      }
      usedEdge[parentEdge[v]] = 0; // boundary span → normally-open sibling tie
      const leadM = ohM + ugM;
      const te = {
        id: treeEdges.length, node: v, parentNode: subNode,
        edgeId: -1, feeder: fid, lenM: leadM, midDistM: leadM / 2,
        bridge: false, underground: ugM > ohM, lead: true, path,
      };
      treeEdgeOfNode[v] = te.id;
      treeEdges.push(te);
      f.edges.push(te.id);
      f.nodes.push(v);
      f.lengthM += leadM; f.leadM = leadM;
      if (te.underground) f.ugLenM += leadM; else f.ohLenM += leadM;
      continue;
    }
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

  // ---------- 9. attach TXs / customers to feeders. A TX exactly at a sub
  // busbar joins that sub's SMALLEST feeder (the largest may already sit
  // at its cap).
  for (const tx of txs) {
    if (tx.sub === -1) { tx.feeder = -1; continue; }
    if (feederOfNode[tx.node] >= 0) {
      tx.feeder = feederOfNode[tx.node];
      feeders[tx.feeder].txCount++;
    } else if (feederOfNode[tx.node] === -2) {
      let f = feeders.filter(f => f.sub === subOf[tx.node])
        .sort((a, b) => a.customers - b.customers || a.id - b.id)[0];
      if (!f) {
        // a sub whose entire load sits at its own busbar has no branches;
        // hang its TXs off the road-nearest feeder of any neighbour
        const near = graph.nearestNode(graph.nx[tx.node], graph.ny[tx.node],
          MAP_MAX, (id) => feederOfNode[id] >= 0);
        if (near.id !== -1) f = feeders[feederOfNode[near.id]];
      }
      tx.feeder = f ? f.id : -1;
      if (f) { tx.sub = f.sub; f.customers += tx.customers.length; f.txCount++; }
      else { tx.sub = -1; orphanTx++; }
    } else {
      tx.feeder = -1; tx.sub = -1; orphanTx++;
    }
  }

  // ---------- 9b. normally-open TIE POINTS between adjacent feeders: a
  // multi-source Dijkstra over UNUSED road/easement edges, seeded from
  // every tree node and labelled by feeder. Where two feeders' frontiers
  // meet within TIE_MAX_M of unused road, the pair gets one tie (its
  // shortest corridor) anchored at the tree nodes the corridor leaves
  // from. Released sibling boundary spans are unused edges too, so
  // sibling feeders sharing a corridor pick up their ties here for free.
  // Backfeed in reliability.js restores device-bounded subtrees that
  // reach a tie anchor from the neighbouring feeder. Normally open —
  // carries nothing day-to-day, so the SAIDI baseline is unchanged until
  // a device uses it.
  const TIE_MAX_M = 2000;
  const tieDist = new Float64Array(nN).fill(Infinity);
  const tieLab = new Int32Array(nN).fill(-1);
  const tieRoot = new Int32Array(nN).fill(-1);
  const tieHeap = new NodeHeap();
  for (let v = 0; v < nN; v++) {
    if (feederOfNode[v] >= 0) {
      tieDist[v] = 0; tieLab[v] = feederOfNode[v]; tieRoot[v] = v;
      tieHeap.push(0, v);
    }
  }
  const tieBest = new Map(); // feeder-pair key → {a, b, lenM}
  const tieSettled = new Uint8Array(nN);
  while (tieHeap.size) {
    const v = tieHeap.pop();
    if (tieSettled[v]) continue;
    tieSettled[v] = 1;
    for (const ei of graph.adj[v]) {
      if (usedEdge[ei]) continue;
      const e = graph.edges[ei];
      const w = e.a === v ? e.b : e.a;
      if (tieLab[w] !== -1 && tieLab[w] !== tieLab[v]) {
        const span = tieDist[v] + e.len + tieDist[w];
        if (span <= TIE_MAX_M) {
          const fa = tieLab[v], fb = tieLab[w];
          const key = fa < fb ? fa * 65536 + fb : fb * 65536 + fa;
          const cur = tieBest.get(key);
          if (!cur || span < cur.lenM) {
            tieBest.set(key, {
              a: tieRoot[v], b: tieRoot[w], feederA: fa, feederB: fb,
              lenM: span,
            });
          }
        }
      }
      const nd = tieDist[v] + e.len;
      if (nd < tieDist[w] && nd <= TIE_MAX_M) {
        tieDist[w] = nd; tieLab[w] = tieLab[v]; tieRoot[w] = tieRoot[v];
        tieHeap.push(nd, w);
      }
    }
  }
  const ties = [...tieBest.values()];
  const tieNodesByFeeder = feeders.map(() => new Set());
  for (const t of ties) {
    tieNodesByFeeder[t.feederA].add(t.a);
    tieNodesByFeeder[t.feederB].add(t.b);
  }

  // ---------- 10. name subs for their nearest town
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

  // ---------- 11. rule checks — asserted where construction guarantees,
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
  const underMinSubs = subs.filter(s => subCust[s.id] < SUB_MIN_CUST);
  let worstTxSubKm = 0;
  for (const tx of txs) {
    if (tx.sub !== -1 && isFinite(dist[tx.node])) worstTxSubKm = Math.max(worstTxSubKm, dist[tx.node] / 1000);
  }
  extraChecks.push({
    name: `Sub rules (${SUB_MIN_CUST}–${SUB_MAX_CUST} cust, ≤ ${SUB_MAX_COUNT} stations, TX ≤ ${SUB_MAX_KM} km by road)`,
    tunable: true,
    pass: overCapSubs.length === 0 && underMinSubs.length === 0 &&
      subs.length <= SUB_MAX_COUNT && worstTxSubKm <= SUB_MAX_KM * 1.1,
    detail: `${subs.length}/${SUB_MAX_COUNT} subs (emergent); ` +
      (overCapSubs.length ? `${overCapSubs.length} over cap ×1.1: ${overCapSubs.map(s => `${s.name}(${Math.round(subCust[s.id])})`).join(", ")}; ` : "") +
      (underMinSubs.length ? `${underMinSubs.length} under min ${SUB_MIN_CUST} (parsimony could not lawfully dissolve): ${underMinSubs.map(s => `${s.name}(${Math.round(subCust[s.id])})`).join(", ")}; ` : "") +
      (!overCapSubs.length && !underMinSubs.length ? "all within band; " : "") +
      `worst TX→sub ${worstTxSubKm.toFixed(1)} km; parsimony dissolved ${parsimony.dissolved} of ${parsimony.trials} trial(s); ` +
      `${reassigned} load node(s) re-homed by road-nearest assignment`,
  });
  // feeder rules — customers and CONDUCTOR capped flat; sibling leads
  // bounded; breaker positions per station bounded
  const badFeeders = feeders.filter(f =>
    f.customers > FEEDER_MAX_CUST * 1.05 || f.lengthM > FEEDER_MAX_KM * 1000 * 1.05);
  const posOver = subs.filter(s => feeders.filter(f => f.sub === s.id).length > FEEDERS_MAX_PER_SUB);
  const maxPos = Math.max(0, ...subs.map(s => feeders.filter(f => f.sub === s.id).length));
  extraChecks.push({
    name: `Feeder rules (≤ ${FEEDER_MAX_CUST} cust, ≤ ${FEEDER_MAX_KM} km conductor, ≤ ${FEEDERS_MAX_PER_SUB} per sub)`,
    tunable: true,
    pass: badFeeders.length === 0 && posOver.length === 0,
    detail: `${feeders.length} feeders (emergent; one per cluster, siblings share exit corridors); ` +
      `busiest sub holds ${maxPos}; ` +
      (badFeeders.length
        ? `${badFeeders.length} outside caps ×1.05: ${badFeeders.map(f => `F${f.id}(${Math.round(f.customers)}c/${(f.lengthM / 1000).toFixed(0)}km)`).join(", ")}`
        : `worst conductor ${(Math.max(...feeders.map(f => f.lengthM)) / 1000).toFixed(1)} km`) +
      (posOver.length ? `; ${posOver.length} sub(s) over ${FEEDERS_MAX_PER_SUB} positions: ${posOver.map(s => s.name).join(", ")}` : ""),
  });
  // express feeders — the sanctioned exception, counted and reported.
  // LOUD expresses (sub-worthy blocks stranded by an exhausted budget or
  // round cap) should be zero on healthy seeds.
  const expressF = feeders.filter(f => f.express);
  const loudF = expressF.filter(f => f.loud);
  extraChecks.push({
    name: `Express feeders (exception, not rule: ${FEEDER_MIN_CUST}–${SUB_MIN_CUST} cust stranded beyond ${LEAD_MAX_M / 1000} km lead)`,
    tunable: true,
    pass: loudF.length === 0 && expressF.length <= 10,
    detail: `${expressF.length} express feeder(s)` +
      (expressF.length ? ` [${expressF.map(f => `F${f.id}(${Math.round(f.customers)}c, ${(f.leadM / 1000).toFixed(1)} km lead)`).join(", ")}]` : "") +
      `; ${loudF.length} LOUD (sub-worthy but station budget/rounds exhausted)` +
      `; ladder ran ${ladderRounds} round(s)`,
  });
  extraChecks.push({
    name: "Every transformer maps to a feeder",
    pass: orphanTx === 0,
    detail: `${txs.length - orphanTx}/${txs.length} TXs mapped, ${orphanTx} orphan(s)`,
  });
  // backfeed ties (informational — a feeder with no neighbour nearby
  // legitimately has none)
  const tiedFeeders = tieNodesByFeeder.filter(s => s.size > 0).length;
  extraChecks.push({
    name: "Backfeed ties (normally open)",
    tunable: true,
    pass: ties.length > 0,
    detail: `${ties.length} tie(s) over ${feeders.length} feeders; ` +
      `${tiedFeeders} feeder(s) (${Math.round(100 * tiedFeeders / Math.max(1, feeders.length))}%) ` +
      `have at least one; mean span ${Math.round(ties.reduce((s, t) => s + t.lenM, 0) / Math.max(1, ties.length))} m`,
  });

  const urbanShare = classOfCust.reduce((t, v) => t + v, 0) / Math.max(1, classOfCust.length);
  const feederOfTx = new Int32Array(txs.length).fill(-1);
  for (const tx of txs) feederOfTx[tx.id] = tx.feeder;
  const subOfFeeder = new Int32Array(feeders.length);
  for (const f of feeders) subOfFeeder[f.id] = f.sub;

  return {
    txs, subs, feeders, ties, tieNodesByFeeder,
    dist, parent, parentEdge, subOf, feederOfNode,
    subtreeCust, custAtNode, treeEdges, treeEdgeOfNode,
    usedEdge, orphanTx,
    membership: {
      classOfCust, urbanShare, loadNodes: loadNodes.length,
      feederOfTx, subOfFeeder, reassigned,
      // spans actually carrying a feeder (rebuild rounds re-use spans laid
      // by earlier rounds, so counting freshly-laid ones would read 0)
      easementSpans: treeEdges.filter(te => te.edgeId >= 0 && graph.edges[te.edgeId].cls === CLS_EASEMENT).length,
      worstTxSubKm: +worstTxSubKm.toFixed(1),
      worstCircuitKm: +(Math.max(...feeders.map(f => f.lengthM)) / 1000).toFixed(1),
      expressFeeders: expressF.length,
      loudExpress: loudF.length,
      parsimony,
      ruleViolations: badFeeders.map(f => `F${f.id} ${Math.round(f.customers)}c/${(f.lengthM / 1000).toFixed(0)}km conductor`)
        .concat(overCapSubs.map(s => `${s.name} ${Math.round(subCust[s.id])} cust`))
        .concat(posOver.map(s => `${s.name} > ${FEEDERS_MAX_PER_SUB} positions`)),
    },
    extraChecks,
  };
}

// Nudge a load centroid to the nearest subtransmission-viable road node:
// prefer nodes on an arterial/collector corridor with gentle slope, and
// prefer nodes WITHOUT a transformer on them (in a town nearly every node
// carries load, so this is a preference, not a ban — the feederless-sub
// edge case is handled at TX attachment instead). Never a pure-easement
// node (off-road power span). Used for the INITIAL growth-cluster siting;
// ladder spawns site AT a median load node on their own corridor instead.
function nudgeToCorridor(terrain, graph, cx, cy, existingSubs, loadAt) {
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
        if (!graph.adj[id].some(ei => graph.edges[ei].cls !== CLS_EASEMENT)) continue;
        const d = Math.hypot(graph.nx[id] - cx, graph.ny[id] - cy);
        if (d > R) continue;
        const onTrunk = graph.adj[id].some(ei => graph.edges[ei].cls <= 1);
        const score = d + (onTrunk ? 0 : 1400) + (loadAt.has(id) ? 900 : 0) +
          terrain.slopeAt(graph.nx[id], graph.ny[id]) * 20000;
        if (score < bestScore) { bestScore = score; best = id; }
      }
    }
  }
  if (best !== -1) return best;
  const near = graph.nearestNode(cx, cy, MAP_MAX,
    (id) => !existingSubs.some(s => s.node === id) &&
      graph.adj[id].some(ei => graph.edges[ei].cls !== CLS_EASEMENT));
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
