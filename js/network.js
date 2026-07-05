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
  FEEDER_MIN_CUST, LEAD_MAX_M, MERGE_HEADROOM, FEEDER_TARGET_SAIDI,
  REHOME_MIN_GAIN_M, BALANCE_ABSORBER_MAX, BALANCE_MIN_DIFF,
} from "./membership.js";
import {
  DEFAULT_FAULT_RATES, LATERAL_FUSE_MAX_CUST, REPAIR_MIN, TRAVEL_KMH,
} from "./reliability.js";
import { CLS_EASEMENT } from "./roads.js";
import { MAP_MAX } from "./terrain.js";

// TX farther than this from a road connects by cross-country line easement.
export const OFFROAD_SNAP_M = 400;

// Town peaks sit near 1.0 (mass-compensated), so this keeps cable to the
// inner core (roughly r < σ) rather than whole towns.
export const UG_DENSITY_THRESH = 0.4; // density units — inner-urban ⇒ cable

// Max unused-road span for a normally-open tie corridor — shared by the
// backfeed tie search and the open-point balancing pass.
export const TIE_SPAN_M = 2000;

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

  const gridN = terrain.nx; // linear cell index stride
  const isUnderground = (a, b) => {
    const mx = (graph.nx[a] + graph.nx[b]) / 2;
    const my = (graph.ny[a] + graph.ny[b]) / 2;
    const [cx, cy] = terrain.cellOf(mx, my);
    return density.grid[cy * gridN + cx] > UG_DENSITY_THRESH;
  };

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
            // BALANCED cuts: aim each resulting feeder at ~c/k rather than
            // "cut until it just fits" — greedy fill-to-cap left every
            // split pinned at the cap beside a dust remainder (a feeder
            // with zero headroom, which no planner would sign off)
            const kParts = Math.max(Math.ceil(c / FEEDER_MAX_CUST),
              Math.ceil(L / (FEEDER_MAX_KM * 1000)));
            const targetC = c / kParts;
            for (let guard = ks.length; guard > 0; guard--) {
              const over = c > FEEDER_MAX_CUST || L > FEEDER_MAX_KM * 1000;
              if (!over && c <= targetC * 1.2) break;
              let pick = -1, best = Infinity;
              for (const k of ks) {
                if (cut.has(k)) continue;
                const sc = subC.get(k);
                if (sc < FEEDER_MIN_CUST || c - sc < FEEDER_MIN_CUST) continue;
                const miss = Math.abs((c - sc) - targetC);
                if (miss < best) { best = miss; pick = k; }
              }
              if (pick === -1) break;
              // don't overshoot far below target unless a hard cap forces it
              if (!over && c - subC.get(pick) < targetC * 0.8) break;
              cut.add(pick); c -= subC.get(pick); L -= subL.get(pick);
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
    // the two stranded bands are guarded SEPARATELY — a dissolve must not
    // trade a small express away for a sub-worthy stranded block (a LOUD
    // express); conserving the total count once allowed exactly that swap
    const worthyCount = (split) => split.clusters.filter(subWorthy).length;
    const exprCount = (split) => split.clusters.filter(c =>
      stranded(c) && c.cust >= FEEDER_MIN_CUST && c.cust < SUB_MIN_CUST).length;
    const tinyCount = (split) =>
      split.clusters.filter(c => c.cust < FEEDER_MIN_CUST).length;
    let split = splitFeeders(lambda);
    let before = {
      worthy: worthyCount(split), expr: exprCount(split), tiny: tinyCount(split),
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
      const exceptOk = worthyCount(trialSplit) <= before.worthy &&
        exprCount(trialSplit) <= before.expr &&
        tinyCount(trialSplit) <= before.tiny;
      if (totalOk && countOk && exceptOk) {
        parsimony.dissolved++;
        split = trialSplit;
        before = {
          worthy: worthyCount(trialSplit), expr: exprCount(trialSplit),
          tiny: tinyCount(trialSplit),
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

  // -------- shared cluster helpers for the repair passes below
  const nodesOfCluster = finalSplit.clusters.map(() => []);
  for (const [node, ci] of finalSplit.nodeCluster) nodesOfCluster[ci].push(node);
  for (const arr of nodesOfCluster) arr.sort((a, b) => a - b);
  // customers below each node WITHIN its cluster (children before parents)
  const belowIn = (nodesArr, root) => {
    const inCl = new Set(nodesArr);
    const orderD = nodesArr.slice().sort((a, b) => dist[b] - dist[a] || a - b);
    const below = new Map(nodesArr.map(v => [v, custAt.get(v) ?? 0]));
    for (const v of orderD) {
      const p = parent[v];
      if (v !== root && inCl.has(p)) below.set(p, below.get(p) + below.get(v));
    }
    return below;
  };
  const kidsWithin = (nodesArr, root) => {
    const inCl = new Set(nodesArr);
    const kidsIn = new Map();
    for (const v of nodesArr) {
      if (v === root) continue;
      const p = parent[v];
      if (!inCl.has(p)) continue;
      let arr = kidsIn.get(p);
      if (!arr) kidsIn.set(p, arr = []);
      arr.push(v);
    }
    return kidsIn;
  };
  const subtreeWithin = (kidsIn, w) => {
    const moved = [];
    const stack = [w];
    while (stack.length) {
      const v = stack.pop();
      moved.push(v);
      const ks = kidsIn.get(v);
      if (ks) for (const k of ks) stack.push(k);
    }
    return moved;
  };
  // lead route root → busbar: re-homed clusters carry an explicit path
  // (their parent[] chain still points at the OLD sub); everything else
  // walks the tree
  const leadWalk = (cl) => {
    if (cl.leadPath) return cl.leadPath;
    const path = [cl.root];
    const subNode = subs[cl.sid].node;
    for (let p = cl.root; parent[p] !== -1;) {
      const q = parent[p];
      path.push(q);
      if (q === subNode) break;
      p = q;
    }
    return path;
  };
  const leadStats = (cl) => {
    const path = leadWalk(cl);
    let ohM = 0, ugM = 0;
    for (let i = 1; i < path.length; i++) {
      const a = path[i - 1], b = path[i];
      const l = Math.hypot(graph.nx[a] - graph.nx[b], graph.ny[a] - graph.ny[b]);
      if (isUnderground(a, b)) ugM += l; else ohM += l;
    }
    return { ohM, ugM, path };
  };

  // ---------- 5b. OVERSHOOT RE-HOMING: a feeder that runs PAST another
  // station hands the far subtree to the nearer one — but only when the
  // receiver is meaningfully closer by road (≥ REHOME_MIN_GAIN_M), has
  // ICP and breaker-position headroom (MERGE_HEADROOM), and the new exit
  // lead is a lawful sibling lead. Overshoots past a FULL station are
  // left alone — that is real capacity interleaving, not an accident.
  // (Main source: the capacity bidding never relaxes λ once its station
  // drops back under cap, leaving displaced boundaries behind.) The old
  // boundary span is released and becomes a tie between the two feeders.
  const rehome = { moved: 0, movedCust: 0, blockedHeadroom: 0, blockedLead: 0 };
  const transferredRoots = new Set();
  {
    // unweighted nearest-station field (λ = 0), with path parents
    const nearDist = new Float64Array(nN).fill(Infinity);
    const nearSub = new Int32Array(nN).fill(-1);
    const nearParent = new Int32Array(nN).fill(-1);
    const nHeap = new NodeHeap();
    for (const s of subs) {
      nearDist[s.node] = 0; nearSub[s.node] = s.id;
      nHeap.push(0, s.node);
    }
    const nSettled = new Uint8Array(nN);
    while (nHeap.size) {
      const v = nHeap.pop();
      if (nSettled[v]) continue;
      nSettled[v] = 1;
      for (const ei of graph.adj[v]) {
        const e = graph.edges[ei];
        const w = e.a === v ? e.b : e.a;
        const nd = nearDist[v] + e.len;
        if (nd < nearDist[w] - 1e-9) {
          nearDist[w] = nd; nearSub[w] = nearSub[v]; nearParent[w] = v;
          nHeap.push(nd, w);
        }
      }
    }
    const subLoad = new Map(), posCount = new Map();
    for (const cl of finalSplit.clusters) {
      subLoad.set(cl.sid, (subLoad.get(cl.sid) ?? 0) + cl.cust);
      posCount.set(cl.sid, (posCount.get(cl.sid) ?? 0) + 1);
    }
    const leadTo = (w, subNode) => {
      const path = [w];
      for (let p = w; p !== subNode && nearParent[p] !== -1;) {
        p = nearParent[p];
        path.push(p);
        if (p === subNode) break;
      }
      return path;
    };
    const nCl = finalSplit.clusters.length; // new clusters are already homed right
    for (let ci = 0; ci < nCl; ci++) {
      const cl = finalSplit.clusters[ci];
      if (cl.cust <= 0) continue;
      const nodesArr = nodesOfCluster[ci];
      const below = belowIn(nodesArr, cl.root);
      const kidsIn = kidsWithin(nodesArr, cl.root);
      const order = nodesArr.slice().sort((a, b) => dist[a] - dist[b] || a - b);
      const skip = new Set();
      for (const w of order) {
        if (skip.has(w)) continue;
        const rs = nearSub[w];
        if (rs === -1 || rs === cl.sid) continue;
        if (dist[w] - nearDist[w] < REHOME_MIN_GAIN_M) continue;
        const n = below.get(w);
        if (n < FEEDER_MIN_CUST) continue;
        // never leave a sub-minimum remnant behind (it would be clipped)
        if (w !== cl.root && cl.cust - n < FEEDER_MIN_CUST) continue;
        if (nearDist[w] > LEAD_MAX_M) {
          rehome.blockedLead++;
          for (const v of subtreeWithin(kidsIn, w)) skip.add(v);
          continue;
        }
        if ((subLoad.get(rs) ?? 0) + n > MERGE_HEADROOM * SUB_MAX_CUST ||
            (posCount.get(rs) ?? 0) >= FEEDERS_MAX_PER_SUB) {
          rehome.blockedHeadroom++;
          for (const v of subtreeWithin(kidsIn, w)) skip.add(v);
          continue;
        }
        // commit: subtree of w re-homes to the nearer station
        const moved = subtreeWithin(kidsIn, w);
        const shift = nearDist[w] - dist[w];
        for (const v of moved) { dist[v] += shift; subOf[v] = rs; skip.add(v); }
        const path = leadTo(w, subs[rs].node);
        transferredRoots.add(w);
        subLoad.set(cl.sid, subLoad.get(cl.sid) - n);
        subLoad.set(rs, (subLoad.get(rs) ?? 0) + n);
        if (w === cl.root) { // the whole feeder belongs to the nearer sub
          posCount.set(cl.sid, posCount.get(cl.sid) - 1);
          posCount.set(rs, (posCount.get(rs) ?? 0) + 1);
          cl.sid = rs; cl.base = false; cl.leadM = nearDist[w]; cl.leadPath = path;
        } else {
          posCount.set(rs, (posCount.get(rs) ?? 0) + 1);
          const nci = finalSplit.clusters.length;
          finalSplit.clusters.push({
            sid: rs, root: w, base: false, cust: n,
            condM: 0, leadM: nearDist[w], leadPath: path, loads: [],
          });
          cl.cust -= n;
          const movedSet = new Set(moved);
          nodesOfCluster[ci] = nodesOfCluster[ci].filter(v => !movedSet.has(v));
          nodesOfCluster.push(moved.slice().sort((a, b) => a - b));
          for (const v of moved) finalSplit.nodeCluster.set(v, nci);
        }
        rehome.moved++;
        rehome.movedCust += Math.round(n);
      }
    }
  }

  // ---------- 5c. OPEN-POINT BALANCING: a short, underloaded feeder
  // ABSORBS load from a bigger neighbour across the shortest tie corridor
  // between them. The corridor is ENERGISED (it becomes real sections of
  // the absorber) and the donor's span above the moved block is released
  // to become the new normally-open tie — the open point MOVES, exactly
  // the operation a planner performs. Guards: absorber stays within
  // MERGE_HEADROOM of the feeder cap (and its station of the station
  // cap), the donor keeps a lawful feeder, balance strictly improves,
  // and the moved block's supply route doesn't blow out.
  const absorb = { moved: 0, movedCust: 0, candidates: 0 };
  {
    const isSubNode = new Uint8Array(nN);
    for (const s of subs) isSubNode[s.node] = 1;
    // corridor discovery: Dijkstra from every cluster node through
    // NON-cluster nodes only (the roads no feeder uses), labelled by
    // cluster — the same shape as the backfeed tie search
    const cDist = new Float64Array(nN).fill(Infinity);
    const cLab = new Int32Array(nN).fill(-1);
    const cRoot = new Int32Array(nN).fill(-1);
    const cParent = new Int32Array(nN).fill(-1);
    const cParentEdge = new Int32Array(nN).fill(-1);
    const cHeap = new NodeHeap();
    for (const [node, ci] of finalSplit.nodeCluster) {
      cDist[node] = 0; cLab[node] = ci; cRoot[node] = node;
      cHeap.push(0, node);
    }
    const corridorBest = new Map(); // pair key → {v, w, edgeId, span}
    const cSettled = new Uint8Array(nN);
    while (cHeap.size) {
      const v = cHeap.pop();
      if (cSettled[v]) continue;
      cSettled[v] = 1;
      for (const ei of graph.adj[v]) {
        const e = graph.edges[ei];
        const w = e.a === v ? e.b : e.a;
        if (isSubNode[w]) continue;
        const wl = finalSplit.nodeCluster.get(w);
        if (wl !== undefined) {
          if (wl !== cLab[v]) {
            const span = cDist[v] + e.len;
            if (span <= TIE_SPAN_M) {
              const a = cLab[v], b = wl;
              const key = a < b ? a * 1048576 + b : b * 1048576 + a;
              const cur = corridorBest.get(key);
              if (!cur || span < cur.span) corridorBest.set(key, { v, w, edgeId: ei, span });
            }
          }
          continue; // never traverse INTO cluster territory
        }
        const nd = cDist[v] + e.len;
        if (nd < cDist[w] - 1e-9) {
          cDist[w] = nd; cLab[w] = cLab[v]; cRoot[w] = cRoot[v];
          cParent[w] = v; cParentEdge[w] = ei;
          cHeap.push(nd, w);
        }
      }
    }
    // parent-walk lead chains: absorbing a block that another cluster's
    // lead runs THROUGH (tree-wise) would break that lead — skip those.
    // Explicit-path leads are physical parallels, unaffected by surgery.
    const chainNodes = () => {
      const s = new Set();
      for (const cl of finalSplit.clusters) {
        if (cl.base || cl.leadPath) continue;
        const path = leadWalk(cl);
        for (let i = 0; i < path.length; i++) s.add(path[i]);
      }
      return s;
    };
    let chains = chainNodes();
    const subLoad = new Map();
    for (const cl of finalSplit.clusters) {
      subLoad.set(cl.sid, (subLoad.get(cl.sid) ?? 0) + cl.cust);
    }
    const cands = [...corridorBest.values()]
      .sort((p, q) => p.span - q.span || p.v - q.v);
    for (const cand of cands) {
      if (absorb.moved >= 40) break;
      // corridor endpoints' clusters may have changed after earlier moves
      const seedNode = cRoot[cand.v];
      const ciSeed = finalSplit.nodeCluster.get(seedNode);
      const ciW = finalSplit.nodeCluster.get(cand.w);
      if (ciSeed === undefined || ciW === undefined || ciSeed === ciW) continue;
      // corridor must still be free road
      const pathNodes = [];
      let stale = false;
      for (let p = cand.v; cDist[p] > 0; p = cParent[p]) {
        if (finalSplit.nodeCluster.has(p)) { stale = true; break; }
        pathNodes.push(p); // cand.v first, seed-adjacent last
      }
      if (stale) continue;
      const clSeed = finalSplit.clusters[ciSeed], clW = finalSplit.clusters[ciW];
      // absorber = the smaller feeder; it must be genuinely underloaded
      const seedIsAbsorber = clSeed.cust <= clW.cust;
      const abs_ = seedIsAbsorber ? clSeed : clW;
      const don = seedIsAbsorber ? clW : clSeed;
      const absAnchor = seedIsAbsorber ? seedNode : cand.w;
      const donAnchor = seedIsAbsorber ? cand.w : seedNode;
      const ciAbs = seedIsAbsorber ? ciSeed : ciW;
      const ciDon = seedIsAbsorber ? ciW : ciSeed;
      absorb.candidates++;
      if (abs_.cust > BALANCE_ABSORBER_MAX) continue;
      if (don.cust - abs_.cust < BALANCE_MIN_DIFF) continue;
      if (stranded(abs_)) continue; // an express lead is no place to add load
      const donNodes = nodesOfCluster[ciDon];
      const below = belowIn(donNodes, don.root);
      const n = below.get(donAnchor);
      if (n === undefined || n < FEEDER_MIN_CUST) continue;
      if (don.cust - n < FEEDER_MIN_CUST) continue;
      if (abs_.cust + n > MERGE_HEADROOM * FEEDER_MAX_CUST) continue;
      if (Math.abs((abs_.cust + n) - (don.cust - n)) >= Math.abs(abs_.cust - don.cust)) continue;
      if (abs_.sid !== don.sid &&
          (subLoad.get(abs_.sid) ?? 0) + n > MERGE_HEADROOM * SUB_MAX_CUST) continue;
      // supply route sanity: the moved block may not detour far
      const newAnchorDist = dist[absAnchor] + cand.span;
      if (newAnchorDist > dist[donAnchor] + 5000) continue;
      const kidsDon = kidsWithin(donNodes, don.root);
      const moved = subtreeWithin(kidsDon, donAnchor);
      let breaksLead = false;
      for (const v of moved) if (chains.has(v)) { breaksLead = true; break; }
      if (breaksLead) continue;
      // ---- commit: energise the corridor toward the absorber
      // pathNodes runs cand.v → … → (adjacent to seedNode)
      if (seedIsAbsorber) {
        // feed direction seedNode → … → cand.v → cand.w
        let prev = seedNode;
        for (let i = pathNodes.length - 1; i >= 0; i--) {
          const p = pathNodes[i];
          parent[p] = prev; parentEdge[p] = cParentEdge[p];
          prev = p;
        }
        parent[cand.w] = prev; parentEdge[cand.w] = cand.edgeId;
      } else {
        // feed direction cand.w → cand.v → … → seedNode
        let prev = cand.w, prevEdge = cand.edgeId;
        for (let i = 0; i < pathNodes.length; i++) {
          const p = pathNodes[i];
          parent[p] = prev; parentEdge[p] = prevEdge;
          prevEdge = cParentEdge[p]; prev = p;
        }
        parent[seedNode] = prev; parentEdge[seedNode] = prevEdge;
      }
      // corridor nodes join the absorber (loadless pass-through)
      let cum = dist[absAnchor];
      const corridorOrder = seedIsAbsorber ? pathNodes.slice().reverse() : pathNodes.slice();
      for (const p of corridorOrder) {
        cum += graph.edges[parentEdge[p]].len;
        dist[p] = cum; subOf[p] = abs_.sid;
        finalSplit.nodeCluster.set(p, ciAbs);
        nodesOfCluster[ciAbs].push(p);
      }
      const shift = (dist[absAnchor] + cand.span) - dist[donAnchor];
      const movedSet = new Set(moved);
      for (const v of moved) {
        dist[v] += shift; subOf[v] = abs_.sid;
        finalSplit.nodeCluster.set(v, ciAbs);
        nodesOfCluster[ciAbs].push(v);
      }
      nodesOfCluster[ciDon] = donNodes.filter(v => !movedSet.has(v));
      abs_.cust += n; don.cust -= n;
      subLoad.set(don.sid, subLoad.get(don.sid) - n);
      subLoad.set(abs_.sid, (subLoad.get(abs_.sid) ?? 0) + n);
      chains = chainNodes();
      absorb.moved++;
      absorb.movedCust += Math.round(n);
    }
  }

  // ---------- 5d. RELIABILITY SPLITS: the caps say what a feeder MAY be;
  // FEEDER_TARGET_SAIDI says what it SHOULD be. Feeders whose expected
  // device-free SAIDI (standard fuses, DEFAULT fault rates — never the
  // live UI sliders, so structure stays deterministic) exceeds the target
  // are split worst-first (by customer·minutes, the league-table order)
  // into siblings, while the station has breaker positions and the cut
  // breaks no rule: lead ≤ LEAD_MAX_M, both halves ≥ FEEDER_MIN_CUST.
  // What cannot lawfully split (long rural corridors whose interior is
  // beyond lead reach, stations out of positions) is REPORTED, not forced.
  const relSplits = { made: 0, blockedLead: 0, blockedPos: 0, overTarget: 0 };
  {
    const DEF_OH = DEFAULT_FAULT_RATES.oh, DEF_UG = DEFAULT_FAULT_RATES.ug;
    // breaker positions AFTER re-homing — count live clusters per station
    const posCount = new Map();
    for (const cl of finalSplit.clusters) {
      posCount.set(cl.sid, (posCount.get(cl.sid) ?? 0) + 1);
    }
    // expected device-free cust·min/yr of one cluster, fuses included
    const clusterCustMin = (ci) => {
      const cl = finalSplit.clusters[ci];
      const nodesArr = nodesOfCluster[ci];
      const below = belowIn(nodesArr, cl.root);
      let cm = 0;
      for (const v of nodesArr) {
        if (v === cl.root && !cl.base) continue; // boundary span carries no circuit
        const len = graph.edges[parentEdge[v]].len;
        const rate = (isUnderground(v, parent[v]) ? DEF_UG : DEF_OH) * (len / 1000);
        const dur = ((dist[v] - len / 2) / 1000) / TRAVEL_KMH * 60 + REPAIR_MIN;
        const n = below.get(v);
        cm += rate * dur * (n <= LATERAL_FUSE_MAX_CUST ? n : cl.cust);
      }
      if (!cl.base) { // lead exposure: a lead fault drops the whole cluster
        const { ohM, ugM } = leadStats(cl);
        cm += ((DEF_OH * ohM + DEF_UG * ugM) / 1000) *
          ((cl.leadM / 2 / 1000) / TRAVEL_KMH * 60 + REPAIR_MIN) * cl.cust;
      }
      return cm;
    };
    const info = finalSplit.clusters.map((cl, ci) =>
      cl.cust > 0 ? { cm: clusterCustMin(ci) } : { cm: 0 });
    const blocked = new Set();
    for (let guard = 0; guard < 80; guard++) {
      let worst = -1, worstCm = 0;
      for (let ci = 0; ci < finalSplit.clusters.length; ci++) {
        if (blocked.has(ci)) continue;
        const cl = finalSplit.clusters[ci];
        if (cl.cust <= 0) continue;
        if (info[ci].cm / cl.cust <= FEEDER_TARGET_SAIDI) continue;
        if (info[ci].cm > worstCm) { worstCm = info[ci].cm; worst = ci; }
      }
      if (worst === -1) break;
      const cl = finalSplit.clusters[worst];
      if ((posCount.get(cl.sid) ?? 0) >= FEEDERS_MAX_PER_SUB) {
        blocked.add(worst); relSplits.blockedPos++; continue;
      }
      // best legal cut: balance customers, lead within the sibling cap
      const nodesArr = nodesOfCluster[worst];
      const below = belowIn(nodesArr, cl.root);
      let pick = -1, best = Infinity;
      for (const w of nodesArr) {
        if (w === cl.root || dist[w] > LEAD_MAX_M) continue;
        const n = below.get(w);
        if (n < FEEDER_MIN_CUST || cl.cust - n < FEEDER_MIN_CUST) continue;
        const miss = Math.abs(n - cl.cust / 2);
        if (miss < best || (miss === best && w < pick)) { best = miss; pick = w; }
      }
      if (pick === -1) { blocked.add(worst); relSplits.blockedLead++; continue; }
      // commit: subtree of the cut node becomes a new sibling cluster
      const kidsIn = kidsWithin(nodesArr, cl.root);
      const moved = subtreeWithin(kidsIn, pick);
      // a cut inside a re-homed cluster inherits its lead route: cut node
      // → cluster root along the tree, then the cluster's explicit path
      let leadPath;
      if (cl.leadPath) {
        leadPath = [pick];
        for (let p = pick; p !== cl.root;) { p = parent[p]; leadPath.push(p); }
        leadPath = leadPath.concat(cl.leadPath.slice(1));
      }
      const nci = finalSplit.clusters.length;
      finalSplit.clusters.push({
        sid: cl.sid, root: pick, base: false, cust: below.get(pick),
        condM: 0, leadM: dist[pick], leadPath, loads: [],
      });
      if (transferredRoots.has(cl.root) || cl.leadPath) transferredRoots.add(pick);
      cl.cust -= below.get(pick);
      const movedSet = new Set(moved);
      nodesOfCluster[worst] = nodesArr.filter(v => !movedSet.has(v));
      nodesOfCluster.push(moved.sort((a, b) => a - b));
      for (const v of moved) finalSplit.nodeCluster.set(v, nci);
      posCount.set(cl.sid, (posCount.get(cl.sid) ?? 0) + 1);
      info[worst] = { cm: clusterCustMin(worst) };
      info.push({ cm: clusterCustMin(nci) });
      relSplits.made++;
    }
    for (let ci = 0; ci < finalSplit.clusters.length; ci++) {
      const cl = finalSplit.clusters[ci];
      if (cl.cust > 0 && info[ci].cm / cl.cust > FEEDER_TARGET_SAIDI) relSplits.overTarget++;
    }
  }

  // ---------- 6. prune to the union of TX→sub paths
  const usedEdge = new Uint8Array(graph.edges.length);
  const custAtNode = new Float64Array(nN);
  let orphanTx = 0;
  for (const tx of txs) {
    if (tx.node === -1 || !isFinite(dist[tx.node])) { tx.sub = -1; orphanTx++; continue; }
    tx.sub = subOf[tx.node];
    custAtNode[tx.node] += tx.customers.length;
    for (let v = tx.node; parent[v] !== -1; v = parent[v]) {
      // a re-homed cluster feeds via its NEW lead — the old corridor
      // beyond its root goes back to being plain road (tie material)
      if (transferredRoots.has(v)) break;
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
  const clusterAtRoot = new Map();
  for (let i = 0; i < finalSplit.clusters.length; i++) {
    const c = finalSplit.clusters[i];
    if (!c.base) { leadFeederAt.set(c.root, feederIdOfCluster[i]); clusterAtRoot.set(c.root, c); }
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
  // re-homed clusters hang from their new sub via an explicit lead, not a
  // busbar-adjacent tree edge — seed their DFS separately (each block is
  // self-contained and parent-before-child, which is all the subtree
  // accumulation below needs)
  for (const r of [...transferredRoots].sort((a, b) => a - b)) {
    if (feederOfNode[r] < 0) continue;
    const stack = [r];
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
      // (re-homed clusters carry an explicit path to their NEW station)
      const { ohM, ugM, path } = leadStats(clusterAtRoot.get(v));
      // boundary span → normally-open sibling tie (no-op for re-homed
      // roots, whose old span was never claimed)
      usedEdge[parentEdge[v]] = 0;
      const leadM = ohM + ugM;
      const te = {
        id: treeEdges.length, node: v, parentNode: subs[f.sub].node,
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
  // reliability splits — planning standard, not a hard cap: what cannot
  // lawfully split is reported, never forced
  extraChecks.push({
    name: `Reliability splits (feeder expected SAIDI ≤ ${FEEDER_TARGET_SAIDI} min/yr, device-free @ default λ)`,
    tunable: true,
    pass: relSplits.overTarget === 0,
    detail: `${relSplits.made} split(s) made worst-first within breaker positions; ` +
      `${relSplits.overTarget} feeder(s) still over target — ` +
      `${relSplits.blockedLead} with no lawful cut (interior beyond the ${LEAD_MAX_M / 1000} km lead cap / minimum sizes), ` +
      `${relSplits.blockedPos} at stations out of positions`,
  });
  // overshoot re-homing — obvious wins only; what stays put stays for a
  // reason (full station, or no lawful lead) and is counted
  extraChecks.push({
    name: `Overshoot re-homing (subtree ≥ ${REHOME_MIN_GAIN_M / 1000} km closer to another station)`,
    tunable: true,
    pass: true, // informational — blocked overshoots are legitimate interleaving
    detail: `${rehome.moved} subtree(s) (${rehome.movedCust} customers) re-homed to their ` +
      `road-nearest station; ${rehome.blockedHeadroom} left in place at stations without ` +
      `ICP/position headroom (capacity interleaving), ${rehome.blockedLead} beyond a lawful ` +
      `${LEAD_MAX_M / 1000} km lead`,
  });
  // open-point balancing — informational: how much load short feeders
  // absorbed from bigger neighbours by moving normally-open points
  extraChecks.push({
    name: `Open-point balancing (feeders ≤ ${BALANCE_ABSORBER_MAX} absorb from neighbours ≥ ${BALANCE_MIN_DIFF} bigger)`,
    tunable: true,
    pass: true, // moves are opportunistic; what stays put failed a guard for a reason
    detail: `${absorb.moved} block(s) (${absorb.movedCust} customers) absorbed across tie ` +
      `corridors (open point moved, old span released as the new tie); ` +
      `${absorb.candidates} pair(s) examined`,
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
      parsimony, relSplits, rehome, absorb,
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
