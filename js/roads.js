// roads.js — road hierarchy: arterial / collector / local.
//
// ASSUMPTIONS:
//  - Arterials: minimum spanning tree between towns (plus one loop link),
//    routed by A* over the terrain grid. Cost = distance x slope multiplier;
//    sea impassable; river cells cost 28x until a bridge exists there
//    (then 1.3x) — so the first crossing "builds" a bridge and later roads
//    reuse it. Roads cross the river ONLY at these generated bridges.
//  - Urban streets: per-town rotated lattice (130–165 m spacing) kept where
//    density is high enough; every 4th line is a collector. Grid-ish in
//    town, absent in the country.
//  - Rural roads: customer clusters (800 m bins) more than ~350 m from a
//    road get a local road spur routed by the same terrain-aware A*.
//  - Any disconnected road component is repaired by A* — the finished
//    graph is asserted fully connected.

import { CELL, GRID_N, MAP_SIZE } from "./terrain.js";

export const CLS_ARTERIAL = 0, CLS_COLLECTOR = 1, CLS_LOCAL = 2;

// ---------------------------------------------------------------- helpers

class MinHeap {
  constructor() { this.f = []; this.v = []; }
  get size() { return this.f.length; }
  push(f, v) {
    const F = this.f, V = this.v;
    let i = F.length;
    F.push(f); V.push(v);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (F[p] <= F[i]) break;
      [F[p], F[i]] = [F[i], F[p]];
      [V[p], V[i]] = [V[i], V[p]];
      i = p;
    }
  }
  pop() {
    const F = this.f, V = this.v;
    const top = V[0];
    const lf = F.pop(), lv = V.pop();
    if (F.length) {
      F[0] = lf; V[0] = lv;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1, r = l + 1;
        let m = i;
        if (l < F.length && F[l] < F[m]) m = l;
        if (r < F.length && F[r] < F[m]) m = r;
        if (m === i) break;
        [F[m], F[i]] = [F[i], F[m]];
        [V[m], V[i]] = [V[i], V[m]];
        i = m;
      }
    }
    return top;
  }
}

function simplifyPolyline(pts, tol) {
  if (pts.length < 3) return pts;
  const keep = new Uint8Array(pts.length);
  keep[0] = keep[pts.length - 1] = 1;
  const stack = [[0, pts.length - 1]];
  while (stack.length) {
    const [a, b] = stack.pop();
    let maxD = -1, maxI = -1;
    const ax = pts[a][0], ay = pts[a][1], bx = pts[b][0], by = pts[b][1];
    const dx = bx - ax, dy = by - ay;
    const len2 = dx * dx + dy * dy || 1;
    for (let i = a + 1; i < b; i++) {
      const t = Math.max(0, Math.min(1, ((pts[i][0] - ax) * dx + (pts[i][1] - ay) * dy) / len2));
      const d = Math.hypot(pts[i][0] - (ax + t * dx), pts[i][1] - (ay + t * dy));
      if (d > maxD) { maxD = d; maxI = i; }
    }
    if (maxD > tol) {
      keep[maxI] = 1;
      stack.push([a, maxI], [maxI, b]);
    }
  }
  return pts.filter((_, i) => keep[i]);
}

// ------------------------------------------------------------- RoadGraph

export class RoadGraph {
  constructor() {
    this.nx = []; this.ny = [];
    this.keyMap = new Map();     // 10 m dedupe grid → node id
    this.edges = [];             // {a,b,len,cls,bridge}
    this.adj = [];               // node id → [edge ids]
    this.edgeKeys = new Set();
    this.hash = new Map();       // 250 m spatial hash → [node ids]
  }
  get nNodes() { return this.nx.length; }

  _hkey(x, y) { return ((x / 250) | 0) * 4096 + ((y / 250) | 0); }

  node(x, y) {
    const k = `${Math.round(x / 10)},${Math.round(y / 10)}`;
    let id = this.keyMap.get(k);
    if (id !== undefined) return id;
    id = this.nx.length;
    this.nx.push(x); this.ny.push(y);
    this.keyMap.set(k, id);
    this.adj.push([]);
    const hk = this._hkey(x, y);
    let arr = this.hash.get(hk);
    if (!arr) this.hash.set(hk, arr = []);
    arr.push(id);
    return id;
  }

  addEdge(a, b, cls, bridge = false) {
    if (a === b) return;
    const key = a < b ? a * 16777216 + b : b * 16777216 + a;
    if (this.edgeKeys.has(key)) {
      // keep the more important class
      for (const ei of this.adj[a]) {
        const e = this.edges[ei];
        if ((e.a === b || e.b === b) && cls < e.cls) e.cls = cls;
      }
      return;
    }
    this.edgeKeys.add(key);
    const len = Math.hypot(this.nx[a] - this.nx[b], this.ny[a] - this.ny[b]);
    const id = this.edges.length;
    this.edges.push({ a, b, len, cls, bridge });
    this.adj[a].push(id);
    this.adj[b].push(id);
  }

  addPolyline(pts, cls, bridgeSpans = null) {
    let prev = -1;
    const ids = [];
    for (const [x, y] of pts) ids.push(this.node(x, y));
    for (let i = 1; i < ids.length; i++) {
      const isBridge = bridgeSpans ? bridgeSpans[i - 1] : false;
      this.addEdge(ids[i - 1], ids[i], cls, isBridge);
    }
    return ids;
  }

  nearestNode(x, y, maxR = 4000, filter = null) {
    const cx = (x / 250) | 0, cy = (y / 250) | 0;
    let best = -1, bestD = Infinity;
    const maxRing = Math.ceil(maxR / 250) + 1;
    for (let ring = 0; ring <= maxRing; ring++) {
      if (best !== -1 && bestD < (ring - 1) * 250) break;
      for (let dy = -ring; dy <= ring; dy++) {
        for (let dx = -ring; dx <= ring; dx++) {
          if (Math.max(Math.abs(dx), Math.abs(dy)) !== ring) continue;
          const arr = this.hash.get((cx + dx) * 4096 + (cy + dy));
          if (!arr) continue;
          for (const id of arr) {
            if (filter && !filter(id)) continue;
            const d = Math.hypot(this.nx[id] - x, this.ny[id] - y);
            if (d < bestD) { bestD = d; best = id; }
          }
        }
      }
    }
    return bestD <= maxR ? { id: best, dist: bestD } : { id: -1, dist: Infinity };
  }
}

// -------------------------------------------------------- terrain-grid A*

class GridRouter {
  constructor(terrain) {
    this.t = terrain;
    const n = GRID_N * GRID_N;
    this.g = new Float64Array(n);
    this.came = new Int32Array(n);
    this.stamp = new Int32Array(n);
    this.gen = 0;
  }
  // cost multiplier for entering a cell
  cellMul(i) {
    const t = this.t;
    if (t.water[i] === 1) return -1;               // sea: blocked
    if (t.water[i] === 2) return t.bridgeCell[i] ? 1.3 : 28; // river
    if (t.water[i] === 0 && !t.mainland[i]) return -1;       // stranded pocket
    return t.slopeCostMul(i);
  }
  route(x0, y0, x1, y1) {
    const t = this.t, n = GRID_N;
    const [sx, sy] = t.cellOf(x0, y0);
    const [tx, ty] = t.cellOf(x1, y1);
    const start = sy * n + sx, goal = ty * n + tx;
    this.gen++;
    const { g, came, stamp } = this;
    const heap = new MinHeap();
    g[start] = 0; came[start] = -1; stamp[start] = this.gen;
    heap.push(0, start);
    const closed = new Set();
    while (heap.size) {
      const cur = heap.pop();
      if (cur === goal) break;
      if (closed.has(cur)) continue;
      closed.add(cur);
      const cx = cur % n, cy = (cur / n) | 0;
      for (let dy = -1; dy <= 1; dy++) {
        for (let dx = -1; dx <= 1; dx++) {
          if (!dx && !dy) continue;
          const ax = cx + dx, ay = cy + dy;
          if (ax < 0 || ay < 0 || ax >= n || ay >= n) continue;
          const ni = ay * n + ax;
          const mul = this.cellMul(ni);
          if (mul < 0) continue;
          const step = CELL * (dx && dy ? Math.SQRT2 : 1) * mul;
          const ng = g[cur] + step;
          if (stamp[ni] !== this.gen || ng < g[ni]) {
            stamp[ni] = this.gen;
            g[ni] = ng;
            came[ni] = cur;
            const h = Math.hypot((ax - tx), (ay - ty)) * CELL;
            heap.push(ng + h, ni);
          }
        }
      }
    }
    if (stamp[goal] !== this.gen) return null;
    const cells = [];
    for (let c = goal; c !== -1; c = came[c]) {
      cells.push(c);
      if (c === start) break;
    }
    cells.reverse();
    return cells;
  }
}

// ------------------------------------------------------------ build roads

// Turn an A* cell path into road edges: registers bridges (consecutive runs
// of river cells), simplifies, flags bridge spans. EVERY routed path —
// arterial, rural spur, or connectivity repair — must go through here so
// that river crossings are always bridges. Returns end node ids.
function addRoutedCells(terrain, graph, cells, cls) {
  const n = GRID_N;
  if (!cells || cells.length < 2) return null;
  let run = [];
  const flushRun = () => {
    if (run.length) {
      let bx = 0, by = 0;
      for (const c of run) {
        terrain.bridgeCell[c] = 1;
        const [x, y] = terrain.cellCentre(c % n, (c / n) | 0);
        bx += x; by += y;
      }
      terrain.bridges.push({ x: bx / run.length, y: by / run.length });
      run = [];
    }
  };
  for (const c of cells) {
    if (terrain.water[c] === 2) run.push(c);
    else flushRun();
  }
  flushRun();
  const pts = cells.map(c => terrain.cellCentre(c % n, (c / n) | 0));
  // Simplify, but pin every point near the river so the polyline never
  // corner-cuts a bend: split at pinned points, simplify each run.
  const pinned = cells.map((c, i) =>
    terrain.water[c] === 2 ||
    (i > 0 && terrain.water[cells[i - 1]] === 2) ||
    (i < cells.length - 1 && terrain.water[cells[i + 1]] === 2));
  const simple = [];
  let runStart = 0;
  for (let i = 1; i < pts.length; i++) {
    if (pinned[i] || i === pts.length - 1) {
      const part = simplifyPolyline(pts.slice(runStart, i + 1), 45);
      for (let k = simple.length ? 1 : 0; k < part.length; k++) simple.push(part[k]);
      runStart = i;
    }
  }
  // bridge flag per segment: exact traversal, no sampling gaps
  const spans = [];
  for (let i = 1; i < simple.length; i++) {
    spans.push(terrain.segmentTouchesRiver(
      simple[i - 1][0], simple[i - 1][1], simple[i][0], simple[i][1]));
  }
  const ids = graph.addPolyline(simple, cls, spans);
  return { first: ids[0], last: ids[ids.length - 1] };
}

export function buildRoads(terrain, towns, customers, rng) {
  const graph = new RoadGraph();
  const router = new GridRouter(terrain);
  const n = GRID_N;
  const addRoutedPath = (cells, cls) => addRoutedCells(terrain, graph, cells, cls);

  // ---- 1. Arterials: MST over towns + one loop link
  const m = towns.length;
  if (m > 1) {
    const inTree = new Uint8Array(m);
    inTree[0] = 1;
    const links = [];
    for (let k = 1; k < m; k++) {
      let bi = -1, bj = -1, bd = Infinity;
      for (let i = 0; i < m; i++) {
        if (!inTree[i]) continue;
        for (let j = 0; j < m; j++) {
          if (inTree[j]) continue;
          const d = Math.hypot(towns[i].x - towns[j].x, towns[i].y - towns[j].y);
          if (d < bd) { bd = d; bi = i; bj = j; }
        }
      }
      inTree[bj] = 1;
      links.push([bi, bj]);
    }
    // loop link: closest pair not already linked
    let extra = null, bd = Infinity;
    for (let i = 0; i < m; i++) {
      for (let j = i + 1; j < m; j++) {
        if (links.some(([a, b]) => (a === i && b === j) || (a === j && b === i))) continue;
        const d = Math.hypot(towns[i].x - towns[j].x, towns[i].y - towns[j].y);
        if (d < bd && d < 14000) { bd = d; extra = [i, j]; }
      }
    }
    if (extra) links.push(extra);
    for (const [i, j] of links) {
      const cells = router.route(towns[i].x, towns[i].y, towns[j].x, towns[j].y);
      addRoutedPath(cells, CLS_ARTERIAL);
    }
  }

  // ---- 2. Urban street grids
  const rStreet = rng.fork("streets");
  for (const t of towns) {
    const theta = rStreet.range(0, Math.PI / 2);
    const s = rStreet.range(130, 165);
    const ux = Math.cos(theta), uy = Math.sin(theta);
    const R = t.sigma * 2.3;
    const half = Math.ceil(R / s);
    const keep = new Map(); // "i,j" -> [x,y]
    for (let j = -half; j <= half; j++) {
      for (let i = -half; i <= half; i++) {
        const x = t.x + (i * ux - j * uy) * s;
        const y = t.y + (i * uy + j * ux) * s;
        if (x < 0 || y < 0 || x > MAP_SIZE || y > MAP_SIZE) continue;
        if (!terrain.buildableAt(x, y)) continue;
        const gauss = t.weight * Math.exp(-((x - t.x) ** 2 + (y - t.y) ** 2) / (2 * t.sigma * t.sigma));
        if (gauss < 0.075 * t.weight) continue;
        keep.set(i + "," + j, [x, y]);
      }
    }
    for (const [key, [x, y]] of keep) {
      const [i, j] = key.split(",").map(Number);
      for (const [di, dj] of [[1, 0], [0, 1]]) {
        const nb = keep.get((i + di) + "," + (j + dj));
        if (!nb) continue;
        if (terrain.segmentCrossesWater(x, y, nb[0], nb[1])) continue;
        const cls = (i % 4 === 0 && dj === 1) || (j % 4 === 0 && di === 1)
          ? CLS_COLLECTOR : CLS_LOCAL;
        graph.addPolyline([[x, y], nb], cls);
      }
    }
  }

  // ---- 3. Rural roads to customer clusters
  const BIN = 800;
  const bins = new Map();
  for (const c of customers) {
    const k = ((c.x / BIN) | 0) * 4096 + ((c.y / BIN) | 0);
    let b = bins.get(k);
    if (!b) bins.set(k, b = { x: 0, y: 0, count: 0 });
    b.x += c.x; b.y += c.y; b.count++;
  }
  const clusters = [...bins.values()]
    .map(b => ({ x: b.x / b.count, y: b.y / b.count, count: b.count }))
    .sort((p, q) => q.count - p.count);
  for (const cl of clusters) {
    const near = graph.nearestNode(cl.x, cl.y, 30000);
    if (near.id === -1) continue;
    if (near.dist <= 350) continue;
    const cells = router.route(graph.nx[near.id], graph.ny[near.id], cl.x, cl.y);
    const ends = addRoutedPath(cells, CLS_LOCAL);
    // stitch exact start node to first path node (same cell → cannot cross water)
    if (ends) graph.addEdge(near.id, ends.first, CLS_LOCAL);
  }

  // ---- 4. Connectivity repair (assert-backed, not assumed)
  const repair = repairConnectivity(graph, terrain, router);

  // ---- 5. Snap customers to nearest road node
  let maxSnap = 0, sumSnap = 0;
  for (const c of customers) {
    const near = graph.nearestNode(c.x, c.y, 30000);
    c.node = near.id;
    maxSnap = Math.max(maxSnap, near.dist);
    sumSnap += near.dist;
  }

  return {
    graph,
    repair,
    snapStats: { max: maxSnap, mean: sumSnap / Math.max(1, customers.length) },
  };
}

export function components(graph) {
  const nN = graph.nNodes;
  const comp = new Int32Array(nN).fill(-1);
  let nComp = 0;
  const stack = [];
  for (let s = 0; s < nN; s++) {
    if (comp[s] !== -1) continue;
    comp[s] = nComp;
    stack.push(s);
    while (stack.length) {
      const v = stack.pop();
      for (const ei of graph.adj[v]) {
        const e = graph.edges[ei];
        const w = e.a === v ? e.b : e.a;
        if (comp[w] === -1) { comp[w] = nComp; stack.push(w); }
      }
    }
    nComp++;
  }
  return { comp, nComp };
}

function repairConnectivity(graph, terrain, router) {
  let merges = 0;
  for (let iter = 0; iter < 80; iter++) {
    const { comp, nComp } = components(graph);
    if (nComp <= 1) return { merges, connected: true };
    // sizes
    const size = new Int32Array(nComp);
    for (let v = 0; v < graph.nNodes; v++) size[comp[v]]++;
    let main = 0;
    for (let c = 1; c < nComp; c++) if (size[c] > size[main]) main = c;
    // pick any non-main component, find closest node pair to main
    let target = -1;
    for (let c = 0; c < nComp; c++) if (c !== main) { target = c; break; }
    let bestA = -1, bestB = -1, bestD = Infinity;
    for (let v = 0; v < graph.nNodes; v++) {
      if (comp[v] !== target) continue;
      const near = graph.nearestNode(graph.nx[v], graph.ny[v], 30000, id => comp[id] === main);
      if (near.id !== -1 && near.dist < bestD) { bestD = near.dist; bestA = v; bestB = near.id; }
    }
    if (bestA === -1) return { merges, connected: false };
    if (bestD < 320 && !terrain.segmentCrossesWater(
      graph.nx[bestA], graph.ny[bestA], graph.nx[bestB], graph.ny[bestB])) {
      graph.addEdge(bestA, bestB, CLS_LOCAL);
    } else {
      const cells = router.route(graph.nx[bestA], graph.ny[bestA], graph.nx[bestB], graph.ny[bestB]);
      const ends = addRoutedCells(terrain, graph, cells, CLS_LOCAL);
      if (!ends) return { merges, connected: false };
      // stitch both ends (each stays within its own cell → no water crossing)
      graph.addEdge(bestA, ends.first, CLS_LOCAL);
      graph.addEdge(ends.last, bestB, CLS_LOCAL);
    }
    merges++;
  }
  const { nComp } = components(graph);
  return { merges, connected: nComp <= 1 };
}
