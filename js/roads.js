// roads.js — road hierarchy: arterial / collector / local.
//
// ASSUMPTIONS:
//  - Arterials: minimum spanning tree between towns (plus one loop link),
//    routed by A* over the terrain grid. Cost = distance x slope multiplier;
//    sea impassable; river cells cost 28x until a bridge exists there
//    (then 1.3x) — so the first crossing "builds" a bridge and later roads
//    reuse it. Roads cross the river ONLY at these generated bridges.
//  - Urban streets: per-town rotated lattice (130–165 m spacing) kept where
//    density is high enough, but ORGANIC rather than stamped: every point
//    wobbles (more towards the edge), fraying starts near the core, and a
//    few streets drop out entirely (dead ends, broken blocks). Every 4th
//    surviving line is a collector.
//  - Rural roads: spurs off arterials into flat country every ~1–1.6 km
//    (many fork), then nearby dead ends are LINKED into a sparse web of
//    back roads, so the countryside reads as through-routes, not stubs.
//  - Line easements (CLS_EASEMENT) are NOT roads: straight cross-country
//    power spans added by the network layer for remote transformers. They
//    are excluded from road drawing and from the bridges-only river rule.
//  - Any disconnected road component is repaired by A* — the finished
//    graph is asserted fully connected.

import { CELL, GRID_NX, GRID_NY, MAP_W, MAP_H, bfsDistanceM } from "./terrain.js";

export const CLS_ARTERIAL = 0, CLS_COLLECTOR = 1, CLS_LOCAL = 2, CLS_EASEMENT = 3;

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
      // keep the more important class, and never lose a bridge flag
      for (const ei of this.adj[a]) {
        const e = this.edges[ei];
        if (e.a === b || e.b === b) {
          if (cls < e.cls) e.cls = cls;
          if (bridge) e.bridge = true;
        }
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

  // bridgeTest(xa, ya, xb, yb) → bool, evaluated on the SNAPPED node
  // coordinates — node dedupe can shift a polyline point a few metres, so
  // testing the raw points can disagree with the exact-traversal check
  // that later audits the finished edge.
  addPolyline(pts, cls, bridgeTest = null) {
    const ids = [];
    for (const [x, y] of pts) ids.push(this.node(x, y));
    for (let i = 1; i < ids.length; i++) {
      const a = ids[i - 1], b = ids[i];
      const isBridge = bridgeTest
        ? bridgeTest(this.nx[a], this.ny[a], this.nx[b], this.ny[b])
        : false;
      this.addEdge(a, b, cls, isBridge);
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

export class GridRouter {
  constructor(terrain) {
    this.t = terrain;
    const n = GRID_NX * GRID_NY;
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
    const t = this.t, nx = GRID_NX, ny = GRID_NY;
    const [sx, sy] = t.cellOf(x0, y0);
    const [tx, ty] = t.cellOf(x1, y1);
    const start = sy * nx + sx, goal = ty * nx + tx;
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
      const cx = cur % nx, cy = (cur / nx) | 0;
      for (let dy = -1; dy <= 1; dy++) {
        for (let dx = -1; dx <= 1; dx++) {
          if (!dx && !dy) continue;
          const ax = cx + dx, ay = cy + dy;
          if (ax < 0 || ay < 0 || ax >= nx || ay >= ny) continue;
          const ni = ay * nx + ax;
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
  const n = GRID_NX; // linear cell index → (c % n, (c / n) | 0)
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
  // bridge flag per segment: exact traversal on the snapped node
  // coordinates, so the flag always agrees with the river-crossing check
  const ids = graph.addPolyline(simple, cls,
    (xa, ya, xb, yb) => terrain.segmentTouchesRiver(xa, ya, xb, yb));
  return { first: ids[0], last: ids[ids.length - 1] };
}

// STRICT LAYER ORDER: roads depend on terrain + settlements (+ their
// provisional corridors) only — NOT on customers, which are sampled later
// along the finished roads.
export function buildRoads(terrain, towns, corridors, rng) {
  const graph = new RoadGraph();
  const router = new GridRouter(terrain);
  const n = GRID_NX; // linear cell index → (c % n, (c / n) | 0)
  const addRoutedPath = (cells, cls) => addRoutedCells(terrain, graph, cells, cls);
  const arterialCellPaths = [];

  // ---- 1. Arterials: the provisional corridor skeleton becomes real
  // (anchor↔anchor + map-edge exits), plus an MST over all towns and one
  // loop link. Duplicate stretches collapse via graph edge dedupe.
  for (const cells of corridors?.paths ?? []) {
    addRoutedPath(cells, CLS_ARTERIAL);
    arterialCellPaths.push(cells);
  }
  const m = towns.length;
  if (m > 1) {
    // A link touching a rural settlement is a COLLECTOR (a sealed country
    // road), links between real towns are ARTERIALS.
    const linkCls = (i, j) =>
      (towns[i].tier === 2 || towns[j].tier === 2) ? CLS_COLLECTOR : CLS_ARTERIAL;
    const linked = new Set();
    const routeLink = (i, j) => {
      const key = i < j ? i * 1024 + j : j * 1024 + i;
      if (linked.has(key)) return;
      linked.add(key);
      const cells = router.route(towns[i].x, towns[i].y, towns[j].x, towns[j].y);
      if (cells) {
        addRoutedPath(cells, linkCls(i, j));
        arterialCellPaths.push(cells);
      }
    };
    // MST keeps every town and settlement connected…
    const inTree = new Uint8Array(m);
    inTree[0] = 1;
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
      routeLink(bi, bj);
    }
    // …then k-NEAREST links add loops: every town and settlement also
    // routes to its 2 nearest neighbours (within 45 km), so the inter-town
    // network is a web with alternative routes, not a bare tree.
    for (let i = 0; i < m; i++) {
      const near = [];
      for (let j = 0; j < m; j++) {
        if (j === i) continue;
        near.push({ j, d: Math.hypot(towns[j].x - towns[i].x, towns[j].y - towns[i].y) });
      }
      near.sort((a, b) => a.d - b.d);
      for (const { j, d } of near.slice(0, 2)) {
        if (d < 45000) routeLink(i, j);
      }
    }
  }

  // ---- 2. Urban street grids: orientation snapped to the local coast /
  // valley axis (never global north), density scaled by population, a
  // regular core fraying to irregular edges.
  const rStreet = rng.fork("streets");
  for (const t of towns) {
    t.theta = townAxis(terrain, t);
    const rT = rStreet.fork("t" + Math.round(t.x) + "_" + Math.round(t.y));
    const s = 175 - 55 * Math.min(1, t.pop); // spacing: 120 m dominant → 170 m village
    const ux = Math.cos(t.theta), uy = Math.sin(t.theta);
    const R = t.sigma * 2.3;
    const half = Math.ceil(R / s);
    const keep = new Map(); // "i,j" -> [x,y]
    for (let j = -half; j <= half; j++) {
      for (let i = -half; i <= half; i++) {
        let x = t.x + (i * ux - j * uy) * s;
        let y = t.y + (i * uy + j * ux) * s;
        const rad = Math.hypot(x - t.x, y - t.y);
        // fray: starts near the core and strengthens outward, so even the
        // centre of town is only grid-ISH, not a stamped lattice
        const fray = Math.max(0, (rad / R - 0.25) / 0.75);
        if (rT.float() < 0.65 * fray * fray) continue;
        const wob = 0.28 + 0.62 * fray; // organic wobble everywhere
        x += (rT.float() - 0.5) * s * wob;
        y += (rT.float() - 0.5) * s * wob;
        if (x < 0 || y < 0 || x > MAP_W || y > MAP_H) continue;
        if (!terrain.buildableAt(x, y)) continue;
        const gauss = Math.exp(-rad * rad / (2 * t.sigma * t.sigma));
        if (gauss < 0.075) continue;
        keep.set(i + "," + j, [x, y]);
      }
    }
    for (const [key, [x, y]] of keep) {
      const [i, j] = key.split(",").map(Number);
      for (const [di, dj] of [[1, 0], [0, 1]]) {
        const nb = keep.get((i + di) + "," + (j + dj));
        if (!nb) continue;
        if (rT.float() < 0.07) continue; // dead ends and broken blocks
        if (terrain.segmentCrossesWater(x, y, nb[0], nb[1])) continue;
        const cls = (i % 4 === 0 && dj === 1) || (j % 4 === 0 && di === 1)
          ? CLS_COLLECTOR : CLS_LOCAL;
        graph.addPolyline([[x, y], nb], cls);
      }
    }
  }

  // ---- 3. Rural roads: spurs branching off arterials into flat country
  // (terrain-driven — rural LOAD follows these roads later, not vice
  // versa). Spur seeds every ~1–1.6 km along arterials; many fork.
  const rSpur = rng.fork("spurs");
  const spurTargets = [];
  for (const cells of arterialCellPaths) {
    let acc = 0;
    for (let i = 1; i < cells.length; i++) {
      acc += CELL;
      if (acc < 1000 + rSpur.float() * 600) continue;
      acc = 0;
      if (rSpur.float() < 0.25) continue;
      const [px, py] = terrain.cellCentre(cells[i] % n, (cells[i] / n) | 0);
      const [qx, qy] = terrain.cellCentre(cells[i - 1] % n, (cells[i - 1] / n) | 0);
      const dl = Math.hypot(px - qx, py - qy) || 1;
      const side = rSpur.float() < 0.5 ? 1 : -1;
      const len = rSpur.range(1100, 5200);
      const tx = px + (-(py - qy) / dl) * len * side + (rSpur.float() - 0.5) * 900;
      const ty = py + ((px - qx) / dl) * len * side + (rSpur.float() - 0.5) * 900;
      if (tx < 400 || ty < 400 || tx > MAP_W - 400 || ty > MAP_H - 400) continue;
      if (!terrain.buildableAt(tx, ty) || terrain.slopeAt(tx, ty) > 0.3) continue;
      const cellsSpur = router.route(px, py, tx, ty);
      if (cellsSpur) {
        addRoutedPath(cellsSpur, CLS_LOCAL);
        spurTargets.push([tx, ty]);
      }
    }
  }
  const linkEnds = spurTargets.slice(); // spur ends + fork ends, for the web pass
  for (const [sx, sy] of spurTargets) {
    if (rSpur.float() > 0.55) continue; // many spurs fork once more
    const tx = sx + (rSpur.float() - 0.5) * 5200;
    const ty = sy + (rSpur.float() - 0.5) * 5200;
    if (tx < 400 || ty < 400 || tx > MAP_W - 400 || ty > MAP_H - 400) continue;
    if (!terrain.buildableAt(tx, ty) || terrain.slopeAt(tx, ty) > 0.3) continue;
    const cellsSpur = router.route(sx, sy, tx, ty);
    if (cellsSpur) {
      addRoutedPath(cellsSpur, CLS_LOCAL);
      linkEnds.push([tx, ty]);
    }
  }

  // ---- 3b. Rural web: join nearby dead ends into through routes, so the
  // back country reads as a sparse road web rather than a comb of stubs.
  // Straight-line river hits are skipped — country lanes don't casually
  // build bridges (arterials still do, at 28x).
  for (let i = 0; i < linkEnds.length; i++) {
    if (rSpur.float() > 0.55) continue;
    const [ax, ay] = linkEnds[i];
    let bj = -1, bd = Infinity;
    for (let j = 0; j < linkEnds.length; j++) {
      if (j === i) continue;
      const d = Math.hypot(linkEnds[j][0] - ax, linkEnds[j][1] - ay);
      if (d > 800 && d < bd) { bd = d; bj = j; } // >800 m: skip trivial stubs
    }
    if (bj === -1 || bd > 6500) continue;
    const [bx, by] = linkEnds[bj];
    if (terrain.segmentTouchesRiver(ax, ay, bx, by)) continue;
    const cellsLink = router.route(ax, ay, bx, by);
    if (cellsLink) addRoutedPath(cellsLink, CLS_LOCAL);
  }

  // ---- 4. Connectivity repair (assert-backed, not assumed)
  const repair = repairConnectivity(graph, terrain, router);

  return { graph, repair };
}

// Local grid axis for a town: coastline tangent when the sea is close,
// river direction when the river is close, else the terrain contour
// direction. Normalised to [0, π/2) — grids are 4-fold symmetric.
function townAxis(terrain, t) {
  const norm = (a) => ((a % (Math.PI / 2)) + Math.PI / 2) % (Math.PI / 2);
  // nearest ocean cell within ~4.5 km
  const [tcx, tcy] = terrain.cellOf(t.x, t.y);
  let oceanD = Infinity, oceanAng = 0;
  const R = Math.ceil(4500 / CELL);
  for (let dy = -R; dy <= R; dy++) {
    for (let dx = -R; dx <= R; dx++) {
      const ax = tcx + dx, ay = tcy + dy;
      if (!terrain.inGrid(ax, ay)) continue;
      if (!terrain.ocean[terrain.idx(ax, ay)]) continue;
      const d = Math.hypot(dx, dy) * CELL;
      if (d < oceanD) { oceanD = d; oceanAng = Math.atan2(dy, dx); }
    }
  }
  // nearest river segment direction
  const rp = terrain.riverPath;
  let riverD = Infinity, riverAng = 0;
  for (let i = 3; i < rp.length - 3; i += 3) {
    const d = Math.hypot(rp[i].x - t.x, rp[i].y - t.y);
    if (d < riverD) {
      riverD = d;
      riverAng = Math.atan2(rp[i + 3].y - rp[i - 3].y, rp[i + 3].x - rp[i - 3].x);
    }
  }
  if (oceanD < 3500 && oceanD <= riverD) return norm(oceanAng + Math.PI / 2); // coast tangent
  if (riverD < 3500) return norm(riverAng);
  // contour direction: perpendicular to the elevation gradient
  const h = 400;
  const gx = terrain.elevAt(t.x + h, t.y) - terrain.elevAt(t.x - h, t.y);
  const gy = terrain.elevAt(t.x, t.y + h) - terrain.elevAt(t.x, t.y - h);
  return norm(Math.atan2(gy, gx) + Math.PI / 2);
}

// Distance-to-road field (metres per grid cell) — feeds roadside rural load.
export function roadDistanceGrid(terrain, graph) {
  const seeds = new Set();
  for (const e of graph.edges) {
    terrain.segmentHits(graph.nx[e.a], graph.ny[e.a], graph.nx[e.b], graph.ny[e.b],
      (i) => { seeds.add(i); return false; });
  }
  return bfsDistanceM(terrain, [...seeds]);
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
  // generous cap: street dropout + fraying on the 100 km map can leave
  // many small fragments, each merged one iteration at a time
  for (let iter = 0; iter < 400; iter++) {
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
      // stitch both ends — bridge-flagged exactly, since an endpoint can
      // be a bridge node sitting inside a river cell
      graph.addEdge(bestA, ends.first, CLS_LOCAL, terrain.segmentTouchesRiver(
        graph.nx[bestA], graph.ny[bestA], graph.nx[ends.first], graph.ny[ends.first]));
      graph.addEdge(ends.last, bestB, CLS_LOCAL, terrain.segmentTouchesRiver(
        graph.nx[ends.last], graph.ny[ends.last], graph.nx[bestB], graph.ny[bestB]));
    }
    merges++;
  }
  const { nComp } = components(graph);
  return { merges, connected: nComp <= 1 };
}
