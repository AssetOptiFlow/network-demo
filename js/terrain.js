// terrain.js — synthetic geography for a ~30 x 30 km map.
//
// ASSUMPTIONS (also surfaced in the UI assumptions panel):
//  - Map is 30 x 30 km sampled on a 200 m grid (150 x 150 cells).
//  - Sea lies along one map edge (chosen from the seed); elevation is
//    fBm noise plus a ramp away from that edge, so the coastline wiggles.
//  - Exactly one river is traced from a high inland cell to the sea by
//    noisy steepest descent; it is carved into the elevation and
//    rasterised as water (60–140 m wide).
//  - "Buildable" = land in the main connected landmass with slope < 0.35 m/m.
//  - Elevation is purely invented (no real data). Vertical scale ~0–700 m.

import { fbm } from "./noise.js";

export const MAP_SIZE = 30000;   // metres
export const CELL = 200;         // metres per grid cell
export const GRID_N = MAP_SIZE / CELL; // 150

const ELEV_SCALE_M = 700;        // multiplier from unit elevation to metres
const SLOPE_BUILD_MAX = 0.35;    // m/m — steeper than this is unbuildable

export class Terrain {
  constructor(rng) {
    const n = GRID_N;
    this.n = n;
    this.elev = new Float32Array(n * n);   // unit elevation, <0 means sea
    this.slope = new Float32Array(n * n);  // m/m
    this.water = new Uint8Array(n * n);    // 0 land, 1 sea, 2 river
    this.mainland = new Uint8Array(n * n); // 1 = in largest land component
    this.bridgeCell = new Uint8Array(n * n); // set by road builder
    this.riverPath = [];                   // [{x,y,w}] metres
    this.bridges = [];                     // [{x,y}] set by road builder

    const seedT = Math.floor(rng.fork("elev").float() * 1e9);
    this.noiseSeed = seedT;
    this.coastEdge = rng.fork("coast").int(0, 3); // 0 W, 1 E, 2 S, 3 N

    this._buildElevation(seedT);
    this._floodOcean();
    this._traceRiver(rng.fork("river"));
    this._computeSlope();
    this._floodMainland();
  }

  idx(cx, cy) { return cy * this.n + cx; }
  inGrid(cx, cy) { return cx >= 0 && cy >= 0 && cx < this.n && cy < this.n; }
  cellOf(x, y) {
    return [
      Math.min(this.n - 1, Math.max(0, Math.floor(x / CELL))),
      Math.min(this.n - 1, Math.max(0, Math.floor(y / CELL))),
    ];
  }
  cellCentre(cx, cy) { return [(cx + 0.5) * CELL, (cy + 0.5) * CELL]; }

  // Normalised distance from the coast edge (0 at coast, 1 far side).
  _coastDist(cx, cy) {
    const t = this.n - 1;
    switch (this.coastEdge) {
      case 0: return cx / t;
      case 1: return (t - cx) / t;
      case 2: return cy / t;
      default: return (t - cy) / t;
    }
  }

  _buildElevation(seed) {
    const n = this.n;
    const freq = 3.4; // noise cycles across map
    for (let cy = 0; cy < n; cy++) {
      for (let cx = 0; cx < n; cx++) {
        const u = cx / n, v = cy / n;
        const g = this._coastDist(cx, cy);
        const nz = fbm(u * freq, v * freq, seed, 5);
        // Ramp keeps the coast edge below sea level, inland rises.
        let e = nz * 0.52 + g * 1.35 - 0.30;
        this.elev[this.idx(cx, cy)] = e;
        if (e < 0) this.water[this.idx(cx, cy)] = 1; // sea
      }
    }
  }

  // Ocean = sea cells connected to the coast edge. Inland elev<0 pockets
  // stay water (lakes) but are NOT ocean — the river must reach the ocean.
  _floodOcean() {
    const n = this.n;
    this.ocean = new Uint8Array(n * n);
    const stack = [];
    for (let k = 0; k < n; k++) {
      const [cx, cy] = this.coastEdge === 0 ? [0, k]
        : this.coastEdge === 1 ? [n - 1, k]
        : this.coastEdge === 2 ? [k, 0] : [k, n - 1];
      const i = this.idx(cx, cy);
      if (this.water[i] === 1 && !this.ocean[i]) { this.ocean[i] = 1; stack.push(i); }
    }
    while (stack.length) {
      const i = stack.pop();
      const cx = i % n, cy = (i / n) | 0;
      for (const [ax, ay] of [[cx - 1, cy], [cx + 1, cy], [cx, cy - 1], [cx, cy + 1]]) {
        if (!this.inGrid(ax, ay)) continue;
        const j = this.idx(ax, ay);
        if (this.water[j] === 1 && !this.ocean[j]) { this.ocean[j] = 1; stack.push(j); }
      }
    }
  }

  elevAt(x, y) { // bilinear, unit elevation
    const gx = Math.min(this.n - 1.001, Math.max(0, x / CELL - 0.5));
    const gy = Math.min(this.n - 1.001, Math.max(0, y / CELL - 0.5));
    const x0 = Math.floor(gx), y0 = Math.floor(gy);
    const fx = gx - x0, fy = gy - y0;
    const e = this.elev, n = this.n;
    const a = e[y0 * n + x0], b = e[y0 * n + x0 + 1];
    const c = e[(y0 + 1) * n + x0], d = e[(y0 + 1) * n + x0 + 1];
    return a + (b - a) * fx + (c - a) * fy + (a - b - c + d) * fx * fy;
  }

  _traceRiver(rng) {
    const n = this.n;
    // Source: a high land cell hard against the edge OPPOSITE the coast,
    // so the river spans the whole map and genuinely bisects it — roads
    // between towns on either side are forced onto bridges.
    let candidates = [];
    const edgeBand = (n - 3) / (n - 1); // outermost ~2 cells
    for (let cy = 1; cy < n - 1; cy++) {
      for (let cx = 1; cx < n - 1; cx++) {
        if (this._coastDist(cx, cy) < edgeBand) continue; // far edge band only
        // central 60% along the far edge, so the river cannot hug a side
        const lat = (this.coastEdge <= 1 ? cy : cx) / (n - 1);
        if (lat < 0.2 || lat > 0.8) continue;
        const i = this.idx(cx, cy);
        if (this.water[i]) continue;
        const score = this.elev[i];
        candidates.push({ cx, cy, score });
      }
    }
    if (candidates.length === 0) { // degenerate fallback: anywhere inland
      for (let cy = 4; cy < n - 4; cy += 2) {
        for (let cx = 4; cx < n - 4; cx += 2) {
          const i = this.idx(cx, cy);
          if (this.water[i]) continue;
          candidates.push({ cx, cy, score: this.elev[i] * this._coastDist(cx, cy) });
        }
      }
    }
    candidates.sort((a, b) => b.score - a.score);
    const src = candidates[rng.int(0, Math.min(11, candidates.length - 1))];
    let [px, py] = this.cellCentre(src.cx, src.cy);
    // Pin the head of the river to the actual map border so no road can
    // route around the headwater — crossings must use bridges.
    const border = [
      { x: MAP_SIZE, y: py }, { x: 0, y: py },
      { x: px, y: MAP_SIZE }, { x: px, y: 0 },
    ][this.coastEdge];
    const pts0 = [];
    const gap = Math.hypot(border.x - px, border.y - py);
    const nGap = Math.max(1, Math.ceil(gap / 90));
    for (let s = 0; s < nGap; s++) {
      const t = s / nGap;
      pts0.push({ x: border.x + (px - border.x) * t, y: border.y + (py - border.y) * t });
    }

    // Coastward unit vector.
    const cw = [[-1, 0], [1, 0], [0, -1], [0, 1]][this.coastEdge];
    let mvx = cw[0], mvy = cw[1]; // momentum
    const step = 100; // metres
    const pts = pts0;
    for (let s = 0; s < 3000; s++) {
      pts.push({ x: px, y: py });
      const [cx, cy] = this.cellOf(px, py);
      if (this.ocean[this.idx(cx, cy)]) break; // reached the actual sea
      // Downhill direction by central differences on bilinear elevation.
      const h = 150;
      let gx = this.elevAt(px + h, py) - this.elevAt(px - h, py);
      let gy = this.elevAt(px, py + h) - this.elevAt(px, py - h);
      let dl = Math.hypot(gx, gy);
      let dhx = 0, dhy = 0;
      if (dl > 1e-9) { dhx = -gx / dl; dhy = -gy / dl; }
      const jit = (rng.float() - 0.5) * 1.1;
      let dx = dhx * 0.55 + mvx * 0.55 + cw[0] * 0.45 + jit * -mvy;
      let dy = dhy * 0.55 + mvy * 0.55 + cw[1] * 0.45 + jit * mvx;
      const dlen = Math.hypot(dx, dy) || 1;
      dx /= dlen; dy /= dlen;
      mvx = dx; mvy = dy;
      px += dx * step; py += dy * step;
      if (px < 0 || py < 0 || px > MAP_SIZE || py > MAP_SIZE) break;
      // Carve so the river never flows uphill visually.
      const ci = this.idx(...this.cellOf(px, py));
      const prev = this.elev[this.idx(...this.cellOf(pts[pts.length - 1].x, pts[pts.length - 1].y))];
      if (this.elev[ci] > prev) this.elev[ci] = prev - 0.002;
    }
    // Width grows downstream.
    const m = pts.length;
    for (let i = 0; i < m; i++) {
      const t = i / Math.max(1, m - 1);
      pts[i].w = 60 + 80 * t;
    }
    this.riverPath = pts;
    // Rasterise into water=2.
    for (const p of pts) {
      const r = p.w / 2 + CELL * 0.35;
      const c0x = Math.max(0, Math.floor((p.x - r) / CELL));
      const c1x = Math.min(n - 1, Math.floor((p.x + r) / CELL));
      const c0y = Math.max(0, Math.floor((p.y - r) / CELL));
      const c1y = Math.min(n - 1, Math.floor((p.y + r) / CELL));
      for (let cy = c0y; cy <= c1y; cy++) {
        for (let cx = c0x; cx <= c1x; cx++) {
          const [mx, my] = this.cellCentre(cx, cy);
          if (Math.hypot(mx - p.x, my - p.y) <= r) {
            const i = this.idx(cx, cy);
            if (this.water[i] === 0) this.water[i] = 2; // river
          }
        }
      }
    }
  }

  _computeSlope() {
    const n = this.n;
    for (let cy = 0; cy < n; cy++) {
      for (let cx = 0; cx < n; cx++) {
        const xm = this.elev[this.idx(Math.max(0, cx - 1), cy)];
        const xp = this.elev[this.idx(Math.min(n - 1, cx + 1), cy)];
        const ym = this.elev[this.idx(cx, Math.max(0, cy - 1))];
        const yp = this.elev[this.idx(cx, Math.min(n - 1, cy + 1))];
        const dzdx = (xp - xm) * ELEV_SCALE_M / (2 * CELL);
        const dzdy = (yp - ym) * ELEV_SCALE_M / (2 * CELL);
        this.slope[this.idx(cx, cy)] = Math.hypot(dzdx, dzdy);
      }
    }
  }

  // Largest connected component of land ∪ river (roads can BRIDGE a river,
  // so both banks belong to the same buildable world; only ocean separates).
  // Customers/roads are restricted to land cells of this component, so
  // nothing gets stranded on an island or ocean-locked pocket.
  _floodMainland() {
    const n = this.n;
    const comp = new Int32Array(n * n).fill(-1);
    let best = -1, bestSize = 0, nComp = 0;
    const stack = [];
    for (let s = 0; s < n * n; s++) {
      if (this.water[s] === 1 || comp[s] !== -1) continue;
      let size = 0;
      stack.push(s); comp[s] = nComp;
      while (stack.length) {
        const i = stack.pop();
        if (this.water[i] === 0) size++; // component size = land cells only
        const cx = i % n, cy = (i / n) | 0;
        const nb = [[cx - 1, cy], [cx + 1, cy], [cx, cy - 1], [cx, cy + 1]];
        for (const [ax, ay] of nb) {
          if (!this.inGrid(ax, ay)) continue;
          const j = this.idx(ax, ay);
          if (this.water[j] !== 1 && comp[j] === -1) { comp[j] = nComp; stack.push(j); }
        }
      }
      if (size > bestSize) { bestSize = size; best = nComp; }
      nComp++;
    }
    for (let i = 0; i < n * n; i++) {
      this.mainland[i] = comp[i] === best && this.water[i] === 0 ? 1 : 0;
    }
  }

  // Straight-line distance (m) to the nearest river path point.
  riverDistAt(x, y) {
    const rp = this.riverPath;
    if (rp.length === 0) return Infinity;
    let bd = Infinity;
    for (let i = 0; i < rp.length; i += 3) {
      const d = (rp[i].x - x) ** 2 + (rp[i].y - y) ** 2;
      if (d < bd) bd = d;
    }
    return Math.sqrt(bd);
  }

  // Which bank of the river a point is on: +1 / -1 (sign of the cross
  // product against the nearest river segment). Used to seed towns on
  // both banks so bridges actually happen.
  riverSide(x, y) {
    const rp = this.riverPath;
    if (rp.length < 2) return 0;
    let bi = 0, bd = Infinity;
    for (let i = 0; i < rp.length - 1; i += 3) {
      const d = (rp[i].x - x) ** 2 + (rp[i].y - y) ** 2;
      if (d < bd) { bd = d; bi = i; }
    }
    const j = Math.min(rp.length - 1, bi + 3);
    const cross = (rp[j].x - rp[bi].x) * (y - rp[bi].y) -
                  (rp[j].y - rp[bi].y) * (x - rp[bi].x);
    return cross >= 0 ? 1 : -1;
  }

  waterAt(x, y) {
    const [cx, cy] = this.cellOf(x, y);
    return this.water[this.idx(cx, cy)];
  }
  slopeAt(x, y) {
    const [cx, cy] = this.cellOf(x, y);
    return this.slope[this.idx(cx, cy)];
  }
  buildableAt(x, y) {
    const [cx, cy] = this.cellOf(x, y);
    const i = this.idx(cx, cy);
    return this.water[i] === 0 && this.mainland[i] === 1 &&
      this.slope[i] < SLOPE_BUILD_MAX;
  }
  buildableCell(i) {
    return this.water[i] === 0 && this.mainland[i] === 1 &&
      this.slope[i] < SLOPE_BUILD_MAX;
  }

  // Exact grid traversal (Amanatides–Woo): returns true if pred(cellIdx)
  // holds for ANY cell the segment passes through. Used for all
  // segment-vs-water tests so detection can never miss a cell corner the
  // way point sampling can.
  segmentHits(x0, y0, x1, y1, pred) {
    const n = this.n;
    let cx = Math.min(n - 1, Math.max(0, Math.floor(x0 / CELL)));
    let cy = Math.min(n - 1, Math.max(0, Math.floor(y0 / CELL)));
    const tx = Math.min(n - 1, Math.max(0, Math.floor(x1 / CELL)));
    const ty = Math.min(n - 1, Math.max(0, Math.floor(y1 / CELL)));
    if (pred(cy * n + cx)) return true;
    const dx = x1 - x0, dy = y1 - y0;
    const stepX = dx > 0 ? 1 : -1, stepY = dy > 0 ? 1 : -1;
    let tMaxX = dx !== 0 ? ((cx + (dx > 0 ? 1 : 0)) * CELL - x0) / dx : Infinity;
    let tMaxY = dy !== 0 ? ((cy + (dy > 0 ? 1 : 0)) * CELL - y0) / dy : Infinity;
    const tDeltaX = dx !== 0 ? Math.abs(CELL / dx) : Infinity;
    const tDeltaY = dy !== 0 ? Math.abs(CELL / dy) : Infinity;
    let guard = 2 * n + 4;
    while ((cx !== tx || cy !== ty) && guard-- > 0) {
      if (tMaxX < tMaxY) { cx += stepX; tMaxX += tDeltaX; }
      else { cy += stepY; tMaxY += tDeltaY; }
      if (cx < 0 || cy < 0 || cx >= n || cy >= n) break;
      if (pred(cy * n + cx)) return true;
    }
    return false;
  }

  // Used to keep street segments and connectors out of the sea and river.
  segmentCrossesWater(x0, y0, x1, y1) {
    return this.segmentHits(x0, y0, x1, y1, (i) => this.water[i] !== 0);
  }

  segmentTouchesRiver(x0, y0, x1, y1) {
    return this.segmentHits(x0, y0, x1, y1, (i) => this.water[i] === 2);
  }

  // ASSUMPTION: road construction cost multiplier grows quadratically with
  // slope, capped at 9x — steep terrain forces detours rather than bans.
  slopeCostMul(i) {
    const s = this.slope[i] / 0.25;
    return 1 + Math.min(8, 6 * s * s);
  }
}

// Grid BFS distance (metres, 4-connected, not through ocean) from a seed
// cell set — shared by the density field (road distance) and siting code.
export function bfsDistanceM(terrain, seedCells) {
  const n = terrain.n;
  const dist = new Float32Array(n * n).fill(Infinity);
  let frontier = [];
  for (const c of seedCells) { dist[c] = 0; frontier.push(c); }
  while (frontier.length) {
    const next = [];
    for (const i of frontier) {
      const cx = i % n, cy = (i / n) | 0;
      for (const [ax, ay] of [[cx - 1, cy], [cx + 1, cy], [cx, cy - 1], [cx, cy + 1]]) {
        if (ax < 0 || ay < 0 || ax >= n || ay >= n) continue;
        const j = ay * n + ax;
        if (terrain.water[j] === 1) continue;
        if (dist[j] > dist[i] + CELL) { dist[j] = dist[i] + CELL; next.push(j); }
      }
    }
    frontier = next;
  }
  return dist;
}

export { SLOPE_BUILD_MAX, ELEV_SCALE_M };
