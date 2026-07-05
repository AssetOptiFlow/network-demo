// subtx.js — subtransmission layer: GXP siting and least-cost routed
// lines GXP → zone subs, with trunk sharing and ≥1 inter-sub tie.
// STRICT LAYER ORDER: depends on terrain + roads + subs; built BEFORE
// feeders. It remains VISUAL ONLY — nothing electrical reads it, and a
// correctness check asserts SAIDI/network structure are unchanged by it.
//
// ASSUMPTIONS:
//  - GXP: a flat mainland map-edge cell near the load centroid, preferring
//    proximity to a road corridor (the national grid enters cheaply).
//  - Line routing: A* over the terrain grid; cost = distance × slope
//    multiplier, ocean impassable, river crossings 6× (towers span rivers
//    more easily than roads bridge them), 0.7× on road corridors, 0.35×
//    on cells already carrying subtransmission — so adjacent subs SHARE
//    trunk corridors before branching.
//  - Tie: one extra line between the closest pair of subs, routed with a
//    PENALTY on existing subtx cells so it takes an independent corridor
//    (a real security-of-supply tie, not a shadow of the trunk).

import { CELL, GRID_N, MAP_SIZE } from "./terrain.js";
import { hash2 } from "./rng.js";

const RIVER_MUL = 6;
const ROAD_CORRIDOR_MUL = 0.7;
const SHARED_TRUNK_MUL = 0.35;
const TIE_REUSE_PENALTY = 1.8;

export function buildSubtx(terrain, subs, loadCentroid, roadDistM) {
  const gxp = pickGxp(terrain, loadCentroid, roadDistM);
  if (!gxp || subs.length === 0) return { gxp, lines: [], maxStraightKm: 0 };

  const used = new Uint8Array(GRID_N * GRID_N);
  const lines = [];
  const order = subs.slice().sort((a, b) =>
    Math.hypot(a.x - gxp.x, a.y - gxp.y) - Math.hypot(b.x - gxp.x, b.y - gxp.y) || a.id - b.id);
  for (const sub of order) {
    const cells = route(terrain, roadDistM, used, gxp.x, gxp.y, sub.x, sub.y, false);
    if (!cells) continue;
    for (const c of cells) used[c] = 1;
    lines.push({ kind: "feed", sub: sub.id, pts: cellsToPts(terrain, cells, gxp, sub) });
  }
  // Inter-sub tie: closest pair, routed to AVOID the existing trunks.
  if (subs.length >= 2) {
    let bi = 0, bj = 1, bd = Infinity;
    for (let i = 0; i < subs.length; i++) {
      for (let j = i + 1; j < subs.length; j++) {
        const d = Math.hypot(subs[i].x - subs[j].x, subs[i].y - subs[j].y);
        if (d < bd) { bd = d; bi = i; bj = j; }
      }
    }
    const cells = route(terrain, roadDistM, used,
      subs[bi].x, subs[bi].y, subs[bj].x, subs[bj].y, true);
    if (cells) lines.push({ kind: "tie", sub: -1, pts: cellsToPts(terrain, cells, subs[bi], subs[bj]) });
  }
  return { gxp, lines, maxStraightKm: maxStraightKm(lines) };
}

// A plausible GXP: flat mainland cell on the map edge nearest the load
// centroid, preferring cells close to a road corridor.
function pickGxp(terrain, loadCentroid, roadDistM) {
  const n = terrain.n;
  let best = null, bestScore = Infinity;
  for (let k = 0; k < n; k++) {
    for (const [cx, cy] of [[k, 0], [k, n - 1], [0, k], [n - 1, k]]) {
      const i = terrain.idx(cx, cy);
      if (terrain.water[i] !== 0 || !terrain.mainland[i]) continue;
      const [x, y] = terrain.cellCentre(cx, cy);
      const rd = roadDistM ? roadDistM[i] : 0;
      const score = Math.hypot(x - loadCentroid.x, y - loadCentroid.y) +
        terrain.slope[i] * 40000 -
        6000 * Math.exp(-(isFinite(rd) ? rd : 1e9) / 600);
      if (score < bestScore) { bestScore = score; best = { x, y }; }
    }
  }
  return best;
}

// Least-cost A* over the terrain grid with the subtx cost profile.
function route(terrain, roadDistM, used, x0, y0, x1, y1, isTie) {
  const n = GRID_N;
  const [sx, sy] = terrain.cellOf(x0, y0);
  const [tx, ty] = terrain.cellOf(x1, y1);
  const start = sy * n + sx, goal = ty * n + tx;
  const g = new Float64Array(n * n).fill(Infinity);
  const came = new Int32Array(n * n).fill(-1);
  const heap = { f: [], v: [] };
  const push = (f, v) => {
    const F = heap.f, V = heap.v; let i = F.length;
    F.push(f); V.push(v);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (F[p] <= F[i]) break;
      [F[p], F[i]] = [F[i], F[p]]; [V[p], V[i]] = [V[i], V[p]]; i = p;
    }
  };
  const pop = () => {
    const F = heap.f, V = heap.v; const top = V[0];
    const lf = F.pop(), lv = V.pop();
    if (F.length) {
      F[0] = lf; V[0] = lv; let i = 0;
      for (;;) {
        const l = 2 * i + 1, r = l + 1; let m = i;
        if (l < F.length && F[l] < F[m]) m = l;
        if (r < F.length && F[r] < F[m]) m = r;
        if (m === i) break;
        [F[m], F[i]] = [F[i], F[m]]; [V[m], V[i]] = [V[i], V[m]]; i = m;
      }
    }
    return top;
  };
  const mul = (i) => {
    if (terrain.water[i] === 1) return -1;                      // ocean: blocked
    if (terrain.water[i] === 0 && !terrain.mainland[i]) return -1;
    let m = terrain.slopeCostMul(i);
    if (terrain.water[i] === 2) m *= RIVER_MUL;                 // tower span
    const rd = roadDistM ? roadDistM[i] : Infinity;
    if (isFinite(rd) && rd <= CELL) m *= ROAD_CORRIDOR_MUL;     // road corridor
    if (used[i]) m *= isTie ? TIE_REUSE_PENALTY : SHARED_TRUNK_MUL;
    // micro-siting variation: land access, pockets of bad ground — keeps
    // lines from running ruler-straight across long flat plains
    m *= 1 + 0.22 * hash2(i % GRID_N, (i / GRID_N) | 0, 7351);
    return m;
  };
  g[start] = 0;
  push(0, start);
  const closed = new Set();
  while (heap.f.length) {
    const cur = pop();
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
        const m = mul(ni);
        if (m < 0) continue;
        const ng = g[cur] + CELL * (dx && dy ? Math.SQRT2 : 1) * m;
        if (ng < g[ni]) {
          g[ni] = ng;
          came[ni] = cur;
          push(ng + Math.hypot(ax - tx, ay - ty) * CELL * 0.3, ni);
        }
      }
    }
  }
  if (!isFinite(g[goal])) return null;
  const cells = [];
  for (let c = goal; c !== -1; c = came[c]) {
    cells.push(c);
    if (c === start) break;
  }
  return cells.reverse();
}

function cellsToPts(terrain, cells, from, to) {
  const n = GRID_N;
  const pts = cells.map(c => terrain.cellCentre(c % n, (c / n) | 0));
  // light simplification for drawing; exact endpoints
  const out = [[from.x, from.y]];
  for (let i = 1; i < pts.length - 1; i += 2) out.push(pts[i]);
  out.push([to.x, to.y]);
  return out;
}

// Longest chord-hugging run (km): the longest stretch of a line whose
// intermediate vertices all sit within 150 m of the straight chord.
// Feeds the MAX_SUBTX_STRAIGHT_KM validation rule.
function maxStraightKm(lines) {
  let worst = 0;
  for (const line of lines) {
    let p = line.pts;
    // 100 km lines carry hundreds of vertices and this scan is cubic-ish;
    // subsampling to ~220 points keeps the 150 m chord test meaningful
    if (p.length > 220) {
      const step = Math.ceil(p.length / 220);
      p = p.filter((_, k) => k % step === 0 || k === p.length - 1);
    }
    for (let i = 0; i < p.length - 1; i++) {
      for (let j = p.length - 1; j > i; j--) {
        const [ax, ay] = p[i], [bx, by] = p[j];
        const len = Math.hypot(bx - ax, by - ay);
        if (len <= worst * 1000) break;
        let ok = true;
        for (let m = i + 1; m < j && ok; m++) {
          const t = Math.max(0, Math.min(1,
            ((p[m][0] - ax) * (bx - ax) + (p[m][1] - ay) * (by - ay)) / (len * len || 1)));
          const d = Math.hypot(p[m][0] - (ax + t * (bx - ax)), p[m][1] - (ay + t * (by - ay)));
          if (d > 150) ok = false;
        }
        if (ok) { worst = Math.max(worst, len / 1000); break; }
      }
    }
  }
  return +worst.toFixed(2);
}
