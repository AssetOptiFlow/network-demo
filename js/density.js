// density.js — settlements (Zipf rank-size, corridor-aware placement) and
// the customer-density field. STRICT LAYER ORDER: settlements depend only
// on terrain + a provisional corridor skeleton routed here; the density
// field additionally depends on the finished road graph (roadside rural
// load) and is built AFTER roads.
//
// ASSUMPTIONS:
//  - Settlement sites are scored on flatness, a coastal↔river blend set by
//    the Inland slider, river-mouth proximity, and proximity to junctions
//    of a PROVISIONAL corridor skeleton (least-cost routes between the two
//    anchor towns and map-edge exits — the same routes later seed the
//    arterials, so no layer is placed independently).
//  - Town populations follow a Zipf rank-size law: pop_r ∝ 1/(r+1)^1.05 —
//    one dominant town, 2–3 medium, the rest small. Town radius σ ∝ √pop
//    (uniform peak density), so realised customer counts track population.
//  - Minimum spacing scales with town size: 2.2 km + 1.6×(σ_i + σ_j).
//  - Towns are seeded on BOTH banks of the river when land allows.
//  - Density = per-town Gaussians + rural background that decays with
//    road distance (sparse rural load ALONG ROADS), all × fBm noise from
//    the same noise family as the terrain, zeroed off buildable land.

import { fbm01 } from "./noise.js";
import { CELL, GRID_N, MAP_SIZE, bfsDistanceM } from "./terrain.js";
import { GridRouter } from "./roads.js";

// α = 1.25 keeps REALISED size ratios comfortably Zipf (prescribed
// largest/median ≈ 4, largest/second ≈ 2.4) even after clipping noise —
// the validation threshold is largest/median ≥ 3.
const ZIPF_ALPHA = 1.25;
const SIGMA_MAX = 2300;   // dominant town radius (m)
const SIGMA_MIN = 850;
const SEP_BASE = 2200;    // min spacing = SEP_BASE + SEP_SIGMA*(σi+σj)
const SEP_SIGMA = 1.6;

const TOWN_NAMES = [
  "Waimotu", "Kereru Flat", "Tōtara Bay", "Pahiwi", "Ōkere",
  "Matai Junction", "Huringa", "Te Awa Iti",
];

export function seedTowns(terrain, rng, nTowns, inlandWeight = 0.25) {
  const w = Math.max(0, Math.min(1, inlandWeight));
  const r = rng.fork("townsites");
  const rp = terrain.riverPath;
  const mouth = rp.length ? rp[rp.length - 1] : null;

  // Candidate pool (terrain-only scores; corridor terms added later).
  const cand = [];
  for (let k = 0; k < 1200; k++) {
    const x = r.range(1500, MAP_SIZE - 1500);
    const y = r.range(1500, MAP_SIZE - 1500);
    if (!terrain.buildableAt(x, y)) continue;
    const [cx, cy] = terrain.cellOf(x, y);
    const slope = terrain.slope[terrain.idx(cx, cy)];
    const flat = 1 - Math.min(1, slope / 0.35);
    const coastBonus = Math.exp(-terrain._coastDist(cx, cy) * 4.5);
    const riverBonus = Math.exp(-terrain.riverDistAt(x, y) / 2500);
    const mouthBonus = mouth ? Math.exp(-Math.hypot(x - mouth.x, y - mouth.y) / 3500) : 0;
    const base = flat * 1.0 +
      ((1 - w) * coastBonus + w * (riverBonus + flat * 0.4)) * 1.5 +
      mouthBonus * 0.9 + r.float() * 0.4;
    cand.push({ x, y, base, score: base, side: terrain.riverSide(x, y) });
  }
  cand.sort((a, b) => b.score - a.score);

  // Zipf populations (pop_0 = 1 for the dominant town).
  const pops = [];
  for (let i = 0; i < nTowns; i++) pops.push(1 / Math.pow(i + 1, ZIPF_ALPHA));
  const sigmaOf = (pop) => Math.max(SIGMA_MIN, SIGMA_MAX * Math.sqrt(pop));

  const towns = [];
  const rSize = rng.fork("townsize");
  const farEnough = (c, sigma) => towns.every(t =>
    Math.hypot(t.x - c.x, t.y - c.y) >= SEP_BASE + SEP_SIGMA * (t.sigma + sigma));
  const pickFrom = (pool, sigma) => pool.find(c => farEnough(c, sigma)) ?? null;
  const push = (c) => {
    const rank = towns.length;
    const pop = pops[rank] * rSize.range(0.9, 1.1);
    towns.push({
      x: c.x, y: c.y, side: c.side,
      pop, sigma: sigmaOf(pop),
      weight: 1.0 * rSize.range(0.9, 1.1), // uniform peak density; area ∝ pop
      name: TOWN_NAMES[rank % TOWN_NAMES.length],
      theta: 0, // grid axis, set by the road layer
    });
  };

  // ---- Phase A: two anchor towns from terrain-only scores (other bank
  // forced for the second when land allows — river towns straddle).
  const a1 = pickFrom(cand, sigmaOf(pops[0]));
  if (a1) push(a1);
  if (nTowns > 1 && towns.length) {
    const other = cand.filter(c => c.side !== towns[0].side);
    const a2 = pickFrom(other, sigmaOf(pops[1])) ?? pickFrom(cand, sigmaOf(pops[1]));
    if (a2) push(a2);
  }

  // ---- Phase B: provisional corridor skeleton between anchors and
  // map-edge exits; junctions of these corridors attract later towns.
  const corridors = buildCorridorSkeleton(terrain, towns);

  // ---- Phase C: remaining towns, rescored with corridor terms.
  const corDist = corridors.corridorDistM, juncDist = corridors.junctionDistM;
  const n = GRID_N;
  for (const c of cand) {
    const [cx, cy] = terrain.cellOf(c.x, c.y);
    const i = cy * n + cx;
    c.score = c.base +
      0.55 * Math.exp(-(corDist[i] ?? 1e9) / 1500) +
      1.1 * Math.exp(-(juncDist[i] ?? 1e9) / 2500);
  }
  cand.sort((a, b) => b.score - a.score);
  while (towns.length < nTowns) {
    const sigma = sigmaOf(pops[towns.length]);
    const sides = new Set(towns.map(t => t.side));
    let c = null;
    if (towns.length >= 1 && sides.size === 1) {
      c = pickFrom(cand.filter(k => k.side !== towns[0].side), sigma);
    }
    if (!c) c = pickFrom(cand, sigma);
    if (!c) break;
    push(c);
  }
  return { towns, corridors };
}

// Least-cost provisional corridors: anchor↔anchor plus each anchor to the
// nearest viable exit cell on every non-coast map edge. These paths later
// seed the arterial layer; their intersections are "route junctions".
function buildCorridorSkeleton(terrain, anchors) {
  const router = new GridRouter(terrain);
  const n = GRID_N;
  const paths = [];

  // Exit points: for each non-coast edge, the mainland border cell nearest
  // the edge midpoint (searched outward from the centre).
  const exits = [];
  for (let e = 0; e < 4; e++) {
    if (e === terrain.coastEdge) continue;
    for (let off = 0; off < n / 2; off++) {
      let found = null;
      for (const s of [-1, 1]) {
        const k = Math.floor(n / 2) + s * off;
        if (k < 0 || k >= n) continue;
        const [cx, cy] = e === 0 ? [0, k] : e === 1 ? [n - 1, k] : e === 2 ? [k, 0] : [k, n - 1];
        const i = terrain.idx(cx, cy);
        if (terrain.water[i] === 0 && terrain.mainland[i]) { found = terrain.cellCentre(cx, cy); break; }
      }
      if (found) { exits.push({ x: found[0], y: found[1] }); break; }
    }
  }

  const legs = [];
  if (anchors.length >= 2) legs.push([anchors[0], anchors[1]]);
  for (const a of anchors) for (const ex of exits) legs.push([a, ex]);
  for (const [from, to] of legs) {
    const cells = router.route(from.x, from.y, to.x, to.y);
    if (cells) paths.push(cells);
  }

  // Junctions: cells used by ≥2 distinct corridors.
  const count = new Map();
  for (const p of paths) {
    const seen = new Set(p);
    for (const c of seen) count.set(c, (count.get(c) ?? 0) + 1);
  }
  const corridorCells = [...count.keys()];
  const junctionCells = corridorCells.filter(c => count.get(c) >= 2);

  return {
    paths, exits, junctionCells,
    corridorDistM: bfsDistanceM(terrain, corridorCells),
    junctionDistM: bfsDistanceM(terrain, junctionCells),
  };
}

// Density field — built AFTER roads so rural load can hug them.
export function buildDensityGrid(terrain, towns, roadDistM, rng) {
  const n = GRID_N;
  const seed = Math.floor(rng.fork("density-noise").float() * 1e9);
  const grid = new Float32Array(n * n);
  let maxD = 0;

  // Mass compensation: a town Gaussian clipped by sea/steep land loses
  // customers, flattening the Zipf distribution. Integrate each Gaussian
  // over BUILDABLE cells and set its weight so realised mass ∝ population
  // (a hemmed-in town gets denser, not smaller — very Wellington).
  const rawW = towns.map(t => {
    let I = 0;
    const R = Math.ceil(3.5 * t.sigma / CELL);
    const [tcx, tcy] = terrain.cellOf(t.x, t.y);
    for (let cy = Math.max(0, tcy - R); cy <= Math.min(n - 1, tcy + R); cy++) {
      for (let cx = Math.max(0, tcx - R); cx <= Math.min(n - 1, tcx + R); cx++) {
        const i = cy * n + cx;
        if (!terrain.buildableCell(i)) continue;
        const [x, y] = terrain.cellCentre(cx, cy);
        I += Math.exp(-((x - t.x) ** 2 + (y - t.y) ** 2) / (2 * t.sigma * t.sigma));
      }
    }
    return t.pop / Math.max(1e-6, I);
  });
  const sortedW = rawW.slice().sort((a, b) => a - b);
  const ref = sortedW[Math.floor(sortedW.length / 2)] || 1;
  towns.forEach((t, i) => { t.weight = Math.min(2.5, rawW[i] / ref); });
  for (let cy = 0; cy < n; cy++) {
    for (let cx = 0; cx < n; cx++) {
      const i = cy * n + cx;
      if (!terrain.buildableCell(i)) continue;
      const [x, y] = terrain.cellCentre(cx, cy);
      // Sparse rural load along roads: decays with road distance.
      const rd = roadDistM ? roadDistM[i] : 0;
      let d = 0.022 * Math.exp(-(isFinite(rd) ? rd : 1e9) / 800);
      for (const t of towns) {
        const r2 = (x - t.x) ** 2 + (y - t.y) ** 2;
        d += t.weight * Math.exp(-r2 / (2 * t.sigma * t.sigma));
      }
      d *= 0.55 + 0.9 * fbm01(cx / n * 4.2, cy / n * 4.2, seed, 4);
      grid[i] = d;
      if (d > maxD) maxD = d;
    }
  }
  return { grid, maxD };
}
