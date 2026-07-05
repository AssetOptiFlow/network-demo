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
//  - Settlements form an EXPLICIT three-tier hierarchy (customer targets
//    as shares of the sampled total, so at 50k customers): ONE large town
//    ≈ 40% (~20,000), each small town 2–4% (1,000–2,000), each rural
//    settlement 0.4–1% (200–500). Kernel weights are set so realised
//    counts hit these targets; the rural background gets the remainder.
//  - Town radius σ ∝ √customers (uniform peak density — area tracks
//    population, density doesn't).
//  - Minimum spacing scales with town size: 5 km + 1.6×(σ_i + σ_j).
//  - Towns are seeded on BOTH banks of the river when land allows.
//  - Density = per-town kernels + rural background that decays with
//    road distance (sparse rural load ALONG ROADS), all × fBm noise from
//    the same noise family as the terrain, zeroed off buildable land.

import { fbm01 } from "./noise.js";
import { CELL, GRID_NX, GRID_NY, MAP_W, MAP_H, bfsDistanceM } from "./terrain.js";
import { GridRouter } from "./roads.js";

// Three-tier hierarchy: customer targets as SHARES of the sampled total.
export const BIG_SHARE = 0.40;          // the one dominant town (~20k at 50k)
export const SMALL_SHARE = [0.02, 0.04];   // per small town (1,000–2,000 at 50k)
export const SETT_SHARE = [0.004, 0.01];   // per rural settlement (200–500 at 50k)
const N_SETTLEMENTS_MIN = 6, N_SETTLEMENTS_MAX = 10;

// σ ∝ √customers, calibrated so a 20,000-customer town has σ ≈ 2 km
// (peak density ≈ 500 customers/km² in every tier).
const SIGMA_REF = 2000, SIGMA_REF_CUST = 20000;
const SIGMA_MIN = 220;
const sigmaOfCust = (cust) =>
  Math.max(SIGMA_MIN, SIGMA_REF * Math.sqrt(cust / SIGMA_REF_CUST));

const SEP_BASE = 5000;    // min spacing = SEP_BASE + SEP_SIGMA*(σi+σj)
const SEP_SIGMA = 1.6;

const TOWN_NAMES = [
  "Waimotu", "Kereru Flat", "Tōtara Bay", "Pahiwi", "Ōkere",
  "Matai Junction", "Huringa", "Te Awa Iti", "Puketea", "Rata Gully",
  "Mānuka Flat", "Kōwhai Bend", "Te Rimu", "Awanui Ford", "Pīpiri",
  "Whero Downs", "Ngaio Corner", "Karaka Crossing", "Mahoe Landing",
  "Tūī Bush",
];

// nTowns = the number of SMALL towns (tier 1); the one large town and the
// seeded 6–10 rural settlements are added around them.
export function seedTowns(terrain, rng, nTowns, inlandWeight = 0.25, nCust = 50000) {
  const w = Math.max(0, Math.min(1, inlandWeight));
  const r = rng.fork("townsites");
  const rp = terrain.riverPath;
  const mouth = rp.length ? rp[rp.length - 1] : null;

  // Candidate pool (terrain-only scores; corridor terms added later) —
  // sized for the 100 km map so good sites are still densely sampled.
  const cand = [];
  for (let k = 0; k < 4000; k++) {
    const x = r.range(1500, MAP_W - 1500);
    const y = r.range(1500, MAP_H - 1500);
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

  // Explicit tier targets: one large town, nTowns small towns, then a
  // seeded handful of rural settlements. Customer counts, not ranks.
  const rSize = rng.fork("townsize");
  const targets = [{ tier: 0, cust: BIG_SHARE * nCust * rSize.range(0.95, 1.05) }];
  for (let i = 0; i < nTowns; i++) {
    targets.push({ tier: 1, cust: rSize.range(SMALL_SHARE[0], SMALL_SHARE[1]) * nCust });
  }
  const nSett = N_SETTLEMENTS_MIN +
    rng.fork("nsett").int(0, N_SETTLEMENTS_MAX - N_SETTLEMENTS_MIN);
  for (let i = 0; i < nSett; i++) {
    targets.push({ tier: 2, cust: rSize.range(SETT_SHARE[0], SETT_SHARE[1]) * nCust });
  }

  const towns = [];
  const farEnough = (c, sigma) => towns.every(t =>
    Math.hypot(t.x - c.x, t.y - c.y) >= SEP_BASE + SEP_SIGMA * (t.sigma + sigma));
  const pickFrom = (pool, sigma) => pool.find(c => farEnough(c, sigma)) ?? null;
  const push = (c, tgt) => {
    const rank = towns.length;
    towns.push({
      x: c.x, y: c.y, side: c.side,
      tier: tgt.tier, cust: tgt.cust,
      pop: tgt.cust / (BIG_SHARE * nCust), // relative size for street spacing
      sigma: sigmaOfCust(tgt.cust),
      weight: 1.0, // absolute kernel weight, set by buildDensityGrid
      name: TOWN_NAMES[rank % TOWN_NAMES.length],
      theta: 0, // grid axis, set by the road layer
    });
  };

  // ---- Phase A: two anchors from terrain-only scores — the large town
  // and the first small town (other bank forced when land allows, so
  // river towns straddle).
  const a1 = pickFrom(cand, sigmaOfCust(targets[0].cust));
  if (a1) push(a1, targets[0]);
  if (targets.length > 1 && towns.length) {
    const other = cand.filter(c => c.side !== towns[0].side);
    const s1 = sigmaOfCust(targets[1].cust);
    const a2 = pickFrom(other, s1) ?? pickFrom(cand, s1);
    if (a2) push(a2, targets[1]);
  }

  // ---- Phase B: provisional corridor skeleton between anchors and
  // map-edge exits; junctions of these corridors attract later towns.
  const corridors = buildCorridorSkeleton(terrain, towns);

  // ---- Phase C: remaining small towns then settlements, rescored with
  // corridor terms (service towns grow where routes meet).
  const corDist = corridors.corridorDistM, juncDist = corridors.junctionDistM;
  for (const c of cand) {
    const [cx, cy] = terrain.cellOf(c.x, c.y);
    const i = cy * GRID_NX + cx;
    c.score = c.base +
      0.55 * Math.exp(-(corDist[i] ?? 1e9) / 1500) +
      1.1 * Math.exp(-(juncDist[i] ?? 1e9) / 2500);
  }
  cand.sort((a, b) => b.score - a.score);
  while (towns.length < targets.length) {
    const tgt = targets[towns.length];
    const sigma = sigmaOfCust(tgt.cust);
    const sides = new Set(towns.map(t => t.side));
    let c = null;
    // keep both banks settled while placing the small towns
    if (tgt.tier === 1 && towns.length >= 1 && sides.size === 1) {
      c = pickFrom(cand.filter(k => k.side !== towns[0].side), sigma);
    }
    if (!c) c = pickFrom(cand, sigma);
    if (!c) break;
    push(c, tgt);
  }
  return { towns, corridors };
}

// Least-cost provisional corridors: anchor↔anchor plus each anchor to the
// nearest viable exit cell on every non-coast map edge. These paths later
// seed the arterial layer; their intersections are "route junctions".
function buildCorridorSkeleton(terrain, anchors) {
  const router = new GridRouter(terrain);
  const nx = GRID_NX, ny = GRID_NY;
  const paths = [];

  // Exit points: for each non-coast edge, the mainland border cell nearest
  // the edge midpoint (searched outward from the centre). W/E edges run
  // along y (length ny), S/N edges along x (length nx).
  const exits = [];
  for (let e = 0; e < 4; e++) {
    if (e === terrain.coastEdge) continue;
    const len = e <= 1 ? ny : nx;
    for (let off = 0; off < len / 2; off++) {
      let found = null;
      for (const s of [-1, 1]) {
        const k = Math.floor(len / 2) + s * off;
        if (k < 0 || k >= len) continue;
        const [cx, cy] = e === 0 ? [0, k] : e === 1 ? [nx - 1, k] : e === 2 ? [k, 0] : [k, ny - 1];
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

// Town density kernel: Gaussian core + exponential shoulder, so density
// eases from town grid through a peri-urban fringe into rural — no cliff
// at the town edge. Shared by the mass-compensation integral and the
// field build so compensation stays exact.
const KERNEL_SHOULDER = 0.28;   // share of peak carried by the fringe
const KERNEL_TAIL_SIGMA = 1.7;  // shoulder e-folding length, in σ
function townKernel(r2, sigma) {
  const core = Math.exp(-r2 / (2 * sigma * sigma));
  const tail = Math.exp(-Math.sqrt(r2) / (KERNEL_TAIL_SIGMA * sigma));
  return (1 - KERNEL_SHOULDER) * core + KERNEL_SHOULDER * tail;
}
const KERNEL_R_SIGMA = 6; // integration / evaluation radius (σ) for the tail

// Density field — built AFTER roads so rural load can hug them.
// Field masses are set in CUSTOMER units: each town kernel integrates to
// its tier target, and the rural background is scaled to the remainder
// (nCust − Σ town targets), so realised counts track the hierarchy. The
// finished grid is then normalised so the densest town core sits near 1.0
// (the underground-cable threshold in network.js reads those units).
export function buildDensityGrid(terrain, towns, roadDistM, rng, nCust = 50000) {
  const nx = GRID_NX, ny = GRID_NY;
  const seed = Math.floor(rng.fork("density-noise").float() * 1e9);
  const grid = new Float32Array(nx * ny);
  let maxD = 0;

  // Mass compensation: a town kernel clipped by sea/steep land loses
  // customers. Integrate each kernel over BUILDABLE cells and set its
  // weight so realised mass = the tier's customer target (a hemmed-in
  // town gets denser, not smaller — very Wellington).
  const rawW = towns.map(t => {
    let I = 0;
    const R = Math.ceil(KERNEL_R_SIGMA * t.sigma / CELL);
    const [tcx, tcy] = terrain.cellOf(t.x, t.y);
    for (let cy = Math.max(0, tcy - R); cy <= Math.min(ny - 1, tcy + R); cy++) {
      for (let cx = Math.max(0, tcx - R); cx <= Math.min(nx - 1, tcx + R); cx++) {
        const i = cy * nx + cx;
        if (!terrain.buildableCell(i)) continue;
        const [x, y] = terrain.cellCentre(cx, cy);
        I += townKernel((x - t.x) ** 2 + (y - t.y) ** 2, t.sigma);
      }
    }
    return t.cust / Math.max(1e-6, I);
  });
  towns.forEach((t, i) => { t.weight = rawW[i]; });
  const peakW = Math.max(1e-6, ...rawW);

  // Rural background pass 1: raw roadside + off-road-shoulder values, so
  // the whole rural field can be scaled to its customer remainder.
  const ruralRaw = new Float32Array(nx * ny);
  let ruralSum = 0;
  for (let i = 0; i < nx * ny; i++) {
    if (!terrain.buildableCell(i)) continue;
    // Sparse rural load along roads (decays with road distance) plus an
    // off-road SHOULDER: farms a few km past the end of the road, reached
    // by cross-country line easements. Hard zero beyond 6 km so nobody
    // settles the trackless back country (which would drag zone-sub
    // centroids — and easement chains — deep into roadless land).
    const rd = roadDistM ? roadDistM[i] : 0;
    const rdm = isFinite(rd) ? rd : 1e9;
    const v = 0.020 * Math.exp(-rdm / 1000) +
      (rdm < 6000 ? 0.0030 * Math.exp(-rdm / 1800) : 0);
    ruralRaw[i] = v;
    ruralSum += v;
  }
  const townTotal = towns.reduce((s, t) => s + t.cust, 0);
  const ruralScale = Math.max(0, nCust - townTotal) / Math.max(1e-9, ruralSum);

  const NOISE_WL = 30000 / 4.2; // metres — size- and aspect-independent
  for (let cy = 0; cy < ny; cy++) {
    for (let cx = 0; cx < nx; cx++) {
      const i = cy * nx + cx;
      if (!terrain.buildableCell(i)) continue;
      const [x, y] = terrain.cellCentre(cx, cy);
      let d = ruralScale * ruralRaw[i];
      for (const t of towns) {
        const r2 = (x - t.x) ** 2 + (y - t.y) ** 2;
        if (r2 < (KERNEL_R_SIGMA * t.sigma) ** 2) d += t.weight * townKernel(r2, t.sigma);
      }
      d /= peakW; // normalise: densest town core ≈ 1.0 (UG threshold units)
      d *= 0.55 + 0.9 * fbm01(x / NOISE_WL, y / NOISE_WL, seed, 4);
      grid[i] = d;
      if (d > maxD) maxD = d;
    }
  }
  return { grid, maxD };
}
