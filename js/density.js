// density.js — seed towns + Gaussian falloff + noise → customer density.
//
// ASSUMPTIONS:
//  - Towns are seeded at buildable sites scored for flatness plus a blend
//    of coastal proximity and river proximity, with a minimum 4 km
//    separation. The "inland weighting" slider (0–100%) shifts the blend:
//    0 = classic coastal towns, 100 = flat river-valley towns (roads
//    follow later either way, so inland towns still get arterials).
//  - Density = sum of per-town Gaussians, modulated by fBm noise from the
//    SAME noise family as the terrain (different fork), zeroed off
//    buildable land. Purely relative units.

import { fbm01 } from "./noise.js";
import { GRID_N, MAP_SIZE } from "./terrain.js";

export function seedTowns(terrain, rng, nTowns, inlandWeight = 0.25) {
  const cand = [];
  const r = rng.fork("townsites");
  const w = Math.max(0, Math.min(1, inlandWeight));
  for (let k = 0; k < 900; k++) {
    const x = r.range(1500, MAP_SIZE - 1500);
    const y = r.range(1500, MAP_SIZE - 1500);
    if (!terrain.buildableAt(x, y)) continue;
    const [cx, cy] = terrain.cellOf(x, y);
    const slope = terrain.slope[terrain.idx(cx, cy)];
    const coast = terrain._coastDist(cx, cy); // 0 at coast
    const flat = 1 - Math.min(1, slope / 0.35);
    const coastBonus = Math.exp(-coast * 4.5);
    const riverBonus = Math.exp(-terrain.riverDistAt(x, y) / 2500);
    const score = flat * 1.1 +
      ((1 - w) * coastBonus + w * (riverBonus + flat * 0.4)) * 1.5 +
      r.float() * 0.5;
    cand.push({ x, y, score });
  }
  cand.sort((a, b) => b.score - a.score);
  for (const c of cand) c.side = terrain.riverSide(c.x, c.y);
  const towns = [];
  const rSize = rng.fork("townsize");
  // ASSUMPTION: towns are seeded on BOTH banks of the river when land
  // allows (river towns straddle rivers in reality) — this is also what
  // forces the road network to bridge.
  const pickFrom = (pool) => {
    for (const c of pool) {
      if (towns.some(t => Math.hypot(t.x - c.x, t.y - c.y) < 4000)) continue;
      return c;
    }
    return null;
  };
  while (towns.length < nTowns) {
    const sidesCovered = new Set(towns.map(t => t.side));
    let c = null;
    if (towns.length >= 1 && towns.length < nTowns && sidesCovered.size === 1) {
      // no town across the river yet — take the best candidate over there
      c = pickFrom(cand.filter(k => k.side !== towns[0].side));
    }
    if (!c) c = pickFrom(cand);
    if (!c) break;
    const rank = towns.length;
    towns.push({
      x: c.x, y: c.y, side: c.side,
      sigma: (rank === 0 ? 2300 : rank === 1 ? 1600 : 1150) * rSize.range(0.85, 1.2),
      weight: (rank === 0 ? 1.0 : rank === 1 ? 0.55 : 0.3) * rSize.range(0.8, 1.2),
      name: TOWN_NAMES[rank % TOWN_NAMES.length],
    });
  }
  return towns;
}

const TOWN_NAMES = [
  "Waimotu", "Kereru Flat", "Tōtara Bay", "Pahiwi", "Ōkere",
  "Matai Junction", "Huringa", "Te Awa Iti",
];

export function buildDensityGrid(terrain, towns, rng) {
  const n = GRID_N;
  const seed = Math.floor(rng.fork("density-noise").float() * 1e9);
  const grid = new Float32Array(n * n);
  let maxD = 0;
  for (let cy = 0; cy < n; cy++) {
    for (let cx = 0; cx < n; cx++) {
      const i = cy * n + cx;
      if (!terrain.buildableCell(i)) continue;
      const [x, y] = terrain.cellCentre(cx, cy);
      let d = 0.012; // faint rural background so remote customers exist
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
