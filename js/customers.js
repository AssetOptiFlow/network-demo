// customers.js — the LOAD layer: weighted sampling of customer sites from
// the density field (which already encodes settlement Gaussians and
// roadside rural decay), then snapping to the finished road graph.
// STRICT LAYER ORDER: load depends on terrain + settlements + roads.
//
// ASSUMPTIONS:
//  - Each customer is an independent point (ICP) drawn from the density
//    field; no parcels/streets-level realism.
//  - Customers only ever land on buildable cells of the main landmass
//    (never in water, never on slopes ≥ 0.35 m/m).

import { CELL, GRID_NX, GRID_NY, MAP_MAX } from "./terrain.js";

export function sampleCustomers(terrain, density, graph, nCust, rng) {
  const nx = GRID_NX, ny = GRID_NY;
  const { grid } = density;
  // Cumulative distribution over cells.
  const cum = new Float64Array(nx * ny);
  let total = 0;
  for (let i = 0; i < nx * ny; i++) {
    total += grid[i];
    cum[i] = total;
  }
  const r = rng.fork("customers");
  const customers = [];
  let guard = 0;
  while (customers.length < nCust && guard++ < nCust * 30) {
    const target = r.float() * total;
    let lo = 0, hi = nx * ny - 1;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (cum[mid] < target) lo = mid + 1; else hi = mid;
    }
    const cx = lo % nx, cy = (lo / nx) | 0;
    const x = (cx + r.float()) * CELL;
    const y = (cy + r.float()) * CELL;
    if (!terrain.buildableAt(x, y)) continue;
    customers.push({ x, y, density: grid[lo], node: -1, tx: -1 });
  }
  // Snap each customer to its nearest road node (service point).
  let maxSnap = 0, sumSnap = 0;
  for (const c of customers) {
    const near = graph.nearestNode(c.x, c.y, MAP_MAX);
    c.node = near.id;
    maxSnap = Math.max(maxSnap, near.dist);
    sumSnap += near.dist;
  }
  return {
    customers,
    snapStats: { max: maxSnap, mean: sumSnap / Math.max(1, customers.length) },
  };
}
