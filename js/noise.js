// noise.js — seeded value noise + fractal Brownian motion.
// The SAME noise family drives terrain elevation and the density field
// (different forked seeds), as required by the brief.

import { hash2 } from "./rng.js";

function smooth(t) { return t * t * (3 - 2 * t); }

// Value noise at (x, y) in lattice units, seeded.
export function valueNoise(x, y, seed) {
  const ix = Math.floor(x), iy = Math.floor(y);
  const fx = x - ix, fy = y - iy;
  const a = hash2(ix, iy, seed);
  const b = hash2(ix + 1, iy, seed);
  const c = hash2(ix, iy + 1, seed);
  const d = hash2(ix + 1, iy + 1, seed);
  const u = smooth(fx), v = smooth(fy);
  return a + (b - a) * u + (c - a) * v + (a - b - c + d) * u * v; // [0,1)
}

// fBm in [-1, 1] (approximately).
export function fbm(x, y, seed, octaves = 5, lacunarity = 2.0, gain = 0.5) {
  let amp = 1, freq = 1, sum = 0, norm = 0;
  for (let o = 0; o < octaves; o++) {
    sum += amp * (valueNoise(x * freq, y * freq, seed + o * 1013) * 2 - 1);
    norm += amp;
    amp *= gain;
    freq *= lacunarity;
  }
  return sum / norm;
}

// fBm remapped to [0, 1].
export function fbm01(x, y, seed, octaves = 5) {
  return fbm(x, y, seed, octaves) * 0.5 + 0.5;
}
