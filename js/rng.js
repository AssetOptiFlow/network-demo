// rng.js — seeded, deterministic PRNG utilities.
// No Math.random() anywhere in this project: every stage forks its own
// named stream from the master seed, so e.g. changing customer count
// does not perturb terrain.

export function hashString(str) {
  // FNV-1a 32-bit
  let h = 0x811c9dc5;
  for (let i = 0; i < str.length; i++) {
    h ^= str.charCodeAt(i);
    h = Math.imul(h, 0x01000193);
  }
  return h >>> 0;
}

export function mulberry32(a) {
  return function () {
    a |= 0;
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export class RNG {
  constructor(seedString) {
    this.seedString = String(seedString);
    this.next = mulberry32(hashString(this.seedString));
  }
  // Independent child stream — deterministic per (seed, label).
  fork(label) {
    return new RNG(this.seedString + "/" + label);
  }
  float() { return this.next(); }
  range(lo, hi) { return lo + (hi - lo) * this.next(); }
  int(lo, hi) { // inclusive
    return lo + Math.floor(this.next() * (hi - lo + 1));
  }
  pick(arr) { return arr[Math.floor(this.next() * arr.length)]; }
  shuffle(arr) {
    for (let i = arr.length - 1; i > 0; i--) {
      const j = Math.floor(this.next() * (i + 1));
      [arr[i], arr[j]] = [arr[j], arr[i]];
    }
    return arr;
  }
}

// 2D integer hash → [0,1), used by the noise lattice.
export function hash2(ix, iy, seed) {
  let h = seed >>> 0;
  h = Math.imul(h ^ ix, 0x27d4eb2f);
  h = Math.imul(h ^ iy, 0x165667b1);
  h ^= h >>> 15;
  h = Math.imul(h, 0x85ebca6b);
  h ^= h >>> 13;
  return (h >>> 0) / 4294967296;
}
