// render.js — canvas map renderer: terrain backdrop (offscreen, cached per
// regen), vector layers, pan/zoom, edge hit-testing, fault playback overlay.
// Colours follow the validated reference dataviz palette (fixed categorical
// order for feeders; reserved status colours for fault states).

import { GRID_N, MAP_SIZE } from "./terrain.js";

export const FEEDER_PALETTE = [
  "#2a78d6", "#1baf7a", "#eda100", "#008300",
  "#4a3aa7", "#e34948", "#e87ba4", "#eb6834",
];
export const STATUS = {
  good: "#0ca30c",      // restored
  warning: "#fab219",   // isolatable, waiting for switching
  serious: "#ec835a",   // downstream, waiting for repair
  critical: "#d03b3b",  // faulted section
};
export const feederColour = (fid) => FEEDER_PALETTE[fid % FEEDER_PALETTE.length];

export class Renderer {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.view = { scale: 0.02, ox: 0, oy: 0 }; // px per metre + pan offset px
    this.layers = {
      terrain: true, density: false, roads: true, customers: true,
      network: true, txs: false, switches: true, bridges: true,
      roadVsLine: false, heat: false, subtx: true, ties: true,
    };
    this._heatCache = null;
    this.world = null;
    this.selectedFeeder = -1;
    this.selectedEdge = -1;
    this.playback = null; // {scenario, tMin}
    this.switchList = [];   // placed sectionalisers (ordered)
    this.recloserList = []; // placed reclosers (ordered)
    this.debugBranch = null; // Set of tree edge ids with 2x fault rate
    this._terrainCache = null;
    this._densityCache = null;
  }

  setWorld(world) {
    this.world = world;
    this.selectedFeeder = -1;
    this.selectedEdge = -1;
    this.playback = null;
    this.switchList = [];
    this.recloserList = [];
    this._terrainCache = this._renderTerrainCache(world.terrain);
    this._densityCache = this._renderDensityCache(world);
    this._heatCache = null; // rebuilt via updateHeat once SAIDI is computed
    this.fit();
  }

  fit() {
    const { width, height } = this.canvas;
    const s = Math.min(width, height) / MAP_SIZE * 0.96;
    this.view.scale = s;
    this.view.ox = (width - MAP_SIZE * s) / 2;
    this.view.oy = (height - MAP_SIZE * s) / 2;
  }

  sx(x) { return x * this.view.scale + this.view.ox; }
  sy(y) { return y * this.view.scale + this.view.oy; }
  wx(px) { return (px - this.view.ox) / this.view.scale; }
  wy(py) { return (py - this.view.oy) / this.view.scale; }

  zoomAt(px, py, factor) {
    const wx = this.wx(px), wy = this.wy(py);
    this.view.scale = Math.min(1.2, Math.max(0.0015, this.view.scale * factor));
    this.view.ox = px - wx * this.view.scale;
    this.view.oy = py - wy * this.view.scale;
  }
  pan(dx, dy) { this.view.ox += dx; this.view.oy += dy; }

  _renderTerrainCache(t) {
    const n = GRID_N;
    const off = document.createElement("canvas");
    off.width = n; off.height = n;
    const c = off.getContext("2d");
    const img = c.createImageData(n, n);
    for (let i = 0; i < n * n; i++) {
      let r, g, b;
      if (t.water[i] === 1) {
        const d = Math.min(1, -t.elev[i] * 2.2);
        r = 122 - 45 * d; g = 166 - 48 * d; b = 205 - 30 * d; // sea, deeper = darker
      } else if (t.water[i] === 2) {
        r = 108; g = 158; b = 200; // river
      } else {
        const e = Math.max(0, t.elev[i]); // 0..~1.5
        const s = Math.min(1, t.slope[i] / 0.5);
        // hypsometric: pale green lowland → tan → grey-brown ridge
        const stops = [
          [0.00, [190, 208, 168]], [0.25, [206, 210, 160]],
          [0.55, [196, 182, 142]], [0.85, [168, 148, 122]],
          [1.20, [142, 128, 116]],
        ];
        let lo = stops[0], hi = stops[stops.length - 1];
        for (let k = 0; k < stops.length - 1; k++) {
          if (e >= stops[k][0] && e <= stops[k + 1][0]) { lo = stops[k]; hi = stops[k + 1]; break; }
        }
        const f = Math.min(1, Math.max(0, (e - lo[0]) / Math.max(1e-6, hi[0] - lo[0])));
        r = lo[1][0] + (hi[1][0] - lo[1][0]) * f;
        g = lo[1][1] + (hi[1][1] - lo[1][1]) * f;
        b = lo[1][2] + (hi[1][2] - lo[1][2]) * f;
        // hillshade-lite: darken by slope, brighten NW-facing
        const cx = i % n, cy = (i / n) | 0;
        const nwSlope = cx > 0 && cy > 0 ? (t.elev[i] - t.elev[i - n - 1]) : 0;
        const shade = 1 - s * 0.35 + nwSlope * 3.2;
        r *= shade; g *= shade; b *= shade;
      }
      img.data[i * 4] = Math.max(0, Math.min(255, r));
      img.data[i * 4 + 1] = Math.max(0, Math.min(255, g));
      img.data[i * 4 + 2] = Math.max(0, Math.min(255, b));
      img.data[i * 4 + 3] = 255;
    }
    c.putImageData(img, 0, 0);
    return off;
  }

  _renderDensityCache(world) {
    const n = GRID_N;
    const off = document.createElement("canvas");
    off.width = n; off.height = n;
    const c = off.getContext("2d");
    const img = c.createImageData(n, n);
    const { grid, maxD } = world.density;
    for (let i = 0; i < n * n; i++) {
      const v = Math.pow(grid[i] / maxD, 0.45);
      img.data[i * 4] = 42; img.data[i * 4 + 1] = 120; img.data[i * 4 + 2] = 214;
      img.data[i * 4 + 3] = Math.round(v * 190);
    }
    c.putImageData(img, 0, 0);
    return off;
  }

  draw() {
    const { ctx, canvas, world } = this;
    ctx.fillStyle = "#e8ecef";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    if (!world) return;
    const s = this.view.scale;
    const px = this.sx(0), py = this.sy(0), sz = MAP_SIZE * s;

    if (this.layers.terrain) {
      ctx.imageSmoothingEnabled = true;
      ctx.drawImage(this._terrainCache, px, py, sz, sz);
    }
    if (this.layers.density) ctx.drawImage(this._densityCache, px, py, sz, sz);
    if (this.layers.heat && this._heatCache) {
      ctx.imageSmoothingEnabled = false; // crisp cells for the heat layer
      ctx.drawImage(this._heatCache, px, py, sz, sz);
      ctx.imageSmoothingEnabled = true;
    }
    if (this.layers.roads) this._drawRoads();
    if (this.layers.customers) this._drawCustomers();
    if (this.layers.subtx) this._drawSubtx();
    if (this.layers.network) this._drawNetwork();
    if (this.layers.ties) this._drawTies();
    if (this.layers.roadVsLine) this._drawRoadVsLine();
    this._drawSubsAndTxs();
    if (this.layers.switches) this._drawSwitches();
    if (this.layers.bridges) this._drawBridges();
    if (this.playback) this._drawFaultMarker();
    this._drawTownLabels();
  }

  _drawRoads() {
    const { ctx, world } = this;
    const g = world.graph;
    const specs = [
      { cls: 2, w: 0.7, col: "rgba(150,148,140,0.55)" },
      { cls: 1, w: 1.1, col: "rgba(120,118,110,0.75)" },
      { cls: 0, w: 1.8, col: "rgba(82,81,78,0.9)" },
    ];
    for (const spec of specs) {
      ctx.strokeStyle = spec.col;
      ctx.lineWidth = spec.w * Math.max(1, this.view.scale * 40);
      ctx.beginPath();
      for (const e of g.edges) {
        if (e.cls !== spec.cls) continue;
        ctx.moveTo(this.sx(g.nx[e.a]), this.sy(g.ny[e.a]));
        ctx.lineTo(this.sx(g.nx[e.b]), this.sy(g.ny[e.b]));
      }
      ctx.stroke();
    }
  }

  _drawCustomers() {
    const { ctx, world } = this;
    ctx.fillStyle = "rgba(60,58,54,0.5)";
    const r = Math.max(0.6, this.view.scale * 25);
    for (const c of world.customers) {
      ctx.fillRect(this.sx(c.x) - r / 2, this.sy(c.y) - r / 2, r, r);
    }
  }

  _edgeClassDuringPlayback(teId, feeder) {
    // returns a colour or null (use feeder colour)
    const p = this.playback;
    if (!p) return null;
    const sc = p.scenario;
    if (feeder !== sc.feeder) return "rgba(137,135,129,0.35)"; // dimmed
    if (sc.outEdges.has(teId)) {
      if (p.tMin >= sc.tRepairDone) return STATUS.good;
      return teId === sc.teId ? STATUS.critical : STATUS.serious;
    }
    if (sc.tieEdges && sc.tieEdges.has(teId)) {
      // tripped but BACKFED from the neighbouring feeder via a tie
      return p.tMin >= (sc.tSwitch ?? Infinity) ? STATUS.good : STATUS.warning;
    }
    if (sc.zoneEdges.has(teId)) {
      // tripped but isolatable behind a sectionaliser
      if (sc.tSwitch === null) {
        return p.tMin >= sc.tRepairDone ? STATUS.good : STATUS.serious;
      }
      return p.tMin >= sc.tSwitch ? STATUS.good : STATUS.warning;
    }
    return null; // upstream of the recloser — never interrupted, base colour
  }

  _drawNetwork() {
    const { ctx, world } = this;
    const g = world.graph, net = world.net;
    const lw = Math.max(1.4, this.view.scale * 60);
    for (const f of net.feeders) {
      const base = feederColour(f.id);
      const sel = this.selectedFeeder === f.id;
      for (const teId of f.edges) {
        const te = net.treeEdges[teId];
        const over = this._edgeClassDuringPlayback(teId, f.id);
        ctx.strokeStyle = over ?? base;
        ctx.lineWidth = (sel ? lw * 1.7 : lw) * (over && over.startsWith("rgba") ? 0.8 : 1);
        ctx.setLineDash(this.debugBranch && this.debugBranch.has(teId) ? [7, 5]
          : te.underground ? [3, 3] : []);
        ctx.beginPath();
        ctx.moveTo(this.sx(g.nx[te.parentNode]), this.sy(g.ny[te.parentNode]));
        ctx.lineTo(this.sx(g.nx[te.node]), this.sy(g.ny[te.node]));
        ctx.stroke();
        if (teId === this.selectedEdge && !this.playback) {
          ctx.strokeStyle = "#0b0b0b";
          ctx.lineWidth = lw * 2.2;
          ctx.globalAlpha = 0.35;
          ctx.stroke();
          ctx.globalAlpha = 1;
        }
      }
    }
    ctx.setLineDash([]);
  }

  _drawRoadVsLine() {
    // For the selected edge (or feeder root if none): straight chord vs the
    // actual tree path back to the sub.
    const { ctx, world } = this;
    if (this.selectedEdge < 0) return;
    const net = world.net, g = world.graph;
    const te = net.treeEdges[this.selectedEdge];
    const f = net.feeders.find(x => x.id === te.feeder);
    const sub = net.subs[f.sub];
    // road path: walk parents
    ctx.strokeStyle = "#0b0b0b";
    ctx.lineWidth = Math.max(2, this.view.scale * 80);
    ctx.setLineDash([]);
    ctx.beginPath();
    let cur = this.selectedEdge;
    ctx.moveTo(this.sx(g.nx[te.node]), this.sy(g.ny[te.node]));
    while (cur !== -1) {
      const e = net.treeEdges[cur];
      ctx.lineTo(this.sx(g.nx[e.parentNode]), this.sy(g.ny[e.parentNode]));
      cur = net.treeEdgeOfNode[e.parentNode];
    }
    ctx.stroke();
    // straight chord
    ctx.setLineDash([6, 5]);
    ctx.strokeStyle = "#d03b3b";
    ctx.beginPath();
    ctx.moveTo(this.sx(sub.x), this.sy(sub.y));
    ctx.lineTo(this.sx(g.nx[te.node]), this.sy(g.ny[te.node]));
    ctx.stroke();
    ctx.setLineDash([]);
  }

  _drawSubsAndTxs() {
    const { ctx, world } = this;
    const net = world.net, g = world.graph;
    if (this.layers.txs) {
      for (const tx of net.txs) {
        if (tx.node < 0) continue;
        const col = tx.feeder >= 0 ? feederColour(tx.feeder) : "#898781";
        ctx.fillStyle = col;
        const r = Math.max(1.6, this.view.scale * 50);
        ctx.beginPath();
        ctx.arc(this.sx(g.nx[tx.node]), this.sy(g.ny[tx.node]), r, 0, Math.PI * 2);
        ctx.fill();
      }
    }
    for (const sub of net.subs) {
      const r = Math.max(5, this.view.scale * 160);
      ctx.fillStyle = "#0b0b0b";
      ctx.fillRect(this.sx(sub.x) - r, this.sy(sub.y) - r, r * 2, r * 2);
      ctx.fillStyle = "#fcfcfb";
      ctx.fillRect(this.sx(sub.x) - r * 0.55, this.sy(sub.y) - r * 0.55, r * 1.1, r * 1.1);
    }
  }

  _drawSwitches() {
    const { ctx, world } = this;
    const net = world.net, g = world.graph;
    // devices sit at the top (parent end) of their edge
    const devicePos = (te) => [
      this.sx(g.nx[te.parentNode] * 0.75 + g.nx[te.node] * 0.25),
      this.sy(g.ny[te.parentNode] * 0.75 + g.ny[te.node] * 0.25),
    ];
    for (const teId of this.switchList) {
      const te = net.treeEdges[teId];
      const [x, y] = devicePos(te);
      const r = Math.max(3.4, this.view.scale * 110);
      const isOpen = this.playback && this.playback.scenario.switchEdge === teId &&
        this.playback.tMin >= (this.playback.scenario.tSwitch ?? Infinity);
      ctx.fillStyle = isOpen ? STATUS.warning : "#fcfcfb";
      ctx.strokeStyle = feederColour(te.feeder);
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.rect(x - r / 2, y - r / 2, r, r);
      ctx.fill(); ctx.stroke();
    }
    for (const teId of this.recloserList) {
      const te = net.treeEdges[teId];
      const [x, y] = devicePos(te);
      const r = Math.max(4, this.view.scale * 130);
      const tripped = this.playback && this.playback.scenario.recloserEdge === teId &&
        this.playback.tMin < this.playback.scenario.tRepairDone;
      ctx.fillStyle = tripped ? STATUS.critical : feederColour(te.feeder);
      ctx.strokeStyle = "#0b0b0b";
      ctx.lineWidth = 1.6;
      ctx.beginPath(); // diamond
      ctx.moveTo(x, y - r * 0.72);
      ctx.lineTo(x + r * 0.72, y);
      ctx.lineTo(x, y + r * 0.72);
      ctx.lineTo(x - r * 0.72, y);
      ctx.closePath();
      ctx.fill(); ctx.stroke();
    }
  }

  // Normally-open tie points between adjacent feeders: a short dashed link
  // with an OPEN circle at its midpoint (open = carries nothing until a
  // fault makes the neighbour backfeed through it).
  _drawTies() {
    const { ctx, world } = this;
    const net = world.net, g = world.graph;
    if (!net.ties) return;
    for (const t of net.ties) {
      ctx.strokeStyle = "#52514e";
      ctx.lineWidth = Math.max(1, this.view.scale * 35);
      ctx.setLineDash([3, 4]);
      ctx.beginPath();
      ctx.moveTo(this.sx(g.nx[t.a]), this.sy(g.ny[t.a]));
      ctx.lineTo(this.sx(g.nx[t.b]), this.sy(g.ny[t.b]));
      ctx.stroke();
      ctx.setLineDash([]);
      const mx = this.sx((g.nx[t.a] + g.nx[t.b]) / 2);
      const my = this.sy((g.ny[t.a] + g.ny[t.b]) / 2);
      const r = Math.max(2.2, this.view.scale * 70);
      ctx.fillStyle = "#fcfcfb";
      ctx.beginPath();
      ctx.arc(mx, my, r, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    }
  }

  _drawBridges() {
    const { ctx, world } = this;
    ctx.strokeStyle = "#0b0b0b";
    ctx.lineWidth = 1.4;
    for (const b of world.terrain.bridges) {
      const r = Math.max(3, this.view.scale * 130);
      ctx.strokeRect(this.sx(b.x) - r / 2, this.sy(b.y) - r / 2, r, r);
    }
  }

  _drawFaultMarker() {
    const { ctx, world } = this;
    const net = world.net, g = world.graph;
    const te = net.treeEdges[this.playback.scenario.teId];
    const x = this.sx((g.nx[te.parentNode] + g.nx[te.node]) / 2);
    const y = this.sy((g.ny[te.parentNode] + g.ny[te.node]) / 2);
    const r = Math.max(6, this.view.scale * 200);
    ctx.strokeStyle = STATUS.critical;
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(x - r, y - r); ctx.lineTo(x + r, y + r);
    ctx.moveTo(x - r, y + r); ctx.lineTo(x + r, y - r);
    ctx.stroke();
  }

  // Heat layer: per-cell max of the serving feeder's normalised
  // customer-minutes (sequential blue ramp — dark = worst).
  updateHeat(world, heatByFeeder) {
    const n = GRID_N;
    const vals = new Float32Array(n * n).fill(-1);
    for (const c of world.customers) {
      const tx = world.net.txs[c.tx];
      if (!tx || tx.feeder < 0) continue;
      const v = heatByFeeder.get(tx.feeder) ?? 0;
      const [cx, cy] = world.terrain.cellOf(c.x, c.y);
      const i = cy * n + cx;
      if (v > vals[i]) vals[i] = v;
    }
    const off = document.createElement("canvas");
    off.width = n; off.height = n;
    const ctx2 = off.getContext("2d");
    const img = ctx2.createImageData(n, n);
    const c0 = [205, 226, 251], c1 = [13, 54, 107]; // palette blue 100→700
    for (let i = 0; i < n * n; i++) {
      const v = vals[i];
      if (v < 0) continue;
      img.data[i * 4] = c0[0] + (c1[0] - c0[0]) * v;
      img.data[i * 4 + 1] = c0[1] + (c1[1] - c0[1]) * v;
      img.data[i * 4 + 2] = c0[2] + (c1[2] - c0[2]) * v;
      img.data[i * 4 + 3] = 120 + 110 * v;
    }
    ctx2.putImageData(img, 0, 0);
    this._heatCache = off;
  }

  // Subtransmission overlay: routed GXP → zone-sub lines (shared trunks)
  // plus the inter-sub tie. VISUAL ONLY — the model never reads these
  // lines; asserted in the checks panel.
  _drawSubtx() {
    const { ctx, world } = this;
    const st = world.subtx;
    if (!st || !st.gxp) return;
    for (const line of st.lines) {
      const tie = line.kind === "tie";
      ctx.strokeStyle = tie ? "#6b6a64" : "#3a3935";
      ctx.lineWidth = Math.max(tie ? 1.6 : 2.2, this.view.scale * (tie ? 60 : 90));
      ctx.setLineDash(tie ? [4, 5] : [11, 7]);
      ctx.beginPath();
      ctx.moveTo(this.sx(line.pts[0][0]), this.sy(line.pts[0][1]));
      for (let i = 1; i < line.pts.length; i++) {
        ctx.lineTo(this.sx(line.pts[i][0]), this.sy(line.pts[i][1]));
      }
      ctx.stroke();
    }
    ctx.setLineDash([]);
    const g = st.gxp;
    const r = Math.max(6, this.view.scale * 190);
    ctx.fillStyle = "#3a3935";
    ctx.fillRect(this.sx(g.x) - r, this.sy(g.y) - r, r * 2, r * 2);
    ctx.strokeStyle = "#fcfcfb";
    ctx.lineWidth = 2;
    ctx.strokeRect(this.sx(g.x) - r * 0.55, this.sy(g.y) - r * 0.55, r * 1.1, r * 1.1);
    ctx.font = "11px system-ui, sans-serif";
    ctx.fillStyle = "#3a3935";
    ctx.textAlign = "center";
    ctx.fillText("GXP — subtransmission (visual only)", this.sx(g.x), this.sy(g.y) - r - 5);
  }

  // Fit the view to one feeder's extent (plus its sub), with margin.
  zoomToFeeder(world, fid) {
    const net = world.net, g = world.graph;
    const f = net.feeders.find(f => f.id === fid);
    if (!f) return;
    let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
    const grow = (x, y) => {
      x0 = Math.min(x0, x); y0 = Math.min(y0, y);
      x1 = Math.max(x1, x); y1 = Math.max(y1, y);
    };
    for (const v of f.nodes) grow(g.nx[v], g.ny[v]);
    const sub = net.subs[f.sub];
    grow(sub.x, sub.y);
    const w = Math.max(1500, x1 - x0), h = Math.max(1500, y1 - y0);
    const { width, height } = this.canvas;
    const s = Math.min(0.5, Math.min(width / w, height / h) * 0.78);
    this.view.scale = s;
    this.view.ox = width / 2 - (x0 + x1) / 2 * s;
    this.view.oy = height / 2 - (y0 + y1) / 2 * s;
  }

  _drawTownLabels() {
    const { ctx, world } = this;
    ctx.font = "12px system-ui, sans-serif";
    ctx.fillStyle = "#52514e";
    ctx.textAlign = "center";
    for (const t of world.towns) {
      ctx.fillText(t.name, this.sx(t.x), this.sy(t.y) - 8);
    }
  }

  // nearest network tree edge to a screen point, within tolerance px
  hitTestEdge(px, py, tolPx = 14) {
    const { world } = this;
    if (!world) return -1;
    const g = world.graph, net = world.net;
    let best = -1, bestD = tolPx;
    for (const te of net.treeEdges) {
      const ax = this.sx(g.nx[te.parentNode]), ay = this.sy(g.ny[te.parentNode]);
      const bx = this.sx(g.nx[te.node]), by = this.sy(g.ny[te.node]);
      const dx = bx - ax, dy = by - ay;
      const len2 = dx * dx + dy * dy || 1;
      const t = Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / len2));
      const d = Math.hypot(px - (ax + t * dx), py - (ay + t * dy));
      if (d < bestD) { bestD = d; best = te.id; }
    }
    return best;
  }
}
