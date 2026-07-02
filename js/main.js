// main.js — pipeline orchestration + UI wiring + headless selftest mode.

import { RNG } from "./rng.js";
import { Terrain, MAP_SIZE } from "./terrain.js";
import { seedTowns, buildDensityGrid } from "./density.js";
import { sampleCustomers } from "./customers.js";
import { buildRoads } from "./roads.js";
import { buildNetwork } from "./network.js";
import {
  computeSaidi, greedyPlace, allCandidates, debugRateExperiment,
  faultScenario, roadVsLine, emptyDevices, setFaultRates, faultRates,
  SWITCH_MIN, REPAIR_MIN,
} from "./reliability.js";
import { runChecks, checkFaultConservation, checkMonotone } from "./checks.js";
import { ASSUMPTIONS } from "./assumptions.js";
import { Renderer, feederColour, STATUS } from "./render.js";

// ------------------------------------------------------------ generation

export function generate(params) {
  const t0 = performance.now();
  const rng = new RNG(params.seed);
  const timings = {};
  const mark = (name, since) => { timings[name] = Math.round(performance.now() - since); };

  let t = performance.now();
  const terrain = new Terrain(rng.fork("terrain"));
  mark("terrain", t);

  t = performance.now();
  const towns = seedTowns(terrain, rng.fork("towns"), params.nTowns);
  const density = buildDensityGrid(terrain, towns, rng.fork("density"));
  mark("density", t);

  t = performance.now();
  const customers = sampleCustomers(terrain, density, params.nCust, rng.fork("cust"));
  mark("customers", t);

  t = performance.now();
  const { graph, repair, snapStats } = buildRoads(terrain, towns, customers, rng.fork("roads"));
  mark("roads", t);

  t = performance.now();
  const net = buildNetwork(terrain, graph, customers, towns, density, rng.fork("net"));
  mark("network", t);

  const world = {
    params, terrain, towns, density, customers, graph, net,
    roadRepair: repair, snapStats,
  };
  t = performance.now();
  world.checks = runChecks(world);
  world.roadVsLine = roadVsLine(net, graph);
  mark("checks", t);
  timings.total = Math.round(performance.now() - t0);
  world.timings = timings;
  return world;
}

// ------------------------------------------------------------- selftest

function selftest() {
  const results = [];
  for (const seed of ["aotearoa-1", "kahikatea-2", "rimu-3"]) {
    const world = generate({ seed, nCust: 8000, nTowns: 5 });
    const { net } = world;
    const greedy = greedyPlace(net, 12, "switch");
    const mono = checkMonotone(greedy.log);
    const greedyRc = greedyPlace(net, 6, "recloser", greedy.devices);
    const monoRc = checkMonotone(greedyRc.log);
    const dbg = debugRateExperiment(net);
    // fault conservation on three deterministic edges, with devices placed
    const ids = [0, Math.floor(net.treeEdges.length / 2), net.treeEdges.length - 1];
    const conservation = ids.map(i => {
      const sc = faultScenario(net, greedyRc.devices, i);
      return checkFaultConservation(sc).pass;
    });
    const rvl = world.roadVsLine;
    // --- diagnostics: river geometry + customer split across the river
    const rp = world.terrain.riverPath;
    let riverLen = 0;
    for (let i = 1; i < rp.length; i++) riverLen += Math.hypot(rp[i].x - rp[i - 1].x, rp[i].y - rp[i - 1].y);
    let left = 0;
    for (const c of world.customers) {
      let bi = 0, bd = Infinity;
      for (let i = 0; i < rp.length - 1; i += 4) {
        const d = (rp[i].x - c.x) ** 2 + (rp[i].y - c.y) ** 2;
        if (d < bd) { bd = d; bi = i; }
      }
      const j = Math.min(rp.length - 1, bi + 4);
      const cross = (rp[j].x - rp[bi].x) * (c.y - rp[bi].y) - (rp[j].y - rp[bi].y) * (c.x - rp[bi].x);
      if (cross > 0) left++;
    }
    const ranked = allCandidates(net, emptyDevices(), "switch");
    // --- diagnostics: how far customers sit from their sub (road metres)
    const dists = world.customers
      .map(c => net.dist[net.txs[c.tx].node]).filter(isFinite).sort((a, b) => a - b);
    const q = (p) => +(dists[Math.floor(p * (dists.length - 1))] / 1000).toFixed(1);
    let minSep = Infinity;
    for (let i = 0; i < net.subs.length; i++) {
      for (let j = i + 1; j < net.subs.length; j++) {
        minSep = Math.min(minSep, Math.hypot(
          net.subs[i].x - net.subs[j].x, net.subs[i].y - net.subs[j].y));
      }
    }
    results.push({
      custDistKm: {
        mean: +(dists.reduce((s, d) => s + d, 0) / dists.length / 1000).toFixed(1),
        p50: q(0.5), p95: q(0.95), max: q(1),
        beyond10kmPct: +(100 * dists.filter(d => d > 10000).length / dists.length).toFixed(1),
      },
      subSepKm: isFinite(minSep) ? +(minSep / 1000).toFixed(1) : null,
      subPos: net.subs.map(s => [Math.round(s.x / 100) / 10, Math.round(s.y / 100) / 10]),
      subCust: net.subs.map(s => Math.round(
        net.feeders.filter(f => f.sub === s.id).reduce((t, f) => t + f.customers, 0))),
      expressKm: (() => {
        const ex = net.feeders.map(f => f.expressOhKm + f.expressUgKm).sort((a, b) => b - a);
        return {
          mean: +(ex.reduce((a, b) => a + b, 0) / ex.length).toFixed(1),
          top3: ex.slice(0, 3).map(x => +x.toFixed(1)),
        };
      })(),
      river: {
        lenKm: +(riverLen / 1000).toFixed(1),
        start: [Math.round(rp[0].x), Math.round(rp[0].y)],
        end: [Math.round(rp[rp.length - 1].x), Math.round(rp[rp.length - 1].y)],
        custLeftShare: +(left / world.customers.length).toFixed(3),
      },
      dbgGains: ranked.slice(0, 6).map(c => ({ f: c.feeder, n: c.node, g: Math.round(c.gain) })),
      townSides: world.towns.map(t => t.side),
      dbgBoostGain: dbg.supported ? Math.round(dbg.boostGain ?? -1) : null,
      seed,
      totalMs: world.timings.total,
      timings: world.timings,
      checks: world.checks.map(c => ({ name: c.name, pass: c.pass, detail: c.detail })),
      feeders: net.feeders.length,
      meanCustPerFeeder: Math.round(world.customers.length / net.feeders.length),
      urbanMeanCust: feederKindMean(net, true),
      ruralMeanCust: feederKindMean(net, false),
      feederSizes: net.feeders.map(f => f.customers).sort((a, b) => b - a),
      subs: net.subs.length,
      txs: net.txs.length,
      roadNodes: world.graph.nNodes,
      bridges: world.terrain.bridges.length,
      snapMeanM: Math.round(world.snapStats.mean),
      snapMaxM: Math.round(world.snapStats.max),
      baselineSaidi: +greedy.baseline.toFixed(1),
      afterSwitchesSaidi: +greedy.final.toFixed(1),
      finalSaidi: +greedyRc.final.toFixed(1),
      greedyMonotone: mono.pass,
      recloserMonotone: monoRc.pass,
      greedyPlacements: greedy.log.filter(l => !l.stopped).length,
      recloserPlacements: greedyRc.log.filter(l => !l.stopped).length,
      debugSupported: dbg.supported,
      debugMoved: dbg.supported ? dbg.moved : null,
      faultConservation: conservation.every(Boolean),
      roadVsLineMin: +Math.min(...rvl.map(r => r.ratio)).toFixed(3),
      roadVsLineMax: +Math.max(...rvl.map(r => r.ratio)).toFixed(3),
    });
  }
  const allPass = results.every(r =>
    r.checks.every(c => c.pass) && r.greedyMonotone && r.recloserMonotone &&
    r.faultConservation &&
    r.meanCustPerFeeder >= 200 && r.meanCustPerFeeder <= 800 &&
    r.totalMs < 5000 && (!r.debugSupported || r.debugMoved !== false));
  const out = { allPass, results };
  const pre = document.getElementById("selftest-out");
  pre.textContent = JSON.stringify(out, null, 1);
  pre.style.display = "block";
  document.title = "SELFTEST-DONE";
  return out;
}

// Mean customers per feeder, split urban vs rural by underground share
// (UG sections exist exactly where density is high).
function feederKindMean(net, urban) {
  const sel = net.feeders.filter(f => {
    const ugShare = f.ugLenM / Math.max(1, f.lengthM);
    return urban ? ugShare >= 0.4 : ugShare < 0.4;
  });
  if (!sel.length) return null;
  return Math.round(sel.reduce((s, f) => s + f.customers, 0) / sel.length);
}

// ------------------------------------------------------------- scaletest
// ?scaletest=1 — one seed at increasing customer counts, reporting timing
// and all correctness checks, to find where the model gets slow or messy.

function scaletest() {
  const sizes = [8000, 12000, 16000, 24000, 32000, 48000, 64000];
  const results = [];
  for (const nCust of sizes) {
    const t0 = performance.now();
    const world = generate({ seed: "aotearoa-1", nCust, nTowns: 5 });
    const genMs = Math.round(performance.now() - t0);
    const t1 = performance.now();
    const greedy = greedyPlace(world.net, 8, "switch");
    const greedyRc = greedyPlace(world.net, 4, "recloser", greedy.devices);
    const greedyMs = Math.round(performance.now() - t1);
    results.push({
      nCust, genMs, greedyMs,
      timings: world.timings,
      checksPass: world.checks.every(c => c.pass),
      failedChecks: world.checks.filter(c => !c.pass).map(c => c.name),
      txs: world.net.txs.length,
      feeders: world.net.feeders.length,
      treeEdges: world.net.treeEdges.length,
      roadNodes: world.graph.nNodes,
      snapMeanM: Math.round(world.snapStats.mean),
      monotone: checkMonotone(greedy.log).pass && checkMonotone(greedyRc.log).pass,
      maxTxLoadShare: +(Math.max(...world.net.txs.map(t => t.customers.length)) / 50).toFixed(2),
    });
  }
  const pre = document.getElementById("selftest-out");
  pre.textContent = JSON.stringify({ results }, null, 1);
  pre.style.display = "block";
  document.title = "SELFTEST-DONE";
}

// ------------------------------------------------------------- UI state

const $ = (id) => document.getElementById(id);
const state = {
  world: null,
  renderer: null,
  greedyLog: [],
  debug: null,       // debugRateExperiment result when debug mode on
  rateMul: null,
  playbackAnim: null,
  faultMode: false,
};

function fmt(x, dp = 1) { return x.toLocaleString("en-NZ", { maximumFractionDigits: dp, minimumFractionDigits: dp }); }

function regenerate() {
  const params = {
    seed: $("seed").value || "aotearoa",
    nCust: +$("nCust").value,
    nTowns: +$("nTowns").value,
  };
  const world = generate(params);
  state.world = world;
  state.greedyLog = [];
  state.debug = null;
  state.rateMul = null;
  state.playbackAnim = null;
  $("debugRate").checked = false;
  state.renderer.setWorld(world);
  const ohKm = world.net.feeders.reduce((s, f) => s + f.ohLenM, 0) / 1000;
  const ugKm = world.net.feeders.reduce((s, f) => s + f.ugLenM, 0) / 1000;
  const meanCust = world.customers.length / world.net.feeders.length;
  $("genTime").textContent =
    `generated in ${world.timings.total} ms — ` +
    `${world.graph.nNodes.toLocaleString()} road nodes, ${world.net.txs.length} TX, ` +
    `${world.net.subs.length} subs, ${world.net.feeders.length} feeders ` +
    `(mean ${Math.round(meanCust)} cust/feeder), ` +
    `${ohKm.toFixed(0)} km OH / ${ugKm.toFixed(0)} km UG, ` +
    `${world.terrain.bridges.length} bridge(s)`;
  renderChecks();
  renderSaidi();
  renderRoadVsLine();
  renderFeederStats(-1);
  $("greedyLog").innerHTML = "";
  $("playbackInfo").innerHTML = "<em>Toggle fault mode, then click a line section on the map.</em>";
  draw();
}

function draw() { state.renderer.draw(); }

function currentDevices() {
  return {
    switches: new Set(state.renderer.switchList),
    reclosers: new Set(state.renderer.recloserList),
  };
}

function renderChecks(extra = []) {
  const el = $("checks");
  const items = [...state.world.checks, ...extra];
  el.innerHTML = items.map(c =>
    `<div class="check ${c.pass ? "pass" : "fail"}">` +
    `<span class="mark">${c.pass ? "✓" : "✗"}</span>` +
    `<div><strong>${c.name}</strong><br><span class="detail">${c.detail}</span></div></div>`
  ).join("");
}

function renderSaidi() {
  const { net } = state.world;
  const dev = currentDevices();
  const base = computeSaidi(net, emptyDevices(), state.rateMul);
  const now = computeSaidi(net, dev, state.rateMul);
  $("saidiBase").textContent = fmt(base.overall);
  $("saidiNow").textContent = fmt(now.overall);
  $("saidiSwitches").textContent = dev.switches.size;
  $("saidiReclosers").textContent = dev.reclosers.size;
  const delta = base.overall - now.overall;
  $("saidiDelta").textContent = fmt(delta);
}

function renderRoadVsLine() {
  const rows = state.world.roadVsLine.map(r =>
    `<tr><td><span class="chip" style="background:${feederColour(r.feeder)}"></span> F${r.feeder}</td>` +
    `<td>${fmt(r.roadKm, 2)}</td><td>${fmt(r.lineKm, 2)}</td><td><strong>${fmt(r.ratio, 2)}×</strong></td></tr>`
  ).join("");
  $("rvlTable").innerHTML =
    `<tr><th>Feeder</th><th>Road km</th><th>Line km</th><th>Ratio</th></tr>` + rows;
}

function renderFeederStats(fid) {
  const el = $("feederStats");
  if (fid < 0) { el.innerHTML = "<em>Click a coloured feeder line on the map.</em>"; return; }
  const { net } = state.world;
  const f = net.feeders.find(f => f.id === fid);
  const dev = currentDevices();
  const base = computeSaidi(net, emptyDevices(), state.rateMul).perFeeder.find(p => p.feeder === fid);
  const now = computeSaidi(net, dev, state.rateMul).perFeeder.find(p => p.feeder === fid);
  const rvl = state.world.roadVsLine.find(r => r.feeder === fid);
  const nSw = [...dev.switches].filter(id => net.treeEdges[id].feeder === fid).length;
  const nRc = [...dev.reclosers].filter(id => net.treeEdges[id].feeder === fid).length;
  const ugShare = f.ugLenM / Math.max(1, f.lengthM);
  const kind = ugShare >= 0.4 ? "urban" : ugShare >= 0.15 ? "mixed" : "rural";
  const perKm = f.customers / Math.max(0.3, f.lengthM / 1000);
  const expressKm = f.expressOhKm + f.expressUgKm;
  el.innerHTML = `
    <div><span class="chip" style="background:${feederColour(fid)}"></span>
      <strong>Feeder F${fid}</strong> — ${net.subs[f.sub].name} zone sub</div>
    <table>
      <tr><td>Type</td><td>${kind} (${fmt(perKm, 0)} cust/km)</td></tr>
      <tr><td>Customers</td><td>${f.customers}</td></tr>
      <tr><td>Transformers</td><td>${f.txCount}</td></tr>
      <tr><td>HV length</td><td>${fmt(f.lengthM / 1000, 1)} km ` +
        `(${fmt(f.ohLenM / 1000, 1)} OH / ${fmt(f.ugLenM / 1000, 1)} UG)</td></tr>
      <tr><td>Express run</td><td>${fmt(expressKm, 1)} km</td></tr>
      <tr><td>Sectionalisers</td><td>${nSw}</td></tr>
      <tr><td>Reclosers</td><td>${nRc}</td></tr>
      <tr><td>SAIDI baseline</td><td>${fmt(base.saidi)} min/yr</td></tr>
      <tr><td>SAIDI now</td><td>${fmt(now.saidi)} min/yr</td></tr>
      <tr><td>Road ÷ line distance</td><td>${fmt(rvl.ratio, 2)}×</td></tr>
    </table>`;
}

// ------------------------------------------ greedy device placement

function placeDevices(kind) {
  const n = +(kind === "switch" ? $("nSwitches") : $("nReclosers")).value;
  const { net } = state.world;
  const existing = currentDevices();
  const nExisting = existing.switches.size + existing.reclosers.size;
  const result = greedyPlace(net, n, kind, existing, state.rateMul);
  const newOnes = result.log.filter(l => !l.stopped);
  if (result.log.some(l => l.stopped)) {
    const div = document.createElement("div");
    div.className = "logentry";
    div.innerHTML = `<em>stopped early — no candidate with positive benefit</em>`;
    $("greedyLog").prepend(div);
  }
  // animate placements landing one by one
  let i = 0;
  const logEl = $("greedyLog");
  const stepIn = () => {
    if (i >= newOnes.length) {
      const mono = checkMonotone(result.log);
      renderChecks([mono]);
      return;
    }
    const l = newOnes[i++];
    (kind === "switch" ? state.renderer.switchList : state.renderer.recloserList).push(l.teId);
    const div = document.createElement("div");
    div.className = "logentry";
    div.innerHTML =
      `<span class="chip" style="background:${feederColour(l.feeder)}"></span>` +
      `#${nExisting + l.step} <strong>${kind === "switch" ? "SW" : "RC"}</strong> ` +
      `node ${l.node} (F${l.feeder}) — ` +
      `<strong>${l.custRestored}</strong> customers ${kind === "switch" ? "restorable" : "protected"}, ` +
      `SAIDI ${fmt(l.saidiBefore)} → <strong>${fmt(l.saidiAfter)}</strong> ` +
      `(−${fmt(l.benefitMin, 2)} min) ${l.monotone ? "" : "⚠ NON-MONOTONE"}`;
    logEl.prepend(div);
    renderSaidi();
    renderFeederStats(state.renderer.selectedFeeder);
    draw();
    setTimeout(stepIn, 240);
  };
  stepIn();
}

function resetDevices() {
  state.renderer.switchList = [];
  state.renderer.recloserList = [];
  $("greedyLog").innerHTML = "";
  renderSaidi();
  renderChecks();
  draw();
}

function toggleDebugRate(on) {
  const { net } = state.world;
  resetDevices();
  if (!on) {
    state.debug = null;
    state.rateMul = null;
    state.renderer.debugBranch = null;
    $("debugInfo").innerHTML = "";
  } else {
    const dbg = debugRateExperiment(net);
    state.debug = dbg;
    if (!dbg.supported) {
      $("debugInfo").innerHTML = `<em>Not supported: ${dbg.reason ?? "no candidates"}.</em>`;
      state.rateMul = null;
      state.renderer.debugBranch = null;
    } else {
      state.rateMul = dbg.rateMul;
      state.renderer.debugBranch = dbg.branch;
      $("debugInfo").innerHTML =
        `λ doubled on the branch below node ${dbg.boostNode} ` +
        `(<span class="chip" style="background:${feederColour(dbg.boostFeeder)}"></span>F${dbg.boostFeeder}, ` +
        `${dbg.branch.size} sections, shown dashed).<br>` +
        `First greedy pick: node ${dbg.basePick.node} (F${dbg.basePick.feeder}) → ` +
        `node ${dbg.debugPick.node} (F${dbg.debugPick.feeder}) — ` +
        (dbg.moved
          ? `<strong class="ok">pick moved ✓ (benefit is rate-weighted)</strong>`
          : `<strong class="bad">pick did NOT move ✗</strong>`);
    }
  }
  renderSaidi();
  draw();
}

// ------------------------------------------------------------ fault playback

function startFault(teId) {
  const { net } = state.world;
  const sc = faultScenario(net, currentDevices(), teId);
  const cons = checkFaultConservation(sc);
  renderChecks([cons]);
  const tEnd = sc.tRepairDone + 15;
  state.playbackAnim = { sc, tEnd, playing: true, tMin: 0 };
  state.renderer.playback = { scenario: sc, tMin: 0 };
  const swText = sc.tSwitch === null
    ? `<span class="chip" style="background:${STATUS.serious}"></span> no sectionaliser in the tripped zone — everyone tripped waits for repair`
    : `<span class="chip" style="background:${STATUS.warning}"></span> ${sc.custIso} customers isolatable — restored at ${SWITCH_MIN} min`;
  const rcText = sc.recloserEdge !== -1
    ? `<div><span class="chip" style="background:${feederColour(sc.feeder)}"></span> ${sc.custUnaffected} customers upstream of the recloser — never interrupted</div>`
    : "";
  $("playbackInfo").innerHTML = `
    <div><strong>Fault on F${sc.feeder}</strong>, section ${sc.teId} — ${sc.custAffected} customers affected` +
    `${sc.recloserEdge !== -1 ? " (recloser zone)" : ""}</div>
    <div><span class="chip" style="background:${STATUS.critical}"></span> faulted section</div>
    <div><span class="chip" style="background:${STATUS.serious}"></span> ${sc.custOut} customers out until repair (travel ${fmt(sc.tTravel)} + ${REPAIR_MIN} min)</div>
    <div>${swText}</div>
    ${rcText}
    <div class="detail ${cons.pass ? "ok" : "bad"}">${cons.pass ? "✓" : "✗"} ${cons.detail}</div>
    <div class="timeline"><input type="range" id="tScrub" min="0" max="${Math.ceil(tEnd)}" value="0" step="1">
    <span id="tLabel">t = 0 min</span></div>`;
  $("tScrub").addEventListener("input", (e) => {
    state.playbackAnim.playing = false;
    state.playbackAnim.tMin = +e.target.value;
    updatePlayback();
  });
  const t0 = performance.now();
  const realMsPerSimMin = 7000 / tEnd; // whole timeline ≈ 7 s
  const tick = () => {
    if (!state.playbackAnim || state.playbackAnim.sc !== sc) return;
    if (state.playbackAnim.playing) {
      state.playbackAnim.tMin = Math.min(tEnd, (performance.now() - t0) / realMsPerSimMin);
      updatePlayback();
      if (state.playbackAnim.tMin < tEnd) requestAnimationFrame(tick);
    }
  };
  requestAnimationFrame(tick);
}

function updatePlayback() {
  const pa = state.playbackAnim;
  state.renderer.playback = { scenario: pa.sc, tMin: pa.tMin };
  const scrub = $("tScrub");
  if (scrub) { scrub.value = Math.round(pa.tMin); }
  const lbl = $("tLabel");
  if (lbl) {
    const zone = pa.sc.recloserEdge !== -1 ? "recloser zone out" : "whole feeder out";
    let phase = `fault — protection open, ${zone}`;
    if (pa.sc.tSwitch !== null && pa.tMin >= pa.sc.tSwitch) phase = "switched — upstream of switch restored";
    if (pa.tMin >= pa.sc.tRepairDone) phase = "repaired — all restored";
    lbl.textContent = `t = ${Math.round(pa.tMin)} min — ${phase}`;
  }
  draw();
}

function clearFault() {
  state.playbackAnim = null;
  state.renderer.playback = null;
  $("playbackInfo").innerHTML = "<em>Toggle fault mode, then click a line section on the map.</em>";
  renderChecks();
  draw();
}

// --------------------------------------------------------------- wiring

function initUI() {
  const canvas = $("map");
  const resize = () => {
    const r = canvas.parentElement.getBoundingClientRect();
    canvas.width = r.width; canvas.height = r.height;
    if (state.renderer) draw();
  };
  state.renderer = new Renderer(canvas);
  window.addEventListener("resize", resize);
  resize();

  // pan/zoom
  let dragging = false, lx = 0, ly = 0, moved = 0;
  canvas.addEventListener("mousedown", (e) => { dragging = true; moved = 0; lx = e.offsetX; ly = e.offsetY; });
  window.addEventListener("mouseup", () => { dragging = false; });
  canvas.addEventListener("mousemove", (e) => {
    if (!dragging) return;
    state.renderer.pan(e.offsetX - lx, e.offsetY - ly);
    moved += Math.abs(e.offsetX - lx) + Math.abs(e.offsetY - ly);
    lx = e.offsetX; ly = e.offsetY;
    draw();
  });
  canvas.addEventListener("wheel", (e) => {
    e.preventDefault();
    state.renderer.zoomAt(e.offsetX, e.offsetY, e.deltaY < 0 ? 1.15 : 1 / 1.15);
    draw();
  }, { passive: false });
  canvas.addEventListener("click", (e) => {
    if (moved > 4) return; // was a drag
    const teId = state.renderer.hitTestEdge(e.offsetX, e.offsetY);
    if (teId < 0) return;
    const fid = state.world.net.treeEdges[teId].feeder;
    state.renderer.selectedFeeder = fid;
    state.renderer.selectedEdge = teId;
    renderFeederStats(fid);
    if (state.faultMode) startFault(teId);
    draw();
  });

  // controls
  $("regen").addEventListener("click", regenerate);
  $("seed").addEventListener("keydown", (e) => { if (e.key === "Enter") regenerate(); });
  $("randomSeed").addEventListener("click", () => {
    $("seed").value = "nz-" + Math.floor(performance.now() * 997 % 100000);
    regenerate();
  });
  for (const id of ["nCust", "nTowns"]) {
    $(id).addEventListener("input", () => { $(id + "Val").textContent = $(id).value; });
  }
  for (const layer of ["terrain", "density", "roads", "customers", "network", "txs", "switches", "bridges", "roadVsLine"]) {
    const box = $("layer-" + layer);
    if (!box) continue;
    box.checked = state.renderer.layers[layer];
    box.addEventListener("change", () => {
      state.renderer.layers[layer] = box.checked;
      draw();
    });
  }
  $("placeSwitches").addEventListener("click", () => placeDevices("switch"));
  $("placeReclosers").addEventListener("click", () => placeDevices("recloser"));
  $("resetSwitches").addEventListener("click", resetDevices);
  for (const id of ["rateOh", "rateUg"]) {
    $(id).addEventListener("change", () => {
      setFaultRates(+$("rateOh").value, +$("rateUg").value);
      // placed devices stay; all figures recompute under the new rates
      state.world.roadVsLine = roadVsLine(state.world.net, state.world.graph);
      renderSaidi();
      renderRoadVsLine();
      renderFeederStats(state.renderer.selectedFeeder);
      renderChecks();
      draw();
    });
  }
  $("debugRate").addEventListener("change", (e) => toggleDebugRate(e.target.checked));
  $("faultMode").addEventListener("change", (e) => {
    state.faultMode = e.target.checked;
    if (!state.faultMode) clearFault();
  });
  $("clearFault").addEventListener("click", () => { $("faultMode").checked = false; state.faultMode = false; clearFault(); });

  // assumptions panel
  $("assumptions").innerHTML = ASSUMPTIONS.map(a =>
    `<div class="assumption"><strong>${a.area}</strong> — ${a.text}</div>`).join("");

  regenerate();

  // ?demo — place switches and freeze a fault mid-timeline (used for
  // headless visual verification; harmless to use by hand too).
  if (location.search.includes("demo")) {
    const { net } = state.world;
    const rSw = greedyPlace(net, 8, "switch");
    for (const l of rSw.log.filter(l => !l.stopped)) state.renderer.switchList.push(l.teId);
    const rRc = greedyPlace(net, 4, "recloser", currentDevices());
    for (const l of rRc.log.filter(l => !l.stopped)) state.renderer.recloserList.push(l.teId);
    renderSaidi();
    const mid = net.treeEdges[Math.floor(net.treeEdges.length * 0.55)];
    startFault(mid.id);
    state.playbackAnim.playing = false;
    state.playbackAnim.tMin = Math.max(SWITCH_MIN + 5,
      Math.min(state.playbackAnim.sc.tRepairDone - 10, 60));
    updatePlayback();
  }
}

// --------------------------------------------------------------- boot

if (location.search.includes("scaletest")) {
  try {
    scaletest();
  } catch (err) {
    const pre = document.getElementById("selftest-out");
    pre.textContent = JSON.stringify({ error: String(err), stack: String(err.stack) });
    pre.style.display = "block";
    document.title = "SELFTEST-DONE";
  }
} else if (location.search.includes("selftest")) {
  // Synchronous at module top level so headless --dump-dom captures it.
  try {
    selftest();
  } catch (err) {
    const pre = document.getElementById("selftest-out");
    pre.textContent = JSON.stringify({ allPass: false, error: String(err), stack: String(err.stack) });
    pre.style.display = "block";
    document.title = "SELFTEST-DONE";
  }
} else {
  initUI();
}
