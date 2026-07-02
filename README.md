# NZ-style Distribution Network Sandbox

**TOY — not a trusted tool.** A procedural, seeded sandbox that invents a
~30 × 30 km NZ-flavoured patch of geography, settles it, roads it, wires it
up with a radial 11 kV-ish distribution network, and lets you play with
reliability (SAIDI, sectionaliser placement, fault playback). Everything is
invented — no real data — and every modelling assumption is labelled in the
UI Assumptions panel and in the source. NZ English, metric units.

## Run it

Any static file server works (ES modules need `http://`, not `file://`):

```powershell
cd nz-grid-sandbox
python -m http.server 8317
# then open http://localhost:8317/
```

Optional URL modes:
- `?selftest=1` — headless self-test: generates three seeds at 8000
  customers, runs every correctness check, greedy placement of both device
  kinds (monotonicity asserted), the debug rate experiment, and
  fault-conservation checks, and prints a JSON report into the page.
- `?demo=1` — regenerates, places 8 sectionalisers + 4 reclosers, and
  freezes a fault mid-timeline (handy for screenshots).
- `?scaletest=1` — one seed at 8k→64k customers with timing + checks.

## Scaling

Measured (Windows, headless Edge): all correctness checks pass and greedy
stays monotone up to at least **64,000 customers** (regen ≈ 2.2 s; TX
clustering dominates). The 5 s regen budget runs out around ~100k. Sub and
feeder counts now scale with load (subs 1–6, feeders ≈ customers/430), so
feeder sizes stay plausible well past 20k; the remaining soft ceiling is
the 6-sub cap (past ~30k each sub serves an unusually large area) and
visual dot saturation in towns. The UI slider stops at 20,000; larger
values work programmatically via `generate()` / `?scaletest=1`.

## Pipeline (all seeded/deterministic, ~0.1–0.4 s at 8000 customers)

1. **Terrain** ([js/terrain.js](js/terrain.js)) — fBm elevation + ramp puts
   the sea along one seed-chosen edge; one river is traced from the far map
   edge to the ocean (so it genuinely bisects the map); slope, lakes,
   buildability, main-landmass flood fill (banks joined via the river —
   roads may bridge rivers, never the ocean).
2. **Density** ([js/density.js](js/density.js)) — towns seeded on flat,
   coastal-biased sites **on both banks**; Gaussian falloff × fBm noise
   (same noise family as the terrain).
3. **Customers** ([js/customers.js](js/customers.js)) — 3000–8000 weighted
   samples on buildable land only.
4. **Roads** ([js/roads.js](js/roads.js)) — arterial MST between towns +
   loop link, A* over the terrain grid (slope² cost, sea blocked, river
   28× until a bridge exists, then reused); rotated urban street lattices;
   rural A* spurs to customer clusters; connectivity repair. River
   crossings register bridges; all segment-vs-water tests use exact
   Amanatides–Woo grid traversal (no sampling gaps).
5. **Electrical** ([js/network.js](js/network.js)) — capacitated TX
   clustering (≤ 50 customers); zone subs by greedy facility location over
   road distance (a new sub must capture ≥ 500 customers and either save
   them ≥ 2 km each on average or relieve a sub serving > 4000 — so urban
   subs max out near 4000 customers and rural subs can be as small as
   ~500); multi-source Dijkstra over the road graph;
   each sub tree is partitioned into feeders sized by local density —
   ~250 customers rural up to ~700+ urban, ≈400–500 average — with runt
   merging and junction-overshoot splitting. Feeder heads reach the sub by
   an express run of parallel circuit along the same roads; heads more
   than 4 km out merge into the feeder owning their trunk (up to the
   1000-customer cap), so long expresses only survive where a corridor is
   genuinely full. Sections in
   high-density cells are underground cable (drawn dashed), the rest
   overhead.
6. **Reliability** ([js/reliability.js](js/reliability.js)) — per-feeder
   SAIDI with separate overhead/underground fault rates (defaults 0.10 /
   0.03 faults·km⁻¹·yr⁻¹, both adjustable live in the UI); outage = crew
   travel from the sub at 50 km/h along roads + flat 120 min repair (both
   line types — flatters cables, labelled); switching restores upstream
   customers at a flat 45 min (flat time ⇒ greedy placement is provably
   monotone); express-run exposure is charged to each feeder as an
   un-switchable SAIDI floor.

## Interactive features

- **Greedy devices** — sectionalisers AND reclosers, each with its own
  count input. Both are scored by closed-form marginal customer-minutes
  saved, placed best-first with recomputation; per-placement log; running
  SAIDI is asserted monotone non-increasing for both kinds.
  - *Sectionaliser*: restoration only — upstream-of-switch customers in the
    tripped zone come back at a flat 45 min.
  - *Recloser*: ideal protection — faults downstream are cleared by the
    recloser, so upstream customers see no sustained interruption at all.
    Momentaries (SAIFI/MAIFI), fuse saving and coordination limits are not
    modelled — labelled in the Assumptions panel.
- **Debug rate mode** — doubles λ on one branch (drawn dashed) chosen so the
  greedy's first pick moves, demonstrating the score is rate-weighted; both
  picks are reported.
- **Fault playback** — toggle fault mode, click a line section; timeline
  animates breaker trip → switching (upstream restored) → repair complete,
  with the faulted / out-until-repair / isolatable / restored classes in
  distinct status colours. Customer conservation (out + isolatable =
  affected) is asserted per fault.
- **Road vs straight line** — per-feeder rate-weighted ratio of crew road
  distance to straight-line distance (bridges/terrain push it above 1);
  with the layer on and a section selected the two routes are drawn.
- Seed input, customer/town sliders, nine layer toggles, click-a-feeder
  stats, pan/zoom.

## Correctness guards (asserted and reported in the Checks panel)

- Road graph fully connected (after repair; merge count reported).
- Every customer served by exactly one feeder; feeder totals conserve the
  population; no customers in water.
- Roads cross the river **only** at generated bridges (exact traversal).
- Baseline SAIDI finite and positive.
- Greedy running SAIDI monotone non-increasing.
- Fault classification conserves customers (no double-count).

## Design choices

Plausible-not-optimal throughout: greedy clustering, greedy switch
placement, MST arterials — real networks accrete, they aren't optimised.
Colours follow a validated colour-blind-safe categorical palette (fixed
slot order) with reserved status colours for fault states.
