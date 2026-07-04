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

## Pipeline — strict dependency order (seeded/deterministic, ~0.15–0.25 s/attempt at 8000 customers)

Layers are produced in order, each consuming only earlier layers:
**terrain → settlements → roads → load → substation catchments →
subtransmission → feeders**, wrapped in a validation pass (below).

1. **Terrain** ([js/terrain.js](js/terrain.js)) — fBm elevation + ramp puts
   the sea along one seed-chosen edge; one river is traced from the far map
   edge to the ocean (so it genuinely bisects the map); slope, lakes,
   buildability, main-landmass flood fill (banks joined via the river —
   roads may bridge rivers, never the ocean).
2. **Settlements** ([js/density.js](js/density.js)) — sites scored on
   flatness, a coastal↔river blend (Inland slider), river-mouth proximity,
   and junctions of a provisional least-cost corridor skeleton (anchors ↔
   map-edge exits) that later seeds the arterials. Populations follow a
   Zipf rank-size law (α = 1.25: one dominant, 2–3 medium, several small),
   σ ∝ √pop, spacing scaled to town size, both river banks settled.
3. **Roads** ([js/roads.js](js/roads.js)) — arterials = corridor skeleton +
   MST + loop link, A* with slope² cost (sea blocked; first river crossing
   builds a bridge at 28×, then reused). Urban grids: spacing scales with
   population, orientation snaps to the local coast/valley axis — never
   global north — regular core fraying to irregular edges. Rural roads:
   spurs off arterials into flat country (some fork). Exact grid traversal
   for all water tests; connectivity repair.
4. **Load** ([js/customers.js](js/customers.js)) — density = mass-
   compensated town Gaussians (a sea-clipped town gets denser, keeping
   realised sizes Zipf) + rural background decaying with ROAD distance, so
   sparse rural load hugs the roads; 3000–20000 customers sampled on
   buildable land, snapped to the road graph.
5. **Substation catchments** ([js/network.js](js/network.js)) —
   deterministic k-means over the customer points (catchments > 4000
   split in two, < 500 merge into their nearest neighbour); each zone sub
   sits at its catchment's **load-weighted centroid**, nudged to the
   nearest subtransmission-viable road node (arterial/collector corridor,
   gentle slope) — never a geometric centre, never a town marker.
6. **Subtransmission** ([js/subtx.js](js/subtx.js)) — GXP on a flat
   map-edge cell near the load centroid; least-cost A* lines GXP → each
   sub (slope penalised, ocean blocked, river crossings 6×, road corridors
   rewarded, micro-siting cost noise); adjacent subs **share trunk
   corridors before branching**, plus one inter-sub tie routed to avoid
   the trunks. VISUAL ONLY — a check asserts SAIDI and network structure
   are unchanged by (re)building it.
7. **Feeders** ([js/network.js](js/network.js)) — capacitated TX
   clustering (≤ 50 customers), multi-source Dijkstra over roads from the
   subs, each sub tree partitioned into feeders sized by local density
   (~250 rural – ~700+ urban, ≈400–500 average) with runt merging,
   junction-overshoot splitting and express-run accounting (heads > 4 km
   out merge into their trunk feeder up to a 1000-customer cap). Sections
   in high-density cells are underground cable (drawn dashed), the rest
   overhead.

### Validation pass ([js/main.js](js/main.js), `VALIDATION`)

Named, tunable rules; a failing world regenerates on a deterministic retry
seed (`seed#retry1`, …); after `MAX_ATTEMPTS = 4` the **best attempt wins
(fewest failures) and the unresolved reasons are reported** in the Checks
panel — never a hard fail.

```
MAX_SUB_CENTROID_KM   = 5    // X: sub too far from its load centroid
MAX_SUBTX_STRAIGHT_KM = 6    // Y: subtx line straight for too long
MIN_GRID_SPREAD_DEG   = 15   // all town grids sharing one orientation
MIN_ZIPF_RATIO        = 3.0  // realised largest/median town size too flat
```

8. **Reliability** ([js/reliability.js](js/reliability.js)) — per-feeder
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
- **Devices-for-improvement table** — the fewest sectionalisers and the
  fewest reclosers (each kind placed greedily from a device-free network)
  needed to cut network SAIDI by ≥ 10%, 25% and 50%. Greedy stops as soon
  as the deepest target is met; ">N" means the target wasn't reached
  within the placement cap (60 SW / 30 RC, matching the UI inputs) and
  "not reachable" means greedy ran out of beneficial sites first. Your
  currently placed mix's improvement is shown below the table.
  Live-updates with fault rates and debug mode.
- **Feeder league table** — feeders ranked worst-first by customer-minutes
  per year (SAIDI × customers); click a row to zoom the map to that feeder.
  The matching "SAIDI heat" layer shades customer cells by the serving
  feeder's customer-minutes (sequential blue, dark = worst).
- **Inland town dispersion** — an Inland slider (0–100%) blends town-site
  scoring from coastal-biased to flat-river-valley-biased, so towns (and
  everything downstream of them) can settle inland.
- **Subtransmission overlay** — dashed heavy lines from a plausible GXP
  (flat map-edge cell near the load centroid) to each zone sub. VISUAL
  ONLY: a correctness check asserts SAIDI and network structure are
  identical with and without it.
- Seed input, customer/town/inland sliders, eleven layer toggles,
  click-a-feeder stats, pan/zoom.

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
