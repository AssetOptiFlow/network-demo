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
- `?selftest=1` — headless self-test: generates three seeds at the fixed
  25 000 customers, runs every correctness check (18-sub count asserted),
  greedy placement of both device kinds (monotonicity asserted), the
  debug rate experiment, and fault-conservation checks, and prints a JSON
  report into the page.
- `?demo=1` — regenerates, places 8 sectionalisers + 4 reclosers, and
  freezes a fault mid-timeline (handy for screenshots).
- `?scaletest=1` — one seed at 8k→64k customers with timing + checks.

## Scaling

Measured (Windows, headless Edge): all **correctness** checks pass and
greedy stays monotone up to at least **64,000 customers** (regen ≈ 5.1 s
at 64k, ≈ 0.3 s at 8k; the membership route/validate/repair loop
dominates). Feeder counts scale with load (≈ customers/100 under the
default caps). Past ~32k the `MAX_SUBS = 24` backstop clamps sub count,
so the tunable feeders-per-sub rule reports honest residuals at scale.
The UI is fixed at 25,000 customers; other sizes work programmatically
via `generate()` / `?scaletest=1` (the 18-sub count stays fixed, so very
large worlds report feeders-per-sub residuals).

## Pipeline — strict dependency order (seeded/deterministic)

Layers are produced in order, each consuming only earlier layers:
**terrain → settlements → roads → load → membership (customers → feeders →
subs, before any routing) → sub siting → feeder routing → subtransmission
(visual only)**, wrapped in a validation pass (below).

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
   compensated town kernels (Gaussian core + exponential shoulder, so
   density eases through a peri-urban fringe rather than cliff-edging at
   the town boundary; a sea-clipped town gets denser, keeping realised
   sizes rank-ordered) + rural background decaying with ROAD distance, so
   sparse rural load hugs the roads; **25 000 customers (fixed in the
   UI)** sampled on buildable land, snapped to the road graph.
5. **Membership** ([js/membership.js](js/membership.js)) — decided BEFORE
   any routing. Customers classified urban/rural by local density
   (`URBAN_CUST_PER_KM2 = 60` within 500 m — absolute, so a denser world
   classifies more urban). Customers (as TX load nodes, ≤ 50 cust/TX) →
   feeders by **road-graph-capacitated clustering** (urban ≤ `N_URBAN_MAX
   = 700`; rural ≤ `N_RURAL_MAX = 300` plus a `RURAL_EXTENT_KM_MAX = 10`
   road-radius cap; growth fills to the cap by skipping oversize nodes,
   and runts < 150 fold into their road-nearest neighbour — targeting
   **~80–120 feeders** at the fixed 25 000 customers). Feeders →
   **exactly `N_SUBS = 18` zone subs (fixed)** by grouping
   **road-adjacent** feeders (shared road-graph Voronoi boundary, not
   straight-line proximity); grouping is forced to the count and every
   repair is count-preserving, with `FEEDERS_PER_SUB_MAX = 8` as the
   tunable per-sub ceiling.
6. **Sub siting** ([js/network.js](js/network.js)) — each zone sub sits at
   the **load-weighted centroid of its feeder group**, nudged to the
   nearest subtransmission-viable road node (arterial/collector corridor,
   gentle slope) — never a geometric centre, never a town marker.
7. **Feeder routing** ([js/network.js](js/network.js)) — LAST, honouring
   membership: one Dijkstra tree per sub over the roads, serving ONLY that
   sub's members; trees may not overlap (one circuit per road corridor).
   A sub only claims road nodes inside its own catchment; a path's
   stretches through a foreign catchment are charged as EXPRESS exposure
   (the second circuit strung along a shared road — bridges especially).
   Every feeder ends up a contiguous subtree with a single root (enforced,
   logged); express runs back to the sub are charged as un-switchable base
   SAIDI. A bounded **validate + repair loop** (`MAX_REPAIR_PASSES = 3`)
   re-splits / re-groups affected members when a feeder trunk exceeds
   `MAX_FEEDER_KM = 20`, a path rides another sub's actual network beyond
   `MAX_FOREIGN_CROSSING_M = 2500`, or any cap is breached — outcomes
   reported in the Checks panel. Sections in high-density cells are
   underground cable (drawn dashed), the rest overhead.
8. **Subtransmission** ([js/subtx.js](js/subtx.js)) — GXP on a flat
   map-edge cell near the load centroid; least-cost A* lines GXP → each
   sub (slope penalised, ocean blocked, river crossings 6×, road corridors
   rewarded, micro-siting cost noise); adjacent subs **share trunk
   corridors before branching**, plus one inter-sub tie routed to avoid
   the trunks. VISUAL ONLY — a check asserts SAIDI and network structure
   are unchanged by (re)building it.

### Validation pass ([js/main.js](js/main.js), `VALIDATION`)

Named, tunable rules; a failing world regenerates on a deterministic retry
seed (`seed#retry1`, …); after `MAX_ATTEMPTS = 4` the **best attempt wins
(fewest failures) and the unresolved reasons are reported** in the Checks
panel — never a hard fail.

```
MAX_SUB_CENTROID_KM   = 5    // X: sub too far from its feeder-group load centroid
MAX_SUBTX_STRAIGHT_KM = 8    // Y: subtx line straight for too long
MIN_GRID_SPREAD_DEG   = 15   // all town grids sharing one orientation
MIN_ZIPF_RATIO        = 2.2  // realised largest/median town size too flat
                             // (the peri-urban kernel spreads big-town mass)
```

### Membership caps ([js/membership.js](js/membership.js), [js/network.js](js/network.js) — all tunable)

```
N_SUBS                  = 18    // zone-sub count — FIXED (grouping forced to it)
URBAN_CUST_PER_KM2      = 60    // urban ⇔ ≥ this many customers within 500 m
N_URBAN_MAX             = 700   // customer cap, urban feeder
N_RURAL_MAX             = 300   // customer cap, rural feeder
RURAL_EXTENT_KM_MAX     = 10    // rural feeder growth radius by road (span ≤ 2×)
FEEDERS_PER_SUB_MAX     = 8     // feeder-count cap per zone sub
URBAN_EXTENT_KM_MAX     = 6     // urban feeder growth radius (compactness guard)
GROUP_SPREAD_KM_MAX     = 16    // max bbox diagonal of one sub's feeder group
MAX_FEEDER_KM           = 20    // trunk: sub → farthest member by road
MAX_FOREIGN_CROSSING_M  = 2500  // transit on another sub's actual network
MAX_REPAIR_PASSES       = 3     // bounded validate/repair loop
```

Rule checks in the Checks panel marked by these caps are **tunable**: a
residual after the bounded repair loop is honest reporting for you to
tune, and does not gate the selftest — correctness checks (conservation,
contiguity, membership honoured, monotonicity, fault conservation) do.

9. **Reliability** ([js/reliability.js](js/reliability.js)) — per-feeder
   SAIDI with separate overhead/underground fault rates (defaults 0.08 /
   0.02 faults·km⁻¹·yr⁻¹, both adjustable live in the UI); outage = crew
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
- **Zone sub summary** — per-sub table (worst first, live): feeder count,
  customers, total HV feeder length, customer-weighted SAIDI under the
  current devices and fault rates. The "road vs line" map layer still
  draws crew road route vs straight line for a selected section.
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
- Seed input, town/inland sliders (customers fixed at 25 000, zone subs
  fixed at 18), eleven layer toggles, click-a-feeder stats, pan/zoom.

## Correctness guards (asserted and reported in the Checks panel)

- Road graph fully connected (after repair; merge count reported).
- Every customer served by exactly one feeder; feeder totals conserve the
  population; no customers in water.
- Roads cross the river **only** at generated bridges (exact traversal).
- Baseline SAIDI finite and positive.
- Greedy running SAIDI monotone non-increasing.
- Fault classification conserves customers (no double-count).
- **Membership honoured by routing**: every load node lands on its own
  feeder, every feeder is a single contiguous subtree with one root, no
  orphan TXs.
- Subtransmission is visual-only (rebuilding it changes no numbers).
- Plus the tunable rule checks (caps, trunk/extent, foreign transit,
  feeders-per-sub, repair-loop convergence) — reported honestly, gated
  separately (see Membership caps above).

## Design choices

Plausible-not-optimal throughout: greedy clustering, greedy switch
placement, MST arterials — real networks accrete, they aren't optimised.
Colours follow a validated colour-blind-safe categorical palette (fixed
slot order) with reserved status colours for fault states.
