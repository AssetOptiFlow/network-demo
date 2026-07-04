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
  25 000 customers, runs every correctness check (emergent sub/feeder
  counts sanity-banded), greedy placement of both device kinds
  (monotonicity asserted), the debug rate experiment, and
  fault-conservation checks, and prints a JSON report into the page.
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
via `generate()` / `?scaletest=1` (sub and feeder counts scale with load
since both emerge from the rule caps).

## Pipeline — strict dependency order (seeded/deterministic)

Layers are produced in order, each consuming only earlier layers:
**terrain → settlements → roads → load → transformers → zone subs (grown,
sited, capacity-balanced) → feeder cuts → subtransmission (visual only)**,
wrapped in a validation pass (below). Sub and feeder counts EMERGE from
simple rule caps — no fixed counts, no minimums.

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
5. **Transformers** ([js/network.js](js/network.js)) — the TX sits AT a
   seed customer (a fixed pole site) and gathers neighbours within
   `TX_MAX_M = 500` m, up to `TX_MAX_CUST = 100` — so the 500 m rule holds
   **exactly by construction**. Rural TXs max out on distance, urban on
   count (≈ 1,050–1,320 TXs at 25 000 customers).
6. **Zone subs** ([js/network.js](js/network.js), [js/membership.js](js/membership.js)) —
   TXs cluster by road-Dijkstra growth (`SUB_MAX_KM = 25` by road or
   `SUB_MAX_CUST = 2000`, no minimums); each sub is sited at its cluster's
   **load-weighted centroid**, nudged to the nearest subtransmission-viable
   road node. Every TX then joins its road-nearest sub via an
   **additively weighted graph Voronoi**: one multi-source Dijkstra whose
   sources start at a per-sub offset λ, so the shortest-path forest is
   DISJOINT per sub by construction; over-cap subs bid their boundary out,
   starved subs bid back in, and a cell that stays over cap **spawns a new
   sub** in its far half — the sub count truly emerges (measured 15–19).
7. **Feeder cuts** ([js/network.js](js/network.js)) — each sub's
   shortest-path road tree is partitioned into contiguous subtrees cut to
   `FEEDER_MAX_CUST = 500` customers and `FEEDER_MAX_KM = 25` circuit
   length (owned + trunk run), relaxed to `FEEDER_LONG_KM = 40` while
   under `FEEDER_LONG_CUST = 50` — a remote valley rides one long skinny
   feeder instead of becoming a micro-sub. Feeder heads reach the sub by
   an express run (the parallel circuit along a shared trunk), charged as
   un-switchable base SAIDI. Measured 75–85 feeders at 25 000 customers.
   Sections in high-density cells are underground cable (drawn dashed),
   the rest overhead.
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

### Rule caps ([js/membership.js](js/membership.js) — all tunable, no minimums)

```
TX_MAX_CUST      = 100   // customers per distribution transformer
TX_MAX_M         = 500   // customer → transformer distance (m) — exact
SUB_MAX_CUST     = 2000  // customers per zone sub
SUB_MAX_KM       = 25    // transformer → zone sub, by road
FEEDER_MAX_CUST  = 500   // customers per feeder (uniform)
FEEDER_MAX_KM    = 25    // feeder circuit length (owned + trunk run)
FEEDER_LONG_KM   = 40    // long-feeder allowance…
FEEDER_LONG_CUST = 50    // …while carrying fewer than this
```

Rules are enforced by construction wherever possible; the residue (sub
totals drifting past the cap at weighted-Voronoi boundaries, realised
distances after siting, irreducible single-node overshoots) is CHECKED
and reported in the Checks panel as **tunable** entries that do not gate
the selftest — correctness checks (conservation, every-TX-mapped,
monotonicity, fault conservation) do. Measured on the test seeds: zero
rule violations.

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
- Seed input, town/inland sliders (customers fixed at 25 000; sub and
  feeder counts emerge from the rule caps), eleven layer toggles,
  click-a-feeder stats, pan/zoom.

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
