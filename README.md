# NZ-style Distribution Network Sandbox

**TOY — not a trusted tool.** A procedural, seeded sandbox that invents a
~100 × 70 km NZ-flavoured patch of geography (landscape aspect to suit the
browser viewport), settles it, roads it, wires it up with a radial
11 kV-ish distribution network, and lets you play with reliability (SAIDI,
sectionaliser placement, fault playback). Everything is invented — no real
data — and every modelling assumption is labelled in the UI Assumptions
panel and in the source. NZ English, metric units.

## Run it

Any static file server works (ES modules need `http://`, not `file://`):

```powershell
cd nz-grid-sandbox
python -m http.server 8317
# then open http://localhost:8317/
```

Optional URL modes:
- `?selftest=1` — headless self-test: generates three seeds at the fixed
  50 000 sampled customers, runs every correctness check (emergent sub/feeder
  counts sanity-banded), greedy placement of both device kinds
  (monotonicity asserted), the debug rate experiment, and
  fault-conservation checks, and prints a JSON report into the page.
- `?demo=1` — regenerates, places 8 sectionalisers + 4 reclosers, and
  freezes a fault mid-timeline (handy for screenshots).
- `?scaletest=1` — one seed at 8k→64k customers with timing + checks.

## Scaling

Measured (Windows, headless Edge, 100 × 70 km map): all **correctness**
checks pass and greedy stays monotone up to at least **64,000 customers**
(regen ≈ 0.6–4.7 s across 8k–64k; the fixed 50k UI world regenerates in
≈ 3–11 s depending on seed, including the prune rebuilds). The UI samples
a fixed 50,000 customers; other sizes work programmatically via
`generate()` / `?scaletest=1` (settlement targets are SHARES of the
sampled count, so the hierarchy scales with it — measured 89 feeders at
8k up to 212 at 64k, after pruning).

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
2. **Settlements** ([js/density.js](js/density.js)) — an EXPLICIT
   three-tier hierarchy, targets as shares of the sampled customers (so at
   50k): ONE large town ≈ 40% (~20,000), each small town 2–4%
   (1,000–2,000; count set by the slider), and 6–10 rural settlements at
   0.4–1% (200–500). Kernel weights are computed so realised counts hit
   the targets, and the rural background gets the remainder. Sites scored
   on flatness, a coastal↔river blend (Inland slider), river-mouth
   proximity, and junctions of a provisional least-cost corridor skeleton
   (anchors ↔ map-edge exits) that later seeds the arterials. σ ∝
   √customers (uniform peak density), spacing scaled to town size, both
   river banks settled.
3. **Roads** ([js/roads.js](js/roads.js)) — inter-town network = corridor
   skeleton + MST over ALL towns and settlements + 2-nearest-neighbour
   links (within 45 km), so the towns connect in a WEB with loop routes,
   not a bare tree; links touching a rural settlement are collectors,
   town↔town links arterials. All A* with slope² cost (sea blocked; first
   river crossing builds a bridge at 28×, then reused). Urban streets: an
   ORGANIC lattice
   — spacing scales with population, orientation snaps to the local
   coast/valley axis (never global north), every point wobbles, fraying
   starts near the core, and ~7% of streets drop out as dead ends — so
   towns read grid-ISH, not stamped. Rural roads: spurs off arterials
   every ~1–1.6 km into flat country (many fork), then nearby dead ends
   are LINKED into a sparse web of back roads. Exact grid traversal for
   all water tests; connectivity repair.
4. **Load** ([js/customers.js](js/customers.js)) — density = mass-
   compensated town kernels (Gaussian core + exponential shoulder, so
   density eases through a peri-urban fringe rather than cliff-edging at
   the town boundary; a sea-clipped town gets denser, keeping realised
   sizes rank-ordered) + rural background decaying with ROAD distance +
   an off-road shoulder (farms up to ~6 km past the end of the road,
   reached by line easements); **50 000 customers (fixed in the UI)**
   sampled on buildable land, snapped to the road graph. Customers landing
   on feeders that end up under 20 customers are PRUNED (see feeder cuts),
   so the served count emerges slightly lower (measured ≈ 49,700–49,850).
5. **Transformers** ([js/network.js](js/network.js)) — the TX sits AT a
   seed customer (a fixed pole site) and gathers neighbours within
   `TX_MAX_M = 500` m, up to `TX_MAX_CUST = 100` — so the 500 m rule holds
   **exactly by construction**. Rural TXs max out on distance, urban on
   count (≈ 4,900–6,200 TXs at 50 000 customers). A TX more than 400 m
   from any road connects by a **line easement** — a straight
   cross-country span to the nearest road node or a nearer easement node
   (spans daisy-chain up remote valleys) — so rural feeders may leave the
   road corridor. Easements never span sea or lakes; river spans are
   allowed (towers, not bridges) and are exempt from the bridges-only
   road rule.
6. **Zone subs** ([js/network.js](js/network.js), [js/membership.js](js/membership.js)) —
   TXs cluster by road-Dijkstra growth (`SUB_MAX_KM = 50` by road or
   `SUB_MAX_CUST = 4000`, no minimums); each sub is sited at its cluster's
   **load-weighted centroid**, nudged to the nearest subtransmission-viable
   road node. Every TX then joins its road-nearest sub via an
   **additively weighted graph Voronoi**: one multi-source Dijkstra whose
   sources start at a per-sub offset λ, so the shortest-path forest is
   DISJOINT per sub by construction; over-cap subs bid their boundary out,
   starved subs bid back in, and a cell that stays over cap **spawns a new
   sub** in its far half; a busbar branch that overflows the feeder caps
   spawns one too, and stations that end up within 6 km of each other are
   MERGED when their combined load fits a couple of feeders — the sub
   count truly emerges (measured ≈ 35 on the 100 × 70 km map at 50k customers).
7. **Feeders + prune** ([js/network.js](js/network.js),
   [js/main.js](js/main.js)) — feeders are the BUSBAR BRANCHES of each
   sub's shortest-path tree, one per branch: **no deep cuts and no
   express runs** (no parallel circuits sharing a trunk). A branch that
   would overflow `FEEDER_MAX_CUST = 1000` customers or reach farther
   than `FEEDER_MAX_KM = 50` (line distance sub → farthest point; total
   conductor is unbounded, like real rural feeders) instead spawns a new
   sub at the branch's far-half load centroid during the assignment loop
   — so stations sit central to the feeders they serve. Residual
   violations (a dense-town road carrying > 1000 on one branch) are
   reported, never hidden. A feeder still carrying fewer than
   `FEEDER_MIN_CUST = 20` customers is PRUNED — the feeder, its
   transformers AND its customers are removed (uneconomic to reticulate)
   and the network is rebuilt from the survivors. Measured 88–105
   feeders (mean ≈ 475–570 customers each) at 50 000 customers.
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
MAX_SUB_CENTROID_KM   = 8    // X: sub too far from its feeder-group load centroid
MAX_SUBTX_STRAIGHT_KM = 15   // Y: subtx line straight for too long
MIN_GRID_SPREAD_DEG   = 15   // all town grids sharing one orientation
MIN_ZIPF_RATIO        = 2.2  // realised largest/median town size too flat
                             // (the peri-urban kernel spreads big-town mass)
```

### Rule caps ([js/membership.js](js/membership.js) — all tunable, no minimums)

```
TX_MAX_CUST      = 100   // customers per distribution transformer
TX_MAX_M         = 500   // customer → transformer distance (m) — exact
SUB_MAX_CUST     = 4000  // customers per zone sub
SUB_MAX_KM       = 50    // transformer → zone sub, by road
FEEDER_MAX_CUST  = 1000  // customers per feeder (uniform)
FEEDER_MAX_KM    = 50    // feeder REACH: line distance sub → farthest point
                         // (flat; total conductor unbounded, feeders branch)
FEEDER_MIN_CUST  = 20    // feeders under this are pruned entirely
                         // (with their customers and transformers)
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
   travel from the sub at 50 km/h along the line route + flat 120 min
   repair (both line types — flatters cables, labelled); switching
   restores upstream customers at a flat 45 min. **Backfeed**: adjacent
   feeders share one normally-open TIE (the shortest unused-road corridor
   ≤ 2 km between them, found by a labelled Dijkstra over unused edges);
   every maximal device-bounded subtree in the wait set that reaches a
   tie and does not contain the fault — lateral branches included — is
   re-energised from the neighbouring feeder at the same flat 45 min
   (tie capacity unlimited, neighbour assumed healthy; labelled). Greedy
   candidate gains are EXACT per-feeder re-evaluations, so placement
   stays provably monotone with backfeed in play. There are NO express
   runs — every feeder roots at its sub busbar — so the device-free
   baseline is honestly high (one breaker per branch and nothing else)
   and devices visibly earn their keep.

## Interactive features

- **Greedy devices** — sectionalisers AND reclosers, each with its own
  count input. Every candidate's marginal customer-minutes saved is an
  EXACT per-feeder re-evaluation (benefits are feeder-local), placed
  best-first with recomputation; per-placement log; running SAIDI is
  asserted monotone non-increasing for both kinds.
  - *Sectionaliser*: switching only — upstream-of-switch customers in the
    tripped zone come back at a flat 45 min, and where the switch bounds a
    subtree that reaches a normally-open tie, that subtree BACKFEEDS from
    the neighbouring feeder at 45 min too (for any fault outside it).
  - *Recloser*: ideal protection — faults downstream are cleared by the
    recloser, so upstream customers see no sustained interruption at all;
    doubles as an isolator for backfeed.
    Momentaries (SAIFI/MAIFI), fuse saving and coordination limits are not
    modelled — labelled in the Assumptions panel.
- **Debug rate mode** — doubles λ on one branch (drawn dashed) chosen so the
  greedy's first pick moves, demonstrating the score is rate-weighted; both
  picks are reported.
- **Fault playback** — toggle fault mode, click a line section; timeline
  animates breaker trip → switching (upstream restored, tie-reaching
  subtrees backfed) → repair complete, with the faulted / out-until-repair /
  isolatable-or-backfed / restored classes in distinct status colours.
  Customer conservation (out + isolatable + backfed = affected) is
  asserted per fault.
- **Zone sub summary** — per-sub table (worst first, live): feeder count,
  customers, total HV feeder length, customer-weighted SAIDI under the
  current devices and fault rates. The "road vs line" map layer still
  draws crew road route vs straight line for a selected section.
- **Devices-for-improvement table** — the fewest sectionalisers and the
  fewest reclosers (each kind placed greedily from a device-free network)
  needed to cut network SAIDI by ≥ 10%, 25% and 50%. Greedy stops as soon
  as the deepest target is met; ">N" means the target wasn't reached
  within the placement cap (120 SW / 60 RC, matching the UI inputs) and
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
- Seed input, small-town count + inland sliders (50 000 customers sampled
  into the three-tier settlement hierarchy, sub-20-customer feeders pruned
  with their customers and TXs; sub and feeder counts emerge from the rule
  caps), eleven layer toggles, click-a-feeder stats, pan/zoom.

## Correctness guards (asserted and reported in the Checks panel)

- Road graph fully connected (after repair; merge count reported).
- Every customer served by exactly one feeder; feeder totals conserve the
  population; no customers in water.
- Roads cross the river **only** at generated bridges (exact traversal;
  line easements are power spans, not roads, and are exempt).
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
