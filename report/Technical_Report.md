# Strategy Engine for UAV Fleet Deployment: A Genetic Algorithm Approach to Multi-Objective Mission Planning

**Course:** Artificial Intelligence (BSCS 5) — Assignment 2
**Institution:** Bahria University, Lahore Campus
**Submitted by:** Muhammad Rayan Bin Waqar — Enrollment No. 03-134242-067

---

## 1. Introduction

Disaster-response and surveillance missions increasingly rely on fleets of
Unmanned Aerial Vehicles (UAVs) that must survey a hazard area quickly,
thoroughly, and safely. Planning such a mission is a multi-objective
optimization problem: the fleet should maximize coverage of high-priority
disaster zones and target detection, while simultaneously minimizing energy
consumption, mission time, and hazard exposure — without violating hard
airspace constraints such as no-fly zones and maximum flight range. These
objectives conflict (covering more area costs more energy), so there is no
single "correct" solution, only trade-off solutions of differing quality.

This report presents a **Strategy Engine** based on a **Genetic Algorithm
(GA)** that evolves complete mission plans for a fleet of 3 UAVs operating on
a discretized 50 × 50 hazard grid. We describe the chromosome representation,
the multi-objective fitness function, the custom genetic operators, and the
constraint-handling strategy, and we evaluate the system through (i) a
comparison against a random-search baseline with an equal evaluation budget
and (ii) a hyperparameter sensitivity analysis over population size, mutation
rate, and selection pressure.

## 2. System Design

### 2.1 Environment model

The operational area is a 50 × 50 grid. Each of the 3 drones launches from its
own base station and must return to it. The grid contains:

- **Two priority zones** (rectangles) with weights 3 and 2 — the disaster
  areas whose coverage is rewarded most.
- **12 detection targets**, mostly clustered in and around the priority zones.
- **Three no-fly zones** (rectangles). One deliberately overlaps a corner of
  the main priority zone, forcing the planner to earn coverage around a
  forbidden region rather than camping inside it.
- **Three hazard pockets** (circles) representing high wind, radiation, and
  turbulence, with severities 0.8, 0.6, and 0.5.
- A sensor radius of 4 cells and a maximum per-drone range of 420 units.

### 2.2 Chromosome representation

A chromosome encodes one complete mission plan for the whole fleet as a
**hybrid three-segment encoding**:

- **Segment A (real-valued): waypoints.** For each drone, 8 waypoints
  (x, y) — the scan positions it visits in order. The full route is
  base → w₁…w₈ → base (9 legs).
- **Segment B (real-valued): speeds.** One speed value per leg, in
  [0.5, 2.0], controlling the energy/time trade-off of each leg.
- **Segment C (binary): sensor schedule.** One bit per waypoint: 1 means the
  drone activates its sensor at that waypoint and scans a disc of radius 4;
  0 means it transits without scanning (saving sensor energy).

**Rationale.** Segmenting by *function* (where to go, how fast, when to scan)
keeps each gene's meaning stable under the genetic operators, so crossover
exchanges meaningful building blocks rather than arbitrary numbers.
Real-valued coordinates avoid the quantization error of binary-encoded
positions, while the binary sensor segment matches the payload's on/off
nature. Encoding the whole fleet in one chromosome lets fitness reward
*cooperation* — drones dividing the priority zones instead of duplicating
each other's coverage.

### 2.3 Multi-objective fitness function

Fitness is a weighted sum of normalized rewards and costs, minus penalties:

> Fitness = 0.35·Coverage + 0.25·Detection − 0.15·Energy − 0.10·Time
> − 0.15·Hazard − Penalties

- **Coverage** — the fraction of priority-zone cells scanned, weighted by
  zone priority (weight-3 cells count triple).
- **Detection** — the fraction of the 12 targets falling inside at least one
  active sensor disc.
- **Energy** — Σ over legs of distance × (0.5 + 0.5·speed²), plus 15 units
  per active scan. The quadratic speed term models the physical reality that
  flying faster costs disproportionately more energy, and prevents the GA
  from trivially maximizing speed on every leg.
- **Time** — the makespan: the slowest drone's total flight time, since the
  mission ends when the last drone returns.
- **Hazard** — hazard severity accumulated over every sampled point (≈1 unit
  apart) of every flight path.

**Weight rationale.** Coverage (0.35) and detection (0.25) dominate because
the mission's purpose is information gathering in a disaster zone; hazard
avoidance (0.15) and energy (0.15) are significant but secondary; time (0.10)
matters least because all drones fly concurrently. All cost terms are divided
by reference maxima so every component lives on a comparable 0–1 scale and
the weights express genuine priorities rather than unit artefacts.

### 2.4 Genetic operators

- **Selection — tournament (size 3).** Three individuals are drawn at random
  and the fittest becomes a parent. Tournament size directly controls
  selection pressure and is varied in the sensitivity analysis.
- **Crossover — segment-swap (rate 0.9).** For each drone, the child inherits
  that drone's *entire* block (waypoints + speeds + sensors) from one parent
  or the other. Whole blocks are exchanged because a good per-drone route is
  a coherent building block; scrambling waypoints between two routes would
  destroy the route geometry that made each parent fit. Speeds and sensor
  bits inside kept blocks receive light uniform mixing.
- **Mutation (rate 0.08 per gene).** Waypoint coordinates receive a Gaussian
  nudge (σ = 2 cells) — small local exploration that refines routes instead
  of teleporting them; speeds jitter by N(0, 0.2); sensor bits flip.
  Mutation is the diversity engine that prevents premature convergence.
- **Elitism.** The top 2 chromosomes are copied unchanged into the next
  generation, so the best plan found is never lost.
- **Termination.** 100 generations, or 25 consecutive generations without
  improvement of the best fitness (stagnation).

### 2.5 Constraint handling

Hard constraints are handled by a **penalty + repair** combination:

- **Penalty function.** Every sampled path point inside a no-fly zone costs
  0.05 fitness; every unit of range beyond the 420 maximum costs 0.01. A
  typical random plan accumulates ≈ 88 no-fly points (penalty ≈ 4.4), which
  dwarfs the maximum achievable reward (≈ 0.6) — so infeasible plans are
  strongly selected against, yet not instantly deleted, allowing the
  population to cross infeasible regions early in evolution while it learns
  the airspace.
- **Repair mechanism.** After every crossover/mutation, all waypoint
  coordinates are clamped into the grid bounds, guaranteeing every offspring
  is structurally valid.

## 3. Experimental Setup

Baseline configuration: population 50, generations 100, mutation rate 0.08,
tournament size 3, crossover rate 0.9, elitism 2, fixed random seed for
reproducibility. The random-search baseline samples uniformly random
chromosomes and keeps the best, given an **equal budget of 5,000 fitness
evaluations** (50 × 100), so the comparison measures the value of evolution
itself rather than of extra computation.

## 4. Results: Evolved Solution vs Random Search

| Metric | GA (evolved) | Random search |
|---|---|---|
| Best fitness | **0.2435** | −0.7851 |
| Priority coverage | **0.793** | 0.309 |
| Targets detected | **9 / 12** | 4 / 12 |
| Energy | **989.8** | 1169.0 |
| Mission time (makespan) | **199.4** | 216.5 |
| Hazard exposure | **22.4** | 55.0 |
| No-fly violations | **0** | 13 |

The convergence curve (best vs average fitness per generation) shows the
classic GA profile: a steep climb in the first ~20 generations as the
population escapes the heavily penalized random region, then progressively
finer improvements as routes are refined, with the average fitness rising
steadily behind the best — evidence that improvement spreads through the
population rather than resting on a single lucky individual.

*Figure 1: [Insert results/convergence.png — fitness convergence curve]*

*Figure 2: [Insert results/best_paths.png — evolved flight paths]*

*Figure 3: [Insert results/heatmap_evolution.png — coverage heatmap across generations]*

Three qualitative findings stand out:

1. **Constraint learning.** The evolved plan has *zero* no-fly violations,
   while even the best of 5,000 random plans still violates them 13 times.
   The paths visibly bend around the no-fly rectangles — including the one
   overlapping the main priority zone — an emergent behaviour produced purely
   by penalty pressure, with no route-planning logic coded anywhere.
2. **Coverage specialization.** The evolved fleet covers 79% of the weighted
   priority cells, ≈ 2.6× the baseline, with drones distributing themselves
   across the two priority zones instead of overlapping.
3. **Balanced trade-offs.** The GA solution is simultaneously better on
   *every* cost metric (energy, time, hazard) — it did not buy coverage by
   spending recklessly; evolution found plans that dominate the baseline on
   all objectives at once.

**Uniqueness.** The final solution is one point in a large family of
near-optimal trade-off plans: a different seed produces a different route
layout with similar fitness, because the weighted-sum fitness defines a
*preference*, not a unique optimum — a feature of multi-objective problems.
Random search, by contrast, never reaches this region of the fitness
landscape at all within the same budget.

## 5. Hyperparameter Sensitivity Analysis

Each hyperparameter was varied individually around the baseline, with 3 runs
per setting (different seeds); values below are mean best fitness ± std.

### 5.1 Population size

| Population | Mean best fitness |
|---|---|
| 20 | 0.166 ± 0.063 |
| 50 | 0.260 ± 0.041 |
| 100 | 0.269 ± 0.054 |

Increasing the population from 20 to 50 improves mean best fitness
substantially (0.166 → 0.260): a larger population maintains more diverse
building blocks and samples more of the search space per generation. Doubling
again to 100 yields only +0.009 — well within run-to-run variation — at twice
the computational cost, indicating diminishing returns beyond 50. One
confound must be acknowledged: with generations fixed at 100, smaller
populations also receive fewer total fitness evaluations (2,000 vs 5,000 vs
10,000), so part of the population effect is a budget effect. Even so, the
flattening between 50 and 100 shows the baseline population of 50 is an
efficient operating point.

*Figure 4: [Insert results/sensitivity_population_size.png]*

### 5.2 Mutation rate

| Mutation rate | Mean best fitness |
|---|---|
| 0.01 | 0.173 ± 0.072 |
| 0.05 | 0.124 ± 0.049 |
| 0.08 | 0.260 ± 0.041 |
| 0.15 | 0.221 ± 0.048 |
| 0.30 | 0.155 ± 0.086 |

Mutation rate was the most sensitive hyperparameter, following the classic
inverted-U pattern. At very low rates (0.01–0.05) the population converges
prematurely: once the initial diversity is consumed by selection and
crossover, almost no new genetic material appears, and the search stalls at
mediocre local optima (0.12–0.17). The baseline rate of 0.08 performed best
(0.260) — high enough to keep routes exploring, low enough to refine them.
At 0.15 performance degrades moderately (0.221), and at 0.30 it collapses
(0.155, with the largest variance): with nearly a third of all genes perturbed
every generation, good building blocks are destroyed faster than selection
can accumulate them and the search degenerates toward random walk. (The
0.05 dip below 0.01 is within overlapping standard deviations and should be
read as noise around the low-rate plateau rather than a real reversal; the
robust findings are the peak at 0.08 and the collapse at 0.30.)

*Figure 5: [Insert results/sensitivity_mutation_rate.png]*

### 5.3 Selection pressure (tournament size)

| Tournament size | Mean best fitness |
|---|---|
| 2 | 0.164 ± 0.044 |
| 3 | 0.260 ± 0.041 |
| 5 | 0.271 ± 0.035 |
| 7 | 0.252 ± 0.056 |

Tournament size 2 applies very weak selection pressure — parents are chosen
almost at random, so good plans do not reliably reproduce and fitness stalls
at 0.164. Raising the size to 3 (baseline) improves results sharply (0.260),
and size 5 is marginally better still (0.271) with the lowest variance of any
setting (± 0.035), i.e. the most consistent runs. At size 7 the mean slips
slightly (0.252) and variance rises: the same few champions win nearly every
tournament, diversity drains, and outcomes depend more heavily on the initial
population. The pattern confirms the theory — moderate-to-strong pressure
helps, excessive pressure begins to cost diversity — and shows the response
is a broad plateau (3–7 all ≈ 0.25–0.27) rather than a sharp peak, so the GA
is robust to this knob once it is not at the weakest setting.

*Figure 6: [Insert results/sensitivity_tournament_size.png]*

## 6. Limitations

The model uses straight-line legs between waypoints (no turning-radius
dynamics), a static hazard map, and a weighted-sum scalar fitness, which
cannot represent concave trade-off fronts the way a Pareto method such as
NSGA-II could. Coverage is credited at waypoint discs rather than
continuously along the path. These simplifications suit a design-stage
strategy engine and keep evaluation fast enough for thousands of fitness calls.

## 7. Conclusion

A generational GA with a hybrid segment encoding, a weighted multi-objective
fitness function, block-preserving crossover, Gaussian mutation, and
penalty-plus-repair constraint handling evolved UAV mission plans that
strictly dominate an equal-budget random search: 0.79 vs 0.31 coverage,
9 vs 4 targets detected, and zero vs 13 airspace violations. The sensitivity
analysis (Section 5) shows the baseline configuration is already near the
optimum: mutation rate is the knob that matters most (0.08, with collapse at
both extremes), population 50 captures nearly all the benefit of larger
populations at half the cost of 100, and tournament sizes 3–5 form a broad,
stable plateau for selection pressure. The system demonstrates that evolutionary
search is a practical tool for mission planning problems where objectives
conflict and no exact solution method exists.
