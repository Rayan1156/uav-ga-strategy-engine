# UAV Strategy Engine — Genetic Algorithm Mission Planner

A Genetic Algorithm (GA) **Strategy Engine** that evolves deployment plans
for a fleet of **3 UAVs** surveying a volatile 50 × 50 hazard grid. The
engine balances competing objectives — priority-zone coverage and target
detection versus energy consumption, mission time, and hazard exposure —
while respecting hard constraints (no-fly zones, maximum flight range).

Built for **Artificial Intelligence (BSCS 5), Assignment 2**,
Bahria University, Lahore Campus.

## Problem Summary

Unlike single-solution algorithmic problems, UAV mission planning is a
**multi-objective optimization** problem with no single correct answer.
This project designs:

1. **A hybrid chromosome representation** — three segments encoding
   waypoints (real-valued), per-leg speeds (real-valued), and sensor
   on/off schedules (binary) for the whole fleet.
2. **A multi-objective fitness function** — weighted sum of coverage and
   detection rewards minus energy, time, and hazard costs.
3. **Custom genetic operators** — tournament selection, segment-swap
   (block) crossover that preserves whole per-drone routes, and Gaussian
   mutation tuned for spatial coordinates.
4. **Constraint handling** — penalty functions for no-fly/range violations
   plus a repair mechanism that clamps coordinates into the grid.

## Project Structure

```
uav-ga-strategy-engine/
├── README.md
├── requirements.txt
├── src/
│   ├── environment.py        # 50x50 grid: zones, targets, hazards, no-fly
│   ├── chromosome.py         # hybrid multi-segment chromosome
│   ├── fitness.py            # multi-objective fitness evaluator
│   ├── operators.py          # selection, crossover, mutation, repair
│   ├── genetic_algorithm.py  # GA engine + random-search baseline
│   ├── visualize.py          # convergence / path / heatmap plots
│   ├── main.py               # GA vs random search + all visualizations
│   └── sensitivity.py        # hyperparameter sensitivity analysis
└── results/                  # generated plots (PNG)
```

## How to Run

```bash
pip install -r requirements.txt
cd src
python main.py          # GA run + random-search baseline + plots
python sensitivity.py   # hyperparameter sensitivity sweeps
```

Outputs are written to `../results/`:

- `convergence.png` — best vs average fitness over generations
- `best_paths.png` — evolved flight paths over the hazard grid
- `random_paths.png` — best random-search paths (baseline)
- `heatmap_final.png` — coverage heatmap of the final solution
- `heatmap_evolution.png` — coverage accumulated across generations
- `sensitivity_*.png` — fitness vs population size / mutation rate /
  tournament size

## Key Results (seed 42)

| Metric | GA (evolved) | Random search (equal budget) |
|---|---|---|
| Best fitness | **0.2435** | −0.7851 |
| Priority coverage | **0.79** | 0.31 |
| Targets detected | **9 / 12** | 4 / 12 |
| No-fly violations | **0** | 13 |

## Configuration

Baseline hyperparameters (in `main.py` / `sensitivity.py`): population 50,
generations 100, mutation rate 0.08, tournament size 3, crossover rate 0.9,
elitism 2, stagnation patience 25.

See the technical report (`report/`) for the design rationale, fitness
weight justification, and the full sensitivity analysis.
