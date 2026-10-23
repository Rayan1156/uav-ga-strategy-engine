"""
main.py - Entry point: run the GA Strategy Engine, run the random-search
baseline with an equal evaluation budget, and produce all visualizations.

Usage:  python main.py
Outputs (in ../results/):
  convergence.png, best_paths.png, initial_paths.png,
  heatmap_final.png, heatmap_evolution.png
Prints a summary comparison of GA vs random search.
"""

import os
import random

from environment import Environment
from fitness import FitnessEvaluator
from genetic_algorithm import GeneticAlgorithm, RandomSearch
import visualize

RESULTS_DIR = os.path.join(os.path.dirname(__file__), "..", "results")
os.makedirs(RESULTS_DIR, exist_ok=True)

POP_SIZE = 50
GENERATIONS = 100
MUTATION_RATE = 0.08
TOURNAMENT_SIZE = 3
SEED = 42


def main():
    env = Environment()
    evaluator = FitnessEvaluator(env)

    # ---- GA run (snapshots kept for the heatmap-evolution figure) ----
    print("Running Genetic Algorithm...")
    ga = GeneticAlgorithm(evaluator, population_size=POP_SIZE,
                          generations=GENERATIONS,
                          mutation_rate=MUTATION_RATE,
                          tournament_size=TOURNAMENT_SIZE, seed=SEED)
    best = ga.run()
    print(f"GA best fitness: {best.fitness:.4f}")
    print(f"GA metrics: {best.metrics}")
    print(f"Generations run: {len(ga.history_best)}")

    # ---- Random search baseline, equal budget ----
    budget = POP_SIZE * GENERATIONS
    print(f"\nRunning random search (budget = {budget} evaluations)...")
    rs = RandomSearch(evaluator, total_evaluations=budget, seed=SEED)
    rs_best = rs.run()
    print(f"Random search best fitness: {rs_best.fitness:.4f}")
    print(f"Random search metrics: {rs_best.metrics}")

    # ---- Visualizations ----
    visualize.plot_convergence(
        ga.history_best, ga.history_avg,
        os.path.join(RESULTS_DIR, "convergence.png"),
        random_best=rs.history_best)
    visualize.plot_paths(best, os.path.join(RESULTS_DIR, "best_paths.png"),
                         title=f"Evolved UAV Flight Paths (fitness "
                               f"{best.fitness:.3f})")
    visualize.plot_paths(rs_best,
                         os.path.join(RESULTS_DIR, "random_paths.png"),
                         title=f"Random Search Best Paths (fitness "
                               f"{rs_best.fitness:.3f})")

    # heatmap of the final solution's coverage
    final_counts = visualize.coverage_counts_over_run([best])
    visualize.plot_heatmap(final_counts,
                           os.path.join(RESULTS_DIR, "heatmap_final.png"),
                           title="Coverage Heatmap - Final Evolved Solution")

    # heatmap accumulated across generations (snapshots + final best)
    evo_counts = visualize.coverage_counts_over_run(ga.snapshots + [best])
    visualize.plot_heatmap(
        evo_counts,
        os.path.join(RESULTS_DIR, "heatmap_evolution.png"),
        title="Coverage Heatmap - Accumulated Across Generations")
    print("\nPlots saved to results/")


if __name__ == "__main__":
    main()
