"""
sensitivity.py - Hyperparameter sensitivity analysis.

Varies ONE hyperparameter at a time around the baseline configuration
(population 50, mutation 0.08, tournament 3, generations 100), runs each
setting 3 times with different seeds, and plots mean best fitness.

Usage:  python sensitivity.py
Outputs: ../results/sensitivity_population.png,
         ../results/sensitivity_mutation.png,
         ../results/sensitivity_tournament.png
Also prints a results table for the report.
"""

import os
import statistics

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from environment import Environment
from fitness import FitnessEvaluator
from genetic_algorithm import GeneticAlgorithm

RESULTS_DIR = os.path.join(os.path.dirname(__file__), "..", "results")
os.makedirs(RESULTS_DIR, exist_ok=True)

BASE = dict(population_size=50, generations=100, mutation_rate=0.08,
            tournament_size=3)
SEEDS = [1, 2, 3]


def run_setting(**overrides):
    config = dict(BASE)
    config.update(overrides)
    env = Environment()
    evaluator = FitnessEvaluator(env)
    scores = []
    for seed in SEEDS:
        ga = GeneticAlgorithm(evaluator, seed=seed, **config)
        best = ga.run()
        scores.append(best.fitness)
    return statistics.mean(scores), statistics.pstdev(scores)


def sweep(name, param, values):
    print(f"\n--- Sensitivity: {name} ---")
    means = []
    for v in values:
        mean, sd = run_setting(**{param: v})
        means.append(mean)
        print(f"{param} = {v}: mean best fitness {mean:.4f} (+/- {sd:.4f})")
    plt.figure(figsize=(7, 4.5))
    plt.plot(values, means, marker="o")
    plt.xlabel(name)
    plt.ylabel("Mean best fitness (3 runs)")
    plt.title(f"Sensitivity to {name}")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(RESULTS_DIR, f"sensitivity_{param}.png"),
                dpi=150)
    plt.close()
    return means


if __name__ == "__main__":
    sweep("Population size", "population_size", [20, 50, 100])
    sweep("Mutation rate", "mutation_rate", [0.01, 0.05, 0.08, 0.15, 0.30])
    sweep("Tournament size (selection pressure)", "tournament_size",
          [2, 3, 5, 7])
    print("\nDone. Plots saved to results/")
