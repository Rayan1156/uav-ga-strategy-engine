"""
genetic_algorithm.py - The Strategy Engine: a generational GA with
elitism, tournament selection, segment-swap crossover and Gaussian
mutation. Termination: fixed generation budget OR stagnation of the
best fitness for `patience` consecutive generations.
"""

import random

from chromosome import Chromosome
from operators import tournament_select, segment_swap_crossover, mutate


class GeneticAlgorithm:
    def __init__(self, evaluator, population_size=50, generations=100,
                 mutation_rate=0.08, crossover_rate=0.9, tournament_size=3,
                 elite_count=2, patience=25, seed=None):
        self.evaluator = evaluator
        self.population_size = population_size
        self.generations = generations
        self.mutation_rate = mutation_rate
        self.crossover_rate = crossover_rate
        self.tournament_size = tournament_size
        self.elite_count = elite_count
        self.patience = patience
        if seed is not None:
            random.seed(seed)
        self.history_best = []
        self.history_avg = []
        self.best = None
        self.snapshots = []

    def initialize_population(self):
        pop = [Chromosome.random() for _ in range(self.population_size)]
        for c in pop:
            self.evaluator.evaluate(c)
        return pop

    def run(self):
        population = self.initialize_population()
        stagnant = 0
        best_so_far = None

        for gen in range(self.generations):
            population.sort(key=lambda c: c.fitness, reverse=True)
            gen_best = population[0]
            avg = sum(c.fitness for c in population) / len(population)
            self.history_best.append(gen_best.fitness)
            self.history_avg.append(avg)
            if gen % 25 == 0:
                self.snapshots.append(gen_best.copy())

            if best_so_far is None or gen_best.fitness > best_so_far.fitness:
                best_so_far = gen_best.copy()
                best_so_far.fitness = gen_best.fitness
                best_so_far.metrics = dict(gen_best.metrics)
                stagnant = 0
            else:
                stagnant += 1
            if stagnant >= self.patience:
                break

            # next generation: elites survive unchanged
            next_pop = [c.copy() for c in population[:self.elite_count]]
            while len(next_pop) < self.population_size:
                p1 = tournament_select(population, self.tournament_size)
                p2 = tournament_select(population, self.tournament_size)
                if random.random() < self.crossover_rate:
                    child1, child2 = segment_swap_crossover(p1, p2)
                else:
                    child1, child2 = p1.copy(), p2.copy()
                mutate(child1, self.mutation_rate)
                mutate(child2, self.mutation_rate)
                self.evaluator.evaluate(child1)
                self.evaluator.evaluate(child2)
                next_pop.append(child1)
                if len(next_pop) < self.population_size:
                    next_pop.append(child2)
            population = next_pop

        self.best = best_so_far
        return best_so_far


class RandomSearch:
    """Baseline: sample random chromosomes, keep the best.
    Uses the same total evaluation budget as the GA it is compared to."""

    def __init__(self, evaluator, total_evaluations=5000, seed=None):
        self.evaluator = evaluator
        self.total_evaluations = total_evaluations
        if seed is not None:
            random.seed(seed)
        self.history_best = []
        self.best = None

    def run(self):
        best = None
        for i in range(self.total_evaluations):
            c = Chromosome.random()
            self.evaluator.evaluate(c)
            if best is None or c.fitness > best.fitness:
                best = c
            if (i + 1) % 100 == 0:
                self.history_best.append(best.fitness)
        self.best = best
        return best
