"""
operators.py - Genetic operators tailored to the hybrid chromosome.

Selection : tournament selection (size = selection pressure knob)
Crossover : segment-swap crossover - children inherit whole drone
            mission blocks from each parent, plus uniform mixing of
            speeds and sensor flags within a block
Mutation  : Gaussian perturbation of waypoint coordinates, small
            random resets of speeds, bit-flips of sensor flags
Repair    : coordinates are clamped into the grid after every
            operation, so offspring are always structurally valid
"""

import random

from chromosome import Chromosome
from environment import Environment


def tournament_select(population, tournament_size):
    """Pick `tournament_size` random individuals, return the fittest."""
    contenders = random.sample(population, tournament_size)
    return max(contenders, key=lambda c: c.fitness)


def segment_swap_crossover(parent1, parent2):
    """Exchange whole drone mission blocks between two parents.

    For each drone: with 50% chance the child takes that drone's entire
    block (waypoints + speeds + sensors) from parent 2 instead of
    parent 1. Within a kept block nothing is scrambled, so good
    per-drone routes survive crossover intact.
    """
    child1 = parent1.copy()
    child2 = parent2.copy()
    for d in range(Chromosome.NUM_DRONES):
        if random.random() < 0.5:
            # swap drone d's whole block between the children
            child1.waypoints[d] = [tuple(w) for w in parent2.waypoints[d]]
            child1.speeds[d] = list(parent2.speeds[d])
            child1.sensors[d] = list(parent2.sensors[d])
            child2.waypoints[d] = [tuple(w) for w in parent1.waypoints[d]]
            child2.speeds[d] = list(parent1.speeds[d])
            child2.sensors[d] = list(parent1.sensors[d])
        else:
            # uniform mixing of speeds/sensors inside the block
            for j in range(Chromosome.NUM_LEGS):
                if random.random() < 0.5:
                    child1.speeds[d][j], child2.speeds[d][j] = \
                        child2.speeds[d][j], child1.speeds[d][j]
            for i in range(Chromosome.NUM_WAYPOINTS):
                if random.random() < 0.5:
                    child1.sensors[d][i], child2.sensors[d][i] = \
                        child2.sensors[d][i], child1.sensors[d][i]
    return child1, child2


def mutate(chrom, mutation_rate, sigma=2.0):
    """In-place mutation; each gene mutates with prob. mutation_rate.

    - waypoint coordinate: Gaussian nudge (sigma cells)
    - speed gene: Gaussian nudge, clamped to [SPEED_MIN, SPEED_MAX]
    - sensor flag: bit flip
    Afterwards a repair step clamps all coordinates into the grid.
    """
    size = Environment.GRID_SIZE - 1
    for d in range(Chromosome.NUM_DRONES):
        for i in range(Chromosome.NUM_WAYPOINTS):
            if random.random() < mutation_rate:
                x, y = chrom.waypoints[d][i]
                x += random.gauss(0, sigma)
                y += random.gauss(0, sigma)
                chrom.waypoints[d][i] = (x, y)
            if random.random() < mutation_rate:
                chrom.sensors[d][i] = 1 - chrom.sensors[d][i]
        for j in range(Chromosome.NUM_LEGS):
            if random.random() < mutation_rate:
                s = chrom.speeds[d][j] + random.gauss(0, 0.2)
                chrom.speeds[d][j] = max(Chromosome.SPEED_MIN,
                                         min(Chromosome.SPEED_MAX, s))
    repair(chrom)
    return chrom


def repair(chrom):
    """Repair mechanism: clamp every waypoint into the grid bounds."""
    size = Environment.GRID_SIZE - 1
    for d in range(Chromosome.NUM_DRONES):
        for i in range(Chromosome.NUM_WAYPOINTS):
            x, y = chrom.waypoints[d][i]
            chrom.waypoints[d][i] = (max(0.0, min(size, x)),
                                     max(0.0, min(size, y)))
    return chrom
