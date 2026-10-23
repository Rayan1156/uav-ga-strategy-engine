"""
chromosome.py - Chromosome representation for the UAV Strategy Engine.

Hybrid multi-segment encoding. One chromosome = one complete mission plan
for the whole fleet (3 drones x 8 waypoints):

  Segment A (real-valued): waypoints[d][i] = (x, y) scan positions
  Segment B (real-valued): speeds[d][j]    = speed on leg j of drone d
                           (9 legs: base->w1, w1->w2, ..., w8->base)
  Segment C (binary):      sensors[d][i]   = 1 if the sensor scans at
                           waypoint i of drone d, else 0
"""

import copy
import random

from environment import Environment


class Chromosome:
    NUM_DRONES = 3
    NUM_WAYPOINTS = 8
    NUM_LEGS = NUM_WAYPOINTS + 1          # includes return-to-base leg
    SPEED_MIN, SPEED_MAX = 0.5, 2.0       # speed multiplier range

    def __init__(self, waypoints=None, speeds=None, sensors=None):
        self.waypoints = waypoints      # [drone][waypoint] -> (x, y)
        self.speeds = speeds            # [drone][leg] -> float
        self.sensors = sensors          # [drone][waypoint] -> 0/1
        self.fitness = None
        self.metrics = None             # filled by the fitness evaluator

    # ---------- construction ----------

    @classmethod
    def random(cls):
        """Create a fully random chromosome (valid ranges by design)."""
        size = Environment.GRID_SIZE - 1
        waypoints = [[(random.uniform(0, size), random.uniform(0, size))
                      for _ in range(cls.NUM_WAYPOINTS)]
                     for _ in range(cls.NUM_DRONES)]
        speeds = [[random.uniform(cls.SPEED_MIN, cls.SPEED_MAX)
                   for _ in range(cls.NUM_LEGS)]
                  for _ in range(cls.NUM_DRONES)]
        sensors = [[random.randint(0, 1)
                    for _ in range(cls.NUM_WAYPOINTS)]
                   for _ in range(cls.NUM_DRONES)]
        return cls(waypoints, speeds, sensors)

    def copy(self):
        new = Chromosome(copy.deepcopy(self.waypoints),
                         copy.deepcopy(self.speeds),
                         copy.deepcopy(self.sensors))
        new.fitness = self.fitness
        new.metrics = copy.deepcopy(self.metrics)
        return new

    # ---------- mission geometry ----------

    def full_path(self, drone):
        """Complete route of one drone: base -> waypoints -> base."""
        base = Environment.BASES[drone]
        return [base] + list(self.waypoints[drone]) + [base]

    def path_length(self, drone):
        path = self.full_path(drone)
        return sum(Environment.distance(path[i], path[i + 1])
                   for i in range(len(path) - 1))

    def scanned_cells(self, env):
        """All grid cells scanned by active sensors (set of (x, y))."""
        cells = set()
        r = Environment.SENSOR_RADIUS
        for d in range(self.NUM_DRONES):
            for i, (wx, wy) in enumerate(self.waypoints[d]):
                if self.sensors[d][i] == 1:
                    cx, cy = int(round(wx)), int(round(wy))
                    for x in range(cx - r, cx + r + 1):
                        for y in range(cy - r, cy + r + 1):
                            if (0 <= x < Environment.GRID_SIZE
                                    and 0 <= y < Environment.GRID_SIZE
                                    and Environment.distance((x, y), (cx, cy)) <= r):
                                cells.add((x, y))
        return cells
