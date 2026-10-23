"""
environment.py - The mission world for the UAV Strategy Engine.

A 50x50 discretized grid representing the operational area.
Contains: drone bases, high-priority coverage zones, detection targets,
no-fly zones (hard constraint) and hazard pockets (soft risk).
"""

import math


class Environment:
    GRID_SIZE = 50

    # Each drone launches from its own base and must return to it.
    BASES = [(5, 5), (5, 44), (44, 5)]

    # High-priority disaster zones: (x1, y1, x2, y2, weight)
    PRIORITY_ZONES = [
        (30, 30, 44, 44, 3),   # main disaster zone - highest weight
        (8, 28, 20, 40, 2),    # secondary zone
    ]

    # Fixed target points to detect: (x, y)
    TARGETS = [
        (35, 35), (40, 32), (33, 41), (38, 44), (31, 30),
        (12, 30), (16, 36), (10, 39), (18, 33),
        (25, 10), (45, 20), (20, 20),
    ]

    # No-fly zones (rectangles): (x1, y1, x2, y2) - entering = penalty
    NO_FLY_ZONES = [
        (12, 10, 20, 17),
        (28, 36, 36, 43),
        (36, 8, 44, 14),
    ]

    # Hazard pockets (circles): (cx, cy, radius, severity 0..1)
    HAZARDS = [
        (24, 24, 6, 0.8),   # high wind pocket
        (42, 28, 5, 0.6),   # radiation pocket
        (14, 44, 4, 0.5),   # turbulence
    ]

    SENSOR_RADIUS = 4        # cells scanned around an active waypoint
    MAX_RANGE = 420.0        # max total flight distance per drone

    # ---------- geometry helpers ----------

    @staticmethod
    def distance(p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    @staticmethod
    def in_rect(x, y, rect):
        x1, y1, x2, y2 = rect[:4]
        return x1 <= x <= x2 and y1 <= y <= y2

    def in_no_fly(self, x, y):
        return any(self.in_rect(x, y, z) for z in self.NO_FLY_ZONES)

    def hazard_severity_at(self, x, y):
        """Highest hazard severity at a point (0 if safe)."""
        sev = 0.0
        for cx, cy, r, s in self.HAZARDS:
            if self.distance((x, y), (cx, cy)) <= r:
                sev = max(sev, s)
        return sev

    def priority_weight_at(self, x, y):
        """Priority weight of a cell (0 if outside priority zones)."""
        w = 0
        for zone in self.PRIORITY_ZONES:
            if self.in_rect(x, y, zone):
                w = max(w, zone[4])
        return w

    def priority_cells(self):
        """All cells inside priority zones, as {(x, y): weight}."""
        cells = {}
        for x in range(self.GRID_SIZE):
            for y in range(self.GRID_SIZE):
                w = self.priority_weight_at(x, y)
                if w > 0:
                    cells[(x, y)] = w
        return cells

    @staticmethod
    def sample_path(points, step=1.0):
        """Sample points along a polyline path (list of (x, y))."""
        samples = []
        for i in range(len(points) - 1):
            p1, p2 = points[i], points[i + 1]
            d = Environment.distance(p1, p2)
            n = max(1, int(d / step))
            for k in range(n):
                t = k / n
                samples.append((p1[0] + t * (p2[0] - p1[0]),
                                p1[1] + t * (p2[1] - p1[1])))
        samples.append(points[-1])
        return samples
