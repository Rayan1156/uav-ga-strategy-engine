"""
fitness.py - Multi-objective fitness evaluation.

Fitness = weighted sum of rewards minus costs minus constraint penalties:

  + W_COVERAGE   * priority-zone coverage   (maximize)
  + W_DETECTION  * target detection rate    (maximize)
  - W_ENERGY     * energy expenditure       (minimize, normalized)
  - W_TIME       * mission makespan         (minimize, normalized)
  - W_HAZARD     * hazard exposure          (minimize, normalized)
  - PENALTY      * no-fly violations + range overflow (hard constraints)
"""

from environment import Environment


class FitnessEvaluator:
    W_COVERAGE = 0.35
    W_DETECTION = 0.25
    W_ENERGY = 0.15
    W_TIME = 0.10
    W_HAZARD = 0.15

    NO_FLY_PENALTY = 0.05     # per sampled path point inside a no-fly zone
    RANGE_PENALTY = 0.01      # per unit of range overflow

    # normalization references (rough maxima for this environment)
    ENERGY_REF = 1500.0
    TIME_REF = 300.0
    HAZARD_REF = 60.0

    def __init__(self, env):
        self.env = env
        self.priority = env.priority_cells()
        self.total_priority_weight = sum(self.priority.values())

    def evaluate(self, chrom):
        env = self.env
        scanned = chrom.scanned_cells(env)

        # --- rewards ---
        covered_weight = sum(w for cell, w in self.priority.items()
                             if cell in scanned)
        coverage = covered_weight / self.total_priority_weight

        detected = 0
        for (tx, ty) in env.TARGETS:
            for d in range(chrom.NUM_DRONES):
                hit = False
                for i, (wx, wy) in enumerate(chrom.waypoints[d]):
                    if (chrom.sensors[d][i] == 1 and
                            Environment.distance((tx, ty), (wx, wy))
                            <= Environment.SENSOR_RADIUS):
                        hit = True
                        break
                if hit:
                    detected += 1
                    break
        detection = detected / len(env.TARGETS)

        # --- costs ---
        energy = 0.0
        makespan = 0.0
        hazard_exposure = 0.0
        no_fly_points = 0
        range_overflow = 0.0

        for d in range(chrom.NUM_DRONES):
            path = chrom.full_path(d)
            samples = env.sample_path(path)
            drone_time = 0.0
            for j in range(len(path) - 1):
                leg_len = Environment.distance(path[j], path[j + 1])
                speed = chrom.speeds[d][j]
                # energy grows with distance and superlinearly with speed
                energy += leg_len * (0.5 + 0.5 * speed ** 2)
                drone_time += leg_len / speed
            # sensor energy: each active scan costs a fixed amount
            energy += 15.0 * sum(chrom.sensors[d])
            makespan = max(makespan, drone_time)

            for (sx, sy) in samples:
                if env.in_no_fly(sx, sy):
                    no_fly_points += 1
                hazard_exposure += env.hazard_severity_at(sx, sy)

            length = chrom.path_length(d)
            if length > Environment.MAX_RANGE:
                range_overflow += length - Environment.MAX_RANGE

        penalty = (self.NO_FLY_PENALTY * no_fly_points
                   + self.RANGE_PENALTY * range_overflow)

        fitness = (self.W_COVERAGE * coverage
                   + self.W_DETECTION * detection
                   - self.W_ENERGY * min(energy / self.ENERGY_REF, 1.5)
                   - self.W_TIME * min(makespan / self.TIME_REF, 1.5)
                   - self.W_HAZARD * min(hazard_exposure / self.HAZARD_REF, 1.5)
                   - penalty)

        chrom.fitness = fitness
        chrom.metrics = {
            "coverage": coverage,
            "detection": detection,
            "energy": energy,
            "time": makespan,
            "hazard": hazard_exposure,
            "no_fly_points": no_fly_points,
            "range_overflow": range_overflow,
            "penalty": penalty,
        }
        return fitness
