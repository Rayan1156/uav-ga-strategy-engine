"""
visualize.py - Output visualizations for the report:
  1. Fitness convergence curve (best vs average per generation)
  2. Evolved flight-path map (paths + zones + targets)
  3. Coverage heatmap (how often each cell is scanned, sampled at
     checkpoints across generations)
"""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle, Circle

from environment import Environment


def plot_convergence(history_best, history_avg, filename,
                     random_best=None):
    plt.figure(figsize=(8, 5))
    gens = range(1, len(history_best) + 1)
    plt.plot(gens, history_best, label="GA best fitness")
    plt.plot(gens, history_avg, label="GA average fitness")
    if random_best is not None:
        gens_r = [100 * (i + 1) / 100 for i in range(len(random_best))]
        plt.plot(range(1, len(random_best) + 1), random_best,
                 label="Random search best (per 100 evals)")
    plt.xlabel("Generation")
    plt.ylabel("Fitness")
    plt.title("Fitness Convergence - Best vs Average")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(filename, dpi=150)
    plt.close()


def _draw_world(ax):
    env = Environment
    for (x1, y1, x2, y2, w) in env.PRIORITY_ZONES:
        ax.add_patch(Rectangle((x1, y1), x2 - x1, y2 - y1,
                               facecolor="gold", alpha=0.35,
                               edgecolor="darkgoldenrod",
                               label="Priority zone" if x1 == 30 else None))
    for (x1, y1, x2, y2) in env.NO_FLY_ZONES:
        ax.add_patch(Rectangle((x1, y1), x2 - x1, y2 - y1,
                               facecolor="red", alpha=0.35,
                               edgecolor="darkred", hatch="//"))
    for (cx, cy, r, s) in env.HAZARDS:
        ax.add_patch(Circle((cx, cy), r, facecolor="purple",
                            alpha=0.25, edgecolor="purple"))
    tx = [t[0] for t in env.TARGETS]
    ty = [t[1] for t in env.TARGETS]
    ax.scatter(tx, ty, marker="*", s=140, c="black", label="Targets")
    bx = [b[0] for b in env.BASES]
    by = [b[1] for b in env.BASES]
    ax.scatter(bx, by, marker="s", s=80, c="blue", label="Bases")
    ax.set_xlim(0, env.GRID_SIZE)
    ax.set_ylim(0, env.GRID_SIZE)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.2)


def plot_paths(chrom, filename, title="Evolved UAV Flight Paths"):
    fig, ax = plt.subplots(figsize=(7, 7))
    _draw_world(ax)
    colors = ["tab:blue", "tab:green", "tab:orange"]
    for d in range(chrom.NUM_DRONES):
        path = chrom.full_path(d)
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        ax.plot(xs, ys, color=colors[d], linewidth=1.6,
                label=f"Drone {d + 1}")
        for i, (wx, wy) in enumerate(chrom.waypoints[d]):
            marker = "o" if chrom.sensors[d][i] == 1 else "x"
            ax.scatter([wx], [wy], marker=marker, s=55, c=colors[d])
    ax.set_title(title + "\n(o = sensor ON, x = sensor OFF)")
    ax.legend(loc="upper left", fontsize=8)
    plt.tight_layout()
    plt.savefig(filename, dpi=150)
    plt.close()


def plot_heatmap(coverage_counts, filename,
                 title="Coverage Heatmap (scan frequency)"):
    grid = np.array(coverage_counts, dtype=float)
    fig, ax = plt.subplots(figsize=(7, 6))
    im = ax.imshow(grid.T, origin="lower", cmap="hot",
                   extent=[0, Environment.GRID_SIZE,
                           0, Environment.GRID_SIZE])
    for (x1, y1, x2, y2) in Environment.NO_FLY_ZONES:
        ax.add_patch(Rectangle((x1, y1), x2 - x1, y2 - y1, fill=False,
                               edgecolor="cyan", linewidth=1.5))
    fig.colorbar(im, ax=ax, label="Times scanned")
    ax.set_title(title)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    plt.tight_layout()
    plt.savefig(filename, dpi=150)
    plt.close()


def coverage_counts_over_run(chroms):
    """Accumulate scan counts from a list of chromosomes (checkpoints)."""
    size = Environment.GRID_SIZE
    env = Environment()
    counts = [[0] * size for _ in range(size)]
    for chrom in chroms:
        for (x, y) in chrom.scanned_cells(env):
            counts[x][y] += 1
    return counts
