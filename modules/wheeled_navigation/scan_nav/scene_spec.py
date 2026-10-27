"""Scene construction and independent ground-truth evaluation only.

Never passed to the occupancy mapper or A* planner.
"""
import math

# name, centre xyz, size xyz
BOXES = [
    ("north", (0, 5, 1.0), (14.2, 0.2, 2.0)),
    ("south", (0, -5, 1.0), (14.2, 0.2, 2.0)),
    ("west", (-7, 0, 1.0), (0.2, 10, 2.0)),
    ("east", (7, 0, 1.0), (0.2, 10, 2.0)),
    ("partition_north", (0, 3.1, 1.0), (0.2, 3.8, 2.0)),
    ("partition_south", (0, -3.1, 1.0), (0.2, 3.8, 2.0)),
    ("crate_left", (-3.0, 1.7, 0.65), (1.5, 1.3, 1.3)),
    ("crate_right", (2.8, 1.9, 0.65), (1.6, 1.5, 1.3)),
    ("crate_lower", (3.7, -2.4, 0.55), (1.3, 1.3, 1.1)),
]

# Preset mapping tour, not autonomous exploration. Crosses the doorway twice.
TOUR = [(-5.5, -3.5), (-5.5, 3.5), (-1.5, 3.5), (-1.5, 0),
        (1.5, 0), (1.5, 3.5), (5.5, 3.5), (5.5, -3.5),
        (1.5, -3.5), (1.5, 0), (-1.5, 0), (-1.5, -3.5), (-5.5, -3.5)]


def tour_samples(step=0.3):
    for a, b in zip(TOUR, TOUR[1:]):
        n = max(1, math.ceil(math.dist(a, b) / step))
        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        for i in range(n):
            yield (a[0] + (b[0] - a[0]) * i / n,
                   a[1] + (b[1] - a[1]) * i / n, yaw)


def clearance(x, y):
    """Point-to-box distance, used solely by the validation oracle."""
    return min(math.hypot(max(abs(x-c[0])-s[0]/2, 0),
                          max(abs(y-c[1])-s[1]/2, 0)) for _, c, s in BOXES)


def path_clearance(path, step=0.025):
    best = float("inf")
    for a, b in zip(path, path[1:]):
        n = max(1, math.ceil(math.dist(a, b) / step))
        for i in range(n + 1):
            t = i / n
            best = min(best, clearance(a[0]*(1-t)+b[0]*t, a[1]*(1-t)+b[1]*t))
    return best
