"""No simulator dependencies: scan integration, map export, and footprint-aware A*."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import heapq
import json
import math
import numpy as np


@dataclass(frozen=True)
class GridSpec:
    resolution: float = 0.1
    xmin: float = -7.5
    ymin: float = -5.5
    width: int = 150
    height: int = 110


class OccupancyMapper:
    """Inverse range model. Unknown stays unknown until a ray traverses it."""

    def __init__(self, spec=GridSpec()):
        self.spec = spec
        self.log_odds = np.zeros((spec.height, spec.width), dtype=np.float32)
        self.observed = np.zeros_like(self.log_odds, dtype=bool)
        self.scan_count = 0

    def cell(self, x, y):
        s = self.spec
        return int(math.floor((x - s.xmin) / s.resolution)), int(math.floor((y - s.ymin) / s.resolution))

    def world(self, col, row):
        s = self.spec
        return s.xmin + (col + 0.5) * s.resolution, s.ymin + (row + 0.5) * s.resolution

    def inside(self, col, row):
        return 0 <= col < self.spec.width and 0 <= row < self.spec.height

    def integrate(self, pose, angles, ranges, max_range, min_range=0.15):
        """pose=(x,y,yaw) is the laser pose; inf means a valid no-return ray.

        NaN, -inf and below-minimum returns are invalid, not free space.
        The sensor's max-range sentinel is considered a no-return.
        """
        x, y, yaw = map(float, pose)
        angles, ranges = np.asarray(angles), np.asarray(ranges)
        if angles.shape != ranges.shape or angles.ndim != 1:
            raise ValueError("angles and ranges must be equally sized 1D arrays")
        free_chunks, hit_cells = [], []
        s = self.spec
        for angle, raw in zip(angles, ranges):
            if not np.isfinite(angle) or np.isnan(raw) or raw < min_range:
                continue
            distance = min(float(raw), max_range)
            hit = np.isfinite(raw) and raw < max_range - 0.01
            n = max(2, int(math.ceil(distance / (s.resolution * 0.45))) + 1)
            steps = np.linspace(0, distance, n)
            cols = np.floor((x + steps * math.cos(yaw + angle) - s.xmin) / s.resolution).astype(int)
            rows = np.floor((y + steps * math.sin(yaw + angle) - s.ymin) / s.resolution).astype(int)
            valid = (cols >= 0) & (cols < s.width) & (rows >= 0) & (rows < s.height)
            flat = np.unique(rows[valid] * s.width + cols[valid])
            if hit and valid[-1]:
                endpoint = rows[-1] * s.width + cols[-1]
                flat = flat[flat != endpoint]
                hit_cells.append(endpoint)
            free_chunks.append(flat)
        if not free_chunks:
            return
        free = np.unique(np.concatenate(free_chunks))
        hits = np.unique(hit_cells).astype(int)
        # Within one scan, a measured obstacle wins over an adjacent grazing ray.
        free = np.setdiff1d(free, hits)
        odds = self.log_odds.ravel()
        seen = self.observed.ravel()
        odds[free] -= 0.45
        odds[hits] += 1.4
        seen[free] = True
        seen[hits] = True
        np.clip(odds, -6, 6, out=odds)
        self.scan_count += 1

    def occupied(self):
        return self.observed & (self.log_odds > 0.65)

    def free(self):
        return self.observed & (self.log_odds < -0.3)

    def blocked(self, radius=0.45):
        """Inflate all non-free cells. Unknown cannot be used as a shortcut.

        The extra cell diagonal makes cell-centre paths conservative with
        respect to finite-sized occupied cells, not just their centres.
        """
        s = self.spec
        if not math.isfinite(radius) or radius < 0:
            raise ValueError("radius must be a finite non-negative number")
        result = ~self.free()
        source = result.copy()
        reach = radius + s.resolution * math.sqrt(2)
        n = math.ceil(reach / s.resolution)
        for dy in range(-n, n + 1):
            for dx in range(-n, n + 1):
                if math.hypot(dx, dy) * s.resolution > reach:
                    continue
                ys, ye = max(0, -dy), min(s.height, s.height - dy)
                xs, xe = max(0, -dx), min(s.width, s.width - dx)
                result[ys + dy:ye + dy, xs + dx:xe + dx] |= source[ys:ye, xs:xe]
        # Off-grid area is also unknown.
        result[:n, :] = result[-n:, :] = True
        result[:, :n] = result[:, -n:] = True
        return result

    def plan(self, start, goal, radius=0.45):
        blocked = self.blocked(radius)
        source, target = self.cell(*start), self.cell(*goal)
        for name, cell in [("start", source), ("goal", target)]:
            if not self.inside(*cell) or blocked[cell[1], cell[0]]:
                raise ValueError(f"{name} is outside known traversable space: {cell}")
        queue = [(math.dist(source, target), 0.0, source)]
        costs, parent = {source: 0.0}, {}
        moves = [(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy]
        while queue:
            _, cost, current = heapq.heappop(queue)
            if cost > costs[current]:
                continue
            if current == target:
                cells = [current]
                while current in parent:
                    current = parent[current]
                    cells.append(current)
                path = [self.world(*c) for c in reversed(cells)]
                # Include the exact requested endpoints, both inside free cells.
                return [tuple(start)] + path + [tuple(goal)]
            x, y = current
            for dx, dy in moves:
                nx, ny = x + dx, y + dy
                if not self.inside(nx, ny) or blocked[ny, nx]:
                    continue
                if dx and dy and (blocked[y, nx] or blocked[ny, x]):
                    continue  # no diagonal corner cutting
                nxt = (nx, ny)
                alt = cost + math.hypot(dx, dy)
                if alt < costs.get(nxt, float("inf")):
                    costs[nxt], parent[nxt] = alt, current
                    heapq.heappush(queue, (alt + math.dist(nxt, target), alt, nxt))
        raise ValueError("No path in known free space")

    def save(self, directory):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        pixels = np.full(self.log_odds.shape, 205, dtype=np.uint8)
        pixels[self.free()] = 254
        pixels[self.occupied()] = 0
        with (directory / "map.pgm").open("wb") as f:
            f.write(f"P5\n{self.spec.width} {self.spec.height}\n255\n".encode())
            f.write(np.flipud(pixels).tobytes())
        (directory / "map.yaml").write_text(
            f"image: map.pgm\nmode: trinary\nresolution: {self.spec.resolution}\n"
            f"origin: [{self.spec.xmin}, {self.spec.ymin}, 0.0]\nnegate: 0\n"
            "occupied_thresh: 0.65\nfree_thresh: 0.196\n", encoding="utf-8")
        np.savez_compressed(directory / "grid.npz", log_odds=self.log_odds,
                            observed=self.observed, spec=json.dumps(vars(self.spec)))

    @classmethod
    def load(cls, directory):
        with np.load(Path(directory) / "grid.npz", allow_pickle=False) as data:
            obj = cls(GridSpec(**json.loads(str(data["spec"]))))
            obj.log_odds[:] = data["log_odds"]
            obj.observed[:] = data["observed"]
        return obj


def path_length(path):
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


def follow_path(pose, path, index, dt=0.1, speed=0.6, yaw_rate=0.7):
    """Unicycle demo driver only; this never invokes or validates the RL policy."""
    x, y, yaw = pose
    while index < len(path) - 1 and math.dist((x, y), path[index]) < 0.12:
        index += 1
    goal = path[index]
    distance = math.dist((x, y), goal)
    if index == len(path) - 1 and distance < 0.06:
        return (x, y, yaw), index, True, (0.0, 0.0)
    error = math.atan2(math.sin(math.atan2(goal[1] - y, goal[0] - x) - yaw),
                       math.cos(math.atan2(goal[1] - y, goal[0] - x) - yaw))
    wz = float(np.clip(2.0 * error, -yaw_rate, yaw_rate))
    vx = min(speed, distance / dt) * max(0.0, math.cos(error)) if abs(error) < 0.5 else 0.0
    mid = yaw + 0.5 * wz * dt
    return (x + vx * math.cos(mid) * dt, y + vx * math.sin(mid) * dt,
            math.atan2(math.sin(yaw + wz * dt), math.cos(yaw + wz * dt))), index, False, (vx, wz)
