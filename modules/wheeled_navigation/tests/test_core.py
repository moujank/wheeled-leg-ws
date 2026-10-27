"""Behavioral checks for invalid scans, unknown space and robot clearance."""
import math
import tempfile
import unittest
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scan_nav.core import GridSpec, OccupancyMapper, follow_path


class MappingPlanningTests(unittest.TestCase):
    def test_invalid_returns_do_not_clear_space(self):
        m = OccupancyMapper()
        m.integrate((0,0,0), np.array([0,1,2]), np.array([np.nan, -np.inf, 0.01]), 10)
        self.assertFalse(m.observed.any())

    def test_hit_and_no_return_have_different_endpoint_semantics(self):
        hit, empty = OccupancyMapper(), OccupancyMapper()
        hit.integrate((0,0,0), np.array([0.0]), np.array([2.0]), 5)
        empty.integrate((0,0,0), np.array([0.0]), np.array([np.inf]), 5)
        c,r = hit.cell(2,0)
        self.assertTrue(hit.occupied()[r,c])
        self.assertFalse(empty.occupied().any())
        self.assertTrue(empty.observed[r,c])

    def test_planner_cannot_cross_unknown(self):
        m = OccupancyMapper()
        with self.assertRaises(ValueError):
            m.plan((-3,0),(3,0))

    def test_narrow_passage_rejected_by_footprint(self):
        m = OccupancyMapper(GridSpec(0.1, 0, 0, 80, 60))
        m.observed[:] = True
        m.log_odds[:] = -3
        m.log_odds[:, 39:41] = 3
        m.log_odds[27:33,39:41] = -3  # 0.6 m opening, robot diameter 0.9 m
        with self.assertRaises(ValueError):
            m.plan((2,3),(6,3),radius=0.45)
        m.log_odds[15:45,39:41] = -3
        self.assertGreater(len(m.plan((2,3),(6,3),radius=0.45)), 2)

    def test_save_reload_and_follower(self):
        m = OccupancyMapper(GridSpec(0.1, -4, -4, 80, 80))
        m.observed[:] = True
        m.log_odds[:] = -3
        with tempfile.TemporaryDirectory() as d:
            m.save(d)
            loaded = OccupancyMapper.load(d)
            self.assertTrue(np.array_equal(m.free(), loaded.free()))
        path = m.plan((-2,-2),(2,2))
        pose, index = (-2,-2,math.pi), 0
        for _ in range(2000):
            pose,index,reached,_ = follow_path(pose,path,index)
            if reached:
                break
        self.assertTrue(reached)
        self.assertLess(math.dist(pose[:2],(2,2)),0.07)


if __name__ == "__main__":
    unittest.main()
