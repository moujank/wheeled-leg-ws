"""Behavioral checks: scan-driven decisions, footprint and actuator boundary."""
import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scan_nav.core import GridSpec,OccupancyMapper
from scan_nav.exploration import ExplorationMap,FrontierExplorer,velocity_to_path,integrate_velocity,measured_stop,map_stop
from scan_nav.footprint import pose_is_free,visible_unknown_gain,measured_polygon_stop


class OnlineExplorationTests(unittest.TestCase):
    def test_blank_map_cannot_produce_a_route(self):
        m=OccupancyMapper(GridSpec(.1,-5,-5,100,100))
        self.assertIsNone(FrontierExplorer().choose(m,(0,0,0)))

    def test_goal_depends_on_which_side_has_been_scanned(self):
        angles=np.linspace(-math.pi,math.pi,360,endpoint=False)
        goals=[]
        for side in (-1,1):
            m=OccupancyMapper(GridSpec(.1,-6,-6,120,120))
            # A short return in one direction, open scan in the other direction.
            ranges=np.where(side*np.cos(angles)>.3,3.,1.2)
            m.integrate((0,0,0),angles,ranges,3.)
            goal=FrontierExplorer(radius=.3).choose(m,(0,0,0))
            self.assertIsNotNone(goal)
            self.assertGreater(side*goal.position[0],.5)
            self.assertTrue(all(m.free()[m.cell(*p)[1],m.cell(*p)[0]] for p in goal.path))
            goals.append(goal.position)
        self.assertGreater(math.dist(*goals),1.)

    def test_new_obstacle_invalidates_old_route_and_forces_detour(self):
        m=OccupancyMapper(GridSpec(.1,-5,-5,100,100));m.observed[:]=True;m.log_odds[:]=-3
        before=m.plan((-2,0),(2,0),.38)
        self.assertLess(max(abs(y) for x,y in before),.2)
        # This update represents new occupied measurements, not a new itinerary.
        m.log_odds[40:60,48:52]=3
        after=m.plan((-2,0),(2,0),.38)
        self.assertGreater(max(abs(y) for x,y in after),1.3)

    def test_sensor_stop_and_velocity_actuator_do_not_teleport(self):
        cmd,stopped=measured_stop((.3,0),(0,0,0),(0,0,0),np.array([0.]),np.array([.5]))
        self.assertTrue(stopped);self.assertEqual(cmd[0],0)
        p=integrate_velocity((0,0,0),(.3,.5),.1)
        self.assertLessEqual(math.hypot(*p[:2]),.030001)
        self.assertAlmostEqual(p[2],.05)
        command,done=velocity_to_path((0,0,math.pi),[(0,0),(2,0)])
        self.assertEqual(command[0],0);self.assertFalse(done)

    def test_partially_observed_map_keeps_unknown_centres_blocked(self):
        m=ExplorationMap(GridSpec(.05,-2,-2,80,80))
        m.observed[10:70,10:70]=True;m.log_odds[m.observed]=-3
        # A newly detected occupied strip stops a moving full-size robot.
        m.log_odds[30:50,50:52]=3
        cmd,stopped=map_stop(m,(0,0,0),(.3,0))
        self.assertTrue(stopped);self.assertEqual(cmd[0],0.)
        with self.assertRaises(ValueError):m.plan((0,0),(-1.8,0),.38)
        with self.assertRaises(ValueError):m.plan((0,0),(.65,0),.38)

    def test_full_size_polygon_can_fit_gap_which_circumcircle_rejects(self):
        m=ExplorationMap(GridSpec(.01,-2,-2,400,400));m.observed[:]=True;m.log_odds[:]=-3
        m.log_odds[:169,:]=3;m.log_odds[231:,:]=3  # 0.62 m corridor
        self.assertTrue(pose_is_free(m,(0,0),0))
        self.assertFalse(pose_is_free(m,(0,0),math.pi/4))
        with self.assertRaises(ValueError):m.plan((-.8,0),(.8,0),.38)
        cmd,stop=measured_polygon_stop((.15,0),np.array([math.pi/2,-math.pi/2]),np.array([.31,.31]))
        self.assertFalse(stop);self.assertGreater(cmd[0],0.)

    def test_information_gain_stops_at_known_walls(self):
        m=ExplorationMap(GridSpec(.05,-3,-3,120,120))
        m.observed[40:80,40:80]=True;m.log_odds[40:80,40:80]=-3
        m.log_odds[40,40:80]=3;m.log_odds[79,40:80]=3
        m.log_odds[40:80,40]=3;m.log_odds[40:80,79]=3
        closed=visible_unknown_gain(m,(0,0))
        m.log_odds[50:70,79]=-3
        opened=visible_unknown_gain(m,(0,0))
        self.assertAlmostEqual(closed,0.)
        self.assertGreater(opened,.2)

if __name__=='__main__':unittest.main()
