"""Config geometry for the shared, conservative trajectory/goal-field radius."""
import copy
import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from cad_dual_mid360_profile import configure_planning_clearance

def params():
    return dict(local_costmap=dict(local_costmap=dict(ros__parameters=dict(resolution=.05))),global_costmap=dict(global_costmap=dict(ros__parameters=dict(
      footprint='[[0.434978334,0],[0,0.434978334],[-0.434978334,0],[0,-0.434978334]]',
      resolution=.05,footprint_padding=.02))),
      controller_server=dict(ros__parameters=dict(FollowPath={})))
class PlanningGeometry(unittest.TestCase):
    def test_global_polygon_and_grid_bound(self):
        p=params();value=configure_planning_clearance(p,.5)
        self.assertGreater(value['required_bound_m'],.48)
        self.assertLess(value['required_bound_m'],.5)
        self.assertTrue(value['shared_trajectory_and_goal_field'])
        self.assertEqual(p['controller_server']['ros__parameters']['FollowPath']['planning_clearance_radius'],.5)
    def test_cannot_shrink_to_existing_physical_floor(self):
        with self.assertRaises(ValueError):configure_planning_clearance(params(),.432883794)
    def test_nonfinite_radius_rejected(self):
        for radius in [math.nan,math.inf,-math.inf]:
            with self.subTest(radius=radius),self.assertRaises(ValueError):
                configure_planning_clearance(params(),radius)
    def test_invalid_geometry_fails_closed(self):
        for key,value in [('resolution',0),('footprint_padding',-1),('footprint','[[0,0]]')]:
            p=params();p['global_costmap']['global_costmap']['ros__parameters'][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):configure_planning_clearance(p,.5)
    def test_complete_local_cell_coverage_keeps_trajectory_radius(self):
        p=params();value=configure_planning_clearance(p,.5)
        self.assertTrue(p['controller_server']['ros__parameters']['FollowPath']['planning_cell_coverage'])
        self.assertAlmostEqual(value['goal_field_cell_cover_radius_m'],.5+.05*math.sqrt(2)/2)
        self.assertEqual(p['controller_server']['ros__parameters']['FollowPath']['planning_clearance_radius'],.5)
    def test_local_resolution_independent_and_invalid_values_rejected(self):
        p=params();p['local_costmap']['local_costmap']['ros__parameters']['resolution']=.1
        self.assertAlmostEqual(configure_planning_clearance(p,.5)['goal_field_cell_half_diagonal_m'],.1*math.sqrt(2)/2)
        for v in [0,-.05,math.nan,math.inf]:
            p=params();p['local_costmap']['local_costmap']['ros__parameters']['resolution']=v
            with self.subTest(v=v),self.assertRaises(ValueError):configure_planning_clearance(p,.5)
if __name__=='__main__':unittest.main()
