import math
from pathlib import Path
import struct
import sys
from types import SimpleNamespace as NS
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from aux_mid_obstacle_cloud import filter_points, xyz_from_cloud, rotation_matrix
from dual_mid360_profile import navigation_params

class AuxiliaryTests(unittest.TestCase):
    def run_filter(self, points, **kw):
        return filter_points(points, np.eye(3), np.zeros(3), **kw)[0]
    def test_valid_xyz_retained(self):
        np.testing.assert_allclose(self.run_filter([[1., 0., .3]]), [[1., 0., .3]])
    def test_floor(self):
        self.assertEqual(len(self.run_filter([[1, 0, .059]])), 0)
    def test_height_edges(self):
        self.assertEqual(len(self.run_filter([[1, 0, .06], [0, 1, 2.]])), 2)
    def test_tall(self):
        self.assertEqual(len(self.run_filter([[1, 0, 2.001]])), 0)
    def test_finite(self):
        self.assertEqual(len(self.run_filter([[math.nan, 0, 1], [1, math.inf, 1]])), 0)
    def test_self_boundary(self):
        self.assertEqual(len(self.run_filter([[.29, 0, .2], [.1, .1, .2]])), 0)
    def test_range(self):
        self.assertEqual(len(self.run_filter([[5.01, 0, .2]])), 0)
    def test_nearest_bin(self):
        np.testing.assert_allclose(self.run_filter([[2, 0, .2], [1, 0, .2]]), [[1, 0, .2]])
    def test_empty(self):
        self.assertEqual(self.run_filter([]).shape, (0, 3))
    def test_rotated_filter_original_frame_output(self):
        r=np.array([[1,0,0],[0,0,-1],[0,1,0]])
        pts,_=filter_points([[1,.2,0]],r,np.array([0,0,.1]))
        np.testing.assert_allclose(pts,[[1,.2,0]])
    def test_translation_floor_filter(self):
        pts,_=filter_points([[1,0,-.05]],np.eye(3),np.array([0,0,.11]))
        self.assertEqual(len(pts),1)
    def test_invalid_bounds(self):
        with self.assertRaises(ValueError):self.run_filter([],min_range=6.)
    def test_invalid_polygon(self):
        with self.assertRaises(ValueError):self.run_filter([],footprint=[[0,0]])
    def test_invalid_transform(self):
        with self.assertRaises(ValueError):filter_points([],np.eye(3),[0,math.nan,0])
    def test_invalid_bins(self):
        with self.assertRaises(ValueError):self.run_filter([],bins=0)
    def test_bad_quaternion(self):
        with self.assertRaises(ValueError):rotation_matrix(NS(x=0,y=0,z=0,w=0))
    def test_quaternion(self):
        np.testing.assert_allclose(rotation_matrix(NS(x=0,y=0,z=0,w=1)),np.eye(3))
    def cloud(self,endian='<'):
        return NS(fields=[NS(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(['x','y','z','intensity'])],
            point_step=16,row_step=20,width=1,height=2,is_bigendian=endian=='>',
            data=b''.join(struct.pack(endian+'ffff',1,0,.2,0)+b'xxxx' for _ in range(2)))
    def test_padded_zero_reflectivity(self):
        np.testing.assert_allclose(xyz_from_cloud(self.cloud()),[[1,0,.2],[1,0,.2]],atol=1e-7)
    def test_big_endian(self):
        np.testing.assert_allclose(xyz_from_cloud(self.cloud('>')),[[1,0,.2],[1,0,.2]],atol=1e-7)
    def test_malformed_layout(self):
        c=self.cloud();c.data=c.data[:-1]
        with self.assertRaises(ValueError):xyz_from_cloud(c)
    def test_wrong_field(self):
        c=self.cloud();c.fields[0].datatype=2
        with self.assertRaises(ValueError):xyz_from_cloud(c)
    def test_profile_params(self):
        f=Path(__file__).resolve().parents[1]/'config/nav2_test.yaml'
        before=f.read_bytes();p=navigation_params(f)
        self.assertEqual(f.read_bytes(),before)
        self.assertEqual(p['controller_server']['ros__parameters']['FollowPath']['time_granularity'],.05)
        self.assertNotIn('rplidar_obstacle_cloud',p)
        for name in ('local_costmap','global_costmap'):
            v=p[name][name]['ros__parameters']
            self.assertNotIn('rplidar_obstacle_layer',v)
            c=v['aux_mid_obstacle_layer']['aux_mid_cloud']
            self.assertEqual(c['sensor_frame'],'aux_mid360')
            self.assertEqual(c['observation_persistence'],2.)
            self.assertFalse(c['clearing'])
            self.assertEqual(v['aux_mid_obstacle_layer']['min_obstacle_intensity'],.5)

if __name__ == '__main__':unittest.main(verbosity=2)
