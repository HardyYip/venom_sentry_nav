import copy
import hashlib
import tempfile
import json
import math
from pathlib import Path
import sys
import unittest

import numpy as np
import yaml
from PIL import Image
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from cad_dual_mid360_profile import mount_poses,fit_native_cover,adapt_prior_pcd,guarded_occupancy,configure_projection,DEFAULT_CONFIG

class MountTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config=json.loads(DEFAULT_CONFIG.read_text())
        workspace=Path(__file__).resolve().parents[3].parent.parent
        cls.mesh=workspace/'src/pb2025_robot_description/resource/models/mid360/meshes/mid360.dae'
        cls.center,cls.fit=fit_native_cover(cls.mesh,cls.config)

    def setUp(self):
        self.value=copy.deepcopy(self.config)
        self.poses=mount_poses(self.value,self.center,.2136)

    def test_exactly_two_roles(self):
        self.assertEqual(set(self.poses),{'primary','auxiliary'})
    def test_both_big_yaw_parent(self):
        self.assertTrue(all(p['parent']=='gimbal_yaw' for p in self.poses.values()))
    def test_primary_cover_registered(self):
        np.testing.assert_allclose(self.poses['primary']['cover_center_base_m'],[-.027154784255,.130273033715,.430748180531],atol=1e-10)
    def test_auxiliary_cover_registered(self):
        np.testing.assert_allclose(self.poses['auxiliary']['cover_center_base_m'],[.024776089228,-.130746284014,.430748180531],atol=1e-10)
    def test_native_registration_residual(self):
        self.assertLess(self.fit['maximum_residual_mm'],.1)
        self.assertLess(abs(self.fit['radius_mm']-22),.2)
    def test_cover_landmark_not_scan_origin(self):
        for pose in self.poses.values():
            r=np.array(pose['rotation_base_sensor'])
            scan=np.array(pose['base_xyz_m'])+r@np.array([0,0,.03])
            self.assertGreater(np.linalg.norm(scan-pose['cover_center_base_m']),.01)
    def test_preserved_native_cover_offset(self):
        for pose in self.poses.values():
            actual=np.array(pose['base_xyz_m'])+np.array(pose['rotation_base_sensor'])@self.center
            np.testing.assert_allclose(actual,pose['cover_center_base_m'],atol=1e-12)
    def test_rpy_roundtrip(self):
        for pose in self.poses.values():
            np.testing.assert_allclose(Rotation.from_euler('xyz',pose['rpy_rad']).as_matrix(),pose['rotation_base_sensor'],atol=1e-12)
    def test_aperture_tilt(self):
        for pose in self.poses.values():
            self.assertAlmostEqual(math.degrees(math.acos(pose['rotation_base_sensor'][2][2])),60)
    def test_yaw_motion_rotates_both_centers(self):
        yaw=Rotation.from_euler('z',math.pi/2).as_matrix()
        for pose in self.poses.values():
            point=np.array(pose['cover_center_base_m'])
            np.testing.assert_allclose(yaw@point,[-point[1],point[0],point[2]],atol=1e-12)
    def test_relative_head_distance_invariant_under_yaw(self):
        delta=np.array(self.poses['primary']['base_xyz_m'])-self.poses['auxiliary']['base_xyz_m']
        for angle in [0,.5,math.pi,-math.pi/2]:
            self.assertAlmostEqual(np.linalg.norm(Rotation.from_euler('z',angle).apply(delta)),np.linalg.norm(delta))
    def test_wrong_parent_rejected(self):
        self.value['parent_frame']='chassis'
        with self.assertRaises(ValueError):mount_poses(self.value,self.center,.2136)
    def test_duplicate_role_rejected(self):
        self.value['radars'][1]['role']='primary'
        with self.assertRaises(ValueError):mount_poses(self.value,self.center,.2136)
    def test_missing_head_rejected(self):
        self.value['radars'].pop()
        with self.assertRaises(ValueError):mount_poses(self.value,self.center,.2136)
    def test_nonfinite_translation_rejected(self):
        self.value['radars'][0]['cad_cover_sphere_center_mm'][0]=float('nan')
        with self.assertRaises(ValueError):mount_poses(self.value,self.center,.2136)
    def test_mirrored_base_frame_rejected(self):
        self.value['cad_to_base_rotation'][0][0]=1
        with self.assertRaises(ValueError):mount_poses(self.value,self.center,.2136)
    def test_bad_housing_rotation_rejected(self):
        self.value['radars'][0]['cad_housing_rotation'][0][0]=9
        with self.assertRaises(ValueError):mount_poses(self.value,self.center,.2136)
    def test_inconsistent_aperture_rejected(self):
        self.value['radars'][0]['expected_aperture_axis_cad']=[0,0,1]
        with self.assertRaises(ValueError):mount_poses(self.value,self.center,.2136)
    def test_physical_calibration_not_claimed(self):
        self.value['simulation_only']=False
        with self.assertRaises(ValueError):mount_poses(self.value,self.center,.2136)
    def test_configuration_not_mutated(self):
        before=copy.deepcopy(self.value)
        mount_poses(self.value,self.center,.2136)
        self.assertEqual(self.value,before)

class PriorFrameTests(unittest.TestCase):
    def fixture(self,root):
        source=root/'raw.pcd';manifest=root/'manifest.json';output=root/'base.pcd'
        values=np.array([[0,0,0],[1,2,3],[-2,1,.5]],dtype='<f4')
        source.write_bytes(b'FIELDS x y z\nTYPE F F F\nDATA binary\n'+values.tobytes())
        data=dict(calibration=dict(translation=[1.,2.,3.],rotation_xyzw=Rotation.from_euler('z',math.pi/2).as_quat().tolist()),
                  file_sha256={source.name:hashlib.sha256(source.read_bytes()).hexdigest()})
        manifest.write_text(json.dumps(data))
        return source,manifest,output,values
    def test_private_prior_restores_base_coordinates_without_touching_original(self):
        with tempfile.TemporaryDirectory() as tmp:
            source,manifest,output,values=self.fixture(Path(tmp))
            original=source.read_bytes()
            result=adapt_prior_pcd(source,manifest,output)
            points=np.frombuffer(output.read_bytes().split(b'DATA binary\n')[1],dtype='<f4').reshape(-1,3)
            expected=np.column_stack([-values[:,1]+1,values[:,0]+2,values[:,2]+3])
            np.testing.assert_allclose(points,expected,atol=1e-6)
            self.assertEqual(source.read_bytes(),original)
            self.assertEqual(result['points'],3)
    def test_wrong_map_fingerprint_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            source,manifest,output,_=self.fixture(Path(tmp))
            data=json.loads(manifest.read_text());data['file_sha256'][source.name]='0'*64
            manifest.write_text(json.dumps(data))
            with self.assertRaises(ValueError):adapt_prior_pcd(source,manifest,output)
            self.assertFalse(output.exists())
    def test_nonbinary_map_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            source,manifest,output,_=self.fixture(Path(tmp))
            source.write_bytes(b'DATA ascii\n1 2 3')
            with self.assertRaises(ValueError):adapt_prior_pcd(source,manifest,output)


class OccupancyGuardTests(unittest.TestCase):
    def fixture(self,root):
        source=root/'original';source.mkdir()
        image=np.full((9,9),254,dtype=np.uint8)
        image[0,:]=image[-1,:]=image[:,0]=image[:,-1]=0;image[4,4]=0
        pgm=source/'base.pgm';Image.fromarray(image).save(pgm)
        config=dict(image=pgm.name,resolution=.05,origin=[-1.55,-7.35,0.],mode='trinary',negate=0,occupied_thresh=.65,free_thresh=.196)
        mapping=source/'base.yaml';mapping.write_text(yaml.safe_dump(config))
        manifest=source/'manifest.json'
        manifest.write_text(json.dumps(dict(file_sha256={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in (pgm,mapping)})))
        output=root/'output';output.mkdir()
        return mapping,manifest,output,image
    def test_boundary_only_adds_occupied_cells_and_preserves_original(self):
        with tempfile.TemporaryDirectory() as tmp:
            mapping,manifest,output,image=self.fixture(Path(tmp))
            original=(mapping.parent/'base.pgm').read_bytes()
            result=guarded_occupancy(mapping,manifest,output)
            actual=np.asarray(Image.open(result['image']))
            self.assertTrue(np.all(actual[image==0]==0))
            self.assertEqual(int(actual[3,4]),0)
            self.assertEqual(int(actual[2,2]),254)
            self.assertEqual((mapping.parent/'base.pgm').read_bytes(),original)
    def test_resolution_origin_and_thresholds_preserved(self):
        with tempfile.TemporaryDirectory() as tmp:
            mapping,manifest,output,_=self.fixture(Path(tmp))
            before=yaml.safe_load(mapping.read_text())
            result=guarded_occupancy(mapping,manifest,output)
            after=yaml.safe_load(Path(result['yaml']).read_text())
            for key in before:
                if key!='image':self.assertEqual(after[key],before[key])
            self.assertEqual(result['cells'],1)
    def test_changed_original_map_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            mapping,manifest,output,_=self.fixture(Path(tmp))
            mapping.write_text(mapping.read_text()+'# unexpected source modification\n')
            with self.assertRaises(ValueError):guarded_occupancy(mapping,manifest,output)
    def test_unknown_occupancy_not_silently_rewritten(self):
        with tempfile.TemporaryDirectory() as tmp:
            mapping,manifest,output,image=self.fixture(Path(tmp))
            image[2,2]=205;pgm=mapping.parent/'base.pgm';Image.fromarray(image).save(pgm)
            d=json.loads(manifest.read_text());d['file_sha256'][pgm.name]=hashlib.sha256(pgm.read_bytes()).hexdigest()
            manifest.write_text(json.dumps(d))
            with self.assertRaises(ValueError):guarded_occupancy(mapping,manifest,output)


class PlanningProjectionConfigurationTests(unittest.TestCase):
    def fixture(self):
        return {'controller_server':{'ros__parameters':{'FollowPath':{'BaseObstacle.scale':.4,'critics':['BaseObstacle','ObstacleFootprint','PathDist','GoalDist'],'max_speed_xy':.5}}},'untouched':{'local_footprint':'original'}}
    def test_only_internal_seed_parameter_added(self):
        import copy
        params=self.fixture();original=copy.deepcopy(params)
        policy=configure_projection(params,.1)
        self.assertTrue(policy['internal_seed_only'] and policy['source_stamps_unchanged'])
        del params['controller_server']['ros__parameters']['FollowPath']['pose_projection_horizon']
        self.assertEqual(params,original)
    def test_disabled_projection_preserves_zero_horizon(self):
        params=self.fixture()
        self.assertEqual(configure_projection(params,0.)['horizon_s'],0.)
    def test_invalid_or_unbounded_projection_rejected(self):
        for horizon in [float('nan'),float('inf'),-.1,.11]:
            with self.subTest(horizon=horizon):
                with self.assertRaises(ValueError):configure_projection(self.fixture(),horizon)

if __name__=='__main__':unittest.main(verbosity=2)
