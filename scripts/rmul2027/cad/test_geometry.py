"""Portable negative tests; no ROS, OpenCascade, or raw STEP input required."""
import copy
import math
import unittest

from build_geometry_manifest import build
from layout_contract import validate


def assembly():
    rows = [dict(path=['root'], name='root', bbox_mm=[-265,-265,-510,265,265,114],
                 extent_mm=[530,530,624])]
    for side, y in enumerate([120,-120]):
        suffix = '' if side == 0 else '(镜像)'
        rows.append(dict(path=['root','housing'+suffix],
            name='MID-360_4_ASM'+suffix, instance='housing'+str(side),
            bbox_mm=[-35,y-35,-370,35,y+35,-300], bbox_center_mm=[0,y,-335]))
        rows.append(dict(path=['root','cover'+suffix],
            name='MID-360 v1'+suffix, instance='cover'+str(side),
            bbox_mm=[-22,y-22,-370,22,y+22,-325], bbox_center_mm=[0,y,-347.5],
            features_world=[
                dict(type='sphere',radius_mm=22,center_mm=[0,y,-348]),
                dict(type='cylinder',radius_mm=22.3,area_mm2=1000,
                     axis=[0,math.sqrt(.75),-.5])]))
    for i in range(4):
        rows.append(dict(path=['root','wheel'+str(i)],name='全向轮总装',
                         bbox_mm=[-50,-50,-70,50,50,82]))
    return dict(records=rows,source_sha256='c'*64,bytes=100,source='fixture.step')


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.value = build(assembly())

    def test_geometry_accepts_valid_evidence(self):
        self.assertEqual(validate(self.value), [])

    def test_reordered_assembly_preserves_pairing(self):
        source = assembly()
        source['records'].reverse()
        self.assertEqual(build(source), self.value)

    def test_two_heads_despite_four_named_parts(self):
        self.assertEqual(len(self.value['radars']), 2)
        self.assertEqual({r['id'] for r in self.value['radars']}, {'primary','auxiliary'})

    def test_missing_housing_rejected(self):
        source = assembly()
        source['records'] = [r for r in source['records'] if r.get('instance') != 'housing1']
        with self.assertRaises(ValueError):
            build(source)

    def test_missing_wheel_rejected(self):
        source = assembly()
        source['records'].pop()
        with self.assertRaises(ValueError):
            build(source)

    def test_wrong_cover_pair_rejected(self):
        source = assembly()
        source['records'][4]['name'] = 'MID-360 v1 nonmirror'
        with self.assertRaises(ValueError):
            build(source)

    def test_wrong_unit_rejected(self):
        self.value['geometry_unit'] = 'm'
        self.assertIn('geometry_unit_must_be_mm', validate(self.value))

    def test_nonfinite_center_rejected(self):
        self.value['radars'][0]['geometric_cover_sphere_center_cad_mm'][0] = float('nan')
        self.assertIn('primary:invalid_geometric_center', validate(self.value))

    def test_zero_axis_rejected(self):
        self.value['radars'][0]['geometric_aperture_axis_cad'] = [0,0,0]
        self.assertIn('primary:invalid_aperture_axis', validate(self.value))

    def test_reversed_box_rejected(self):
        self.value['original_cad_bbox_mm'][0] = 999
        self.assertIn('invalid_assembly_bbox', validate(self.value))

    def test_missing_fingerprint_rejected(self):
        self.value.pop('source_sha256')
        self.assertIn('invalid_source_sha256', validate(self.value))

    def test_cover_outside_housing_envelope_rejected(self):
        self.value['radars'][0]['geometric_cover_sphere_center_cad_mm'][0] = 1000
        self.assertIn('primary:cover_center_outside_radar_bbox', validate(self.value))

    def test_reference_scale_mismatch_rejected(self):
        self.value['radars'][0]['reference_view_xyz_m'][2] *= 1000
        self.assertIn('primary:inconsistent_reference_view', validate(self.value))

    def test_invalid_record_rejected(self):
        self.value['radars'][0] = None
        self.assertIn('invalid_radar_record', validate(self.value))

    def test_runtime_always_refuses_geometry_product(self):
        self.assertIn('geometry_manifest_cannot_start_navigation', validate(self.value, runtime=True))

    def test_readiness_flags_cannot_unlock_runtime(self):
        self.value.update(runtime_ready=True, status='SIMULATION_RUNTIME_VERIFIED')
        self.assertIn('geometry_manifest_cannot_start_navigation', validate(self.value, runtime=True))

    def test_guessed_pose_cannot_unlock_geometry_product(self):
        self.value.update(runtime_ready=True, status='SIMULATION_RUNTIME_VERIFIED')
        for radar in self.value['radars']:
            radar.update(parent_frame='chassis', kinematic_relation='guessed_fixed',
                         runtime_pose=[0,0,0,0,0,0], lidar_optical_origin='guessed')
        self.assertIn('geometry_manifest_cannot_start_navigation', validate(self.value, runtime=True))

    def test_unknown_parent_is_explicitly_reported(self):
        errors = validate(self.value, runtime=True)
        self.assertIn('primary:mount_parent_unconfirmed', errors)
        self.assertIn('auxiliary:mount_parent_unconfirmed', errors)

    def test_builder_does_not_mutate_source(self):
        source = assembly()
        before = copy.deepcopy(source)
        build(source)
        self.assertEqual(source, before)


if __name__ == '__main__':
    unittest.main(verbosity=2)
