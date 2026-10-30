"""Validate C071 geometry evidence. This tool never authorizes a ROS launch."""
import argparse
import json
import math
from pathlib import Path
import re


def finite_vector(value, size):
    return (
        isinstance(value, (list, tuple)) and len(value) == size
        and all(isinstance(x, (int, float)) and not isinstance(x, bool)
                and math.isfinite(x) for x in value)
    )


def valid_box(value):
    return finite_vector(value, 6) and all(value[i] < value[i + 3] for i in range(3))


def validate(value, runtime=False):
    errors = []
    if value.get('schema') != 1:
        errors.append('unsupported_geometry_schema')
    if value.get('geometry_unit') != 'mm':
        errors.append('geometry_unit_must_be_mm')
    if not re.fullmatch(r'[0-9a-f]{64}', str(value.get('source_sha256', ''))):
        errors.append('invalid_source_sha256')
    if not valid_box(value.get('original_cad_bbox_mm')):
        errors.append('invalid_assembly_bbox')
    radars = value.get('radars')
    if not isinstance(radars, list):
        radars = []
    if len(radars) != 2:
        errors.append('expected_two_radars')
    if any(not isinstance(x, dict) for x in radars):
        errors.append('invalid_radar_record')
        radars = [x for x in radars if isinstance(x, dict)]
    if {x.get('id') for x in radars} != {'primary', 'auxiliary'}:
        errors.append('distinct_primary_auxiliary_required')
    for radar in radars:
        prefix = str(radar.get('id', '?')) + ':'
        point = radar.get('geometric_cover_sphere_center_cad_mm')
        axis = radar.get('geometric_aperture_axis_cad')
        box = radar.get('combined_bbox_cad_mm')
        if not finite_vector(point, 3):
            errors.append(prefix + 'invalid_geometric_center')
        if not finite_vector(axis, 3) or abs(sum(x*x for x in axis) - 1) > 1e-6:
            errors.append(prefix + 'invalid_aperture_axis')
        if not valid_box(box):
            errors.append(prefix + 'invalid_radar_bbox')
        elif finite_vector(point, 3) and any(
                point[i] < box[i] - 1e-3 or point[i] > box[i+3] + 1e-3 for i in range(3)):
            errors.append(prefix + 'cover_center_outside_radar_bbox')
        reference = radar.get('reference_view_xyz_m')
        plane = value.get('wheel_reference_plane_cad_z_mm')
        if not finite_vector(reference, 3) or not finite_vector([plane], 1):
            errors.append(prefix + 'invalid_reference_view')
        elif finite_vector(point, 3):
            expected = [-point[0]*.001, point[1]*.001, (plane-point[2])*.001]
            if any(abs(a-b) > 1e-9 for a,b in zip(reference, expected)):
                errors.append(prefix + 'inconsistent_reference_view')
        if runtime:
            if radar.get('kinematic_relation') == 'unconfirmed_by_user' or not radar.get('parent_frame'):
                errors.append(prefix + 'mount_parent_unconfirmed')
            if not radar.get('runtime_pose'):
                errors.append(prefix + 'runtime_pose_not_verified')
            if radar.get('lidar_optical_origin') == 'not_calibrated_from_cover_sphere':
                errors.append(prefix + 'cover_center_is_not_verified_scan_origin')
    if runtime:
        # Deliberately unconditional: changing readiness flags cannot turn geometric
        # cover measurements into a calibrated, kinematically verified runtime profile.
        # C070 launchers do not consume this manifest.
        errors.append('geometry_manifest_cannot_start_navigation')
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path)
    parser.add_argument('--mode', choices=['geometry', 'runtime'], default='geometry')
    args = parser.parse_args()
    errors = validate(json.loads(args.manifest.read_text()), runtime=args.mode == 'runtime')
    print(json.dumps(dict(passed=not errors, mode=args.mode, errors=errors), ensure_ascii=False))
    return 2 if errors else 0


if __name__ == '__main__':
    raise SystemExit(main())
