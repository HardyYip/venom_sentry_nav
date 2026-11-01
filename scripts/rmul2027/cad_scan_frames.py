"""Known simulator scan-origin convention; never a physical MID calibration."""
import math
import xml.etree.ElementTree as ET
def native_scan_origin(path, require_imu=True):
    root=ET.parse(path).getroot()
    sensor=root.find(".//sensor[@type='gpu_lidar']")
    if sensor is None:raise ValueError('missing native GPU lidar')
    pose=sensor.find('pose')
    values=[float(x) for x in (pose.text if pose is not None else '0 0 0 0 0 0').split()]
    if len(values)!=6 or not all(math.isfinite(x) for x in values) or any(abs(x)>1e-12 for x in values[3:]) or (pose is not None and pose.get('relative_to')):
        raise ValueError('unsupported native scan pose')
    if require_imu:
        imu=root.find(".//sensor[@type='imu']")
        if imu is None:raise ValueError('missing native IMU')
        xyzrpy=[float(x) for x in imu.findtext('pose','0 0 0 0 0 0').split()]
        if len(xyzrpy)!=6 or any(not math.isfinite(x) or abs(x)>1e-12 for x in xyzrpy):
            raise ValueError('native IMU must remain at the mount origin')
    return values[:3]
def add_scan_frames(urdf, frames):
    root=ET.fromstring(urdf)
    if root.tag!='robot':raise ValueError('expected URDF robot')
    existing={link.get('name') for link in root.findall('link')}
    for spec in frames:
        parent,child,xyz=spec['mount'],spec['frame'],spec['xyz']
        if parent not in existing or child in existing or len(xyz)!=3 or not all(math.isfinite(x) for x in xyz):
            raise ValueError('invalid or duplicate TF-only scan frame')
        ET.SubElement(root,'link',name=child)
        joint=ET.SubElement(root,'joint',name=child+'_fixed_joint',type='fixed')
        ET.SubElement(joint,'parent',link=parent);ET.SubElement(joint,'child',link=child)
        ET.SubElement(joint,'origin',xyz=' '.join(str(x) for x in xyz),rpy='0 0 0')
        existing.add(child)
    return ET.tostring(root,encoding='unicode')
