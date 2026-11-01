import math
from pathlib import Path
import sys,tempfile,unittest
import xml.etree.ElementTree as ET
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from cad_scan_frames import native_scan_origin,add_scan_frames
class ScanFrameTests(unittest.TestCase):
    def model(self,scan='0 0 .03 0 0 0',imu='0 0 0 0 0 0'):
        return '<sdf><model><link><sensor type="gpu_lidar"><pose>'+scan+'</pose></sensor><sensor type="imu"><pose>'+imu+'</pose></sensor></link></model></sdf>'
    def read(self,text):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'model.xml';path.write_text(text)
            return native_scan_origin(path)
    def test_native_scan_origin_is_distinct_from_imu(self):
        self.assertEqual(self.read(self.model()),[0.,0.,.03])
    def test_nonzero_imu_origin_rejected(self):
        with self.assertRaises(ValueError):self.read(self.model(imu='0 0 .01 0 0 0'))
    def test_unhandled_scan_rotation_rejected(self):
        with self.assertRaises(ValueError):self.read(self.model(scan='0 0 .03 0 .1 0'))
    def test_nonfinite_scan_pose_rejected(self):
        with self.assertRaises(ValueError):self.read(self.model(scan='0 0 nan 0 0 0'))
    def test_new_frames_are_tf_only_fixed_children(self):
        original='<robot name="test"><link name="front"/><link name="aux"/></robot>'
        frames=[dict(mount=n,frame=n+'_scan',xyz=[0.,0.,.03]) for n in ['front','aux']]
        result=ET.fromstring(add_scan_frames(original,frames))
        self.assertEqual(len(result.findall('link')),4)
        self.assertEqual(len(result.findall('joint')),2)
        self.assertFalse(result.findall('.//inertial') or result.findall('.//collision') or result.findall('.//sensor'))
        for joint in result.findall('joint'):
            self.assertEqual(joint.get('type'),'fixed')
            self.assertEqual(joint.find('origin').get('xyz'),'0.0 0.0 0.03')
    def test_duplicate_or_missing_parent_rejected(self):
        for spec in [dict(mount='missing',frame='scan',xyz=[0,0,.03]),dict(mount='front',frame='front',xyz=[0,0,.03])]:
            with self.assertRaises(ValueError):add_scan_frames('<robot><link name="front"/></robot>',[spec])
    def test_invalid_tf_translation_rejected(self):
        with self.assertRaises(ValueError):add_scan_frames('<robot><link name="front"/></robot>',[dict(mount='front',frame='scan',xyz=[0,0,float('nan')])])
if __name__=='__main__':unittest.main(verbosity=2)
