import copy,sys,unittest
from pathlib import Path
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from final_hardware_contract import DEFAULT_PROFILE,inventory_errors,deployment_gaps,interface_template,audit_legacy_model

class FinalHardwareContract(unittest.TestCase):
    def setUp(self):self.p=yaml.safe_load(DEFAULT_PROFILE.read_text())
    def test_report_inventory_complete(self):self.assertEqual(inventory_errors(self.p),[])
    def test_rplidar_cannot_stand_in_for_second_mid(self):
        self.p["sensors"]["lidars"][1]["model"]="RPLidar_A2"
        self.assertTrue(inventory_errors(self.p))
    def test_missing_second_device_rejected(self):
        self.p["sensors"]["lidars"].pop()
        self.assertTrue(inventory_errors(self.p))
    def test_duplicate_frames_rejected(self):
        self.p["sensors"]["lidars"][1]["frame"]=self.p["sensors"]["lidars"][0]["frame"]
        self.assertTrue(inventory_errors(self.p))
    def test_duplicate_topics_rejected(self):
        self.p["sensors"]["lidars"][1]["cloud_topic"]="livox/lidar"
        self.assertTrue(inventory_errors(self.p))
    def test_two_imu_streams_must_not_feed_old_lio(self):
        self.p["integration"]["simultaneous_imu_inputs_to_lio"]=2
        self.assertTrue(inventory_errors(self.p))
    def test_fast_not_implicitly_enabled(self):
        self.p["chassis"]["automatically_enable_fast"]=True
        self.assertTrue(inventory_errors(self.p))
    def test_infantry_pitch_not_accepted(self):
        self.p["gimbal"]["pitch_target_degrees"]=[-12,35]
        self.assertTrue(inventory_errors(self.p))
    def test_unmeasured_lidar_prevents_nav_deployment(self):
        self.assertTrue(any("calibration" in x for x in deployment_gaps(self.p)))
    def test_enable_flag_does_not_bypass_calibration(self):
        self.p["integration"]["hardware_deployment_enabled"]=True
        self.assertTrue(deployment_gaps(self.p))
        self.assertTrue(any("not implemented" in x for x in deployment_gaps(self.p)))
    def test_unknowns_are_not_replaced_by_zero(self):
        template=interface_template(self.p)
        self.assertIsNone(template["lidars"][1]["extrinsics"]["xyz_m"])
        self.assertIsNone(template["lidars"][1]["connection"]["ip_address"])
        self.assertFalse(template["hardware_ready"])
    def test_primary_input_stays_compatible(self):
        t=interface_template(self.p)
        self.assertEqual(t["lio"]["cloud_input"],"livox/lidar")
        self.assertEqual(t["lio"]["imu_input"],"livox/imu")
        self.assertTrue(t["lio"]["secondary_imu_not_fused"])
    def test_partial_navigation_does_not_claim_camera_or_firing_done(self):
        self.assertGreater(len(deployment_gaps(self.p,full=True)),len(deployment_gaps(self.p)))
    def test_three_five_minute_rounds_are_not_short_demo(self):
        self.p["acceptance"]["minutes_per_round"]=1
        self.assertTrue(inventory_errors(self.p))
    def test_generated_template_does_not_mutate_profile(self):
        before=copy.deepcopy(self.p);interface_template(self.p)
        self.assertEqual(before,self.p)
    def test_legacy_model_identifies_wrong_second_lidar(self):
        p=Path("/home/hardy/pb2025_nav_ws/src/pb2025_robot_description/resource/xmacro/simulation_robot.sdf.xmacro")
        a=audit_legacy_model(p)
        self.assertFalse(a["matches_final_lidar_inventory"])
        self.assertEqual(a["found"],{"MID-360":1,"RPLidar_A2":1})

if __name__=="__main__":unittest.main()
