"""Compare the explicit recovery builder to the protected original shell builder."""
import copy,json,os,subprocess,sys,tempfile,unittest
from pathlib import Path
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from cad_recovery_profile import build_parameters,checked_budget
P=Path(__file__).resolve().parents[3]
SOURCE=P/'scripts/rmul2027/config/nav2_test.yaml'
class RecoveryProfile(unittest.TestCase):
    def test_only_budget_and_target_sampling_differ_from_actual_legacy_builder_in_all_topic_modes(self):
        text=(P/'rmul2027_decision/tools/start_low_speed.sh').read_text()
        code=text.split("<<'PY'\n",1)[1].split("\nPY\n",1)[0]
        modes=[{},dict(RMUL_RECOVERY_GICP_TOPIC='recovery/test_registered_scan'),
            dict(RMUL_RECOVERY_GICP_TOPIC='recovery/test_registered_scan',RMUL_RECOVERY_IMU_TOPIC='recovery/test_imu')]
        for mode in modes:
            with self.subTest(mode=mode),tempfile.TemporaryDirectory() as directory:
                env=dict(os.environ);env.pop('RMUL_RECOVERY_GICP_TOPIC',None);env.pop('RMUL_RECOVERY_IMU_TOPIC',None)
                env.update(mode,RMUL_IMU_RECOVERY='1')
                target=Path(directory)/'legacy.yaml'
                subprocess.run([sys.executable,'-',str(SOURCE),str(target)],input=code,text=True,env=env,check=True)
                legacy=yaml.safe_load(target.read_text());new=build_parameters(yaml.safe_load(SOURCE.read_text()),mode,64)
                self.assertEqual(new['small_gicp_relocalization']['ros__parameters'].pop('registration_max_iterations'),64)
                self.assertIs(new['small_gicp_relocalization']['ros__parameters'].pop('deterministic_global_sampling'),True)
                self.assertIs(new['small_gicp_relocalization']['ros__parameters'].pop('metric_correspondences'),True)
                self.assertEqual(new['small_gicp_relocalization']['ros__parameters']['global_leaf_size'],.02)
                self.assertEqual(legacy['small_gicp_relocalization']['ros__parameters']['global_leaf_size'],.1)
                new['small_gicp_relocalization']['ros__parameters']['global_leaf_size']=.1
                self.assertEqual(new,legacy)
    def test_invalid_resource_budgets_fail_closed(self):
        for value in [0,-1,65,True,'32',1.5]:
            with self.subTest(value=value),self.assertRaises(ValueError):checked_budget(value)
    def test_source_is_not_mutated(self):
        source=yaml.safe_load(SOURCE.read_text());before=copy.deepcopy(source)
        build_parameters(source,{},32);self.assertEqual(source,before)
    def test_original_gap_heartbeat_and_step_thresholds_remain(self):
        result=build_parameters(yaml.safe_load(SOURCE.read_text()),{},32)
        self.assertEqual(result['point_lio']['ros__parameters']['max_imu_gap'],.1)
        self.assertEqual(result['fake_vel_transform']['ros__parameters']['recovery_stop_timeout'],.3)
        self.assertEqual(result['fake_vel_transform']['ros__parameters']['decision_stop_timeout'],.3)
        self.assertEqual(result['controller_server']['ros__parameters']['FollowPath']['time_granularity'],.05)
    def test_source_sampling_and_original_source_configuration_remain(self):
        source=yaml.safe_load(SOURCE.read_text());result=build_parameters(source,{},32)
        self.assertEqual(source['small_gicp_relocalization']['ros__parameters']['global_leaf_size'],.1)
        self.assertEqual(result['small_gicp_relocalization']['ros__parameters']['registered_leaf_size'],.05)
        self.assertEqual(result['small_gicp_relocalization']['ros__parameters']['max_dist_sq'],source['small_gicp_relocalization']['ros__parameters']['max_dist_sq'])
if __name__=='__main__':unittest.main()
