import sys,unittest,math
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'recovery'))
from core import Recovery,Limits
from stationary import displacement

def quality(epoch=0,seq=1,stamp=1.,**kw):
    return dict(epoch=epoch,sequence=seq,stamp=stamp,seed_stamp=stamp,converged=True,source_points=2000,
                inliers=1800,error_per_inlier=.01,hessian_ratio=.001,**kw)
def running():
    r=Recovery()
    for i in range(4):r.registration(quality(seq=i+1,stamp=1+i*.5),i*.5,1+i*.5)
    r.tick(1.5,healthy=True,stationary=True,external=True,pose=True)
    assert r.permit
    return r
class Contract(unittest.TestCase):
    def test_no_startup_permission(self):self.assertFalse(Recovery().permit)
    def test_requires_real_registration(self):
        r=Recovery();r.tick(2,healthy=True,stationary=True,external=True,pose=True);self.assertFalse(r.permit)
    def test_bootstrap_complete(self):self.assertTrue(running().permit)
    def test_failure_inhibits_immediately(self):
        r=running();r.tick(2,healthy=False);self.assertFalse(r.permit);self.assertEqual(r.state,'TERMINAL')
    def test_no_reset_before_terminal(self):
        r=running();r.trigger_fault('gap',2);self.assertIsNone(r.tick(3,stationary=True,external=True));self.assertEqual(r.attempts,0)
    def test_terminal_timeout_stays_failed(self):
        r=running();r.trigger_fault('gap',2);r.tick(7.01);self.assertEqual(r.fault,'old_goal_terminal_unconfirmed')
        r.tick(8,terminal=True,stationary=True,external=True);self.assertEqual(r.attempts,0)
    def test_moving_prevents_restart(self):
        r=running();r.trigger_fault('gap',2);r.tick(3,terminal=True)
        r.tick(4,terminal=True,external=True,stationary=False);self.assertEqual(r.state,'STATIC')
    def test_external_emergency_prevents_restart(self):
        r=running();r.trigger_fault('gap',2);r.tick(3,terminal=True)
        r.tick(4,terminal=True,stationary=True,external=False);self.assertEqual(r.state,'STATIC')
    def test_static_timeout(self):
        r=running();r.trigger_fault('gap',2);r.tick(3,terminal=True);r.tick(33.1)
        self.assertEqual(r.state,'FAILED')
    def test_restarts_increment_epoch_once(self):
        r=running();r.trigger_fault('gap',2);r.tick(3,terminal=True)
        self.assertEqual(r.tick(5,terminal=True,stationary=True,external=True),'restart')
        self.assertEqual((r.epoch,r.attempts),(1,1));self.assertFalse(r.permit)
        self.assertIsNone(r.tick(6,terminal=True,stationary=True,external=True))
    def test_restart_failure(self):
        r=Recovery();r.state='RESTARTING';r.restarted(2,2,False);self.assertEqual(r.state,'FAILED')
    def test_restart_timeout(self):
        r=Recovery();r.state='RESTARTING';r.tick(61);self.assertEqual(r.fault,'restart_timeout')
    def test_verify_timeout(self):
        r=Recovery();r.tick(31);self.assertEqual(r.fault,'new_epoch_localization_unconfirmed')
    def test_budget_exhaustion(self):
        r=Recovery();r.state='STATIC';r.attempts=2;r.tick(3,terminal=True,stationary=True,external=True)
        self.assertEqual(r.fault,'recovery_budget_exhausted')
    def test_old_epoch_rejected(self):
        r=Recovery();r.epoch=1;self.assertFalse(r.registration(quality(),1,1));self.assertFalse(r.permit)
    def test_seed_ack_alone_is_not_quality(self):
        r=Recovery();r.epoch=1;r.restart_stamp=2
        self.assertFalse(r.registration(dict(kind='seed_ack',epoch=1,stamp=3),3,3))
    def test_pre_restart_seed_rejected(self):
        r=Recovery();r.epoch=1;r.restart_stamp=2;q=quality(epoch=1,stamp=3);q['seed_stamp']=1
        self.assertFalse(r.registration(q,3,3))
    def test_pre_restart_stamp_rejected(self):
        r=Recovery();r.restart_stamp=2;self.assertFalse(r.registration(quality(stamp=1),1,1))
    def test_duplicate_or_rollback_rejected(self):
        r=Recovery();self.assertTrue(r.registration(quality(),1,1))
        self.assertFalse(r.registration(quality(),1.1,1.1));self.assertEqual(len(r.quality),0)
    def test_stale_future_nan_rejected(self):
        for stamp,now in [(1,3),(3,1),(math.nan,1),(math.inf,1)]:
            with self.subTest(stamp=stamp):self.assertFalse(Recovery().registration(quality(stamp=stamp),1,now))
    def test_bad_alignment_rejected(self):
        for key,value in [('converged',False),('inliers',20),('error_per_inlier',1.),('error_per_inlier',math.nan),('hessian_ratio',0.),('source_points',0)]:
            q=quality();q[key]=value
            with self.subTest(key=key,value=value):self.assertFalse(Recovery().registration(q,1,1))
    def test_expired_quality_cannot_release(self):
        r=running();r.state='VERIFY';r.tick(5,healthy=True,stationary=True,external=True,pose=True);self.assertFalse(r.permit)
    def test_pose_stability_still_required(self):
        r=running();r.state='VERIFY';r.tick(1.6,healthy=True,stationary=True,external=True,pose=False);self.assertFalse(r.permit)
    def test_recovery_needs_all_checks_again(self):
        r=running();r.trigger_fault('gap',2);r.tick(3,terminal=True);r.tick(5,terminal=True,stationary=True,external=True)
        r.restarted(6,6,True)
        for i in range(4):r.registration(quality(epoch=1,seq=i+1,stamp=6.1+i*.5),6.1+i*.5,6.1+i*.5)
        r.tick(7.6,healthy=True,stationary=True,external=True,pose=True);self.assertTrue(r.permit)

class Lidar(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rng=np.random.default_rng(64)
        xy=rng.uniform(-3,3,(2000,2))
        cls.corner=np.vstack([np.column_stack([xy,np.full(2000,-1.)]),
                              np.column_stack([np.full(2000,3.),xy]),
                              np.column_stack([xy[:,0],np.full(2000,3.),xy[:,1]])])
    def test_static_corner(self):self.assertTrue(displacement(self.corner,self.corner)['stationary'])
    def test_translation_detected_without_odometry(self):
        r=displacement(self.corner,self.corner+[.04,0,0]);self.assertFalse(r['stationary']);self.assertGreater(r['translation'],.03)
    def test_rotation_detected(self):
        angle=.04;c=math.cos(angle);s=math.sin(angle);rot=np.array([[c,-s,0],[s,c,0],[0,0,1]])
        self.assertFalse(displacement(self.corner,self.corner@rot.T)['stationary'])
    def test_flat_floor_degenerate(self):
        with self.assertRaises(ValueError):displacement(self.corner[:2000],self.corner[:2000])
    def test_sparse_invalid(self):
        for p in [np.zeros((50,3)),np.full((300,3),np.nan)]:
            with self.assertRaises(ValueError):displacement(p,p)

if __name__=='__main__':unittest.main(verbosity=2)
