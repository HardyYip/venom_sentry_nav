"""Actual recovery policy with TF timing inputs; no simulator required."""
import sys,unittest,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"recovery"))
from timing import PoseTiming
from core import Recovery

class TimingContract(unittest.TestCase):
    def test_fresh_source_advances(self):
        g=PoseTiming()
        for i in range(30):self.assertEqual(g.observe(10+i*.05,10.05+i*.05,i*.05),"")
    def test_exact_source_age_limit_is_accepted(self):
        self.assertEqual(PoseTiming().observe(10,10.5,0),"")
    def test_source_age_exceeds_limit(self):
        self.assertEqual(PoseTiming().observe(10,10.500001,0),"tf_source_stale")
    def test_future_within_original_tolerance(self):
        self.assertEqual(PoseTiming().observe(10,9.91,0),"")
    def test_future_outside_original_tolerance(self):
        self.assertEqual(PoseTiming().observe(10,9.899,0),"tf_source_future")
    def test_constant_clock_does_not_extend_wall_stall(self):
        g=PoseTiming();g.observe(10,10.05,0)
        for i in range(1,6):self.assertEqual(g.observe(10,10.05,i*.1),"")
        self.assertEqual(g.observe(10,10.05,.500001),"tf_stamp_not_advancing")
    def test_repeated_tf_receipts_do_not_refresh(self):
        g=PoseTiming();g.observe(10,10.05,0)
        for i in range(1,10):g.observe(10,10.05,i*.05)
        self.assertEqual(g.observe(10,10.05,.51),"tf_stamp_not_advancing")
    def test_backward_tf_does_not_refresh(self):
        g=PoseTiming();g.observe(10.05,10.1,0);g.observe(10,10.1,.49)
        self.assertEqual(g.observe(10.05,10.1,.51),"tf_stamp_not_advancing")
    def test_invalid_future_does_not_refresh(self):
        g=PoseTiming();g.observe(10,10.05,0);g.observe(11,10.05,.49)
        self.assertEqual(g.observe(10,10.05,.51),"tf_stamp_not_advancing")
    def test_missing_does_not_refresh(self):
        g=PoseTiming();g.observe(10,10.05,0);self.assertEqual(g.missing(.49),"tf_missing")
        self.assertEqual(g.observe(10,10.05,.51),"tf_stamp_not_advancing")
    def test_fresh_progress_can_recover_health(self):
        g=PoseTiming();g.observe(10,10.05,0);g.observe(10,10.05,.51)
        self.assertEqual(g.observe(10.1,10.15,.52),"")
    def test_nonfinite_and_zero_rejected(self):
        for v in (math.nan,math.inf,-math.inf,0,-1):
            self.assertEqual(PoseTiming().observe(v,10,0),"tf_invalid_time")
    def test_invalid_ros_clock_rejected(self):
        for v in (math.nan,math.inf,0,-1):
            self.assertEqual(PoseTiming().observe(10,v,0),"tf_invalid_time")
    def test_monotonic_regression_rejected(self):
        g=PoseTiming();g.observe(10,10.05,2)
        self.assertEqual(g.observe(10.1,10.15,1),"steady_time_regressed")
    def test_invalid_limits_rejected(self):
        for a,b in ((0,.1),(-1,.1),(.5,-.1),(math.nan,.1),(.5,math.inf)):
            with self.assertRaises(ValueError):PoseTiming(a,b)
    def test_epoch_reset_is_explicit(self):
        g=PoseTiming();g.observe(10,10.05,0);g.reset()
        self.assertEqual(g.observe(9,9.05,1),"")
    def test_running_tf_fault_triggers_existing_terminal_protocol(self):
        r=Recovery();r.state="RUNNING"
        self.assertIsNone(r.tick(1,healthy=False,health_reason="tf_stamp_not_advancing"))
        self.assertEqual(r.state,"TERMINAL");self.assertFalse(r.permit)
        self.assertEqual(r.trigger,"tf_stamp_not_advancing");self.assertEqual(r.attempts,0)
    def test_tf_fault_cannot_skip_terminal_wait(self):
        r=Recovery();r.state="RUNNING";r.tick(1,healthy=False,health_reason="tf_source_stale")
        self.assertIsNone(r.tick(2,healthy=True,stationary=True,external=True))
        self.assertEqual(r.state,"TERMINAL");self.assertEqual(r.attempts,0)

class ExhaustedTimingRecovery(unittest.TestCase):
    def test_exhausted_budget_latches_after_terminal_without_static_inputs(self):
        r=Recovery();r.state="RUNNING";r.attempts=2;r.epoch=2
        r.tick(1,healthy=False,health_reason="tf_source_stale")
        r.tick(2,terminal=True)
        r.tick(3,terminal=True,healthy=False,stationary=False,external=False)
        self.assertEqual(r.state,"FAILED");self.assertEqual(r.fault,"recovery_budget_exhausted")
        self.assertEqual(r.epoch,2);self.assertFalse(r.permit)
    def test_exhausted_budget_does_not_skip_old_terminal(self):
        r=Recovery();r.state="RUNNING";r.attempts=2;r.epoch=2
        r.tick(1,healthy=False,health_reason="tf_source_stale")
        r.tick(2,terminal=False,stationary=True,external=True)
        self.assertEqual(r.state,"TERMINAL");self.assertFalse(r.permit)
        r.tick(6.01,terminal=False)
        self.assertEqual(r.fault,"old_goal_terminal_unconfirmed")

if __name__=="__main__":unittest.main()
