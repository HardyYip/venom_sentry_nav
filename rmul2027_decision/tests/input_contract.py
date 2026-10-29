import math,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"recovery"))
from imu import ImuWindow
V=(0.,0.,0.,0.,0.,9.81)
class InputContract(unittest.TestCase):
    def sample(self,stamp=10.,ros=10.,steady=0.,values=V):
        w=ImuWindow();w.observe(stamp,ros,steady,values);return w
    def test_contiguous_inputs_keep_original_start(self):
        w=ImuWindow()
        for i in range(440):w.observe(10+i*.005,10+i*.005,i*.005,V)
        self.assertEqual(w.snapshot()['since'],0.);self.assertTrue(w.snapshot()['samples'][-1][2])
    def test_real_gap_resets_continuity(self):
        w=self.sample();w.observe(10.105,10.105,.105,V)
        self.assertIsNone(w.snapshot()['since']);self.assertFalse(w.snapshot()['samples'][-1][2])
    def test_restart_continuity_starts_after_gap(self):
        w=self.sample();w.observe(10.2,10.2,.2,V);w.observe(10.205,10.205,.205,V)
        self.assertEqual(w.snapshot()['since'],.205)
    def test_replayed_stamp_rejected(self):
        w=self.sample();w.observe(10.,10.01,.01,V);self.assertIsNone(w.snapshot()['since'])
    def test_backward_stamp_rejected(self):
        w=self.sample();w.observe(9.99,10.,.01,V);self.assertIsNone(w.snapshot()['since'])
    def test_stale_queued_data_is_not_made_fresh(self):
        w=self.sample();w.observe(10.005,10.206,.201,V);self.assertIsNone(w.snapshot()['since'])
    def test_future_data_rejected(self):
        self.assertIsNone(self.sample(10.2,10.).snapshot()['since'])
    def test_nonfinite_measurement_rejected(self):
        self.assertIsNone(self.sample(values=(math.nan,)+V[1:]).snapshot()['since'])
    def test_invalid_history_is_bounded(self):
        w=self.sample()
        for i in range(30):w.observe(10.,10.,i*.01,V)
        self.assertEqual(len(w.snapshot()['invalid']),16)
    def test_immutable_reader_snapshot(self):
        w=self.sample();before=w.snapshot();w.observe(10.005,10.005,.005,V)
        self.assertEqual(len(before['samples']),1);self.assertEqual(before['stamp'],10.)
    def test_snapshot_mutation_cannot_change_reader(self):
        w=self.sample();w.observe(10.,10.,.01,V);s=w.snapshot()
        s['report']['valid']=True;s['invalid'][0]['valid']=True
        self.assertFalse(w.snapshot()['report']['valid']);self.assertFalse(w.snapshot()['invalid'][0]['valid'])
    def test_sample_window_keeps_original_duration(self):
        w=ImuWindow()
        for i in range(800):w.observe(10+i*.005,10+i*.005,i*.005,V)
        s=w.snapshot();self.assertLessEqual(s['samples'][-1][0]-s['samples'][0][0],2.5)
        self.assertEqual(s['since'],0.)
if __name__=="__main__":unittest.main()
