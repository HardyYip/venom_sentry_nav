"""Minimal source/receipt checks, shared through immutable snapshots."""
import math
import threading
from collections import deque

class ImuWindow:
    def __init__(self):
        self.lock=threading.Lock()
        self.samples=deque();self.invalid=deque(maxlen=16)
        self.at=-1e20;self.stamp=0.;self.since=None;self.report={}
    def observe(self,stamp,ros,steady,values):
        values=tuple(values)
        with self.lock:
            delta=stamp-self.stamp
            valid=all(math.isfinite(v) for v in values) and -.1<=ros-stamp<=.1
            valid=valid and (not self.stamp or 0<delta<=.1)
            self.report=dict(steady=steady,ros=ros,stamp=stamp,source_age_s=ros-stamp,
                             stamp_delta_s=delta,receipt_gap_s=steady-self.at,valid=valid)
            if not valid:
                self.invalid.append(self.report.copy());self.since=None;self.samples.clear()
            elif self.since is None:self.since=steady
            self.stamp=stamp;self.at=steady;self.samples.append((steady,values,valid))
            while self.samples and steady-self.samples[0][0]>2.5:self.samples.popleft()
    def snapshot(self):
        with self.lock:
            return dict(samples=tuple(self.samples),invalid=tuple(dict(x) for x in self.invalid),
                        at=self.at,stamp=self.stamp,since=self.since,report=dict(self.report))
