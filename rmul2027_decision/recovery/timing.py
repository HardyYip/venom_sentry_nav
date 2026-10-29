"""Observe map-to-base source timing; never infer freshness from receipt alone.
The limits match the existing Nav2 PoseFresh contract. This is a recovery
trigger, not a replacement for the independent motion stop gates.
"""
import math

class PoseTiming:
    def __init__(self, timeout=.5, future=.1):
        if not math.isfinite(timeout) or timeout <= 0 or not math.isfinite(future) or future < 0:
            raise ValueError("positive timeout and nonnegative future tolerance required")
        self.timeout=timeout;self.future=future;self.reset()
    def reset(self):
        self.highwater=None;self.progress=None;self.report={}
    def observe(self, stamp, ros, steady):
        self.report={k:(v if math.isfinite(v) else None) for k,v in (("stamp",stamp),("ros",ros),("steady",steady))}
        if not all(math.isfinite(v) for v in (stamp,ros,steady)) or stamp <= 0 or ros <= 0:
            return self._reason("tf_invalid_time")
        if self.progress is not None and steady < self.progress:
            return self._reason("steady_time_regressed")
        age=ros-stamp;self.report["source_age_s"]=age
        if age < -self.future:return self._reason("tf_source_future")
        if age > self.timeout:return self._reason("tf_source_stale")
        if self.highwater is None or stamp > self.highwater:
            self.highwater=stamp;self.progress=steady
        self.report.update(highwater=self.highwater,progress_wall_s=steady-self.progress)
        if steady-self.progress > self.timeout:return self._reason("tf_stamp_not_advancing")
        return self._reason("")
    def missing(self, steady):
        self.report={"steady":steady,"highwater":self.highwater}
        return self._reason("tf_missing")
    def _reason(self, reason):
        self.report["reason"]=reason
        return reason
