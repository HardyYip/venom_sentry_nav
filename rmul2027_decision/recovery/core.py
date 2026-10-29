"""Fail-closed IMU recovery policy. All durations use monotonic receipt time.
Source stamps are independently checked; epochs cannot be inferred from fresh TF.
"""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class Limits:
    imu_gap: float=.1
    static_window: float=2.
    terminal_timeout: float=5.
    static_timeout: float=30.
    restart_timeout: float=60.
    verify_timeout: float=30.
    max_recoveries: int=2
    quality_samples: int=4
    quality_error: float=.5
    hessian_ratio: float=1e-6

class Recovery:
    def __init__(self,t=0.,limits=Limits()):
        self.limits=limits;self.state='VERIFY';self.since=t;self.epoch=0
        self.attempts=0;self.fault='';self.trigger='startup';self.restart_stamp=0.
        self.quality=[];self.last_sequence=0;self.last_source=-math.inf
    @property
    def permit(self):return self.state=='RUNNING' and not self.fault
    def fail(self,why,t):
        self.fault=why;self.state='FAILED';self.since=t
    def trigger_fault(self,why,t):
        if self.state!='RUNNING':return
        self.trigger=why;self.state='TERMINAL';self.since=t;self.quality=[]
    def tick(self,t,*,healthy=False,terminal=False,stationary=False,external=False,pose=False):
        self.quality=[v for v in self.quality if 0<=t-v<2.]
        if self.state=='FAILED':return None
        if self.state=='RUNNING':
            if not healthy:self.trigger_fault('imu_or_odometry_unhealthy',t)
        elif self.state=='TERMINAL':
            if terminal:
                self.state='STATIC';self.since=t
            elif t-self.since>self.limits.terminal_timeout:self.fail('old_goal_terminal_unconfirmed',t)
        elif self.state=='STATIC':
            if terminal and stationary and external:
                if self.attempts>=self.limits.max_recoveries:self.fail('recovery_budget_exhausted',t)
                else:
                    self.attempts+=1;self.epoch+=1;self.state='RESTARTING';self.since=t
                    self.quality=[];self.last_sequence=0;self.last_source=-math.inf
                    return 'restart'
            elif t-self.since>self.limits.static_timeout:self.fail('stationary_or_external_not_confirmed',t)
        elif self.state=='RESTARTING':
            if t-self.since>self.limits.restart_timeout:self.fail('restart_timeout',t)
        elif self.state=='VERIFY':
            if healthy and stationary and external and pose and len(self.quality)>=self.limits.quality_samples:
                self.state='RUNNING';self.since=t
            elif t-self.since>self.limits.verify_timeout:self.fail('new_epoch_localization_unconfirmed',t)
        return None
    def restarted(self,t,stamp,success):
        if self.state!='RESTARTING':return
        if not success:self.fail('restart_failed_or_old_process_remaining',t)
        else:self.state='VERIFY';self.since=t;self.restart_stamp=stamp
    def registration(self,q,t,ros):
        try:
            valid=(q['epoch']==self.epoch and q['sequence']>self.last_sequence
                and q['stamp']>max(self.restart_stamp,self.last_source)
                and (self.epoch==0 or self.restart_stamp<=q.get('seed_stamp',-1)<=q['stamp'])
                and -.1<=ros-q['stamp']<=.8 and q['converged'] is True
                and q['source_points']>=200 and q['inliers']>=200
                and q['inliers']/q['source_points']>=.5
                and 0<=q['error_per_inlier']<=self.limits.quality_error
                and q['hessian_ratio']>=self.limits.hessian_ratio
                and all(math.isfinite(q[k]) for k in ('stamp','error_per_inlier','hessian_ratio')))
        except (KeyError,TypeError,ValueError,ZeroDivisionError):valid=False
        if self.state!='VERIFY':return False
        if not valid:self.quality=[];return False
        self.last_sequence=q['sequence'];self.last_source=q['stamp']
        self.quality=[v for v in self.quality if t-v<3.]+[t]
        return True
