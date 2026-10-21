"""Deterministic software bus. Never imports/opens the physical transport."""
import time
import numpy as np
from centering.motors import RIGHT_GRIPPER_CLOSED


class SimulatedBus:
    def __init__(self,req,now=time.monotonic):
        self.req=req;self.now=now;self.q=np.array(req['start']);self.enabled=False;self.enable_attempted=False
        self.states={};self.targets=0;self.releases=0;self.fault='';self.contact=False;self.poll()
    def poll(self):
        if self.fault:raise RuntimeError('SIMULATED '+self.fault)
        for side,q in [('right',self.q),('left',self.req['left'])]:
            mode=2 if side=='right' and self.enabled else 0
            for i,value in enumerate(q,1):self.states[side,i]=(float(value),mode,self.now())
            opening=RIGHT_GRIPPER_CLOSED if side=='right' else self.req['left_opening_m']/ .044*1.0472
            self.states[side,8]=(opening,mode,self.now())
    def fresh(self):return len(self.states)==16 and not self.fault
    def begin_arm(self,start):
        self.enable_attempted=True;self.enabled=True;self.poll();yield
    def set_positions(self,q):
        if not self.enabled:raise RuntimeError('Simulated drives disabled')
        self.targets+=1
        if not self.contact:self.q=np.asarray(q).copy()
    def release_at_verified_rest(self,start):
        if np.max(abs(self.q-start))>np.radians(.2):raise RuntimeError('Mock release away from start')
        self.releases+=1;self.enabled=False;self.poll()
    def close(self):pass
