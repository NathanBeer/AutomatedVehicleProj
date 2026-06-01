from __future__ import annotations
import math
from collections import deque
from typing import Deque, List, Optional
import carla

def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))

class SpeedPID:
    def __init__(self, Kp=1.0, Ki=0.05, Kd=0.1, dt=0.05):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self._integral = 0.0
        self._prev_error = None

    def step(self, target, current):
        err = target - current
        self._integral = _clamp(self._integral + err * self.dt, -10.0, 10.0)
        deriv = ((err - self._prev_error) / self.dt) if self._prev_error is not None else 0.0
        self._prev_error = err
        return _clamp(self.Kp * err + self.Ki * self._integral + self.Kd * deriv, -1.0, 1.0)

class PurePursuitLateral:
    _WHEELBASE_M = 2.850
    _MAX_STEER_RAD = math.radians(70.0)

    def __init__(self, waypoints, lookahead=6.0):
        self._wps = deque(waypoints)
        self.lookahead = lookahead

    def step(self, transform):
        while len(self._wps) > 1 and math.dist((transform.location.x, transform.location.y), (self._wps[0].x, self._wps[0].y)) < self.lookahead * 0.4:
            self._wps.popleft()
        
        target = self._wps[0]
        for wp in self._wps:
            if math.dist((transform.location.x, transform.location.y), (wp.x, wp.y)) >= self.lookahead:
                target = wp
                break
        
        fwd = transform.get_forward_vector()
        ego_yaw = math.atan2(fwd.y, fwd.x)
        alpha = math.atan2(target.y - transform.location.y, target.x - transform.location.x) - ego_yaw
        delta = math.atan2(2.0 * self._WHEELBASE_M * math.sin(alpha), max(self.lookahead, 1e-6))
        return _clamp(delta / self._MAX_STEER_RAD, -1.0, 1.0)

    def is_done(self, loc):
        return math.dist((loc.x, loc.y), (self._wps[-1].x, self._wps[-1].y)) < 4.0

class WaypointFollower:
    def __init__(self, waypoints, target_speed_kmh=30.0, dt=0.05):
        self.target_ms = target_speed_kmh / 3.6
        self._lateral = PurePursuitLateral(waypoints, lookahead=max(5.0, target_speed_kmh / 8.0))
        self._longitudinal = SpeedPID(dt=dt)
        self.done = False

    def run_step(self, transform, current_speed_ms):
        if self._lateral.is_done(transform.location):
            self.done = True
            return carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0)
        
        steer = self._lateral.step(transform)
        pid = self._longitudinal.step(self.target_ms, current_speed_ms)
        
        return carla.VehicleControl(
            throttle=float(max(0.0, pid)), 
            steer=float(steer), 
            brake=float(max(0.0, -pid))
        )
