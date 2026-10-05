"""
Low-Level Powertrain Cruise Controller (Longitudinal PID).
Regulates vehicle speed via normalized throttle/braking effort.
"""

import numpy as np  # noqa: F401


class PIDLongitudinalController:
    """Low-Level Powertrain Cruise Controller / Electronic Speed Control (ESC).

    Translates high-level velocity requests into normalized throttle/brake effort.
    Because physical vehicles experience friction and speed-squared aerodynamic drag,
    a closed-loop speed regulator is required to maintain target velocity.
    """

    def __init__(self, kp=1.0, ki=0.2, kd=0.05, dt=0.1,
                 max_throttle=1.0, max_brake=1.0, integral_limit=2.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.max_throttle = max_throttle
        self.max_brake = max_brake
        self.integral_limit = integral_limit

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_vel = None

    def compute(self, target_vel, current_vel):
        """Computes normalized throttle/braking effort in [-1.0, 1.0]."""

        error = target_vel - current_vel

        p = self.kp * error

        if self.prev_vel is None:
            d = 0.0
        else:
            d = -self.kd * (current_vel - self.prev_vel) / self.dt

        temp_integral = self.integral + error * self.dt
        temp_integral = np.clip(temp_integral,-self.integral_limit,self.integral_limit)

        temp_i = self.ki * temp_integral
        temp_output = p + temp_i + d

        if not (
            (temp_output > self.max_throttle and error > 0) or
            (temp_output < -self.max_brake and error < 0)
        ):
            self.integral = temp_integral

        i = self.ki * self.integral

        output = p + i + d

        output = float(np.clip(output,-self.max_brake,self.max_throttle))

        self.prev_error = error
        self.prev_vel = current_vel

        return output
    

    def reset(self):
        """Resets integrator and previous error state."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_vel= None
