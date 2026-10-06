"""
High-Level Lateral Steering Controller: Reactive Lateral PID.
Steers based on instantaneous Cross-Track Error (CTE) and Heading Error.
"""

import math
import numpy as np  # noqa: F401


class LateralPIDController:
    """Lateral PID steering controller based on Cross-Track Error (CTE) and Heading Error.

    Commands front wheel steering based on instantaneous lateral offset (cross-track error)
    and orientation error relative to the nearest path waypoint.
    """

    def __init__(self, kp=0.8, ki=0.02, kd=0.15, k_yaw=0.5, dt=0.1,
                 max_steer_rad=math.radians(35.0), integral_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.k_yaw = k_yaw
        self.dt = dt
        self.max_steer_rad = max_steer_rad
        self.integral_limit = integral_limit

        self.integral_cte = 0.0
        self.prev_cte = None

    def compute_steering(self, cte, heading_err):
        """Computes front wheel steering angle delta in radians.

        Args:
            cte: Signed cross-track error in meters (positive = vehicle is left of path).
            heading_err: Heading error in radians (psi_vehicle - psi_path).

        Returns:
            delta_rad: Commanded front steering angle in radians [-max_steer_rad, max_steer_rad].
        """
        # TODO: Milestone 5.2 — Reactive Lateral PID Controller
        # This is the lateral steering controller. It corrects for how far the car
        # is off the path (CTE) and how misaligned its heading is.
        # Implement PID on the CTE with anti-windup, add a heading correction term,
        # and clamp the output to the steering limits.

        ## car is left to the road (+ve error) -> needs to turn right (requires to be -ve steering) so it has to be reversed
        p= - self.kp * cte
        d = - (((cte-self.prev_cte) / self.dt) * self.kd) if self.prev_cte is not None else 0.0

        temp_integral = (self.integral_cte + cte * self.dt)
        temp_integral = np.clip(temp_integral, -self.integral_limit, self.integral_limit) 
        temp_i = temp_integral * self.ki
         

        temp_output = p - temp_i + d - heading_err* self.k_yaw

        saturated = (temp_output > self.max_steer_rad and cte < 0) or (temp_output < -self.max_steer_rad and cte > 0)

        if not saturated:
            self.integral_cte = temp_integral

        i= self.integral_cte * self.ki
        output = p - i +d - heading_err * self.k_yaw
        output= np.clip(output, -self.max_steer_rad,self.max_steer_rad)
        self.prev_cte = cte
        return float(output)
        




    def reset(self):
        """Resets integrator and previous error state."""
        self.integral_cte = 0.0
        self.prev_cte = None
