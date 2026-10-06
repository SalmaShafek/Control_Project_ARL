"""
High-Level Lateral Steering Controller: Geometric Pure Pursuit.
Calculates steering curvature from lookahead arc geometry.
"""

import math  # noqa: F401
import numpy as np  # noqa: F401


class PurePursuitController:
    """Adaptive Pure Pursuit lateral controller."""

    def __init__(self, wheelbase=1.25, kv=0.25, l_min=0.8, l_max=2.5,
                 max_steer_rad=math.radians(35.0)):
        self.L = wheelbase
        self.kv = kv
        self.l_min = l_min
        self.l_max = l_max
        self.max_steer_rad = max_steer_rad

    def compute_lookahead(self, v):
        """Adaptive lookahead distance: Ld = clip(kv * v + l_min, l_min, l_max)."""
        # TODO: Milestone 5.3 Step 1 — Adaptive Lookahead Horizon
        # The car looks further ahead at higher speeds to plan smoother turns.
        # Implement the speed-scaled lookahead formula and clamp it to the allowed range.
        Ld = np.clip(self.kv*v + self.l_min,self.l_min,self.l_max)
        return Ld

    def find_target_waypoint(self, x, y, path_points, lookahead):
        """Searches along path for the target waypoint at lookahead distance."""
        # TODO: Milestone 5.3 Step 2 — Target Waypoint Selection
        # This selects the goal point the car will steer toward.
        # Find the nearest waypoint on the path, then walk forward until
        # you reach one that is at least 'lookahead' meters away.
    
        target_index =0
        nearest_index =0
        
        #finding nearest point
        nearest_point_distance = math.sqrt((path_points[0][0] - x)**2 + (path_points[0][1] -y)**2)
        min_distance =  math.sqrt((path_points[0][0] - x)**2 + (path_points[0][1] -y)**2)
        for i in range(0,len(path_points)):
            point = path_points[i]
            dx= point[0] - x
            dy= point[1] -y

            distance = math.sqrt((dx**2) + (dy**2))

            if distance < nearest_point_distance:
                nearest_index=i
                nearest_point_distance =distance

        nearest_point = path_points[nearest_index]

        for i in range(nearest_index, len(path_points)):
            point = path_points[i]
            dx= point[0] - x
            dy= point[1] -y
            
            distance = math.sqrt((dx**2) + (dy**2))
            if distance >= lookahead:
                target_index=i
                break

        return target_index,path_points[target_index]    
                             


        

    def compute_steering(self, x, y, yaw, target_pt, lookahead):
        """Computes steering angle in radians using Pure Pursuit geometry."""
        # TODO: Milestone 5.3 Steps 3 & 4 — Coordinate Transformation & Arc Law
        # This is the core of Pure Pursuit: transform the target into the vehicle's
        # local frame, then use the arc geometry formula to compute the steering angle.
        

        dx=target_pt[0]-x
        dy = target_pt[1]-y

        x_local = dx * math.cos(yaw) + dy * math.sin(yaw)
        y_local = -dx * math.sin(yaw) + dy * math.cos(yaw) ## same formula as in the debugging task of solomission only rotated by -yaw

        alpha = math.atan2(y_local,x_local)

        #pure pursuit rule: delta= tan^-1 (2*L*sin(alpha)) / Ld) (control session 2)
        term= (2 * self.L * math.sin(alpha)) / lookahead

        delta = math.atan(term)
        delta= np.clip(delta, -self.max_steer_rad,self.max_steer_rad)

        return delta

