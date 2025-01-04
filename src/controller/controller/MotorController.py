import time
import sys
import math
from geometry_msgs.msg import Twist

def bound(value, low, high):
    return max(low, min(high, value))

class MotorController:
    def __init__(self, cmd_vel_publisher):
        self.cmd_vel_publisher = cmd_vel_publisher
        

    def step(self, current_heading, goal_heading, speed, turn_strength: float = 1.0):

        rotation = self._goal_rotation(current_heading, goal_heading)
        #print(f"goal diff: {rotation}")
        
        rotation_max = (1 / 8) * (2 * math.pi)
        rotation_scaler = bound(rotation, -rotation_max, rotation_max) / rotation_max
        max_rotation_speed = 0.5 * math.pi * turn_strength
        min_rotation_speed = 0
        if abs(rotation_scaler) < 0.03:
            rotation_speed = 0
        else:
            sign = 1 if rotation_scaler >= 0 else -1
            val = abs(rotation_scaler)
            speed_range = max_rotation_speed - min_rotation_speed
            rotation_speed = sign * (speed_range * val + min_rotation_speed)
            
        #print(f"rotation speed: {rotation_speed}")
        #print(f"speed: {speed}")
              
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(rotation_speed)
        self.cmd_vel_publisher.publish(msg)

    def _goal_rotation(self, current_heading, goal_heading):
        diff = goal_heading - current_heading

        if diff > math.pi:
            return diff - (2 * math.pi)

        if diff < -math.pi:
            return diff - (-2 * math.pi)

        return diff