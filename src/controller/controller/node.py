#import sys
#print(sys.path)

from controller.MotorController import MotorController
from enum import Enum
from controller.geometry import heading_diff
import numpy as np
import random
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, Point
from std_msgs.msg import Float32

class RobotState(Enum):
    PICKING_WANDER_HEADING = 1
    FOLLOWING_WANDER_HEADING = 2
    FOLLOWING_BALL = 3
    LOST_BALL = 4
    BACKING_UP = 5

wander_heading_threshold = (5 / 360) * (2 * math.pi)
follow_wander_heading_clearance = 1.0
wall_clearance = 1.0
speed = 0.5

def period_print(string):
    timestamp = int(time.time())
    if timestamp % 2 == 0:
        print(string)

def wrap(num, min, max):
    if max <= min:
        raise Exception(f"max must be greater than min")

    if num > max:
        return min + num - max
    elif num < min:
        return max + num - min
    else:
        return num

class Robot(Node):
    INTERVAL = 0.1
    
    def __init__(self):
        super().__init__('controller')
        self.sensor_distance = math.inf
        self.sensor_subscription = self.create_subscription(Point, 'distance_sensor', self.handle_sensor_hit, 1)
        
        self.pose = None
        self.subscription = self.create_subscription(Pose, 'pose', self.handle_pose, 1)
        
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.motor_controller = MotorController(self.cmd_vel_publisher)
        self.timer = self.create_timer(self.INTERVAL, self.step)
        
        self.ball_heading_subscription = self.create_subscription(Float32, 'ball_heading', self.handle_ball_heading, 1)

        #self.node = Node(ctx, headers={"Role": "Robot"}, groups=["BallPositionFeed"])
        #self.node.whisper_recieved += self.whisper_recieved
        #self.node.start()

        self.state = RobotState.PICKING_WANDER_HEADING
        self.wander_heading = None
        self.last_ball_heading_time = None
        self.ball_relative_heading = None

        self.state_lookup = {}
        self.state_lookup[RobotState.PICKING_WANDER_HEADING] = self._picking_wander_heading
        self.state_lookup[RobotState.FOLLOWING_WANDER_HEADING] = self._following_wander_heading
        self.state_lookup[RobotState.FOLLOWING_BALL] = self._following_ball
        self.state_lookup[RobotState.BACKING_UP] = self._backing_up
        
    def handle_sensor_hit(self, msg: Point) -> None:
        if self.pose is None:
            return
        
        self.sensor_distance = math.sqrt((self.pose.position.x - msg.x) ** 2 + (self.pose.position.y - msg.y) ** 2)
        
    def handle_pose(self, msg: Pose) -> None:
        self.pose = msg

    def handle_ball_heading(self, msg: Float32) -> None:
        self.last_ball_heading_time = time.time()
        self.ball_relative_heading = float(msg.data)

    def _set_state(self, state):
        print(f"Transition from {self.state} to {state}")
        self.state = state

    def _obstructed(self):
        return self.sensor_distance < wall_clearance

    def _picking_wander_heading(self, current_heading):
        if self.ball_relative_heading is not None:
            self._set_state(RobotState.FOLLOWING_BALL)
            self.wander_heading = None
            return

        if self.wander_heading is None:
            opposite_heading = current_heading + math.pi
            wander_heading = opposite_heading + random.uniform(-0.5 * math.pi, 0.5 * math.pi)
            if wander_heading > 2 * math.pi:
                wander_heading = wander_heading - 2 * math.pi
            self.wander_heading = wander_heading
            print(f"Set Wander Heading {self.wander_heading}")

        self.motor_controller.step(current_heading, self.wander_heading, 0, turn_strength=0.3)

        if abs(heading_diff(current_heading, self.wander_heading)) < wander_heading_threshold:
            print("Wander heading reached")
            if self.sensor_distance < follow_wander_heading_clearance:
                self.wander_heading = None
                self.motor_controller.step(current_heading, current_heading, 0)
            else:
                self._set_state(RobotState.FOLLOWING_WANDER_HEADING)

    def _following_wander_heading(self, current_heading):
        if self.ball_relative_heading is not None:
            self._set_state(RobotState.FOLLOWING_BALL)
            self.wander_heading = None
            return

        if self._obstructed():
            self._set_state(RobotState.BACKING_UP)
            self.wander_heading = None
            self.motor_controller.step(current_heading, current_heading, 0)
            return

        self.motor_controller.step(current_heading, self.wander_heading, speed, turn_strength=0.5)

    def _following_ball(self, current_heading):

        if self._obstructed():
            self._set_state(RobotState.BACKING_UP)
            self.wander_heading = None
            self.motor_controller.step(current_heading, current_heading, 0)
            return

        if self.ball_relative_heading is None or (self.last_ball_heading_time and (time.time() - self.last_ball_heading_time) > 2):
            self._set_state(RobotState.PICKING_WANDER_HEADING)
            self.wander_heading = None
            
            relative_heading = 0
            follow_speed = 0
        else:
            relative_heading = self.ball_relative_heading
            follow_speed = speed
            
        goal_heading = current_heading + relative_heading
        if goal_heading > 2 * math.pi:
            goal_heading = goal_heading - 2 * math.pi

        self.motor_controller.step(current_heading, goal_heading, follow_speed)

    def _backing_up(self, current_heading):
        if self._obstructed():
            self.motor_controller.step(current_heading, current_heading, -speed)
        else:
            self._set_state(RobotState.PICKING_WANDER_HEADING)
            self.wander_heading = None
            self.motor_controller.step(current_heading, current_heading, 0)

    def step(self):
        if self.last_ball_heading_time and (time.time() - self.last_ball_heading_time) > 0.5:
            self.last_ball_heading_time = None
            self.ball_relative_heading = None
            
        current_heading = (2 * math.atan2(self.pose.orientation.z, self.pose.orientation.w)) if self.pose is not None else 0
        self.state_lookup[self.state](current_heading)
        
def main(args=None) -> None:
    rclpy.init(args=args)
    controller = Robot()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()