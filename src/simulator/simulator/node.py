import math

import rclpy
from geometry_msgs.msg import Pose, Twist, Point, PoseArray
from nav_msgs.msg import GridCells
from rclpy.node import Node
import numpy as np
#from messages.srv import GetMap

map_def_arr = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
]
map_def = np.array(map_def_arr)
wall_size = 1
half_wall_size = wall_size / 2
square_radius = math.sqrt(half_wall_size ** 2 + half_wall_size ** 2)
field_x_offset = -len(map_def[0]) * wall_size / 2
field_y_offset = -len(map_def) * wall_size / 2

class Simulator(Node):
    INTERVAL = 0.1

    def __init__(self) -> None:
        super().__init__('simulator')
        self.pose_publisher_ = self.create_publisher(Pose, 'pose', 1)
        self.distance_sensor_publisher_ = self.create_publisher(Point, 'distance_sensor', 1)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.handle_velocity_command, 1)
        #self.get_map_service = self.create_service(GetMap, 'add_two_ints', self.get_map_callback)
        self.pose = Pose()
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        self.timer = self.create_timer(self.INTERVAL, self.step_simulation)
        
        print("Simulator initialized")
        
    #def get_map_callback(self, request, response: GridCells):
    #    response.cell_width = wall_size
    #    response.cell_height = wall_size
    #    for y, x_list in enumerate(map_def_arr):
    #        for x, cell in enumerate(x_list):
    #            if cell:
    #                response.cells.append(Point(x=x * wall_size, y=y * wall_size, z=0))
    #    return response

    def handle_velocity_command(self, msg: Twist) -> None:
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def step_simulation(self) -> None:
        self.move_bot()
        self.run_distance_sensor()
        
    def run_distance_sensor(self):
        sensor_range = 2.0
        sensor_resolution = 5
        yaw = 2 * math.atan2(self.pose.orientation.z, self.pose.orientation.w)
        hit = False
        for i in range(0, sensor_resolution):
            point_distance = sensor_range * i / sensor_resolution + half_wall_size
            sensor_x = self.pose.position.x + point_distance * math.cos(yaw)
            sensor_y = self.pose.position.y + point_distance * math.sin(yaw)
            x_on_field, y_on_field = self.point_to_field_coords(sensor_x, sensor_y)
            x_idx, y_idx = int(x_on_field), int(y_on_field)
            if map_def[y_idx, x_idx]:
                hit = True
                self.distance_sensor_publisher_.publish(Point(x=sensor_x, y=sensor_y, z=0.0))
                break
        
        if not hit:
            max_distance = 1000
            sensor_x = self.pose.position.x + max_distance * math.cos(yaw)
            sensor_y = self.pose.position.y + max_distance * math.sin(yaw)
            self.distance_sensor_publisher_.publish(Point(x=sensor_x, y=sensor_y, z=0.0))

        
    def move_bot(self):
        yaw = 2 * math.atan2(self.pose.orientation.z, self.pose.orientation.w)
        new_x = self.pose.position.x + self.linear_velocity * math.cos(yaw) * self.INTERVAL
        new_y = self.pose.position.y + self.linear_velocity * math.sin(yaw) * self.INTERVAL
        
        x_on_field, y_on_field = self.point_to_field_coords(new_x, new_y)
        
        min_x = int(x_on_field - half_wall_size)
        max_x = int(x_on_field + half_wall_size)
        min_y = int(y_on_field - half_wall_size)
        max_y = int(y_on_field + half_wall_size)
        
        collided = np.any(map_def[min_y:max_y + 1, min_x:max_x + 1])
        
        if not collided:
            self.pose.position.x = new_x
            self.pose.position.y = new_y
        
        yaw += self.angular_velocity * self.INTERVAL
        self.pose.orientation.z = math.sin(yaw / 2)
        self.pose.orientation.w = math.cos(yaw / 2)
        self.pose_publisher_.publish(self.pose)
        
    def point_to_field_coords(self, x, y):
        return x - field_x_offset + half_wall_size, y - field_y_offset + half_wall_size
        


def main(args=None) -> None:
    rclpy.init(args=args)
    simulator = Simulator()
    rclpy.spin(simulator)
    simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
