import math
import threading
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose, Twist, Point, Quaternion
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nicegui import Client, app, ui, ui_run

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
wall_size = 1
field_x_offset = -len(map_def_arr[0]) * wall_size / 2
field_y_offset = -len(map_def_arr) * wall_size / 2

class NiceGuiNode(Node):

    def __init__(self) -> None:
        super().__init__('nicegui')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.subscription = self.create_subscription(Pose, 'pose', self.handle_pose, 1)
        self.sensor_subscription = self.create_subscription(Point, 'distance_sensor', self.handle_sensor_hit, 1)
        
        self.cmd_ball_pose_publisher = self.create_publisher(Pose, 'cmd_ball_pose', 1)

        with Client.auto_index_client:
            with ui.row().classes('items-stretch'):
                with ui.card().classes('w-44 text-center items-center'):
                    ui.label('Control').classes('text-2xl')
                    ui.joystick(color='blue', size=50,
                                on_move=lambda e: self.send_speed(float(e.y), float(e.x)),
                                on_end=lambda _: self.send_speed(0.0, 0.0))
                    ui.label('Publish steering commands by dragging your mouse around in the blue field').classes('mt-6')
                with ui.card().classes('w-44 text-center items-center'):
                    ui.label('Data').classes('text-2xl')
                    ui.label('linear velocity').classes('text-xs mb-[-1.8em]')
                    slider_props = 'readonly selection-color=transparent'
                    self.linear = ui.slider(min=-1, max=1, step=0.05, value=0).props(slider_props)
                    ui.label('angular velocity').classes('text-xs mb-[-1.8em]')
                    self.angular = ui.slider(min=-1, max=1, step=0.05, value=0).props(slider_props)
                    ui.label('position').classes('text-xs mb-[-1.4em]')
                    self.position = ui.label('---')
                    self.orientation = ui.label('---')
                    self.yaw = ui.label('---')
                with ui.card().classes('w-96 h-96 items-center'):
                    ui.label('Visualization').classes('text-2xl')
                    with ui.scene(350, 300, drag_constraints='z=0') as scene:
                        scene.move_camera(0, 0, 10)
                        with scene.group() as self.field:
                            for y, x_list in enumerate(map_def_arr):
                                for x, cell in enumerate(x_list):
                                    if cell:
                                        square = [[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]
                                        scene.extrusion(square, 1).move(x * wall_size + field_x_offset, y * wall_size + field_y_offset).material('#4488ff')
                        with scene.group() as self.robot_3d:
                            prism = [[-0.5, -0.5], [0.5, -0.5], [0.75, 0], [0.5, 0.5], [-0.5, 0.5]]
                            self.robot_object = scene.extrusion(prism, 0.4).material('#4488ff', 0.5)
                        with scene.group() as self.sensor_hit:
                            prism = [[-0.05, -0.05], [0.05, -0.05], [0.05, 0.05], [-0.05, 0.05]]
                            self.sensor_hit_object = scene.extrusion(prism, 1.5).material('#ff0000', 0.5)
                        self.sensor_hit.visible(False)
                        with scene.group() as self.ball:
                            self.ball_object = scene.sphere(0.25).material('#ff0000').draggable()
                        self.ball.move(0, 2, 0)
                            
                        scene.on('drag', self.on_drag)
                            
    def on_drag(self, e) -> None:
        if e.args['object_id'] == self.ball.id:
            ball_pose = Pose(
                position=Point(x=float(e.args['x']), y=float(e.args['y']), z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            )
            self.cmd_ball_pose_publisher.publish(ball_pose)

    def send_speed(self, x: float, y: float) -> None:
        msg = Twist()
        msg.linear.x = x
        msg.angular.z = -y
        self.linear.value = x
        self.angular.value = y
        self.cmd_vel_publisher.publish(msg)

    def handle_pose(self, msg: Pose) -> None:
        self.position.text = f'x: {msg.position.x:.2f}, y: {msg.position.y:.2f}'
        self.orientation.text = f'z: {msg.orientation.z:.2f}, w: {msg.orientation.w:.2f}'
        self.yaw.text = f'yaw: {2 * math.atan2(msg.orientation.z, msg.orientation.w):.2f}'
        self.robot_3d.move(msg.position.x, msg.position.y)
        self.robot_3d.rotate(0, 0, 2 * math.atan2(msg.orientation.z, msg.orientation.w))

    def handle_sensor_hit(self, msg: Point) -> None:
        robot_x, robot_y = self.robot_3d.x, self.robot_3d.y
        sensor_x, sensor_y = msg.x, msg.y
        distance = math.sqrt((robot_x - sensor_x) ** 2 + (robot_y - sensor_y) ** 2)
        if distance < 10:
            self.sensor_hit.move(msg.x, msg.y)
            self.sensor_hit.visible(True)
        else:
            self.sensor_hit.visible(False)


def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
    pass


def ros_main() -> None:
    rclpy.init()
    node = NiceGuiNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


app.on_startup(lambda: threading.Thread(target=ros_main).start())
ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖')
