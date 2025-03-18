import time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation
from ..device_base import DeviceBase


class Se3Extreme3DPro(DeviceBase, Node):
    """Extreme 3D Pro Joystick for SE(3) control"""

    def __init__(self, pos_sensitivity=0.05, rot_sensitivity=1.6, dead_zone=0.01):
        if not rclpy.ok():
            rclpy.init()

        DeviceBase.__init__(self)
        Node.__init__(self, 'se3_extreme3dpro')

        self.pos_sensitivity = pos_sensitivity
        self.rot_sensitivity = rot_sensitivity
        self.dead_zone = dead_zone

        self.subscription = self.create_subscription(
            Joy, '/joy', self._on_joy_event, qos_profile_sensor_data
        )
        print("[DEBUG] Subscription to /joy topic created")

        self._close_gripper = False
        self._delta_pose = np.zeros(6)
        self._additional_callbacks = {}
        self.last_gripper_toggle_time = time.time()

    def _on_joy_event(self, msg: Joy):
        """ROS 2 Joy message callback"""
        current_time = time.time()

        # 조이스틱 입력 매핑
        move_x = msg.axes[1] * self.pos_sensitivity if abs(msg.axes[1]) > self.dead_zone else 0.0  # 앞뒤 이동
        move_y = msg.axes[0] * self.pos_sensitivity if abs(msg.axes[0]) > self.dead_zone else 0.0
        move_z = 0.0

        rotate_roll = msg.axes[4] * self.pos_sensitivity if abs(msg.axes[4]) > self.dead_zone else 0.0
        rotate_pitch = msg.axes[5] * self.pos_sensitivity if abs(msg.axes[5]) > self.dead_zone else 0.0
        rotate_yaw = msg.axes[2] * self.rot_sensitivity if abs(msg.axes[2]) > self.dead_zone else 0.0  # 그리퍼 회전


        # 버튼 1을 누르면 하강, 버튼 4를 누르면 상승
        if msg.buttons[0] == 1:
            move_z = -self.pos_sensitivity
        if msg.buttons[3] == 1:
            move_z = self.pos_sensitivity
        
        self._delta_pose = np.array([move_x, move_y, move_z, rotate_roll, rotate_pitch, rotate_yaw])

        # 버튼 2번 (그리퍼 조작)
        if msg.buttons[1] == 1 and (current_time - self.last_gripper_toggle_time) > 0.7:
            self._close_gripper = not self._close_gripper
            self.last_gripper_toggle_time = current_time
            # print(f"[DEBUG] Gripper state changed: {self._close_gripper}")


    def add_callback(self, key: int, func):
        """Add additional functions to bind joystick buttons"""
        self._additional_callbacks[key] = func

    def reset(self):
        """Reset joystick state"""
        self._close_gripper = False
        self._delta_pose.fill(0.0)
        print("[DEBUG] Joystick position reset to initial state.")

    def advance(self) -> tuple[np.ndarray, bool]:
        """Provides the result from joystick event state"""
        rclpy.spin_once(self, timeout_sec=0.01)  # 메시지를 한 번 처리한 후 반환
        rot_vec = Rotation.from_euler("XYZ", self._delta_pose[3:]).as_rotvec()
        return np.concatenate([self._delta_pose[:3], rot_vec]), self._close_gripper
