# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""8BitDo controller for SE(2) control."""

import time
import numpy as np
import weakref
from collections.abc import Callable
from scipy.spatial.transform import Rotation

import carb.input
import omni
import omni.appwindow

from ..device_base import DeviceBase


class Se28BitDo(DeviceBase):
    """8BitDo controller for SE(2) control (2D plane movement and rotation).

    This class provides a 8BitDo controller interface for 2D robotic control.
    It maps controller inputs to SE(2) delta poses and gripper commands.

    The command comprises of two parts:
    * delta pose: a 3D vector of (x, y, yaw) in meters and radians.
    * gripper: a binary command to open or close the gripper.

    Controller mappings:
        ============================ ========================= =========================
        Description                  Stick/Button (+ve axis)   Stick/Button (-ve axis)
        ============================ ========================= =========================
        Boost (가속도)               A Button                  A Button
        Move along x-axis            Left Stick Up             Left Stick Down
        Move along y-axis            Left Stick Right          Left Stick Left
        Rotate along z-axis          Right Shoulder            Left Shoulder
        Fine adjustment              D-Pad Left/Right          D-Pad Left/Right
        ============================ ========================= =========================
    """

    def __init__(self, pos_sensitivity: float = 0.05, rot_sensitivity: float = 3.0, dead_zone: float = 0.01, left: int = 0, boost_multiplier: float = 2.0):
        """Initialize the 8BitDo controller layer for SE(2).

        Args:
            pos_sensitivity: Magnitude of input position command scaling. Defaults to 0.05.
            rot_sensitivity: Magnitude of scale input rotation commands scaling. Defaults to 3.0.
            dead_zone: Magnitude of dead zone for controller. An event value less than
                this value will be ignored. Defaults to 0.01.
            left: Number of 90-degree counterclockwise rotations for controller mapping.
                0: 기본 모드, 1: 90도 회전, 2: 180도 회전, 3: 270도 회전. Defaults to 0.
            boost_multiplier: Multiplier for sensitivity when boost button is pressed. Defaults to 2.0.
        """
        # turn off simulator gamepad control
        carb_settings_iface = carb.settings.get_settings_interface()
        carb_settings_iface.set_bool("/persistent/app/omniverse/gamepadCameraControl", False)
        
        # store inputs
        self.pos_sensitivity = pos_sensitivity
        self.rot_sensitivity = rot_sensitivity
        self.dead_zone = dead_zone
        self.left = left % 4  # 0, 1, 2, 3으로 제한
        self.boost_multiplier = boost_multiplier
        
        # acquire omniverse interfaces
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._gamepad = self._appwindow.get_gamepad(0)
        
        # note: Use weakref on callbacks to ensure that this object can be deleted when its destructor is called
        self._gamepad_sub = self._input.subscribe_to_gamepad_events(
            self._gamepad,
            lambda event, *args, obj=weakref.proxy(self): obj._on_gamepad_event(event, *args),
        )
        
        # bindings for controller to command
        self._create_key_bindings()
        
        # command buffers - 키보드처럼 3D 배열로 변경
        self._base_command = np.zeros(3)
        # 각 방향별 현재 입력 상태 저장
        self._input_state = {
            'forward': 0.0,
            'backward': 0.0,
            'right': 0.0,
            'left': 0.0,
            'yaw_left': 0.0,
            'yaw_right': 0.0,
            'boost': 0.0  # 가속도 버튼 상태 추가
        }
        self.last_gripper_toggle_time = time.time()
        self._additional_callbacks = dict()
        # 버튼 상태 추적용 딕셔너리
        self.previous_button_state = {
            carb.input.GamepadInput.LEFT_STICK_UP: False,
            carb.input.GamepadInput.LEFT_STICK_DOWN: False,
            carb.input.GamepadInput.LEFT_STICK_LEFT: False,
            carb.input.GamepadInput.LEFT_STICK_RIGHT: False,
            # 필요시 추가
        }

    def __del__(self):
        """Unsubscribe from gamepad events."""
        self._input.unsubscribe_from_gamepad_events(self._gamepad, self._gamepad_sub)
        self._gamepad_sub = None

    def __str__(self) -> str:
        """Returns: A string containing the information of controller."""
        msg = f"8BitDo Controller for SE(2): {self.__class__.__name__}\n"
        msg += f"\tDevice name: {self._input.get_gamepad_name(self._gamepad)}\n"
        msg += f"\tLeft-handed mode: {self.left}\n"
        msg += "\t----------------------------------------------\n"
        if self.left == 1:
            msg += "\tController rotated 90° counterclockwise for left-handed grip\n"
            msg += "\tMove forward: Left Stick Right\n"
            msg += "\tMove backward: Left Stick Left\n"
            msg += "\tMove left: Left Stick Up\n"
            msg += "\tMove right: Left Stick Down\n"
        elif self.left == 2:
            msg += "\tController rotated 180° counterclockwise for left-handed grip\n"
            msg += "\tMove forward: Left Stick Down\n"
            msg += "\tMove backward: Left Stick Up\n"
            msg += "\tMove left: Left Stick Right\n"
            msg += "\tMove right: Left Stick Left\n"
        elif self.left == 3:
            msg += "\tController rotated 270° counterclockwise for left-handed grip\n"
            msg += "\tMove forward: Left Stick Left\n"
            msg += "\tMove backward: Left Stick Right\n"
            msg += "\tMove left: Left Stick Down\n"
            msg += "\tMove right: Left Stick Up\n"
        else:
            msg += "\tMove forward: Left Stick Up\n"
            msg += "\tMove backward: Left Stick Down\n"
            msg += "\tMove left: Left Stick Left\n"
            msg += "\tMove right: Left Stick Right\n"
        msg += "\tRotate: Right Stick Left/Right\n"
        msg += "\tBoost: Left Shoulder ({}x multiplier)\n".format(self.boost_multiplier)
        msg += "\tSensitivity - Position: {:.3f}, Rotation: {:.3f}\n".format(self.pos_sensitivity, self.rot_sensitivity)
        return msg

    def reset(self):
        """Reset the internals."""
        self._base_command.fill(0.0)
        print("[DEBUG] 8BitDo Controller (SE2) position reset to initial state.")

    def add_callback(self, key: carb.input.GamepadInput, func: Callable):
        """Add additional functions to bind controller buttons.

        Args:
            key: The button to check against.
            func: The function to call when key is pressed. The callback function should not
                take any arguments.
        """
        self._additional_callbacks[key] = func

    def advance(self) -> tuple[np.ndarray, bool]:
        """Provides the controller event state for SE(2).

        Returns:
            A tuple containing:
                - delta_pose: A 6D vector of (x, y, 0, 0, 0, yaw_rotvec) in meters and radians.
                - gripper_state: A boolean indicating whether the gripper should be closed.
        """
        # 키보드처럼 3D 배열 반환
        return self._base_command, False

    def _on_gamepad_event(self, event, *args, **kwargs):
        """Process gamepad events and update command buffers for SE(2)."""
        current_time = time.time()
        
        # get the event input and value
        event_input = event.input
        event_value = event.value
        
        # dead zone 적용
        if abs(event_value) < self.dead_zone:
            event_value = 0.0
            
        # 입력 상태 갱신 - left 옵션에 따라 매핑 변경
        if self.left == 1:
            # 90도 반시계방향 회전: UP->LEFT, LEFT->DOWN, DOWN->RIGHT, RIGHT->UP
            if event_input == carb.input.GamepadInput.LEFT_STICK_UP:
                self._input_state['left'] = event_value
            elif event_input == carb.input.GamepadInput.LEFT_STICK_DOWN:
                self._input_state['right'] = event_value
            elif event_input == carb.input.GamepadInput.LEFT_STICK_RIGHT:
                self._input_state['forward'] = event_value
            elif event_input == carb.input.GamepadInput.LEFT_STICK_LEFT:
                self._input_state['backward'] = event_value
        elif self.left == 2:
            # 180도 반시계방향 회전: UP->DOWN, LEFT->RIGHT, DOWN->UP, RIGHT->LEFT
            if event_input == carb.input.GamepadInput.LEFT_STICK_UP:
                self._input_state['backward'] = event_value
            elif event_input == carb.input.GamepadInput.LEFT_STICK_DOWN:
                self._input_state['forward'] = event_value
            elif event_input == carb.input.GamepadInput.LEFT_STICK_RIGHT:
                self._input_state['left'] = event_value
            elif event_input == carb.input.GamepadInput.LEFT_STICK_LEFT:
                self._input_state['right'] = event_value
        elif self.left == 3:
            # 270도 반시계방향 회전: UP->RIGHT, LEFT->UP, DOWN->LEFT, RIGHT->DOWN
            if event_input == carb.input.GamepadInput.LEFT_STICK_UP:
                self._input_state['right'] = event_value
            elif event_input == carb.input.GamepadInput.LEFT_STICK_DOWN:
                self._input_state['left'] = event_value
            elif event_input == carb.input.GamepadInput.LEFT_STICK_RIGHT:
                self._input_state['backward'] = event_value
            elif event_input == carb.input.GamepadInput.LEFT_STICK_LEFT:
                self._input_state['forward'] = event_value
        else:
            # 기본 매핑
            if event_input == carb.input.GamepadInput.LEFT_STICK_UP:
                self._input_state['forward'] = event_value
            elif event_input == carb.input.GamepadInput.LEFT_STICK_DOWN:
                self._input_state['backward'] = event_value
            elif event_input == carb.input.GamepadInput.LEFT_STICK_RIGHT:
                self._input_state['right'] = event_value
            elif event_input == carb.input.GamepadInput.LEFT_STICK_LEFT:
                self._input_state['left'] = event_value
        
        # 숄더 버튼으로 회전 처리 (키보드처럼 고정값 사용)
        if event_input == carb.input.GamepadInput.LEFT_SHOULDER:
            if event_value > 0:
                self._input_state['yaw_right'] = 1.0
            else:
                self._input_state['yaw_right'] = 0.0
            print(f"[DEBUG] 왼쪽 숄더 (우회전): {event_value}")
        elif event_input == carb.input.GamepadInput.RIGHT_SHOULDER:
            if event_value > 0:
                self._input_state['yaw_left'] = 1.0
            else:
                self._input_state['yaw_left'] = 0.0
            print(f"[DEBUG] 오른쪽 숄더 (좌회전): {event_value}")
        
        # 가속도 버튼 처리 (A 버튼으로 변경)
        elif event_input == carb.input.GamepadInput.A:
            self._input_state['boost'] = event_value
            print(f"[DEBUG] 가속도 버튼 (A): {event_value}")
        
        # base_command 직접 세팅 - 가속도 적용
        boost_factor = self.boost_multiplier if self._input_state['boost'] > 0 else 1.0
        x = self._input_state['forward'] * self.pos_sensitivity * boost_factor - self._input_state['backward'] * self.pos_sensitivity * boost_factor
        # y축(좌우) 방향만 반대로
        y = self._input_state['left'] * self.pos_sensitivity * boost_factor - self._input_state['right'] * self.pos_sensitivity * boost_factor
        yaw = self._input_state['yaw_right'] * self.rot_sensitivity * boost_factor - self._input_state['yaw_left'] * self.rot_sensitivity * boost_factor
        self._base_command = np.array([x, y, yaw])
        
        # 디버그 출력
        boost_status = "ON" if self._input_state['boost'] > 0 else "OFF"
        print(f"[DEBUG] 입력상태: {self._input_state}, base_command: {self._base_command}, Boost: {boost_status}")
        
        # 추가 콜백
        if event_input in self._additional_callbacks:
            self._additional_callbacks[event_input]()

    def _create_key_bindings(self):
        """Creates default key binding."""
        self._INPUT_STICK_VALUE_MAPPING = {
            # forward command (positive X) - 위로
            carb.input.GamepadInput.LEFT_STICK_UP: np.asarray([1.0, 0.0, 0.0]) * self.pos_sensitivity,
            # backward command (negative X) - 아래로
            carb.input.GamepadInput.LEFT_STICK_DOWN: np.asarray([-1.0, 0.0, 0.0]) * self.pos_sensitivity,
            # right command (positive Y) - 오른쪽
            carb.input.GamepadInput.LEFT_STICK_RIGHT: np.asarray([0.0, 1.0, 0.0]) * self.pos_sensitivity,
            # left command (negative Y) - 왼쪽
            carb.input.GamepadInput.LEFT_STICK_LEFT: np.asarray([0.0, -1.0, 0.0]) * self.pos_sensitivity,
            # rotation commands - 숄더 버튼으로 변경
            carb.input.GamepadInput.LEFT_SHOULDER: np.asarray([0.0, 0.0, 1.0]) * self.rot_sensitivity,
            carb.input.GamepadInput.RIGHT_SHOULDER: np.asarray([0.0, 0.0, -1.0]) * self.rot_sensitivity
        } 