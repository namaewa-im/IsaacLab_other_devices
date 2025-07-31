# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""8BitDo controller for SE(3) control."""

import time
import numpy as np
import weakref
from collections.abc import Callable
from scipy.spatial.transform import Rotation

import carb
import omni
import omni.appwindow

from ..device_base import DeviceBase


class Se38BitDo(DeviceBase):
    """8BitDo controller for SE(3) control with 6DOF movement and gripper control.

    This class provides a 8BitDo controller interface for robotic arm control.
    It maps controller inputs to SE(3) delta poses and gripper commands.

    The command comprises of two parts:
    * delta pose: a 6D vector of (x, y, z, roll, pitch, yaw) in meters and radians.
    * gripper: a binary command to open or close the gripper.

    Controller mappings:
        ============================ ========================= =========================
        Description                  Stick/Button (+ve axis)   Stick/Button (-ve axis)
        ============================ ========================= =========================
        Toggle gripper(open/close)   A Button                  A Button
        Move along x-axis            Left Stick Left           Left Stick Right
        Move along y-axis            Left Stick Down           Left Stick Up
        Move along z-axis            B Button (up)             X Button (down)
        Rotate along x-axis          D-Pad Left                D-Pad Right
        Rotate along y-axis          Right Stick Down          Right Stick Up
        Rotate along z-axis          Right Stick Left          Right Stick Right
        ============================ ========================= =========================
    """

    def __init__(self, pos_sensitivity: float = 0.05, rot_sensitivity: float = 1.6, dead_zone: float = 0.01):
        """Initialize the 8BitDo controller layer.

        Args:
            pos_sensitivity: Magnitude of input position command scaling. Defaults to 0.05.
            rot_sensitivity: Magnitude of scale input rotation commands scaling. Defaults to 1.6.
            dead_zone: Magnitude of dead zone for controller. An event value less than
                this value will be ignored. Defaults to 0.01.
        """
        # turn off simulator gamepad control
        carb_settings_iface = carb.settings.get_settings_interface()
        carb_settings_iface.set_bool("/persistent/app/omniverse/gamepadCameraControl", False)
        
        # store inputs
        self.pos_sensitivity = pos_sensitivity
        self.rot_sensitivity = rot_sensitivity
        self.dead_zone = dead_zone
        
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
        
        # command buffers
        self._close_gripper = False
        self.last_gripper_toggle_time = time.time()
        
        # When using the gamepad, two values are provided for each axis.
        # (positive, negative), (x, y, z, roll, pitch, yaw)
        self._delta_pose_raw = np.zeros([2, 6])
        
        # dictionary for additional callbacks
        self._additional_callbacks = dict()

    def __del__(self):
        """Unsubscribe from gamepad events."""
        self._input.unsubscribe_from_gamepad_events(self._gamepad, self._gamepad_sub)
        self._gamepad_sub = None

    def __str__(self) -> str:
        """Returns: A string containing the information of controller."""
        msg = f"8BitDo Controller for SE(3): {self.__class__.__name__}\n"
        msg += f"\tDevice name: {self._input.get_gamepad_name(self._gamepad)}\n"
        msg += "\t----------------------------------------------\n"
        msg += "\tToggle gripper (open/close): A\n"
        msg += "\tMove arm along x-axis: Left Stick Left/Right\n"
        msg += "\tMove arm along y-axis: Left Stick Down/Up\n"
        msg += "\tMove arm along z-axis: B (up) / X (down)\n"
        msg += "\tRotate arm along x-axis: D-Pad Left/Right\n"
        msg += "\tRotate arm along y-axis: Right Stick Down/Up\n"
        msg += "\tRotate arm along z-axis: Right Stick Left/Right\n"
        return msg

    def reset(self):
        """Reset the internals."""
        self._close_gripper = False
        self._delta_pose_raw.fill(0.0)
        print("[DEBUG] 8BitDo Controller position reset to initial state.")

    def add_callback(self, key: carb.input.GamepadInput, func: Callable):
        """Add additional functions to bind controller buttons.

        Args:
            key: The button to check against.
            func: The function to call when key is pressed. The callback function should not
                take any arguments.
        """
        self._additional_callbacks[key] = func

    def advance(self) -> tuple[np.ndarray, bool]:
        """Provides the controller event state.

        Returns:
            A tuple containing:
                - delta_pose: A 6D vector of (x, y, z, roll, pitch, yaw) in meters and radians.
                - gripper_state: A boolean indicating whether the gripper should be closed.
        """
        # resolve the command buffer
        delta_pose = self._resolve_command_buffer(self._delta_pose_raw)
        return delta_pose, self._close_gripper

    def _on_gamepad_event(self, event, *args, **kwargs):
        """Process gamepad events and update command buffers."""
        current_time = time.time()
        
        # get the event input and value
        event_input = event.input
        event_value = event.value
        
        # check if the event is a button press
        cur_val = event_value
        if abs(cur_val) < self.dead_zone:
            cur_val = 0
            
        # Process continuous axis values for Nintendo Pro Controller
        # Left stick X-axis (ABS_X) - Left/Right movement
        if event_input == carb.input.GamepadInput.LEFT_STICK_LEFT:
            # Normalize to [0, 1] range and apply sensitivity
            normalized_val = abs(cur_val) / 32767.0  # Max value from evtest
            if normalized_val > self.dead_zone:
                self._delta_pose_raw[1, 0] = normalized_val * self.pos_sensitivity  # Left movement
            else:
                self._delta_pose_raw[1, 0] = 0.0
                
        elif event_input == carb.input.GamepadInput.LEFT_STICK_RIGHT:
            # Normalize to [0, 1] range and apply sensitivity
            normalized_val = abs(cur_val) / 32767.0  # Max value from evtest
            if normalized_val > self.dead_zone:
                self._delta_pose_raw[0, 0] = normalized_val * self.pos_sensitivity  # Right movement
            else:
                self._delta_pose_raw[0, 0] = 0.0
                
        # Left stick Y-axis (ABS_Y) - Forward/Backward movement
        elif event_input == carb.input.GamepadInput.LEFT_STICK_UP:
            # Normalize to [0, 1] range and apply sensitivity
            normalized_val = abs(cur_val) / 32767.0  # Max value from evtest
            if normalized_val > self.dead_zone:
                self._delta_pose_raw[0, 1] = normalized_val * self.pos_sensitivity  # Forward movement
            else:
                self._delta_pose_raw[0, 1] = 0.0
                
        elif event_input == carb.input.GamepadInput.LEFT_STICK_DOWN:
            # Normalize to [0, 1] range and apply sensitivity
            normalized_val = abs(cur_val) / 32767.0  # Max value from evtest
            if normalized_val > self.dead_zone:
                self._delta_pose_raw[1, 1] = normalized_val * self.pos_sensitivity  # Backward movement
            else:
                self._delta_pose_raw[1, 1] = 0.0
                
        # Right stick Y-axis (ABS_RY) for Z movement
        elif event_input == carb.input.GamepadInput.RIGHT_STICK_UP:
            # Normalize to [0, 1] range and apply sensitivity
            normalized_val = abs(cur_val) / 32767.0  # Max value from evtest
            if normalized_val > self.dead_zone:
                self._delta_pose_raw[0, 2] = normalized_val * self.pos_sensitivity  # Up movement
            else:
                self._delta_pose_raw[0, 2] = 0.0
                
        elif event_input == carb.input.GamepadInput.RIGHT_STICK_DOWN:
            # Normalize to [0, 1] range and apply sensitivity
            normalized_val = abs(cur_val) / 32767.0  # Max value from evtest
            if normalized_val > self.dead_zone:
                self._delta_pose_raw[1, 2] = normalized_val * self.pos_sensitivity  # Down movement
            else:
                self._delta_pose_raw[1, 2] = 0.0
                
        # Right stick X-axis (ABS_RX) for Yaw rotation
        elif event_input == carb.input.GamepadInput.RIGHT_STICK_LEFT:
            # Normalize to [0, 1] range and apply sensitivity
            normalized_val = abs(cur_val) / 32767.0  # Max value from evtest
            if normalized_val > self.dead_zone:
                self._delta_pose_raw[1, 5] = normalized_val * self.rot_sensitivity  # Counter-clockwise rotation
            else:
                self._delta_pose_raw[1, 5] = 0.0
                
        elif event_input == carb.input.GamepadInput.RIGHT_STICK_RIGHT:
            # Normalize to [0, 1] range and apply sensitivity
            normalized_val = abs(cur_val) / 32767.0  # Max value from evtest
            if normalized_val > self.dead_zone:
                self._delta_pose_raw[0, 5] = normalized_val * self.rot_sensitivity  # Clockwise rotation
            else:
                self._delta_pose_raw[0, 5] = 0.0
        
        # D-pad for Roll rotation (ABS_HAT0X)
        elif event_input == carb.input.GamepadInput.DPAD_LEFT:
            if cur_val > 0.5:
                self._delta_pose_raw[0, 3] = self.rot_sensitivity  # Roll rotation
                
        elif event_input == carb.input.GamepadInput.DPAD_RIGHT:
            if cur_val > 0.5:
                self._delta_pose_raw[0, 3] = -self.rot_sensitivity  # Roll rotation
                
        # D-pad for Pitch rotation (ABS_HAT0Y)
        elif event_input == carb.input.GamepadInput.DPAD_UP:
            if cur_val > 0.5:
                self._delta_pose_raw[0, 4] = self.rot_sensitivity  # Pitch rotation
                
        elif event_input == carb.input.GamepadInput.DPAD_DOWN:
            if cur_val > 0.5:
                self._delta_pose_raw[0, 4] = -self.rot_sensitivity  # Pitch rotation
        
        # process button events
        elif event_input == carb.input.GamepadInput.A_BUTTON:
            if event_value == 1 and (current_time - self.last_gripper_toggle_time) > 0.7:
                self._close_gripper = not self._close_gripper
                self.last_gripper_toggle_time = current_time
                print(f"[DEBUG] Gripper state changed: {self._close_gripper}")
                
        elif event_input == carb.input.GamepadInput.B_BUTTON:
            if event_value == 1:
                self._delta_pose_raw[0, 2] = self.pos_sensitivity  # Z up
                
        elif event_input == carb.input.GamepadInput.X_BUTTON:
            if event_value == 1:
                self._delta_pose_raw[0, 2] = -self.pos_sensitivity  # Z down
        
        # call additional callbacks
        if event_input in self._additional_callbacks:
            self._additional_callbacks[event_input]()

    def _create_key_bindings(self):
        """Creates default key binding."""
        self._INPUT_STICK_VALUE_MAPPING = {
            # forward command
            carb.input.GamepadInput.LEFT_STICK_UP: (0, 0, self.pos_sensitivity),
            # backward command
            carb.input.GamepadInput.LEFT_STICK_DOWN: (1, 0, self.pos_sensitivity),
            # right command
            carb.input.GamepadInput.LEFT_STICK_RIGHT: (0, 1, self.pos_sensitivity),
            # left command
            carb.input.GamepadInput.LEFT_STICK_LEFT: (1, 1, self.pos_sensitivity),
            # upward command
            carb.input.GamepadInput.RIGHT_STICK_UP: (0, 2, self.pos_sensitivity),
            # downward command
            carb.input.GamepadInput.RIGHT_STICK_DOWN: (1, 2, self.pos_sensitivity),
            # yaw command (positive)
            carb.input.GamepadInput.RIGHT_STICK_RIGHT: (0, 5, self.rot_sensitivity),
            # yaw command (negative)
            carb.input.GamepadInput.RIGHT_STICK_LEFT: (1, 5, self.rot_sensitivity),
        }

    def _resolve_command_buffer(self, raw_command: np.ndarray) -> np.ndarray:
        """Resolve the command buffer to get the final delta pose.

        Args:
            raw_command: Raw command buffer of shape (2, 6).

        Returns:
            Resolved delta pose as a 6D vector.
        """
        # take the maximum value between positive and negative for each axis
        delta_pose = np.maximum(raw_command[0], raw_command[1])
        
        # convert rotation from euler angles to rotation vector
        rot_vec = Rotation.from_euler("XYZ", delta_pose[3:]).as_rotvec()
        
        # return [x, y, z, roll_rotvec, pitch_rotvec, yaw_rotvec]
        return np.concatenate([delta_pose[:3], rot_vec]) 