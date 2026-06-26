#!/usr/bin/env python

import logging
import time
from functools import cached_property
from typing import Any, Optional

import numpy as np

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from lerobot.robots.robot import Robot
from .config_franka_research3 import ControlMode, FrankaResearch3Config
from franky import (
    Affine,
    CartesianImpedanceTracker,
    CartesianMotion,
    CartesianVelocityMotion,
    Duration,
    JointMotion,
    JointVelocityMotion,
    ReferenceType,
    RelativeDynamicsFactor,
    Robot as FrankyRobot,
    Twist,
)

from .robot_utils import (
    matrix_to_pose7d,
    quaternion_to_euler,
    rotation_6d_to_quaternion,
)

logger = logging.getLogger(__name__)

JOINT_DOF = 7


def _get_serial_gripper_class():
    try:
        from .serial_gripper import SerialGripper
    except ImportError as exc:
        raise ImportError(
            "Serial gripper backend requires xensegripper support. "
            "Please ensure the Xense gripper SDK dependencies are installed."
        ) from exc
    return SerialGripper


def _quat_wxyz_to_xyzw(quat_wxyz: np.ndarray) -> np.ndarray:
    """Convert internal [qw, qx, qy, qz] quaternions to Franky/Scipy [qx, qy, qz, qw]."""
    quat_wxyz = np.asarray(quat_wxyz, dtype=np.float64)
    if quat_wxyz.shape != (4,):
        raise ValueError(f"Expected quaternion shape (4,), got {quat_wxyz.shape}")
    return np.array(
        [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]],
        dtype=np.float64,
    )


def _slerp_quat_xyzw(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    """SLERP between two [qx,qy,qz,qw] quaternions. t=1 returns q1, t=0 returns q0.

    Used as the rotation-side equivalent of a first-order low-pass on a
    streamed Cartesian target.
    """
    q0 = np.asarray(q0, dtype=np.float64)
    q1 = np.asarray(q1, dtype=np.float64)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        # Nearly identical orientations — linear blend + renormalize avoids
        # numerical issues from acos near 1.
        out = q0 + t * (q1 - q0)
    else:
        theta_0 = np.arccos(np.clip(dot, -1.0, 1.0))
        sin_theta_0 = np.sin(theta_0)
        s0 = np.sin((1.0 - t) * theta_0) / sin_theta_0
        s1 = np.sin(t * theta_0) / sin_theta_0
        out = s0 * q0 + s1 * q1
    n = np.linalg.norm(out)
    return out / n if n > 0 else q0

class FrankaResearch3(Robot):
    config_class = FrankaResearch3Config
    name = "franka_research3"
    def __init__(self, config: FrankaResearch3Config):
        super().__init__(config)
        self.config = config
        self.robot = None  
        self.gripper_server_ip = config.gripper_server_ip
        self.gripper_server_port = config.gripper_server_port
        self.gripper_url = f"http://{self.gripper_server_ip}:{self.gripper_server_port}"

        self._gripper: Any | None = None
        if config.use_gripper:
            if config.gripper is None:
                raise ValueError("Serial gripper is enabled but config.gripper is missing.")
            self._gripper = _get_serial_gripper_class()(config.gripper)
        
        self._gripper_key = "gripper.pos"

        # Initialize keys and buffers based on control mode
        if config.control_mode == ControlMode.JOINT_IMPEDANCE:
            self._init_joint_mode()
        elif config.control_mode in (
            ControlMode.CARTESIAN_IMPEDANCE,
            ControlMode.CARTESIAN_IMPEDANCE_TORQUE,
        ):
            self._init_cartesian_mode()
        else:
            raise ValueError(f"Unsupported control_mode: {config.control_mode}")

        # Long-lived torque-mode impedance controller, lazily created in
        # _start_impedance_tracker() once the robot is connected. Only used
        # when control_mode == CARTESIAN_IMPEDANCE_TORQUE.
        self._impedance_tracker: CartesianImpedanceTracker | None = None
        # First-order low-pass state for the impedance target. Reset whenever
        # the tracker (re)starts so the filter doesn't carry stale state
        # across a home/reset.
        self._impedance_filtered_pos: np.ndarray | None = None
        self._impedance_filtered_quat_xyzw: np.ndarray | None = None

        self.cameras = make_cameras_from_configs(config.cameras)

        self._is_connected = False
        self._robot_connected = False
        self._gripper_connected = False
        
        logger.info(f"Initialized {self.name}")
        logger.info(f"  Robot: Franka Follower at {config.fci_ip}")
        if config.use_gripper:
            logger.info(
                f"  Gripper: Xense Hand via serial ({config.gripper_sn or config.gripper_port})"
            )
        else:
            logger.info("  Gripper: disabled")
        logger.info(f"  Cameras: {len(self.cameras)} camera(s)")
        logger.info(f"  Synchronize Actions: {config.synchronize_actions}")

    def _init_joint_mode(self) -> None:
        """Initialize keys and buffers for JOINT_POSITION control mode."""
        # Joint state observation keys: joint_{1-7}.{pos, vel, effort}
        self._joint_pos_keys = tuple(f"joint_{i}.pos" for i in range(1, JOINT_DOF + 1))
        self._joint_vel_keys = tuple(f"joint_{i}.vel" for i in range(1, JOINT_DOF + 1))
        self._joint_effort_keys = tuple(f"joint_{i}.effort" for i in range(1, JOINT_DOF + 1))

        # Joint action keys: joint_{1-7}.pos
        self._action_joint_keys = tuple(f"joint_{i}.pos" for i in range(1, JOINT_DOF + 1))

        # Pre-cache config values as lists (for API calls)
        # self._max_vel = self.config.joint_max_vel  # Already a list
        # self._max_acc = self.config.joint_max_acc  # Already a list

    def _init_cartesian_mode(self) -> None:
        """Initialize keys and buffers for CARTESIAN_POSITION control mode.

        Uses 6D rotation representation (r1-r6) instead of quaternion for:
        - Continuity: No discontinuities like Euler angles (gimbal lock)
        - No double-cover: Unlike quaternions where q and -q represent same rotation
        - Better for neural networks: Continuous representation is easier to learn

        Reference: "On the Continuity of Rotation Representations in Neural Networks"
        """
        # TCP pose observation/action keys: tcp.{x, y, z, r1, r2, r3, r4, r5, r6}
        # 6D rotation: r1-r3 = first column, r4-r6 = second column of rotation matrix
        self._tcp_pose_keys = (
            "tcp.x",
            "tcp.y",
            "tcp.z",
            "tcp.r1",
            "tcp.r2",
            "tcp.r3",
            "tcp.r4",
            "tcp.r5",
            "tcp.r6",
        )

        # TCP velocity observation keys: tcp.{vx, vy, vz, wx, wy, wz}
        self._tcp_vel_keys = (
            "tcp.vx",
            "tcp.vy",
            "tcp.vz",
            "tcp.wx",
            "tcp.wy",
            "tcp.wz",
        )

        # TCP pose action keys (same as observation keys for 6D rotation)
        self._action_tcp_pose_keys = self._tcp_pose_keys

        # Pre-cache max contact wrench (always needed in Cartesian mode for safety)
        # self._max_contact_wrench = self.config.max_contact_wrench

        # Initialize force-related keys if use_force is enabled
        if self.config.use_force:
            # Wrench keys: tcp.{fx, fy, fz, mx, my, mz}
            # Used for both observation (external wrench) and action (target wrench)
            self._wrench_keys = (
                "tcp.fx",
                "tcp.fy",
                "tcp.fz",
                "tcp.mx",
                "tcp.my",
                "tcp.mz",
            )
            # Action wrench keys are the same as observation wrench keys
            self._action_wrench_keys = self._wrench_keys

            # Pre-cache force control axis
            # self._force_control_axis = tuple(self.config.force_control_axis)

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        features = {}

        if self.config.control_mode == ControlMode.JOINT_IMPEDANCE:
            # Robot: 7 joint positions
            features.update(dict.fromkeys(self._joint_pos_keys, float))
            # Joint velocities (7 joints)
            features.update(dict.fromkeys(self._joint_vel_keys, float))
            # Joint efforts/torques (7D)
            features.update(dict.fromkeys(self._joint_effort_keys, float))
        elif self.config.control_mode in (
            ControlMode.CARTESIAN_IMPEDANCE,
            ControlMode.CARTESIAN_IMPEDANCE_TORQUE,
        ):
            # TCP pose (9D: xyz + 6D rotation)
            features.update(dict.fromkeys(self._action_tcp_pose_keys, float))
            if self.config.use_force:
                # + target wrench (6D)
                features.update(dict.fromkeys(self._action_wrench_keys, float))
        else:
            raise ValueError(f"Unsupported control mode: {self.config.control_mode}")
        
        # Gripper: position (0.0=closed, 1.0=open)
        if self.config.use_gripper:
            features[self._gripper_key] = float
        
        # Cameras
        for cam_name, cam_config in self.config.cameras.items():
            features[cam_name] = (cam_config.height, cam_config.width, 3)
        
        return features
    
    @cached_property
    def action_features(self) -> dict[str, type]:
        action_dict = {}

        if self.config.control_mode == ControlMode.JOINT_IMPEDANCE:
            # 7 joint position commands
            action_dict.update(dict.fromkeys(self._action_joint_keys, float))
        elif self.config.control_mode in (
            ControlMode.CARTESIAN_IMPEDANCE,
            ControlMode.CARTESIAN_IMPEDANCE_TORQUE,
        ):
            # Cartesian position commands (x, y, z, r1, r2, r3, r4, r5, r6)
            action_dict.update(dict.fromkeys(self._action_tcp_pose_keys, float))
            if self.config.use_force:
                # + target wrench (6D)
                action_dict.update(dict.fromkeys(self._action_wrench_keys, float))
        else:
            raise ValueError(f"Unsupported control mode: {self.config.control_mode}")
        
        # Gripper position
        if self.config.use_gripper:
            action_dict[self._gripper_key] = float
        return action_dict
    
    # ======================== Robot ========================
    def _get_robot_state(self) -> Optional[dict]:
        """Get full robot state from franky"""
        if not self.is_connected:
            return None
        try:
            state = self.robot.state
            positions = np.array(state.q, dtype=np.float32) # joint position (7D)
            velocities = np.array(state.dq, dtype=np.float32) # joint velocity (7D)
            ee_pose_matrix = np.array(state.O_T_EE.matrix, dtype=np.float32) # end-effector pose (4x4)
            ee_pose = ee_pose_matrix.flatten() # Flattened end-effector pose (16D)
            joint_torques = state.tau_J  # joint torque (7D)
            filtered_torques = state.tau_ext_hat_filtered  # Filtered joint torque (7D)
            ee_force_base = state.O_F_ext_hat_K # end-effector force in base frame (6D)
            ee_force_ee = state.K_F_ext_hat_K # end-effector force in end-effector frame (6D)
            return {
                "joint_positions": positions,
                "joint_velocities": velocities,
                "ee_pose": ee_pose,
                "joint_torques": joint_torques,
                "filtered_torques": filtered_torques,
                "ee_force_base": ee_force_base,
                "ee_force_ee": ee_force_ee
            }
        except Exception as e:
            logger.error(f"Failed to read robot state: {e}")
            return None

    # ======================== Gripper ========================
    def _gripper_health_check(self) -> bool:
        """Check whether the configured serial gripper object exists."""
        if not self.config.use_gripper:
            return True
        return self._gripper is not None
        
    def _get_gripper_position(self) -> float:
        if not self.config.use_gripper:
            return self.config.gripper_home_position
        try:
            if self._gripper is None:
                return self.config.gripper_home_position
            return float(
                np.clip(
                    self._gripper.get_gripper_position(),
                    self.config.gripper_min_position,
                    self.config.gripper_max_position,
                )
            )
        except Exception as exc:
            logger.error(f"Failed to get serial gripper position: {exc}")
            return self.config.gripper_home_position
    
    def _send_gripper_position_command(self, position: float) -> bool:
        """Send normalized target position [0.0, 1.0], where 0.0=closed and 1.0=open."""
        if not self.config.use_gripper:
            return True
        try:
            if self._gripper is None:
                return False
            target = float(
                np.clip(
                    position,
                    self.config.gripper_min_position,
                    self.config.gripper_max_position,
                )
            )
            self._gripper.set_gripper_position(target)
            return True
        except Exception as exc:
            logger.error(f"Failed to send serial gripper position command: {exc}")
            return False
        
    # ======================== Connection Management ========================
    @property
    def is_connected(self) -> bool:
        """Check if both robot and gripper are connected."""
        if self.config.use_gripper:
            return self._is_connected and self._robot_connected and self._gripper_connected
        else:
            return self._is_connected and self._robot_connected

    @property
    def is_calibrated(self) -> bool:
        """Check if robot is calibrated."""
        return self.is_connected  # Franka gripper doesn't require calibration

    def calibrate(self) -> None:
        """Calibrate robot (gripper doesn't require calibration)."""
        pass

    def configure(self) -> None:
        """Configure robot and gripper."""
        if not FrankaResearch3.is_connected.fget(self):
            raise DeviceNotConnectedError(f"{self} is not connected")
        logger.debug(f"{self} configured")

        if self.config.control_mode == ControlMode.JOINT_IMPEDANCE:
            logger.info("Setting robot to JOINT_IMPEDANCE mode")
            # Additional configuration can be added here if needed
            # TODO
        elif self.config.control_mode == ControlMode.CARTESIAN_IMPEDANCE:
            logger.info("Setting robot to CARTESIAN_IMPEDANCE mode")
            # Additional configuration can be added here if needed
            # TODO
        elif self.config.control_mode == ControlMode.CARTESIAN_IMPEDANCE_TORQUE:
            logger.info(
                "Setting robot to CARTESIAN_IMPEDANCE_TORQUE mode "
                f"(K_t={self.config.cartesian_translational_stiffness}, "
                f"K_r={self.config.cartesian_rotational_stiffness}, "
                f"K_n={self.config.cartesian_nullspace_stiffness}, "
                f"max_dtau={self.config.cartesian_max_delta_tau})"
            )
        else:
            raise ValueError(f"Unsupported control_mode: {self.config.control_mode}")

    def _apply_absolute_dynamics_limits(self) -> None:
        """Apply optional absolute dynamics limits from config to Franky robot."""
        if self.robot is None:
            return

        scalar_limits = (
            ("translation_velocity_limit", self.config.translation_velocity_limit),
            ("rotation_velocity_limit", self.config.rotation_velocity_limit),
            ("elbow_velocity_limit", self.config.elbow_velocity_limit),
            ("translation_acceleration_limit", self.config.translation_acceleration_limit),
            ("rotation_acceleration_limit", self.config.rotation_acceleration_limit),
            ("elbow_acceleration_limit", self.config.elbow_acceleration_limit),
            ("translation_jerk_limit", self.config.translation_jerk_limit),
            ("rotation_jerk_limit", self.config.rotation_jerk_limit),
            ("elbow_jerk_limit", self.config.elbow_jerk_limit),
        )
        vector_limits = (
            ("joint_velocity_limit", self.config.joint_velocity_limit),
            ("joint_acceleration_limit", self.config.joint_acceleration_limit),
            ("joint_jerk_limit", self.config.joint_jerk_limit),
        )

        for attr_name, value in scalar_limits:
            if value is None:
                continue
            getattr(self.robot, attr_name).set(float(value))
            logger.info("Applied Franky absolute limit: %s=%.6f", attr_name, value)

        for attr_name, value in vector_limits:
            if value is None:
                continue
            float_values = [float(v) for v in value]
            getattr(self.robot, attr_name).set(float_values)
            logger.info("Applied Franky absolute limit: %s=%s", attr_name, float_values)

    def connect(self, calibrate: bool = True, go_to_start: bool = True) -> None:
        """Connect to franka robot and gripper."""
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")
        
        try:
            # Connect to robot via FCI
            logger.info(f"Connecting to Franka robot at {self.config.fci_ip}")
            self.robot = FrankyRobot(self.config.fci_ip)
            self.robot.relative_dynamics_factor = RelativeDynamicsFactor(
                self.config.velocity,
                self.config.acceleration,
                self.config.jerk,
            )
            logger.info(
                "Set Franky relative dynamics factor: "
                f"velocity={self.config.velocity:.3f}, "
                f"acceleration={self.config.acceleration:.3f}, "
                f"jerk={self.config.jerk:.3f}"
            )
            self._apply_absolute_dynamics_limits()
            self._robot_connected = True

            if self.config.use_gripper:
                if self._gripper is None:
                    raise ConnectionError("Serial gripper is not initialized")
                logger.info("Connecting to serial gripper...")
                self._gripper.connect()
                self._gripper_connected = True
                logger.info("Gripper connection successful")
            
            # Connect cameras
            logger.info("Connecting cameras...")
            for cam in self.cameras.values():
                cam.connect()
            
            self._is_connected = True
            logger.info(f"✅ {self} connected successfully")
            
            # Move to start position if requested (use parameter if provided, otherwise use config)
            self.config.go_to_start = go_to_start if go_to_start is not None else self.config.go_to_start
            if self.config.go_to_start:
                self._go_to_start()

            # Switch to the configured control mode
            # TODO
            self._switch_to_control_mode()

            # Configure control parameters
            self.configure()

        except Exception as e:
            # Cleanup on partial failure
            try:
                if self._robot_connected and self.robot is not None:
                    self.robot.stop()
            except Exception:
                pass
            try:
                if self._gripper is not None and self._gripper_connected:
                    self._gripper.disconnect()
            except Exception:
                pass
            try:
                for cam in self.cameras.values():
                    try:
                        if cam.is_connected:
                            cam.disconnect()
                    except:
                        pass
            except:
                pass
            
            self._robot_connected = False
            self._gripper_connected = False
            self._is_connected = False
            logger.error(f"Failed to connect: {e}")
            raise

    def _go_to_home(self) -> None:
        """Move robot to home position using MoveJ primitive.

        Uses ExecutePrimitive("MoveJ") to move to factory-defined home pose:
        - target: [0.0, -40.0, 0.0, 90.0, 0.0, 40.0, 0.0] degrees
        - jntVelScale: 20
        """
        if not self._is_connected or self.robot is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        logger.info("Moving to home position...")

        # Factory-defined home position in degrees
        home_position_deg = self.config.robot_home_position
        try:
            motion = JointMotion(home_position_deg, ReferenceType.Absolute)
            self.robot.move(motion, asynchronous=False)
            time.sleep(0.1)  # Wait for motion to complete
        except Exception as e:
            logger.warning(f"Error sending robot action: {e}")
            try:
                if getattr(self.robot, "has_errors", False):
                    self.robot.recover_from_errors()
                    logger.info("🚨 Robot recovered from Reflex mode")
            except Exception as rec_err:
                logger.warning(f"recover_from_errors() failed in _go_to_home: {rec_err}")

    def _go_to_start(self) -> None:
        """Move robot to home position using MoveJ primitive.

        Uses ExecutePrimitive("MoveJ") to move to factory-defined home pose:
        - target: [0.0, -40.0, 0.0, 90.0, 0.0, 40.0, 0.0] degrees
        - jntVelScale: 20
        """
        if not self._is_connected or self.robot is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        logger.info("Moving to home position...")

        # Factory-defined home position in degrees
        home_position_deg = self.config.robot_home_position
        try:
            motion = JointMotion(home_position_deg, ReferenceType.Absolute)
            self.robot.move(motion, asynchronous=False)
            time.sleep(0.1)  # Wait for motion to complete
        except Exception as e:
            logger.warning(f"Error sending robot action: {e}")
            self.robot.recover_from_errors()
            logger.info("🚨 Robot recovered from Reflex mode")

    def reset_to_initial_position(self) -> None:
        """Reset robot to initial position based on config.go_to_start.

        If config.go_to_start=True, calls _go_to_start().
        Otherwise, calls _go_to_home().
        """
        if not self._is_connected or self.robot is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Stop the impedance controller before issuing JointMotion: franky
        # refuses a different motion type while one is still running.
        self._stop_impedance_tracker()

        if self.config.go_to_start:
            logger.info("Resetting to start position (config.go_to_start=True)")
            self._go_to_start()
        else:
            logger.info("Resetting to home position (config.go_to_start=False)")
            self._go_to_home()

        # Switch back to control mode after reset
        self._switch_to_control_mode()

    def _switch_to_control_mode(self) -> None:
        """Start mode-specific long-lived controllers.

        For CARTESIAN_IMPEDANCE_TORQUE this starts (or restarts) the franky
        CartesianImpedanceTracker so subsequent send_action() calls just push
        a new reference into its handle. Other modes don't need anything here
        — they re-issue motions per send_action() call.
        """
        if self.config.control_mode == ControlMode.CARTESIAN_IMPEDANCE_TORQUE:
            self._start_impedance_tracker()

    def _start_impedance_tracker(self) -> None:
        """Spin up the Cartesian impedance tracking controller."""
        if self.robot is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        # If a previous tracker is still around (e.g. after a reset), tear it
        # down before starting a new one so franky doesn't see two motions.
        self._stop_impedance_tracker()
        # Reset filter state — the tracker re-seeds from the current pose,
        # so the next send_action() should treat its first target as fresh.
        self._impedance_filtered_pos = None
        self._impedance_filtered_quat_xyzw = None

        nullspace_target = None
        if self.config.cartesian_use_home_as_nullspace_target:
            nullspace_target = np.asarray(
                self.config.robot_home_position, dtype=np.float64
            )

        self._impedance_tracker = CartesianImpedanceTracker(
            self.robot,
            translational_stiffness=self.config.cartesian_translational_stiffness,
            rotational_stiffness=self.config.cartesian_rotational_stiffness,
            nullspace_target=nullspace_target,
            nullspace_stiffness=self.config.cartesian_nullspace_stiffness,
            max_delta_tau=self.config.cartesian_max_delta_tau,
            gains_time_constant=self.config.cartesian_gains_time_constant,
        )
        logger.info(
            "Started CartesianImpedanceTracker "
            f"(K_t={self.config.cartesian_translational_stiffness}, "
            f"K_r={self.config.cartesian_rotational_stiffness}, "
            f"K_n={self.config.cartesian_nullspace_stiffness})"
        )

    def _stop_impedance_tracker(self) -> None:
        """Stop the Cartesian impedance tracker if it's running.

        If the tracker is already dead (e.g. after a Reflex), skip the
        graceful stop() — franky's internal join_motion() will get rejected
        by libfranka in Reflex mode and just spam warnings. The async motion
        thread has already unwound; we just need to drop the reference.
        """
        if self._impedance_tracker is None:
            return
        # Only attempt graceful stop when controller is still alive.
        if getattr(self._impedance_tracker, "is_running", False):
            try:
                self._impedance_tracker.stop()
            except Exception as e:
                logger.warning(f"CartesianImpedanceTracker.stop() failed: {e}")
        self._impedance_tracker = None

    # ======================== Observation ========================
    
    def get_observation(self) -> dict[str, Any]:
        """Get synchronized observation from robot, gripper, and cameras."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")
        
        obs_dict = {}

        state = self.robot.state
        # positions = np.array(state.q, dtype=np.float32) # joint position (7D)
        # velocities = np.array(state.dq, dtype=np.float32) # joint velocity (7D)
        # ee_pose_matrix = np.array(state.O_T_EE.matrix, dtype=np.float32) # end-effector pose (4x4)
        # ee_pose = ee_pose_matrix.flatten() # Flattened end-effector pose (16D)
        # joint_torques = state.tau_J  # joint torque (7D)
        # filtered_torques = state.tau_ext_hat_filtered  # Filtered joint torque (7D)
        # ee_force_base = state.O_F_ext_hat_K # end-effector force in base frame (6D)
        # ee_force_ee = state.K_F_ext_hat_K # end-effector force in end-effector frame (6D)

        if self.config.control_mode == ControlMode.JOINT_IMPEDANCE:
            # Joint positions (7D)
            for i, key in enumerate(self._joint_pos_keys):
                obs_dict[key] = state.q[i]
            # Joint velocities (7D)
            for i, key in enumerate(self._joint_vel_keys):
                obs_dict[key] = state.dq[i]
            # Joint efforts/torques (7D)
            for i, key in enumerate(self._joint_effort_keys):
                obs_dict[key] = state.tau_J[i]

        elif self.config.control_mode in (
            ControlMode.CARTESIAN_IMPEDANCE,
            ControlMode.CARTESIAN_IMPEDANCE_TORQUE,
        ):
            # TCP pose from SDK: [r1 r4 r7 px; r2 r5 r8 py; r3 r6 r9 pz; 0 0 0 1]
            tcp_pose = np.array(state.O_T_EE.matrix, dtype=np.float32).flatten()

            # Position (3D)
            obs_dict["tcp.x"] = tcp_pose[3]
            obs_dict["tcp.y"] = tcp_pose[7]
            obs_dict["tcp.z"] = tcp_pose[11]
            # 6D Rotation (r1-r6)
            obs_dict["tcp.r1"] = tcp_pose[0]
            obs_dict["tcp.r2"] = tcp_pose[4]
            obs_dict["tcp.r3"] = tcp_pose[8]
            obs_dict["tcp.r4"] = tcp_pose[1]
            obs_dict["tcp.r5"] = tcp_pose[5]
            obs_dict["tcp.r6"] = tcp_pose[9]

            if self.config.use_force:
                # + external wrench (6D)
                ext_wrench = state.O_F_ext_hat_K
                for i, key in enumerate(self._wrench_keys):
                    obs_dict[key] = ext_wrench[i]

        else:
            raise ValueError(f"Unsupported control_mode: {self.config.control_mode}")
        
        if self.config.use_gripper:
            # Get gripper position via serial driver
            gripper_position = self._get_gripper_position()
            obs_dict[self._gripper_key] = gripper_position
        
        # Get camera observations
        for cam_name, cam in self.cameras.items():
            obs_dict[cam_name] = cam.async_read()

        return obs_dict

    def get_current_tcp_pose_euler(self) -> np.ndarray:
        """Get current TCP 4*4 pose in Euler angles format [x, y, z, roll, pitch, yaw, gripper_pos].

        This method can be used for getting the current TCP pose in Euler angles format for initializing teleoperators (e.g., 6-DoF controller) with the robot's
        current TCP pose. Only available in CARTESIAN_IMPEDANCE mode.

        Returns:
            numpy array of shape (7,) with [x, y, z, roll, pitch, yaw, gripper_pos]
        """
        if not self.is_connected or self.robot is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if self.config.control_mode not in (
            ControlMode.CARTESIAN_IMPEDANCE,
            ControlMode.CARTESIAN_IMPEDANCE_TORQUE,
        ):
            raise ValueError(
                "get_current_tcp_pose_euler requires a Cartesian control mode "
                "(CARTESIAN_IMPEDANCE or CARTESIAN_IMPEDANCE_TORQUE)"
            )

        state = self.robot.state
        ee_pose_matrix = np.array(state.O_T_EE.matrix, dtype=np.float32)
        tcp_pose = matrix_to_pose7d(ee_pose_matrix, output_format="wxyz")

        # Convert quaternion to Euler angles
        euler = quaternion_to_euler(tcp_pose[3], tcp_pose[4], tcp_pose[5], tcp_pose[6])
        roll, pitch, yaw = euler[0], euler[1], euler[2]

        # Get gripper position directly from the serial gripper
        gripper_pos = 0.0
        if self.config.use_gripper:
            gripper_pos = self._get_gripper_position()

        # Return [x, y, z, roll, pitch, yaw, gripper_pos]
        return np.array(
            [tcp_pose[0], tcp_pose[1], tcp_pose[2], roll, pitch, yaw, gripper_pos],
            dtype=np.float32,
        )

    def get_current_tcp_pose_quat(self) -> np.ndarray:
        """Get current TCP 4*4 pose in quaternion format [x, y, z, qw, qx, qy, qz, gripper_pos].

        This method can be used for getting the current TCP pose in quaternion format for initializing teleoperators (e.g., pico4) with the robot's
        current TCP pose. Only available in CARTESIAN_IMPEDANCE mode.

        Returns:
            numpy array of shape (8,) with [x, y, z, qw, qx, qy, qz, gripper_pos]
        """
        if not self.is_connected or self.robot is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if self.config.control_mode not in (
            ControlMode.CARTESIAN_IMPEDANCE,
            ControlMode.CARTESIAN_IMPEDANCE_TORQUE,
        ):
            raise ValueError(
                "get_current_tcp_pose_quat requires a Cartesian control mode "
                "(CARTESIAN_IMPEDANCE or CARTESIAN_IMPEDANCE_TORQUE)"
            )

        state = self.robot.state
        ee_pose_matrix = np.array(state.O_T_EE.matrix, dtype=np.float32)
        tcp_pose = matrix_to_pose7d(ee_pose_matrix, output_format="wxyz")

        # Get gripper position directly from the serial gripper
        gripper_pos = 0.0
        if self.config.use_gripper:
            gripper_pos = self._get_gripper_position()

        # Return [x, y, z, qw, qx, qy, qz, gripper_pos]
        return np.array(
            [*tcp_pose, gripper_pos],
            dtype=np.float32,
        )

    # ======================== Action ========================
    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Send synchronized action to robot and gripper.
        
        Args:
            action: Dictionary containing:
                - delta_x, delta_y, delta_z: Cartesian velocity commands (m/s)
                - gripper: Gripper position (0.0=closed, 1.0=open)
        
        Returns:
            The action that was sent
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")
        

        # Send robot arm action
        if self.config.control_mode == ControlMode.JOINT_IMPEDANCE:
            result = self._send_joint_position_action(action)

        elif self.config.control_mode == ControlMode.CARTESIAN_IMPEDANCE:
            if self.config.use_force:
                result = self._send_cartesian_motion_force_action(action)
            else:
                result = self._send_cartesian_pure_motion_action(action)

        elif self.config.control_mode == ControlMode.CARTESIAN_IMPEDANCE_TORQUE:
            result = self._send_cartesian_impedance_torque_action(action)

        else:
            raise ValueError(f"Unsupported control_mode: {self.config.control_mode}")

        # Send gripper action
        self._send_gripper_action(action)

        return result if result is not None else action

    def _send_joint_position_action(self, action: dict[str, Any]) -> dict[str, Any]:
        # target_pos = []
        # for i in range(7):
        #     # Get velocity command (rad/s) from action dict
        #     Joint_pos = float(action.get(f'joint_{i}.pos', 0.0))
        #     # print("velocity",Joint_pos)
        #     target_pos.append(Joint_pos)
        try:
            joint_pos = [action[key] for key in self._action_joint_keys]
            joint_pos = np.array(joint_pos, dtype=np.float32)
            motion = JointMotion(joint_pos.tolist(), ReferenceType.Absolute)
            self.robot.move(motion, asynchronous=True)
        except Exception as e:
            logger.warning(f"Error sending robot action: {e}")
            self.robot.recover_from_errors()
            logger.info("🚨 Robot recovered from Reflex mode")
        return action
    
    def _send_cartesian_pure_motion_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Send Cartesian pure motion command (NRT mode, no force control).

        Action keys: action.tcp.{x,y,z,r1,r2,r3,r4,r5,r6}

        The action uses 6D rotation representation which is converted to an
        internal [qw, qx, qy, qz] quaternion, then reordered for Franky's
        Affine constructor ([qx, qy, qz, qw], same as scipy Rotation.as_quat()).
        """
        try:
            # Extract position
            x, y, z = action["tcp.x"], action["tcp.y"], action["tcp.z"]
            # Extract 6D rotation and convert to quaternion
            r6d = np.array(
                [
                    action["tcp.r1"],
                    action["tcp.r2"],
                    action["tcp.r3"],
                    action["tcp.r4"],
                    action["tcp.r5"],
                    action["tcp.r6"],
                ]
            )
            quat_wxyz = rotation_6d_to_quaternion(r6d)  # Returns [qw, qx, qy, qz]
            quat_xyzw = _quat_wxyz_to_xyzw(quat_wxyz)
            logger.debug(
                "Sending Franky Cartesian target: "
                f"pos=({x:.6f}, {y:.6f}, {z:.6f}), "
                f"quat_wxyz={quat_wxyz}, quat_xyzw={quat_xyzw}"
            )
            motion = CartesianMotion(Affine([x, y, z], quat_xyzw))
            self.robot.move(motion, asynchronous=True)
        except Exception as e:
            logger.warning(f"Error sending robot action: {e}")
            self.robot.recover_from_errors()
            logger.info("🚨 Robot recovered from Reflex mode")
        return action

    def _send_cartesian_motion_force_action(self, action: dict[str, Any]) -> dict[str, Any]:
        return action

    def _send_cartesian_impedance_torque_action(
        self, action: dict[str, Any]
    ) -> dict[str, Any]:
        """Push a new target into the Cartesian impedance tracker.

        Same action contract as `_send_cartesian_pure_motion_action`
        (tcp.{x,y,z,r1..r6} with 6D rotation), but the target is delivered to
        franky's CartesianImpedanceTrackingMotion via its reference handle
        instead of starting a fresh CartesianMotion each cycle. The torque-mode
        impedance controller stays alive throughout, so commands are smooth at
        the teleop rate.
        """
        if self._impedance_tracker is None:
            raise DeviceNotConnectedError(
                f"{self} is not in CARTESIAN_IMPEDANCE_TORQUE mode or the "
                "impedance tracker has not been started."
            )

        # Reflex/collision detection runs inside franky's async motion thread,
        # so set_target() against a dead controller would fail silently. If
        # the tracker has stopped (because the user pushed the arm past the
        # collision limits), clear the error and restart so teleop continues.
        if not self._impedance_tracker.is_running:
            logger.warning(
                "Impedance controller is no longer running (likely Reflex / "
                "collision). Recovering and restarting tracker..."
            )
            # Drop the dead tracker reference WITHOUT calling its stop() —
            # in Reflex mode libfranka rejects the stop motion and just
            # raises ControlException. The async thread has already unwound.
            self._impedance_tracker = None
            self._impedance_filtered_pos = None
            self._impedance_filtered_quat_xyzw = None

            # Always call recover — has_errors can be False momentarily even
            # when the controller has aborted; recover_from_errors() is
            # idempotent and safe when there's nothing to clear.
            try:
                self.robot.recover_from_errors()
                logger.warning("🚨 Robot recovered (impedance reflex cleared)")
            except Exception as rec_err:
                logger.error(
                    f"recover_from_errors() failed after impedance abort: {rec_err}"
                )
                return None

            # Brief pause for libfranka to settle out of Reflex mode before
            # we open a new control loop. Without this, the new motion can
            # be rejected with "command not possible in the current mode".
            time.sleep(0.05)

            try:
                self._start_impedance_tracker()
            except Exception as restart_err:
                logger.error(
                    f"Failed to restart impedance tracker after recovery: {restart_err}"
                )
                return None

            # IMPORTANT: skip pushing this cycle's pico4 target. The arm is
            # currently wherever the user pushed it, and pico4's last target
            # may be far from there — pushing it now would yank the arm and
            # often re-trigger Reflex. The freshly-started tracker holds at
            # the current pose; the user should release the grip and the
            # teleop loop will re-sync on the next press.
            return None

        try:
            x, y, z = action["tcp.x"], action["tcp.y"], action["tcp.z"]
            r6d = np.array(
                [
                    action["tcp.r1"],
                    action["tcp.r2"],
                    action["tcp.r3"],
                    action["tcp.r4"],
                    action["tcp.r5"],
                    action["tcp.r6"],
                ]
            )
            quat_wxyz = rotation_6d_to_quaternion(r6d)
            quat_xyzw = _quat_wxyz_to_xyzw(quat_wxyz)

            # First-order low-pass on the streamed target. With pico4 at
            # ~30Hz and a 1kHz impedance loop, an unfiltered target is a
            # staircase that the controller chases every 33ms; the EMA
            # below softens those steps and removes most of the EE buzz.
            alpha = float(np.clip(self.config.cartesian_target_filter_alpha, 0.0, 1.0))
            target_pos = np.array([x, y, z], dtype=np.float64)
            if 0.0 < alpha < 1.0 and self._impedance_filtered_pos is not None:
                self._impedance_filtered_pos = (
                    alpha * target_pos + (1.0 - alpha) * self._impedance_filtered_pos
                )
                self._impedance_filtered_quat_xyzw = _slerp_quat_xyzw(
                    self._impedance_filtered_quat_xyzw, quat_xyzw, alpha
                )
            else:
                # alpha=1.0 (filter disabled) or first sample after restart
                # — pass the raw target through and seed the filter state.
                self._impedance_filtered_pos = target_pos
                self._impedance_filtered_quat_xyzw = quat_xyzw

            self._impedance_tracker.set_target(
                Affine(
                    self._impedance_filtered_pos.tolist(),
                    self._impedance_filtered_quat_xyzw,
                )
            )
            return action
        except Exception as e:
            logger.warning(f"Error pushing impedance target: {e}")
            try:
                if getattr(self.robot, "has_errors", False):
                    self.robot.recover_from_errors()
                    logger.info("🚨 Robot recovered from Reflex mode")
            except Exception as rec_err:
                logger.warning(
                    f"recover_from_errors() failed in impedance dispatch: {rec_err}"
                )
            return None

    def _send_gripper_action(self, action: dict[str, Any]) -> None:
        if not self.config.use_gripper or self._gripper_key not in action:
            return

        try:
            target_gripper_position = float(
                np.clip(
                    float(action[self._gripper_key]),
                    self.config.gripper_min_position,
                    self.config.gripper_max_position,
                )
            )
            if not self._send_gripper_position_command(target_gripper_position):
                logger.warning("Failed to send action to gripper")
        except Exception as e:
            logger.warning(f"Error sending gripper action: {e}")


    # ======================== Reset & Recovery ========================
    def disconnect(self) -> None:
        """Disconnect from both robot and gripper."""
        if not FrankaResearch3.is_connected.fget(self):
            raise DeviceNotConnectedError(f"{self} is not connected")
        robot_success = True
        gripper_success = True
        # Stop robot motion. The previous teleop loop issues async motions
        # (e.g. CartesianMotion); franky refuses a different motion type while
        # one is still running ("The type of motion cannot change during
        # runtime"), and tearing down the FrankyRobot while its internal
        # control thread is still joinable causes a C++ "terminate called
        # without an active exception" crash. So: stop -> join -> recover ->
        # go_to_home -> stop again, each guarded.
        try:
            # Tear down the long-lived impedance tracker (if any) before any
            # other motion is issued, otherwise the JointMotion in _go_to_home
            # collides with the still-running torque controller.
            self._stop_impedance_tracker()

            if self.robot is not None:
                try:
                    self.robot.stop()
                except Exception as e:
                    logger.warning(f"robot.stop() before home failed: {e}")
                try:
                    self.robot.join_motion()
                except Exception as e:
                    logger.warning(f"robot.join_motion() before home failed: {e}")
                try:
                    if getattr(self.robot, "has_errors", False):
                        self.robot.recover_from_errors()
                except Exception as e:
                    logger.warning(f"recover_from_errors() before home failed: {e}")

                try:
                    self._go_to_home()
                except Exception as e:
                    logger.warning(f"Failed to move to home before disconnect: {e}")

                try:
                    self.robot.join_motion()
                except Exception as e:
                    logger.warning(f"robot.join_motion() after home failed: {e}")
                try:
                    self.robot.stop()
                except Exception as e:
                    logger.warning(f"robot.stop() after home failed: {e}")
        except Exception as e:
            logger.warning(f"Error stopping robot before disconnect: {e}")
            robot_success = False

        # Disconnect gripper
        try:
            if self._gripper is not None and self._gripper_connected:
                self._gripper.disconnect()
        except Exception as e:
            logger.error(f"Failed to disconnect gripper: {e}")
            gripper_success = False

        # Disconnect cameras
        try:
            for cam in self.cameras.values():
                cam.disconnect()
        except Exception as e:
            logger.error(f"Failed to disconnect cameras: {e}")
        # Drop the franky Robot reference deterministically so its destructor
        # (which joins the internal control thread) runs while we still hold
        # the GIL, instead of during interpreter shutdown.
        self.robot = None
        self._is_connected = False
        self._robot_connected = False
        self._gripper_connected = False

        if robot_success and gripper_success:
            logger.info(f"{self} disconnected successfully")
        else:
            logger.warning(f"{self} disconnected with errors")

    def recover_from_errors(self) -> bool:
        """Recover both robot and gripper from errors."""
        robot_recovered = False
        
        try:
            if self.robot is not None:
                self.robot.recover_from_errors()
            logger.info("Robot recovered from errors")
            robot_recovered = True
        except Exception as e:
            logger.error(f"Error recovering robot: {e}")
        
        return robot_recovered

    def __repr__(self) -> str:
        return (
            f"FrankaResearch3("
            f"fci_ip={self.config.fci_ip}, "
            f"gripper={self.config.gripper_sn if self.config.use_gripper else 'N/A'}, "
            f"connected={self.is_connected})"
        )
