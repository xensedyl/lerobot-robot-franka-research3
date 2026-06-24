from dataclasses import dataclass, field
from typing import Dict
from enum import Enum

from lerobot.cameras.utils import CameraConfig
from lerobot.cameras.realsense import RealSenseCameraConfig
from lerobot.robots.config import RobotConfig
from lerobot.cameras.configs import ColorMode
from .config_serial_gripper import SerialGripperConfig
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.cameras.xense import XenseOutputType, XenseTactileCameraConfig

class ControlMode(str, Enum):
    """Control mode for Flexiv Rizon4.

    JOINT_IMPEDANCE:
        Joint impedance control (maps to NRT_JOINT_IMPEDANCE).
        Uses impedance control with configurable stiffness via stiffness_ratio.
        - Action: joint positions (7D) + gripper (1D) = 8D
        - Observation: joint positions (7D) + velocities (7D) + efforts (7D) + gripper (1D) = 22D

    CARTESIAN_IMPEDANCE:
        Cartesian motion control (maps to NRT_CARTESIAN_MOTION_FORCE).
        When use_force=False: pure motion control
        When use_force=True: motion + force control
        - Action: TCP pose (7D) + gripper (1D) = 8D, or pose + wrench (13D) + gripper (1D) = 14D
        - Observation: TCP pose (7D) + gripper (1D) = 8D, or pose + wrench (13D) + gripper (1D) = 14D
    """

    JOINT_IMPEDANCE = "joint_impedance"
    CARTESIAN_IMPEDANCE = "cartesian_impedance"
    # Client-side Cartesian impedance via franky's torque mode
    # (CartesianImpedanceTrackingMotion + CartesianReferenceHandle).
    # Same TCP-pose action contract as CARTESIAN_IMPEDANCE; the difference is
    # the underlying controller, which makes it suitable for compliant teleop.
    CARTESIAN_IMPEDANCE_TORQUE = "cartesian_impedance_torque"

@RobotConfig.register_subclass("franka_research3")
@dataclass
class FrankaResearch3Config(RobotConfig):

    # ======================== Franka Follower Arm Configuration ========================
    
    # FCI (Fast Communication Interface) IP for Franka control
    fci_ip: str = "192.168.99.111"
    
    # Pico4 teleoperation sends Cartesian TCP targets, so Cartesian impedance is
    # the default for this Franky-backed FR3 driver.
    control_mode: ControlMode = ControlMode.CARTESIAN_IMPEDANCE

    # use_force: Enable force control (only applies to CARTESIAN_MOTION_FORCE mode)
    #   - False: pure motion control, action/observation = TCP pose (7D)
    #   - True: motion + force control, action/observation = pose + wrench (13D)
    use_force: bool = False

    # Franky global relative dynamics factors. These scale the robot's maximum
    # velocity, acceleration, and jerk limits. For Pico4 teleop, keep these
    # conservative because targets are updated continuously.
    velocity: float = 0.8
    acceleration: float = 0.05
    jerk: float = 0.05

    # Optional absolute dynamics limits. When any of these are set, the
    # corresponding Franky limit is applied directly via robot.*_limit.set(...)
    # so you can tune exact values instead of only relative scaling.
    translation_velocity_limit: float | None = None
    rotation_velocity_limit: float | None = None
    elbow_velocity_limit: float | None = None
    translation_acceleration_limit: float | None = None
    rotation_acceleration_limit: float | None = None
    elbow_acceleration_limit: float | None = None
    translation_jerk_limit: float | None = None
    rotation_jerk_limit: float | None = None
    elbow_jerk_limit: float | None = None
    joint_velocity_limit: list[float] | None = None
    joint_acceleration_limit: list[float] | None = None
    joint_jerk_limit: list[float] | None = None
    
    # ======================== Cartesian Impedance (torque mode) ========================
    # Used only when control_mode == CARTESIAN_IMPEDANCE_TORQUE. These map
    # directly to franky's CartesianImpedanceTracker constructor kwargs.
    # Damping is chosen internally as critically damped wrt these stiffnesses.
    #
    # Defaults are tuned for pico4 ~30Hz teleop, where the reference target
    # arrives as a 33ms step every cycle. High stiffness + high torque slew
    # makes the 1kHz controller chase each step aggressively and produces
    # audible end-effector buzz; the values below trade a little tracking
    # response for a smoother, more compliant feel. Push K_t/K_r back up
    # (e.g. 1200/80) once you switch to a smoother reference source.
    cartesian_translational_stiffness: float = 600.0   # N/m  (was 1200)
    cartesian_rotational_stiffness: float = 20.0       # Nm/rad (was 80)
    cartesian_nullspace_stiffness: float = 5.0         # Nm/rad in nullspace (was 10)
    # If True, use config.robot_home_position as the nullspace posture target.
    # When False, no nullspace target is sent (nullspace_stiffness then has no
    # effect even if non-zero).
    cartesian_use_home_as_nullspace_target: bool = True
    # Per-cycle limit on the change in commanded joint torque [Nm]. Lower
    # values smooth out reference jumps but slow tracking response. The 0.3
    # default is the single biggest jitter knob for stepped teleop input.
    cartesian_max_delta_tau: float = 0.3                # was 1.0
    # Time constant for the gains-handle smoothing in the RT loop [s].
    cartesian_gains_time_constant: float = 0.1
    # Optional first-order low-pass on the Cartesian target before it's
    # pushed into the reference handle. alpha in (0, 1]; 1.0 = no filter,
    # smaller = stronger smoothing. 0.4 takes the edge off pico4's 30Hz
    # step input without adding noticeable lag.
    cartesian_target_filter_alpha: float = 0.4

    # Connection behavior
    go_to_start: bool = (
        True  # If True, move robot to start position after connecting. If False, stay at current position.
    )

    # Home position for robot (7 joint angles in radians)
    # robot_home_position: list = field(default_factory=lambda: [-0.030264, -0.523095, -0.091621, -2.812467, -0.089465,  2.25039,   0.709976])
    # robot_home_position: list = field(default_factory=lambda: [-0.08211147, -0.6067168,  -0.03138583, -2.7927575,  -0.02443479,  2.211011, 0.67388374])
    robot_home_position: list = field(default_factory=lambda: [ -0.09385, -0.17559,  0.02542, -2.05487,  0.028,    1.88188, -0.89032 ])
    
    # ======================== Xense Gripper Configuration ========================

    # Whether to use the gripper
    use_gripper: bool = True

    # Pure-serial Xense gripper settings.
    gripper_sn: str = "000015"
    gripper_port: str = ""
    gripper_baudrate: int = 115200
    gripper_serial_timeout: float = 1.0
    gripper_device_id: int = 1
    gripper_init_open: bool = True

    enable_gripper_wrist_camera: bool = True
    gripper_wrist_camera_sn: str = "XC000015"
    enable_gripper_tactile_sensors: bool = True
    gripper_tactile_camera_sn_0: str = "OG000938"
    gripper_tactile_camera_sn_1: str = "OG000937"

    # Legacy hand server settings kept for CLI/config compatibility. This
    # Franky-backed robot uses the serial gripper driver above.
    gripper_server_ip: str = "127.0.0.1"
    gripper_server_port: int = 7001
    
    # Gripper hardware identification
    gripper_id: str = "7ec0c7f50ea6"  # USB device ID
    
    # Gripper motion parameters
    gripper_default_velocity: float = 100.0   # vel
    gripper_default_force: float = 30.0        # force
    
    # Gripper position limits (0.0=closed, 1.0=open)
    gripper_min_position: float = 0.0
    gripper_max_position: float = 1.0
    
    # Gripper home position
    gripper_home_position: float = 1.0  # fully open
    
    # Physical width mapping (mm)
    gripper_min_width_mm: float = 0.0    # fully closed
    gripper_max_width_mm: float = 85.0   # fully open
    
    # Gripper communication timeout (seconds)
    gripper_timeout: float = 2.0
    
    # ======================== Camera Configuration ========================
    
    # RealSense cameras (2 cameras recommended: main + wrist)
    cameras: Dict[str, RealSenseCameraConfig] = field(default_factory=lambda: {
        # # Main external camera
        # "image": RealSenseCameraConfig(
        #     serial_number_or_name="135522074323",
        #     fps=30,
        #     width=640,
        #     height=480,
        #     color_mode=ColorMode.RGB
        # ),
        # # Wrist-mounted camera
        # "wrist_image": RealSenseCameraConfig(
        #     serial_number_or_name="249322063436",
        #     fps=30,
        #     width=640,
        #     height=480,
        #     color_mode=ColorMode.RGB
        # )
    })
    
    # ======================== Action Synchronization ========================
    
    # Send arm and hand actions simultaneously
    synchronize_actions: bool = True
    
    # Timeout for synchronized actions (seconds)
    action_timeout: float = 0.1

    gripper: SerialGripperConfig | None = field(default=None, init=False)

    def __post_init__(self):
        for name in ("velocity", "acceleration", "jerk"):
            value = getattr(self, name)
            if not 0.0 < value <= 1.0:
                raise ValueError(
                    f"{name} must be in (0, 1] for Franky relative dynamics, got {value}."
                )

        for name in ("joint_velocity_limit", "joint_acceleration_limit", "joint_jerk_limit"):
            values = getattr(self, name)
            if values is not None:
                if len(values) != 7:
                    raise ValueError(f"{name} must contain 7 values, got {len(values)}.")
                if any(v <= 0 for v in values):
                    raise ValueError(f"{name} values must be > 0, got {values}.")

        scalar_limit_names = (
            "translation_velocity_limit",
            "rotation_velocity_limit",
            "elbow_velocity_limit",
            "translation_acceleration_limit",
            "rotation_acceleration_limit",
            "elbow_acceleration_limit",
            "translation_jerk_limit",
            "rotation_jerk_limit",
            "elbow_jerk_limit",
        )
        for name in scalar_limit_names:
            value = getattr(self, name)
            if value is not None and value <= 0:
                raise ValueError(f"{name} must be > 0 when set, got {value}.")

        if self.use_gripper:
            self.gripper = SerialGripperConfig(
                sn=self.gripper_sn or None,
                port=self.gripper_port,
                baudrate=self.gripper_baudrate,
                serial_timeout=self.gripper_serial_timeout,
                device_id=self.gripper_device_id,
                gripper_min_pos=self.gripper_min_width_mm,
                gripper_max_pos=self.gripper_max_width_mm,
                gripper_v_max=self.gripper_default_velocity,
                gripper_f_max=self.gripper_default_force,
                init_open=self.gripper_init_open,
            )
        else:
            self.gripper = None


        if self.use_gripper and self.enable_gripper_wrist_camera:
            self.cameras["wrist"] = OpenCVCameraConfig(
                index_or_path=self.gripper_wrist_camera_sn,
                fourcc="MJPG",
                width=640,
                height=480,
                fps=30,
                warmup_s=1.0,
            )

        if self.use_gripper and self.enable_gripper_tactile_sensors:
            self.cameras.update(
                {
                    "tactile_0": XenseTactileCameraConfig(
                        serial_number=self.gripper_tactile_camera_sn_0,
                        fps=30,
                        output_types=[XenseOutputType.RECTIFY],
                        warmup_s=0.05,
                    ),
                    "tactile_1": XenseTactileCameraConfig(
                        serial_number=self.gripper_tactile_camera_sn_1,
                        fps=30,
                        output_types=[XenseOutputType.RECTIFY],
                        warmup_s=0.05,
                    ),
                }
            )


        super().__post_init__()
    
