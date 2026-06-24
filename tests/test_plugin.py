from lerobot.robots.config import RobotConfig
from lerobot.robots.utils import make_robot_from_config

from lerobot_robot_franka_research3 import FrankaResearch3, FrankaResearch3Config


def test_franka_research3_config_registered():
    cfg = FrankaResearch3Config(use_gripper=False)

    assert cfg.type == "franka_research3"
    assert RobotConfig.get_choice_class("franka_research3") is FrankaResearch3Config


def test_franka_research3_can_be_instantiated_without_gripper():
    robot = make_robot_from_config(FrankaResearch3Config(use_gripper=False))

    assert isinstance(robot, FrankaResearch3)
    assert robot.name == "franka_research3"
    assert "tcp.x" in robot.action_features
    assert "gripper.pos" not in robot.action_features
    assert "tcp.x" in robot.observation_features
