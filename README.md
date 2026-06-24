# LeRobot Franka Research 3 Robot

[中文版说明](./README.zh-CN.md)

Standalone LeRobot robot plugin for Franka Research 3 using `franky` as the
low-level robot backend.

The package registers:

- `--robot.type=franka_research3`

It supports:

- Franky joint position motion
- Franky Cartesian motion
- Franky Cartesian impedance torque tracking
- optional Xense serial gripper
- optional wrist camera and Xense tactile cameras attached through LeRobot

## Dependencies

This package depends on `franky`, from:

```text
git@github.com:xensedyl/franky.git
```

The Python distribution name is `franky-control`, and the import module is
`franky`.

### Install Franky From Git

You can also install the fork directly:

```bash
conda activate xensehand
pip install "franky-control @ git+ssh://git@github.com/xensedyl/franky.git"
```

If your robot or libfranka version requires a locally patched build, prefer the
editable local checkout above.

### Xense Serial Gripper

The default config has `--robot.use_gripper=true`, so the serial gripper driver
requires the `xgripper` package, which provides the `xensegripper` Python
module:

```bash
cd ~/XGripper
pip install -e . --no-deps
```
or

```bash
pip install -e ./XGripper --no-deps
```


If you run without a gripper:

```bash
--robot.use_gripper=false
```

then `xgripper` is not required.

## Install This Plugin

```bash
cd lerobot-robot-franka-research3
pip install -e .
```

## Teleoperation

Pico4 Cartesian teleoperation:

```bash
lerobot-teleoperate \
  --robot.type=franka_research3 \
  --robot.fci_ip=192.168.99.111 \
  --robot.control_mode=cartesian_impedance \
  --robot.use_gripper=false \
  --teleop.type=pico4 \
  --fps=30 \
  --display_data=false
```

With the serial gripper enabled:

```bash
lerobot-teleoperate \
  --robot.type=franka_research3 \
  --robot.fci_ip=192.168.99.111 \
  --robot.control_mode=cartesian_impedance \
  --robot.use_gripper=true \
  --robot.gripper_sn=000015 \
  --teleop.type=pico4 \
  --fps=30
```

## Dynamics Tuning

Relative Franky dynamics factors:

```bash
--robot.velocity=0.8 \
--robot.acceleration=0.05 \
--robot.jerk=0.05
```

Optional absolute limits can also be set:

```bash
--robot.translation_velocity_limit=0.2
--robot.rotation_velocity_limit=0.8
--robot.joint_velocity_limit='[1.0,1.0,1.0,1.0,1.0,1.0,1.0]'
```

## Notes

Before moving a physical Franka, verify that:

- the robot is on the expected network and FCI is active
- brakes are released
- the active user has real-time permissions
- the configured start pose is reachable and collision-free
- `franky` can connect to the robot independently
