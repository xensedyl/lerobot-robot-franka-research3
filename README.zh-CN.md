# LeRobot Franka Research 3 Robot

这是 Franka Research 3 的独立 LeRobot robot 插件包，底层使用
`franky` 控制机械臂。

该包注册：

- `--robot.type=franka_research3`

支持：

- Franky 关节位置运动
- Franky 笛卡尔运动
- Franky 笛卡尔阻抗 torque tracking
- 可选 Xense 串口夹爪
- 可选腕部相机和 Xense 触觉相机

## franky 依赖

本包依赖：

```text
git@github.com:xensedyl/franky.git
```

Python 包名是 `franky-control`，import 模块名是 `franky`。

### 从本地源码安装 franky

如果使用本地 `franky` 源码，安装前必须先初始化 git submodule。
其中 `ruckig` 是 CMake 必需的子模块；如果 `ruckig/` 是空目录，会报
`does not contain a CMakeLists.txt file`。

```bash
cd franky
git submodule update --init --recursive
pip install -e . --no-build-isolation
```

如果当前环境里的 `cmake` 来自 Python `cmake` 包，建议带
`--no-build-isolation`。否则 pip 的隔离构建环境里可能在执行
`cmake --version` 时出现 `ModuleNotFoundError: No module named 'cmake'`。

如果是重新 clone `franky`，可以直接带上 submodule：

```bash
git clone --recursive git@github.com:xensedyl/franky.git
```

### 直接从 Git 安装 franky

```bash
pip install "franky-control @ git+ssh://git@github.com/xensedyl/franky.git"
```

如果你本地 `third_party/franky` 有额外补丁，优先用本地 editable 安装。

## Xense 串口夹爪

默认 `--robot.use_gripper=true`，所以会用到 `xgripper` 包提供的
`xensegripper` 模块：

```bash
cd ~/XGripper
pip install -e . --no-deps
```
or

```bash
pip install -e ./XGripper --no-deps
```

如果没有夹爪，可以运行时关闭：

```bash
--robot.use_gripper=false
```

这种情况下不需要安装 `xgripper`。

## 安装本插件

```bash
cd lerobot-robot-franka-research3
pip install -e .
```

## 运行示例

Pico4 笛卡尔遥操作：

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

启用串口夹爪：

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

## Franky 动力学参数

相对速度/加速度/加加速度：

```bash
--robot.velocity=0.8 \
--robot.acceleration=0.05 \
--robot.jerk=0.05
```

也可以设置绝对限制：

```bash
--robot.translation_velocity_limit=0.2
--robot.rotation_velocity_limit=0.8
--robot.joint_velocity_limit='[1.0,1.0,1.0,1.0,1.0,1.0,1.0]'
```

## Qt 图形界面调试工具

`franka_qt/franky_qt.py` 提供一个基于 PySide6 的图形界面，用于手动调试机械臂：
读取当前位姿/关节、笛卡尔点动（XYZ 平移 + 绕 XYZ 旋转）、复位到预设位姿、
控制夹爪开合、急停和清除错误。

### 依赖

```bash
pip install PySide6 numpy scipy requests
```

机械臂控制依赖 `franky`（安装方式见上文）。`franka_qt.ui` 必须与
`franky_qt.py` 放在同一目录。

### 运行

```bash
cd franka_qt
python franky_qt.py
```

界面会直接连接 `FRANKA_IP`（脚本内默认 `192.168.99.111`），如有不同请修改脚本顶部常量。

### 夹爪 HTTP 服务

夹爪通过 HTTP 控制，脚本默认访问 `GRIPPER_URL`（默认 `http://127.0.0.1:7001`）：

- `POST /move`，参数 `pos`、`vmax`、`fmax`
- `GET /get_pos`，返回 `{"position": ...}`

使用夹爪相关按钮前，需要先启动对应的夹爪 HTTP 服务。

### 常见问题

- **`No module named 'rclpy._rclpy_pybind11'`**：脚本已不依赖 ROS，无需处理。
- **`Could not find the Qt platform plugin "xcb"`**：通常是 conda 安装的
  `qt6-main` 与 pip 安装的 PySide6 版本冲突。脚本顶部已自动把 Qt 插件路径
  指向 PySide6 自带目录来规避此问题。

## 注意

真机运动前确认：

- Franka 网络和 FCI 正常
- 刹车已释放
- 当前用户具备实时权限
- 起始位姿可达且无碰撞
- `franky` 能独立连接机器人
