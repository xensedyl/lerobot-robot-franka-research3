import os
import PySide6
# 强制使用 PySide6 自带的 Qt 插件，避免与 conda 安装的 qt6-main(6.10.2) 冲突导致 xcb 插件加载失败
_qt_root = os.path.join(os.path.dirname(PySide6.__file__), "Qt")
os.environ.setdefault("QT_PLUGIN_PATH", os.path.join(_qt_root, "plugins"))
os.environ.setdefault("QT_QPA_PLATFORM_PLUGIN_PATH", os.path.join(_qt_root, "plugins", "platforms"))

from PySide6 import QtCore, QtGui, QtUiTools
from PySide6.QtGui import QImage
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtCore import QEventLoop, QTimer
import threading as th
import numpy as np
from scipy.spatial.transform import Rotation
from franky import Robot, Affine, CartesianMotion, JointMotion, RelativeDynamicsFactor
import requests

import os, time, sys

np.set_printoptions(precision=5, suppress=True)

FRANKA_IP = "192.168.99.111"
GRIPPER_URL = "http://127.0.0.1:7001"
JOINT_HOME = [0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4]

class qt_cy(QMainWindow):
    def __init__(self):
        super().__init__()
        self.qt_init()
        self.franky_init()

    def franky_init(self):
        self.robot = Robot(FRANKA_IP)
        self.robot.recover_from_errors()
        # self.robot.relative_dynamics_factor = 0.05
        self.robot.relative_dynamics_factor = RelativeDynamicsFactor(
            velocity=0.25, acceleration=0.05, jerk=0.05
        )
        self.init_p = [0.4, 0, 0.55, 1, 0, 0, 0]
        self.grab_water_p = [0.3, -0.15, 0.1, 0.5, 0.5, -0.5, 0.5]

    def qt_init(self):
        self.ui = QtUiTools.QUiLoader().load(f"{os.path.dirname(__file__)}/franka_qt.ui")  # 加载文件
        self.ui.pushButton.clicked.connect(lambda: self.start_robot(msg="get_pos"))
        self.ui.pushButton_2.pressed.connect(lambda: self.start_robot(msg='reset'))
        self.ui.pushButton_2.released.connect(lambda: self.stop_move)
        self.ui.pushButton_3.clicked.connect(lambda: self.start_robot(msg='open_gripper'))
        self.ui.pushButton_4.clicked.connect(lambda: self.start_robot(msg='close_gripper'))
        self.ui.pushButton_5.clicked.connect(lambda: self.start_robot(msg='move_gripper'))
        buttons = {
            "pushButton_6": "x+",   # X+
            "pushButton_7": "x-",   # X-
            "pushButton_8": "y+",   # Y+
            "pushButton_9": "y-",   # Y-
            "pushButton_10": "z+",   # Z+
            "pushButton_11": "z-",   # Z-
            "pushButton_12": "qx+",  # Rx+
            "pushButton_13": "qx-",  # Rx-
            "pushButton_14": "qy+", # Ry+
            "pushButton_15": "qy-", # Ry-
            "pushButton_16": "qz+", # Rz+
            "pushButton_17": "qz-"  # Rz-
            }
        for btn_name, action in buttons.items():
            btn = getattr(self.ui, btn_name, None)
            if btn:
                btn.pressed.connect(lambda a=action: self.start_move(a))
                btn.released.connect(self.stop_move)
        self.ui.pushButton_18.clicked.connect(lambda: self.start_robot(msg='stop'))
        self.ui.pushButton_19.clicked.connect(lambda: self.start_robot(msg='test'))
        self.ui.pushButton_20.clicked.connect(lambda: self.start_robot(msg='clear_err'))
        
        self.ui.horizontalSlider.valueChanged.connect(self.update_label)
        self.ui.label.setText(f"夹爪宽度:{self.ui.horizontalSlider.value()} mm")

    def start_move(self, action):
        self.running = True
        self.target_action = action
        self.move_thread = th.Thread(target=self.move_th, daemon=True)
        self.move_thread.start()

    def stop_move(self):
        self.running = False
        if self.move_thread:
            self.move_thread.join()

    def move_th(self):
        step_pos = 0.01
        step_angle = np.radians(3)
        start_pos = self.get_pos()
        while self.running:
            curr_pos = self.get_pos().copy()  
            # 处理平移
            if self.target_action == "x+":
                curr_pos[0] += step_pos
            elif self.target_action == "x-":
                curr_pos[0] -= step_pos
            elif self.target_action == "y+":
                curr_pos[1] += step_pos
            elif self.target_action == "y-":
                curr_pos[1] -= step_pos
            elif self.target_action == "z+":
                curr_pos[2] += step_pos
            elif self.target_action == "z-":
                curr_pos[2] -= step_pos
            elif self.target_action in ["qx+", "qx-", "qy+", "qy-", "qz+", "qz-"]:
                axis_map = {
                    "qx+": [1, 0, 0], "qx-": [1, 0, 0],
                    "qy+": [0, 1, 0], "qy-": [0, 1, 0],
                    "qz+": [0, 0, 1], "qz-": [0, 0, 1]
                }
                angle = step_angle if "+" in self.target_action else -step_angle
                curr_pos[3:] = self.apply_rotation(curr_pos[3:], axis_map[self.target_action], angle)
            # 机械臂运动
            if self.target_action in ["x+", "x-", "y+", "y-", "z+", "z-"]:
                start_pos[:3] = curr_pos[:3]
            elif self.target_action in ["qx+", "qx-", "qy+", "qy-", "qz+", "qz-"]:
                start_pos[3:] = curr_pos[3:]
            translation = np.array(start_pos[:3], dtype=np.float64).tolist()
            quaternion = np.array(start_pos[3:], dtype=np.float64).tolist()  # [qw, qx, qy, qz]
            motion = CartesianMotion(Affine(translation, quaternion))
            self.robot.move(motion, asynchronous=True)
            time.sleep(0.05)

    def start_robot(self,msg):
        self.robot_thread = th.Thread(target=self.robot_th, args=(msg,), daemon=True)
        self.robot_thread.start()

    def robot_th(self, msg):
        if msg == "get_pos":
            pos = self.get_pos()
            joint = self.get_joint()
            print("pos:",pos)
            print("joint:",joint)
        elif msg == "open_gripper":
            self.gripper_move(pos = 85)
        elif msg == "close_gripper":
            self.gripper_move(pos = 0)
        elif msg == "move_gripper":
            pos = self.ui.horizontalSlider.value()
            self.gripper_move(pos = pos)
        elif msg == "reset":
            self.clear_err()
            pos_list = [self.init_p, self.grab_water_p]
            self.xarm_move(pos=np.array(pos_list[self.ui.comboBox.currentIndex()]))
        elif msg == "stop":
            self.running = False
            self.robot.stop()
            if self.move_thread:
                self.move_thread.join()
        elif msg == "clear_err":
            self.clear_err()
        elif msg == "test":
            if self.target_pose is None:
                print("尚未收到 /target_pose_tool 消息，无法执行")
                return
            now_pos = self.get_pos()
            grab_pos = now_pos[:3] + self.target_pose[:3]
            grab_pos = np.concatenate([grab_pos, self.target_pose[3:]])
            print("grab_pos",grab_pos)
            grab_min_pos = np.array(grab_pos)
            grab_min_pos[2] += 0.2
            min_home = np.array([grab_min_pos[0], grab_min_pos[1], grab_min_pos[2],1,0,0,0])
            put_min_pos = np.array([0.36771, 0.16987, 0.26, -0.5, -0.5, -0.5, 0.5])
            put_pos = np.array([0.363, 0.16987, 0.13677, -0.5, -0.5, -0.5, 0.5])

            # first grasp
            self.gripper_move(pos=42)
            self.xarm_move(pos=grab_min_pos)
            time.sleep(0.3)
            self.xarm_move(pos=grab_pos)
            self.gripper_move(pos=15)
            time.sleep(1)
            self.xarm_move(pos=grab_min_pos)
            time.sleep(0.3)
            self.xarm_move(pos=min_home)
            self.xarm_move(pos=put_min_pos)
            self.xarm_move(pos=put_pos)
            self.gripper_move(pos=42)

            # reset
            self.xarm_move(pos=put_min_pos)
            self.xarm_move(pos=np.array(self.init_p))
            self.gripper_move(pos=85)
            time.sleep(2)

            # second grasp
            if self.target_pose is None:
                print("尚未收到 /target_pose_tool 消息，无法执行")
                return
            now_pos = self.get_pos()
            grab_pos2 = now_pos[:3] + self.target_pose[:3]
            grab_pos2 = np.concatenate([grab_pos2, self.target_pose[3:]])
            print("grab_pos2",grab_pos2)
            grab_min_pos2 = np.array(grab_pos2)
            grab_min_pos2[2] += 0.2
            min_home2 = np.array([grab_min_pos2[0], grab_min_pos2[1], grab_min_pos2[2],1,0,0,0])
            put_min_pos2 = np.array([0.44771, 0.16987, 0.26, -0.5, -0.5, -0.5, 0.5])
            put_pos2 = np.array([0.443, 0.16987, 0.13677, -0.5, -0.5, -0.5, 0.5])
            self.gripper_move(pos=42)
            self.xarm_move(pos=grab_min_pos2)
            self.xarm_move(pos=grab_pos2)
            self.gripper_move(pos=15)
            time.sleep(1)
            self.xarm_move(pos=grab_min_pos2)
            self.xarm_move(pos=min_home2)
            self.xarm_move(pos=put_min_pos2)
            self.xarm_move(pos=put_pos2)
            self.gripper_move(pos=42)

            # reset
            self.xarm_move(pos=put_min_pos2)
            self.xarm_move(pos=np.array(self.init_p))
            self.gripper_move(pos=85)

        else:
            print("输入信息有误")
            print(f"当前信息为{msg}")

    def joint_reset(self):
        print("正在执行关节复位...")
        motion = JointMotion(JOINT_HOME)
        self.robot.move(motion)

    def get_grip_pos(self):
        response = requests.get(f"{GRIPPER_URL}/get_pos")
        return response.json()["position"]

    def get_pos(self):
        affine = self.robot.current_cartesian_state.pose.end_effector_pose
        t = np.array(affine.translation, dtype=np.float64)
        q = np.array(affine.quaternion, dtype=np.float64)  # [qw, qx, qy, qz]
        return np.concatenate([t, q])  # [x, y, z, qw, qx, qy, qz]

    def get_joint(self):
        joint_pos = self.robot.current_joint_state.position
        return np.array(joint_pos)

    def xarm_move(self, x=None, y=None, z=None, qw=None, qx=None, qy=None, qz=None, pos=None, asynchronous=False):
        if pos is None:
            pos = np.array([x, y, z, qw, qx, qy, qz])
        translation = pos[:3].tolist()
        quaternion = pos[3:].tolist()  # [qw, qx, qy, qz]
        motion = CartesianMotion(Affine(translation, quaternion))
        self.robot.move(motion, asynchronous=asynchronous)

    def clear_err(self):
        self.robot.recover_from_errors()

    def xarm_reset(self):
        pos = self.grab_water_p[:]
        self.xarm_move(pos=np.array(pos))

    def gripper_move(self, pos, vmax=100, fmax=30):
        print(f"gripper_move:{pos}")
        requests.post(f"{GRIPPER_URL}/move", params={"pos": pos, "vmax": vmax, "fmax": fmax})
    
    def update_label(self,value):
        self.ui.label.setText(f"夹爪宽度:{value} mm")
    
    def apply_rotation(self, current_quat, axis, angle):
        """
        使用scipy的Rotation实现旋转
        :param current_quat: 当前四元数 [qx,qy,qz,qw]
        :param axis: 旋转轴 [x,y,z]
        :param angle: 旋转角度(弧度)
        :return: 新四元数 [qx,qy,qz,qw]
        """
        current_rot = Rotation.from_quat(current_quat)
        delta_rot = Rotation.from_rotvec(angle * np.array(axis))
        new_rot = delta_rot * current_rot
        return new_rot.as_quat()  # 返回 [qx,qy,qz,qw]

    
    def run(self):
        pass

    def shutdown(self):
        pass

if __name__ == "__main__":

    app = QApplication(sys.argv)
    q = qt_cy()
    q.ui.show()
    q.run()
    ret = app.exec()
    q.shutdown()
    sys.exit(ret)