# ############################################
# Example to use solveInversePositionKinematics for a target position
#
# Author : Deepak Raina @ IIT Delhi
# Version : 0.1
# ############################################

import numpy as np
from pybullet_controller import RobotController
import pybullet as p
import time

# 創建 RobotController 物件
robot = RobotController(end_eff_index=5)
robot.createWorld(GUI=True)

# 獲取機器人的世界座標位置和方向
base_position, base_orientation = p.getBasePositionAndOrientation(robot.robot_id)
base_orientation_euler = p.getEulerFromQuaternion(base_orientation)
print("機器人的世界座標位置:", base_position)
print("機器人的世界座標方向 (Euler 角):", np.degrees(base_orientation_euler))

# 設定目標位置 (x, y, z) 和方向 (roll, pitch, yaw)
target_position = [0.1 + base_position[0], 0.2 + base_position[1], 0.3 + base_position[2]]
target_orientation = [0, np.pi/2, 0]

# 將目標位置和方向合併為逆運動學所需的格式
end_eff_pose = target_position + target_orientation
joint_angles = robot.solveInversePositionKinematics(end_eff_pose)
joint_angles_degrees = np.degrees(joint_angles)
print("計算得到的關節角度:", joint_angles_degrees)
input("Press Enter to continue...")

# 初始化關節角度列表，只有第一軸角度從 0 到 180 變化，其他保持不變
initial_joint_angles = [0.0] * len(robot.controllable_joints)
for angle in range(0, 181, 5):  # 每次增加 5 度，直到 180 度
    target_joint_angles = initial_joint_angles.copy()  # 複製初始角度
    target_joint_angles[3] = np.radians(angle)  # 僅修改第一軸的角度
    robot.setJointPosition(target_joint_angles)  # 設定關節位置
    print(f"當前第一軸目標角度 (度數): {angle}")
    time.sleep(0.1)  # 延遲以觀察變化

print("完成第一軸逐步移動至 180 度。")

# 暫停以檢查機器人位置
input("Press Enter to end...")
p.disconnect()
