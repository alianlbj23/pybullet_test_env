# ############################################
# Example to use solveInversePositionKinematics for a target position
#
# Author : Deepak Raina @ IIT Delhi
# Version : 0.1
# ############################################

import numpy as np
from pybullet_controller import RobotController
import pybullet as p

# 創建 RobotController 物件
robot = RobotController(end_eff_index=5)
robot.createWorld(GUI=True)
input("enter")
base_position, base_orientation = p.getBasePositionAndOrientation(robot.robot_id)
base_orientation_euler = p.getEulerFromQuaternion(base_orientation)
print("機器人的世界座標位置:", base_position)
print("機器人的世界座標方向 (Euler 角):", np.degrees(base_orientation_euler))

# 設定目標位置 (x, y, z) 和方向 (roll, pitch, yaw)
# 例如，將目標設置在機器人工作空間內的特定位置和方向
target_position = [0.3 + base_position[0], -0.2 + base_position[1], 0.2 + base_position[2]]  # x, y, z
target_orientation = [0, np.pi/2, 0]  # roll, pitch, yaw
robot.setJointPosition([1.57, 1.57, 1.57, 1.57, 1.57])
# 計算逆運動學得到的關節角度
end_eff_pose = target_position  # 將位置和方向合併為6個值的列表
joint_angles = robot.solveInversePositionKinematics(end_eff_pose)
joint_angles = robot.moveTowardsTarget(target_position)
# 檢查並顯示計算得到的關節角度
joint_angles_degrees = np.degrees(joint_angles)
print("計算得到的關節角度:", joint_angles_degrees)
input("Press Enter to continue...")
angle = ([0,0,0,0])
# robot.setJointPosition(angle)
# 使用計算得到的角度來設定機器人的關節位置
# 如果 `joint_angles` 的數量與 `robot.controllable_joints` 的數量相符，則進行設定
if joint_angles and len(joint_angles) >= len(robot.controllable_joints):
    robot.setJointPosition(joint_angles[:len(robot.controllable_joints)])
else:
    print("Error: 計算的關節角度與可控制的關節數量不匹配。")

# 暫停以檢查機器人位置
input("Press Enter to end...")
p.disconnect()
