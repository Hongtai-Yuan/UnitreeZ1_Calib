import sys
sys.path.append("../lib")
sys.path.append("/home/x86/Z1Rbot/z1_sdk/lib")
sys.path.append("/home/x86/Z1Rbot/z1_sdk/lib")
import unitree_arm_interface
import time
import numpy as np
import os

print("Press ctrl+\ to quit process.")

os.system("gnome-terminal -- bash -c 'cd /home/x86/Z1Rbot/z1_controller/build && ./z1_ctrl'")
np.set_printoptions(precision=3, suppress=True)
arm =  unitree_arm_interface.ArmInterface(hasGripper=True)
armState = unitree_arm_interface.ArmFSMState
arm.loopOn()

# 1. highcmd_basic : armCtrlInJointCtrl
arm.labelRun("forward")
arm.startTrack(armState.JOINTCTRL)
jnt_speed = 0.8
# for i in range(0, 1000):
#     # dp = directions * speed; include 7 joints
#     arm.jointCtrlCmd([0,0,0,-1,0,0,-1], jnt_speed)
#     time.sleep(arm._ctrlComp.dt)

# 2. highcmd_basic : armCtrlByFSM
arm.labelRun("forward")
print("Current armp: ", arm.q)
T_forward = arm._ctrlComp.armModel.forwardKinematics(arm.q, 6)  # 计算末端位姿
print("Current position: ", T_forward)
gripper_pos = 0.0
jnt_speed = 1.0  
arm.MoveJ([0,0,0,0.481,-0.189,0.297], gripper_pos, jnt_speed)
print("Current armp: ", arm.q)
T_forward = arm._ctrlComp.armModel.forwardKinematics(arm.q, 6)  # 计算末端位姿
print("Current position: ", T_forward)
time.sleep(2)
# gripper_pos = -1.0
# cartesian_speed = 0.5
# arm.MoveL([0,0,0,0.45,-0.2,0.2], gripper_pos, cartesian_speed)
# gripper_pos = 0.0
# time.sleep(2)
# arm.MoveC([0,0,0,0.45,0,0.4], [0,0,0,0.45,0.2,0.2], gripper_pos, cartesian_speed)

# 3. highcmd_basic : armCtrlInCartesian
arm.labelRun("forward")
arm.startTrack(armState.CARTESIAN)
# angular_vel = 0.3
# linear_vel = 0.3
# for i in range(0, 1000):
#     arm.cartesianCtrlCmd([0,0,0,0,0,-1,-1], angular_vel, linear_vel)
#     time.sleep(arm._ctrlComp.dt)

arm.backToStart()
arm.loopOff()