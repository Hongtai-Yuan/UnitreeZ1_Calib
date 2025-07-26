import sys
sys.path.append("../lib")
sys.path.append("/home/x86/Z1Rbot/z1_sdk/lib")
sys.path.append("/home/x86/Z1Rbot/z1_sdk/lib")
import unitree_arm_interface
import time
import numpy as np
import time
import os

print("Press ctrl+\ to quit process.")
os.system("gnome-terminal -- bash -c 'cd /home/x86/Z1Rbot/z1_controller/build && ./z1_ctrl'")
np.set_printoptions(precision=3, suppress=True)
arm =  unitree_arm_interface.ArmInterface(hasGripper=True)
armState = unitree_arm_interface.ArmFSMState
# ser = serial.Serial('/dev/Bluetooth',115200,timeout=1,bytesize=8, stopbits=1)
# # AABB开合开合开合1e14转速转速转速5aCCDD
# start_string = "AABBd9d9d91e14007878785aCCDD"
# stop_string = "AABBd3d3d31e14000000005aCCDD"
# lose_string = "AABBe6e6e61e14000000005aCCDD"
# start = bytes(bytearray.fromhex(start_string))
# stop = bytes(bytearray.fromhex(stop_string))
# lose = bytes(bytearray.fromhex(lose_string))
# ball_init
# ser.write(stop)
#name = "start"

# run = True

while True:
    arm.loopOn()
    time.sleep(1)
    arm.labelRun('0620')
    choice_0 = input("would you want to play the robot manually?(y/n):")
    if choice_0 == "y":
        choice_1 = input("Do you want to play the point or the trajectory?(p/t):")
        if choice_1 == "p":
            name = input('Type the name of new position:')
            arm.labelRun(name)

            T_forward = arm._ctrlComp.armModel.forwardKinematics(arm.q, 6)  # 计算末端位姿
            print("Current position: ", T_forward)

            finish = input("Do you finish?(y/n):")
            if finish == "y":
                break
        elif choice_1 == "t":
            name = input('Type the name of trajectory:')
            arm.teachRepeat(name)
            finish = input("Do you finish?(y/n):")
            if finish == "y":
                break
        else:
            print("wrong input, please try again.")
            continue
    else:
        break

    
    
arm.backToStart()
arm.loopOff()
## 关闭z1_ctrl
os.system('pkill gnome-terminal') 
