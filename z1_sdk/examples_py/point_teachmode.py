import sys
sys.path.append("../lib")
sys.path.append("/home/x86/Z1Rbot/z1_sdk/lib")
sys.path.append("/home/x86/Z1Rbot/z1_sdk/lib")
import unitree_arm_interface
import time
import numpy as np
import time
import csv
import os
import serial

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
    arm.labelRun("test01")
    #arm.labelRun(name)
    # arm.labelRun("p1")
    # arm.labelRun("p3")
    time.sleep(1)
    choice = input("would you want to teach the robot manually?(y/n):")
    if choice == "y":
        arm.teach('now')
        print("please move the robotic arm to the right place.")
        finish = input("Do you finish?(y/n):")
        if finish == "y":
            arm.setFsm(armState.JOINTCTRL)
            name = input('Type the name of this position:(p)')
            arm.labelSave(name)
            run = input("still want to continue?(y/n):")
            # print(run)
            if run == "n":
                break
    else:
        break

    
    
arm.backToStart()
arm.loopOff()
## 关闭z1_ctrl
os.system('pkill gnome-terminal') 
