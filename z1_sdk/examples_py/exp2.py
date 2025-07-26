import sys
sys.path.append("../lib")
sys.path.append("../lib")
sys.path.append("/home/greenddb/Projects/IEEE_RAL_EXP2_SETUP/temporary/robotic_arm/z1_sdk/lib")
import unitree_arm_interface
import time
import numpy as np
import time
import csv
import serial
import os
# import rigid_grasp

print("Press ctrl+\ to quit process.")
# 
os.system("gnome-terminal -- bash -c 'cd ~/Projects/IEEE_RAL_EXP2_SETUP/temporary/robotic_arm/z1_controller/build && ./z1_ctrl'")
np.set_printoptions(precision=3, suppress=True)
arm =  unitree_arm_interface.ArmInterface(hasGripper=False)
armState = unitree_arm_interface.ArmFSMState
ser = serial.Serial('/dev/Bluetooth',115200,timeout=1,bytesize=8, stopbits=1)
# AABB开合开合开合1e14转速转速转速5aCCDD
start_string = "AABBd9d9d91e14007878785aCCDD"
stop_string = "AABBd3d3d31e14000000005aCCDD"
lose_string = "AABBe6e6e61e14000000005aCCDD"
start = bytes(bytearray.fromhex(start_string))
stop = bytes(bytearray.fromhex(stop_string))
lose = bytes(bytearray.fromhex(lose_string))
# ball_init
ser.write(stop)
#ball
# ser = serial.Serial('/dev/Bluetooth',115200,timeout=1,bytesize=8, stopbits=1)
# # start_string = "AABBd0d0d01e14007878785aCCDD"
# start_string = "AABBe6e6e61e14007878785aCCDD"
# stop_string = "AABBd0d0d01e14000000005aCCDD"
# start = bytes(bytearray.fromhex(start_string))
# stop = bytes(bytearray.fromhex(stop_string))

#rigid
# rigid_grasp = rigid_grasp.rigid_move(0,100,20) 

# #soft
# grasp_string = "AABB0080CCDD"
# drop_string = "AABB01FFCCDD"
# #soft
# grasp = bytes(bytearray.fromhex(grasp_string))
# drop = bytes(bytearray.fromhex(drop_string))

# ball_init
# ser.write(stop)

# soft_init

arm.loopOn()
arm.labelRun("start")

while True:
    #ball
    arm.labelRun("f1_top")
    ser.write(start)
    arm.labelRun("f1")
    time.sleep(0.5)
    ser.write(stop)
    # arm.labelRun("p1")
    # arm.labelRun("start")
    arm.labelRun("apple")
    # arm.labelRun("f1_lose")
    ser.write(lose)
    # time.sleep(0.5)
    # arm.labelRun("f1_mid")
    arm.labelRun("start")
    # #soft
    # # time.sleep(0.5)
    # ser.write(drop)
    # time.sleep(1.5)
    # arm.labelRun("sp1")
    # arm.labelRun("offset_sp")
    # ser.write(grasp)
    # time.sleep(1.5)
    # arm.labelRun("sp1")
    # arm.labelRun("start")
    # # ser.write(drop)

    # # rigit
    # rigid_grasp.open()
    # arm.labelRun("rp1")
    # arm.labelRun("offset_rp")
    # rigid_grasp.grasp()
    # time.sleep(0.6)
    # arm.labelRun("rp1")
    # arm.labelRun("start")  





    run = input("still want to continue?(y/n):")
    if run == "n":
        break

   
    
arm.backToStart()
arm.loopOff()
os.system('pkill gnome-terminal') 

