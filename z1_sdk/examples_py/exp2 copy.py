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

arm.loopOn()
# arm.labelRun("start")

while True:
    # name_list = []
    arm.labelRun("start")
    # arm.labelRun("p1")
    # arm.labelRun("p3")
    time.sleep(1)
    choice = input("would you want to teach the robot manually?(y/n):")
    if choice == "y":
        arm.teach('now')
        print("please move the robotic arm to the right place.")
        finish = input("Did you finish?(y/n):")
        if finish == "y":
            arm.setFsm(armState.JOINTCTRL)
            name = input('Type the name of this position:(p)')
            arm.labelSave(name)
            run = input("still want to continue?(y/n):")
            # print(run)
            if run == "n":
                break
            elif run == "y":
                continue
            else:
                raise Exception("invalid input")
        elif finish == "n":
            pass
        else:
            raise Exception("invalid input")
    elif choice == "n":
        move = input("would you want to move?(y/n):")
        if move == "y":
            #ball
            arm.labelRun("f1_top")
            ser.write(start)
            arm.labelRun("f1")
            time.sleep(0.5)
            ser.write(stop)
            # arm.labelRun("p1")
            arm.labelRun("apple_m")
            arm.labelRun("apple")
            # arm.labelRun("f1_lose")
            ser.write(lose)
            time.sleep(0.5)
            arm.labelRun("apple_m")
            # time.sleep(0.5)
            # arm.labelRun("f1_mid")
            arm.labelRun("start")
        elif move == "n":
            break
    else:
        raise Exception("invalid input")
    
    run = input("still want to continue?(y/n):")
    if run == "n":
        break
    elif run == "y":
        continue
    else:
        raise Exception("invalid input")
    
    # move = input("would you want to move?(y/n):")
    # if move == "y":
    #     #ball
    #     arm.labelRun("f1_top")
    #     ser.write(start)
    #     arm.labelRun("f1")
    #     time.sleep(0.5)
    #     ser.write(stop)
    #     # arm.labelRun("p1")
    #     # arm.labelRun("start")
    #     arm.labelRun("apple")
    #     # arm.labelRun("f1_lose")
    #     ser.write(lose)
    #     # time.sleep(0.5)
    #     # arm.labelRun("f1_mid")
    #     arm.labelRun("start")
    # elif move == "n":
    #     break
    # else:
    #     raise Exception("invalid input")
    
    # run = input("still want to continue?(y/n):")
    # if run == "n":
    #     break
    
    
    

 

   
    
arm.backToStart()
arm.loopOff()
os.system('pkill gnome-terminal') 

