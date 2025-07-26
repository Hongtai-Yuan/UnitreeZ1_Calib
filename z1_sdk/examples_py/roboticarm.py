import sys
sys.path.append("../lib")
sys.path.append("/home/njau/robotic_arm/z1_sdk/lib")
sys.path.append("/home/njau/projects/my_yolo")
import unitree_arm_interface
import time
import numpy as np
import time
import csv
import final
import serial


# def 

print("Press ctrl+\ to quit process.")

np.set_printoptions(precision=3, suppress=True)
arm =  unitree_arm_interface.ArmInterface(hasGripper=True)
armState = unitree_arm_interface.ArmFSMState


ser = serial.Serial('/dev/Bluetooth',115200,timeout=1,bytesize=8, stopbits=1)
grasp_string = "AABB0001CCDD"
drop_string = "AABB0000CCDD"
drop_max = "AABB0101CCDD"


grasp = bytes(bytearray.fromhex(grasp_string))
drop = bytes(bytearray.fromhex(drop_string))
drop_m = bytes(bytearray.fromhex(drop_max))

while True:
    ser.write(drop)
    time.sleep(1)
    arm.loopOn()
    # arm.labelRun("stow")
    arm.labelRun("start")
    # arm.labelRun("forward")

    a = [0.0, 1.5, 0.0, 0.0, 0.0, 0.0]
    time.sleep(1)    
    l = float(input("Type the lenght of you grasper:(m)"))
    matirx = np.array([[-1.0,0.0,0.0],
                    [0.0,1.0,0.0],
                         [0.0,0.0,-1.0]])

    got_position = final.main()[0]
    print(got_position)
    position = np.array(got_position)
    b1 = np.dot(matirx,position)
    # c1 = np.array([0.573+0.052, 0.270, 0.665])
    c1 = np.array([0.4, 0.38, 0.61])
    c2 = np.array([-0.049, -0.33, 0.0])
    # c2 = np.array([-0.049, -0.29, 0.0])
    b = np.dot(matirx,position)+c1+c2
    print(b)
    a[3] = b[0] + 0.02
    a[4] = b[1] - 0.1
    # a[4] = b[1] + 0.052
    a[5] = b[2] + l + 0.02
    print(a)
    array = np.array(a)
    gripper_pos = 0 #-1.57rad~0rad
    jnt_speed = 0.8
    arm.MoveJ(array, gripper_pos, jnt_speed)
    # time.sleep(0.5)
    a[5] = b[2] + l - 0.02
    print(a)

    array = np.array(a)

    arm.MoveJ(array, gripper_pos, 0.5)

    # grasp_1 = input("would you want to grasp it?(y/n):")
    # if grasp_1 == "y":
    ser.write(grasp)
    time.sleep(2)
    print("have graspe/d yet")
    arm.labelRun("start")
#######################################################
    # finish = input("have grasped(y/n):.")
    print(ser.in_waiting)
    meg = ser.in_waiting
    if meg >0:
        point_b = np.array(a)
        point_b[4] = point_b[4] + 0.15
        point_b[5] = point_b[5] + 0.05
        arm.MoveJ(point_b, gripper_pos, jnt_speed)
        # drop1 = input("would you want to droop it?(y/n):")
        # if drop1 == "y":
        ser.write(drop)
        ser.write(drop_m)
        time.sleep(1)
        print('have dropped yet')
    else:
        print("no messages was received")
    time.sleep(1)
    arm.labelRun("start")


    # grasp_2 = input("would you want to take your arm back?(y/n):")
    # if grasp_2 == "y":
        # arm.labelRun("stow")
#########################################################################


    run = input("still want to continue?(y/n):")
    if run == "n":
        break
    

#ser.close()
arm.backToStart()
arm.loopOff()

