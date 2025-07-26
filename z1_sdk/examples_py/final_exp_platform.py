import sys
sys.path.append("../lib")
sys.path.append("/home/greenddb/Projects/IEEE_RAL_EXP2_SETUP/temporary/robotic_arm/z1_sdk/lib")
sys.path.append("yolov8/src")
import unitree_arm_interface
import time
import numpy as np
import time
import csv
import arm_detector
import cv2
import os
import subprocess
import serial
import pyrealsense2 as rs
import threading
import time
import matplotlib.pyplot as plt

objects = None
annotated_frame = None
model = None
d435 = None
arm = None
armState = None
a = None
trans_matrix = None
ser = None
start = None
stop = None
lose = None
gripper_length = None
grasp_distance = None


# print("Press ctrl+\ to quit process.")
# ##打开z1_ctrl
# os.system("gnome-terminal -- bash -c 'cd ~/Projects/IEEE_RAL_EXP2_SETUP/temporary/robotic_arm/z1_controller/build && ./z1_ctrl'")
# np.set_printoptions(precision=3, suppress=True)
# arm =  unitree_arm_interface.ArmInterface(hasGripper=False)
# armState = unitree_arm_interface.ArmFSMState
# # # open the visualize.py
# # subprocess.Popen(["python", "yolov8/src/arm_detector.py"])
# # # initial the yolo
# # model, d435 = arm_detector.init()
# # 手爪的长度
# gripper_length = 0.25
# # 手抓要靠近物体的距离
# grasp_distance = 0.08
# # 启动手爪通讯
# ser = serial.Serial('/dev/Bluetooth',115200,timeout=1,bytesize=8, stopbits=1)
# start_string = "AABBd3d3d31e14007878785aCCDD"
# stop_string = "AABBd3d3d31e14000000005aCCDD"
# lose_string = "AABBe6e6e61e14000000005aCCDD"
# start = bytes(bytearray.fromhex(start_string))
# stop = bytes(bytearray.fromhex(stop_string))
# lose = bytes(bytearray.fromhex(lose_string))
# # ball_init
# ser.write(stop)
# a = [0.0, 1.5, 0.0, 0.0, 0.0, 0.0]
# # obtain the objects positions
# # aruco与机械臂坐标系的变换,机械臂坐标系在aruco坐标系下平移0.27m,逆时针旋转90度
# trans_matrix = np.array([[0.0, -1.0, 0.0, -0.21],
#                     [1.0, 0.0, 0.0, 0.0],
#                     [0.0, 0.0, 1.0, 0.6],
#                     [0.0, 0.0, 0.0, 1.0]])
# trans_matrix = np.linalg.inv(trans_matrix)

def init():
    global model, d435, arm, armState, a, trans_matrix, ser, start, stop, lose, gripper_length, grasp_distance
    # initial the yolo
    model, d435 = arm_detector.init()
    print("Press ctrl+\ to quit process.")
    ##打开z1_ctrl
    os.system("gnome-terminal -- bash -c 'cd ~/Projects/IEEE_RAL_EXP2_SETUP/temporary/robotic_arm/z1_controller/build && ./z1_ctrl'")
    np.set_printoptions(precision=3, suppress=True)
    arm =  unitree_arm_interface.ArmInterface(hasGripper=False)
    armState = unitree_arm_interface.ArmFSMState
    # # open the visualize.py
    # subprocess.Popen(["python", "yolov8/src/arm_detector.py"])
    # # initial the yolo
    # model, d435 = arm_detector.init()
    # 手爪的长度
    gripper_length = 0.25
    # 手抓要靠近物体的距离
    grasp_distance = 0.03
    # 连接座长度
    link = 0.025
    # 启动手爪通讯
    ser = serial.Serial('/dev/Bluetooth',115200,timeout=1,bytesize=8, stopbits=1)
    start_string = "AABBd4d4d41e14007878785aCCDD"
    stop_string = "AABBd3d3d31e14000000005aCCDD"
    lose_string = "AABBe6e6e61e14000000005aCCDD"
    start = bytes(bytearray.fromhex(start_string))
    stop = bytes(bytearray.fromhex(stop_string))
    lose = bytes(bytearray.fromhex(lose_string))
    # ball_init
    ser.write(stop)
    a = [0.0, 1.5, 0.0, 0.0, 0.0, 0.0]
    # obtain the objects positions
    # aruco与机械臂坐标系的变换,机械臂坐标系在aruco坐标系下平移0.21m,逆时针旋转90度
    trans_matrix = np.array([[0.0, -1.0, 0.0, -0.215],
                        [1.0, 0.0, 0.0, 0.02],
                        [0.0, 0.0, 1.0, 0.8],
                        [0.0, 0.0, 0.0, 1.0]])
    trans_matrix = np.linalg.inv(trans_matrix)


def detector():
    global objects, annotated_frame, model, d435
    # # initial the yolo
    # model, d435 = arm_detector.init()
    # 获取物体信息和图像
    objects, annotated_frame = arm_detector.main(model, d435)
    cv2.imshow("YOLOv8 Inference", annotated_frame)
    # return objects

# def visualize_and_exec():
#     global objects, annotated_frame
#     while True:
#         cv2.imshow("annotated_frame", annotated_frame)
#         # plt.imshow(annotated_frame)
#         # exec the main commond
#         thread = threading.Thread(target=main, args=())
#         # start the thread
#         thread.start()
#         # wait for the thread to finish
#         thread.join()
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             print("q pressed, exiting...")
#             rs.pipeline().stop()
#         break
    

    
def main():
    global objects, annotated_frame, a, trans_matrix, gripper_length, grasp_distance, ser, d435
    # print("Press ctrl+\ to quit process.")
    # ##打开z1_ctrl
    # os.system("gnome-terminal -- bash -c 'cd ~/Projects/IEEE_RAL_EXP2_SETUP/temporary/robotic_arm/z1_controller/build && ./z1_ctrl'")
    # np.set_printoptions(precision=3, suppress=True)
    # arm =  unitree_arm_interface.ArmInterface(hasGripper=False)
    # armState = unitree_arm_interface.ArmFSMState
    # # # open the visualize.py
    # # subprocess.Popen(["python", "yolov8/src/arm_detector.py"])
    # # # initial the yolo
    # # model, d435 = arm_detector.init()
    # # 手爪的长度
    # gripper_length = 0.25
    # # 手抓要靠近物体的距离
    # grasp_distance = 0.05
    # # 启动手爪通讯
    # ser = serial.Serial('/dev/Bluetooth',115200,timeout=1,bytesize=8, stopbits=1)
    # start_string = "AABBd9d9d91e14007878785aCCDD"
    # stop_string = "AABBd3d3d31e14000000005aCCDD"
    # lose_string = "AABBe6e6e61e14000000005aCCDD"
    # start = bytes(bytearray.fromhex(start_string))
    # stop = bytes(bytearray.fromhex(stop_string))
    # lose = bytes(bytearray.fromhex(lose_string))
    # # ball_init
    # ser.write(stop)


    # while True:
    # # 获取物体信息和图像
    # objects, annotated_frame = arm_detector.main(model, d435)
    # # 显示图像
    # cv2.imshow("YOLOv8 Inference", annotated_frame)
    # ser.write(drop)
    # time.sleep(1)
    arm.loopOn()
    # arm.labelRun("stow")
    arm.labelRun("start")
    # arm.labelRun("forward")
    # 创建空列表存位置
    id_position = []
    # 判断是否识别到物体
    if objects:
    # 提取出不同目标的坐标
        for obj in objects:
            # obj变为齐次
            # print(obj)
            homo_obj = np.concatenate((obj[0:3], [1.0]), axis=0)
            # 坐标变换到机械臂坐标系
            homo_obj = np.dot(trans_matrix, homo_obj)
            # print(homo_obj)
            # 每次轮询都创建一个副本，防止修改原数组
            array = a.copy()
            # 加上便宜误差
            array[3] = homo_obj[0]
            # 加上偏移误差
            array[4] = homo_obj[1]
            # z轴坐标为物体顶端距离加手爪长度减去抓取距离
            array[5] = homo_obj[2] + gripper_length - grasp_distance
            # array[5] = homo_obj[2]
            print(f"arm_base_pos: {array}")
            id = obj[3]
            # 将坐标存入列表
            id_position.append(array)
            # print(id)
            # 对列表进行排序,z轴坐标从大到小排序
        if id_position:
            id_position = sorted(id_position, key=lambda x: x[-1], reverse=True)
            print(f"id_position: {id_position}")
            ser.write(start)
            time.sleep(0.5)
            # add the mid position
            id_position_mid = id_position[0].copy()
            id_position_mid[5] = id_position_mid[5] + grasp_distance + 0.05
            arm.MoveJ(id_position_mid, 0.5)
            arm.MoveJ(id_position[0], 0.5)
            time.sleep(0.5)
            ser.write(stop)
            run = input("still want to continue?(y/n):")
            if run == "n":
                d435.pipeline.stop() 
                ser.close()
                arm.backToStart()
                arm.loopOff()
                ## 关闭z1_ctrl
                os.system('pkill gnome-terminal') 
            # if run == "y":
            #     print("continue")
            # else:
            #     print("invalid input")
            #     arm_detector.main().d435.pipeline.stop() 
            #     break

            # arm.labelRun("f1_mid")
            # arm.labelRun("f1_lose")
            # ser.write(lose)
            # time.sleep(0.5)
            # arm.labelRun("f1_mid")
            # arm.labelRun("start")
    else:
        raise Exception("No objects detected")


    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break
        # run = input("still want to continue?(y/n):")
        # if run == "n":
        #     arm_detector.main().d435.pipeline.stop() 
        #     break
        # if run == "y":
        #     continue
        # else:
        #     print("invalid input")
        #     arm_detector.main().d435.pipeline.stop() 
        #     break


    #ser.close()
    # arm.backToStart()
    # arm.loopOff()
    # ## 关闭z1_ctrl
    # os.system('pkill gnome-terminal') 
    
if __name__ == '__main__':
    init()
    while True:
        detector()
        main()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("q pressed, exiting...")
            rs.pipeline().stop()
            arm.backToStart()
            arm.loopOff()
            ## 关闭z1_ctrl
            os.system('pkill gnome-terminal') 
            break
    # visualize_and_exec()
    # while True:
    #     main()
    # create thread
    # robot_arm = threading.Thread(target=main, args=(objects,))
    # visualize = threading.Thread(target=visualize, args=(annotated_frame,))
    # start thread
    # visualize(annotated_frame)
    # robot_arm.start()
    # visualize.start()
    # robot_arm = main(objects)
    # wait for thread to finish
    # robot_arm.join()
    # visualize.join()
    # print("所有线程执行完毕")

