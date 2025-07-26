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
import queue
import multiprocessing
import matplotlib.pyplot as plt

# 全局变量
exit_flag = False
objects = None
annotated_frame = None
# 创建队列
frame_queue = queue.Queue(maxsize=1)


def detector_process(model,d435):
    global objects, annotated_frame, exit_flag
    # 添加打印确认线程启动
    print("detector_thread started")  
    print(f"detect:{exit_flag}")
    while True:
        # try:
        # 获取物体信息和图像
        objects, annotated_frame = arm_detector.main(model, d435)
        # time.sleep(0.05)  # 控制检测频率，减少资源占用
        # 将结果放进队列
        if not frame_queue.full(): # 如果队列不满，则放入结果
            frame_queue.put((objects, annotated_frame)) 
        if exit_flag:
            break
        # except Exception as e:
        #     print(f"Error in detector_thread: {e}")
            # break
        # time.sleep(0.05) # 控制检测频率，减少资源占用
            # print(f"objects: {objects}")
            # print(f"annotated_frame: {annotated_frame}")
        # time.sleep(0.1)
        # if objects:
        #     print(f"Detected objects: {objects}")
        # else:
        #     print("No objects detected")
    print("detector_thread exited")

def visualize_process():
    global annotated_frame, exit_flag
    # 添加打印确认线程启动
    print("visualize_thread started")  
    print(f"visul:{exit_flag}")
    while True:
        try:
            # 从队列中获取结果
            objects, annotated_frame = frame_queue.get(timeout=1)
            if annotated_frame is not None:
                plt.imshow(annotated_frame)
                
                # cv2.imshow("annotated_frame", annotated_frame)  
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rs.pipeline().stop()
                print("q pressed, exiting...")
                exit_flag = True
                break
            # time.sleep(0.1)
        except queue.Empty:
            pass # 如果队列为空，则跳过这一帧
    print("visualize_thread exiting")

    

def arm_control_process():
    global objects, exit_flag
    # 添加打印确认线程启动
    print("arm_control_thread started") 
    print("Press ctrl+\ to quit process.")
    ##打开z1_ctrl
    os.system("gnome-terminal -- bash -c 'cd ~/Projects/IEEE_RAL_EXP2_SETUP/temporary/robotic_arm/z1_controller/build && ./z1_ctrl'")
    np.set_printoptions(precision=3, suppress=True)
    arm =  unitree_arm_interface.ArmInterface(hasGripper=False)
    armState = unitree_arm_interface.ArmFSMState
    # 手爪的长度
    gripper_length = 0.25
    # 手抓要靠近物体的距离
    grasp_distance = 0.05
    # 启动手爪通讯
    ser = serial.Serial('/dev/Bluetooth',115200,timeout=1,bytesize=8, stopbits=1)
    start_string = "AABBd9d9d91e14007878785aCCDD"
    stop_string = "AABBd3d3d31e14000000005aCCDD"
    lose_string = "AABBe6e6e61e14000000005aCCDD"
    start = bytes(bytearray.fromhex(start_string))
    stop = bytes(bytearray.fromhex(stop_string))
    lose = bytes(bytearray.fromhex(lose_string))
    # ball_init
    ser.write(stop)


    print(f"arm:{exit_flag}")
    while not exit_flag:
        try:
            # cv2.imshow("annotated_frame", annotated_frame)
        
            arm.loopOn()
            # arm.labelRun("stow")
            arm.labelRun("start")
            # arm.labelRun("forward")
            a = [0.0, 1.5, 0.0, 0.0, 0.0, 0.0]
            # obtain the objects positions
            # aruco与机械臂坐标系的变换,机械臂坐标系在aruco坐标系下平移0.27m,逆时针旋转90度
            trans_matrix = np.array([[0.0, -1.0, 0.0, -0.21],
                                [1.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 1.0, 0.0],
                                [0.0, 0.0, 0.0, 1.0]])
            trans_matrix = np.linalg.inv(trans_matrix)
            # 创建空列表存位置
            id_position = []
            objects, annotated_frame = frame_queue.get(timeout=0.1)
            # if objects is not None:
            if objects:
                # cv2.imshow("annotated_frame", annotated_frame)
                
                # 提取出不同目标的坐标
                for obj in objects:
                    # obj变为齐次
                    # print(obj)
                    homo_obj = np.concatenate((obj[0:3], [1.0]), axis=0)
                    # 坐标变换到机械臂坐标系
                    homo_obj = np.dot(trans_matrix, homo_obj)
                    print(homo_obj)
                    # 每次轮询都创建一个副本，防止修改原数组
                    array = a.copy()
                    # 加上便宜误差
                    array[3] = homo_obj[0]
                    # 加上偏移误差
                    array[4] = homo_obj[1]
                    # z轴坐标为物体顶端距离加手爪长度减去抓取距离
                    array[5] = homo_obj[2] + gripper_length - grasp_distance
                    print(array)
                    id = obj[3]
                    # 将坐标存入列表
                    id_position.append(array)
                    # print(id)
                    # 对列表进行排序,z轴坐标从大到小排序
                # cv2.imshow("annotated_frame", annotated_frame)
                    
                if id_position:
                    id_position = sorted(id_position, key=lambda x: x[-1], reverse=True)
                    ser.write(start)
                    time.sleep(0.5)
                    # add the mid position
                    id_position_mid = id_position[0].copy()
                    id_position_mid[5] = id_position_mid[5] + 0.05
                    # cv2.imshow("annotated_frame", annotated_frame)
                    
                    arm.MoveJ(id_position_mid, 0.5)
                    # cv2.imshow("annotated_frame", annotated_frame)
                    
                    arm.MoveJ(id_position[0], 0.5)
                    # cv2.imshow("annotated_frame", annotated_frame)
                    
                    time.sleep(1)
                    ser.write(stop)
                    # cv2.imshow("annotated_frame", annotated_frame)
                    
                    arm.labelRun("f_mid")
                    # cv2.imshow("annotated_frame", annotated_frame)
                    
                    arm.labelRun("f_lose")
                    # cv2.imshow("annotated_frame", annotated_frame)
                    
                    ser.write(lose)
                    # cv2.imshow("annotated_frame", annotated_frame)
                    
                    arm.labelRun("f_mid")
                    # cv2.imshow("annotated_frame", annotated_frame)
                    
                    arm.labelRun("start")
                    # cv2.imshow("annotated_frame", annotated_frame)
                    
            else:
                raise Exception("No objects detected")
                # continue

        except queue.Empty:
            continue  # 如果队列为空，则继续等待
        if exit_flag:
            break
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
    print("arm_control_thread exiting")
    arm.backToStart()
    arm.loopOff()
    ## 关闭z1_ctrl
    os.system('pkill gnome-terminal') 
    
if __name__ == '__main__':
    # initial the yolo
    model, d435 = arm_detector.init()
    if model is None or d435 is None:
        print("Failed to initialize YOLO model or camera")
        exit_flag = True  # 防止无限循环
        
    detection_process = multiprocessing.Process(target=detector_process, args=(model, d435))
    detection_process.start()  # 启动检测进程

    visualization_process = multiprocessing.Process(target=visualize_process)
    visualization_process.start()  # 启动可视化进程

    arm_process = multiprocessing.Process(target=arm_control_process)
    arm_process.start()  # 启动控制进程

    print("All processes started")

    # 等待进程完成
    detection_process.join()
    visualization_process.join()
    arm_process.join()

    print("所有进程执行完毕，程序退出")


