import threading
import pyrealsense2 as rs
import numpy as np
import cv2
import sys
import time
import os

sys.path.append("../lib")
sys.path.append("/home/x86/Z1Rbot/z1_sdk/lib")
import unitree_arm_interface

# 线程同步事件
capture_event = threading.Event()
stop_event    = threading.Event()

# 全局保存目录
save_dir = 'handeye_img_pose/img'
os.makedirs(save_dir, exist_ok=True)

# 机械臂接口
arm = unitree_arm_interface.ArmInterface(hasGripper=True)
arm.loopOn()




import csv

presets = []
with open('z1_controller/config/eyehandpose.csv', newline='') as f:
    reader = csv.reader(f)
    count = 0
    for row in reader:
        if not row or row[0].startswith('#'):
            continue
        name = row[0].strip()
        # 跳过 forward（如果仍需要）
        if name == 'forward':
            continue

        values = [float(x) for x in row[1:1+6]]
        presets.append((name, values))

        count += 1
        if count >= 30:
            break

print(presets)

current_joints = None
num            = 0
captured_poses = []


import math

def rotm_to_rpy(R):
    """
    从 3×3 旋转矩阵 R 提取 ZYX 欧拉角:
    - roll  = 绕 X 轴旋转
    - pitch = 绕 Y 轴旋转
    - yaw   = 绕 Z 轴旋转
    返回 (roll, pitch, yaw)，单位：弧度
    """
    sy = math.hypot(R[0,0], R[1,0])
    singular = sy < 1e-6

    if not singular:
        roll  = math.atan2( R[2,1],  R[2,2])
        pitch = math.atan2(-R[2,0],   sy    )
        yaw   = math.atan2( R[1,0],   R[0,0])
    else:
        # 接近奇异情况
        roll  = math.atan2(-R[1,2],  R[1,1])
        pitch = math.atan2(-R[2,0],   sy    )
        yaw   = 0.0

    return roll, pitch, yaw



def matrix_to_rpy_xyz(T):
    R = T[:3, :3]
    roll, pitch, yaw = rotm_to_rpy(R)
    x, y, z = T[0,3], T[1,3], T[2,3]
    return [roll, pitch, yaw, x, y, z]






def camera_thread():
    global num
    pipeline = rs.pipeline()
    cfg      = rs.config()
    cfg.enable_stream(rs.stream.depth, 640,480, rs.format.z16,30)
    cfg.enable_stream(rs.stream.color,640,480, rs.format.bgr8,30)
    pipeline.start(cfg)
    align = rs.align(rs.stream.color)

    try:
        # 当 arm_thread 结束并且 capture_event 也处理完毕后退出
        while not stop_event.is_set() or capture_event.is_set():
            frames  = pipeline.wait_for_frames()
            aligned = align.process(frames)
            c_frame = aligned.get_color_frame()
            if not c_frame:
                continue

            color_img = np.asanyarray(c_frame.get_data())
            cv2.imshow('RGB', color_img)
            cv2.waitKey(1)

            if capture_event.is_set():
                num += 1
                # 保存 RGB 图，命名如 save_dir+'/img1.png'
                cv2.imwrite(save_dir + '/img' + str(num) + '.png', color_img)
                # 累积关节角度
                T_forward = arm._ctrlComp.armModel.forwardKinematics(current_joints.copy(), 6)  # 计算末端位姿
                T_in = matrix_to_rpy_xyz(T_forward)
                captured_poses.append(T_in)
                print(f"num:{num}, joints:{current_joints}, pose:{T_forward}, T_in:{T_in}")
                capture_event.clear()
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        # 结束时一次性保存所有关节角度数组
        if captured_poses:
            with open(save_dir + '/pose.txt', 'w') as f:
                for pose in captured_poses:
                    # 将每个关节角度转换为字符串并用逗号分隔
                    pose_str = ','.join(map(str, pose))
                    f.write(pose_str + '\n')
            print(f"[Camera] Saved all {len(captured_poses)} poses to pose.txt")

def arm_thread():
    try:
        # 先回 Home
        arm.labelRun('forward');    time.sleep(10)
        # 对每个 preset：移动→等待→拍照→等待
        for name, joints in presets:
            arm.labelRun(name); time.sleep(10)
            globals()['current_joints'] = joints
            print(f"[Arm] Reached {name}, triggering capture")
            capture_event.set()
            # 等待相机拍照完成
            while capture_event.is_set():
                time.sleep(0.1)
            # time.sleep(1)
        # 全部拍完后回 Home 并 backToStart
        arm.labelRun('forward');   
        arm.backToStart();        
    finally:
        # 通知相机线程安全退出
        stop_event.set()
        print("[Arm] Workflow complete, stopping.")

if __name__ == '__main__':
    t_cam = threading.Thread(target=camera_thread, daemon=True)
    t_arm = threading.Thread(target=arm_thread,   daemon=True)
    t_cam.start()
    t_arm.start()
    t_cam.join()
    t_arm.join()
    print("All done.")
