#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Standalone Vision 检测脚本 (Realsense D435)
• 检测 AprilTag(tag36h11)
• apriltag.detection_pose → 相机→Tag 姿态
• 绘制 Tag 坐标轴
• 输出 Tag 在基座系下的坐标 (Xb,Yb,Zb)
按 Ctrl+C 或 q 退出。任何异常或中断都会先执行 backToStart()。
"""
from motor_controller import MotorManager, load_config   # 仅在子进程导入
import sys
import time
import signal
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import apriltag
import realsenseD435 as D435
import example_http_client as robot
import os

sys.path.append("../lib")
sys.path.append("/home/x86/Z1Rbot/z1_sdk/lib")
# os.system("gnome-terminal -- bash -c 'cd /home/x86/Z1Rbot/z1_controller/build && ./z1_ctrl'")
# os.system("gnome-terminal -- bash -c 'cd /home/x86/Z1Rbot/z1_sdk/examples_py && python3 /home/x86/Z1Rbot/z1_sdk/examples_py/example_http_service.py'")
config = load_config()
motor  = MotorManager(config)
print("[Gripper] 准备就绪")
tag = 'first_step'
# ---------------- 可调参数 ----------------
TAG_FAMILY   = 'tag36h11'
TAG_SIZE_M   = 0.05
AXIS_LEN     = TAG_SIZE_M * 0.75
N_SAMPLES    = 100
THRESH_RPY   = 0.08   # radian 阈值
THRESH_XYZ   = 0.03   # 米 阈值
# -----------------------------------------

# T_cg = np.array([
#     [-0.01403472,  0.01473964,  0.99979286,  0.03035240],
#     [-0.99979204,  0.01458845, -0.01424978,  0.02932516],
#     [-0.01479547, -0.99978494,  0.01453183,  0.09955026],
#     [0, 0, 0, 1.0]
# ], dtype=np.float64)
T_cg = np.array([
    [-0.00613444 ,  0.02743463,  0.99960478,   0.04132850],
    [-0.99932147,   0.03613644,  -0.00712449,   0.02278375],
    [-0.03631761,  -0.99897022,   0.02719434,   0.09926429],
    [0, 0, 0, 1.0]
], dtype=np.float64)

import numpy as np

# 计算位置分量欧氏距离
def pose_diff(p1, p2):
    dp = np.array(p1[3:]) - np.array(p2[3:])
    return np.linalg.norm(dp)

def rpy_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    从 roll（绕 X 轴）, pitch（绕 Y 轴）, yaw（绕 Z 轴）生成旋转矩阵 R = Rz * Ry * Rx
    roll, pitch, yaw 单位为弧度
    返回 3x3 numpy.ndarray
    """
    # 绕 X 轴的旋转
    Rx = np.array([
        [1,           0,            0],
        [0,  np.cos(roll), -np.sin(roll)],
        [0,  np.sin(roll),  np.cos(roll)]
    ])
    # 绕 Y 轴的旋转
    Ry = np.array([
        [ np.cos(pitch), 0, np.sin(pitch)],
        [             0, 1,            0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    # 绕 Z 轴的旋转
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [          0,            0, 1]
    ])
    # 注意乘法顺序：先 Rx，再 Ry，再 Rz
    return Rz @ Ry @ Rx


def backToStart():
    """重置机器人到初始状态。"""
    try:
        robot.backToStart()
        print("[Vision] 已执行 backToStart()")
    except Exception as e:
        print(f"[Vision] backToStart() 调用失败：{e!r}")

def build_homogeneous(rpy_xyz):
    r, p, y, tx, ty, tz = rpy_xyz
    T = np.eye(4)
    T[:3, :3] = R.from_euler('xyz', [r, p, y]).as_matrix()
    T[:3, 3]  = [tx, ty, tz]
    return T

def main_loop():
    cam = D435.RealsenseD435()
    K, dist = cam.cam_intrinsics, cam.dist_coeffs
    fx, fy, cx, cy = K[0,0], K[1,1], K[0,2], K[1,2]
    cam_params = (fx, fy, cx, cy)
    detector = apriltag.Detector(apriltag.DetectorOptions(families=TAG_FAMILY))
    history = []

    print("[Vision] 开始检测…  按 Ctrl+C 或 q 退出")

    flag = 1
    tag = 'first_step'

    while True:
        # robot.labelRun("zuhui")
        if tag == 'first_step' and flag:
            print("进入位置1")
            robot.labelRun("zuhui")
            flag = 0
        elif tag == 'second_step' and flag:
            print("进入位置2")
            robot.labelRun("zuhui2")
            flag = 0
        color_img, _ = cam.get_data()
        gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
        vis = color_img.copy()

        detections = detector.detect(gray)
        best = max(detections,
                   key=lambda d: cv2.contourArea(d.corners.astype(np.float32)),
                   default=None)
        if best is None:
            cv2.imshow("RGB with AprilTag", vis)
            if cv2.waitKey(1) in (ord('q'), ord('Q')):
                break
            continue

        # —— 计算 Tag 位姿 —— #
        pose_mat, *_ = detector.detection_pose(best, cam_params, TAG_SIZE_M)
        R_tc = pose_mat[:3, :3]
        t_tc = pose_mat[:3, 3]
        rx, ry, rz = R.from_matrix(R_tc).as_euler('xyz', degrees=False)

        # —— 转换到基座系 —— #
        p_c = np.append(t_tc, 1.0)
        p_g = T_cg @ p_c
        pose_bg = robot.getPose().json()['posture']
        T_bg = build_homogeneous(pose_bg)
        p_b = T_bg @ p_g
        Xb, Yb, Zb = p_b[:3]

        Roll  = pose_bg[0] + rz
        Pitch = pose_bg[1] - rx
        Yaw   = pose_bg[2] - ry

        current = [Roll, Pitch, Yaw, Xb, Yb, Zb]
        history.append(current)
        if len(history) > N_SAMPLES:
            history.pop(0)

        print(f"[Vision] 基座系坐标: Xb={Xb:.4f}, Yb={Yb:.4f}, Zb={Zb:.4f}, "
              f"Roll={Roll:.4f}, Pitch={Pitch:.4f}, Yaw={Yaw:.4f}")

        # —— 判断姿态稳定并 MoveJ —— #
        if len(history) == N_SAMPLES:
            arr = np.array(history)
            ranges = arr.max(axis=0) - arr.min(axis=0)
            if (ranges[0:3] < THRESH_RPY).all() and (ranges[3:6] < THRESH_XYZ).all():
                avg = arr.mean(axis=0).tolist()
                print(f"[Vision] 姿态稳定，调用 MoveJ: {avg}")

                # 1. 拆出平均姿态
                roll, pitch, yaw = avg[0], avg[1], avg[2]
                x, y, z = avg[3], avg[4], avg[5]

                # 2. 计算工具偏移在基坐标系下的分量
                R_gb = rpy_to_rot(roll, pitch, yaw).astype(np.float64)
                # offset = 0.0  # 工具延长臂长度，米
                offset_tool   = [0.18, 0.0, 0.0]  # 工具偏移量，米
                delta = (R_gb @ np.array([[offset_tool[0]], [offset_tool[1]], [offset_tool[2]]], dtype=np.float64)).flatten()
                
                # 3. 从平均位置中减去这部分偏移，得到“工具尖端”真正要到达的位置
                x_target = x - delta[0]
                y_target = y - delta[1]
                z_target = z - delta[2]

                # 4. 重新组装 MoveJ 的目标
                target = [roll, pitch, yaw, x_target, y_target, z_target]

                # 5. 执行拿取动作
                motor.open_gripper()
                time.sleep(1)
                motor.open_gripper()
                time.sleep(4)
                pt1 = robot.getPose().json()['posture']
                robot.MoveJ(target, 1, 0, 0.3)
                # flag = 1
                pt2 = robot.getPose().json()['posture']
                if pose_diff(pt1,pt2) < 0.01:
                    print("⚠ 该位置无运动学逆解")
                    continue
                motor.close_gripper()
                time.sleep(5)
                robot.labelRun("zuhui")
                # if tag == 'first_step':
                #     robot.labelRun("zuhui1")
                #     tag = 'second_step'
                #     flag = 1
                # else:
                #     robot.labelRun("zuhui3")
                #     tag = 'first_step'
                #     flag = 1
                # motor.open_gripper()
                robot.labelRun("zuhui3")
                time.sleep(3)
                motor.open_gripper()
                time.sleep(1)
                motor.open_gripper()
                time.sleep(3)
                robot.labelRun("zuhui")
                # # robot.backToStart()
                # time.sleep(5)

                # 6. 清空历史，继续下一轮
                history.clear()


        # —— 可视化 —— #
        rvec, _ = cv2.Rodrigues(R_tc)
        tvec = t_tc.reshape(3,1)
        axis = np.float32([[0,0,0],[AXIS_LEN,0,0],[0,AXIS_LEN,0],[0,0,-AXIS_LEN]])
        imgpts, _ = cv2.projectPoints(axis, rvec, tvec, K, dist)
        pts = imgpts.reshape(-1,2).astype(int)
        cv2.line(vis, tuple(pts[0]), tuple(pts[1]), (0,0,255), 2)
        cv2.line(vis, tuple(pts[0]), tuple(pts[2]), (0,255,0), 2)
        cv2.line(vis, tuple(pts[0]), tuple(pts[3]), (255,0,0), 2)

        u, v = best.center.astype(int)
        cv2.circle(vis, (u,v), 4, (0,255,255), -1)
        now_txt = f"NOW: {pose_bg[0]:+.3f},{pose_bg[1]:+.3f},{pose_bg[2]:+.3f}," \
                  f"{pose_bg[3]:.3f},{pose_bg[4]:.3f},{pose_bg[5]:.3f}"
        cv2.putText(vis, now_txt, (20,30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 2)
        if 'avg' in locals():
            exp_txt = f"EXP: {avg[0]:+.3f},{avg[1]:+.3f},{avg[2]:+.3f}," \
                      f"{avg[3]:.3f},{avg[4]:.3f},{avg[5]:.3f}"
            cv2.putText(vis, exp_txt, (20,70), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 2)
        cv2.putText(vis, f"B:({Xb:.3f},{Yb:.3f},{Zb:.3f})", (u+10,v-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

        cv2.imshow("RGB with AprilTag", vis)
        if cv2.waitKey(1) in (ord('q'), ord('Q')):
            break

def sigint_handler(signum, frame):
    print("\n[Vision] 捕获 SIGINT (Ctrl+C)，执行 backToStart() 并退出…")
    backToStart()
    cv2.destroyAllWindows()
    sys.exit(0)

def main():
    # 注册 Ctrl+C 信号处理
    signal.signal(signal.SIGINT, sigint_handler)

    try:
        main_loop()
    except Exception as e:
        print(f"[Vision] 发生未捕获异常：{e!r}，执行 backToStart() 并重启…")
        backToStart()
        time.sleep(1)
        main()  # 重启整个流程
    finally:
        cv2.destroyAllWindows()
        print("[Vision] 程序结束")

if __name__ == "__main__":
    main()
