#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Eye-in-Hand (Hand-Eye) calibration script.
输出：camera2gripper.txt（4×4）、
       detected_images/ 下的检测图像（从 img_0001.png 开始）、real_poses.txt 中的真实位姿（逐行）
"""
import sys
import os
import time
import signal
import atexit

sys.path.append("../lib")
sys.path.append("/home/x86/Z1Rbot/z1_sdk/lib")

import example_http_client as robot
import realsenseD435 as D435
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import numpy.random as npr

CHECKER_SIZE   = (7, 5)
SQUARE_SIZE_M  = 0.025

# ========= 姿态与空间采样范围 =========
ROLL_RANGE   = (0.4, 0.6)           # 弧度
PITCH_RANGE  = (0.8, 1.2)
YAW_RANGE    = (-0.3, 0.3)
ORI_STEPS    = (3, 3, 3)            # roll, pitch, yaw 各方向采样点数

workspace_limits = np.asarray([[0.10, 0.20],    # x (m)
                               [-0.20, 0.20],   # y (m)
                               [0.30, 0.45]])   # z (m)
calib_grid_step  = 0.05                  # (m)


def gen_grid_tool_cmds():
    roll_vals  = np.linspace(*ROLL_RANGE,  ORI_STEPS[0])
    pitch_vals = np.linspace(*PITCH_RANGE, ORI_STEPS[1])
    yaw_vals   = np.linspace(*YAW_RANGE,   ORI_STEPS[2])
    gx = np.arange(workspace_limits[0,0], workspace_limits[0,1] + 1e-6, calib_grid_step)
    gy = np.arange(workspace_limits[1,0], workspace_limits[1,1] + 1e-6, calib_grid_step)
    gz = np.arange(workspace_limits[2,0], workspace_limits[2,1] + 1e-6, calib_grid_step)

    tool_cmds = []
    for r in roll_vals:
        for p in pitch_vals:
            for y in yaw_vals:
                for x in gx:
                    for yy in gy:
                        for z in gz:
                            tool_cmds.append([r, p, y, x, yy, z])

    npr.default_rng().shuffle(tool_cmds)
    return tool_cmds


# ========== 保存准备 ==========
os.makedirs("detected_images", exist_ok=True)
pos_file = open("real_poses.txt", "a", buffering=1)

# 用来去重，只记录已经保存过的 (x_mm, y_mm, z_mm)
saved_positions = set()

# ========== 安全退出 ==========
def move_to_safe_pose():
    try:
        print("[INFO] 程序退出，机械臂回安全位 …")
        robot.backToStart()
    except Exception as e:
        print(f"[WARN] 回零失败: {e}")

atexit.register(move_to_safe_pose)
for sig in (signal.SIGINT, signal.SIGTERM):
    signal.signal(sig, lambda s, f: sys.exit(0))

# ========== 工具函数 ==========
rng = npr.default_rng()
def rpy_to_rot(roll, pitch, yaw):
    return R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()

# ========== 相机 & 棋盘 ==========
camera = D435.RealsenseD435()
objp = (np.mgrid[0:CHECKER_SIZE[0], 0:CHECKER_SIZE[1]]
        .T.reshape(-1, 2) * SQUARE_SIZE_M).astype(np.float32)
objp = np.hstack([objp, np.zeros((objp.shape[0], 1), np.float32)])

# ========== 运动模式选择 ==========
SUCCESS_FILE = "successful_tool_cmds.txt"
def select_mode():
    return input("使用历史成功位姿？(y/n): ").strip().lower() == 'y'

def gen_success_tool_cmds():
    if not os.path.exists(SUCCESS_FILE):
        print("[ERROR] 未找到 successful_tool_cmds.txt，请先跑随机采样模式生成。")
        sys.exit(1)
    with open(SUCCESS_FILE, 'r') as f:
        lines = [ln.strip() for ln in f.readlines()[1:] if ln.strip()]
    if not lines:
        print("[ERROR] successful_tool_cmds.txt 为空！")
        sys.exit(1)
    cmds = [list(map(float, ln.split(','))) for ln in lines]
    print(f"[INFO] 读入 {len(cmds)} 条成功位姿")
    return cmds

use_success = select_mode()
tool_cmd_list = gen_success_tool_cmds() if use_success else gen_grid_tool_cmds()
print(f"Total target positions: {len(tool_cmd_list)}")

# ========== 采集与标定数据缓存 ==========
R_g2b, t_g2b = [], []
R_t2c, t_t2c = [], []
succ = 0
save_idx = 1  # 用于图片和 pose 的编号，从 1 开始

for idx, xyz in enumerate(tool_cmd_list, 1):
    r, p, y, x, yy, z = xyz
    print(f"[{idx:04d}] MoveJ → RPY=({r:+.3f},{p:+.3f},{y:+.3f})  XYZ=({x:.3f},{yy:.3f},{z:.3f})")
    robot.MoveJ(xyz, 1, 0, 1.5)
    time.sleep(1.0)

    # 棋盘检测
    color_img, _ = camera.get_data()
    gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
    ok, corners = cv2.findChessboardCorners(gray, CHECKER_SIZE, cv2.CALIB_CB_ADAPTIVE_THRESH)
    if not ok:
        print("    ✖ 未检测到棋盘")
        continue
    corners = cv2.cornerSubPix(gray, corners, (5,5), (-1,-1),
                               (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

    # solvePnP
    ok, rvec, tvec = cv2.solvePnP(objp, corners, camera.cam_intrinsics, None,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        print("    ✖ solvePnP 失败")
        continue
    R_tc = cv2.Rodrigues(rvec)[0].astype(np.float64)
    t_tc = tvec.astype(np.float64).reshape(3,1)

    # 获取真实末端位姿并保存
    try:
        pose = robot.getPose().json()['posture']  # [roll, pitch, yaw, x, y, z]

        # 重排序并单位换算：x,y,z → 毫米
        x_mm, y_mm, z_mm = pose[3], pose[4], pose[5]
        key = (int(round(x_mm*1000)), int(round(y_mm*1000)), int(round(z_mm*1000)))
        if key in saved_positions:
            print(f"    ⚠ 位置 {key} 已保存过，跳过")
            continue

        roll, pitch, yaw = pose[0], pose[1], pose[2]

        # 保存检测图像，命名从 1 开始
        img_path = os.path.join("detected_images", f"img_{save_idx:04d}.png")
        cv2.imwrite(img_path, color_img)

        # 写入 real_poses.txt：格式 x_mm,y_mm,z_mm,roll,pitch,yaw
        pos_file.write(f"{x_mm:.3f},{y_mm:.3f},{z_mm:.3f},{roll:.6f},{pitch:.6f},{yaw:.6f}\n")

        print(f"    ✔ Saved [{save_idx:04d}] pos={key}")
        saved_positions.add(key)
        save_idx += 1  # 仅在成功保存并去重后自增

        # 缓存用于手眼标定
        R_gb = rpy_to_rot(roll, pitch, yaw).astype(np.float64)
        t_gb = np.asarray([pose[3], pose[4], pose[5]], dtype=np.float64).reshape(3,1)
    except Exception as e:
        print(f"    ✖ 末端姿态获取失败: {e}")
        continue

    R_g2b.append(R_gb)
    t_g2b.append(t_gb)
    R_t2c.append(R_tc)
    t_t2c.append(t_tc)
    succ += 1

cv2.destroyAllWindows()
pos_file.close()

print(f"[INFO] 有效采样 {succ} 组")
if succ < 10:
    sys.exit("[ERROR] 有效数据不足 10 组，标定终止！")

# ========== 手眼标定 ==========
print("[INFO] 运行 cv2.calibrateHandEye() …")
R_cg, t_cg = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c,
                                  method=cv2.CALIB_HAND_EYE_TSAI)
T_cg = np.eye(4)
T_cg[:3,:3] = R_cg
T_cg[:3, 3] = t_cg.ravel()

print("相机→末端 变换 (camera2gripper):\n", T_cg)
np.savetxt("camera2gripper.txt", T_cg, fmt="%.8f")
print("[✓] 标定完成，已保存 camera2gripper.txt")
