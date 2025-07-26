#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多轴同步小幅度姿态扫描脚本（略增幅度）

功能：
1. 根据法兰末端（flange）位姿与工具偏移计算 TCP（Tip）位姿
2. 根据期望 TCP 位姿与工具偏移反算出法兰末端位姿
3. 在保持 TCP 空间位置不变的条件下，沿 roll、pitch、yaw 三轴同步扫描，幅度略增，让机械臂整体动作大一点

示例：
    python tcp_synchronized_sweep.py
"""
import time
from math import cos, sin, pi
import numpy as np
import example_http_client as robot

def rpy_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    Rx = np.array([[1, 0, 0],
                   [0,  cos(roll), -sin(roll)],
                   [0,  sin(roll),  cos(roll)]], dtype=float)
    Ry = np.array([[ cos(pitch), 0, sin(pitch)],
                   [          0, 1,          0],
                   [-sin(pitch), 0, cos(pitch)]], dtype=float)
    Rz = np.array([[cos(yaw), -sin(yaw), 0],
                   [sin(yaw),  cos(yaw), 0],
                   [       0,         0, 1]], dtype=float)
    return Rz @ Ry @ Rx

def tip_to_flange(tip_pose, offset_tool):
    roll, pitch, yaw, x_t, y_t, z_t = map(float, tip_pose)
    dx, dy, dz = map(float, offset_tool)
    delta = rpy_to_rot(roll, pitch, yaw) @ np.array([dx, dy, dz])
    x_f, y_f, z_f = np.array([x_t, y_t, z_t]) - delta
    return [roll, pitch, yaw, x_f, y_f, z_f]

if __name__ == "__main__":
    # --- 配置区域 ---
    tip_pose_init = [0.0185, 1.3895, -0.0362, 0.398365, -0.021382, 0.052578]
    offset_tool   = [0.256841, -0.002321, 0.001634]

    # 每轴同步小幅度（略增）
    amp_roll  = pi / 6   # ±6°
    amp_pitch = pi / 6   # ±6°
    amp_yaw   = pi / 7   # ±15°

    num_steps = 50        # 扫描点数
    # ------------------

    from math import sin as _sin, cos as _cos
    fixed_xyz = tip_pose_init[3:]
    base_r, base_p, base_y = tip_pose_init[:3]

    for i, t in enumerate(np.linspace(0, 2*pi, num_steps, endpoint=False), start=1):
        # 正弦/余弦组合，三个轴同步扫描
        roll  = base_r  + amp_roll  * _sin(t)
        pitch = base_p  + amp_pitch * _sin(2*t)
        yaw   = base_y  + amp_yaw   * _cos(1.5*t)

        new_tip = [roll, pitch, yaw, *fixed_xyz]
        flange  = tip_to_flange(new_tip, offset_tool)

        print(f"[{i}/{num_steps}] Δroll={roll-base_r:+.3f}, Δpitch={pitch-base_p:+.3f}, Δyaw={yaw-base_y:+.3f}")
        robot.MoveJ(flange, 1.0, 0.0, 0.6)
        time.sleep(0.5)
    robot.backToStart()
    print("同步小幅度扫描完成，TCP 位置始终保持不变。")