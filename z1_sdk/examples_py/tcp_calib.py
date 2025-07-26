#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TCP 球点（pivot）标定脚本

功能：
1. 使用直接在脚本中定义的多组机器人法兰（flange）位姿数据（roll, pitch, yaw, x, y, z）
2. 利用最小二乘方法求解 TCP 相对于法兰的偏移向量 d 和空间固定点 p
3. 打印并保存标定结果

用法：
    python tcp_pivot_calibration.py
"""

import numpy as np
from math import cos, sin
import os


def rpy_to_rot(roll, pitch, yaw):
    """从 roll‑pitch‑yaw 生成旋转矩阵 (右手坐标系, XYZ 欧拉角)."""
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


def pivot_calibration(flange_poses):
    """执行 pivot 标定，返回 d_est, p_est, residual"""
    N = len(flange_poses)
    if N < 6:
        raise ValueError("数据不足，至少需要 6 组位姿")

    A = np.zeros((3*N, 6))
    b = np.zeros((3*N,))

    for i, (r, p, y, x_f, y_f, z_f) in enumerate(flange_poses):
        R = rpy_to_rot(r, p, y)
        A[3*i:3*i+3, 0:3] = R
        A[3*i:3*i+3, 3:6] = -np.eye(3)
        b[3*i:3*i+3] = -np.array([x_f, y_f, z_f])

    x_est, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
    d_est = x_est[:3]
    p_est = x_est[3:]
    return d_est, p_est, residuals


def save_results(d_est, p_est, residuals, output_path="calibration_result.txt"):
    """保存标定结果到文本文件"""
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write("TCP Pivot Calibration Result\n")
        f.write(f"Offset d (m): {d_est.tolist()}\n")
        f.write(f"Pivot point p (m): {p_est.tolist()}\n")
        f.write(f"Residuals: {residuals}\n")
    print(f"结果已保存至 {output_path}")


if __name__ == "__main__":
    # ----- 用户直接在此定义法兰位姿列表 -----
    # 每组格式: [roll, pitch, yaw, x, y, z]
    flange_poses = [
        [-0.00023661171794545332, 1.3898012363238201, -0.05319115162357115, 0.351248690212424, -0.018886082919167574, 0.3053712346890942],
        [-1.79864573912718, 1.129698092042875, -0.8051439057058322, 0.3215402906275836, 0.05959611800486651, 0.28088500979085534],
        [-0.9422567578617305, 0.8227760966901843, -0.5425984438815176, 0.24744820474605297, 0.06986095785966581, 0.23856406175899952],
        [1.1803089006402834, 0.5995211045427066, 0.4684024250222016, 0.20996219885012052, -0.112206481380838, 0.19957584750470997],
        [-0.1636134631336078, 0.8620059189650546, 1.0259613613859635, 0.3128105098497125, -0.16553855272942675, 0.2513392608050228],
        [-2.0033940574041793, 0.9206788372321152, -1.081341903994654, 0.3222115420012227, 0.11373954280246087, 0.2555986142730515],
        [-1.0329206109343314, 0.5619888996452395, -0.46613323033351034, 0.20264219840830666, 0.07947848054306039, 0.18496684742579428],
        [1.2132851785356664, 0.6860035813047024, 0.028575125851439214, 0.1989444178941704, -0.02384116773977313, 0.21562028068109826],#一般
        [-1.2334164165741566, 1.3263912716527195, -1.0814311076254313, 0.3651772737436153, 0.03410121839425359, 0.30421256938804364],
        [-1.9614836210755031, 0.7078017921179703, -1.0597628427867507, 0.29700801286451733, 0.14710063785075295, 0.21602010093111146]
    ]
    # ------------------------------------------

    print(f"使用 {len(flange_poses)} 组法兰位姿进行标定。")
    try:
        d_est, p_est, residuals = pivot_calibration(flange_poses)
        print("标定完成。结果如下:")
        print(f"  TCP 偏移 d = [{d_est[0]:.6f}, {d_est[1]:.6f}, {d_est[2]:.6f}] m")
        print(f"  空间点 p = [{p_est[0]:.6f}, {p_est[1]:.6f}, {p_est[2]:.6f}] m")
        if residuals.size > 0:
            print(f"  残差平方和 = {residuals[0]:.6e}")
        save_results(d_est, p_est, residuals)
    except Exception as e:
        print(f"标定失败: {e}")
