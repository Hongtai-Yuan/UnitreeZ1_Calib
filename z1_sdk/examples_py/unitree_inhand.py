#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Eye‑in‑Hand 手眼标定（ChArUco 9×6）— 朝向自适应版
--------------------------------------------------
与原脚本相比，**姿态不再使用固定 RPY 网格**，而是：

1. 在工作空间内采样 (x, y, z)；
2. 根据“工具→目标点”矢量实时计算 (roll, pitch, yaw)，确保相机始终指向目标；
3. 给 yaw / pitch 叠加 3° 高斯抖动，并对 roll 做均匀随机，保证样本多样同时不会拍不到棋盘。

其它改进（亚像素角点、多帧质量评估、角点平均）保持不变。
"""

import sys, os, time, signal, atexit, collections
import numpy as np, cv2, numpy.random as npr
from scipy.spatial.transform import Rotation as R

import example_http_client as robot
import realsenseD435 as D435

# ========= 棋盘参数 =========
SQUARE_SIZE_M  = 0.025
MARKER_SIZE_M  = 0.018
ARUCO_DICT     = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
CHARUCO_BOARD  = cv2.aruco.CharucoBoard_create(
    squaresX=9, squaresY=6,
    squareLength=SQUARE_SIZE_M,
    markerLength=MARKER_SIZE_M,
    dictionary=ARUCO_DICT
)

# ========= 优化相关超参数 =========
MULTI_ATTEMPTS       = 8   # 每次检测后再额外抓取多少帧
BEST_K               = 3   # 取质量最好的 K 帧做平均
MIN_GOOD_CORNERS     = 40  # 有效帧的最少 ChArUco 角点
SUBPIX_WIN           = (5, 5)
SUBPIX_ZERO          = (-1, -1)
SUBPIX_CRITERIA      = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# ========= 姿态扫描参数（新的自适应算法） =========
TARGET_POINT = np.array([0.7, 0.00, 0.25])      # 相机要对准的工作点（自行测量！单位 m）
ROLL_RANGE   = (-0.35, 0.35)                      # 允许的滚转范围 (rad)
JITTER_STD   = np.deg2rad(3)                      # yaw/pitch 3° 高斯抖动

workspace_limits = np.asarray([[0.25, 0.4],       # x y z 立方工作空间 (m)
                               [-0.25, 0.25],
                               [0.00, 0.45]])
calib_grid_step  = 0.025                          # 位置采样分辨率 (m)

SUCCESS_FILE = "successful_tool_cmds.txt"
HEADER       = "roll,pitch,yaw,x,y,z\n"

# ========= 工具函数 =========

def rpy_to_rot(r, p, y):
    return R.from_euler('xyz', [r, p, y]).as_matrix()


def pose_diff(p1, p2):
    return np.linalg.norm(np.array(p1[3:]) - np.array(p2[3:]))


def get_stable_pose(robot, samples=10, tol=1e-3, delay=0.01):
    """连续读取姿态直至稳定，用于过滤抖动。"""
    time.sleep(1)
    poses = np.zeros((samples, 6))
    for i in range(samples):
        poses[i, :] = np.asarray(robot.getPose().json()['posture'], float)
        time.sleep(delay)
    if np.all((poses.max(0) - poses.min(0)) < tol):
        return poses.mean(0)
    raise RuntimeError("Pose unstable")

# ---------- 新增：根据位置实时计算姿态 ----------

def look_at_rpy(tool_xyz, target_xyz, rng=None):
    """返回让相机 +Z 轴指向 target_xyz 的 (roll, pitch, yaw)。"""
    if rng is None:
        rng = npr.default_rng()

    dx, dy, dz = (target_xyz - tool_xyz)
    yaw   = np.arctan2(dy, dx)                    # 平面内对准目标
    dist_xy = np.hypot(dx, dy)
    pitch = -np.arctan2(dz, dist_xy)              # 抬/俯视目标（向下为负）

    # 随机扰动
    yaw   += rng.normal(0, JITTER_STD)
    pitch += rng.normal(0, JITTER_STD)
    roll   = rng.uniform(*ROLL_RANGE)
    return roll, pitch, yaw

# ========= 新增：亚像素 ChArUco 多帧检测 =========

def _refine_aruco_subpix(gray, corners):
    for c in corners:
        cv2.cornerSubPix(gray, c, SUBPIX_WIN, SUBPIX_ZERO, SUBPIX_CRITERIA)


def detect_charuco_multiframe(camera):
    """获取 quality 最好的 BEST_K 帧并做角点平均。"""
    candidates = []
    for _ in range(MULTI_ATTEMPTS):
        color_img, _ = camera.get_data()
        gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, ARUCO_DICT)
        if ids is None or len(ids) == 0:
            continue
        _refine_aruco_subpix(gray, corners)
        retval, ch_c, ch_id = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, CHARUCO_BOARD)
        if retval < MIN_GOOD_CORNERS:
            continue
        candidates.append({
            "score": retval,
            "corners": ch_c.copy(),
            "ids": ch_id.copy(),
            "img": color_img,
        })

    if len(candidates) < BEST_K:
        return None, None, None

    candidates.sort(key=lambda d: -d["score"])
    top = candidates[:BEST_K]

    id_map = collections.defaultdict(list)
    for item in top:
        for pt, cid in zip(item["corners"], item["ids"].flatten()):
            id_map[int(cid)].append(pt[0])

    common_ids = [cid for cid, lst in id_map.items() if len(lst) == BEST_K]
    if len(common_ids) < MIN_GOOD_CORNERS:
        return None, None, None

    common_ids.sort()
    avg_corners = np.zeros((len(common_ids), 1, 2), np.float32)
    avg_ids     = np.zeros((len(common_ids), 1), np.int32)
    for i, cid in enumerate(common_ids):
        avg_corners[i, 0, :] = np.mean(id_map[cid], axis=0)
        avg_ids[i, 0] = cid

    vis_img = top[0]["img"].copy()
    return avg_corners, avg_ids, vis_img

# ========= 生成扫描姿态 =========

def gen_grid_tool_cmds():
    """遍历 xyz 网格，根据每个点实时生成看向 TARGET_POINT 的 RPY。"""
    rng = npr.default_rng()
    gx = np.arange(workspace_limits[0,0], workspace_limits[0,1] + 1e-6, calib_grid_step)
    gy = np.arange(workspace_limits[1,0], workspace_limits[1,1] + 1e-6, calib_grid_step)
    gz = np.arange(workspace_limits[2,0], workspace_limits[2,1] + 1e-6, calib_grid_step)

    cmds = []
    for x in gx:
        for y in gy:
            for z in gz:
                r, p, yaw = look_at_rpy(np.array([x, y, z]), TARGET_POINT, rng)
                cmds.append([r, p, yaw, x, y, z])

    rng.shuffle(cmds)
    return cmds


def gen_success_tool_cmds():
    if not os.path.exists(SUCCESS_FILE):
        print("[ERROR] 未找到 successful_tool_cmds.txt，请先跑随机采样模式生成。")
        sys.exit(1)
    with open(SUCCESS_FILE) as f:
        cmds = [list(map(float, ln.split(','))) for ln in f.readlines()[1:] if ln.strip()]
    print(f"[INFO] 读入 {len(cmds)} 条成功位姿")
    return cmds

# ========= 主流程 =========

camera = D435.RealsenseD435()
R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []

use_success = input("使用历史成功位姿？(y/n): ").strip().lower() == 'y'
cmd_list = gen_success_tool_cmds() if use_success else gen_grid_tool_cmds()

processed_cmds, existing_success = set(), set()
if os.path.exists(SUCCESS_FILE):
    with open(SUCCESS_FILE) as f:
        existing_success.update(tuple(map(float, ln.split(','))) for ln in f.readlines()[1:] if ln.strip())
if not use_success and not os.path.exists(SUCCESS_FILE):
    open(SUCCESS_FILE, 'w').write(HEADER)
print(f"Total target positions: {len(cmd_list)}")

succ = 0
for idx, cmd in enumerate(cmd_list, 1):
    # 读取移动前姿态
    try:
        prev = get_stable_pose(robot)
    except Exception:
        print(f"[{idx:04d}] 无法读取初始姿态，跳过")
        continue

    print(f"[{idx:04d}] MoveJ → RPY=({cmd[0]:+.3f},{cmd[1]:+.3f},{cmd[2]:+.3f}) XYZ=({cmd[3]:.3f},{cmd[4]:.3f},{cmd[5]:.3f})")
    robot.MoveJ(cmd, 1, 0, 1.5)
    time.sleep(1.0)

    # 读取移动后姿态
    try:
        new = get_stable_pose(robot)
    except Exception:
        print("    ✖ 无法读取后置姿态，跳过")
        continue
    if pose_diff(prev, new) < 0.01:
        print("    ⚠ 机械臂未移动，跳过此组数据")
        continue

    # ---------- 角点检测 ----------
    ch_c, ch_id, vis = detect_charuco_multiframe(camera)
    if ch_c is None:
        print("    ✖ 角点质量不足或数量不足")
        continue

    ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
        ch_c, ch_id, CHARUCO_BOARD, camera.cam_intrinsics, camera.dist_coeffs, None, None)
    if not ok:
        print("    ✖ 位姿估计失败")
        continue

    tcmd = tuple(cmd)
    if tcmd in processed_cmds:
        print("    ℹ️ 此位姿已标定过，跳过")
        continue
    processed_cmds.add(tcmd)

    # 末端真值收集
    try:
        pt = get_stable_pose(robot)
        R_gb = rpy_to_rot(*pt[:3]); t_gb = np.asarray(pt[3:], float).reshape(3,1)
    except Exception:
        print("    ✖ 末端姿态获取失败")
        continue

    R_g2b.append(R_gb); t_g2b.append(t_gb)
    R_t2c.append(cv2.Rodrigues(rvec)[0]); t_t2c.append(tvec.reshape(3,1))
    succ += 1

    # 保存历史成功位姿
    if not use_success and tcmd not in existing_success:
        with open(SUCCESS_FILE, 'a') as f:
            f.write(','.join(f"{v:.6f}" for v in cmd) + "\n")
        existing_success.add(tcmd)

    # 可视化
    cv2.aruco.drawDetectedCornersCharuco(vis, ch_c, ch_id, (0,255,0))
    cv2.aruco.drawAxis(vis, camera.cam_intrinsics, camera.dist_coeffs, rvec, tvec, 0.05)
    cv2.imshow("Calibration", vis)
    cv2.waitKey(50)

cv2.destroyAllWindows()
print(f"[INFO] 有效采样 {succ} 组")
if succ < 10:
    sys.exit("[ERROR] 有效数据不足 10 组，标定终止！")

# ========= 手眼求解 =========
print("[INFO] 运行 cv2.calibrateHandEye() …")
R_cg, t_cg = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=cv2.CALIB_HAND_EYE_TSAI)
T_cg = np.eye(4); T_cg[:3,:3] = R_cg; T_cg[:3,3] = t_cg.ravel()
np.savetxt("camera2gripper.txt", T_cg, fmt="%.8f")
print("相机→末端 变换 (camera2gripper):\n", T_cg)
print("[✓] 标定完成，已保存 camera2gripper.txt")
