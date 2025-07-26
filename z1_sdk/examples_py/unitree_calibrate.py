import example_http_client as robot
import realsenseD435 as D435
import numpy as np
import time
import signal
import atexit
import sys
import cv2

# 安全退出动作
def move_to_safe_pose():
    """程序关闭前最后一次调用的动作"""
    try:
        print("[INFO] 程序退出，机械臂回安全位 …")
        robot.backToStart()
    except Exception as e:
        print(f"[WARN] 安全回位失败: {e}")

atexit.register(move_to_safe_pose)
for sig in (signal.SIGINT, signal.SIGTERM):
    signal.signal(sig, lambda s, f: sys.exit(0))

# 相机初始化
camera = D435.RealsenseD435()

# 手眼标定列表
R_gripper2base_list = []
t_gripper2base_list = []
R_target2cam_list = []
t_target2cam_list = []

# 定义棋盘尺寸和世界坐标
checkerboard_size = (5, 5)
square_size = 0.025  # 棋盘方格边长，单位米
objp = np.zeros((checkerboard_size[0]*checkerboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
objp *= square_size

# ------------ 使用 workspace_limits 和 calib_grid_step 生成位姿 ------------
workspace_limits = np.asarray([[0.2, 0.3],  # x 范围
                               [-0.15, 0.15],  # y 范围
                               [0.3, 0.5]])   # z 范围
calib_grid_step = 0.05  # 各轴步长

grid_x = np.arange(workspace_limits[0,0], workspace_limits[0,1] + 1e-6, calib_grid_step)
grid_y = np.arange(workspace_limits[1,0], workspace_limits[1,1] + 1e-6, calib_grid_step)
grid_z = np.arange(workspace_limits[2,0], workspace_limits[2,1] + 1e-6, calib_grid_step)

# 生成所有姿态列表 (row, pitch, yaw 固定为 0)
poses = []
for x in grid_x:
    for y in grid_y:
        for z in grid_z:
            poses.append([0, 0, 0, x, y, z])
print(f"Total calibration poses: {len(poses)}")

# 主循环: 机械臂移动并采集数据
for idx, pose in enumerate(poses):
    print(f"Pose {idx}: MoveJ to {pose}")
    robot.MoveJ(pose, speed=1, acceleration=0, time=0.5)
    time.sleep(2)

    # 获取当前手臂末端位姿
    T = robot.get_current_pose()  # 返回 4x4 数组
    R_gp = np.array(T[:3, :3])
    t_gp = np.array(T[:3, 3], dtype=np.float32).reshape(3, 1)
    R_gripper2base_list.append(R_gp)
    t_gripper2base_list.append(t_gp)

    # 获取图像并检测棋盘
    color_img, depth_img = camera.get_data()
    gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, checkerboard_size)
    if not found:
        print(f"Pose {idx}: 未检测到棋盘，跳过")
        continue
    corners_refined = cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    )

    # solvePnP 获取棋盘在相机下的位姿
    ret, rvec, tvec = cv2.solvePnP(
        objp, corners_refined,
        np.array(camera.cam_intrinsics),
        np.array(camera.dist_coeffs)
    )
    if not ret:
        print(f"Pose {idx}: solvePnP 失败，跳过")
        continue
    R_tc, _ = cv2.Rodrigues(rvec)
    t_tc = tvec.reshape(3, 1)
    R_target2cam_list.append(R_tc)
    t_target2cam_list.append(t_tc)

# 将列表转换为 numpy 数组
R_gripper2base_list = np.array(R_gripper2base_list)
t_gripper2base_list = np.array(t_gripper2base_list)
R_target2cam_list = np.array(R_target2cam_list)
t_target2cam_list = np.array(t_target2cam_list)

# 调用 OpenCV 手眼标定
R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
    R_gripper2base_list, t_gripper2base_list,
    R_target2cam_list, t_target2cam_list,
    method=cv2.CALIB_HAND_EYE_TSAI
)

# 保存标定结果
np.savetxt('R_cam2gripper.txt', R_cam2gripper)
np.savetxt('t_cam2gripper.txt', t_cam2gripper)
print("手眼标定完成，结果已保存。")
