import cv2
import numpy as np
import transforms3d
import glob
import os

# 图像质量检测函数
def assess_image_quality(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
    return laplacian_var

# 将机械臂末端的姿态向量转换为旋转矩阵和位移向量
def pose_vectors_to_end2base_transforms(pose_vectors):
    R_end2bases = []
    t_end2bases = []
    for pose_vector in pose_vectors:
        R_end2base = euler_to_rotation_matrix(pose_vector[3], pose_vector[4], pose_vector[5])
        t_end2base = pose_vector[:3]  # 位移向量
        R_end2bases.append(R_end2base)
        t_end2bases.append(t_end2base)
    return R_end2bases, t_end2bases

# 欧拉角转换为旋转矩阵
def euler_to_rotation_matrix(rx, ry, rz, unit='deg'):
    if unit == 'deg':
        rx, ry, rz = np.radians([rx, ry, rz])
    Rx = transforms3d.axangles.axangle2mat([1, 0, 0], rx)
    Ry = transforms3d.axangles.axangle2mat([0, 1, 0], ry)
    Rz = transforms3d.axangles.axangle2mat([0, 0, 1], rz)
    return np.dot(Rz, np.dot(Ry, Rx))

# 输入部分
file_path = 'real_poses.txt'
if not os.path.exists(file_path):
    raise FileNotFoundError(f"文件未找到: {file_path}")

# 按行读取 txt，每行逗号分隔成一个列表
pose_vectors_list = []
with open(file_path, 'r', encoding='utf-8') as f:
    for lineno, line in enumerate(f, 1):
        line = line.strip()
        if not line:
            continue  # 跳过空行
        parts = line.split(',')
        if len(parts) < 6:
            raise ValueError(f"第 {lineno} 行格式错误，期望至少 6 个用逗号分隔的数值: '{line}'")
        # 转为 float
        nums = [float(x) for x in parts[:6]]
        pose_vectors_list.append(nums)
pose_vectors = np.array(pose_vectors_list, dtype=np.float64)

# 后续保持不变
square_size = 0.025  # 棋盘格每格大小（mm）
pattern_size = (7, 5)
images = glob.glob('detected_images/*.png')

obj_points = []
img_points = []
objp = np.zeros((np.prod(pattern_size), 3), dtype=np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size

det_success_num = 0
filtered_pose_vectors = []
low_quality_images = []

for i, image in enumerate(images):
    img = cv2.imread(image)
    if img is None:
        print(f"无法读取图像: {image}")
        continue

    if i == 0:
        img_height, img_width = img.shape[:2]

    quality_score = assess_image_quality(img)
    quality_threshold = 80
    print(f"图像 {image} 的质量分数: {quality_score}")

    if quality_score < quality_threshold:
        low_quality_images.append(image)
        cv2.putText(img, "Low Quality", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow('Low Quality Image', img)
        cv2.waitKey(1000)
    else:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, pattern_size)
        if ret:
            det_success_num += 1
            obj_points.append(objp)
            img_points.append(corners)
            filtered_pose_vectors.append(pose_vectors[i])
            cv2.drawChessboardCorners(img, pattern_size, corners, ret)
            cv2.imshow('img', img)
            cv2.waitKey(500)

cv2.destroyAllWindows()

pose_vectors = np.array(filtered_pose_vectors)
if det_success_num == 0:
    raise RuntimeError("没有检测到任何有效的棋盘格角点，无法进行标定。")

image_size = (img_width, img_height)
ret, K, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    obj_points, img_points, image_size, None, None
)
print("相机内参矩阵 (K):\n", K)
print("畸变系数:\n", dist_coeffs)

R_board2cameras, t_board2cameras = [], []
for i in range(det_success_num):
    ret, rvec, t_board2camera = cv2.solvePnP(obj_points[i], img_points[i], K, dist_coeffs)
    Rb2c, _ = cv2.Rodrigues(rvec)
    R_board2cameras.append(Rb2c)
    t_board2cameras.append(t_board2camera)
    print(f"图像 {i+1} 的 R_board2camera:\n{Rb2c}")
    print(f"图像 {i+1} 的 t_board2camera (mm):\n{t_board2camera}")

R_end2bases, t_end2bases = pose_vectors_to_end2base_transforms(pose_vectors)
R_camera2end, t_camera2end = cv2.calibrateHandEye(
    R_end2bases, t_end2bases, R_board2cameras, t_board2cameras,
    method=cv2.CALIB_HAND_EYE_TSAI
)
T_camera2end = np.eye(4)
T_camera2end[:3, :3] = R_camera2end
T_camera2end[:3, 3] = t_camera2end.reshape(3)
np.set_printoptions(suppress=True, precision=10)
print("T_camera2end (齐次矩阵):\n", T_camera2end)



# 点位验证
robot_pos = [[525.470, 38.334, 280.296, 2.968, 176.830, 87.019]]
R_end2bases, t_end2bases = pose_vectors_to_end2base_transforms(robot_pos)
RT_gripper2base1 = np.column_stack((np.array(R_end2bases[0]), np.array(t_end2bases).T))
RT_gripper2base1 = np.row_stack((RT_gripper2base1, np.array([0, 0, 0, 1])))
print(RT_gripper2base1 @ T_camera2end @ np.array([[13.4], [-78.0], [451.7], [1]]))
print(np.array([13.4, -78.0, 451.7, 1] @ T_camera2end.T @ RT_gripper2base1.T))

# 反算标定板在机械臂末端下的位姿，并计算机械臂基座下的标定板位姿
for i in range(det_success_num):
    # 将标定板在相机坐标系下的位姿转换到机械臂末端坐标系
    T_board2camera = np.eye(4)
    T_board2camera[:3, :3] = R_board2cameras[i]
    T_board2camera[:3, 3] = t_board2cameras[i].reshape(3)

    T_board2end = np.dot(T_camera2end, T_board2camera)

    # 计算标定板在机械臂基座下的位姿
    T_end2base = np.eye(4)
    T_end2base[:3, :3] = R_end2bases[i]
    T_end2base[:3, 3] = t_end2bases[i]

    T_board2base = np.dot(T_end2base, T_board2end)

    print(f"图像 {i + 1} 中的标定板在基座坐标系下的齐次变换矩阵 (T_board2base):")
    print(T_board2base)

# 输出质量较差的图像
if low_quality_images:
    print("质量较差的图像:")
    for low_quality_image in low_quality_images:
        print(low_quality_image)
else:
    print("所有图像质量良好。")