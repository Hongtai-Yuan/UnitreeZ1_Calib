import pyrealsense2 as rs
import numpy as np
import cv2
from collections import deque
import sys
sys.path.append("../lib")
sys.path.append("/home/x86/Z1Rbot/z1_sdk/lib")
sys.path.append("/home/x86/Z1Rbot/z1_sdk/lib")
import unitree_arm_interface
import time
import numpy as np
import os

print("Press ctrl+\ to quit process.")

os.system("gnome-terminal -- bash -c 'cd /home/x86/Z1Rbot/z1_controller/build && ./z1_ctrl'")
np.set_printoptions(precision=3, suppress=True)
arm =  unitree_arm_interface.ArmInterface(hasGripper=True)
armState = unitree_arm_interface.ArmFSMState
def main():
    # —— 1. 内参和畸变系数硬编码 —— #
    K = np.array([
        [607.54630191,   0.0,               298.03414178],
        [0.0,                 602.33843031, 260.54716074],
        [0.0,                 0.0,               1.0]
    ], dtype=np.float64)

    dist = np.array([
        0.0205847,
        1.04277836,
        0.00911131,
        -0.02023654,
        -4.46459504
    ], dtype=np.float64)

    # —— 2. 外参 T_cb: 从相机系到基座(World)系的齐次变换 —— #

# [
# [np.float64(-0.05830301314928257)],
# [np.float64(0.030454334005602757)],
# [np.float64(0.05851310890616507)]
# ]





    T_cb = np.array([
        [-0.00377731, 0.7256378,  -0.6880665,  -0.25664434],
        [-0.64494236,  0.52407519, 0.55623245,  -0.10567834],
        [0.76422188, 0.4458643,  0.46601497, -0.17495751],
        [ 0.0,                   0.0,                   0.0,                   1.0]
    ], dtype=np.float64)

    # —— 稳定性缓存，用于存储最近10次的 getobj —— #
    buf = deque(maxlen=10)

    # —— 3. RealSense 初始化 —— #
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    align = rs.align(rs.stream.color)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            depth_frame = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            depth_img = np.asanyarray(depth_frame.get_data())
            color_img = np.asanyarray(color_frame.get_data())

            # —— 4. HSV 分割红色目标 —— #
            hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
            m1 = cv2.inRange(hsv, (0,120,70),   (10,255,255))
            m2 = cv2.inRange(hsv, (160,120,70), (180,255,255))
            mask = cv2.morphologyEx(cv2.bitwise_or(m1, m2),
                                     cv2.MORPH_OPEN,
                                     np.ones((5,5), np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                     np.ones((5,5), np.uint8))

            # —— 5. 找轮廓并选最近最大的目标 —— #
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            targets = []
            for c in cnts:
                area = cv2.contourArea(c)
                if area < 100:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                u, v = x + w//2, y + h//2
                Z = depth_frame.get_distance(u, v)
                if Z == 0:
                    continue
                targets.append((c, area, u, v, Z))

            if targets:
                # 按 深度升序 & 面积降序 排序，取第一个
                c, area, u, v, Z = sorted(targets, key=lambda t: (t[4], -t[1]))[0]

                # —— 6. 畸变校正 + 像素坐标→相机坐标 —— #
                norm = cv2.undistortPoints(
                    np.array([[[u, v]]], dtype=np.float32),
                    cameraMatrix=K,
                    distCoeffs=dist
                )[0,0]
                X_cam = norm[0] * Z
                Y_cam = norm[1] * Z

                # —— 7. 转换到基座(World)坐标系 —— #
                p_cam  = np.array([X_cam, Y_cam, Z, 1.0], dtype=np.float64)
                p_base = T_cb.dot(p_cam)
                Xb, Yb, Zb = p_base[:3]

                # 构造 getobj 列表
                getobj = [0.0, 0.0, 0.0, Z, Yb, Zb]
                print("当前 getobj:", getobj)

                # —— 稳定性检测：缓存最近10次值，检查误差范围 —— #
                buf.append(getobj)
                if len(buf) == 10:
                    arr = np.array(buf)   # shape (10,6)
                    diffs = arr.max(axis=0) - arr.min(axis=0)
                    if np.all(diffs <= 0.05):
                        print("🎉 稳定达标！连续10次误差均 ≤ 0.05")
                        arm.loopOn()
                        time.sleep(1)
                        arm.labelRun("forward")
                        arm.startTrack(armState.JOINTCTRL)
                        gripper_pos = 0.0
                        jnt_speed = 0.6
                        arm.MoveJ(getobj, gripper_pos, jnt_speed)
                        arm.labelRun("forward")
                        arm.backToStart()
                        arm.loopOff()
                        time.sleep(1)
                        
                        buf.clear()  # 可选：清空缓存，避免重复提示

                # —— 8. 可视化框与文本（可选） —— #
                # x, y, w, h = cv2.boundingRect(c)
                # cv2.rectangle(color_img, (x, y), (x + w, y + h), (0,255,0), 2)
                # cv2.circle(color_img, (u, v), 5, (0,0,255), -1)
                # cv2.putText(color_img, f"{Zb:.3f}m,B", (x, y - 10),
                #             cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

            # —— 9. 显示图像 —— #
            disp = np.uint8((depth_img / depth_img.max()) * 255)
            cv2.imshow('RGB',   color_img)
            cv2.imshow('Depth', cv2.cvtColor(disp, cv2.COLOR_GRAY2BGR))
            cv2.imshow('Mask',  mask)

            if cv2.waitKey(5) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
