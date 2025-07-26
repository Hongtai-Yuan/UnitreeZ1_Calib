#!/usr/bin/env python3
# coding: utf-8
"""
双进程 + 双向队列 + 夹爪独立进程
    task_q      : Vision ➜ Arm        (MoveJ 目标)
    ack_q       : Arm    ➜ Vision     ("ok")
    grip_cmd_q  : Arm    ➜ Gripper    ("close"/"open")
    grip_ack_q  : Gripper ➜ Arm       ("closed"/"opened")

流程：
1. Vision 找到稳定目标 → task_q.put(target)
2. Arm MoveJ 到位 → grip_cmd_q.put("close")
3. Gripper 执行 close，等待 10 s → grip_ack_q.put("closed")
4. Arm 收到 "closed" → arm.labelRun("open") → ack_q.put("ok")
5. Vision 得到 ack 后继续下一轮
Ctrl+C / Ctrl+\ 任意时刻安全退出，所有子进程都会回零/停机。
"""

import os, sys, time, signal, queue, multiprocessing as mp
from collections import deque
import numpy as np, cv2, pyrealsense2 as rs

# ---------- 用户自定义 ---------- #
UNITREE_SDK_PATH = "/home/x86/Z1Rbot/z1_sdk/lib"
GNOME_CMD_PATH   = "/home/x86/Z1Rbot/z1_controller/build && ./z1_ctrl"
# -------------------------------- #

os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
sys.path += ["../lib", UNITREE_SDK_PATH]
import unitree_arm_interface


# ========= Gripper 进程 ========= #
def gripper_worker(cmd_q: mp.Queue, ack_q: mp.Queue):
    """
    cmd_q 收到 "close" → 夹爪闭合并等待 10 s，再发 "closed"
                  "open"  → 张开夹爪并立即发 "opened"
                  None    → 退出
    """
    from motor_controller import MotorManager, load_config   # 仅在子进程导入
    config = load_config()
    motor  = MotorManager(config)
    print("[Gripper] 准备就绪")

    try:
        while True:
            cmd = cmd_q.get()
            if cmd is None:
                break

            if cmd == "close":
                motor.close_gripper()
                print("[Gripper] 已闭合，等待 10 s…")
                time.sleep(10)
                ack_q.put("closed")
            elif cmd == "open":
                motor.open_gripper()
                print("[Gripper] 已打开")
                ack_q.put("opened")
    finally:
        motor.stop_and_disable()
        print("[Gripper] 进程结束")


# ========= Arm 进程 ========= #
def arm_worker(task_q: mp.Queue, ack_q: mp.Queue,
               grip_cmd_q: mp.Queue, grip_ack_q: mp.Queue):
    arm = unitree_arm_interface.ArmInterface(hasGripper=True)
    armState = unitree_arm_interface.ArmFSMState
    arm.loopOn()
    print("[Arm] loopOn OK")
    gripper_pos, jnt_speed = 0.0, 0.4   #此处保留，不再直接控制张合

    try:
        while True:
            data = task_q.get()
            if data is None:                   # Vision 发的哨兵
                print("[Arm] 收到退出哨兵，回零")
                break

            # ===== 正常任务 =====
            arm.labelRun("open")
            arm.startTrack(armState.JOINTCTRL)
            arm.MoveJ(data, gripper_pos, jnt_speed)

            # ===== 令夹爪闭合 =====
            grip_cmd_q.put("close")
            if grip_ack_q.get() == "closed":   # 阻塞等待 10 s 后的回执
                arm.labelRun("open")           # 回到“open”姿态
                ack_q.put("ok")
                # grip_cmd_q.put("open")                # 通知 Vision
                # ack_q.put("ok")
                
    finally:
        try:
            arm.backToStart()                  # 兜底回零
        except Exception as e:
            print(f"[Arm] 回零失败: {e}")
        arm.loopOff()
        print("[Arm] loopOff，进程结束")


# ========= Vision 进程 ========= #
def vision_worker(task_q: mp.Queue, ack_q: mp.Queue):
    K = np.array([[631.3256856003788, 0, 324.11646638254126],
                  [0, 631.3093321979653, 246.65753824813333],
                  [0, 0, 1]], np.float64)
    dist = np.array([0.0456697768015125, 0.812734276031, 0.0022038192218,
                     0.0017764860540,   -4.18214043804], np.float64)
    T_cb = np.array([[-0.01257614, -0.02377752,  0.99963817, -0.02171796],
                     [-0.99972126,  0.02027590,  0.01209490,  0.02905920],
                     [-0.01998098, -0.99951164, -0.02402588,  0.10111825],
                     [0,0,0,1]], np.float64)


    pipe, align = rs.pipeline(), rs.align(rs.stream.color)
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, 640,480, rs.format.z16,30)
    cfg.enable_stream(rs.stream.color, 640,480, rs.format.bgr8,30)
    pipe.start(cfg)

    buf, waiting_ack = deque(maxlen=10), False
    print("[Vision] 开始检测…  Ctrl+C / q 退出")

    try:
        while True:
            # ── 1. 收 Arm 完成信号 ──
            if waiting_ack:
                try:
                    if ack_q.get_nowait() == "ok":
                        waiting_ack = False
                        print("[Vision] ✔ Arm 完成")
                except queue.Empty:
                    pass

            # ── 2. 相机帧 ──
            try:
                frames = pipe.wait_for_frames(timeout_ms=1000)
            except RuntimeError:
                continue
            depth, color = align.process(frames).get_depth_frame(), \
                           align.process(frames).get_color_frame()
            if not depth or not color:
                continue
            depth_img = np.asanyarray(depth.get_data())
            hsv = cv2.cvtColor(np.asanyarray(color.get_data()), cv2.COLOR_BGR2HSV)

            m1 = cv2.inRange(hsv, (0,120,70),  (10,255,255))
            m2 = cv2.inRange(hsv, (160,120,70),(180,255,255))
            mask = cv2.morphologyEx(cv2.bitwise_or(m1,m2), cv2.MORPH_OPEN,
                                    np.ones((5,5),np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                    np.ones((5,5),np.uint8))

            cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            targets = []
            for c in cnts:
                if cv2.contourArea(c) < 100: continue
                x,y,w,h = cv2.boundingRect(c); u,v = x+w//2, y+h//2
                Z = depth.get_distance(u,v)
                if Z == 0: continue
                targets.append((c, cv2.contourArea(c), u, v, Z))

            if targets and not waiting_ack:
                c,area,u,v,Z = sorted(targets, key=lambda t:(t[4], -t[1]))[0]
                norm = cv2.undistortPoints(np.array([[[u,v]]], np.float32),
                                           K, dist)[0,0]
                X_cam, Y_cam = norm * Z
                Xb, Yb, Zb = (T_cb @ np.array([X_cam, Y_cam, Z, 1]))[:3]
                getobj = [0,0,0, float(Z)-0.16, float(Yb), float(Zb)]
                buf.append(getobj)
                if len(buf) == 10:
                    arr = np.asarray(buf)
                    if ((arr.max(0) - arr.min(0)) <= 0.05).all():
                        task_q.put(buf[-1])
                        waiting_ack = True
                        buf.clear()
                        print("[Vision] → 发送目标")

            cv2.imshow("mask", mask)
            if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q')):
                break
    finally:
        pipe.stop(); cv2.destroyAllWindows()
        task_q.put(None)            # 发哨兵
        print("[Vision] 进程结束")


# ========= 主进程 ========= #
def main():
    mp.set_start_method('spawn')     # 跨平台
    task_q, ack_q = mp.Queue(), mp.Queue()              # Vision ↔ Arm
    grip_cmd_q, grip_ack_q = mp.Queue(), mp.Queue()     # Arm ↔ Gripper

    # 可选：开底层控制程序
    os.system(f"gnome-terminal -- bash -c 'cd {GNOME_CMD_PATH}'")

    arm_p  = mp.Process(target=arm_worker,
                        args=(task_q, ack_q, grip_cmd_q, grip_ack_q))
    vis_p  = mp.Process(target=vision_worker,
                        args=(task_q, ack_q))
    grip_p = mp.Process(target=gripper_worker,
                        args=(grip_cmd_q, grip_ack_q))

    arm_p.start(); vis_p.start(); grip_p.start()

    def graceful_exit(signum, _frame):
        print("\n[Main] 收到退出信号，转发给 Vision…")
        # Vision 会在 finally 中给 Arm 和 Gripper 发哨兵
        vis_p.send_signal(signal.SIGINT)

    # Ctrl+\ / Ctrl+C 都走 graceful_exit
    signal.signal(signal.SIGQUIT, graceful_exit)
    signal.signal(signal.SIGINT,  graceful_exit)

    arm_p.join(); vis_p.join()
    grip_cmd_q.put(None)     # 通知 gripper_worker 退出
    grip_p.join()

if __name__ == '__main__':
    print("Press Ctrl+C 或 Ctrl+\\ 退出")
    main()
