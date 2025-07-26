#!/usr/bin/env python3
# coding: utf-8
"""
双进程 + 双向队列
    task_q : Vision ➜ Arm    (getobj)
    ack_q  : Arm    ➜ Vision ("ok")
按  Ctrl+C / Ctrl+\  均会让机械臂最后回零
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


# ========= Arm 进程 ========= #
def arm_worker(task_q: mp.Queue, ack_q: mp.Queue):
    arm = unitree_arm_interface.ArmInterface(hasGripper=True)
    armState = unitree_arm_interface.ArmFSMState
    arm.loopOn()
    print("[Arm] loopOn OK")
    gripper_pos, jnt_speed = 0.0, 0.6

    try:
        while True:
            data = task_q.get()
            if data is None:                   # ← 哨兵：主进程/视觉要求退出
                print("[Arm] 收到退出哨兵，回零")
                break

            # ===== 正常任务 =====
            arm.labelRun("start")
            arm.startTrack(armState.JOINTCTRL)
            arm.MoveJ(data, gripper_pos, jnt_speed)
            arm.labelRun("start")
            # arm.backToStart()
            ack_q.put("ok")
    finally:
        try:
            arm.backToStart()                  # 再保险：无论啥原因都回零一次
        except Exception as e:
            print(f"[Arm] 回零失败: {e}")
        arm.loopOff()
        print("[Arm] loopOff，进程结束")

def main():
    # —— 1. 内参和畸变系数硬编码 —— #
    K = np.array([
        [607.54630191,   0.0,               298.03414178],
        [0.0,                 602.33843031, 260.54716074],
        [0.0,                 0.0,               1.0]
    ], dtype=np.float64)



# ========= Vision 进程 ========= #
def vision_worker(task_q: mp.Queue, ack_q: mp.Queue):
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
    T_cb = np.array([[-0.00007629, -0.01211961,  0.99992655, 0.95664434],
                     [-0.99969708,  0.02461114,  0.00022203,  0.2867834],
                     [-0.02461202, -0.99962363, -0.01211782, 0.54495751],
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
                    if ack_q.get_nowait()=="ok":
                        waiting_ack=False
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
            if not depth or not color: continue
            depth_img = np.asanyarray(depth.get_data())
            hsv = cv2.cvtColor(np.asanyarray(color.get_data()), cv2.COLOR_BGR2HSV)

            m1=cv2.inRange(hsv,(0,120,70),(10,255,255))
            m2=cv2.inRange(hsv,(160,120,70),(180,255,255))
            mask=cv2.morphologyEx(cv2.bitwise_or(m1,m2),cv2.MORPH_OPEN,
                                  np.ones((5,5),np.uint8))
            mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,
                                  np.ones((5,5),np.uint8))

            cnts,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            targets=[]
            for c in cnts:
                if cv2.contourArea(c)<100: continue
                x,y,w,h=cv2.boundingRect(c); u,v=x+w//2,y+h//2
                Z=depth.get_distance(u,v)
                if Z==0: continue
                targets.append((c,cv2.contourArea(c),u,v,Z))

            if targets and not waiting_ack:
                c,area,u,v,Z = sorted(targets,key=lambda t:(t[4],-t[1]))[0]
                norm = cv2.undistortPoints(np.array([[[u,v]]],np.float32),
                                           K,dist)[0,0]
                X_cam,Y_cam = norm*Z
                Xb,Yb,Zb = (T_cb @ np.array([X_cam,Y_cam,Z,1]))[:3]
                getobj=[0,0,0,float(Z-0.55),float(Yb),float(Zb)]
                buf.append(getobj)
                if len(buf)==10:
                    arr=np.asarray(buf)
                    if ((arr.max(0)-arr.min(0))<=0.05).all():
                        task_q.put(buf[-1])
                        waiting_ack=True
                        buf.clear()
                        print("[Vision] → 发送目标")

            cv2.imshow("mask",mask)
            if cv2.waitKey(1)&0xFF in (ord('q'), ord('Q')):
                break
    finally:
        pipe.stop(); cv2.destroyAllWindows()
        task_q.put(None)            # 发哨兵
        print("[Vision] 进程结束")


# ========= 主进程 ========= #
def main():
    mp.set_start_method('spawn')     # 跨平台
    task_q, ack_q = mp.Queue(), mp.Queue()
    # 可选：开底层控制程序
    os.system(f"gnome-terminal -- bash -c 'cd {GNOME_CMD_PATH}'")

    arm_p = mp.Process(target=arm_worker, args=(task_q, ack_q))
    vis_p = mp.Process(target=vision_worker,args=(task_q, ack_q))
    arm_p.start(); vis_p.start()

    def graceful_exit(signum, _frame):
        print("\n[Main] 收到退出信号，转发给 Vision…")
        # 给 Vision 发 SIGINT，它会走 finally → 发送哨兵
        vis_p.send_signal(signal.SIGINT)

    # Ctrl+\ / Ctrl+C 都走 graceful_exit
    signal.signal(signal.SIGQUIT, graceful_exit)
    signal.signal(signal.SIGINT,  graceful_exit)

    arm_p.join(); vis_p.join()

if __name__ == '__main__':
    print("Press Ctrl+C 或 Ctrl+\\ 退出")
    main()
