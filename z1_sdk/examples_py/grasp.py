from motor_controller import MotorManager, load_config   # 仅在子进程导入
import time
config = load_config()
motor  = MotorManager(config)
print("[Gripper] 准备就绪")

try:
    while True:
        cmd = input("请输入指令：")     
        if cmd == "0":
            motor.close_gripper()
            print("[Gripper] 已闭合，等待 5 s…")
            time.sleep(5)
        elif cmd == "1":
            motor.open_gripper()
            print("[Gripper] 已打开")
finally:
    motor.stop_and_disable()
    print("[Gripper] 进程结束")