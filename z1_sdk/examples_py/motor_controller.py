# motor_controller.py
import serial
import os
import sys
from DM_CAN import *
import yaml

# def load_config(path="config.yaml"):
#     with open(path, 'r') as f:
#         return yaml.safe_load(f)
def load_config():
    # 获取当前脚本所在的真实路径
    base_path = getattr(sys, '_MEIPASS', os.path.dirname(os.path.abspath(__file__)))
    config_path = os.path.join(base_path, 'config.yaml')
    
    with open(config_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


class MotorManager:
    def __init__(self, config):
        self.port = config["motor"]["port"]
        self.baud = config["motor"]["baudrate"]
        self.pos_open = config["motor"]["open_position"]
        self.pos_close = config["motor"]["close_position"]
        self.force = config["motor"]["force"]
        self.speed = config["motor"]["speed"]

        self.ser = None
        self.Motor1 = None
        self.MotorControl1 = None

        self.connect_motor()

    def connect_motor(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            self.ser.flush()
            print(f"[电机] 串口已打开: {self.port}")
        except Exception as e:
            print(f"[电机] 串口打开失败: {e}")
            return

        self.Motor1 = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)
        self.MotorControl1 = MotorControl(self.ser)
        self.MotorControl1.addMotor(self.Motor1)
        self.MotorControl1.switchControlMode(self.Motor1, Control_Type.Torque_Pos)
        self.MotorControl1.set_zero_position(self.Motor1)
        self.MotorControl1.enable(self.Motor1)

    def open_gripper(self):
        self.MotorControl1.control_pos_force(self.Motor1, self.pos_open, self.force, self.speed)

    def close_gripper(self):
        self.MotorControl1.control_pos_force(self.Motor1, self.pos_close, self.force, self.speed)

    def stop_and_disable(self):
        if self.Motor1:
            self.MotorControl1.control_pos_force(self.Motor1, self.Motor1.getPosition(), 0, 0)
            self.MotorControl1.disable(self.Motor1)
        
