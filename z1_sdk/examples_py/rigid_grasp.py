import time
import serial # type: ignore
import crcmod # type: ignore
import struct

# 引入所需库

# portx = "COM3"
portx = "/dev/Bluetooth"
bps = 115200
timex = 5
# 定义串口参数

ser = serial.Serial(portx, bps, 8, 'N', 1)
flag = ser.is_open

def crc_calculate(number: str):
    data_hex = number
    crc16 = crcmod.predefined.Crc('modbus')
    data = bytes.fromhex(data_hex)
    crc16.update(data)
    crc_value = crc16.crcValue
    crc_hex = format(crc_value, '04x')
    # 计算crc校验码
    num = crc_value
    hex_str = format(num, '04x')
    hex_digit1 = hex_str[0:2]
    hex_digit2 = hex_str[2:4]
    new_hex_str = hex_digit2 + " " + hex_digit1
    # new_num = int(new_hex_str, 16)
    # 计算正确的crc校验码（前后对调）
    finaldata = data_hex + ' ' + new_hex_str
    # print("CRC 校验码（十六进制）:", crc_hex.upper())
    return finaldata

class rigid_move:
    def __init__(self, pos, speed, stre):
        if pos >= 256:
            pos = hex(pos) 
            pos = "0" + pos[2:3] + " " + pos[3:]
        elif 16 <= pos < 256:
            pos = hex(pos)
            pos = "00" + " " + pos[2:] + "0"
        elif pos < 16:
            pos = hex(pos)
            pos = "00" + " " + "0" + pos[2:]
        else:
            pos = hex(pos)
            pos = "00" + " " + pos[2:]
        stre = hex(stre)
        stre = "00" + " " + stre[2:]
        speed = hex(speed)
        speed = "00" + " " + speed[2:]
        pos = '01 06 01 03' + ' ' + pos
        speed = '01 06 01 04' + ' ' + speed
        stre = '01 06 01 01' + ' ' + stre
        self.position = pos
        self.speed = speed
        self.strength = stre
    def grasp(self):
        # position = '01 06 01 03' + str(pos)
        # speed = '01 06 01 04' + str(speed)
        # strength = '01 06 01 01 ' + str(pos)
        # initail = crc_calculate('01 06 01 00 00 a5 ')
        position_data = crc_calculate(self.position)
        # position_data = '01 06 01 03 03 E8 78 21'
        speed_data = crc_calculate(self.speed)
        strength_data = crc_calculate(self.strength)
        # ser.write(bytes.fromhex(initail))
        # ser.flushOutput()
        # time.sleep(0.1)
        ser.write(bytes.fromhex(speed_data))
        ser.flushOutput()
        time.sleep(0.1)
        ser.write(bytes.fromhex(strength_data))
        ser.flushOutput()
        time.sleep(0.1)
        ser.write(bytes.fromhex(position_data))
        ser.flushOutput()
        time.sleep(0.1)

    def open(self):
        # initail = crc_calculate('01 06 01 00 00 a5 ')
        position_data = crc_calculate('01 06 01 03 03 e8')
        # position_data = '01 06 01 03 03 E8 78 21'
        speed_data = crc_calculate(self.speed)
        strength_data = crc_calculate(self.strength)
        # ser.write(bytes.fromhex(initail))
        # ser.flushOutput()
        # time.sleep(0.1)
        ser.write(bytes.fromhex(speed_data))
        ser.flushOutput()
        time.sleep(0.1)
        ser.write(bytes.fromhex(strength_data))
        ser.flushOutput()
        time.sleep(0.1)
        ser.write(bytes.fromhex(position_data))
        ser.flushOutput()
        time.sleep(0.1)

# if __name__ == "__main__":
#     start()
    
