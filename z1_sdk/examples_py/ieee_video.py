import sys
sys.path.append("../lib")
sys.path.append("/home/njau/robotic_arm/z1_sdk/lib")
import time
import numpy as np
import time
import csv
import serial

print("Press ctrl+\ to quit process.")
# 
np.set_printoptions(precision=3, suppress=True)
ser = serial.Serial('/dev/Bluetooth',115200,timeout=1,bytesize=8, stopbits=1)

#ball
start_string = "AABBe6e6e61e14007878785aCCDD"
stop_string = "AABBd9d9d91e14000000005aCCDD"
#ball
start = bytes(bytearray.fromhex(start_string))
stop = bytes(bytearray.fromhex(stop_string))

#soft
# grasp_string = "AABB2001CCDD"
# drop_string = "AABB2000CCDD"
# #soft
# grasp = bytes(bytearray.fromhex(grasp_string))
# drop = bytes(bytearray.fromhex(drop_string))
timedelay=1
timedelay_1=3
# ser.write(bytes(bytearray.fromhex("AABBd9d9d91e14007878785aCCDD")))
time.sleep(timedelay)
ser.write(bytes(bytearray.fromhex("AABBd9d9d91e14007800005aCCDD")))
time.sleep(timedelay)
ser.write(bytes(bytearray.fromhex("AABBd9d9d91e14000078005aCCDD")))
time.sleep(timedelay)
ser.write(bytes(bytearray.fromhex("AABBd9d9d91e14000000785aCCDD")))
# time.sleep(timedelay)
# # ser.write(bytes(bytearray.fromhex("AABBd9d9d91e14007878005aCCDD")))
# # time.sleep(timedelay)
# # ser.write(bytes(bytearray.fromhex("AABBd9d9d91e14007800785aCCDD")))
# # time.sleep(timedelay)
# # ser.write(bytes(bytearray.fromhex("AABBd9d9d91e14000078785aCCDD")))
# # time.sleep(timedelay)
# ser.write(bytes(bytearray.fromhex("AABBf0f0f01e14000000005aCCDD")))
# time.sleep(timedelay)
# ser.write(bytes(bytearray.fromhex("AABBf0f0aa1e14000000005aCCDD")))
# time.sleep(timedelay)
# ser.write(bytes(bytearray.fromhex("AABBf0f0f01e14000000005aCCDD")))
# time.sleep(timedelay)
# ser.write(bytes(bytearray.fromhex("AABBf0aaf01e14000000005aCCDD")))
# time.sleep(timedelay)
# ser.write(bytes(bytearray.fromhex("AABBf0f0f01e14000000005aCCDD")))
# time.sleep(timedelay)
# ser.write(bytes(bytearray.fromhex("AABBaaf0f01e14000000005aCCDD")))
# time.sleep(timedelay)
# ser.write(bytes(bytearray.fromhex("AABBf0f0f01e14000000005aCCDD")))
# time.sleep(timedelay)
# ser.write(bytes(bytearray.fromhex("AABBdadada1e14007878785aCCDD")))
# # time.sleep(timedelay)
# # ser.write(bytes(bytearray.fromhex("AABBe6e6e61e30007878785aCCDD")))
time.sleep(timedelay_1)
ser.write(stop)