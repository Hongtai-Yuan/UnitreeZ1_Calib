import requests
import numpy as np
import time
np.set_printoptions(precision=3)

url = "http://127.0.0.1:12000/unitree/z1"
database = {
    "func": "",
    "args": {},
}

def labelRun(label):
    assert len(label) < 10

    # copy data
    data = database.copy()
    data["func"] = "labelRun"
    data["args"] = {
        "label": label,
    }
    return requests.post(url, json=data)

def labelSave(label):
    assert len(label) < 10

    # copy data
    data = database.copy()
    data["func"] = "labelSave"
    data["args"] = {
        "label": label,
    }
    return requests.post(url, json=data)

def backToStart():
    data = database.copy()
    data["func"] = "backToStart"
    return requests.post(url, json=data)

def Passive():
    data = database.copy()
    data["func"] = "Passive"
    return requests.post(url, json=data)

def getQ():
    data = database.copy()
    data["func"] = "getQ"
    return requests.post(url, json=data)
    
def getPose():
    data = database.copy()
    data["func"] = "getPose"
    return requests.post(url, json=data)

def MoveJ(q_or_posture: list, flag = 0,gripperPos = 0, speed = 0.5):
    assert len(q_or_posture) == 6

    data = database.copy()
    data["func"] = "MoveJ"
    if flag == 0:
        data["args"] = {
            "q": q_or_posture,
            "gripperPos": gripperPos,
            "maxSpeed": speed,
        }
    if flag == 1:
        data["args"] = {
            "posture": q_or_posture,
            "gripperPos": gripperPos,
            "maxSpeed": speed,
        }

    return requests.post(url, json=data)
# # test12
# labelRun("zuhui3")

# while True:
    # time.sleep(1)
    # print(getPose().json()['posture'])
# print(getPose().json()['posture'])
# -0.948909, 1.328608, -2.460461, 1.461961, -0.015457, 0.002684
# MoveJ([0.004935573222249629, 1.3894254311457606,-0.04995410867804875,0.35810619966191076,-0.019163907882137732,0.2678677468900666],1,0,0.8)
# [0.003946975984977373, 1.389505332522784, -0.050785575493687674, 0.3512178309012187, -0.018830311120400355, 0.30546753533829907]
# backToStart()
# Passive()