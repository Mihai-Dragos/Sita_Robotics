from robot import Robot
import json

class RobotData():
    serialNumber:str
    position:dict[str, float]
    rotation:dict[str, float]
    def __init__(self, robot:Robot):
        self.serialNumber = f"{robot.index:08}"
        x, z, y = robot.pos[0].item(), robot.pos[1].item(), robot.pos[2].item()
        self.position = {
            'x':x,
            'y':y,
            'z':z
        }
        roll, pitch, yaw = robot.euler_ori[0].item(), robot.euler_ori[1].item(), robot.euler_ori[2].item()
        self.rotation = {
            'pitch':pitch,
            'yaw':yaw,
            'roll':roll
        }

import numpy as np
import random

class RandomRobot(Robot):
    index:int
    pos:np.ndarray
    vel:np.ndarray

    def __init__(self):
        self.index = random.randint(0, 2)
        self.pos = np.array([np.float(random.random() * 4), np.float(random.random() * 4), np.float(0.45)])
        self.ori = np.array([np.float(0), np.float(0), np.float(random.random() * 2 * np.pi - np.pi)])