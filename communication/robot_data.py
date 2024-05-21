from robot import Robot

class RobotData():
    serialNumber:str
    position:dict[str, float]
    rotation:dict[str, float]

    def __init__(self, robot:Robot):
        self.serialNumber = f"{robot.index:08}"
        x, z, y = robot.pos
        self.position = {
            'x':x,
            'y':y,
            'z':z
        }
        roll, pitch, yaw = robot.ori
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
        self.pos = np.array([random.random() * 4, random.random() * 4, 0.45])
        self.ori = np.array([0, 0, random.random() * 2 * np.pi - np.pi])