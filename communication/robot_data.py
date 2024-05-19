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
            'x':pitch,
            'y':yaw,
            'z':roll
        }