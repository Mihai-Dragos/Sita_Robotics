from directory import directory_setup
directory_setup()

from util import log, debug_log, add_debug_context
from communication.robot_data import RobotData, RandomRobot
from robot import Robot
import time
from timeline import save_robot_data
from communication.channel import send_message, channel

import json

def send_robot_data(robot:Robot):
    data = RobotData(robot)
    message = json.dumps(data.__dict__)
    save_robot_data(data)
    send_message(message)

if (__name__ == "__main__"):
    while True:
        time.sleep(0.1)
        log("TCP Client", "Waiting for message or type 'stop' to end connection:")
        input_string = input()
        if (input_string == "stop"): 
            break
        if (input_string == "debug"):
            add_debug_context("TCP Client")
            continue
        if (channel and channel.is_finished()):
            channel.close()
            break
        if (input_string == "robot"):
            send_robot_data(RandomRobot())
            continue
        send_message(input_string)

    debug_log("TCP Client", "Closing connection")
    if channel:
        channel.close()