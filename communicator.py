from util import log, debug_log, add_debug_context
from communication.robot_data import RobotData, RandomRobot
from communication.TCP_client import create_tcp_connection
from robot import Robot
import time

import json

class Channel():
    def __init__(self):
        self.connection = create_tcp_connection()
        self.connection.add_receive_listener(Channel.on_receive_message)

    def send_message(self, message:str):
        log("Channel", "Sending message:")
        log("", f"{message}")
        self.connection.send_message(message.encode("utf-8"))

    def on_receive_message(data:bytes):
        log("Channel", "Received message")
        message = data.decode("utf-8")
        log("", f"\'{message}\'")

    def close(self):
        self.connection.close()

    def is_finished(self):
        return not self.connection._reading and not self.connection._writing

channel = None

try:
    channel = Channel()
except (Exception):
    debug_log("TCP Client", "Exception occurred")

def send_robot_data(robot:Robot):
    data = RobotData(robot)
    message = json.dumps(data.__dict__)

    if channel: 
        channel.send_message(message)

if (__name__ == "__main__"):
    while True:
        time.sleep(0.1)
        log("TCP Client", "Waiting for message or type 'stop' to end connection:")
        input_string = input()
        if (input_string == "stop"): break
        if (input_string == "debug"): 
            add_debug_context("TCP Client")
            continue
        if (channel.is_finished()):
            channel.close()
            break
        if (input_string == "robot"): 
            send_robot_data(RandomRobot())
            continue
        channel.send_message(input_string)

    debug_log("TCP Client", "Closing connection")
    channel.close()