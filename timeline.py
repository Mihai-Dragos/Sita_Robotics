from settings import SITUATION_NAME, SAVE_TIMELINE, SEND_TIMELINE, LIMIT_TIMELINE_RATE, TIMELINE_RATE, TIMELINE_PATH
from util import debug_log, log
from communication.robot_data import RobotData
from communication.channel import send_message

from threading import Thread
import time
import json
import os


last_save = time.time()

def get_timeline_file_address():
    folder_address = os.path.join(os.path.dirname(__file__), TIMELINE_PATH, SITUATION_NAME)
    file_address = f"{folder_address}.json"
    return file_address

def save_robot_data(robotdata:RobotData):
    if not SAVE_TIMELINE:
        return
    debug_log("Timeline", "Trying to save robot data to timeline")
    global last_save
    waitTime = time.time() - last_save
    if LIMIT_TIMELINE_RATE and TIMELINE_RATE > waitTime:
        return
    
    file_address = get_timeline_file_address()
    debug_log("Timeline", f"Adding robot data to timeline with file address {file_address}")

    with open(file_address, 'a') as file:
        file.write(f"{waitTime}\n")
        file.write(f"{json.dumps(robotdata.__dict__)}\n")
        last_save = time.time()


def clear_timeline():
    file_address = get_timeline_file_address()
    debug_log("Timeline", f"Clearing timeline file with file address {file_address}")
    with open(file_address, 'w') as file:
        file.write("")


last_load = time.time()

def timeline_thread():
    file_address = get_timeline_file_address()
    
    if not os.path.isfile(file_address):
        debug_log("Timeline", f"Could not load timeline from file address {file_address}")
        return
    
    log("Timeline", "Starting to send timeline data")
    global last_load
    
    with open(file_address, 'r') as file:
        while True:
            # Check if there is still a line to read.
            readline = file.readline()
            if not readline:
                break

            waitTime = float(readline)
            debug_log("Timeline", f"Read wait time: {waitTime}")
            data = file.readline()
            debug_log("Timeline", f"Read timeline data: {data}")

            processTime = time.time() - last_load

            debug_log("Timeline", f"Waiting: {waitTime - processTime}s")
            time.sleep(waitTime - processTime)
            
            last_load = time.time()

            debug_log("Timeline", f"Sending timeline data not load timeline from file address {file_address}")
            send_message(data)

    log("Timeline", "Finished sending timeline data")


def send_timeline():
    debug_log("Timeline", "Sending data from timeline")
    Thread(target=timeline_thread).start()

if SAVE_TIMELINE:
    clear_timeline()
elif SEND_TIMELINE:
    send_timeline()