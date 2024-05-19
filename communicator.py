from util import log, debug_log
from communication.TCP_client import *

class Channel():
    def __init__(self):
        self.connection = create_tcp_connection()
        self.connection.add_receive_listener(Channel.on_receive_message)

    def send_message(self, message:str):
        log("Channel", f"Sending message:")
        log("", f"{message}")
        self.connection.send_message(message.encode("utf-8"))

    def on_receive_message(data:bytes):
        log("Channel", f"Received message")
        log("", f"\'{data.decode("utf-8")}\'")

    def close(self):
        self.connection.close()

    def is_finished(self):
        return not self.connection._reading and not self.connection._writing

channel = Channel()
while True:
    time.sleep(0.1)
    log("TCP Client", f"Waiting for message or type 'stop' to end connection:")
    input_string = input()
    if (input_string == "stop"): break
    if (input_string == "debug"): 
        DEBUG_LOG = not DEBUG_LOG
        continue
    if (channel.is_finished()):
        channel.close()
        break
    channel.send_message(input_string)

debug_log("TCP Client", f"Closing connection")
channel.close()
