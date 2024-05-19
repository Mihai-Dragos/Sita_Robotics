from util import log, debug_log
from communication.connection import Connection

from socket import socket, AF_INET, SOCK_STREAM
import time

server_address = ("127.0.0.1", 5000)

# Open Ipv4 TCP socket
tcp_socket = socket(AF_INET, SOCK_STREAM)

def create_tcp_connection(auto_start:bool=True) -> Connection:
    log("TCP Client", f"Connecting socket to server address: {server_address}")
    # Connect socket to the server address
    tcp_socket.connect(server_address)

    return Connection(tcp_socket, auto_start)

if __name__ == "__main__":
    connection = create_tcp_connection(False)
    
    def print_received_listener(received_data:bytes):
        message = bytes.decode(received_data, "utf-8")
        log("Receiver", f"Received message:")
        log("", f"\'{message}\'")
    connection.add_receive_listener(print_received_listener)

    connection.send_message(str.encode("Hey server, how are you doing?", "utf-8"))

    connection.start()

    while True:
        time.sleep(0.1)
        log("TCP Client", f"Waiting for message or type 'stop' to end connection:")
        input_string = input()
        if (input_string == "stop"): break
        if (input_string == "debug"): DEBUG_TCP_CLIENT = not DEBUG_TCP_CLIENT
        connection.send_message(str.encode(input_string))

    debug_log("TCP Client", f"Closing connection")
    connection.close()