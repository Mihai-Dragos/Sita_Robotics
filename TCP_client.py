from socket import socket, AF_INET, SOCK_STREAM, SHUT_RDWR, SHUT_RD, SHUT_WR
import json
import time
from queue import Queue 
from threading import Thread
from collections.abc import Callable
from util import log

DEBUG_TCP_CLIENT = False

def debug_log(context:str, message:str, shifted:bool=False):
    if DEBUG_TCP_CLIENT: log(context, message, shifted)

class Header():
    version = 0
    '''Specifies the header version to check header data for.'''

    def __init__(self, header_data:bytes):
        '''Constructor for header using header data to extract values.'''

        self.message_size = 0
        '''Describes number of bytes in the message'''

        if (len(header_data) != Header.get_size()):
            debug_log(f"Header v{Header.version}", f"Header data has {len(header_data)}" +
                f" amount of bytes, while this version expects {Header.get_size()}")
            return
        
        if (header_data[0] != Header.version):
            debug_log(f"Header v{Header.version}", f"Header version {header_data[0]}" + 
                    f" does not match this header version {Header.version}")
            return
        
        self.message_size = int.from_bytes(header_data[1:5])
        debug_log(f"Header v{Header.version}", f"Header received message size {self.message_size}")

    def create(message_data:bytes) -> bytes:
        '''Get the header bytes'''
        header = bytes()

        # 1 byte header version data
        header += Header.version.to_bytes(1, byteorder="big", signed=False)
        
        # 4 bytes specifying message length
        header += len(message_data).to_bytes(4, byteorder="big", signed=False)

        debug_log(f"Header v{Header.version}", "Created header data:")
        debug_log("", f"[{header.hex("|")}]")
        return header
    
    def get_size() -> int:
        '''Get the size of the header in number of bytes.'''
        return 5
    
class Connection():

    def __init__(self, connected_socket:socket):
        log("Connection", f"Initializing connection for " +
            f"connected socket {connected_socket.getsockname()}")
        self.socket = connected_socket
        self._writing = True
        self._reading = True

        #tcp_socket.settimeout(60)

        self._messages_to_send = Queue[str]()

        self.receiver = Thread(target=self.__receiver__)
        self.sender = Thread(target=self.__sender__)

        def end_of_file_listener(called_socket:socket):
            if (called_socket == self.socket):
                self.__stop_reading__()
                self.__stop_writing__()
        self.end_listener = end_of_file_listener
        Message.add_end_of_file_listener(self.end_listener)

        debug_log("Connection", f"Starting receiver and sender thread")
        self.receiver.start()
        self.sender.start()

    def __stop_reading__(self):
        '''Stop the connection from reading on the socket'''
        if (self._reading):
            debug_log("Connection", "Stop reading from the socket")
            self._reading = False
            self.socket.shutdown(SHUT_RD)
    
    def __stop_writing__(self):
        '''Stop the connection from writing on the socket'''
        if (self._writing):
            debug_log("Connection", "Stop writing to the socket")
            self._writing = False
            self.send_message("") # Unblock sender from waiting
            self.socket.shutdown(SHUT_WR)
        
    def close(self):
        '''Close the connection of the socket to the current server'''
        log("Connection", f"Closing the TCP connection with socket {self.socket.getsockname()}")
        
        self.__stop_writing__()
        debug_log("Connection", f"Waiting for sender thread to join")
        self.sender.join()
        
        debug_log("Connection", f"Waiting for reading to stop " +
                                 f"from EOF signal or 4 second timeout")
        self.socket.settimeout(4)
        while(self._reading):
            time.sleep(0.1)
        
        debug_log("Connection", "Waiting for receiver thread to join")
        self.receiver.join()
            
        debug_log("Connection", f"Closing connection to socket {self.socket.getsockname()}")
        self.socket.close()
        Message.remove_end_of_file_listener(self.end_listener)

    def send_message(self, message:str):
        '''Add a message to the connection to send'''
        debug_log("Connection", f"Adding message to the queue:")
        debug_log("", f"\"{message}\"")
        self._messages_to_send.put(message)

    def __receiver__(self):
        '''Program to run on a thread, which continously receives messages from the connection'''
        debug_log("Receiver", f"Start of receiver thread")

        while self._reading:
            received_data = self._receive()
            debug_log("Receiver", f"Received message from the connection")
        
            if (not received_data): 
                continue
        
            log("Receiver", f"Received message:")
            log("", f"\"{bytes.decode(received_data, "utf-8")}\"")
        
        self.__stop_reading__() # Incase the receiver was ended by exception
        debug_log("Receiver", f"Ending receiver thread")

    def __sender__(self):
        '''Program to run on a thread, which writes all messages to the connection'''
        debug_log("Sender", f"Start of sender thread")

        while True:
            data_to_send = self._messages_to_send.get()
            debug_log("Sender", f"Got new data to send from queue")

            if (not self._writing): 
                break # End sender thread as we should stop writing 
            
            log("Sender", "Sending message:")
            log("", f"{data_to_send}")
            self._send(str.encode(data_to_send, "utf-8"))
        
        self.__stop_writing__() # Incase the receiver was ended by exception
        debug_log("Sender", f"Ending sender thread")

    def _receive(self) -> bytes:
        '''Wait to receive a message from the socket connection'''
        debug_log("Connection", f"Waiting to receive message")
        return Message.receive(self.socket)

    def _send(self, data:bytes):
        '''Send a message to the socket connection'''
        debug_log("Connection", f"Sending message")
        Message(data).send(self.socket)

class Message():

    def __init__(self, data:bytes):
        '''Create a message with specified data payload'''
        self.data = data

    def send(self, socket:socket):
        '''Send the created message to the specified socket'''
        debug_log("Message", f"Sending message header")
        socket.send(Header.create(self.data))

        debug_log("Message", f"Sending message data")
        socket.send(self.data)

    def receive(socket:socket) -> bytes:
        '''Wait to receive a message from the specified socket'''
        header_data = Message.receive_sized_message(socket, Header.get_size())

        if (not header_data): return

        debug_log("Message", f"Received message header")
        message_data = Message.receive_sized_message(socket, Header(header_data).message_size)

        if (not message_data): return

        debug_log("Message", f"Received message data")
        return message_data
    
    def receive_sized_message(socket:socket, size:int) -> bytes:
        '''Receive a message of specified size from the socket.'''
        data_received = bytearray()

        # Continue to read data until we got the specified amount
        while len(data_received) < size:
            data = socket.recv(size)

            # Check if data was received
            if not data:
                debug_log("Message", f"Received End Of File signal")
                # No data received is End Of File signal, so we alert listeners
                Message.end_of_file_handler(socket)
                return
            
            # Add data to currently received data buffer
            data_received.extend(data)

        return bytes(data_received)
    
    __end_of_file_listeners__ =  list[Callable[[socket], None]]()
    '''List of listeners for the End Of File signal'''

    def add_end_of_file_listener(listener:Callable[[socket], None]):
        '''Add a listener to the End Of File signal'''
        debug_log("Message", f"Adding End Of File listener")
        Message.__end_of_file_listeners__.append(listener)

    def remove_end_of_file_listener(listener:Callable[[socket], None]):
        '''Remove a listener for the End Of File signal'''
        debug_log("Message", f"Removing End Of File listener")
        Message.__end_of_file_listeners__.remove(listener)

    def end_of_file_handler(socket:socket):
        '''Alert all the listeners of the End Of File signal'''
        for listener in Message.__end_of_file_listeners__:
            listener(socket)

server_address = ("127.0.0.1", 5000)

# Open Ipv4 TCP socket
tcp_socket = socket(AF_INET, SOCK_STREAM)

if __name__ == "__main__":
    log("TCP Client", f"Connecting socket to server address: {server_address}")
    # Connect socket to the server address
    tcp_socket.connect(server_address)

    connection = Connection(tcp_socket)

    connection.send_message("Hey server, how are you doing?")

    while True:
        time.sleep(0.1)
        log("TCP Client", f"Waiting for message or type 'stop' to end connection:")
        input_string = input()
        if (input_string == "stop"): break
        if (input_string == "debug"): DEBUG_TCP_CLIENT = not DEBUG_TCP_CLIENT
        connection.send_message(input_string)

    debug_log("TCP Client", f"Closing connection")
    connection.close()