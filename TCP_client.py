from socket import socket, AF_INET, SOCK_STREAM, SHUT_RDWR, SHUT_RD, SHUT_WR
import json
import time
from queue import Queue 
from threading import Thread
from collections.abc import Callable

class Header():
    version = 0
    '''Specifies the header version to check header data for.'''

    def __init__(self, header_data:bytes):
        '''Constructor for header using header data to extract values.'''
        self.message_size = 0
        if (len(header_data) != Header.get_size()):
            print(f"Header v{Header.version}" + " Header data has wrong amount of bytes for this version.")
            return
        if (header_data[0] != Header.version):
            print(f"Header v{Header.version}" + "Header version does not match this header version.")
            return
        
        self.message_size = int.from_bytes(header_data[1:5])
        '''Describes number of bytes in the message'''

    def create(message_data:bytes) -> bytes:
        '''Get the header bytes'''
        header = bytes()
        # 1 byte header version data
        header += Header.version.to_bytes(1, byteorder="big", signed=False)
        # 4 bytes specifying message length
        header += len(message_data).to_bytes(4, byteorder="big", signed=False)
        return header
    
    def get_size() -> int:
        '''Get the size of the header in number of bytes.'''
        return 5
    
class Connection():

    def __init__(self, connected_socket:socket):
        self.socket = connected_socket
        self._writing = True
        self._reading = True

        tcp_socket.settimeout(60)

        self._messages_to_send = Queue[str]()

        self.receiver = Thread(target=self._receiver)
        self.sender = Thread(target=self._sender)

        def end_of_file_listener(called_socket:socket):
            if (called_socket == self.socket):
                self.__stop_reading__()
                self.__stop_writing__()
        self.end_listener = end_of_file_listener
        Message.add_end_of_file_listener(self.end_listener)

        self.receiver.start()
        self.sender.start()

    def __stop_reading__(self):
        if (self._reading):
            print("Stop reading on connection")
            self._reading = False
            self.socket.shutdown(SHUT_RD)
    
    def __stop_writing__(self):
        if (self._writing):
            print("Stop writing on connection")
            self._writing = False
            self.send_message("Unblock sender from waiting")
            self.socket.shutdown(SHUT_WR)
        
    def close(self):
        print("Closing connection")
        
        self.__stop_writing__()
        print("Waiting for sender thread to join")
        self.sender.join()
        
        print("Waiting for reading to stop from EOF signal")
        self.socket.settimeout(16)
        while(self._reading):
            time.sleep(0.1)
        
        print("Waiting for receiver thread to join")
        self.receiver.join()
            
        print("Closing connection to the socket")
        self.socket.close()
        Message.remove_end_of_file_listener(self.end_listener)

    def send_message(self, message:str):
        self._messages_to_send.put(message)

    def _receiver(self):
        while self._reading:
            received_data = self._receive()
            if (not received_data): return
            print(bytes.decode(received_data, "utf-8"))

    def _sender(self):
        while True:
            data_to_send = self._messages_to_send.get()
            if (not self._writing): return
            self._send(str.encode(data_to_send, "utf-8"))

    def _receive(self) -> bytes:
        return Message.receive(self.socket)

    def _send(self, data:bytes):
        Message(data).send(self.socket)

class Message():

    def __init__(self, data:bytes):
        self.data = data

    def send(self, socket:socket):
        print(f"Header bytes: {Header.create(self.data)}")
        socket.send(Header.create(self.data))
        socket.send(self.data)

    def receive(socket:socket) -> bytes:
        header_data = Message.receive_sized_message(socket, Header.get_size())
        if (not header_data): return
        print(f"Received message header.")
        message_data = Message.receive_sized_message(socket, Header(header_data).message_size)
        if (not message_data): return
        print(f"Received message header.")
        return message_data
    
    def receive_sized_message(socket:socket, size:int) -> bytes:
        data_received = bytearray()
        while len(data_received) < size:
            data = socket.recv(size)
            if not data:
                print("Message | Recieved EOF")
                Message.end_of_file_handler(socket)
                return
            data_received.extend(data)
        return bytes(data_received)
    
    _end_of_file_listeners =  list[Callable[[socket], None]]()

    def add_end_of_file_listener(listener:Callable[[socket], None]):
        Message._end_of_file_listeners.append(listener)

    def remove_end_of_file_listener(listener:Callable[[socket], None]):
        Message._end_of_file_listeners.remove(listener)

    def end_of_file_handler(socket:socket):
        for listener in Message._end_of_file_listeners:
            listener(socket)

server_address = ("127.0.0.1", 5000)

# Open Ipv4 TCP socket
tcp_socket = socket(AF_INET, SOCK_STREAM)

# Connect socket to the server address
tcp_socket.connect(server_address)

connection = Connection(tcp_socket)

connection.send_message("Hey server, how are you doing?")

while True:
    input_string = input()
    if (input_string == "stop"): break
    connection.send_message(input_string)

connection.close()