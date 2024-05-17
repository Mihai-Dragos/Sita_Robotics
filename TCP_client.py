from socket import socket, AF_INET, SOCK_STREAM
import json
import time
from queue import Queue 
from threading import Thread 

class Header():
    version = 0
    '''Specifies the header version to check header data for.'''

    def __init__(self, header_data:bytes):
        '''Constructor for header using header data to extract values.'''
        if (len(header_data) != Header.get_size()):
            print(f"Header v{Header.version}" + "Header data has wrong amount of bytes for this version.")
        if (header_data[0] != Header.version):
            print(f"Header v{Header.version}" + "Header version does not match this header version.")
        
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
    
class Message():
    def __init__(self, data:bytes):
        self.data = data

    def send(self, socket:socket):
        print(f"Header bytes: {Header.create(self.data)}")
        socket.send(Header.create(self.data))
        socket.send(self.data)

    def recieve(socket:socket) -> bytes:
        header_data = Message.recieveSizedMessage(socket, Header.get_size())
        message_data = Message.recieveSizedMessage(socket, Header(header_data).message_size)
        return message_data
    
    def recieveSizedMessage(socket:socket, size:int) -> bytes:
        data_received = bytearray()
        while len(data_received) < size:
            data = socket.recv(size)
            if not data:
                raise Exception('Message | Recieved unexpected EOF')
            data_received.extend(data)
        return bytes(data_received)
    
server_address = ("127.0.0.1", 5000)
active = True
messagesToSend = Queue()

def SendMessages(socket:socket):
    while active:
        Message(messagesToSend.get().encode("utf-8")).send(socket)

def RecieveMessages(socket:socket):
    while active:
        print(bytes.decode(Message.recieve(socket), "utf-8"))

# Open Ipv4 TCP socket
with socket(AF_INET, SOCK_STREAM) as tcp_socket:
    # Connect socket to the server address
    tcp_socket.connect(server_address)
    tcp_socket.settimeout(6400)

    receiver = Thread(target=RecieveMessages, args=(tcp_socket,))
    sender = Thread(target=SendMessages, args=(tcp_socket,))
    receiver.start()
    sender.start()

    messagesToSend.put("Hey server, how are you doing?")

    # message = "Hey server, how are you doing?".encode("utf-8")
    # Message(message).send(tcp_socket)
    
    # recieved_message = Message.recieve(tcp_socket)
    # print(bytes.decode(recieved_message, "utf-8"))

    while True:
        input_string = input()
        if (input_string == "stop"): break
        messagesToSend.put(input_string)

    active = False
    tcp_socket.shutdown()
    sender.join()
    receiver.join()

tcp_socket.close()


