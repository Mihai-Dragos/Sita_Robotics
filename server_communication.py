from TCP_client import *

connection = create_tcp_connection()

connection.send_message(str.encode("message", "utf-8"))