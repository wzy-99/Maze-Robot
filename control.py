
import socket
from tcp import Server

HOST = socket.gethostbyname(socket.gethostname())


class TcpControl:
    def __init__(self, addr=HOST, port=7777):
        self.addr = addr
        self.port = port
        self.server = None
        self.client = None

        self._set_socket()

    def _set_socket(self):
        self.server = socket.socket()
        self.server.bind((self.addr, self.port))
        self.server.listen(2)

    def accept(self):
        self.client, addr = self.server.accept()
        print('client address:', addr)

    def run(self):
