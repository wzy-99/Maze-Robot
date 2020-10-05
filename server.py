import socket


class Server:
    def __init__(self, addr, port):
        self.addr = addr
        self.port = port
        self.server = None

        self.set_socket()

    def set_socket(self):
        self.server = socket.socket()
        self.server.bind((self.addr, self.port))
        self.server.listen(2)

    def accept(self):
        client, addr = self.server.accept()
        print('client address', addr)
        return client
