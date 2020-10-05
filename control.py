# !/usr/bin/env python

import socket
import struct
import threading
from constant import Command

HOST = socket.gethostbyname(socket.gethostname())

command = Command()


class TcpControl:
    def __init__(self, addr=HOST, port=7777):
        self.addr = addr
        self.port = port
        self.server = None
        self.client = None

        self.set_socket()

    def set_socket(self):
        self.server = socket.socket()
        self.server.bind((self.addr, self.port))
        self.server.listen(2)

    def accept(self):
        self.client, addr = self.server.accept()
        print('client address', addr)


def run(client):
    while True:
        try:
            recv = client.recv(8)
            print('recv', recv)
            head, code, data = struct.unpack('ccHHH', recv)
            if head == command.head:
                if code == command.speed:
                    pass
                elif code == command.angle:
                    pass
                elif code == command.turn:
                    pass
                elif code == command.stop:
                    pass
            else:
                continue
        except socket.error as e:
            print(e)
            client.close()
            del client
            return
        except Exception as e:
            print(e)


def add_thread(client):
    thread_control = threading.Thread(None, target=run, args=(client,))
    thread_control.start()


# for test
if __name__ == '__main__':
    control = TcpControl()
    while True:
        try:
            control.accept()
            add_thread(control.client)
        except Exception as e:
            print(e)
