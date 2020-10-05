# !/usr/bin/env python

import socket


class Client:
    def __init__(self, addr, port):
        self.addr = addr
        self.port = port


class Server:
    def __init__(self, addr, port):
        self.addr = addr
        self.port = port
        self.ser = None
        self.cli = None

        self.set_socket()

    def set_socket(self):
        self.ser = socket.socket()
        self.ser.bind((self.addr, self.port))
        self.ser.listen(2)

    def accept(self):
        cli_, addr_ = self.ser.accept()
        self.cli = cli_
        return cli_


class FPV:
    def __init__(self):
        pass


class Manipulate:
    def __init__(self):
        pass

