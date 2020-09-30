# !/usr/bin/env python

# import pyserial
from constant import Command


class Serial:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud

    def init(self):
        pass

    def open(self):
        pass

    def close(self):
        pass

    def read(self, size):
        pass

    def write(self, size):
        pass

    def __del__(self):
        pass


class SerialSolution:
    def __init__(self, port, baud):
        self.ser = Serial(port, baud)

    def connect(self):
        pass

    def disconnect(self):
        pass

    def formatting(self, code, data):
        pass

    def parse(self):
        pass

    def send(self, msg):
        pass

    def recv(self):
        pass

    def subscribe(self):
        pass

    def publish(self):
        pass

    def run(self):
        pass

    def __del__(self):
        del self.ser
