#!/usr/bin/env python

import cv2
import base64
from server import Server, get_host_ip

# global image for muilt-process
g_cap = None
g_img = None

HOST = get_host_ip()


class Camera:
    def __init__(self, width, height, device=0):
        self.width = width
        self.height = height
        self.device = device
        self.cap = None

        self.init()

    def init(self):
        global g_cap
        if g_cap is None:
            self.cap = cv2.VideoCapture(self.device)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        else:
            self.cap = g_cap

    def read(self):
        try:
            return g_img
        except Exception as e:
            print(e)

    def spin(self):
        global g_img
        g_img = self.cap.read()

    def set_height(self, height):
        self.height = height
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def set_width(self, width):
        self.width = width
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)

    def __del__(self):
        self.cap.release()


class FPV:
    def __init__(self, width, height, device=0):
        self.camera = Camera(width, height, device)
        self.server = Server(HOST, port=5555)

    def to_base64(self, img):
        _, buf = cv2.imencode(".jpg", img)
        return base64.b64encode(buf)

    def run(self, client):
        while True:
            img = g_img
            img_code = self.to_base64(img)
            try:
                recv = cli.recv(2)
                print('recv', recv)
                if recv == b'BG':  # BEGIN
                    print('send', len(img_code))
                    cli.send(struct.pack("l", len(img_code)))
                    recv = cli.recv(2)
                    print('recv', recv)
                    if recv == b'OK':
                        cli.send(img_code)
                    else:
                        continue
                elif recv == b'SP':  # STOP
                    flag = 0
                else:
                    continue
            except Exception as e:
                print(e)
                cli.close()
                return

