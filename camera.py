# !/usr/bin/env python

import cv2


class Camera:
    def __init__(self, width, height, device):
        self.width = width
        self.height = height
        self.device = device
        self.cap = None

        self.init()

    def init(self):
        self.cap = cv2.VideoCapture(self.device)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)

    def read(self):
        try:
            return self.cap.read()
        except Exception as e:
            print(e)

    def set_height(self, height):
        self.height = height
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def set_width(self, width):
        self.width = width
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)

    def __del__(self):
        self.cap.release()
