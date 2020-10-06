#!/usr/bin/env python

import time


class Timer:
    def __init__(self):
        self.t = 0

    def __enter__(self):
        self.t = time.time()

    def __exit__(self, exc_type, exc_value, exc_tb):
        print(time.time() - self.t)


class Rater:
    def __init__(self, cycle):
        self.fps = 0
        self.fps_cycle = cycle
        self.fps_number = 0
        self.fps_time = time.time()

    def count(self):
        self.fps_number = self.fps_number + 1
        if self.fps_number % self.fps_cycle == 0:
            time_now = time.time()
            self.fps = self.fps_cycle / (time_now - self.fps_time)
            self.fps_time = time_now

    def get(self):
        return round(self.fps)


