#!/usr/bin/env python

import time
import math
from constant import StateEnum
from pid import PD

state = StateEnum()


# In this class, kinematic control method is defined, such as turn back, turn right and so on.
# And you should know that belowed methond is high-level control one rather than basic control methond.
class KinematicControl:
    def __init__(self, motor_instance, encode_instance=None):

        # config variable
        self.turn_speed = 90
        # self.turn_dtime = 1.05
        self.turn_right_time = 0.85
        self.turn_left_time = 0.70
        self.run_time = 1.55
        self.run_unblock_time = 0.1
        self.turn_unblock_time = 0.1
        self.run_distance = 1350
        self.turn_right_distance = 650
        self.turn_left_distance = 565
        self.adjust_k = 0.05
        self.angle_pd = PD(kd=-1.0, kp=-1.0, target=50.0)

        # motor instance
        self.left_motor, self.right_motor = motor_instance
        self.left_encode, self.right_encode = encode_instance

        # inital state is stop
        # inital speed is 100
        # inital angle is 50

        # local variable
        self.angle = 50
        self.speed = 100
        self.left_speed = 0
        self.right_speed = 0
        self.state = state.stop
        # self.last_state = state.stop
        self.state_change = False
        self.start_time = 0
        self.end_time = 0
        self.start_distance = 0
        self.end_distance = 0
        self.block = False
        self.finish = True

    def set_state(self, _state_):
        if self.state != _state_ and self.block is False:
            self.state = _state_
            self.state_change = True

    def set_angle(self, angle=50):
        self.angle = int(max(0, min(angle, 100)))

    def set_speed(self, speed=0):
        self.speed = int(max(0, min(speed, 100)))

    def check_finish(self):
        if self.finish:
            self.finish = False
            return True
        else:
            return False

    def turn_right_time_(self):
        if self.state_change:
            self.left_motor.set_forword()
            self.right_motor.set_back()
            self.state_change = False
            self.left_speed = self.turn_speed
            self.right_speed = self.turn_speed
            self.left_motor.set_speed(self.left_speed)
            self.right_motor.set_speed(self.right_speed)
            self.start_time = time.time()
            self.end_time = self.start_time + self.turn_right_time
            self.block = True
        else:
            if time.time() < self.end_time:
                self.left_speed = self.turn_speed
                self.right_speed = self.turn_speed
                self.left_motor.set_speed(self.left_speed)
                self.right_motor.set_speed(self.right_speed)
            else:
                self.start_time = 0
                self.end_time = 0
                self.state = state.run_forward_time
                self.state_change = True
                self.block = False

    def turn_left_time_(self):
        if self.state_change:
            self.left_motor.set_back()
            self.right_motor.set_forword()
            self.state_change = False
            self.left_speed = self.turn_speed
            self.right_speed = self.turn_speed
            self.left_motor.set_speed(self.left_speed)
            self.right_motor.set_speed(self.right_speed)
            self.start_time = time.time()
            self.end_time = self.start_time + self.turn_left_time
            self.block = True
        else:
            if time.time() < self.end_time:
                self.left_speed = self.turn_speed
                self.right_speed = self.turn_speed
                self.left_motor.set_speed(self.left_speed)
                self.right_motor.set_speed(self.right_speed)
            else:
                self.start_time = 0
                self.end_time = 0
                self.state = state.run_forward_time
                self.state_change = True
                self.block = False

    def run_forward_time_(self):
        if self.state_change:
            self.left_motor.set_forword()
            self.right_motor.set_forword()
            self.state_change = False
            self.start_time = time.time()
            self.end_time = self.start_time + self.run_time
            self.block = True
        else:
            if time.time() < self.end_time:
                self.dif_speed(self.speed, self.angle)
                self.left_motor.set_speed(self.left_speed)
                self.right_motor.set_speed(self.right_speed)
            else:
                self.start_time = 0
                self.end_time = 0
                self.state = state.stop
                self.state_change = True
                self.block = False
                self.finish = True

    def run_back_time_(self):
        if self.state_change:
            self.left_motor.set_back()
            self.right_motor.set_back()
            self.state_change = False
            self.start_time = time.time()
            self.end_time = self.start_time + self.run_time
            self.block = True
        else:
            if time.time() < self.end_time:
                self.dif_speed(self.speed, 100 - self.angle)
                self.left_motor.set_speed(self.left_speed)
                self.right_motor.set_speed(self.right_speed)
            else:
                self.start_time = 0
                self.end_time = 0
                self.state = state.stop
                self.state_change = True
                self.block = False
                self.finish = True

    def turn_right_distance_(self):
        if self.state_change:
            self.left_motor.set_forword()
            self.right_motor.set_back()
            self.state_change = False
            self.left_speed = self.turn_speed
            self.right_speed = self.turn_speed
            self.left_motor.set_speed(self.left_speed)
            self.right_motor.set_speed(self.right_speed)
            self.left_encode.clear()
            self.right_encode.clear()
            self.start_distance = (abs(self.left_encode.get_count()) +
                                   abs(self.right_encode.get_count())) // 2
            self.end_distance = self.start_distance + self.turn_right_distance
            self.block = True
        else:
            cout = (abs(self.left_encode.get_count()) +
                    abs(self.right_encode.get_count())) // 2
            if cout < self.end_distance:
                self.left_speed = self.turn_speed
                self.right_speed = self.turn_speed
                self.left_motor.set_speed(self.left_speed)
                self.right_motor.set_speed(self.right_speed)
            else:
                self.start_distance = 0
                self.end_distance = 0
                self.state = state.run_forward_distance
                self.state_change = True
                self.block = False

    def turn_left_distance_(self):
        if self.state_change:
            self.left_motor.set_back()
            self.right_motor.set_forword()
            self.state_change = False
            self.left_speed = self.turn_speed
            self.right_speed = self.turn_speed
            self.left_motor.set_speed(self.left_speed)
            self.right_motor.set_speed(self.right_speed)
            self.left_encode.clear()
            self.right_encode.clear()
            self.start_distance = (abs(self.left_encode.get_count()) +
                                   abs(self.right_encode.get_count())) // 2
            self.end_distance = self.start_distance + self.turn_left_distance
            self.block = True
        else:
            cout = (abs(self.left_encode.get_count()) +
                    abs(self.right_encode.get_count())) // 2
            if cout < self.end_distance:
                self.left_speed = self.turn_speed
                self.right_speed = self.turn_speed
                self.left_motor.set_speed(self.left_speed)
                self.right_motor.set_speed(self.right_speed)
            else:
                self.start_distance = 0
                self.end_distance = 0
                self.state = state.run_forward_distance
                self.state_change = True
                self.block = False

    def run_forward_distance_(self):
        if self.state_change:
            self.left_motor.set_forword()
            self.right_motor.set_forword()
            self.state_change = False
            self.left_encode.clear()
            self.right_encode.clear()
            self.start_distance = self.left_encode.get_count()
            self.end_distance = self.start_distance + self.run_distance
            self.block = True
        else:
            if self.left_encode.get_count() < self.end_distance:
                self.dif_speed(self.speed, self.angle)
                self.left_motor.set_speed(self.left_speed)
                self.right_motor.set_speed(self.right_speed)
            else:
                self.start_distance = 0
                self.end_distance = 0
                self.state = state.stop
                self.state_change = True
                self.block = False
                self.finish = True

    def run_back_distance_(self):
        if self.state_change:
            self.left_motor.set_back()
            self.right_motor.set_back()
            self.state_change = False
            self.left_encode.clear()
            self.right_encode.clear()
            self.start_distance = self.left_encode.get_count()
            self.end_distance = self.start_distance + self.run_distance
            self.block = True
        else:
            if abs(self.left_encode.get_count()) < self.end_distance:
                self.dif_speed(self.speed, 100 - self.angle)
                self.left_motor.set_speed(self.left_speed)
                self.right_motor.set_speed(self.right_speed)
            else:
                self.start_distance = 0
                self.end_distance = 0
                self.state = state.stop
                self.state_change = True
                self.block = False
                self.finish = True

    def run_forward_unblock_(self):
        if self.state_change:
            self.left_motor.set_forword()
            self.right_motor.set_forword()
            self.state_change = False
            self.start_time = time.time()
            self.end_time = self.start_time + self.run_unblock_time
        else:
            if time.time() < self.end_time:
                self.dif_speed(self.speed, self.angle)
                self.left_motor.set_speed(self.left_speed)
                self.right_motor.set_speed(self.right_speed)
            else:
                self.start_time = 0
                self.end_time = 0
                self.state = state.stop
                self.state_change = True

    def run_back_unblock_(self):
        if self.state_change:
            self.left_motor.set_back()
            self.right_motor.set_back()
            self.state_change = False
            self.start_time = time.time()
            self.end_time = self.start_time + self.run_unblock_time
        else:
            if time.time() < self.end_time:
                self.dif_speed(self.speed, 100 - self.angle)
                self.left_motor.set_speed(self.left_speed)
                self.right_motor.set_speed(self.right_speed)
            else:
                self.start_time = 0
                self.end_time = 0
                self.state = state.stop
                self.state_change = True

    def turn_right_unblock_(self):
        if self.state_change:
            self.left_motor.set_forword()
            self.right_motor.set_back()
            self.state_change = False
            self.left_speed = self.turn_speed
            self.right_speed = self.turn_speed
            self.left_motor.set_speed(self.left_speed)
            self.right_motor.set_speed(self.right_speed)
            self.start_time = time.time()
            self.end_time = self.start_time + self.turn_unblock_time
        else:
            if time.time() < self.end_time:
                self.left_speed = self.turn_speed
                self.right_speed = self.turn_speed
                self.left_motor.set_speed(self.left_speed)
                self.right_motor.set_speed(self.right_speed)
            else:
                self.start_time = 0
                self.end_time = 0
                self.state = state.stop
                self.state_change = True
                self.block = False

    def turn_left_unblock_(self):
        if self.state_change:
            self.left_motor.set_back()
            self.right_motor.set_forword()
            self.state_change = False
            self.left_speed = self.turn_speed
            self.right_speed = self.turn_speed
            self.left_motor.set_speed(self.left_speed)
            self.right_motor.set_speed(self.right_speed)
            self.start_time = time.time()
            self.end_time = self.start_time + self.turn_unblock_time
        else:
            if time.time() < self.end_time:
                self.left_speed = self.turn_speed
                self.right_speed = self.turn_speed
                self.left_motor.set_speed(self.left_speed)
                self.right_motor.set_speed(self.right_speed)
            else:
                self.start_time = 0
                self.end_time = 0
                self.state = state.stop
                self.state_change = True

    def stop_(self):
        if self.state_change:
            self.left_speed = 0
            self.right_speed = 0
            self.left_motor.set_speed(self.left_speed)
            self.right_motor.set_speed(self.right_speed)
            self.left_encode.clear()
            self.right_encode.clear()
            self.state_change = False
        else:
            self.left_speed = 0
            self.right_speed = 0
            self.left_motor.set_speed(self.left_speed)
            self.right_motor.set_speed(self.right_speed)

    def adjust_angle(self, left_distance, right_distance):
        if self.state == state.run_forward_time or \
                self.state == state.run_back_time:
            d_distance = left_distance - right_distance
            self.angle = self.angle_pd.update(d_distance * self.adjust_k)

    def dif_speed(self, speed, angle):
        # TODO dif speed algorithm
        angle = (1 - angle / 100.0) * math.pi
        # y_speed = speed * math.sin(angle)
        x_speed = speed * math.cos(angle)
        # when car turn left, angle > 90, x_speed < 0;
        # when car turn right, angle < 90, x_speed > 0.
        self.left_speed = self.speed + x_speed // 2
        self.right_speed = self.speed - x_speed // 2
        self.left_speed = int(max(0, min(100, self.left_speed)))
        self.right_speed = int(max(0, min(100, self.right_speed)))

    def spin(self):
        if self.state == state.run_forward_time:
            self.run_forward_time_()
        elif self.state == state.run_back_time:
            self.run_back_time_()
        elif self.state == state.turn_left_time:
            self.turn_left_time_()
        elif self.state == state.turn_right_time:
            self.turn_right_time_()
        # elif self.state == state.turn_back:
        #     self.turn_back_()
        elif self.state == state.stop:
            self.stop_()
        elif self.state == state.run_forward_unblock:
            self.run_forward_unblock_()
        elif self.state == state.run_back_unblock:
            self.run_back_unblock_()
        elif self.state == state.turn_left_unblock:
            self.turn_left_unblock_()
        elif self.state == state.turn_right_unblock:
            self.turn_right_unblock_()
        elif self.state == state.run_forward_distance:
            self.run_forward_distance_()
        elif self.state == state.run_back_distance:
            self.run_back_distance_()
        elif self.state == state.turn_right_distance:
            self.turn_right_distance_()
        elif self.state == state.turn_left_distance:
            self.turn_left_distance_()
        self.left_motor.spin()
        self.right_motor.spin()
        # show(self)


def show(ki):
    print('speed', ki.speed)
    print('angle', ki.angle)
    print('state', ki.state)
    print('chang', ki.state_change)
    print('left ', ki.left_speed, ki.left_motor.speed, ki.left_motor.pwm_value)
    print('right', ki.right_speed, ki.right_motor.speed, ki.right_motor.pwm_value)
    print('time1', ki.end_time, 'time2', time.time())


# for test
if __name__ == '__main__':
    import RPi.GPIO as GPIO
    from constant import Pin, StateEnum
    from peripheral import MotorOpen

    state = StateEnum()
    pin_conf = Pin()
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    motor_left = MotorOpen(pin_conf.left_pin1, pin_conf.left_pin2, pin_conf.left_enale)
    motor_right = MotorOpen(pin_conf.right_pin1, pin_conf.right_pin2, pin_conf.right_enale)
    kinematic = KinematicControl((motor_left, motor_right))


    def test(dtime=5):
        end_time = time.time() + dtime
        while time.time() < end_time:
            print('speed', kinematic.speed)
            print('angle', kinematic.angle)
            print('state', kinematic.state)
            print('chang', kinematic.state_change)
            print('left ', kinematic.left_speed, kinematic.left_motor.speed, kinematic.left_motor.pwm_value)
            print('right', kinematic.right_speed, kinematic.right_motor.speed, kinematic.right_motor.pwm_value)
            print('time1', kinematic.end_time, 'time2', time.time())
            print('****************************')
            kinematic.spin()


    kinematic.set_state(state.run_forward)
    test(2)
    kinematic.set_state(state.turn_left)
    test(2)

# Car will turn left, turn right, (turn back,) go ahead, go back and stop.
# When car turn left or turn right, main program will be blocked until turn over.
# When car go ahead or go back, speed parameter and angle parameter should be given,
# which are computed by deviation distance between two sides.
# Thus, a deviation node should be create to computed this and can be blocked when needed.
# And turn left or turn left can be called by maze solution node.
