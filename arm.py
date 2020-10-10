import traceback
from _XiaoRGEEK_SERVO_ import XR_Servo
from multiprocessing.connection import Listener

Servo = XR_Servo()

claw_close = 80
claw_open = 0
node1_in = 0
node2_in = 50
node1_out = 180
node2_out = 180
claw_state = 0  # 0 for close, 1 for open
arm_state = 0  # 0 for in, 1 for out


def arm_init():
    Servo.XiaoRGEEK_SetServoAngle(1, 0)
    Servo.XiaoRGEEK_SetServoAngle(2, 50)
    Servo.XiaoRGEEK_SetServoAngle(3, 80)
    Servo.XiaoRGEEK_SetServoAngle(4, 80)


def arm_set(num, angle):
    Servo.XiaoRGEEK_SetServoAngle(num, angle)


def claw_turn():
    global claw_state
    if claw_state:  # if open
        Servo.XiaoRGEEK_SetServoAngle(4, claw_close)
    else:
        Servo.XiaoRGEEK_SetServoAngle(4, claw_open)
    claw_state = 1 - claw_state


def arm_turn():
    global arm_state
    if arm_state: # if out
        Servo.XiaoRGEEK_SetServoAngle(2, node2_in)
        Servo.XiaoRGEEK_SetServoAngle(1, node1_in)
    else: # else in
        Servo.XiaoRGEEK_SetServoAngle(2, node2_out)
        Servo.XiaoRGEEK_SetServoAngle(1, node1_out)
    arm_state = 1 - arm_state


def echo_client(conn):
    try:
        while True:
            _recv_ = conn.recv()
            print('recv', _recv_)
            number, data = _recv_
            if number == 0:
                if data == 0:
                    claw_turn()
                elif data == 1:
                    arm_turn()
            else:
                arm_set(number, data)
    except EOFError:
        print('Connection closed')


def echo_server(address, authkey):
    serv = Listener(address, authkey=authkey)
    while True:
        try:
            client = serv.accept()
            echo_client(client)
        except Exception as e:
            print(e)
            traceback.print_exc()


# for test
if __name__ == '__main__':
    arm_init()
    echo_server(('', 8888), authkey=b'arm')
