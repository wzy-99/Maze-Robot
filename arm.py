import traceback
from _XiaoRGEEK_SERVO_ import XR_Servo
from multiprocessing.connection import Listener

Servo = XR_Servo()

claw_close = 100
claw_open = 0
node1_in = 30
node2_in = 50
node1_out = 100
node2_out = 70
claw_state = 0  # 0 for close, 1 for open
arm_state = 0  # 0 for in, 1 for out

node = [0, 30, 80, 30]


def arm_init():
    Servo.XiaoRGEEK_SetServoAngle(1, node[0])  # - in + out
    Servo.XiaoRGEEK_SetServoAngle(2, node[1])
    Servo.XiaoRGEEK_SetServoAngle(3, node[2])
    Servo.XiaoRGEEK_SetServoAngle(4, node[3])


def arm_set(num, data):
    global node
    if data == 1:
        node[num] = max(0, min(180, node[num] + 1))
    elif data == 2:
        node[num] = max(0, min(180, node[num] - 1))
    else:
        pass
    Servo.XiaoRGEEK_SetServoAngle(num + 1, node[num])


def claw_turn():
    global claw_state
    if claw_state:  # if open
        node[3] = claw_close
        Servo.XiaoRGEEK_SetServoAngle(4, claw_close)
    else:
        node[3] = claw_open
        Servo.XiaoRGEEK_SetServoAngle(4, claw_open)
    claw_state = 1 - claw_state


def arm_turn():
    global arm_state
    if arm_state: # if out
        node[1] = node2_in
        node[0] = node1_in
        Servo.XiaoRGEEK_SetServoAngle(2, node2_in)
        Servo.XiaoRGEEK_SetServoAngle(1, node1_in)
    else: # else in
        node[1] = node2_out
        node[0] = node1_out
        Servo.XiaoRGEEK_SetServoAngle(2, node2_out)
        Servo.XiaoRGEEK_SetServoAngle(1, node1_out)
    arm_state = 1 - arm_state


def echo_client(conn):
    try:
        while True:
            _recv_ = conn.recv()
            print('recv', _recv_)
            number, data = _recv_
            if number < 10:
                if data == 0:
                    claw_turn()
                elif data == 1:
                    arm_turn()
            else:
                number = number - 10
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


# 30 50 in
# 100 70 out
