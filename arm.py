from _XiaoRGEEK_SERVO_ import XR_Servo

Servo = XR_Servo()

claw_num = 1
claw_close = 160
claw_open = 0
claw_state = 0  # 0 for close, 1 for open


def arm_init():
    Servo.XiaoRGEEK_SetServoAngle(1, 90)
    Servo.XiaoRGEEK_SetServoAngle(2, 90)
    Servo.XiaoRGEEK_SetServoAngle(3, 90)
    Servo.XiaoRGEEK_SetServoAngle(4, 90)


def arm_set(num, angle):
    Servo.XiaoRGEEK_SetServoAngle(num, angle)


def claw_turn():
    global claw_state
    if claw_state:  # if open
        Servo.XiaoRGEEK_SetServoAngle(claw_num, claw_close)
    else:
        Servo.XiaoRGEEK_SetServoAngle(claw_num, claw_open)
    claw_state = 1 - claw_state


# for test
if __name__ == '__main__':
    arm_init()
