import os
import time
import sys
import serial
from fabrik import Arm


def restart():
        print("argv was",sys.argv)
        print("sys.executable was", sys.executable)
        print("restart now")
        os.execv(sys.executable, ['python'] + sys.argv)
        print('System restart... \n')

def angleString(arm):
    # List van posities. Deze posities worden later in één string aan elkaar geregen.
    positions = []

    # Format eerst de z-as van de robot.
    if (0 <= arm.zAngle <= 180):
        substring = '{}:{}&'.format(1, int(arm.zAngle))
        positions.append(substring)
    else:
        print("Servo angle must be an integer between 0 and 180.\n")
        restart()

    # Format de hoek van elk segment.
    for i in range(len(arm.segments)):
        if (0 <= arm.segments[i].angle <= 180):
            substring = '{}:{}&'.format(i + 2, int(arm.segments[i].angle))
            positions.append(substring)
        else:
            print("Servo angle must be an integer between 0 and 180.\n")
            restart()

    data = "".join(positions).strip('&')

    return data

def move(arm):

    output = angleString(arm)

    print(output  + '\n')

    # Stuur string van hoeken naar Arduino.
    ser.write(bytes(str(output), 'utf-8'))

    time.sleep(2)

try:
    port = "COM3"

    # Init serial.
    ser = serial.Serial(port, 9600, timeout=1)

    print('Starting connection... \n')
    time.sleep(3)

    # Oneindige loop...
    while True:
        print('Making new arm... \n')

        arm = Arm()

        arm.addSegment(150, 180)
        arm.addSegment(150, 180)

        arm.calc2D(150, 70)
        # arm.plt2D()
        arm.zAngle = 0
        print('move 1')
        move(arm)

        arm.calc2D(200, 0)
        # arm.plt2D()
        arm.zAngle = 180
        print('move 2')
        move(arm)

        arm.calc2D(60, 40)
        # arm.plt2D()
        arm.zAngle = 30
        print('move 3')
        move(arm)

        arm.calc2D(100, 220)
        # arm.plt2D()
        arm.zAngle = 120
        print('move 4')
        move(arm)

        arm.calc2D(215, 120)
        # arm.plt2D()
        arm.zAngle = 60
        print('move 5')
        move(arm)

        del arm

except Exception as e:
    print('An error occured: {}'.format(e))
    restart()
