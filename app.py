import os
import time
import serial
from fabrik import Arm

def angleString(arm):
    # List van posities. Deze posities worden later in één string aan elkaar geregen.
    positions = []

    # Format eerst de z-as van de robot.
    if (0 <= arm.zAngle <= 180):
        substring = '{}:{}&'.format(1, int(arm.zAngle))
        positions.append(substring)
    else:
        print("Servo angle must be an integer between 0 and 180.\n")

    # Format de hoek van elk segment.
    for i in range(len(arm.segments)):
        if (0 <= arm.segments[i].angle <= 180):
            substring = '{}:{}&'.format(i + 2, int(arm.segments[i].angle))
            positions.append(substring)
        elif ():
            angle = 180 - abs(arm.segments[i].angle)
            substring = '{}:{}&'.format(i + 2, int(angle))
            positions.append(substring)
            print("Servo angle must be an integer between 0 and 180.\n")

    data = "".join(positions).strip('&')

    return data

def move(arm):
    time.sleep(1)

    output = angleString(arm)

    print(output  + '\n')

    # Stuur string van hoeken naar Arduino.
    ser.write(bytes(str(output), 'utf-8'))

    time.sleep(2)


arm = Arm()

arm.addSegment(150, 180)
arm.addSegment(150, 180)

arm.calc2D(20, 200)
arm.plt2D()

arm.calc2D(250, 10)
arm.plt2D()

# port = "COM3"
#
# # Init serial.
# ser = serial.Serial(port, 9600, timeout=1)

# # Oneindige loop...
# while True:
#
#     arm.calc2D(150, 70)
#     arm.plt2D()
#     print('move 1')
#     move(arm)
#
#     arm.calc2D(200, 0)
#     arm.plt2D()
#     print('move 2')
#     move(arm)
#
#     arm.calc2D(60, 40)
#     arm.plt2D()
#     print('move 3')
#     move(arm)
#
#     arm.calc2D(100, 220)
#     arm.plt2D()
#     print('move 4')
#     move(arm)
#
#     arm.calc2D(215, 120)
#     arm.plt2D()
#     print('move 5')
#     move(arm)
