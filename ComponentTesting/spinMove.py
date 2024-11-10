import numpy as np
import cv2 as cv
import argparse
import serial
import time
import moteus
import math
import asyncio

pi = math.pi
gRodPosLow = 0
defRodPosLow = 0
mRodPosLow = 0
gRodPosHigh =  int(2*pi *100)
defRodPosHigh = int(2*pi *100)
mRodPosHigh = int(2*pi *100)
aRodPosLow = 0
aRodPosHigh = int(2*pi *100)
window_capture_name = 'Video footage'
window_detection_name = 'Rod Controls'
gRodName = 'G Spin'
dRodName = 'Def Spin'
mRodName = 'Mid Spin'
aRodName = 'A Spin'
Angles = [0, 0, 0, 0]
max_torque = 1.2
vel_lim = 55

mm2step = 800/(23*math.pi) ##Conversion factor from mm to steps = pulley diameter *pi / steps per revolution

def G_trackbar(val):
    global Angles
    Angles[0] = val/100.0
    cv.setTrackbarPos(gRodName, window_detection_name, val)


def D_trackbar(val):
    global Angles
    Angles[1] = val/100.0
    cv.setTrackbarPos(dRodName, window_detection_name, val)


def M_trackbar(val):
    global Angles
    Angles[2] = val/100.0
    cv.setTrackbarPos(mRodName, window_detection_name, val)

def A_trackbar(val):
    global Angles
    Angles[3] = val/100.0
    cv.setTrackbarPos(aRodName, window_detection_name, val)

async def moveToStart():
    [await servo.set_stop()  for servo in controllers.values()]
    [await servo.set_position(
        position=0.0, velocity_limit=0.5, accel_limit=5.0, watchdog_timeout=math.nan)
     for servo in controllers.values()]
async def loop():
    while(1):
        ret, frame = cap.read()
        if frame is None:
            break

        for i in range(4):
            await controllers[i+1].set_position(position = Angles[i], velocity_limit=vel_lim, maximum_torque=max_torque)
        await asyncio.sleep(0.02)
        key = cv.waitKey(30)
        if key == ord('q') or key == 27:
            break

parser = argparse.ArgumentParser(description='Code for Thresholding Operations using inRange tutorial.')
parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
args = parser.parse_args()

SERVO_IDS = [2, 1, 3, 4]
qr = moteus.QueryResolution()
controllers = {x: moteus.Controller(x, query_resolution=qr) for x in SERVO_IDS}
asyncio.run(moveToStart())


## [cap]
cap = cv.VideoCapture(args.camera, cv.CAP_DSHOW)
## [cap]
cv.namedWindow(window_capture_name)
cv.namedWindow(window_detection_name)
cv.createTrackbar(gRodName, window_detection_name, gRodPosLow, gRodPosHigh, G_trackbar)
cv.createTrackbar(dRodName, window_detection_name, defRodPosLow, defRodPosHigh, D_trackbar)
cv.createTrackbar(mRodName, window_detection_name, mRodPosLow, mRodPosHigh, M_trackbar)
cv.createTrackbar(aRodName, window_detection_name, aRodPosLow, aRodPosHigh, A_trackbar)


clock = time.time()
asyncio.run(loop())