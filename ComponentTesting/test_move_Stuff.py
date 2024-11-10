import numpy as np
import cv2 as cv
import argparse
import serial
import time
import moteus
import math

gRodPosLow = 0
defRodPosLow = 0
mRodPosLow = 0
gRodPosHigh =  2040
defRodPosHigh = 4000
mRodPosHigh = 1225
aRodPosLow = 0
aRodPosHigh = 2510
window_capture_name = 'Video footage'
window_detection_name = 'Rod Controls'
gRodName = 'G Rod'
dRodName = 'Def Rod'
mRodName = 'Mid Rod'
aRodName = 'A Rod'
gRodPos = 0
dRodPos = 0
mRodPos = 0
aRodPos = 0

mm2step = 800/(22.5*math.pi) ##Conversion factor from mm to steps = pulley diameter *pi / steps per revolution
ENDCHAR = '#'
arduino_port = 'COM3'  # Arduino's port
baud_rate = 115200 #Arduino baud rate
ser = serial.Serial(arduino_port,baud_rate, timeout = 1)

def G_trackbar(val):
    global gRodPos
    gRodPos = val
    cv.setTrackbarPos(gRodName, window_detection_name, gRodPos)


def D_trackbar(val):
    global dRodPos
    dRodPos = val
    cv.setTrackbarPos(dRodName, window_detection_name, dRodPos)


def M_trackbar(val):
    global mRodPos
    mRodPos = val
    cv.setTrackbarPos(mRodName, window_detection_name, mRodPos)

def A_trackbar(val):
    global aRodPos
    aRodPos = val
    cv.setTrackbarPos(aRodName, window_detection_name, aRodPos)

parser = argparse.ArgumentParser(description='Code for Thresholding Operations using inRange tutorial.')
parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
args = parser.parse_args()

## [cap]
cap = cv.VideoCapture(args.camera, cv.CAP_DSHOW)
## [cap]
cv.namedWindow(window_capture_name)
cv.namedWindow(window_detection_name)
cv.createTrackbar(gRodName, window_detection_name, gRodPosLow, gRodPosHigh, G_trackbar)
cv.createTrackbar(dRodName, window_detection_name, defRodPosLow, defRodPosHigh, D_trackbar)
cv.createTrackbar(mRodName, window_detection_name, mRodPosLow, mRodPosHigh, M_trackbar)
cv.createTrackbar(aRodName, window_detection_name, aRodPosLow, aRodPosHigh, A_trackbar)

send = ''
clock = time.time()
while True:
    tick = time.time()
    ## [while]
    ret, frame = cap.read()
    if frame is None:
        break
    
    send = str(gRodPos) + '|' + str(dRodPos) + '|' + str(mRodPos) + '|' + str(aRodPos) + '|'+ ENDCHAR
    # print(send)
    if time.time() - clock > 0.1:
        ser.write(send.encode('utf-8'))
        clock = time.time()
    # ser.read_until(ENDCHAR)  # Read the data sent back by the Arduino   

    ## [show]
    cv.imshow(window_capture_name, frame)
    ## [show]

    key = cv.waitKey(30)
    if key == ord('q') or key == 27:
        break