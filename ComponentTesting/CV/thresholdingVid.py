from __future__ import print_function
import cv2 as cv
import argparse
import numpy as np
import time
max_value = 255
max_value_H = 360//2
low_H = 0
low_S = 0
low_V = 0
high_H = max_value_H
high_S = max_value  
high_V = max_value
window_capture_name = 'Video Capture'
window_detection_name = 'Object Detection'
low_H_name = 'Low H'
low_S_name = 'Low S'
low_V_name = 'Low V'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'
def loadData():
    file_path = 'calibration_data.xml'
    # Load the data
    fs = cv.FileStorage(file_path, cv.FILE_STORAGE_READ)
    camera_matrix = fs.getNode('camera_matrix').mat()
    dist_coeffs = fs.getNode('dist_coeffs').mat()

    # Read rvecs and tvecs
    rvecs = []
    tvecs = []

    rvecs_node = fs.getNode("rvecs")
    for i in range(rvecs_node.size()):
        rvecs.append(rvecs_node.at(i).mat())

    tvecs_node = fs.getNode("tvecs")
    for i in range(tvecs_node.size()):
        tvecs.append(tvecs_node.at(i).mat())

    fs.release()
    return camera_matrix, dist_coeffs, rvecs, tvecs
def undistort(frame):
    # Load the data
    camera_matrix, dist_coeffs, rvecs, tvecs = loadData()
    h, w = frame.shape[:2]
    new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
    undistorted_image = cv.undistort(frame, camera_matrix, dist_coeffs, None, new_camera_matrix)
    # Undistort the frame
    undistorted_frame = cv.undistort(frame, camera_matrix, dist_coeffs)
    return undistorted_frame
## [low]
def on_low_H_thresh_trackbar(val):
    global low_H
    global high_H
    low_H = val
    low_H = min(high_H-1, low_H)
    cv.setTrackbarPos(low_H_name, window_detection_name, low_H)
## [low]

## [high]
def on_high_H_thresh_trackbar(val):
    global low_H
    global high_H
    high_H = val
    high_H = max(high_H, low_H+1)
    cv.setTrackbarPos(high_H_name, window_detection_name, high_H)
## [high]

def on_low_S_thresh_trackbar(val):
    global low_S
    global high_S
    low_S = val
    low_S = min(high_S-1, low_S)
    cv.setTrackbarPos(low_S_name, window_detection_name, low_S)

def on_high_S_thresh_trackbar(val):
    global low_S
    global high_S
    high_S = val
    high_S = max(high_S, low_S+1)
    cv.setTrackbarPos(high_S_name, window_detection_name, high_S)

def on_low_V_thresh_trackbar(val):
    global low_V
    global high_V
    low_V = val
    low_V = min(high_V-1, low_V)
    cv.setTrackbarPos(low_V_name, window_detection_name, low_V)

def on_high_V_thresh_trackbar(val):
    global low_V
    global high_V
    high_V = val
    high_V = max(high_V, low_V+1)
    cv.setTrackbarPos(high_V_name, window_detection_name, high_V)

def measureLoopTime(tick):
        global tockercounter
        global tickertocker
        tock = time.time()
        tickertocker = tickertocker + tock - tick
        tockercounter = 1 + tockercounter
        if tockercounter > 100:
            print(tickertocker / tockercounter)
            tockercounter = 0
            tickertocker = 0
# parser = argparse.ArgumentParser(description='Code for Thresholding Operations using inRange tutorial.')
# parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
# args = parser.parse_args()

## [cap]
# cap = cv.VideoCapture(args.camera, cv.CAP_DSHOW)
## [cap]

video_path = 'Sample_Video/Normal conditions1.mp4'
cap = cv.VideoCapture(video_path)
# Check if the video was opened successfully
if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

## [window]
cv.namedWindow(window_capture_name)
cv.namedWindow(window_detection_name)
        # b_low_H = 11
        # b_high_H = 22
        # b_low_S = 207
        # b_high_S = 255
        # b_low_V = 51
        # b_high_V = 193
b_thresholds = [3,
53,
24,
189,
14,
251
    ]

start_low_H = b_thresholds[0]
start_high_H = b_thresholds[1]
start_low_S = b_thresholds[2]
start_high_S = b_thresholds[3]
start_low_V = b_thresholds[4]
start_high_V = b_thresholds[5]

# 0
# 144
# 60
# 222
# 0
# 240

## [trackbar]
cv.createTrackbar(low_H_name, window_detection_name , start_low_H, max_value_H, on_low_H_thresh_trackbar)
cv.createTrackbar(high_H_name, window_detection_name , start_high_H, max_value_H, on_high_H_thresh_trackbar)
cv.createTrackbar(low_S_name, window_detection_name , start_low_S, max_value, on_low_S_thresh_trackbar)
cv.createTrackbar(high_S_name, window_detection_name , start_high_S, max_value, on_high_S_thresh_trackbar)
cv.createTrackbar(low_V_name, window_detection_name , start_low_V, max_value, on_low_V_thresh_trackbar)
cv.createTrackbar(high_V_name, window_detection_name , start_high_V, max_value, on_high_V_thresh_trackbar)
## [trackbar]

frame_width = 480
frame_height = 640
global tockercounter
global tickertocker

tockercounter = 0
tickertocker = 0
while True:
    tick = time.time()
    ## [while]
    ret, frame = cap.read()
    if frame is None:
        break
    # frame = undistort(frame)
    resized_frame = cv.resize(frame, (frame_height, frame_width))
    frame = resized_frame

    
    # frame = cv.createBackgroundSubtractorMOG2().apply(frame)
    # DETECT EDGESS  TAKES 50 MS??!?! WTF
    # low_threshold = 63

    # img_blur = cv.blur(frame, (3,3))
    # detected_edges = cv.Canny(img_blur, low_threshold, low_threshold*3, 3)
    # mask = detected_edges != 0
    # dst = frame * (mask[:,:,None].astype(frame.dtype))
    # frame = dst
    
    frame_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    # frame_HSV = cv.GaussianBlur(frame_HSV,(3,3),0)
    # frame_HSV = cv.medianBlur(frame_HSV,3)
	
    frame_threshold = cv.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))
    ## [while]

    ## [show]
    cv.imshow(window_capture_name, frame)
    cv.imshow(window_detection_name, frame_threshold)
    ## [show]
    measureLoopTime(tick)
    key = cv.waitKey(30)
    if key == ord('q') or key == 27:
        break
    if key == ord('s'):
        print(low_H)
        print(high_H)
        print(low_S)
        print(high_S)
        print(low_V)
        print(high_V)