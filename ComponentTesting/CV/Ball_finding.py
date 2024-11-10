from __future__ import print_function
import cv2 as cv
import argparse
import numpy as np
import time
import concurrent.futures

global threshold

window_capture_name = 'Video Feed'
window_detection_name = 'Object Detection'
def on_threshold_change(val):
    global threshold
    threshold = val / 100
# Create a trackbar for the threshold value
threshold_name = 'Threshold Value(%)'
def findLED(mask, roi_x, roi_y):
        LEDtemplate = cv.imread('C:/Users/juls6/Desktop/Classes/FOOSTABLE/Software/liveFrames/Template_62.png', cv.IMREAD_GRAYSCALE)
        current_frame = mask[:,:,1]
        LED_threshold = 0.3
        #given the mask of an roi return the (x, y) coordinates of the LED
        #mask is the mask of the roi
        #roi_x is the x coordinate of the roi in the original image
        #roi_y is the y coordinate of the roi in the original image
        #returns the (x, y) coordinates of the LED in the original image

        # Find contours of green LEDs
        # Sort the contours based on area in descending order
        # contours = sorted(contours, key=cv.contourArea, reverse=True)

        #loop over the top 5 contours and find the LED x, y coordinates
        results = cv.matchTemplate(current_frame, LEDtemplate, cv.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv.minMaxLoc(results)
        if max_val > LED_threshold:
            top_left = max_loc
            w, h = LEDtemplate.shape
            bottom_right = (top_left[0] + w, top_left[1] + h)
            # cv.rectangle(mask, top_left, bottom_right, 255, 2)
            LED = [top_left[0] + w//2 + roi_y, top_left[1] + h//2+roi_x]
            # cv.imshow("LED", mask)
            return LED
        else:
            LED = [0, 0]
        return LED
def getHomographyMatrix(corners_image):
        #takes the current frame and the corners of the table in the image and returns the homography matrix
        #all "dest points" are a TOP DOWN perspective of the table
        #assume the table is 660.4mm x 1203.325mm
        # topRight = [0, 1203.325]
        # topLeft = [0,0]
        # bottomRight = [660.4, 1203.325]
        # bottomLeft = [660.4, 0]
        #Actual measurements of the table
        # topRight = [1190.625, 25.4]
        # topLeft = [12.7,19.75]
        # bottomRight = [1196.975, 647.7]
        # bottomLeft = [19.75, 638.175]
        
        corners_real = np.array([[19.75, 638.175], [1196.975, 647.7], [12.7,19.75], [1181.1, 19.05]])
        topLeft = corners_real[0]
        topRight = corners_real[1]
        bottomLeft = corners_real[2]
        bottomRight = corners_real[3]

        destPoints = np.array([topLeft, topRight, bottomLeft, bottomRight])
        homographyMatrix, _ = cv.findHomography(corners_image, destPoints)
        return homographyMatrix
def findCorners(current_frame):
        global PREV_corners_image
        #frame is the current image frame
        #returns a 4 x 2 array of the (x, y) coordinates of the corners of the table IN THE IMAGE
        # current_frame = self.current_frame

        percentofCorner = 0.2
        frame_HSV = cv.cvtColor(current_frame, cv.COLOR_BGR2HSV)

        height, width, _ = current_frame.shape


        # print(frame_HSV.shape)

        ##Need 4 roi for each of the corners            

        #roi1 is the bottom left corner (GREEN LED ON THE TOP LEFT SIDE OF THE SCREEN)
        roi1 = frame_HSV[0:int(height*percentofCorner), 0:int(width*percentofCorner)]
        #roi2 is the bottom right corner (GREEN LED ON THE TOP RIGHT SIDE OF THE SCREEN)
        roi2 = frame_HSV[int(height*0):int(height*percentofCorner), int(width*(1-percentofCorner)):width]
        #roi3 is the top left corner (GREEN LED ON THE BOTTOM LEFT SIDE OF THE SCREEN)
        roi3 = frame_HSV[int(height*(1-percentofCorner)):int(height*1), 0:int(width*percentofCorner)]
        #roi4 is the top right corner (GREEN LED ON THE BOTTOM RIGHT SIDE OF THE SCREEN)
        roi4 = frame_HSV[int(height*(1-percentofCorner)):int(height*1), int(width*(1-percentofCorner)):width]
        #GREEN LED:
        #Thresholds are for normal camera settings
        # Define lower and upper thresholds for green LED  
        greenthresholds = [7,
129,
78,
255,
12,
255]
        green_low_H = greenthresholds[0]
        green_high_H = greenthresholds[1]
        green_low_S = greenthresholds[2]
        green_high_S = greenthresholds[3]
        green_low_V = greenthresholds[4]
        green_high_V = greenthresholds[5]
        # Create masks for green LEDs using thresholds
        green_mask1 = cv.inRange(roi1, (green_low_H, green_low_S, green_low_V), (green_high_H, green_high_S, green_high_V))
        green_mask2 = cv.inRange(roi2, (green_low_H, green_low_S, green_low_V), (green_high_H, green_high_S, green_high_V))
        green_mask3 = cv.inRange(roi3, (green_low_H, green_low_S, green_low_V), (green_high_H, green_high_S, green_high_V))
        green_mask4 = cv.inRange(roi4, (green_low_H, green_low_S, green_low_V), (green_high_H, green_high_S, green_high_V))

        # apply mask to HSV frame and spit out S part for analysis
        roi1 = cv.bitwise_and(roi1, roi1, mask=green_mask1)
        roi2 = cv.bitwise_and(roi2, roi2, mask=green_mask2)
        roi3 = cv.bitwise_and(roi3, roi3, mask=green_mask3)
        roi4 = cv.bitwise_and(roi4, roi4, mask=green_mask4)
        # Find the (x, y) coordinates of the green LEDs using findLED
        LED1 = findLED(roi1, 0, 0)
        LED2 = findLED(roi2, int(width* (1-percentofCorner)), 0)
        LED3 = findLED(roi3, 0, int(height*(1-percentofCorner)))
        LED4 = findLED(roi4, int(width*(1-percentofCorner)), int(height*(1-percentofCorner)))

        #stack all LED positions into a 4x2 array
        corners_image = np.array([LED1, LED2, LED3, LED4])
        print(corners_image)
        #make sure no corners are (0, 0) and if they are replace them with the previous found value
        count=0
        for corner in corners_image:
            if corner[0] == 0 and corner[1] == 0:
                corner[0] = PREV_corners_image[count][0]
                corner[1] = PREV_corners_image[count][1]
                corners_image[count] = PREV_corners_image[count]
            count += 1
        # self.corners_image = corners_image

        # froi1 = current_frame[0:int(height*percentofCorner), 0:int(width*percentofCorner)]
        # froi2 = current_frame[int(height*0):int(height*percentofCorner), int(width*(1-percentofCorner)):width]
        # froi3 = current_frame[int(height*(1-percentofCorner)):int(height*1), 0:int(width*percentofCorner)]
        # froi4 = current_frame[int(height*(1-percentofCorner)):int(height*1), int(width*(1-percentofCorner)):width]
        # height, width = roi1.shape[:2]
        # roi2 = cv.resize(roi2, (width, height))
        # roi3 = cv.resize(roi3, (width, height))
        # roi4 = cv.resize(roi4, (width, height))

        # top_row = cv.hconcat([roi1, roi2])
        # bottom_row = cv.hconcat([roi3, roi4])
        # cornerFrame = cv.vconcat([top_row, bottom_row])
        
        # top_row = cv.hconcat([froi1, froi2])
        # bottom_row = cv.hconcat([froi3, froi4])
        # rois = cv.vconcat([top_row, bottom_row])

        # top_row = cv.hconcat([green_mask1, green_mask2])
        # bottom_row = cv.hconcat([green_mask3, green_mask4])
        # masks = cv.vconcat([top_row, bottom_row])
        # cv.imshow("masks",masks)
        # cv.imshow('rois', cornerFrame)
        # cv.imshow('LEDs', rois)
        # cv.imshow('Corners', frame_HSV) 
        return corners_image
def match_template(template, frame, method=cv.TM_CCORR_NORMED):
    return cv.matchTemplate(frame, template, method)
def parallel_template_matching(templates, frame):
    # Use ThreadPoolExecutor to parallelize across cores
    with concurrent.futures.ThreadPoolExecutor() as executor:
        # Submit tasks to match each template
        results = list(executor.map(lambda tpl: match_template(tpl, frame), templates))
    best_match_score = 0
    best_match_location = (0, 0)
    best_template = None
    min_val, max_val, min_loc, max_loc = None, None, None, None
    best_result = None
    for result, template in zip(results, templates):
        min_val, max_val, min_loc, max_loc = cv.minMaxLoc(result)
        if max_val > best_match_score:
            best_match_score = max_val
            best_match_location = max_loc
            best_template = template
    # for result, template in zip(results, templates):
    #     min_val, max_val, min_loc, max_loc = cv.minMaxLoc(result)
    #     if min_val > best_match_score:
    #         best_match_score = min_val
    #         best_match_location = min_loc
    #         best_template = template
    return best_template, best_match_location
def findballxy(current_frame):

    b_thresholds = [3,
53,
9,
255,
14,
251,
    ]
    b_lowH = b_thresholds[0]
    b_highH = b_thresholds[1]
    b_lowS = b_thresholds[2]
    b_highS = b_thresholds[3]
    b_lowV = b_thresholds[4]
    b_highV = b_thresholds[5]
    frame_HSV = cv.cvtColor(current_frame, cv.COLOR_BGR2HSV)

    # Apply threshold on frame_HSV
    b_mask = cv.inRange(frame_HSV, (b_lowH, b_lowS, b_lowV), (b_highH, b_highS, b_highV))
    # search_Frame = cv.bitwise_and(current_frame, current_frame, mask=b_mask)
    search_Frame = cv.bitwise_and(frame_HSV[:,:,1], frame_HSV[:,:,1], mask=b_mask)
    gray_frame = search_Frame
    # gray_frame = cv.cvtColor(search_Frame, cv.COLOR_BGR2GRAY)
    # cv.imshow("H_frame", frame_HSV[:,:,0])
    # cv.imshow("S_frame", frame_HSV[:,:,1])
    # cv.imshow("V_frame", frame_HSV[:,:,2])
    # cv.imshow("gray_frame", gray_frame)
    ##assigns roi as the LED locations
    # if np.any(corners_image == 0):
    #     roi = gray_frame
    # else:
    #     offset_x = min(int(corners_image[0,1]), int(corners_image[1,1]))
    #     offset_y = min(int(corners_image[0,0]), int(corners_image[2,0]))
    #     roi = gray_frame[
    #         max(int(corners_image[2,1]), int(corners_image[3,1])),
    #         min(int(corners_image[0,0]), int(corners_image[2,0])): 
    #         min(int(corners_image[0,1]), int(corners_image[1,1])): 
    #         max(int(corners_image[1,0]), int(corners_image[3,0]))
    #     ]
    # template = cv.imread('Sample_Video\Frame_139_Templates\Template_1.png', cv.IMREAD_GRAYSCALE)
    best_match_score = 0
    best_match_location = (0, 0)
    best_template = None
    template_List = []

    # avg 40 ms per template
    for template_path in [
                        #   'Sample_Video\Frame_0_Templates\Template_0.png',
                        #   'Sample_Video\Frame_0_Templates\Template_1.png',
                        #   'Sample_Video\Frame_0_Templates\Template_2.png',
                        #   'Sample_Video\Frame_0_Templates\Template_3.png',    
                        #   'Sample_Video\Frame_0_Templates\Template_4.png',
                        #   'Sample_Video\Frame_139_Templates\Template_0.png',
                        #   'Sample_Video\Frame_139_Templates\Template_1.png',
                        #   'Sample_Video\Frame_139_Templates\Template_2.png',
                        #   'Sample_Video\Frame_139_Templates\Template_3.png',
                        #   'Sample_Video\Frame_139_Templates\Template_4.png',
                        #   'Sample_Video\Frame_256_Templates\Template_0.png',
                        #   'Sample_Video\Frame_256_Templates\Template_1.png',
                        #   'Sample_Video\Frame_256_Templates\Template_2.png',
                        #   'Sample_Video\Frame_256_Templates\Template_3.png',
                        #   'Sample_Video\Frame_256_Templates\Template_4.png',
                        #   'Sample_Video\Frame_411_Templates\Template_0.png',
                        #   'Sample_Video\Frame_411_Templates\Template_1.png',
                        #   'Sample_Video\Frame_411_Templates\Template_2.png',
                        #   'Sample_Video\Frame_411_Templates\Template_3.png',
                        #   'Sample_Video\Frame_439_Templates\Template_0.png',
                        #   'Sample_Video\Frame_439_Templates\Template_1.png',
                        #   'Sample_Video\Frame_439_Templates\Template_2.png',
                        #   'Sample_Video\Frame_439_Templates\Template_3.png',
                        #   'Sample_Video\Frame_439_Templates\Template_4.png',
                        #   'Sample_Video\Frame_738_Templates\Template_0.png',
                        #   'Sample_Video\Frame_738_Templates\Template_1.png',
                        #   'Sample_Video\Frame_738_Templates\Template_2.png',
                        #   'Sample_Video\Frame_738_Templates\Template_3.png',
                        #   'Sample_Video\Frame_746_Templates\Template_0.png',
                        #   'Sample_Video\Frame_746_Templates\Template_1.png',
                        #   'Sample_Video\Frame_746_Templates\Template_2.png',
                        #   'Sample_Video\Frame_746_Templates\Template_3.png',
                        #   'Sample_Video\Frame_820_Templates\Template_0.png',
                        #   'Sample_Video\Frame_820_Templates\Template_1.png',
                        #   'Sample_Video\Frame_820_Templates\Template_2.png',
                        #   'Sample_Video\Frame_820_Templates\Template_3.png',
                            # 'Sample_Video\sample1\Frame_67_Templates\Template_0.png',
                            # 'Sample_Video\sample1\Frame_74_Templates\Template_0.png',
                            # 'Sample_Video\sample1\Frame_196_Templates\Template_0.png',
                            # 'Sample_Video\sample1\Frame_512_Templates\Template_0.png',
                            # 'Sample_Video\sample1\Frame_518_Templates\Template_0.png',
                            # 'Sample_Video\sample1\Frame_598_Templates\Template_0.png',
                            # 'Sample_Video\sample1\Frame_666_Templates\Template_0.png',
                            # 'Sample_Video\sample1\Frame_701_Templates\Template_0.png',
                            # 'Sample_Video\sample1\doctored templates\Template_0 (1).png',
                            # 'Sample_Video\sample1\Frame_62_Templates\Template_0.png',
                            # 'liveFrames\Template_23.png',
                            # 'liveFrames\Template_46.png',
                            'liveFrames\Template_25.png',
                        ]:
        template = cv.imread(template_path, cv.IMREAD_GRAYSCALE)
        template_List.append(template)

    # for template in template_List:
    #     #match template to image
    #     result = cv.matchTemplate(gray_frame, template, cv.TM_CCOEFF_NORMED)
    #     # Find the best match for this template
    #     min_val, max_val, min_loc, max_loc = cv.minMaxLoc(result)
    #     if max_val > best_match_score:
    #         best_match_score = max_val
    #         best_match_location = max_loc
    #         best_template = template

    best_template, best_match_location = parallel_template_matching(template_List, gray_frame)
    if best_template is not None:
        top_left = best_match_location
        w, h = best_template.shape[::-1]
        bottom_right = (top_left[0] + w, top_left[1] + h)
        cv.rectangle(frame, top_left, bottom_right, 255, 2)
        ball_list = [top_left[0] + w//2, top_left[1] + h//2]
    else:
        ball_list = [0, 0]
    # Draw rectangles around matched regions
    # ball_list = []
    # w, h = template.shape[::-1]
    # for pt in zip(*loc[::-1]):
    #     cv.rectangle(frame, pt, (pt[0] + w, pt[1] + h), (0, 255, 0), 1)
    #     ball_list.append(pt)
    return ball_list
# def findballxy(current_frame):
             
#         #frame is the current image frame
#         #returns the (x, y) coordinates of the ball in the image
#         frame = current_frame
#         corners_image = findCorners(frame)
#         b_low_H = 11
#         b_high_H = 22
#         b_low_S = 207
#         b_high_S = 255
#         b_low_V = 51
#         b_high_V = 193
#         #convert the frame to HSV
#         offset_x = 0
#         offset_y = 0
#         # print(self.corners_image)
#         if np.any(corners_image == 0):
#             roi = frame
#         else:
#             offset_x = min(int(corners_image[0,1]), int(corners_image[1,1]))
#             offset_y = min(int(corners_image[0,0]), int(corners_image[2,0]))
#             roi = frame[
#                 min(int(corners_image[0,1]), int(corners_image[1,1])): 
#                 max(int(corners_image[2,1]), int(corners_image[3,1])),
#                 min(int(corners_image[0,0]), int(corners_image[2,0])): 
#                 max(int(corners_image[1,0]), int(corners_image[3,0]))
#             ]
#         roi = cv.cvtColor(roi, cv.COLOR_BGR2HSV)
#         roi = cv.GaussianBlur(roi, (3, 3), 0)
#         roi = cv.medianBlur(roi, 3)                ###########BLURRING 
#         # create mask for ball
#         b_mask = cv.inRange(roi, (b_low_H, b_low_S, b_low_V), (b_high_H, b_high_S, b_high_V))
#         # Find contours of ball
#         contours, _ = cv.findContours(b_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
#         # Sort the contours based on area in descending order
#         contours = sorted(contours, key=cv.contourArea, reverse=True)
#         #loop over the top 5 contours and find the ball x, y coordinates
#         # cv.imshow("ball mask", b_mask)
#         # self.ball_pos_image = self.PREV_ball_pos_image
#         for cnt in contours[:5]:
#             #get the area of the contour
#             area = cv.contourArea(cnt)
#             #conditionally set contour based on area
#             if area <450 and area > 50:
#                 x, y, w, h = cv.boundingRect(cnt)
#                 # cv.rectangle(b_mask, (x, y), (x + w, y + h), (255, 255, 255), 2)
#                 # cv.circle(b_mask, (x + h//2, y + w//2), 25, (255, 0, 0), -1)
#                 [x,y] = (x + w//2 + offset_y, y + h//2 + offset_x)
#                 # print(x, y)
#                 # cv.imshow("ball mask", b_mask)
                
#                 cv.circle(current_frame, (x, y), radius = 5, color = (0, 0, 255), thickness = -1)
#                 ball_pos_image = np.array([x, y])
#                 return ball_pos_image
#         return np.array([0, 0])
def getBall_pos(frame):
        #takes the current frame and the homography matrix and returns the real position of the ball
        #First we find the camera position of the ball and the corners of the field
        #Then we use the corners to find the homography matrix
        #Then we use the homography matrix to find the real position of the ball
        ##x is the distance from the left side of the table
        ##y is the distance from the top of the table
        ball_pos_real_list = []
        #finds the corners of the field (in the image)
        corners_image = findCorners(frame)
        # print(self.corners_image)
        # returns the image ball position
        ball_pos_list = findballxy(frame)
        # PREV_ball_pos_image = ball_pos_image
        # print(corners)
        hmat = getHomographyMatrix(corners_image)
        # ball = np.array([287, 260])
        ball_pos_list = np.array(ball_pos_list, dtype=np.float32)
        ball_pos_list= ball_pos_list.reshape(-1, 1, 2)
        if ball_pos_list is not None and len(ball_pos_list)>0 and hmat is not None:
            ball_pos_real = cv.perspectiveTransform(ball_pos_list, hmat)
        else: 
            ball_pos_real = np.array([0, 0])
        return ball_pos_real
        
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
# Fuck it we'll do it live
# parser = argparse.ArgumentParser(description='Code for Thresholding Operations using inRange tutorial.')
# parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
# args = parser.parse_args()
# cap = cv.VideoCapture(args.camera, cv.CAP_DSHOW)
 ## Stock video
video_path = 'Sample_Video/Normal conditions1.mp4'
cap = cv.VideoCapture(video_path)
# video_path= 'Sample_Video/Foosball_sample2 light.mp4'
# video_path = 'Sample_Video\Dark_sample.mp4'
# video_path = 'Sample_Video/Normal conditions Sample2.mp4'

# Check if the video was opened successfully
if not cap.isOpened():
    print("Error: Could not open video.")
    exit()
    
cv.namedWindow(window_capture_name)
# cv.namedWindow(window_detection_name)
start_value = 80
cv.createTrackbar(threshold_name, window_capture_name, start_value, 100, on_threshold_change)

# Get the initial threshold value
initial_threshold = cv.getTrackbarPos(threshold_name, window_capture_name) / 100

# Set the initial threshold value
threshold = initial_threshold


frame_width = 480
frame_height = 640
global tockercounter
global tickertocker
global PREV_corners_image
global frame
PREV_corners_image = [(0,0),(0,0),(0,0),(0,0)]
tockercounter = 0
tickertocker = 0

# template = cv.imread('Sample_Video\Frame_0_Templates\Template_2.png', cv.IMREAD_GRAYSCALE)
# w, h = template.shape[::-1]
printTimer = time.time()

balltimer = time.time()
ballclock = 0
while True:
    tick = time.time()
    ret, frame = cap.read()
    FRAME = frame
    if frame is None:
        break
    # frame = undistort(frame)
    resized_frame = cv.resize(frame, (frame_height, frame_width))
    frame = resized_frame
    # gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    # result = cv.matchTemplate(gray_frame, template, cv.TM_CCOEFF_NORMED)

    # Set a threshold to identify matching regions
    
    # loc = np.where(result >= threshold)

    # Draw rectangles around matched regions
    # for pt in zip(*loc[::-1]):
    #     cv.rectangle(frame, pt, (pt[0] + w, pt[1] + h), (0, 255, 0), 1)
    balltimer = time.time()
    ball_pos_real = getBall_pos(frame)
    ballclock = time.time() - balltimer
    
    if time.time() - printTimer > 1:
        if ball_pos_real is not None:
            for ball_pos_real in ball_pos_real:
                print("real Ball pos = " + str(ball_pos_real))
        print("time to find: "+ str(ballclock))
        print()
        printTimer = time.time()


    cv.imshow(window_capture_name, frame)
    # cv.imshow(window_detection_name, frame_threshold)
    

    measureLoopTime(tick)
    key = cv.waitKey(30)
    if key == ord('q') or key == 27:
        break