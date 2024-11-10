from __future__ import print_function
import cv2 as cv
import argparse
import numpy as np
import time

if cv.ocl.haveOpenCL():
    print("OpenCL is available.")
    print("Using OpenCL: ", cv.ocl.useOpenCL())
    cv.ocl.setUseOpenCL(True)
else:
    print("OpenCL is not available.")
global threshold

window_capture_name = 'Video Feed'
window_detection_name = 'Object Detection'
def on_threshold_change(val):
    global threshold
    threshold = val / 100
# Create a trackbar for the threshold value
threshold_name = 'Threshold Value(%)'
def findLED(mask, roi_x, roi_y):
        #given the mask of an roi return the (x, y) coordinates of the LED
        #mask is the mask of the roi
        #roi_x is the x coordinate of the roi in the original image
        #roi_y is the y coordinate of the roi in the original image
        #returns the (x, y) coordinates of the LED in the original image

        # Find contours of green LEDs
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        # Sort the contours based on area in descending order
        # contours = sorted(contours, key=cv.contourArea, reverse=True)

        #loop over the top 5 contours and find the LED x, y coordinates
        for cnt in contours[:]:
            #get the area of the contour
            area = cv.contourArea(cnt)
            #conditionally set contour based on area
            if area <500 and area > 5:
                x, y, w, h = cv.boundingRect(cnt)
                corners_image = (x + roi_x + w//2, y + roi_y + h//2)
                return corners_image
        corners_image = (0, 0)
        return corners_image
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
        #frame is the current image frame
        #returns a 4 x 2 array of the (x, y) coordinates of the corners of the table IN THE IMAGE
        percentofCorner = 0.22
        corners = np.zeros((4,2))
        radius = 5
        thickness = -1
        frame_HSV = cv.cvtColor(current_frame, cv.COLOR_BGR2HSV)

        height, width, _ = current_frame.shape
        # Blur the frame for style points B)


        frame_HSV = cv.GaussianBlur(frame_HSV, (3, 3), 0)
        frame_HSV = cv.medianBlur(frame_HSV, 3) #################### BLURRING 
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
        green_low_H = 50
        green_high_H = 101
        green_low_S = 166
        green_high_S = 223
        green_low_V = 61
        green_high_V = 238
        # Create masks for green LEDs using thresholds
        green_mask1 = cv.inRange(roi1, (green_low_H, green_low_S, green_low_V), (green_high_H, green_high_S, green_high_V))
        green_mask2 = cv.inRange(roi2, (green_low_H, green_low_S, green_low_V), (green_high_H, green_high_S, green_high_V))
        green_mask3 = cv.inRange(roi3, (green_low_H, green_low_S, green_low_V), (green_high_H, green_high_S, green_high_V))
        green_mask4 = cv.inRange(roi4, (green_low_H, green_low_S, green_low_V), (green_high_H, green_high_S, green_high_V))

        # Find the (x, y) coordinates of the green LEDs using findLED
        LED1 = findLED(green_mask1, 0, 0)
        LED2 = findLED(green_mask2, int(width* (1-percentofCorner)), 0)
        LED3 = findLED(green_mask3, 0, int(height*(1-percentofCorner)))
        LED4 = findLED(green_mask4, int(width*(1-percentofCorner)), int(height*(1-percentofCorner)))

        corners_image = np.array([LED1, LED2, LED3, LED4])
        return corners_image
def findballxy(current_frame):

    # corners_image = findCorners(current_frame)
    gray_frame = cv.cvtColor(current_frame, cv.COLOR_BGR2GRAY)
    # Convert to UMat first
    gray_frame_umat = cv.UMat(gray_frame)

    # Convert UMat to float32 using cv.convertTo
    gray_frame = cv.UMat(gray_frame)
    gray_frame = cv.UMat(gray_frame.get().astype('float32'))

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
                          'Sample_Video\Frame_0_Templates\Template_0.png',
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


                        ]:
        template = cv.imread(template_path, cv.IMREAD_GRAYSCALE)
        template_List.append(template)
    for template in template_List:
        ocl_template = cv.UMat(template.astype('float32'))
    
        #match template to image
        result = cv.matchTemplate(gray_frame, ocl_template, cv.TM_CCOEFF_NORMED)
        # Find the best match for this template
        # Print result before and after downloading to verify consistency
        # print("Before downloading:", result)
        result_ocl = result.get()
        # print("After downloading:", result_ocl)

        min_val, max_val, min_loc, max_loc = cv.minMaxLoc(result_ocl)
        if max_val > best_match_score:
            best_match_score = max_val
            best_match_location = max_loc
            best_template = template

    if best_template is not None:
        top_left = best_match_location
        w, h = best_template.shape[::-1]
        bottom_right = (top_left[0] + w, top_left[1] + h)
        cv.rectangle(frame, top_left, bottom_right, 255, 2)
        ball_list = [top_left[0] + w//2, top_left[1] + h//2]
    # Draw rectangles around matched regions
    # ball_list = []
    # w, h = template.shape[::-1]
    # for pt in zip(*loc[::-1]):
    #     cv.rectangle(frame, pt, (pt[0] + w, pt[1] + h), (0, 255, 0), 1)
    #     ball_list.append(pt)
    


        
    return ball_list
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
## [low]

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
## Fuck it we'll do it live
# parser = argparse.ArgumentParser(description='Code for Thresholding Operations using inRange tutorial.')
# parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
# args = parser.parse_args()

 ## Stock video
video_path = 'Sample_Video\Dark_sample.mp4'
cap = cv.VideoCapture(video_path)
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