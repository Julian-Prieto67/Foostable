import cv2 as cv
import argparse
import numpy as np
# This pretty much works now!
frame_height = 1960//3
frame_width = 1080//3

desired_width = 1920//2  # Set your desired width FOR STREAM
desired_height = 1080//2  # Set your desired height FOR STREAM
FRAMERATE = 30 # Set your desired framerate FOR STREAM
corners = np.zeros((4,2))
global last_corners
last_corners = np.zeros((4,2))
global lastBallpos
lastBallpos = np.zeros((1,2))
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
    offset = 0
    new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w-offset, h-offset), 1, (w, h))
    undistorted_image = cv.undistort(frame, camera_matrix, dist_coeffs, None, new_camera_matrix)
    return undistorted_image

def findballxy(frame, corners):
    #frame is the current image frame
    #returns the (x, y) coordinates of the ball in the image

    # Define lower and upper thresholds for Ball
    # These thresholds are for EV COMP: 0.5 WB: NATIVE ISO MAX: 3200
    # b_low_H = 9
    # b_high_H = 19
    # b_low_S = 152
    # b_high_S = 195
    # b_low_V = 134
    # b_high_V = 230
    # These thresholds are for normal camera settings
    # cv.createBackgroundSubtractorMOG2().apply(frame)
    b_low_H = 11
    b_high_H = 22
    b_low_S = 207
    b_high_S = 255
    b_low_V = 51
    b_high_V = 193
    
    b_frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    b_frame = cv.GaussianBlur(b_frame, (3, 3), 0)
    b_frame = cv.medianBlur(b_frame, 3)
    offset_x = 0
    offset_y = 0
    # roi = b_frame
    if np.any(corners == 0):
        roi = b_frame
    else:
        offset_x = min(int(corners[0][1]), int(corners[1][1]))
        offset_y = min(int(corners[0][0]), int(corners[2][0]))
        roi = b_frame[min(int(corners[0][1]), int(corners[1][1])): max(int(corners[2][1]), int(corners[3][1])), min(int(corners[0][0]), int(corners[2][0])):max(int(corners[1][0]), int(corners[3][0]))]
    #create mask for ball
    b_mask = cv.inRange(roi, (b_low_H, b_low_S, b_low_V), (b_high_H, b_high_S, b_high_V))
    # Find contours of ball
    contours, _ = cv.findContours(b_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    # Sort the contours based on area in descending order
    contours = sorted(contours, key=cv.contourArea, reverse=True)
    #loop over the top 5 contours and find the ball x, y coordinates
    
    for cnt in contours[:5]:
        #get the area of the contour
        area = cv.contourArea(cnt)
        #conditionally set contour based on area
        if area <450 and area > 50:
            x, y, w, h = cv.boundingRect(cnt)
            # cv.rectangle(b_mask, (x, y), (x + w, y + h), (255, 255, 255), 2)
            # cv.circle(b_mask, (x + h//2, y + w//2), 25, (255, 0, 0), -1)
            [x,y] = (x + w//2 + offset_y, y + h//2 + offset_x)
            # print(x, y)
            cv.imshow("ball mask", b_mask)
            
            cv.circle(frame, (x, y), radius = 5, color = (255, 0, 0), thickness = -1)
            return (x, y)
    cv.imshow("ball mask", b_mask)
    return (0,0)
def findLED(mask, roi_x, roi_y):
    #given the mask of an roi return the (x, y) coordinates of the LED
    #mask is the mask of the roi
    #roi_x is the x coordinate of the roi in the original image
    #roi_y is the y coordinate of the roi in the original image
    #returns the (x, y) coordinates of the LED in the original image

    # Find contours of red and green LEDs
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    # Sort the contours based on area in descending order
    contours = sorted(contours, key=cv.contourArea, reverse=True)

    #loop over the top 5 contours and find the LED x, y coordinates
    for cnt in contours[:5]:
        #get the area of the contour
        area = cv.contourArea(cnt)
        #conditionally set contour based on area
        if area <500 and area > 5:
            x, y, w, h = cv.boundingRect(cnt)
            corner = (x + roi_x + w//2, y + roi_y + h//2)
            return corner
    return (0, 0)
def getHomographyMatrix(frame, corners):
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

    topRight = [1196.975, 647.7]
    topLeft = [19.75, 638.175]
    bottomRight = [1181.1, 19.05]
    bottomLeft = [12.7,19.75]


    destPoints = np.array([topLeft, topRight, bottomLeft, bottomRight])
    homographyMatrix, _ = cv.findHomography(corners, destPoints)
    return homographyMatrix

def findCorners(frame):
    #frame is the current image frame
    #returns a 4 x 2 array of the (x, y) coordinates of the corners of the table IN THE IMAGE
    #what variables I need
    global last_corners
    corners = np.zeros((4,2))
    radius = 5
    color = (0, 0, 255)  # RED color in BGR
    thickness = -1
    frame_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    height, width, _ = frame.shape
    # Blur the frame for style points B)


    frame_HSV = cv.GaussianBlur(frame_HSV, (3, 3), 0)
    frame_HSV = cv.medianBlur(frame_HSV, 3)
    # print(frame_HSV.shape)

    ##Need 4 roi for each of the corners
    percentofCorner = 0.22

    #roi1 is the bottom left corner (GREEN LED ON THE TOP LEFT SIDE OF THE SCREEN)
    roi1 = frame_HSV[0:int(height*percentofCorner), 0:int(width*percentofCorner)]
    #roi2 is the bottom right corner (GREEN LED ON THE TOP RIGHT SIDE OF THE SCREEN)
    roi2 = frame_HSV[int(height*0):int(height*percentofCorner), int(width*(1-percentofCorner)):width]
    #roi3 is the top left corner (GREEN LED ON THE BOTTOM LEFT SIDE OF THE SCREEN)
    roi3 = frame_HSV[int(height*(1-percentofCorner)):int(height*1), 0:int(width*percentofCorner)]
    #roi4 is the top right corner (GREEN LED ON THE BOTTOM RIGHT SIDE OF THE SCREEN)
    roi4 = frame_HSV[int(height*(1-percentofCorner)):int(height*1), int(width*(1-percentofCorner)):width]

    # real pics of the roi
    froi1 = frame[0:int(height*percentofCorner), 0:int(width*percentofCorner)]
    froi2 = frame[int(height*0):int(height*percentofCorner), int(width*(1-percentofCorner)):width]
    froi3 = frame[int(height*(1-percentofCorner)):int(height*1), 0:int(width*percentofCorner)]
    froi4 = frame[int(height*(1-percentofCorner)):int(height*1), int(width*(1-percentofCorner)):width]
    
    #These are the thresholds I've come up  with  but they kind of work?

    #RED LED:  DEPRECATED
    #These thresholds are for normal camera settings
    # red_low_H = 116
    # red_high_H = 180
    # red_low_S = 112
    # red_high_S = 255
    # red_low_V = 130
    # red_high_V = 250
    #Thresholds are for WB: NATIVE EVCOMP: 0.5 ISO MAX: 3200
    # red_low_H = 0 
    # red_high_H = 9
    # red_low_S = 78
    # red_high_S = 216
    # red_low_V = 105
    # red_high_V = 204

    #GREEN LED:
    #Thresholds are for normal camera settings
    # Define lower and upper thresholds for green LED 
    green_low_H = 50
    green_high_H = 101
    green_low_S = 166
    green_high_S = 223
    green_low_V = 61
    green_high_V = 238
    #Thresholds are for WB: NATIVE EVCOMP: 0.5 ISO MAX: 3200
    # green_low_H = 34
    # green_high_H = 50
    # green_low_S = 53
    # green_high_S = 139
    # green_low_V = 79
    # green_high_V = 118

    # Create masks for red and green LEDs using thresholds
    # print(roi1.shape)
    green_mask1 = cv.inRange(roi1, (green_low_H, green_low_S, green_low_V), (green_high_H, green_high_S, green_high_V))
    green_mask2 = cv.inRange(roi2, (green_low_H, green_low_S, green_low_V), (green_high_H, green_high_S, green_high_V))
    green_mask3 = cv.inRange(roi3, (green_low_H, green_low_S, green_low_V), (green_high_H, green_high_S, green_high_V))
    green_mask4 = cv.inRange(roi4, (green_low_H, green_low_S, green_low_V), (green_high_H, green_high_S, green_high_V))

    LED1 = findLED(green_mask1, 0, 0)
    LED2 = findLED(green_mask2, int(width* (1-percentofCorner)), 0)
    LED3 = findLED(green_mask3, 0, int(height*(1-percentofCorner)))
    LED4 = findLED(green_mask4, int(width*(1-percentofCorner)), int(height*(1-percentofCorner)))
    

    corners = np.array([LED1, LED2, LED3, LED4])

    count = 0
    color = (0, 255, 0)
    for corner in corners:
        if corner[0] == 0 and corner[1] == 0:
            corner[0] = last_corners[count][0]
            corner[1] = last_corners[count][1]
            corners[count] = last_corners[count]
        # print(corners)
        # print(frame.shape)
        # print()
        cv.circle(frame_HSV, (int(corner[0]), int(corner[1])), radius, color, thickness)
        cv.circle(frame, (int(corner[0]), int(corner[1])), radius, color, thickness)
        count = count + 1

    height, width = roi1.shape[:2]
    roi2 = cv.resize(roi2, (width, height))
    roi3 = cv.resize(roi3, (width, height))
    roi4 = cv.resize(roi4, (width, height))

    top_row = cv.hconcat([roi1, roi2])
    bottom_row = cv.hconcat([roi3, roi4])
    cornerFrame = cv.vconcat([top_row, bottom_row])

    # resized_frame = cv.resize(frame, (frame_height//2, frame_width//2))
    # frame = resized_frame

    
    # resized_frame = cv.resize(frame_HSV, (frame_height//2, frame_width//2))
    # frame_HSV = resized_frame

    top_row = cv.hconcat([froi1, froi2])
    bottom_row = cv.hconcat([froi3, froi4])
    rois = cv.vconcat([top_row, bottom_row])

    top_row = cv.hconcat([green_mask1, green_mask2])
    bottom_row = cv.hconcat([green_mask3, green_mask4])
    masks = cv.vconcat([top_row, bottom_row])
    cv.imshow("masks",masks)
    # cv.imshow('rois', cornerFrame)
    cv.imshow('LEDs', rois)
    # cv.imshow('Corners', frame_HSV)
    last_corners = corners
    return corners

def getBallPosition(frame):
    #takes the current frame and the homography matrix and returns the real position of the ball
    #First we find the amera position of the ball and the corners of the field
    #Then we use the corners to find the homography matrix
    #Then we use the homography matrix to find the real position of the ball

    radius = 5
    color = (255, 0, 0)  # RED color in BGR
    thickness = -1

    #finds the corners of the field (in the image)
    corners = findCorners(frame)
    global lastBallpos
    # returns the image ball position
    ball = findballxy(frame, corners)
    lastBallpos = ball
    # print(corners)
    hmat = getHomographyMatrix(frame, corners)
    ball = np.array([ball], dtype=np.float32)
    ball = ball.reshape(-1, 1, 2)  # Reshape to (1, 1, 2)
    print(ball)
    print(corners)
    print(hmat)
    print()
    if hmat is not None and ball is not None:
        ball = cv.perspectiveTransform(ball, hmat)
    else: 
        ball = np.array([0, 0])
    return ball

# Define lower and upper thresholds for Ball

window_capture_name = 'Video Capture'
window_detection_name = 'Object Detection'

# Grab the camera 
parser = argparse.ArgumentParser(description='Camera')
parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
args = parser.parse_args()  

# FOR CAMERA INPUT 
cap = cv.VideoCapture(args.camera, cv.CAP_DSHOW)

# Setting the resolution its not really working well....


# cap.set(cv.CAP_PROP_FRAME_WIDTH, desired_width)
# cap.set(cv.CAP_PROP_FRAME_HEIGHT, desired_height)

# Check if the resolution was set correctly
actual_width = cap.get(cv.CAP_PROP_FRAME_WIDTH)
actual_height = cap.get(cv.CAP_PROP_FRAME_HEIGHT)
# cap.set(cv.CAP_PROP_FPS, FRAMERATE)
print(f'Resolution set to {actual_width}x{actual_height}')


# # FOR VIDEO INPUT 
# video_path = 'Sampletablevid.mp4'
# cap = cv.VideoCapture(video_path)
# # Check if the video was opened successfully
# if not cap.isOpened():
#     print("Error: Could not open video.")
#     exit()


while True:
    # read frame dataw
    ret, frame = cap.read()
    if frame is None:
        print("Error: No frame.")
        continue
    if not ret:
        print("Failed to capture frame, retrying...")
        continue  # Retry capturing the frame
    frame = undistort(frame)
    height, width, _ = frame.shape
    # print(height, width)
    
    #resize frame to make it visible...

    # resized_frame = cv.resize(frame, (frame_height, frame_width))
    # frame = resized_frame

    # fix distortion in frame according to calibration data in calibration_data.xml
    
    frame_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    # roi = frame_HSV[int(height*0.083):int(height*0.917), int(width*0.3):int(height*0.9)]
    
    #find corners and draw them
    print(getBallPosition(frame)/25.4)
    # getBallPosition(frame)  
    print()




    
    cv.imshow("frame", frame)
    # cv.imshow("fixed", fixedframe)
    key = cv.waitKey(30)
    if key == ord('q') or key == 27:
        break