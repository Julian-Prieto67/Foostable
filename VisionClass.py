import cv2 as cv
import numpy as np
import time
import argparse
import concurrent.futures

class Vision:
    def __init__(self, playback = False):
        self.playback = playback
        self.displayFrame = True

        self.ball_pos_real = np.array([0, 0])
        self.ball_pos_image = np.array([0, 0])
        self.corners_image = np.array([[0, 0], [0, 0], [0, 0], [0, 0]])
        self.PREV_corners_image = np.array([[0, 0], [0, 0], [0, 0], [0, 0]])
        # self.corners_real = np.array([[1.125*25.4, 25*25.4], [46.125*25.4, 24.875*25.4],[1.25*25.4, 0.625*25.4], [46.5*25.4, 0.5*25.4]])
        self.corners_real = np.array([[7/8*25.4, 25.25*25.4], [46.375*25.4, 25.25*25.4],[0.875*25.4, 0.625*25.4], [46.5*25.4, 0.75*25.4]])
        self.cornerTimer = time.time()
        self.Balltemplates = []
        for template_path in [
                            # 'templates\Frames\liveFrames\Template_23.png',
                            # 'templates\Frames\liveFrames\Template_46.png',
                            # r'C:/Users/juls6/Documents/GitHub/Foostable/templates/Usedtemplates/Template_25.png',
                            'templates\\Frames\\liveFrames\\NEWTemplate_3.png',
                        ]:
            template = cv.imread(template_path, cv.IMREAD_GRAYSCALE)
            self.Balltemplates.append(template)

        ##Camera Variables
        self.parser = argparse.ArgumentParser(description='Camera')
        self.parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
        self.args = self.parser.parse_args()
        # video_path = 'C:/Users/juls6/Desktop/Classes/FOOSTABLE/Software/Sample_Video/11-6-LightSample.mp4'
        video_path = '11-6LightSample2.mp4'
        self.current_frame = None
        self.current_frameHSV = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rvecs = []
        self.tvecs = []
        self.loadData()
        self.percentofCorner = 0.20
        ##DEPENDENCY INJECTION DECIDING BETWEEN PLAYBACK OR NOT
        if self.playback:
            self.cam = cv.VideoCapture(video_path)
        else:
            self.cam = cv.VideoCapture(self.args.camera, cv.CAP_DSHOW)
            # self.cam.set(cv.CAP_PROP_FRAME_WIDTH, 1920)
            # self.cam.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)
        

##################METHODS##################
    def loadData(self):
        file_path = 'calibration_data.xml'
        # Load the data
        fs = cv.FileStorage(file_path, cv.FILE_STORAGE_READ)
        self.camera_matrix = fs.getNode('camera_matrix').mat()
        self.dist_coeffs = fs.getNode('dist_coeffs').mat()

        # Read rvecs and tvecs

        rvecs_node = fs.getNode("rvecs")
        for i in range(rvecs_node.size()):
            self.rvecs.append(rvecs_node.at(i).mat())

        tvecs_node = fs.getNode("tvecs")
        for i in range(tvecs_node.size()):
            self.tvecs.append(tvecs_node.at(i).mat())

        fs.release()


    def UpdateFrame(self):
        #updates the current frame in the class
        #returns True if the frame was successfully updated
        ret, frame = self.cam.read()
        if not ret:
            print('Failed to read frame')
            return ret
        frame = cv.resize(frame, (640, 480), interpolation = cv.INTER_AREA)
        self.current_frame = frame
        self.current_frameHSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        # self.ser.UpdateGui(self.ball_pos_real, self.corners_image,self.corners_real, self.ball_pos_image, self.GRod, self.DRod, self.MRod, self.ARod)
        # self.ser.UpdateGui(self.ball_pos_real, self.corners_image,self.corners_real, self.ball_pos_image, self.GRod, self.DRod, self.MRod, self.ARod)
        return ret
    
    def ShowField(self):
        #displays the current frame of the camera 
        cv.imshow('FoosTable', self.current_frame)
        cv.waitKey(1)


    def undistort(self):
        h, w = self.current_frame.shape[:2]
        offset = 0
        new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (w-offset, h-offset), 1, (w, h))
        undistorted_image = cv.undistort(self.current_frame, self.camera_matrix, self.dist_coeffs, None, new_camera_matrix)
        self.current_frame = undistorted_image
    
    def match_template(self,template, frame, method=cv.TM_CCORR_NORMED):
        return cv.matchTemplate(frame, template, method)


    def parallel_template_matching(self,templates, frame):
        # Use ThreadPoolExecutor to parallelize across cores
        with concurrent.futures.ThreadPoolExecutor() as executor:
            # Submit tasks to match each template
            results = list(executor.map(lambda tpl: self.match_template(tpl, frame), templates))

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

        return best_template, best_match_location, max_val
    
    def findballxy(self):
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
        frame_HSV = self.current_frameHSV

        # Apply threshold on frame_HSV
        b_mask = cv.inRange(frame_HSV, (b_lowH, b_lowS, b_lowV), (b_highH, b_highS, b_highV))
        # Apply mask on frame_HSV to get the search frame
        search_Frame = cv.bitwise_and(frame_HSV[:,:,1], frame_HSV[:,:,1], mask=b_mask)
        gray_frame = search_Frame
        
        best_match_score = 0
        best_match_location = (0, 0)
        best_template = None

        best_template, best_match_location, max_val = self.parallel_template_matching(self.Balltemplates, gray_frame)
        
        if best_template is not None and max_val > 0.91:
            top_left = best_match_location
            w, h = best_template.shape[::-1]
            bottom_right = (top_left[0] + w, top_left[1] + h) #uncomment both to draw rectangle on ball
            cv.rectangle(self.current_frame, top_left, bottom_right, 255, 2)
            ball_pos = [top_left[0] + w//2, top_left[1] + h//2]
        else:
            ball_pos = [0, 0]
        self.ball_pos_image = ball_pos
        return
    
    def getHomographyMatrix(self):
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

        topLeft = self.corners_real[0]
        topRight = self.corners_real[1]
        bottomLeft = self.corners_real[2]
        bottomRight = self.corners_real[3]

        # destPoints = np.array([topLeft, topRight, bottomLeft, bottomRight])
        destPoints = np.array([bottomLeft, bottomRight, topLeft, topRight])
        homographyMatrix, _ = cv.findHomography(self.corners_image, destPoints)
        return homographyMatrix
    
    def findLED(self, mask, roi_x, roi_y):
        #given the mask of an roi return the (x, y) coordinates of the LED
        #mask is the mask of the roi
        #roi_x is the x coordinate of the roi in the original image
        #roi_y is the y coordinate of the roi in the original image
        #returns the (x, y) coordinates of the LED in the original image
        
        #Only need the S channel of the HSV image
        mask = mask[:,:,1]
        # Find contours of green LEDs
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        # Sort the contours based on area in descending order
        contours = sorted(contours, key=cv.contourArea, reverse=True)
        
        #loop over the top 5 contours and find the LED x, y coordinates
        for cnt in contours[:]:
            #get the area of the contour
            area = cv.contourArea(cnt)
            #conditionally set contour based on area
            if area <500 and area > 5:
                x, y, w, h = cv.boundingRect(cnt)
                LED = [x + roi_x + w//2, y + roi_y + h//2]
                cv.circle(self.current_frame, (LED[0], LED[1]), radius=3, color=(0, 255, 0), thickness=-1)
                return LED
        LED = [0, 0]
        return LED
    
    def findCorners(self):
        #frame is the current image frame
        #returns a 4 x 2 array of the (x, y) coordinates of the corners of the table IN THE IMAGE
        current_frame = self.current_frame

        percentofCorner = self.percentofCorner
        frame_HSV = self.current_frameHSV

        height, width, _ = current_frame.shape

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
        greenthresholds = [41,
94,
127,
255,
34,
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

        # apply mask to HSV frame
        roi1 = cv.bitwise_and(roi1, roi1, mask=green_mask1)
        roi2 = cv.bitwise_and(roi2, roi2, mask=green_mask2)
        roi3 = cv.bitwise_and(roi3, roi3, mask=green_mask3)
        roi4 = cv.bitwise_and(roi4, roi4, mask=green_mask4)

        # Find the (x, y) coordinates of the green LEDs using findLED
        LED1 = self.findLED(roi1, 0, 0)
        LED2 = self.findLED(roi2, int(width* (1-percentofCorner)), 0)
        LED3 = self.findLED(roi3, 0, int(height*(1-percentofCorner)))
        LED4 = self.findLED(roi4, int(width*(1-percentofCorner)), int(height*(1-percentofCorner)))

        #stack all LED positions into a 4x2 array

        corners_image = np.array([LED1, LED2, LED3, LED4])
        #make sure no corners are (0, 0) and if they are replace them with the previous found value
        for i in range(len(self.corners_image)):
            if np.all(self.corners_image[i] == 0):
                self.corners_image[i] = self.PREV_corners_image[i]

        self.corners_image = corners_image
        self.PREV_corners_image = corners_image


    def getBallPos(self):
        #takes the current frame and the homography matrix and returns the real position of the ball
        #First we find the camera position of the ball and the corners of the field
        #Then we use the corners to find the homography matrix
        #Then we use the homography matrix to find the real position of the ball
        ##x is the distance from the left side of the table
        ##y is the distance from the top of the table

        #finds the corners of the field (in the image) if any corners are 0 or if 30 seconds have passed since the last time the corners were found
        if time.time() - self.cornerTimer > 60*0.5 or np.any(self.corners_image == 0):
            self.findCorners()
            self.cornerTimer = time.time()
        
        
        # returns the image ball position
        self.findballxy()
        # print(corners)
        hmat = self.getHomographyMatrix()
        ball = np.array([self.ball_pos_image], dtype=np.float32)
        ball = ball.reshape(-1, 1, 2)  # Reshape to (1, 1, 2)
        # print(ball)
        # corners = np.array([(27, 38), (592, 40), (15, 444), (595, 439)])
        # ball = np.array([287, 260])
        if hmat is not None and ball is not [0,0] not in self.corners_image:
            self.ball_pos_real = cv.perspectiveTransform(ball, hmat)
            self.ball_pos_real = self.ball_pos_real.ravel()
            return self.ball_pos_real
        else: 
            self.ball_pos_real = np.array([0, 0])
            return self.ball_pos_real
    