import numpy as np
import cv2 as cv
import argparse
import serial
import time
import asyncio
import moteus
import math
import gc
from RodClass import RodFake, RodReal
import concurrent.futures
from ArduinoClass import ArduinoFake, Arduino

class FoosBot:
    def __init__(self, difficulty = 0):
        self.difficulty = difficulty
        self.playback = True

        ##time variables
        self.tick = 0
        self.tock = 0
        self.tockercounter = 0
        self.tickertocker = 0

        ##CV Variables
        self.ball_pos_real = np.array([0, 0])
        self.ball_pos_image = np.array([0, 0])
        self.PREV_ball_pos_image = np.array([[0, 0], [0, 0], [0, 0], [0, 0]])
        self.corners_image = np.array([[0, 0], [0, 0], [0, 0], [0, 0]])
        self.PREV_corners_image = np.array([[0, 0], [0, 0], [0, 0], [0, 0]])
        ### corners are in BL, BR, TL, TR order
        # self.corners_real = np.array([[19.75, 638.175], [1196.975, 647.7], [12.7,19.75], [1181.1, 19.05]]) OLD
        ##Redo these from the bottom 
        self.corners_real = np.array([[1.125*25.4, 25*25.4], [46.125*25.4, 24.875*25.4],[1.25*25.4, 0.625*25.4], [46.5*25.4, 0.5*25.4]])

        self.corners_real = np.array([[7/8*25.4, 25.25*25.4], [46.375*25.4, 25.25*25.4],[0.875*25.4, 0.625*25.4], [46.5*25.4, 0.75*25.4]])

        self.LED_threshold = 0.5
        # self.LEDtemplate = cv.imread('C:/Users/juls6/Desktop/Classes/FOOSTABLE/Software/liveFrames/Template_62.png', cv.IMREAD_GRAYSCALE)
        self.LEDtemplates = []
        for template_path in [
                            'C:\\Users\\juls6\\Documents\\GitHub\\Foostable\\templates\\Usedtemplates\\Template_0.png',
                            'C:\\Users\\juls6\\Documents\\GitHub\\Foostable\\templates\\Usedtemplates\\Template_62.png',
        ]:
            template = cv.imread(template_path, cv.IMREAD_GRAYSCALE)
            self.LEDtemplates.append(template)
        self.Balltemplates = []
        for template_path in [
                            # 'liveFrames\Template_23.png',
                            # 'liveFrames\Template_46.png',
                            r'C:/Users/juls6/Documents/GitHub/Foostable/templates/Usedtemplates/Template_25.png',
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
        self.percentofCorner = 0.15

        ##PLAYBACK
        self.gui = None
        self.GUIFLAG = True

        ##DEPENDENCY INJECTION DECIDING BETWEEN PLAYBACK OR NOT
        if not self.playback:
            self.GRod = RodReal(1) ##initializes goalie rod
            self.DRod = RodReal(2) ##initializes defense rod
            self.MRod = RodReal(3) ##initializes midfield rod
            self.ARod = RodReal(4) ##initializes attack rod
            self.ser = Arduino() ##initializes arduino
            self.cam = cv.VideoCapture(self.args.camera, cv.CAP_DSHOW)
        else:
            self.GRod = RodFake(1) ##initializes goalie rod
            self.DRod = RodFake(2) ##initializes defense rod
            self.MRod = RodFake(3) ##initializes midfield rod
            self.ARod = RodFake(4) ##initializes attack rod
            self.ser = ArduinoFake() ##initializes arduino
            self.cam = cv.VideoCapture(video_path)
        ##Stepper Motion Variables
        #Change to new pulley diameter
        self.mm2step = 800/(23*math.pi) ##Conversion factor from mm to steps = pulley diameter *pi / steps per revolution
        self.ENDCHAR = '#'

        


    ##############################CAMERA VIEWING FXNS############################
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
        self.ser.UpdateGui(self.ball_pos_real, self.corners_image,self.corners_real, self.ball_pos_image, self.GRod, self.DRod, self.MRod, self.ARod)
        return ret
    def ShowField(self):
        #displays the current frame of the camera 
        if self.current_frame is None:
            print('No frame to display')
            return
        cv.imshow('Field', self.current_frame)
        if self.GUIFLAG:
            self.ser.showGUI()
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
    def undistort(self):
        h, w = self.current_frame.shape[:2]
        offset = 0
        new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (w-offset, h-offset), 1, (w, h))
        undistorted_image = cv.undistort(self.current_frame, self.camera_matrix, self.dist_coeffs, None, new_camera_matrix)
        self.current_frame = undistorted_image

    ##############################FIND BALL FXNS################################
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
        # for result, template in zip(results, templates):
        #     min_val, max_val, min_loc, max_loc = cv.minMaxLoc(result)
        #     if min_val > best_match_score:
        #         best_match_score = min_val
        #         best_match_location = min_loc
        #         best_template = template

        return best_template, best_match_location
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
        # cv.imshow("S_frame", frame_HSV[:,:,1])
        # cv.imshow("V_frame", frame_HSV[:,:,2])
        # cv.imshow("gray_frame", gray_frame)
        best_match_score = 0
        best_match_location = (0, 0)
        best_template = None

        best_template, best_match_location = self.parallel_template_matching(self.Balltemplates, gray_frame)
        if best_template is not None:
            top_left = best_match_location
            w, h = best_template.shape[::-1]
            bottom_right = (top_left[0] + w, top_left[1] + h)
            cv.rectangle(self.current_frame, top_left, bottom_right, 255, 2)
            ball_list = [top_left[0] + w//2, top_left[1] + h//2]
        else:
            ball_list = [0, 0]
        self.ball_pos_image = ball_list
        # Draw rectangles around matched regions
        # ball_list = []
        # w, h = template.shape[::-1]
        # for pt in zip(*loc[::-1]):
        #     cv.rectangle(frame, pt, (pt[0] + w, pt[1] + h), (0, 255, 0), 1)
        #     ball_list.append(pt)

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
        mask = mask[:,:,1]
        # Find contours of green LEDs
        # contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        # Sort the contours based on area in descending order
        # contours = sorted(contours, key=cv.contourArea, reverse=True)
        
        #loop over the top 5 contours and find the LED x, y coordinates
        # for cnt in contours[:]:
        #     #get the area of the contour
        #     area = cv.contourArea(cnt)
        #     #conditionally set contour based on area
        #     if area <500 and area > 5:
        #         x, y, w, h = cv.boundingRect(cnt)
        #         LED = (x + roi_x + w//2, y + roi_y + h//2)
        #         cv.circle(self.current_frame, (LED[0], LED[1]), radius=10, color=(0, 255, 0), thickness=-1)
        #         return LED
        # LED = (0, 0)
        # return LED
        


        ### DIFFERENT VERSION OF FINDING LED WITH IMAGE MATCHING
        best_template, best_match_location = self.parallel_template_matching(self.LEDtemplates, mask)
        if best_template is not None:
            top_left = best_match_location
            w, h = best_template.shape[::-1]
            bottom_right = (top_left[0] + w, top_left[1] + h)
            ledpos = [top_left[0] + w//2 +roi_x, top_left[1] + h//2 +roi_y]
            cv.circle(self.current_frame, (ledpos[0], ledpos[1]), 5, (0, 255, 0), -1)
            return ledpos
        else:
            ledpos = [0, 0]
    # Draw rectangles around matched regions
    # ball_list = []
    # w, h = template.shape[::-1]
    # for pt in zip(*loc[::-1]):
    #     cv.rectangle(frame, pt, (pt[0] + w, pt[1] + h), (0, 255, 0), 1)
    #     ball_list.append(pt)
        return ledpos
        # current_frame = mask[:,:,1]
        # best
        # min_val, max_val, min_loc, max_loc = cv.minMaxLoc(results)
        # if max_val > self.LED_threshold:
        #     top_left = max_loc
        #     w, h = self.LEDtemplate.shape
        #     bottom_right = (top_left[0] + w, top_left[1] + h)
        #     # cv.rectangle(mask, top_left, bottom_right, 255, 2)
        #     LED = [top_left[0] + w//2 + roi_x, top_left[1] + h//2+roi_y]
        #     cv.circle(self.current_frame, (LED[0], LED[1]), 5, (0, 255, 0), -1)
        #     # cv.imshow("LED", mask)
        #     return LED
        # else:
        #     LED = [0, 0]
        # return LED
    def findCorners(self):
        #frame is the current image frame
        #returns a 4 x 2 array of the (x, y) coordinates of the corners of the table IN THE IMAGE
        current_frame = self.current_frame

        percentofCorner = self.percentofCorner
        frame_HSV = self.current_frameHSV

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
        greenthresholds = [41,
94,
127,
255,
34,
255]
        # Thresholds for green LED for low light conditions (Contouring)
#         greenthresholds = [50,
# 101,
# 166,
# 223,
# 61,
# 238]
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
        count=0
        # print(self.corners_image)
        for corner in self.corners_image:
            if corner[0] == 0 and corner[1] == 0:
                corner[0] = self.PREV_corners_image[count][0]
                corner[1] = self.PREV_corners_image[count][1]
                self.corners_image[count] = self.PREV_corners_image[count]
            count += 1
        self.corners_image = corners_image
    def getBallPos(self):
        #takes the current frame and the homography matrix and returns the real position of the ball
        #First we find the camera position of the ball and the corners of the field
        #Then we use the corners to find the homography matrix
        #Then we use the homography matrix to find the real position of the ball
        ##x is the distance from the left side of the table
        ##y is the distance from the top of the table

        #finds the corners of the field (in the image)
        self.findCorners()
        # print(self.corners_image)
        # returns the image ball position
        self.findballxy()
        self.PREV_ball_pos_image = self.ball_pos_image
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
        else: 
            self.ball_pos_real = np.array([0, 0])
        
    async def moveRods(self):
        #moves the rods to whatever position stored in the rod classes
        ##send the rod positions to the stepper motor on a timer (so we don't overload the serial port)
        await self.GRod.RotateRod()
        await self.DRod.RotateRod()
        await self.MRod.RotateRod()
        await self.ARod.RotateRod()
        # await self.GRod.kickAtWill(self.ball_pos_real)
        # await self.DRod.kickAtWill(self.ball_pos_real)
        # await self.MRod.kickAtWill(self.ball_pos_real)
        # await self.ARod.kickAtWill(self.ball_pos_real)
        send = self.GRod.returnRodPos() + self.DRod.returnRodPos() + self.MRod.returnRodPos() + self.ARod.returnRodPos() + self.ENDCHAR
        self.ser.write(send.encode('utf-8'))
    async def blockBall(self):
        #block the ball with the rods
        #find the position of the ball
        ##block the ball with the rods

        self.GRod.blockBall(self.ball_pos_real)
        self.DRod.blockBall(self.ball_pos_real)
        self.MRod.blockBall(self.ball_pos_real)
        self.ARod.blockBall(self.ball_pos_real)

    def changeSpeed(self, speed):
        ## changes the speed of the stepper motor in steps per second max of like 100000
        send = 'SPD' + str(speed) + '|' + str(speed)+ '|' + str(speed)+'|' + str(speed)+'|'+ self.ENDCHAR
        self.ser.write(send)
    def measureLoopTime(self):
        self.tock = time.time()
        self.tickertocker = self.tickertocker + self.tock - self.tick
        self.tockercounter = 1 + self.tockercounter
        if self.tockercounter > 100:
            # DIAGNOSTICS

            # print('Motor States\n')
            # print('G_motor' + str(self.GRod.returnMotorState()))
            # print('\nD_motor' + str(self.DRod.returnMotorState()))
            # print('\nM_motor' + str(self.MRod.returnMotorState()))
            # print('\nA_motor' + str(self.ARod.returnMotorState()))
            print('\nBall Position\n')
            print(self.ball_pos_real)
            print('\n Corner Position\n')
            print(self.corners_image)
            print('\nLoop Time\n')
            print(self.tickertocker/self.tockercounter)
            print()
            self.tockercounter = 0
            self.tickertocker = 0
    def CalibrateRods(self):
        #calibrates the rods to their starting position
        #by sending "HOM" to the arduino
        send = 'HOM' + self.ENDCHAR
        self.ser.write(send.encode('utf-8'))

    ##############################MAIN LOOP################################
    async def run(self):
        gc.collect()
        await self.GRod.clearFaults()
        await self.DRod.clearFaults()
        await self.MRod.clearFaults()
        await self.ARod.clearFaults()
        # Load the data
        self.loadData()
        time.sleep(2)  # Wait for the connection to be established
        
        self.tockercounter = 0
        self.tickertocker = 0
        self.CalibrateRods()
        while True: 
            self.tick = time.time()
            # call this as often as possible to update current frames
            ret = self.UpdateFrame()
            if not ret:
                break
            self.undistort()
            ##Find the ball position IRL
            self.getBallPos() ## MAKE FASTER
            
            ##insert strategy here
            await self.blockBall()
            await self.moveRods()

            self.ShowField()
            self.measureLoopTime()
            if cv.waitKey(47) & 0xFF == ord('q'):
                break
            if cv.waitKey(1) & 0xFF == ord('p'):
                while True:
                    if cv.waitKey(1) & 0xFF == ord('p'):
                        break
        self.cam.release()
        cv.destroyAllWindows()

if __name__ == '__main__':
    Foos = FoosBot()
    asyncio.run(Foos.run())