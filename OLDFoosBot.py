import numpy as np
import cv2 as cv
import argparse
import serial
import time
import asyncio
import moteus
import math
import gc
from RodClass import Rod

class FoosBot:
    def __init__(self, difficulty = 0):
        self.difficulty = difficulty
        ##time variables
        self.tick = 0
        self.tock = 0
        self.tockercounter = 0
        self.tickertocker = 0

        ##CV Positioning Variables
        self.ball_pos_real = np.array([0, 0])
        self.ball_pos_image = np.array([0, 0])
        self.PREV_ball_pos_image = np.array([[0, 0], [0, 0], [0, 0], [0, 0]])
        self.corners_image = np.array([[0, 0], [0, 0], [0, 0], [0, 0]])
        self.PREV_corners_image = np.array([[0, 0], [0, 0], [0, 0], [0, 0]])
        self.corners_real = np.array([[19.75, 638.175], [1196.975, 647.7], [12.7,19.75], [1181.1, 19.05]])

        ##Camera Variables
        self.parser = argparse.ArgumentParser(description='Camera')
        self.parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
        self.args = self.parser.parse_args()  
        self.cam = cv.VideoCapture(self.args.camera, cv.CAP_DSHOW)
        self.current_frame = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rvecs = []
        self.tvecs = []
        self.percentofCorner = 0.22

        ##ROD CLASSES
        self.GRod = Rod(1) ##initializes goalie rod
        self.DRod = Rod(2) ##initializes defense rod
        self.MRod = Rod(3) ##initializes midfield rod
        self.ARod = Rod(4) ##initializes attack rod

        ##Stepper Motion Variables
        #Change to new pulley diameter
        self.mm2step = 800/(23*math.pi) ##Conversion factor from mm to steps = pulley diameter *pi / steps per revolution
        self.ENDCHAR = '#'
        self.arduino_port = 'COM3'  # Arduino's port
        self.baud_rate = 115200 #Arduino baud rate
        self.ser = serial.Serial(self.arduino_port,self.baud_rate, timeout = 1)
        self.serial_timer = time.time()
        self.waittime = 0.1 ##time to wait between serial messages


    ##############################CAMERA VIEWING FXNS############################
    def UpdateFrame(self):
        #updates the current frame in the class
        #returns True if the frame was successfully updated

        ret, frame = self.cam.read()
        self.current_frame = frame
        return ret
    def ShowField(self):
        #displays the current frame of the camera 
        if self.current_frame is None:
            print('No frame to display')
            return
        cv.imshow('Field', self.current_frame)
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
        # Load the data
        self.loadData()
        h, w = self.current_frame.shape[:2]
        offset = 0
        new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (w-offset, h-offset), 1, (w, h))
        undistorted_image = cv.undistort(self.frame, self.camera_matrix, self.dist_coeffs, None, new_camera_matrix)
        self.current_frame = undistorted_image

    ##############################FIND BALL FXNS################################
    def findballxy(self):
            
        #frame is the current image frame
        #returns the (x, y) coordinates of the ball in the image
        frame = self.current_frame
        
        b_low_H = 11
        b_high_H = 22
        b_low_S = 207
        b_high_S = 255
        b_low_V = 51
        b_high_V = 193
        #convert the frame to HSV
        offset_x = 0
        offset_y = 0
        # print(self.corners_image)
        if np.any(self.corners_image == 0):
            roi = frame
        else:
            offset_x = min(int(self.corners_image[0,1]), int(self.corners_image[1,1]))
            offset_y = min(int(self.corners_image[0,0]), int(self.corners_image[2,0]))
            roi = frame[
                min(int(self.corners_image[0,1]), int(self.corners_image[1,1])): 
                max(int(self.corners_image[2,1]), int(self.corners_image[3,1])),
                min(int(self.corners_image[0,0]), int(self.corners_image[2,0])): 
                max(int(self.corners_image[1,0]), int(self.corners_image[3,0]))
            ]
        roi = cv.cvtColor(roi, cv.COLOR_BGR2HSV)
        roi = cv.GaussianBlur(roi, (5, 5), 0)
        roi = cv.medianBlur(roi, 7)                ###########BLURRING 
        # create mask for ball
        b_mask = cv.inRange(roi, (b_low_H, b_low_S, b_low_V), (b_high_H, b_high_S, b_high_V))
        # Find contours of ball
        contours, _ = cv.findContours(b_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        # Sort the contours based on area in descending order
        contours = sorted(contours, key=cv.contourArea, reverse=True)
        #loop over the top 5 contours and find the ball x, y coordinates
        # cv.imshow("ball mask", b_mask)
        self.ball_pos_image = self.PREV_ball_pos_image
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
                # cv.imshow("ball mask", b_mask)
                
                cv.circle(self.current_frame, (x, y), radius = 5, color = (0, 0, 255), thickness = -1)
                self.ball_pos_image = (x, y)
                return
    def findLED(self, mask, roi_x, roi_y):
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
                self.corners_image = (x + roi_x + w//2, y + roi_y + h//2)
                return self.corners_image
        self.corners_image = (0, 0)
        return self.corners_image
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

        destPoints = np.array([topLeft, topRight, bottomLeft, bottomRight])
        homographyMatrix, _ = cv.findHomography(self.corners_image, destPoints)
        return homographyMatrix
    def findCorners(self):
        #frame is the current image frame
        #returns a 4 x 2 array of the (x, y) coordinates of the corners of the table IN THE IMAGE

        corners = np.zeros((4,2))
        radius = 5
        thickness = -1
        frame_HSV = cv.cvtColor(self.current_frame, cv.COLOR_BGR2HSV)

        height, width, _ = self.current_frame.shape
        # Blur the frame for style points B)


        frame_HSV = cv.GaussianBlur(frame_HSV, (3, 3), 0)
        frame_HSV = cv.medianBlur(frame_HSV, 3) #################### BLURRING 
        # print(frame_HSV.shape)

        ##Need 4 roi for each of the corners

        #roi1 is the bottom left corner (GREEN LED ON THE TOP LEFT SIDE OF THE SCREEN)
        roi1 = frame_HSV[0:int(height*self.percentofCorner), 0:int(width*self.percentofCorner)]
        #roi2 is the bottom right corner (GREEN LED ON THE TOP RIGHT SIDE OF THE SCREEN)
        roi2 = frame_HSV[int(height*0):int(height*self.percentofCorner), int(width*(1-self.percentofCorner)):width]
        #roi3 is the top left corner (GREEN LED ON THE BOTTOM LEFT SIDE OF THE SCREEN)
        roi3 = frame_HSV[int(height*(1-self.percentofCorner)):int(height*1), 0:int(width*self.percentofCorner)]
        #roi4 is the top right corner (GREEN LED ON THE BOTTOM RIGHT SIDE OF THE SCREEN)
        roi4 = frame_HSV[int(height*(1-self.percentofCorner)):int(height*1), int(width*(1-self.percentofCorner)):width]

        # real pics of the roi
        froi1 = self.current_frame[0:int(height*self.percentofCorner), 0:int(width*self.percentofCorner)]
        froi2 = self.current_frame[int(height*0):int(height*self.percentofCorner), int(width*(1-self.percentofCorner)):width]
        froi3 = self.current_frame[int(height*(1-self.percentofCorner)):int(height*1), 0:int(width*self.percentofCorner)]
        froi4 = self.current_frame[int(height*(1-self.percentofCorner)):int(height*1), int(width*(1-self.percentofCorner)):width]

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

        LED1 = self.findLED(green_mask1, 0, 0)
        LED2 = self.findLED(green_mask2, int(width* (1-self.percentofCorner)), 0)
        LED3 = self.findLED(green_mask3, 0, int(height*(1-self.percentofCorner)))
        LED4 = self.findLED(green_mask4, int(width*(1-self.percentofCorner)), int(height*(1-self.percentofCorner)))
        

        self.corners_image = np.array([LED1, LED2, LED3, LED4])

        count = 0
        color = (0, 255, 0)
        for corner in self.corners_image:
            if corner[0] == 0 and corner[1] == 0:
                corner[0] = self.PREV_corners_image[count][0]
                corner[1] = self.PREV_corners_image[count][1]
                self.corners_image[count] = self.PREV_corners_image[count]
            cv.circle(self.current_frame, (int(corner[0]), int(corner[1])), radius, color, thickness)
            count = count + 1
        self.PREV_corners_image = self.corners_image
        ##FOR VIEWING CORNERS OR MASKS
        # height, width = roi1.shape[:2]
        # roi2 = cv.resize(roi2, (width, height))
        # roi3 = cv.resize(roi3, (width, height))
        # roi4 = cv.resize(roi4, (width, height))

        # top_row = cv.hconcat([roi1, roi2])
        # bottom_row = cv.hconcat([roi3, roi4])
        # cornerFrame = cv.vconcat([top_row, bottom_row])
        # 
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
        if time.time() - self.serial_timer > self.waittime:
            self.ser.write(send.encode('utf-8'))
            self.serial_timer = time.time()
    async def blockBall(self):
        #block the ball with the rods
        #find the position of the ball
        ##block the ball with the rods

        self.GRod.blockBall(self.ball_pos_real)
        if self.ball_pos_real[0] < self.DRod.D_xlevel:
            self.DRod.blockBall(self.ball_pos_real)
        if self.ball_pos_real[0] < self.MRod.M_xlevel:
            self.MRod.blockBall(self.ball_pos_real)
        if self.ball_pos_real[0] < self.ARod.F_xlevel:
            self.ARod.blockBall(self.ball_pos_real)
    def changeSpeed(self, speed):
        ## changes the speed of the stepper motor in steps per second max of like 100000
        send = 'SPD' + str(speed) + '|' + str(speed)+ '|' + str(speed)+'|' + str(speed)+'|'+ self.ENDCHAR
        self.ser.write(send.encode('utf-8'))
    def measureLoopTime(self):
        self.tock = time.time()
        self.tickertocker = self.tickertocker + self.tock - self.tick
        self.tockercounter = 1 + self.tockercounter
        if self.tockercounter > 100:
            print('Motor States\n')
            print('G_motor' + str(self.GRod.returnMotorState()))
            print('\nD_motor' + str(self.DRod.returnMotorState()))
            print('\nM_motor' + str(self.MRod.returnMotorState()))
            print('\nA_motor' + str(self.ARod.returnMotorState()))
            print('\nBall Position\n')
            print(self.ball_pos_real)
            print('\nLoop Time\n')
            print(self.tickertocker/self.tockercounter)
            print()
            self.tockercounter = 0
            self.tickertocker = 0

    ##############################MAIN LOOP################################
    async def run(self):
        gc.collect()
        await self.GRod.clearFaults()
        await self.DRod.clearFaults()
        await self.MRod.clearFaults()
        await self.ARod.clearFaults()
        time.sleep(2)  # Wait for the connection to be established
        
        self.tockercounter = 0
        self.tickertocker = 0

        while True: 
            self.tick = time.time()
            # call this as often as possible to update current frames
            ret = self.UpdateFrame()
            if not ret:
                break
            ##Find the ball position IRL
            self.getBallPos() ## MAKE FASTER
            
            ##insert strategy here
            await self.blockBall()
            await self.moveRods()

            self.ShowField()
            self.measureLoopTime()
            if cv.waitKey(30) & 0xFF == ord('q'):
                break
        self.cam.release()
        cv.destroyAllWindows()

if __name__ == '__main__':
    Foos = FoosBot()
    asyncio.run(Foos.run())