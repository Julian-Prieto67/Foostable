from ArduinoClass import Arduino
from RodClass import RodReal, RodFake
from ArduinoClass import Arduino, ArduinoFake
import numpy as np
import time
import gc
import asyncio
from collections import deque   


class Controller():
    def __init__(self, playback = False):
        self.playback = playback

        
        self.ball_pos_real = np.array([0, 0])
        ##below is the current Ball state in the format [x, y, dx, dy] as a column vector
        self.ball_speed = np.array([0, 0])
        self.ball_state = np.array([
        [self.ball_pos_real[0]],
        [self.ball_pos_real[1]],
        [self.ball_speed[0]],
        [self.ball_speed[1]]
        ])
        self.ball_pos_list = deque(maxlen = 2)
        self.ball_time = deque(maxlen = 2)
        self.ball_trajectory = []
        self.minSpeedthreshold = 150 ##minimum speed of the ball to be considered in possesion

        self.ENDCHAR = '#'
        ##Dependency Injection for rods and arduino
        
        self.corners_real = np.array([[7/8*25.4, 25.25*25.4], [46.375*25.4, 25.25*25.4],[0.875*25.4, 0.625*25.4], [46.5*25.4, 0.75*25.4]])
        if not self.playback:
            self.GRod = RodReal(1) ##initializes goalie rod
            self.DRod = RodReal(2) ##initializes defense rod
            self.MRod = RodReal(3) ##initializes midfield rod
            self.ARod = RodReal(4) ##initializes attack rod
            self.ser = Arduino() ##initializes arduino
        else:
            self.GRod = RodFake(1) ##initializes goalie rod
            self.DRod = RodFake(2) ##initializes defense rod
            self.MRod = RodFake(3) ##initializes midfield rod
            self.ARod = RodFake(4) ##initializes attack rod
            self.ser = ArduinoFake() ##initializes arduino
        self.possesion = 0  ##0 or 1, 1 means I have the ball
        self.loop_start_time = time.time()
        self.loop_time = 0

    def CalibrateRods(self):
        #calibrates the rods to their starting position
        #by sending "HOM" to the arduino
        send = 'HOM' + self.ENDCHAR
        self.ser.write(send)
        print("Calibrating Rods")

    async def initializeClass(self):
        gc.collect()
        await self.GRod.clearFaults()
        await self.DRod.clearFaults()
        await self.MRod.clearFaults()
        await self.ARod.clearFaults()
        time.sleep(2)
        self.CalibrateRods()
    
    def showGUI(self):
        self.ser.UpdateGui(self.ball_pos_real, self.corners_real, self.GRod, self.DRod, self.MRod, self.ARod, self.ball_speed, self.ball_trajectory)
        self.ser.showGUI()

    def ControlTimer(self):
    ##returns the amount of time that has passed since the last time it was called
    ##prints the average of 100 calls to the function
        self.loop_start_time
        current_time = time.time()
        time_passed = current_time - self.loop_start_time
        self.loop_time = time_passed
        self.loop_start_time = current_time

    def UpdateBallPos(self, ball_pos):
        self.ball_pos_real = ball_pos
        self.ball_trajectory = []
        self.ball_time.append(time.time())
        self.ball_pos_list.append((ball_pos[0], ball_pos[1]))

    def changeSpeed(self, speed):
        ## changes the speed of the stepper motor in steps per second max of like 100000
        send = 'SPD' + str(speed) + '|' + str(speed)+ '|' + str(speed)+'|' + str(speed)+'|'+ self.ENDCHAR
        self.ser.write(send)

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

        ##if the ball is not found, go home
        if np.all(self.ball_pos_real) == 0:
            self.GRod.goHome()
            self.DRod.goHome()
            self.MRod.goHome()
            self.ARod.goHome()
            send = self.GRod.returnRodPos() + self.DRod.returnRodPos() + self.MRod.returnRodPos() + self.ARod.returnRodPos() + self.ENDCHAR
        self.ser.write(send)
        
    def blockBall(self):
        #block the ball with the rods
        #find the position of the ball
        ##block the ball with the rods
        self.MRod.blockBall(self.ball_pos_real)
        self.ARod.blockBall(self.ball_pos_real)
    def goalTend(self):
        ##Advanced blocking function that blocks only with the defense and keeps goalie still until trajectory is found
        if self.ball_trajectory == []:
            self.GRod.setRodPos(self.GRod.RodRange[0]+self.GRod.RodRange[1]//2)
            self.DRod.blockBall(self.ball_pos_real)
        else:
            self.blockTrajectory(self.GRod)
            self.DRod.blockBall(self.ball_pos_real)
    def blockTrajectory(self, rod):
        #block the ball with the rods using the trajectory
        #set the rod position to be the y position where the x trajectory intersects the x-level of the rod
        if self.ball_trajectory == []:
            return
        for i in range(len(self.ball_trajectory)):
            if self.ball_trajectory[i][0] > rod.x_level:
                rod.setRodPos(self.ball_trajectory[i][1])
                break
        rod.blockBall(self.ball_pos_real)
    
    def generateTrajectory(self):
        ##generate the trajectory of the ball
        ##this is a list of tuples of the form (ball_pos, ball_time)
        ##where ball_pos is the position of the ball and ball_time is the time the ball was at that position
        ##this is used to predict where the ball will be in the future
        if len(self.ball_pos_list) < 2 or np.all(np.subtract(self.ball_pos_list[0], self.ball_pos_list[1])) == 0:
            return
        
        predictTime = 50 ##looks ahead x many steps in the future
        self.ball_trajectory = []
        ball_speed_x = (self.ball_pos_list[1][0] - self.ball_pos_list[0][0]) / (self.ball_time[1] - self.ball_time[0])
        ball_speed_y = (self.ball_pos_list[1][1] - self.ball_pos_list[0][1]) / (self.ball_time[1] - self.ball_time[0])
        self.ball_speed = np.array([ball_speed_x, ball_speed_y])
        #create local variable for speed and pos that can be changed
        ball_speed = self.ball_speed
        ball_pos = self.ball_pos_list[0]
        if np.linalg.norm(ball_speed) < self.minSpeedthreshold:
            return
        self.ball_trajectory.append(ball_pos)
        for i in range(predictTime):
            ##predict where the ball will be for next loop time
            ball_pos = self.ball_trajectory[i] + self.ball_speed * self.loop_time
        
            ##if the ball is going to hit the wall, change the speed and recalculate the position
            if ball_pos[0] > 46.375*25.4 or ball_pos[0] < 0.875*25.4:
                ball_speed[0] = -ball_speed[0]
            if ball_pos[1] > 25.25*25.4 or ball_pos[1] < 0.625*25.4:
                ball_speed[1] = -ball_speed[1]
                
            ball_pos = self.ball_trajectory[i] + ball_speed * self.loop_time
            ##add the ball position to the trajectory
            self.ball_trajectory.append(ball_pos)

    def findPossesion(self):
        ## first check if the speed is low enough to be in possesion and if the ball pos is not 0,0
        ## If the ball is in a region where the rods can reach it, then we have possesion
        if np.any(self.ball_pos_real) == 0 or np.linalg.norm(self.ball_speed) > self.minSpeedthreshold:
            self.possesion = 0
            return
        ##check if the ball is in the region of the rods
        if self.GRod.checkBall(self.ball_pos_real) or self.DRod.checkBall(self.ball_pos_real) or self.MRod.checkBall(self.ball_pos_real) or self.ARod.checkBall(self.ball_pos_real):
            self.possesion = 1
        else:
            self.possesion = 0
    def checkBall(self):
        ##check if the ball is in the region of the rods
        if self.GRod.checkBall(self.ball_pos_real) or self.DRod.checkBall(self.ball_pos_real) or self.MRod.checkBall(self.ball_pos_real) or self.ARod.checkBall(self.ball_pos_real):
            return 1
        else:
            return 0
        
    #######FIX KICKING
    async def Play(self):
        ##Execute the control loop
        self.ControlTimer()
        self.findPossesion()
        self.generateTrajectory()
        if self.possesion == 0:
            ##if the ball is not in my possesion, block the ball
            if self.ball_trajectory == []:
                ##if the ball trajectory is empty, block the ball using the current position
                self.blockBall()
                self.goalTend()
            else:
                ##if the ball trajectory is not empty, block the ball using the trajectory
                ##set the rod position to be the y position where the x trajectory intersects the x-level of the rod
                self.blockTrajectory(self.GRod)
                self.blockTrajectory(self.DRod)
                self.blockTrajectory(self.MRod)
                self.blockTrajectory(self.ARod)
        else:
            ##if the ball is in my possesion wait until the ball is in the right position
            ##then kick the ball
            if self.GRod.checkBall(self.ball_pos_real):
                self.GRod.blockBall(self.ball_pos_real)
                await self.GRod.kickAtWill(self.ball_pos_real)
            elif self.DRod.checkBall(self.ball_pos_real):
                self.DRod.blockBall(self.ball_pos_real)
                await self.DRod.kickAtWill(self.ball_pos_real)
            elif self.MRod.checkBall(self.ball_pos_real):
                self.MRod.blockBall(self.ball_pos_real)
                await self.MRod.kickAtWill(self.ball_pos_real)
            elif self.ARod.checkBall(self.ball_pos_real):
                self.ARod.blockBall(self.ball_pos_real)
                await self.ARod.kickAtWill(self.ball_pos_real)
        await self.moveRods()

