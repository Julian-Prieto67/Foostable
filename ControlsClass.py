from ArduinoClass import Arduino
from RodClass import RodReal, RodFake
from ArduinoClass import Arduino, ArduinoFake
import numpy as np
import time
import gc
import asyncio


class Controller():
    def __init__(self, playback = False):
        self.playback = playback

        self.ball_time = time.time()
        self.ball_pos_real = np.array([0, 0])
        ##below is the current Ball state in the format [x, y, dx, dy] as a column vector
        self.ball_speed = np.array([0, 0])
        self.ball_state = np.array([
    [self.ball_pos_real[0]],
    [self.ball_pos_real[1]],
    [self.ball_speed[0]],
    [self.ball_speed[1]]
])
        self.ENDCHAR = '#'

        
        ##Dependency Injection for rods and arduino
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
    def CalibrateRods(self):
        #calibrates the rods to their starting position
        #by sending "HOM" to the arduino
        send = 'HOM' + self.ENDCHAR
        self.ser.write(send)
    async def initializeClass(self):
        gc.collect()
        await self.GRod.clearFaults()
        await self.DRod.clearFaults()
        await self.MRod.clearFaults()
        await self.ARod.clearFaults()
        time.sleep(2)
        self.CalibrateRods()
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
        self.ser.write(send)
    def changeSpeed(self, speed):
        ## changes the speed of the stepper motor in steps per second max of like 100000
        send = 'SPD' + str(speed) + '|' + str(speed)+ '|' + str(speed)+'|' + str(speed)+'|'+ self.ENDCHAR
        self.ser.write(send)
    def newBallPos(self, ball_pos):
        ##updates the ball position
        self.ball_pos_real = ball_pos
        self.ball_time = time.time()

    def KalmanFilter(self):
        #Kalman filter for the ball
        #predict the next position of the ball
        #update the position of the ball

        pass
        
    async def blockBall(self):
        #block the ball with the rods
        #find the position of the ball
        ##block the ball with the rods

        self.GRod.blockBall(self.ball_pos_real)
        self.DRod.blockBall(self.ball_pos_real)
        self.MRod.blockBall(self.ball_pos_real)
        self.ARod.blockBall(self.ball_pos_real)
    
    def run(self):
        ##Execute the control loop
        pass
