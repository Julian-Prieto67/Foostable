import serial
import time
import cv2 as cv
import numpy as np

class Arduino:
    def __init__(self):
        self.arduino_port = 'COM5'  # Arduino's port
        self.baud_rate = 115200 #Arduino baud rate
        self.ser = serial.Serial(self.arduino_port,self.baud_rate, timeout = 1)
        self.serial_timer = time.time()
        self.waittime = 0.005 ##time to wait between serial messages or the messages jumble
        self.max_speed = 0 ##this is the max speed reached
        self.screen_offset = (0, 50)

    def write(self, message):
        #messsage is a string
        if time.time() - self.serial_timer > self.waittime:
            self.ser.write(message.encode())
            self.serial_timer = time.time()

    def tupleadd(self, a, b):
        return tuple(a + b for a, b in zip(a, b))
    def plotRodpos(self, rod, window):
        ##add text for the pos of rods
        text = f"{rod.name}: {rod.returnRodPosmm():.2f}"
        # cv.putText(window, text, ball_pos_image, cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        text_loc = (int(rod.x_level), 20)
        cv.putText(window, text, text_loc, cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    def plotPlayerPos(self, rod, window):
        ##add text for the pos of rods
        for y in rod.returnPlayerPos():
            player_num = rod.returnPlayerPos().index(y)
            text = f"Player {player_num} Pos: {y[1]:.3f}"
            text_loc = (int(rod.x_level)+20, int(y[1]))
            cv.putText(window, text, text_loc, cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    def UpdateGui(self, ball_pos_real, corners_real, GRod, DRod, MRod, ARod, ball_speed, ball_trajectory, rodsKicking):
        # Create a green window
        #every pixel is a mm
        window = np.zeros((660+self.screen_offset[1], 1203 +self.screen_offset[0], 3), dtype=np.uint8)
        ##make it green 
        window[:, :] = (0, 50, 0)
        # Draw white dots at the positions in ball_trajectory
        for point in ball_trajectory:
            point = np.array(point).astype(int)
            point = tuple(point + self.screen_offset)
            cv.circle(window, point, 5, (255, 255, 255), -1)
        # Draw green circles at the positions of self.corners_image
        for corner in corners_real:
            cv.circle(window, self.tupleadd(tuple(corner.astype(int)),self.screen_offset), 2, (0, 255, 0), -1)

        # Draw a yellow circle at the position of where the ball is in the real world
        ball_pos_real = self.tupleadd(ball_pos_real.astype(int), self.screen_offset)
        cv.circle(window, ball_pos_real, 5, (0, 255, 255), -1)
        # Add text at the position of the circle
        text = f"Ball Position: {ball_pos_real}\n Ball Speed: {ball_speed}"
        # cv.putText(window, text, ball_pos_image, cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        text_loc = (int(window.shape[1] / 2), 50)
        cv.putText(window, text, text_loc, cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        self.plotRodpos(GRod, window)
        self.plotRodpos(DRod, window)
        self.plotRodpos(MRod, window)
        self.plotRodpos(ARod, window)
        self.plotPlayerPos(GRod, window)
        self.plotPlayerPos(DRod, window)
        self.plotPlayerPos(MRod, window)
        self.plotPlayerPos(ARod, window)

        
    
        # Draw the foosmen as dots 
        for i, player in enumerate([GRod, DRod, MRod, ARod]):
            posList = player.returnPlayerPos()
            if not rodsKicking[i]:
                for index, pos in enumerate(posList):
                    pos = np.array(pos).astype(int)
                    if index == player.returnBlockingPlayer():
                        cv.circle(window, self.tupleadd(pos,self.screen_offset), 2, (0, 255, 255), -1)
                    else:
                        cv.circle(window, self.tupleadd(pos,self.screen_offset), 2, (0, 0, 255), -1)
            else:
                for index, pos in enumerate(posList):
                    pos = np.array(pos).astype(int)
                    if index == player.returnBlockingPlayer():
                        cv.circle(window, self.tupleadd(pos,self.screen_offset), 5, (0, 255, 255), -1)
                    else:
                        cv.circle(window, self.tupleadd(pos,self.screen_offset), 5, (0, 0, 255), -1)

        self.gui = window
    def showGUI(self):
        showthis = cv.resize(self.gui, (640, 480))
        cv.imshow("GUI", showthis)
        cv.waitKey(1)


class ArduinoFake(Arduino):
    def __init__(self):
        self.gui = None
        self.screen_offset = (0, 50)
    def write(self, message):
        pass
    def tupleadd(self, a, b):
        return tuple(a + b for a, b in zip(a, b))
    def plotRodpos(self, rod, window):
        ##add text for the pos of rods
        text = f"{rod.name}: {rod.returnRodPosmm():.2f}"
        # cv.putText(window, text, ball_pos_image, cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        text_loc = (int(rod.x_level), 20)
        cv.putText(window, text, text_loc, cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    def plotPlayerPos(self, rod, window):
        ##add text for the pos of rods
        for y in rod.returnPlayerPos():
            player_num = rod.returnPlayerPos().index(y)
            text = f"Player {player_num} Pos: {y[1]:.3f}"
            text_loc = (int(rod.x_level)+20, int(y[1]))
            cv.putText(window, text, text_loc, cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    def UpdateGui(self, ball_pos_real, corners_real, GRod, DRod, MRod, ARod, ball_speed, ball_trajectory, rodsKicking):
        # Create a green window
        #every pixel is a mm
        window = np.zeros((660+self.screen_offset[1], 1203 +self.screen_offset[0], 3), dtype=np.uint8)
        ##make it green 
        window[:, :] = (0, 50, 0)
        # Draw white dots at the positions in ball_trajectory
        for point in ball_trajectory:
            point = np.array(point).astype(int)
            point = tuple(point + self.screen_offset)
            cv.circle(window, point, 5, (255, 255, 255), -1)
        # Draw green circles at the positions of self.corners_image
        for corner in corners_real:
            cv.circle(window, self.tupleadd(tuple(corner.astype(int)),self.screen_offset), 2, (0, 255, 0), -1)

        # Draw a yellow circle at the position of where the ball is in the real world
        ball_pos_real = self.tupleadd(ball_pos_real.astype(int), self.screen_offset)
        cv.circle(window, ball_pos_real, 5, (0, 255, 255), -1)
        # Add text at the position of the circle
        text = f"Ball Position: {ball_pos_real}\n Ball Speed: {ball_speed}"
        # cv.putText(window, text, ball_pos_image, cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        text_loc = (int(window.shape[1] / 2), 50)
        cv.putText(window, text, text_loc, cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        self.plotRodpos(GRod, window)
        self.plotRodpos(DRod, window)
        self.plotRodpos(MRod, window)
        self.plotRodpos(ARod, window)
        self.plotPlayerPos(GRod, window)
        self.plotPlayerPos(DRod, window)
        self.plotPlayerPos(MRod, window)
        self.plotPlayerPos(ARod, window)

        
    
        # Draw the foosmen as dots 
        for i, player in enumerate([GRod, DRod, MRod, ARod]):
            posList = player.returnPlayerPos()
            if not rodsKicking[i]:
                for index, pos in enumerate(posList):
                    pos = np.array(pos).astype(int)
                    if index == player.returnBlockingPlayer():
                        cv.circle(window, self.tupleadd(pos,self.screen_offset), 2, (0, 255, 255), -1)
                    else:
                        cv.circle(window, self.tupleadd(pos,self.screen_offset), 2, (0, 0, 255), -1)
            else:
                for index, pos in enumerate(posList):
                    pos = np.array(pos).astype(int)
                    if index == player.returnBlockingPlayer():
                        cv.circle(window, self.tupleadd(pos,self.screen_offset), 5, (0, 255, 255), -1)
                    else:
                        cv.circle(window, self.tupleadd(pos,self.screen_offset), 5, (0, 0, 255), -1)

        self.gui = window
    def showGUI(self):
        showthis = cv.resize(self.gui, (640, 480))
        cv.imshow("GUI", showthis)
        cv.waitKey(1)


    