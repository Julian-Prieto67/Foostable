import serial
import time
import cv2 as cv
import numpy as np

class Arduino:
    def __init__(self):
        self.arduino_port = 'COM3'  # Arduino's port
        self.baud_rate = 115200 #Arduino baud rate
        self.ser = serial.Serial(self.arduino_port,self.baud_rate, timeout = 1)
        self.serial_timer = time.time()
        self.waittime = 0.1 ##time to wait between serial messages
    def write(self, message):
        if time.time() - self.serial_timer < self.waittime:
            self.ser.write(message.encode())
            self.serial_timer = time.time()
    def UpdateGui(self, ball_pos_real, corners_image, ball_pos_image, GRod, DRod, MRod, ARod):
        pass
    def showGUI(self):
        pass


class ArduinoFake(Arduino):
    def __init__(self):
        self.gui = None
    def write(self, message):
        pass
    def UpdateGui(self, ball_pos_real, corners_image, corners_real, ball_pos_image, GRod, DRod, MRod, ARod):
        # Create a green window
        #every pixel is a mm
        window = np.zeros((660, 1203, 3), dtype=np.uint8)
        window[:, :] = (0, 50, 0)
    
        # Draw green circles at the positions of self.corners_image
        for corner in corners_real:
            cv.circle(window, tuple(corner.astype(int)), 2, (0, 255, 0), -1)

        # Draw a yellow circle at the position of where the ball is in the real world
        ball_pos_real = tuple(ball_pos_real.astype(int))
        cv.circle(window, ball_pos_real, 5, (0, 255, 255), -1)
        # Add text at the position of the circle
        text = f"Ball Position: {ball_pos_real}"
        # cv.putText(window, text, ball_pos_image, cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        text_loc = (int(window.shape[1] / 2), 20)
        cv.putText(window, text, text_loc, cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    
        # Draw the foosmen as dots 
        for player in [GRod, DRod, MRod, ARod]:
            posList = player.returnPlayerPos()
            for pos in posList:
                pos = np.array(pos).astype(int)
                cv.circle(window, pos, 2, (0, 0, 255), -1)
            
        self.gui = window
    def showGUI(self):
        cv.imshow("GUI", self.gui)

    