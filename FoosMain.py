import multiprocessing as mp
import time
import cv2 as cv
import numpy as np
from FoosBot import FoosBot
from ArduinoClass import Arduino
from ControlsClass import Controller
from VisionClass import Vision
import asyncio



playback = True


global start_time
global timer_counter
global Accumulated_time
global start_time2
global timer_counter2
global Accumulated_time2
start_time = time.time()
timer_counter = 0
Accumulated_time = 0
start_time2 = time.time()
timer_counter2 = 0
Accumulated_time2 = 0

def ControlTimer():
    ##returns the amount of time that has passed since the last time it was called
    ##prints the average of 100 calls to the function
    global start_time
    global timer_counter
    global Accumulated_time
    current_time = time.time()
    time_passed = current_time - start_time
    start_time = current_time

    Accumulated_time = Accumulated_time + time_passed 
    timer_counter += 1
    if timer_counter > 1e5:
        print("Average time for 1e5 calls to Controls function:")
        print(Accumulated_time / timer_counter)
        timer_counter = 0
        Accumulated_time = 0
    return time_passed
def VisionTimer():
    ##returns the amount of time that has passed since the last time it was called
    ##prints the average of 100 calls to the function
    global start_time2
    global timer_counter2
    global Accumulated_time2
    current_time = time.time()
    time_passed = current_time - start_time2
    start_time2 = current_time

    Accumulated_time2 = Accumulated_time2 + time_passed 
    timer_counter2 += 1
    if timer_counter2 > 100:
        print("Average time for 100 calls to Vision function:")
        print(Accumulated_time2 / timer_counter2)
        timer_counter2 = 0
        Accumulated_time2 = 0
    return time_passed

def vision_process(queue, playback):
    vision = Vision(playback)
    while True:
    ##find the ball and send data to the queue
        ##FIND THE POSITION OF THE BALL
        vision.UpdateFrame()
        BallPos = vision.getBallPos()
        print(BallPos)
        ##send the data to the queue
        queue.put(BallPos)
        ##Time the function
        VisionTimer()


def Controls_process(queue, playback):
    Controls = Controller(playback)
    asyncio.run(Controls.initializeClass())
    while True:
    ##get the data from the queue and move the rods
        if not queue.empty():
            ##get the data from the queue A NEW POSITION IS FOUND
            BallPos = queue.get()
            ##STRATEGY HERE
            ##Move the rods to the new position
            Controls.newBallPos(BallPos)
        else:
            ##No new position found, use KF to interpolate
            BallPos = Controls.KalmanFilter()
        

        ##move the rods to block the ball
        Controls.blockBall()

        ##Time the function
        ControlTimer()


if __name__ == "__main__":
    ##Create the queue
    queue = mp.Queue()

    ##Create the processes
    vision = mp.Process(target=vision_process, args=(queue, playback))
    controls = mp.Process(target=Controls_process, args=(queue,playback))

    ##Start the processes
    vision.start()
    controls.start()

    ##Not sure if this is needed because the above processes are always running?
    vision.join()
    controls.join()
    