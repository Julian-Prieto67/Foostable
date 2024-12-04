import time

global start_time
global timer_counter
global Accumulated_time

start_time = time.time()
timer_counter = 0
Accumulated_time = 0

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
    if timer_counter > 1e2:
        print("Average time for 100 calls to Controls function:")
        print(Accumulated_time / timer_counter)
        timer_counter = 0
        Accumulated_time = 0
    return time_passed


list = []
for i in range(102):
    list.append(ControlTimer())
