from time import time
import numpy as np
from matplotlib import pyplot as plt
import lds_driver
import time

def read_state():
    #this should read from stm32 and return the current mode
    #assume 0 is for recording route and 1 is for running
    current_state = 0
    
    return current_state

def record_input(x_stack, y_stack, spd_stack, spd, ang, t_interval = 0.01):
    if (x_stack == []):
        # Handle initial condition
        x_stack.append(0.0)
        y_stack.append(0.0)
        spd_stack.append(0.5)  #this may cause BUG
        
    if read_state() == 0:
        x_new = x_stack[-1] + spd * t_interval * np.cos(ang)
        y_new = y_stack[-1] + spd * t_interval * np.sin(ang)
        x_stack.append(x_new)
        y_stack.append(y_new)
        spd_stack.append(spd)

    return
    
def record_output(x_stack, y_stack, spd_stack, t_interval = 0.01):
    length = len(x_stack)
    step_size = 1
    nxt_run = 0
    now_run = length
    if read_state() == 1:
        x_direc = x_stack[nxt_run] - x_stack[now_run]
        y_direc = y_stack[nxt_run] - y_stack[now_run]
        ins_ang = np.arctan2(y_direc, x_direc)
        ins_spd = spd_stack[now_run]    #The speed will cause problem if there is a 0

        now_run = nxt_run
        if nxt_run + step_size > length: 
            nxt_run = 0
        else:
            nxt_run = nxt_run + step_size     
        # Next, should send ins_ang, ins_spd to stm32  
    return 

def lds_hold(ser, rge_old):
    if read_state() == 1:
        hold = 0
        rge = lds_driver.lds_poll(ser)
        if min(rge) < 1 and max(rge_old - rge) > 0.8:
            hold = 1
    return rge, hold

def fix_route_main(t = 0.01):
    x_stack = []
    y_stack = []
    spd_stack = []
    ser = lds_driver.lds_driver_init()
    rge_old = lds_driver.lds_poll(ser)
    
    while 1:
        record_input(x_stack, y_stack, spd_stack, spd, ang)
        record_output(x_stack, y_stack, spd_stack)
        rge_old, hold = lds_hold(ser, rge_old)
        if hold == 1:
            time.sleep(5)
    return

fix_route_main()

    
