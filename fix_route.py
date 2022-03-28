from distutils.log import debug
from time import time
import numpy as np
import threading
from matplotlib import pyplot as plt
import lds_driver
import time
from data_rw import data_send, init_data_rw, data_read

def record_input(cur_state, x_stack, y_stack, spd_stack, spd, ang, t_interval = 0.01):
    if (x_stack == []):
        # Handle initial condition
        x_stack.append(0.0)
        y_stack.append(0.0)
        spd_stack.append(0.5)  #this may cause BUG
        
    x_new = x_stack[-1] + spd * t_interval * np.cos(ang/180*np.pi)
    y_new = y_stack[-1] + spd * t_interval * np.sin(ang/180*np.pi)
    x_stack.append(x_new)
    y_stack.append(y_new)
    spd_stack.append(spd)

    return
    
def record_output(x_stack, y_stack, spd_stack, cur_point):
    
    length = len(x_stack)
    step_size = 1
    now_run, nxt_run = cur_point, cur_point + 1
    x_direc = x_stack[nxt_run] - x_stack[now_run]
    y_direc = y_stack[nxt_run] - y_stack[now_run]
    ins_ang = np.arctan2(y_direc, x_direc)/np.pi*180 # radius, should change to degree
    ins_ang += 180
    ins_spd = spd_stack[now_run]    #The speed will cause problem if there is a 0

    global target_v, target_angle
    target_v, target_angle = ins_spd, ins_ang

    return 

def lds_hold(cur_state, ser):
    if cur_state == 0:
        rge = lds_driver.lds_poll(ser)
        if min(rge) < 1:
            return 1
    return 0

# This one to put global vars
x_stack,y_stack,spd_stack  = [],[],[]
last_angle, truth_angle= None, None # This one used for null shift
cur_v, cur_angle, mode, error_status = None, None, None, None
target_v, target_angle, cur_point = 0, 0, 0
record_end = False # Indicating record ends or not


def stm32_communication():
    '''
        This code keep talking to stm32, and update info in the global vars
    '''
    global x_stack, y_stack, spd_stack, last_angle, truth_angle
    global cur_v, cur_angle, mode, error_status, target_v, target_angle, record_end, cur_point
    dev = init_data_rw() # will be used later in the communication
    
    while(1):
        # Mode 0: nano control; Mode 1: Remote control; Mode 2: Programming 
        cur_v, cur_angle, mode, error_status = data_read(dev)
        if (truth_angle == None):
            truth_angle = cur_angle
            last_angle = cur_angle
        elif (last_angle-cur_angle > 0.2): # handle null shift
            truth_angle = cur_angle
        last_angle = cur_angle
        
        if (mode == 2):
            if (record_end == 1):
                record_end, cur_point = 0, 0
                x_stack, y_stack, spd_stack = [],[],[]

            record_input(mode, x_stack, y_stack, spd_stack, cur_v, truth_angle)
        elif (mode == 0):
            record_end = 1

        if True: #(mode == 0):           #THIS IS CHANGED FOR TESTING 2022.03.28
            #delta_angle = (target_angle - last_angle + 180 ) % 360  #this is WRONG
            
            delta_angle = last_angle - target_angle
            if delta_angle > 180:
                delta_angle = delta_angle - 360
            elif delta_angle < -180:
                delta_angle = delta_angle + 360
            delta_angle += 180
            data = [target_v, delta_angle, mode, error_status]
            global ang_out
            ang_out = delta_angle
            data_send(data, dev)

def route_decision():
    '''
        This function decide the route based on given info, only works when mode become 0 
    '''
    global x_stack, y_stack, spd_stack, last_angle, truth_angle
    global cur_v, cur_angle, mode, error_status, target_v, target_angle, record_end, cur_point
    global debug_mode
    if debug_mode == 0: 
        ser = lds_driver.lds_driver_init() # init for lidar   
    while (1):
        if (record_end == True) and (debug_mode != 2):
            if debug_mode != 1:
                if lds_hold(ser) == 1:
                    print('LDS Hold Start')
                    target_v, target_angle = 0, last_angle
                    time.sleep(5)
                    print('LDS Hold End')
            
            record_output(x_stack, y_stack, spd_stack, cur_point)
            cur_point += 1
            if (cur_point == len(x_stack) - 1):
                cur_point = 0
            
        
def monitor():
    '''
        This function monitors the running of the system, prints out needed elements for debugging
        Also is for doing manual debug
    '''
    global x_stack, y_stack, spd_stack, last_angle, truth_angle
    global cur_v, cur_angle, mode, error_status, target_v, target_angle, record_end, cur_point
    global ang_out, debug_mode 
    
    # Debug mode 0: will run the full fix route program
    # Debug mode 1: will run the fix route program without LDS hold
    # Debug mode 2: will run manual route assignment, this will stop route_desicion() from changing the two target variable
    debug_mode = 2
    while(1):
        if debug_mode == 2:
            target_v = 0
            target_angle = 0
            
        print('read: ', cur_v, cur_angle, ' || ', 'send: ', target_v, ang_out, ' || ', 'mode: ', mode)

    

def fix_route_main():
    ''' 
        This code implement the fix route algorithm, forcing the robot to record the message, memorize it,
        then go along this route. When detecting somebody nearby, stop.
    '''
    # Now comes the recording mode
    t1 = threading.Thread(target = stm32_communication)
    t2 = threading.Thread(target = route_decision)
    t3 = threading.Thread(target = monitor)
    t1.start()
    t2.start()
    t3.start()
    while(1):
        pass

fix_route_main()

    
