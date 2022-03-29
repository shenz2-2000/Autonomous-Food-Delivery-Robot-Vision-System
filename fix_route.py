from distutils.log import debug
from time import time
import numpy as np
import threading
from matplotlib import pyplot as plt
import lds_driver
import time
from data_rw import data_send, init_data_rw, data_read

def record_input(cur_state, x_stack, y_stack, spd_stack, spd_x, spd_y, ang, t_interval = 0.01):
    if (x_stack == []):
        # Handle initial condition
        x_stack.append(0.0)
        y_stack.append(0.0)
        spd_stack.append([0.0, 0.0])  #this may cause BUG
        
    x_new = x_stack[-1] + t_interval * (np.cos(ang/180*np.pi) * spd_y + np.cos((ang-90)/180*np.pi) * spd_x)
    y_new = y_stack[-1] + t_interval * (np.sin(ang/180*np.pi) * spd_y + np.sin((ang-90)/180*np.pi) * spd_x)
    x_stack.append(x_new)
    y_stack.append(y_new)
    spd = np.sqrt(spd_x**2+spd_y**2)
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
last_angle, truth_angle= 0, 0 # This one used for null shift
cur_vx, cur_vy, cur_angle, mode, error_status = 0, 0, 0, 0, 0
target_v, target_angle, cur_point = 0, 0, 0
record_end = False # Indicating record ends or not


def stm32_communication():
    '''
        This code keep talking to stm32, and update info in the global vars
    '''
    global x_stack, y_stack, spd_stack, last_angle, truth_angle
    global cur_vx, cur_vy, cur_angle, mode, error_status, target_v, target_angle, record_end, cur_point
    global ang_out, delta_angle_manual
    dev = init_data_rw() # will be used later in the communication
    ang_out = 0
    delta_angle_manual = 0

    cur_vx, cur_vy, cur_angle, mode, error_status = data_read(dev)
    if debug_mode == 2:
        target_angle = cur_angle
    while(1):
        # Mode 0: nano control; Mode 1: Remote control; Mode 2: Programming 
        cur_vx, cur_vy, cur_angle, mode, error_status = data_read(dev)

        if (truth_angle == 0):
            truth_angle = cur_angle
            last_angle = cur_angle
        elif (last_angle-cur_angle > 0.2): # handle null shift
            truth_angle = cur_angle
        last_angle = cur_angle
        if (mode == 2):
            if (record_end == 1):
                record_end, cur_point = 0, 0
                x_stack, y_stack, spd_stack = [],[],[]

            record_input(mode, x_stack, y_stack, spd_stack, cur_vx, cur_vy, truth_angle)
        elif (mode == 0):
            record_end = 1

        if True: #(mode == 0):           #THIS IS CHANGED FOR TESTING 2022.03.28
            delta_angle = last_angle - target_angle
            if delta_angle > 180:
                delta_angle = delta_angle - 360
            elif delta_angle < -180:
                delta_angle = delta_angle + 360

            #The plus 180 work is moved to data_rw, here is only [-180,180] indicating the desired degree
            if debug_mode != 2:
                data = [target_v, delta_angle, mode, error_status]
                ang_out = delta_angle
                data_send(data, dev)
            elif debug_mode == 2:
                data = [target_v, delta_angle_manual, mode, error_status]
                ang_out = delta_angle_manual
                data_send(data, dev)

def route_decision():
    '''
        This function decide the route based on given info, only works when mode become 0 
    '''
    global x_stack, y_stack, spd_stack, last_angle, truth_angle
    global cur_vx, cur_vy, cur_angle, mode, error_status, target_v, target_angle, record_end, cur_point
    global debug_mode
    if debug_mode == 0:  #Onlt init if lds_fix_route is chosen 
        ser = lds_driver.lds_driver_init()   
    while (1):
        if (record_end == True) and (debug_mode != 2) and (debug_mode != 3):  #output fix route when debug mode is not manual
            if debug_mode == 0: 
                if lds_hold(ser) == 1:
                    print('LDS Hold Start')
                    target_v, target_angle = 0, last_angle
                    time.sleep(5)
                    print('LDS Hold End')
            
            record_output(x_stack, y_stack, spd_stack, cur_point) # If debug_mode == 1, then lds will be skipped and fixroute will run without lds
            cur_point += 1
            if (cur_point == len(x_stack) - 1):
                cur_point = 0
            
        
def monitor():
    '''
        This function monitors the running of the system, prints out needed elements for debugging
        Also is for doing manual debug
    '''
    global x_stack, y_stack, spd_stack, last_angle, truth_angle
    global cur_vx, cur_vy, cur_angle, mode, error_status, target_v, target_angle, record_end, cur_point
    global ang_out, debug_mode, delta_angle_manual
    ang_out = 0
    # Mode 0: nano control
    # Mode 1: Remote control
    # Mode 2: Programming 
    # Debug mode 0: fix route program
    # Debug mode 1: fix route program without LDS
    # Debug mode 2: manual speed, directly assigned delta angle
    # Debug mode 3: manual speed, absolute target angle
        
    debug_mode = 2
    debug_duration_time = 10 
    manual_target_v = 100
    manual_target_angle = 280
    detla_angle_manual_private = 0
    ##
    is_debug = 'inactive'
    start_time = time.time()
    while(1):
        if debug_mode == 2 or debug_mode == 3:
            if True: #time.time() - start_time < debug_duration_time:
                target_v = manual_target_v
                if debug_mode == 2:
                    delta_angle_manual = detla_angle_manual_private
                elif debug_mode == 3:
                    target_angle = manual_target_angle
                is_debug = 'active'
            else:
                target_v = 0
                target_angle = last_angle
                is_debug = 'holding'

        print('read: ', cur_vx, cur_vy, cur_angle, ' || ', 'send: ', target_v, ang_out, ' || ', 'mode: ', mode, ' || ','debug: ', is_debug)
        time.sleep(0.05)
        # v, angle: mm/s and degree, both absolute value, ang_out is [-180, 180]


def fix_route_main():
    ''' 
        This code implement the fix route algorithm, forcing the robot to record the message, memorize it,
        then go along this route. When detecting somebody nearby, stop.
    '''
    # Now comes the recording mode
    t3 = threading.Thread(target = monitor)
    t1 = threading.Thread(target = stm32_communication)
    t2 = threading.Thread(target = route_decision)
    t3.start()
    t1.start()
    t2.start()

    while(1):
        pass

fix_route_main()

    
