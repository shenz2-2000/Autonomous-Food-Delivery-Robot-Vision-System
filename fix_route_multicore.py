from multiprocessing import Process, Pipe, RLock
import time
import lds_driver
import numpy as np
from data_rw import data_send, init_data_rw, data_read, data_read_test
from multiprocessing import Value, Array

def lds_hold(cur_state, ser):
    if cur_state == 0:
        rge = lds_driver.lds_poll(ser)
        if min(rge) < 1:
            return 1
    return 0


def lds_decision(is_hold, debug_mode):
    '''
    Read from LDS and send hold status to other core
    '''
    if debug_mode == 0:
        ser = lds_driver.lds_driver_init()
        while 1:
            trigger_time = 0
            
            old_time = time.time()
            if lds_hold(ser) == 1:  #If lds is triggered, update trigger time, enable is_hold
                trigger_time = old_time
                print('LDS Hold Triggered')
                is_hold.value = 1
            elif time.time() - trigger_time < 5: #If lds not triggered, but still in 5 second interval, enable is_hold
                print('LDS Hold Triggered')
                is_hold.value = 1
            else:
                is_hold.value = 0
    else:   # If debug_mode is not 0, constantly set is_hold to 0
        while 1:
            is_hold.value = 0

    
def route_decision(dev, debug_mode, is_hold, target_v, target_angle, angle_stack, distance_stack, type_stack, cur_x_pos, cur_y_pos,\
                        cur_angle, cur_vy, mode):
    '''
    Read from STM and send data to other core via read_pip
    '''
    cur_point, ins_spd, ins_ang, tar_ang  = 0, 100, 0, 0
    old_time = None
    cnt = 0
    is_done = 0
    get_ang = 0
    while 1:
        if (mode.value == 0 and (debug_mode == 0 or debug_mode == 1)):
            tar_dis, delta_ang, tar_type = distance_stack[cur_point], angle_stack[cur_point], type_stack[cur_point]
            # print(tar_dis,ins_spd)
            #print("dis:", distance_stack[0:10])
            if tar_type == 1: #go in vy
                if old_time == None:
                    old_time = time.time()
                if abs(cnt) <= abs(tar_dis):
                    #print(cnt, tar_dis)
                    if tar_dis >= 0:
                        ins_spd = 100
                    else: 
                        ins_spd = -100
                    ins_ang = cur_angle.value
                    cur_time = time.time()
                    cnt += cur_vy.value * (cur_time - old_time)
                    old_time = cur_time
                else: 
                    cnt = 0
                    is_done = 1
            elif tar_type == 2:
                if get_ang == 0:
                    now_ang = cur_angle.value
                    get_ang = 1
                tar_ang = delta_ang + now_ang
                if tar_ang > 360:
                    tar_ang -= 360
                elif tar_ang < 0:
                    tar_ang += 360
                if abs(cur_angle.value - tar_ang) > 10:
                    ins_ang = tar_ang
                    ins_spd = 0
                else:
                    is_done = 1
                    get_ang = 0
                    old_time = time.time()
            if is_done:
                print('DONE', cur_point, distance_stack[cur_point], angle_stack[cur_point])
                print("dis:", distance_stack[0:10])
                print("ang:", angle_stack[0:10])
                cur_point += 1
                is_done = 0
            if type_stack[cur_point] == 0:
                cur_point = 0
            # if is_hold:
            #     ins_spd = 0
            #     ins_ang = cur_angle.value
            target_angle.value, target_v.value = ins_ang, ins_spd
            #print(ins_ang, tar_ang, type_stack[cur_point], cur_angle.value)



def stm32_communication(dev, debug_mode, is_hold, target_v, target_angle, angle_stack, distance_stack, type_stack, cur_x_pos, cur_y_pos,\
                        cur_vx, cur_vy, cur_angle, mode, turning_status, record_length):
    '''
    Responsible for getting the info from STM and LDS, form instruction,
    and send instruction to STM

    %para:
        target_v, target_angle: represent the target v and angle we wanna acheive
        x_stack, y_stack: record the track of robot
        record_length: length of total track..
    '''

    # delta_angle_manual = 0

    if debug_mode == 2:
        ############### TO Manual input angle, change this line ################
        delta_angle_manual = 0
        ############### TO Manual input angle, change this line ################

    # Main Work is done here    
    commu_cnt = 0
    last_status = 0
    cnt = 0.0
    last_time = None
    while(1):
        commu_cnt += 1
        if (commu_cnt == 10):
            dev.reset()
            commu_cnt = 0
        # Mode 0: nano control; Mode 1: Remote control; Mode 2: Programming 
        cur_vx.value, cur_vy.value, cur_angle.value, mode.value, turning_status.value = data_read(dev)
        if (mode.value == 2):
            if (turning_status.value != last_status):
                # print("########################")
                # print("dis:", distance_stack[0:10])
                # print("type:", type_stack[0:10])
                # print("ang:", angle_stack[0:10])
                # print("########################")
                if (last_status == 2): # if last status is turning, then record the difference
                    angle_stack[record_length.value] = cur_angle.value - cnt
                elif (last_status == 1):
                    distance_stack[record_length.value] = cnt
                record_length.value += 1
                type_stack[record_length.value] = turning_status.value
                if (turning_status.value == 1):
                    cnt = 0.0
                    last_time = time.time()
                elif (turning_status.value == 2):
                    cnt = cur_angle.value
            else:
                if (turning_status.value == 1): # Going straight

                    cur_time = time.time()
                    cnt += cur_vy.value * (cur_time - last_time)
                    last_time = cur_time

            last_status = turning_status.value
    
        if True: #(mode == 0):           #THIS IS CHANGED FOR TESTING 2022.03.28
            delta_angle =  - cur_angle.value + target_angle.value
            if (abs(delta_angle) < 5):
                delta_angle = 0
            if delta_angle > 180:
                delta_angle = delta_angle - 360
            elif delta_angle < -180:
                delta_angle = delta_angle + 360

            #The plus 180 work is moved to data_rw, here is only [-180,180] indicating the desired degree
            if debug_mode != 2:
                data = [target_v.value, delta_angle, mode.value, 0]
                if (mode.value == 0):
                    #print("data", data)
                    #print("fucking output angle", last_angle, target_angle, delta_angle, mode)
                    data_send(data, dev)
            elif debug_mode == 2:
                data = [target_v.value, delta_angle_manual, mode.value, 0]
                data_send(data, dev)
        
            
#def nano_mtc_main():
if __name__ == '__main__':
    # debug_mode 0 will enable LDS
    # debug_mode 1 will disable LDS
    # debug mode 2 will fix an angle and let the velocity keeps being zero
    debug_mode = 1

    # Put all the shared memory variable here.. Plz keep comments well maintained
    is_hold = Value('i', 0)                 # indicate whether the lds is hold
    cur_vx, cur_vy, cur_angle = Value('d', 0.0), Value('d', 0.0), Value('d', 0.0)
    cur_x_pos, cur_y_pos = Value('d', 0.0), Value('d', 0.0)
    target_v, target_angle = Value('d', 0.0), Value('d', 0.0)
    mode, turning_status, record_length = Value('i', 1), Value('i', 0), Value('i', -1)
    angle_stack, distance_stack, type_stack = Array('d', [0.0]*10000), Array('d', [0.0]*10000), Array('i', [0]*10000)
    dev = init_data_rw()
    
    p1 = Process(target = route_decision, args = (dev, debug_mode, is_hold, target_v, target_angle, angle_stack, distance_stack, type_stack, cur_x_pos, cur_y_pos,\
                                                        cur_angle, cur_vy, mode))

    # This process keep talking to lds and return whether it's hold
    p3 = Process(target = lds_decision, args = (is_hold, debug_mode))
    
    # stm32 communication 
    p2 = Process(target = stm32_communication, args = (dev, debug_mode, is_hold, target_v, target_angle, angle_stack, distance_stack, type_stack, cur_x_pos, cur_y_pos,\
                                                       cur_vx, cur_vy, cur_angle, mode, turning_status, record_length)) 

    p1.start()
    p2.start()
    p3.start()
    
