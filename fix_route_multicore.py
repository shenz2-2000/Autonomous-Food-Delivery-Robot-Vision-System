from multiprocessing import Process, Pipe, RLock
import time
# import lds_driver
import numpy as np
# import cv2
# import face_recognition
import jetson.inference
import jetson.utils
from data_rw import data_send, init_data_rw, data_read
from multiprocessing import Value, Array
from heapq import nsmallest

VALID_HEIGHT = 140
VALID_WIDTH = 140

TOO_CLOSE = np.array([0.6, 0.5, 0.5, 0.5, 0.6])
A_BIT_CLOSE = np.array([1.3, 1.2, 1.15, 1.2, 1.3])
# TOO_CLOSE = 0.5  # 0.6
# A_BIT_CLOSE = 1.1  # the range you should care
COMING = 0.2  # the relative speed defined as approaching
REST_TIME = 2  # the time to rest after successful detecting


def face_detector(lds_dists, sec_id, detected):
    time.sleep(1)
    last_too_close_flag = 0
    last_turned_time = 0
    net = jetson.inference.detectNet("facenet", threshold=0.15)
    camera = jetson.utils.videoSource("/dev/video0",['--input-codec=mjpeg'])      # '/dev/video0' for V4L2
    # display = jetson.utils.videoOutput()
    while True:
        # Grab a single frame of video
        # print(np.array(lds_dists))
        diff = np.array(lds_dists) - TOO_CLOSE
        flag_too_close = np.min(diff) < 0
        diff = np.array(lds_dists) - A_BIT_CLOSE
        flag_bit_close = np.min(diff) < 0
        if not flag_bit_close:
            # print("clear")
            detected.value = 0
            continue
        tar_sec = np.argmin(diff)
        if flag_too_close or (tar_sec != sec_id and not detected.value):
            turn_time = time.time()
            if turn_time-last_turned_time > 0.5:
                sec_id.value = tar_sec
                last_turned_time = time.time()

        camera_detected = 0
        img = camera.Capture()
        detections = net.Detect(img)

        # for detection in detections:
        #     print(detection)
        cnt = 0
        for detection in detections:
            if detection.Height<VALID_HEIGHT and detection.Width<VALID_WIDTH:
                continue
            else:
                cnt += 1
        if cnt > 0:
            print('detected by the camera', sec_id.value)
            camera_detected = 1
            # works but currently not used
            # sizes = []
            # for (top, right, bottom, left) in face_locations:
            #     sizes.append( (bottom-top) * (right-left) )
            # min_size = min(sizes)
            # if min_size < last_min_size*0.95:
            #     print('approaching')
            # last_min_size = min_size
        # process_this = not process_this

        if flag_too_close:
            print("TOO close; the detected position", tar_sec, lds_dists[tar_sec])

        diff = np.array(lds_dists) - TOO_CLOSE
        flag_too_close = np.min(diff) < 0
        if camera_detected or (flag_too_close and last_too_close_flag):
            if detected.value == 0: detected.value = 1
        else:
            if detected.value == 1: detected.value = 0

        last_too_close_flag = flag_too_close


# TODO: the problem is that, for a previously spotted sector, when it was not spotted once, the camera might go away rather than wait
# def face_detector(lds_dists, sec_id, detected):
#     """
#     sec_id: the section that the camera is facing
#     """
#     time.sleep(1)
#     video_capture = cv2.VideoCapture(0)
#     print('streaming now')

#     last_turned_time = 0

#     face_locations = []
#     process_this = True

#     # last_min_size = 0

    # while True:
    #     # Grab a single frame of video
    #     if min(lds_dists) > A_BIT_CLOSE:
    #         # print("clear")
    #         detected.value = 0
    #         continue
    #     print("The detected position", sec_id.value)
    #     tar_sec = np.argmin(np.array(lds_dists))
    #     if (min(lds_dists) < TOO_CLOSE) or (tar_sec != sec_id and not detected.value):
    #         turn_time = time.time()
    #         if turn_time-last_turned_time > 1:
    #             sec_id.value = tar_sec
    #             last_turned_time = time.time()

    #     camera_detected = 0
    #     if process_this:
    #         ret, frame = video_capture.read()

    #         # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
    #         rgb_small_frame = frame[:, :, ::-1]
    #         rgb_small_frame = cv2.resize(rgb_small_frame, (0, 0), fx=0.5, fy=0.5)
    #         face_locations = face_recognition.face_locations(rgb_small_frame)

    #         if len(face_locations) > 0:
    #             print('detected by the camera')
    #             camera_detected = 1
    #             # works but currently not used
    #             # sizes = []
    #             # for (top, right, bottom, left) in face_locations:
    #             #     sizes.append( (bottom-top) * (right-left) )
    #             # min_size = min(sizes)
    #             # if min_size < last_min_size*0.95:
    #             #     print('approaching')
    #             # last_min_size = min_size
    #     # process_this = not process_this

    #     if min(lds_dists)<TOO_CLOSE:
    #         print("TOO CLOSE")

    #     if camera_detected or min(lds_dists)<TOO_CLOSE:
    #         if detected.value == 0: detected.value = 1
    #     else:
    #         if detected.value == 1: detected.value = 0

    # video_capture.release()
    # cv2.destroyAllWindows()

# def new_lds_decision(debug_mode, detected_angle, mn_distance, detected):
# def new_lds_decision(debug_mode, lds_dists, lds_speeds):
#     if debug_mode == 0:
#         time.sleep(3)
#         print("LDS Start")
#         while True:
#
#             try:
#                 a = input()
#             except Exception as e:
#                 print(e)
#                 a = "error"
#             if (a[0]=="d" and a[1]=="i"):
#                 target = a.split("_")
#                 distance, angle = float(target[1]), int(target[3])
#                 if (not distance == 2):
#                     # detected.value = 1
#                     for i in range(4):
#                         lds_dists[i] = 100
#                     if (angle>315 and angle<45):
#                         lds_dists[3] = distance
#                     elif (angle>45 and angle<135):
#                         lds_dists[0] = distance
#                     elif (angle>135 and angle<225):
#                         lds_dists[1] = distance
#                     else:
#                         lds_dists[2] = distance
#     else:
#         time.sleep(0.5)


# def lds_decision(debug_mode, lds_dists, lds_speeds):
#     '''
#     Read from LDS and send hold status to other core
#     '''
#     if debug_mode == 0:
#         ser = lds_driver.lds_driver_init()
#         last_dists = np.array([-1,-1,-1,-1])
#         last_speed = np.array([0,0,0,0])
#         last_time = time.time()
#         trigger_time = 0
#         while 1:
#             # print(lds_hold(ser))
#             rge = lds_driver.lds_poll(ser)
#             poll_time = time.time()
#             sectors = [rge[315:] + rge[:45],  # front
#                        rge[45:135],  # right
#                        rge[135:225],  # back
#                        rge[225:315]]  # left
#             for id_sec in range(4):
#                 sec = sectors[id_sec]
#                 sec = np.array(sec)
#                 sec = sec[sec > 0.01]
#                 if len(sec > 3):
#                     mins = nsmallest(3, sec)
#                     # lds_dists[id_sec] = 0.5 * (mins[0] + mins[1]) if mins[0] < 0.4 else mins[0]
#                     if mins[0]<0.25:
#                         lds_dists[id_sec] = 1/3*(mins[0] + mins[1]+mins[2])
#                     else:
#                         lds_dists[id_sec] = 0.5 * (mins[0] + mins[1]) if mins[0] < 0.5 else mins[0]
#                 # print(lds_dists[0], lds_dists[1], lds_dists[2], lds_dists[3])
#             if last_dists[0] != -1:
#                 speed = (np.array(lds_dists)-np.array(last_dists))/(poll_time-last_time)
#                 filtered = (speed + last_speed) * 0.5
#                 # speed like 2.14528556 1.58223644 2.49413467 3.34359059... and sec3 sometimes goes under 0
#                 for i in range(4):
#                     lds_speeds[i] = filtered[i]
#                 last_speed = speed
#             for i in range(4):
#                 last_dists[i] = lds_dists[i]
#             last_time = poll_time
#
#
#     else:   # If debug_mode is not 0, constantly set is_hold to 0
#         time.sleep(0.5)


def route_decision(debug_mode, target_v, target_angle, angle_stack, distance_stack, type_stack, cur_x_pos, cur_y_pos,\
                        cur_angle, cur_vy, mode, record_length, sec_id, detected):
    '''
    Read from STM and send data to other core via read_pip
    '''
    cur_point, ins_spd, ins_ang, tar_ang = 0, 100, 0, 0
    old_time = None
    cnt = 0
    is_done = 0
    get_ang = 0

    customer = -1
    customer_time = -1
    while 1:
        if (mode.value == 0 and (debug_mode == 0 or debug_mode == 1)):
            # print(detected.value, sec_id.value)
            if customer_time<time.time():
                customer = -1
                customer_time = -1
            if customer<0 and detected.value==1:
                customer = sec_id.value
                customer_time = time.time() + REST_TIME
            if (customer==sec_id.value) and detected.value==1:
                if customer_time-REST_TIME < time.time(): customer_time=time.time() + REST_TIME

            tar_dis, delta_ang, tar_type = distance_stack[cur_point], angle_stack[cur_point], type_stack[cur_point]
            if (type_stack[cur_point] == 0):
                cur_point += 1
            # print(tar_dis,ins_spd)
            #print("dis:", distance_stack[0:10])
            # print(tar_type)
            if tar_type == 1: #go in vy
                if old_time == None:
                    old_time = time.time()
                if abs(cnt-tar_dis) > 10:
                    # print(cnt, tar_dis)
                    if tar_dis >= 0:
                        ins_spd = 200
                    else:
                        ins_spd = -200
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
                if abs(cur_angle.value - tar_ang) > 1:
                    ins_ang = tar_ang
                    ins_spd = 0
                else:
                    is_done = 1
                    get_ang = 0
                    old_time = time.time()
            if is_done:
                #print('DONE', cur_point, distance_stack[cur_point], angle_stack[cur_point])
                #print("dis:", distance_stack[0:10])
                #print("ang:", angle_stack[0:10])
                cur_point += 1
                # print("Done", cur_point, record_length.value)
                is_done = 0
            if cur_point == record_length.value:
                cur_point = 0
            if customer>=0:
                ins_spd = 0
                ins_ang = cur_angle.value
            # print("target angle and v:", ins_ang, ins_spd)
            target_angle.value, target_v.value = ins_ang, ins_spd
            # print(ins_ang, tar_ang, type_stack[cur_point], cur_angle.value)


def stm32_communication(debug_mode, target_v, target_angle, angle_stack, distance_stack, type_stack, cur_x_pos, cur_y_pos,\
                        cur_vx, cur_vy, cur_angle, mode, turning_status, record_length, sec_id):
    '''
    Responsible for getting the info from STM and LDS, form instruction,
    and send instruction to STM
    %para:
        target_v, target_angle: represent the target v and angle we wanna acheive
        x_stack, y_stack: record the track of robot
        record_length: length of total track..
    '''

    # delta_angle_manual = 0
    dev = init_data_rw()
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
        # commu_cnt += 1
        # if (commu_cnt == 2):
        #     dev.reset()
        #     commu_cnt = 0
        # Mode 0: nano control; Mode 1: Remote control; Mode 2: Programming
        _vx, _vy, _angle, _mode, _ts = data_read(dev)
        if _vx is not None:
            cur_vx.value, cur_vy.value, cur_angle.value, mode.value, turning_status.value = _vx, _vy, _angle, _mode, _ts
        else:
            print("Update fail")
        # print("turning status:", turning_status.value)
        # print(mode.value, turning_status.value, last_status)
        if (mode.value == 2):
            if (turning_status.value != last_status):

                # print("########################")
                print("dis:", distance_stack[0:10])
                print("type:", type_stack[0:10])
                print("ang:", angle_stack[0:10])
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
            if (mode.value == 1 or mode.value == 2):
                data = [0, 0, mode.value, 0] # sec_id.value
                # print("sending", data)
                data_send(data, dev)
            else:
                delta_angle =  - cur_angle.value + target_angle.value

                if (abs(delta_angle) < 1):
                    delta_angle = 0
                # print("TEST!", delta_angle, cur_angle.value, target_angle.value)
                if delta_angle > 180:
                    delta_angle = delta_angle - 360
                elif delta_angle < -180:
                    delta_angle = delta_angle + 360

                #The plus 180 work is moved to data_rw, here is only [-180,180] indicating the desired degree
                if debug_mode != 2:
                    data = [target_v.value, delta_angle, mode.value, sec_id.value]
                    if (mode.value == 0):
                        # print("data", data)
                        data_send(data, dev)
                elif debug_mode == 2:
                    data = [target_v.value, delta_angle_manual, mode.value, 0]
                    data_send(data, dev)


#def nano_mtc_main():
if __name__ == '__main__':
    # debug_mode 0 will enable LDS
    # debug_mode 1 will disable LDS
    # debug mode 2 will fix an angle and let the velocity keeps being zero
    # print("Finish Importing")
    debug_mode = 0

    # Put all the shared memory variable here.. Plz keep comments well maintained
    lds_dists = Array('d', [0.0]*5)                # indicate min distance at each sector
    lds_speeds = Array('d', [0.0]*4)

    detected = Value('i', 0) # whether a human is detected as the current sector
    sec_id = Value('i', 0) # current target sector, 0-front, 1-right, 3-left

    detected_angle, mn_distance = Value('d', 0), Value('d', 0)

    cur_vx, cur_vy, cur_angle = Value('d', 0.0), Value('d', 0.0), Value('d', 0.0)
    cur_x_pos, cur_y_pos = Value('d', 0.0), Value('d', 0.0)
    target_v, target_angle = Value('d', 0.0), Value('d', 0.0)
    mode, turning_status, record_length = Value('i', 1), Value('i', 0), Value('i', -1)
    angle_stack, distance_stack, type_stack = Array('d', [0.0]*10000), Array('d', [0.0]*10000), Array('i', [0]*10000)

    p1 = Process(target = route_decision, args = (debug_mode, target_v, target_angle, angle_stack, distance_stack, type_stack, cur_x_pos, cur_y_pos,\
                                                        cur_angle, cur_vy, mode, record_length, sec_id, detected))
    # This process keep talking to lds and return whether it's hold
    # p3 = Process(target = new_lds_decision, args = (debug_mode, lds_dists, lds_speeds))

    # stm32 communication
    p2 = Process(target = stm32_communication, args = (debug_mode, target_v, target_angle, angle_stack, distance_stack, type_stack, cur_x_pos, cur_y_pos,\
                                                       cur_vx, cur_vy, cur_angle, mode, turning_status, record_length, sec_id))
    p4 = Process(target=face_detector, args=(lds_dists, sec_id, detected))

    p1.start()
    p2.start()
    # p3.start()
    p4.start()

    if debug_mode == 0:
        time.sleep(2)
        print("LDS Start")
        while True:
            try:
                a = input()
            except Exception as e:
                print(e)
                a = "error"
            if (a[0] == "s" and a[1] == "e" and a[2] == "c"):
                target = a.split("_")
                # print(target)
                lds_dists[0], lds_dists[1], lds_dists[2], lds_dists[3], lds_dists[4] = \
                    float(target[1]), float(target[3]), float(target[5]), float(target[7]), float(target[9])

                # distance, angle = float(target[1]), int(target[3])

                # if (angle > 315 and angle < 45):
                #     match_sector = 3 # left
                # elif (angle > 45 and angle < 135):
                #     match_sector = 0 # front
                # elif (angle > 135 and angle < 225):
                #     match_sector = 1 # right
                # else:
                #     match_sector = 2 # back
                # for i in range(4):
                #     if i==match_sector:
                #         lds_dists[i] = distance
                #     else:
                #         lds_dists[i] = 100
                # print(distance, angle, match_sector)
    else:
        time.sleep(0.5)
