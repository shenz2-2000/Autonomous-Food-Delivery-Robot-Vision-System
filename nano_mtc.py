from multiprocessing import Process, Pipe, RLock
import time
import lds_driver
from data_rw import data_send, init_data_rw, data_read, data_read_test

def lds_hold(cur_state, ser):
    if cur_state == 0:
        rge = lds_driver.lds_poll(ser)
        if min(rge) < 1:
            return 1
    return 0

def lds_decision(lds_pipe, debug_mode, lock):
    '''
    Read from LDS and send hold status to other core via lds_pipe
    '''
    if debug_mode == 0:
        ser = lds_driver.lds_driver_init()
        while 1:
            trigger_time = 0
            is_hold = 0
            old_time = time.time()
            if lds_hold(ser) == 1:  #If lds is triggered, update trigger time, enable is_hold
                trigger_time = old_time
                print('LDS Hold Triggered')
                is_hold = 1
            elif time.time() - trigger_time < 5: #If lds not triggered, but still in 5 second interval, enable is_hold
                print('LDS Hold Triggered')
                is_hold = 1
            lds_pipe.send(is_hold)
    else:   # If debug_mode is not 0, constantly set is_hold to 0
        while 1:
            lds_pipe.send(0)

def fix_rt(dev, read_pipe, lock):
    '''
    Read from STM and send data to other core via read_pip
    '''
    vx, vy, ang, mode, error = 0, 0, 0, 0, 0
    while 1:
        t = time.time()
        lock.acquire()
        #print('read running')
        #vx, vy, ang, mode, error = data_read(dev)
        #vx, vy, ang, mode, error = data_read_test(dev, t) #Read Data
        lock.release() 



    
def in_out(dev, lds_pipe, read_pipe, lock):
    '''
    Responsible for getting the info from STM and LDS, form instruction,
    and send instruction to STM
    '''
    while 1:
        is_hold = lds_pipe.recv() #
 #You can access the data from stm32 with this
        to_send = [0, 0, 0, 0]
        if is_hold:
            to_send = [1, 1, 1, 1]

        #lock.acquire()
        #vx, vy, ang, mode, error = data_read(dev)
        vx, vy, ang, mode, error = data_read_test(dev) #Read Data
        data_send(to_send, dev)  #Send Data
        #lock.release() 
        
        data = [vx, vy, ang, mode, error]
        read_pipe.send(data)
        #
        
            
#def nano_mtc_main():
if __name__ == '__main__':
    # debug_mode 0 will enable LDS
    debug_mode = 1

    lock = RLock()
    (lds_in, lds_out) = Pipe()
    (read_in, read_out) = Pipe()
    dev = init_data_rw()
    
    p1 = Process(target = fix_rt, args = (dev, read_out, lock))
    p3 = Process(target = lds_decision, args = (lds_in, debug_mode, lock))
    p2 = Process(target = in_out, args = (dev, lds_out, read_in, lock)) 
    p1.start()
    p2.start()
    p3.start()
    
