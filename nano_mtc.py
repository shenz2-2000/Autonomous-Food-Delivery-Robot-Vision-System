from multiprocessing import Process, Pipe
import time
import lds_driver
from data_rw import data_send, init_data_rw, data_read, data_read_test

def lds_hold(cur_state, ser):
    if cur_state == 0:
        rge = lds_driver.lds_poll(ser)
        if min(rge) < 1:
            return 1
    return 0

def route_decision(lds_pipe, debug_mode):
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

def status_read(dev, read_pipe):
    '''
    Read from STM and send data to other core via read_pip
    '''
    while 1:
        vx, vy, ang, mode, error = data_read(dev)
        data = [vx, vy, ang, mode, error]
        read_pipe.send(data) 
    
def ins_send(dev, lds_pipe, read_pipe):
    '''
    Responsible for getting the info from STM and LDS, form instruction,
    and send instruction to STM
    '''
    while 1:
        is_hold = lds_pipe.recv() #
        data = read_pipe.recv() #You can access the data from stm32 with this
        to_send = [0, 0, 0, 0]
        if is_hold:
            to_send = [1, 1, 1, 1]
        data_send(to_send, dev)
        #To Monitor:
        print(data)
        #
        
            
#def nano_mtc_main():
if __name__ == '__main__':
    # debug_mode 0 will enable LDS
    debug_mode = 1
    
    (lds_in, lds_out) = Pipe()
    (read_in, read_out) = Pipe()
    dev = init_data_rw()
    
    p1 = Process(target = status_read, args = (dev, read_in,))
    p3 = Process(target = route_decision, args = (lds_in, debug_mode,))
    p2 = Process(target = ins_send, args = (dev, lds_out, read_out,)) 
    p1.start()
    p2.start()
    p3.start()
    
