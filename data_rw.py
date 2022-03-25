import usb.core
import usb.util
import numpy as np

def int_to_bytes(x: int) -> bytes:
    return x.to_bytes((x.bit_length() + 7) // 8, 'big')

def bytes_to_int(bytes):
    result = 0
    for b in bytes:
        result = result * 256 + int(b)
    return result

def init_data_rw():
    id_intf = 0 # use interface 0
    ep_addr = 0x81 # receive endpoint\
    len_msg = 12 
    dev = usb.core.find(idVendor=0x0483, idProduct=0x5740) # find the device  
    if dev.is_kernel_driver_active(id_intf):
        dev.detach_kernel_driver(id_intf)
    return dev

def data_rw(data_send, dev, len_msg = 12):
    #data_send should be [v, alpha, mode, error_status]
    
    #send data to stm32
    # data send is in v>>8, v, alpha/360*8192>>8, alpha/360*8192, mode, error
    bytes_send = []
    bytes_send.append(int_to_bytes((data_send[0])>>8), int_to_bytes(data_send[0])) #send v
    bytes_send.append(int_to_bytes((data_send[1]*8192/360)>>8), int_to_bytes(data_send[1]*8192/360)) #send alpha/360*8192
    bytes_send.append(int_to_bytes(data_send[2]))
    bytes_send.append(data_send[3])
    dev.reset()
    pkg = array.array('B', data_send)
    num_bytes = dev.write(1,pkg)
    
    #read data from stm32
    unit_array = [1, 1, 1, 1, 360/8192, 1, 1]
    read_byte = dev.read(0x81, len_msg, 100)
    data_stack = []
    for i in range(0, 10, 2):
        tem_data = bytes_to_int(read_byte[i] << 8 + read_byte[i+1])
        data_stack.append(tem_data)

    data_stack.append(bytes_to_int(read_byte[10]))
    data_stack.append(bytes_to_int(read_byte[11]))
    
    unit_data_stack = np.multiply(unit_array, data_stack)
    v_fr, v_fl, v_br, v_bl, mode, error_status = unit_data_stack[0], unit_data_stack[1], unit_data_stack[2], unit_data_stack[3], unit_data_stack[4], unit_data_stack[5]
    return v_fr, v_fl, v_br, v_bl, mode, error_status