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
    ep_addr = 0x81 # receive endpoint
    dev = usb.core.find(idVendor=0x0483, idProduct=0x5740) # find the device  
    if dev.is_kernel_driver_active(id_intf):
        dev.detach_kernel_driver(id_intf)
    return dev

def data_send(data_send, dev):
    #data_send should be [v, alpha, mode, error_status]
    
    #send data to stm32
    # data send is in v>>8, v, alpha/360*8192>>8, alpha/360*8192, mode, error
    dev.reset()
    import numpy as np
    V = data_send[0]
    alpha, mode, error_status = data_send[1], data_send[2], data_send[3]

    Vx, Vy = V * np.cos(alpha/180*np.pi), np.sin(alpha/180*np.pi)

    bytes_send = []
    bytes_send.extend([Vx>>8, Vx&0xff]) #send vx
    bytes_send.extend([Vy>>8, Vy&0xff]) #send vy
    bytes_send.extend([alpha*8192/360>>8, alpha*8192/360&0xff]) #send alpha/360*8192
    bytes_send.append(mode)
    bytes_send.append(error)

    return 

def data_read(dev, len_msg = 13)
    dev.reset()
    #pkg = array.array('B', bytes_send)
    num_bytes = dev.write(1,bytes_send)
    
    #read data from stm32
    # data_stack format [V_FR, V_FL, V_BL, V_BR, alpha, mode, error]
    read_byte = dev.read(0x81, len_msg, 100)
    data_stack = []
    for i in range(1, 11, 2):
        tem_data = (read_byte[i] << 8 + read_byte[i+1])
        data_stack.append(tem_data)

    data_stack.append((read_byte[11]))
    data_stack.append((read_byte[12]))
    data_stack[4] = data_stack[4]*360/8192
    
    return data_stack[0], data_stack[1], data_stack[2], data_stack[3], data_stack[4], data_stack[5]