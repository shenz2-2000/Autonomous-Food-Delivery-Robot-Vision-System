import usb.core
import usb.util
from spd_solve import get_chasis_spd
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
    #alpha, mode, error_status = 170, data_send[2], data_send[3]

    Vx, Vy = int(V * np.cos(alpha/180*np.pi)+3000), int(np.sin(alpha/180*np.pi)+3000)  #Now all V send will +3000 to ensure the send is positive, even when actual is negative 2022.03.28

    bytes_send = []
    bytes_send.extend([Vx>>8, Vx&0xff]) #send vx
    bytes_send.extend([Vy>>8, Vy&0xff]) #send vy
    bytes_send.extend([ int(alpha*8192/360)>>8, int(alpha*8192/360) & 0xff]) #send alpha/360*8192  #added "int()" to fix a float and int error 2022.03.28
    bytes_send.append(mode)
    bytes_send.append(error_status)

    #pkg = array.array('B', bytes_send)
    num_bytes = dev.write(1,bytes_send)
    print("nano send: ", Vx, Vy, alpha, mode, error_status)
    return 

def data_read(dev, len_msg = 13):
    dev.reset()
    
    
    #read data from stm32
    # data_stack format [V_FR, V_FL, V_BL, V_BR, alpha, mode, error]  #Now all V received will -2500 to have both positive and negative 2022.03.28
    read_byte = dev.read(0x81, len_msg, 100)
    data_stack = []
    for i in range(1, 11, 2):
        tem_data = (int(read_byte[i] << 8) + int(read_byte[i+1]))
        data_stack.append(tem_data)

    data_stack.append((read_byte[11]))
    data_stack.append((read_byte[12]))
    data_stack[4] = data_stack[4]*360/8192
    #print("nano_read: ", data_stack)
    return get_chasis_spd(data_stack[0]-2500, data_stack[1]-2500, data_stack[2]-2500, data_stack[3]-2500), data_stack[4], data_stack[5], data_stack[6]