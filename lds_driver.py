import serial

def int_to_bytes(x: int) -> bytes:
    return x.to_bytes((x.bit_length() + 7) // 8, 'big')

def lds_poll(ser):
    got_scan = False
    raw_bytes = [0] * 2520
    res = [0] * 360
    # print(raw_bytes)
    start_count, good_sets, motor_speed = 0, 0, 0
    while (~got_scan):
        raw_bytes[start_count] = ser.read(1)
        # print(ser.read(1))
        # if (raw_bytes[start_count] == b'\xfa'):
        #     print("nms")
        if (start_count == 0):
            if (raw_bytes[start_count] == b'\xfa'):
                start_count = 1
                # print("gdl")
        elif (start_count == 1):
            if (raw_bytes[start_count] == b'\xa0'):
                # print("gdl2")
                start_count = 0
                got_scan = True
                data = ser.read(2518)
                for i in range(2518):
                    raw_bytes[2+i] = data[i]
                
                for i in range(0,2520,42):
                    if (raw_bytes[i] == b'\xfa' and raw_bytes[i+1] == (b'\xa0' + int_to_bytes(int(i / 42)))):
                        good_sets+=1
                        motor_speed += (raw_bytes[i+3]<<8) + raw_bytes[i+2]
                        s=(raw_bytes[i+3]<<8|raw_bytes[i+2])/10
                        
                        for j in range(i+4, i+40, 6):
                            index = 6*(i/42) + (j-4-i)/6
                            byte0, byte1, byte2, byte3 = raw_bytes[j], raw_bytes[j+1], raw_bytes[j+2], raw_bytes[j+3]

                            intensity = (byte1 << 8) + byte0
                            rge = (byte3 << 8) + byte2
                            res[359-int(index)] = rge / 1000.0
            else:
                start_count = 0
        print(res)             
    return 
def lds_driver_init():
    # Setting up baud rate and io service
    port = '/dev/ttyUSB0'
    baud_rate = 230400
    ser = serial.Serial(port, baud_rate)

    while (1):
        lds_poll(ser)
    
    return

lds_driver_init()