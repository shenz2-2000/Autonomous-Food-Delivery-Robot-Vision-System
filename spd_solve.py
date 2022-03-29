import numpy as np

def get_chasis_spd(v_fr, v_fl, v_br, v_bl): # it should actually be bl,br, just written format
    #w_to_v_ratio = (wheel_base + wheel_tread) / 2 / 360 * np.pi
    w_to_v_ratio = (420 + 372) / 2 / 360 * np.pi
    #v_to_wheel = 360 / wheel_circumference
    v_to_wheel = 360 / 478
    
    sum = np.mat([[0], [0], [0]])
    
    V_4 = np.mat([[v_fr],[v_fl],[v_br]])
    V_3 = np.mat([[v_fr],[v_fl],[v_bl]])
    V_2 = np.mat([[v_fr],[v_br],[v_bl]])
    V_1 = np.mat([[v_fl],[v_br],[v_bl]])
    
    V = [V_4, V_3, V_2, V_1]
    
    #A_X are inverse of the 3 by 3 matrix, 
    # which is acquired by removing the X row of the 4 by 3 matrix in formula.c
    
    A_4 = np.mat([[0, 0.5, -0.5],
                  [-0.5, 0.5, 0],
                  [0.5, 0, 0.5]])
    
    A_3 = np.mat([[0.5, 0, -0.5],
                [-0.5, 0.5, 0],
                [0, 0.5, 0.5]])
    
    A_2 = np.mat([[0.5, 0, -0.5],
                [0, 0.5, -0.5],
                [0.5, 0.5, 0]])
    
    A_1 = np.mat([[0.5, -0.5, 0],
                [0, 0.5, -0.5],
                [0.5, 0, 0.5]])
    
    A = [A_4, A_3, A_2, A_1]
    
    num_of_ave = 4
    for i in range(num_of_ave):
        sum = np.dot(A[i], V[i]) / v_to_wheel + sum    
    
    v_x = sum[0] / num_of_ave
    v_y = sum[1] / num_of_ave
    w = sum[2] / num_of_ave /  w_to_v_ratio
    spd = np.sqrt(v_x**2 + v_y**2)
    # print(v_x,v_y,w)
    #return spd
    # This fucking spd should be the absolute speed if work as expected, mm/s
    return spd
    
#get_chasis_spd(0,7.53138075,0,-7.53138075)
    
    
    
                 