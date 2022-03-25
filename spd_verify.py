import numpy as np
import spd_solve

def spd(vx, vy, w):
    w_to_v = (420+372)/2/360*np.pi
    v_to_w = 360/478
    A = np.mat([[1,-1,1],
                [1, 1, 1],
                [-1,1,1],
                [-1,-1,1]])
    V = np.mat([[vx],[vy],[w*w_to_v]])
    Out = v_to_w * np.dot(A, V)
    print(Out)
    return Out

#Out = spd(5, 5, 0)
