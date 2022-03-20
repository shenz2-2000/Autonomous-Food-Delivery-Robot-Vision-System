import numpy as np
from matplotlib import pyplot as plt

def input_recording(x_stack, y_stack, spd_stack, spd, ang, t_interval = 0.01)
    if (x_stack == []):
        # Handle initial condition
        x_stack.append(0.0)
        y_stack.append(0.0)
        spd_stack.append(0.0)

    x_new = x_stack[-1] + spd * t_interval * np.cos(ang)
    y_new = y_stack[-1] + spd * t_interval * np.sin(ang)
    x_stack.append(x_new)
    y_stack.append(y_new)
    spd_stack.append(spd)

    return

