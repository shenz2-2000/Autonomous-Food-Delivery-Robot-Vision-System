#!/usr/bin/env python

import os
import signal
import time
from subprocess import Popen, PIPE, STDOUT
process = Popen('./delta_2b_lidar_node', stdin=PIPE,stdout=PIPE, universal_newlines = True, shell = True, preexec_fn = os.setsid)
# time.sleep(3)
while True:
        output = process.stdout.readline()
        if output == '' and process.poll is not None:
            break
        if output:
            print(output)