#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2023 Westwood Robotics Corporation"
__date__      = "September 6, 2023"
__version__   = "0.0.3"
__status__    = "Production"

'''
Set data to shared memory
'''

import time
import numpy as np
import Examples.shared_memory.memory_manager as mm

if __name__ == '__main__':
    t0 = time.time()
    while True:
        data = {'time': np.array([time.time() - t0])}
        mm.TIME_STATE.set(data)
        print("Time set: {:.6f} [sec]".format(data['time'][0]), end='\r')
