#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2023 Westwood Robotics Corporation"
__date__      = "September 6, 2023"
__version__   = "0.0.3"
__status__    = "Production"

'''
Example of creating shared memory segments
'''

import numpy as np
from Library.SHARED_MEMORY import Manager as shmx

# Time state
TIME_STATE = shmx.SHMEMSEG(robot_name='EXAMPLE', seg_name='TIME_STATE', init=False)
TIME_STATE.add_block(name='time', data=np.zeros(1))

# Joint state
JOINT_STATE = shmx.SHMEMSEG(robot_name='EXAMPLE', seg_name='JOINT_STATE', init=False)
JOINT_STATE.add_block(name='joint_positions',  data=np.zeros(2))
JOINT_STATE.add_block(name='joint_velocities', data=np.zeros(2))


def init():
    """Init if main"""
    TIME_STATE.initialize  = True
    JOINT_STATE.initialize = True


def connect():
    """Connect and create segment"""
    TIME_STATE.connect_segment()
    JOINT_STATE.connect_segment()


if __name__ == '__main__':
    init()
    connect()
else:
    connect()
