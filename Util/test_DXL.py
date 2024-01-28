#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2023 Westwood Robotics Corporation"
__date__      = "September 6, 2023"
__version__   = "0.0.3"
__status__    = "Production"

"""
Test communication with Dynamixel motors
"""

import time
from Settings.BRUCE_macros import *
from Library.ACTUATORS.DXL_controller import DynamixelController


if __name__ == '__main__':
    DXL_controller = DynamixelController(port=DXL_port, baudrate=DXL_baudrate)

    DXL_TEST_LIST = [DXL_SHOULDER1_R, DXL_SHOULDER2_R, DXL_ELBOW_R,
                     DXL_SHOULDER1_L, DXL_SHOULDER2_L, DXL_ELBOW_L]
    DXL_controller.setup_sync_read_all(DXL_TEST_LIST)

    while True:
        loop_start_time = time.time()
        pos, vel, error = DXL_controller.sync_read_posvel(DXL_TEST_LIST)
        # print(1. / (time.time() - loop_start_time))
        print(pos)
        time.sleep(0.01)
