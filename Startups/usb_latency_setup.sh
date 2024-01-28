#!/bin/bash

<< 'COMMENT'
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2023 Westwood Robotics Corporation"
__date__      = "September 6, 2023"
__version__   = "0.0.3"
__status__    = "Production"
COMMENT

sudo sh -c "echo '1' > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer"
sudo sh -c "echo '1' > /sys/bus/usb-serial/devices/ttyUSB1/latency_timer"