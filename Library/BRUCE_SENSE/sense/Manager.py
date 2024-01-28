#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2023 Westwood Robotics Corporation"
__date__      = "September 6, 2023"
__version__   = "0.0.3"
__status__    = "Production"

import serial
from Library.BRUCE_SENSE.sense import Packet


class SENSOR(Packet.PKT):
    def __init__(self,
                 port='COM8',
                 baudrate='115200',
                 bytesize=serial.EIGHTBITS,
                 parity=serial.PARITY_NONE,
                 stopbits=serial.STOPBITS_ONE,
                 timeout=0.0):
        """
        Provide reading IMU and contact info/sending motor temperature info from/to Pico
        :param port: Port address
        :param baudrate: Specified baudrate
        """

        self.port      = port
        self.baudrate  = baudrate
        self.bytesize  = bytesize
        self.parity    = parity
        self.stopbits  = stopbits
        self.timeout   = timeout

        super(SENSOR, self).__init__(self.port, self.baudrate)

    def get_dump(self):
        """
        Get all IMU data from dumper
        """
        return self._read_dump()

    def get_dump_YOLO(self):
        """
        Get all IMU data from dumper but only try to read once
        :return None if timeout
        """
        return self._read_dump_YOLO()

    def send_data(self, temperature=20.0, mode='run'):
        """
        Send motor temperature info and command to Pico
        :param temperature: motor temperature
        :param mode: 0 to  idle Pico
                     1 to   run Pico
                     2 to reset Pico
        """
        if mode == 'idle':
            mode = 0.0
        elif mode == 'reset':
            mode = 2.0
        else:
            mode = 1.0
        self._send_data(temperature, mode)
