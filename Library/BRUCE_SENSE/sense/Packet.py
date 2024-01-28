#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2023 Westwood Robotics Corporation"
__date__      = "September 6, 2023"
__version__   = "0.0.3"
__status__    = "Production"

import sys
import time
import serial
import struct


class PKT(object):
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port,
                                 baudrate,
                                 bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 timeout=0.0  # default to 0 for non-blocking mode
                                 )

        if sys.version_info[0] < 3:  # Python 2
            self.ord_adapt = lambda x: ord(x)
            self.sustr_adapt = lambda val: ''.join(chr(idx) for idx in val)
            self.sustr_loop_adapt = lambda idx, val: ''.join(chr(idx) for idx in val[idx:idx+4])
            pass
        else:  # Python 3
            self.ord_adapt = lambda x: x
            self.sustr_adapt = lambda val: bytearray(val)
            self.sustr_loop_adapt = lambda idx, val: bytearray(val[idx:idx+4])

        self.TIMEOUT_MAX = 0.0005  # in seconds

    def close(self):
        """
        Close the serial port
        """
        if self.ser:
            self.ser.close()

    def _read_dump(self):
        """
        This command is to read data from dumper
        """
        # Timeout prevention if communication error starts occurring
        incoming_bytes = [0, 0]
        got_packet = False
        t_bus_init = time.time()
        self.ser.reset_input_buffer()
        while True:
            if self.ser.inWaiting() > 1:
                # Check for start of packet
                incoming_bytes.extend(self.ser.read(1))
                incoming_bytes.pop(0)
                if ((incoming_bytes[1] << 8) | incoming_bytes[0]) == 0xFFFF:
                    # This is the beginning of the packet
                    got_packet = True
                    incoming_bytes.extend(self.ser.read(1))
                    break
            if time.time() - t_bus_init > self.TIMEOUT_MAX:
                print("[BRUCE_SENSE | WARNING] :: Status response timed out. Is dumper alright?")
                t_bus_init = time.time()
        if got_packet:
            t_bus_init = time.time()
            pktlen = incoming_bytes[2]
            while self.ser.inWaiting() < pktlen:
                if time.time() - t_bus_init > self.TIMEOUT_MAX:
                    print("[BRUCE_SENSE | WARNING] :: Status response timed out waiting for rest of packet")
            status_packet = self.ser.read(pktlen)
        # Temporary absolute error watch:
        if len(status_packet) < pktlen:
            print("ser.read() returned too soon, status_packet length is only", len(status_packet))
            error_code = 0b00000001
        elif status_packet[-1] != 0x0F:
            print("Corrupted packet.")
            error_code = 0b00000010
        else:
            error_code = status_packet[0]
            contact = status_packet[-2]
        status_packet = [self.ord_adapt(idx) for idx in status_packet[1:-2]]
        return self.__hex_to_float32(status_packet), contact, error_code

    def _read_dump_YOLO(self):
        """
        This command is to read data from dumper, but it only read once and return None if no data in buffer
        """
        # Timeout prevention if communication error starts occurring
        incoming_bytes = [0, 0]
        got_packet = False
        t_bus_init = time.time()
        while True:
            if self.ser.inWaiting() > 1:
                # Check for start of packet
                incoming_bytes.extend(self.ser.read(1))
                incoming_bytes.pop(0)
                if ((incoming_bytes[1] << 8) | incoming_bytes[0]) == 0xFFFF:
                    # This is the beginning of the packet
                    got_packet = True
                    incoming_bytes.extend(self.ser.read(1))
                    break
            if time.time() - t_bus_init > self.TIMEOUT_MAX:
                # Timeout with no data
                return None
        if got_packet:
            t_bus_init = time.time()
            pktlen = incoming_bytes[2]
            while self.ser.inWaiting() < pktlen:
                if time.time() - t_bus_init > self.TIMEOUT_MAX:
                    print("[BRUCE_SENSE | WARNING] :: Status response timed out waiting for rest of packet")
            status_packet = self.ser.read(pktlen)
            self.ser.reset_input_buffer()
        # Temporary absolute error watch:
        if len(status_packet) < pktlen:
            print("ser.read() returned too soon, status_packet length is only", len(status_packet))
            error_code = 0b00000001
        elif status_packet[-1] != 0x0F:
            print("Corrupted packet.")
            error_code = 0b00000010
        else:
            error_code = status_packet[0]
            contact = status_packet[-2]
        status_packet = [self.ord_adapt(idx) for idx in status_packet[1:-2]]
        return self.__hex_to_float32(status_packet), contact, error_code

    def _send_data(self, temperature, mode):
        data1 = struct.pack('f', temperature)
        data2 = struct.pack('f', mode)
        self.ser.write([0xFF, data1[2], data1[3], data2[2], data2[3], 0xFE])

    # Utility function
    def __hex_to_float32(self, val):
        if len(val) > 4:
            tmpval = []
            for idx in range(0, len(val), 4):
                tmpval.append(struct.unpack('<f', self.sustr_loop_adapt(idx, val))[0])
        else:
            tmpval = struct.unpack('<f', self.sustr_adapt(val))[0]
        return tmpval
