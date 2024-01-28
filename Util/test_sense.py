#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2023 Westwood Robotics Corporation"
__date__      = "September 6, 2023"
__version__   = "0.0.3"
__status__    = "Production"

"""
Script for monitoring sense data
"""

import sys
import time
import matplotlib.pyplot as plt
from Settings.BRUCE_macros import *
from Library.BRUCE_SENSE.sense import Manager as sense


if __name__ == '__main__':
    # Sense Setting
    s = sense.SENSOR(port=PICO_port, baudrate=PICO_baudrate)
    s.send_data(mode='run')

    ts = []
    x  = []
    y  = []
    z  = []
    w  = []

    num = 800
    target = 0

    fig, ax = plt.subplots()  # can add multiple subplots here, e.g., fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)
    line1, = ax.plot([], [], animated=True, label='x')
    line2, = ax.plot([], [], animated=True, label='y')
    line3, = ax.plot([], [], animated=True, label='z')
    line4, = ax.plot([], [], animated=True, label='w')

    tmax = 10  # maximum time spam to show [unit in second]
    ax.grid()
    ax.set_xticks(np.arange(0, 100, 1))
    ax.set_xlim([0, tmax])
    ax.set_ylim([-1, 1])
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Data')
    fig.legend()
    fig.show()
    fig.canvas.draw()
    background = fig.canvas.copy_from_bbox(ax.bbox)

    t0 = time.time()
    R  = np.eye(3)
    wp = np.zeros(3)
    ang = np.zeros(3)
    omega = np.zeros(3)
    accel = np.zeros(3)

    data_his = []

    np.set_printoptions(formatter={'float': '{:0.2f}'.format})  # show 2 decimal places

    t0 = time.time()
    while True:
        loop_start_time = time.time()

        # Get sense data
        rtn = s.get_dump_YOLO()

        if rtn is not None:
            imu_data     = rtn[0]
            contact_data = rtn[1]
            err          = rtn[2]

            contact_rt = contact_data >> 1 & 1
            contact_rh = contact_data >> 0 & 1
            contact_lt = contact_data >> 2 & 1
            contact_lh = contact_data >> 3 & 1

            foot_contacts = 1. - np.array([contact_rt, contact_rh, contact_lt, contact_lh])
            accel  = np.array([imu_data[0], imu_data[1], imu_data[2]])
            omega  = np.array([imu_data[3], imu_data[4], imu_data[5]])

        # plot_data = foot_contacts
        plot_data = omega
        data_his.append(plot_data[0])

        sys.stdout.write("\r {}".format(plot_data))
        sys.stdout.flush()

        x.append(plot_data[0])
        y.append(plot_data[1])
        z.append(plot_data[2])

        if np.shape(plot_data)[0] == 4:
            w.append(plot_data[3])

        te = time.time() - t0
        ts.append(te)
        rem = ts[-1] // tmax  # calculate remainder
        if rem > target:      # check if current time is beyond the current time spam shown on the figure
            ax.set_xlim([rem * tmax, (rem + 1) * tmax])
            target += 1
            fig.canvas.draw()

        line1.set_data(ts[-num:], x[-num:])
        line2.set_data(ts[-num:], y[-num:])
        line3.set_data(ts[-num:], z[-num:])

        if np.shape(plot_data)[0] == 4:
            line4.set_data(ts[-num:], w[-num:])

        fig.canvas.restore_region(background)

        ax.draw_artist(line1)
        ax.draw_artist(line2)
        ax.draw_artist(line3)

        if np.shape(plot_data)[0] == 4:
            ax.draw_artist(line4)

        fig.canvas.blit(ax.bbox)
        fig.canvas.flush_events()
