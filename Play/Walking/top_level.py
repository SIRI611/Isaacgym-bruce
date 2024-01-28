#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2023 Westwood Robotics Corporation"
__date__      = "September 6, 2023"
__version__   = "0.0.3"
__status__    = "Production"

"""
Keyboard User Input
"""

import os
import select
import random
import subprocess
import tty, sys, termios
import Startups.memory_manager as MM
from termcolor import colored
from Play.initialize import *


def float2str(f, t='default'):
    if t == 'len':
        f = round(f * 1000.) / 10.
    elif t == 'ang':
        f = round(f * 180. / np.pi * 100.) / 100.
    else:
        f = round(f * 100.) / 100.

    if f >= 10.:
        s = '+' + str(f)
    elif 0. < f < 10.:
        s = '+' + str(f) + '0'
    elif f == 0.:
        s = ' ' + str(f) + '0'
    elif -10. < f < 0.:
        s = str(f) + '0'
    else:
        s = str(f)

    return s


def main_loop():
    # BRUCE SETUP
    Bruce = RDS.BRUCE()

    loop_freq         = 100  # run at 100 Hz
    display_freq      = 10   # display monitor at 10 Hz
    loop_duration     = 1. / loop_freq
    display_duration  = 1. / display_freq
    display_last_time = time.time()

    input_data = {'walk': np.zeros(1),
                  'com_xy_velocity': np.zeros(2),
                  'yaw_rate': np.zeros(1)}

    walk    = False  # walking command
    recover = False  # go back to original configuration before walking
                     # go back to          zero velocity before balancing
    init    = True   # initialize command
    squat   = False  # squat mode
    wave    = False  # wave hand

    tol = 1e-3       # tolerance for floating zero

    # SQUAT
    T = 3.0   # period   [s]
    A = 0.05  # distance [m]
    N = 2     # count

    # WALKING
    # overall speed
    vx = 0.
    vy = 0.
    yaw_rate = 0.

    dvx = 0.01
    dvy = 0.01
    dyaw_rate = 1. / 180. * np.pi

    vx_max  = 0.1                 # [m/s]
    vy_max  = 0.1                 # [m/s]
    yaw_max = 20. / 180. * np.pi  # [rad/s]

    # foot yaw [rad]
    yaw_r     = 0.
    yaw_l     = 0.
    dyaw_f    = 1.  / 180. * np.pi
    yaw_f_max = 10. / 180. * np.pi

    # foot clearance [m]
    dy     = 0.10
    ddy    = 0.01
    dy_max = 0.14
    dy_min = 0.06

    # BALANCING
    # moving CoM
    px_scale = 0.
    py_scale = 0.
    pz       = 0.

    px_max =  0.30
    py_max =  0.60
    pz_max =  0.04
    pz_min = -0.04

    dpx = 0.025
    dpy = 0.025
    dpz = 0.002

    # BOTH
    # body orientation
    rx = 0.
    ry = 0.
    rz = 0.

    rx_max = 30. / 180. * np.pi  # [rad]
    ry_max = 30. / 180. * np.pi  # [rad]
    rz_max = 30. / 180. * np.pi  # [rad]

    drx = 1. / 180. * np.pi      # [rad]
    dry = 1. / 180. * np.pi      # [rad]
    drz = 1. / 180. * np.pi      # [rad]

    # detect if screen is used
    screen_flag = True if 'bruce' in subprocess.check_output(["screen -ls; true"], shell=True).decode("utf-8") else False

    filedes = termios.tcgetattr(sys.stdin)
    t0 = time.time()
    try:
        tty.setcbreak(sys.stdin)

        print('Press any key to start!')
        cmd = sys.stdin.read(1)[0]

        while True:
            loop_start_time = time.time()

            # check threading error
            if Bruce.thread_error():
                Bruce.stop_threading()

            if init:
                # initialize
                init = not init
            elif wave:
                hand = random.random() > 0.5
                p0 = 1.2 if hand else -1.2
                arm_positions = np.array([-0.7, 1.3, 2.0, 0.0, p0, 0.0]) if hand else np.array([0.0, p0, 0.0, 0.7, -1.3, -2.0])
                SHOULDER_ROLL = SHOULDER_ROLL_L if hand else SHOULDER_ROLL_R
                set_joint_positions(Bruce, 100, 0.01, arm_move=True, arm_goal_positions=arm_positions)
                traj_time = np.linspace(0, 2.75 * 2 * np.pi, 150)
                for tdx in traj_time:
                    Bruce.joint[SHOULDER_ROLL]['q_goal'] = p0 + 0.3 * np.sign(p0) * np.sin(tdx)
                    Bruce.set_command_arm_positions()
                    Bruce.sleep(0.01)
                Bruce.sleep(0.2)
                set_joint_positions(Bruce, 100, 0.01, arm_move=True, arm_goal_positions=np.array([-0.7, 1.3, 2.0, 0.7, -1.3, -2.0]))
                wave = False
            elif squat:
                te = loop_start_time - t0
                pz = pz_squat + (np.cos(2. * np.pi / T * te) - 1.) * A / 2.
                if te >= N * T:
                    squat = False
                    pz = pz_squat
                    termios.tcflush(sys.stdin, termios.TCIOFLUSH)  # clear buffer
            elif recover:
                # set back to default
                if walk:
                    if vx > tol:
                        vx -= dvx
                    elif vx < -tol:
                        vx += dvx

                    if vy > tol:
                        vy -= dvy
                    elif vy < -tol:
                        vy += dvy

                    if yaw_rate > tol:
                        yaw_rate -= dyaw_rate
                    elif yaw_rate < -tol:
                        yaw_rate += dyaw_rate

                    if pz > tol:
                        pz -= dpz
                    elif pz < -tol:
                        pz += dpz

                    if rx > tol:
                        rx -= drx
                    elif rx < -tol:
                        rx += drx

                    if ry > tol:
                        ry -= dry
                    elif ry < -tol:
                        ry += dry

                    if rz > tol:
                        rz -= drz
                    elif rz < -tol:
                        rz += drz

                    if np.abs(vx) < tol and np.abs(vy) < tol and np.abs(yaw_rate) < tol and np.abs(pz) < tol and np.abs(rx) < tol and np.abs(ry) < tol and np.abs(rz) < tol:
                        recover = False
                        termios.tcflush(sys.stdin, termios.TCIOFLUSH)   # clear buffer
                else:
                    if px_scale > tol:
                        px_scale -= dpx
                    elif px_scale < -tol:
                        px_scale += dpx

                    if py_scale > tol:
                        py_scale -= dpy
                    elif py_scale < -tol:
                        py_scale += dpy

                    if pz > tol:
                        pz -= dpz
                    elif pz < -tol:
                        pz += dpz

                    if rx > tol:
                        rx -= drx
                    elif rx < -tol:
                        rx += drx

                    if ry > tol:
                        ry -= dry
                    elif ry < -tol:
                        ry += dry

                    if rz > tol:
                        rz -= drz
                    elif rz < -tol:
                        rz += drz

                    if np.abs(px_scale) < tol and np.abs(py_scale) < tol and np.abs(pz) < tol and np.abs(rx) < tol and np.abs(ry) < tol and np.abs(rz) < tol:
                        recover = False
                        termios.tcflush(sys.stdin, termios.TCIOFLUSH)  # clear buffer

                time.sleep(0.2)
            else:
                # read keyboard
                is_input, _, _ = select.select([sys.stdin], [], [], loop_duration / 2.)  # adding timeout to keyboard input
                cmd = sys.stdin.read(1)[0] if is_input else None

                # r - set back to default
                if cmd == 'r':
                    recover = True

                # 1 - squat mode
                if cmd == '1':
                    t0 = time.time()
                    squat = True
                    pz_squat = pz

                # 2 - wave hand mode
                if cmd == '2':
                    wave = True

                # space bar - walking or balancing (need to press twice)
                if cmd == ' ':
                    cmd = sys.stdin.read(1)[0]

                    if cmd == ' ':
                        if walk and np.abs(vx) > tol or np.abs(vy) > tol or np.abs(yaw_rate) > tol:
                            print(colored('Nonzero Velocity Command!!!', 'red'))
                            print(colored('Press R to Zero Velocity!!!', 'red'))
                            time.sleep(1.)
                        elif not walk and np.abs(px_scale) > tol or np.abs(py_scale) > tol or np.abs(pz) > tol \
                                or np.abs(rx) > tol or np.abs(ry) > tol or np.abs(rz) > tol:
                            print(colored('Not Original Configuration!!!', 'red'))
                            print(colored('Press R to Original Configuration!!!', 'red'))
                            time.sleep(1.)
                        else:
                            walk = not walk

                            # reset if restart walking
                            if walk:
                                vx = 0.
                                vy = 0.
                                yaw_rate = 0.
                            if not walk:
                                px_scale = 0.
                                py_scale = 0.
                                pz       = 0.
                                rx       = 0.
                                ry       = 0.
                                rz       = 0.

                # body orientation
                # RX
                # y - x positive rotation
                if cmd == 'y':
                    if rx_max - rx > tol:
                        rx += drx

                # t - x negative rotation
                if cmd == 't':
                    if rx_max + rx > tol:
                        rx -= drx

                # RY
                # i - y positive rotation
                if cmd == 'i':
                    if ry_max - ry > tol:
                        ry += dry

                # u - y negative rotation
                if cmd == 'u':
                    if ry_max + ry > tol:
                        ry -= dry

                # RZ
                # p - z positive rotation
                if cmd == 'p':
                    if rz_max - rz > tol:
                        rz += drz

                # o - z negative rotation
                if cmd == 'o':
                    if rz_max + rz > tol:
                        rz -= drz

                if walk:
                    # if in walking
                    # X
                    # w - x forward speed
                    if cmd == 'w':
                        if vx_max - vx > tol:
                            vx += dvx

                    # s - x backward speed
                    if cmd == 's':
                        if vx_max + vx > tol:
                            vx -= dvx

                    # Y
                    # a - y left speed
                    if cmd == 'a':
                        if vy_max - vy > tol:
                            vy += dvy

                    # d - y right speed
                    if cmd == 'd':
                        if vy_max + vy > tol:
                            vy -= dvy

                    # OVERALL YAW
                    # e - positive yaw rate
                    if cmd == 'e':
                        if yaw_max - yaw_rate > tol:
                            yaw_rate += dyaw_rate

                    # q - negative yaw rate
                    if cmd == 'q':
                        if yaw_max + yaw_rate > tol:
                            yaw_rate -= dyaw_rate

                    # FOOT YAW
                    # LEFT
                    # x - positive yaw
                    if cmd == 'x':
                        if yaw_f_max - yaw_l > tol:
                            yaw_l += dyaw_f

                    # z - negative yaw
                    if cmd == 'z':
                        if yaw_f_max + yaw_l > tol:
                            yaw_l -= dyaw_f

                    # RIGHT
                    # v - positive yaw
                    if cmd == 'v':
                        if yaw_f_max - yaw_r > tol:
                            yaw_r += dyaw_f

                    # c - negative yaw
                    if cmd == 'c':
                        if yaw_f_max + yaw_r > tol:
                            yaw_r -= dyaw_f

                    # FOOT CLEARANCE
                    # m - increase
                    if cmd == 'm':
                        if dy_max - dy > tol:
                            dy += ddy

                    # n - decrease
                    if cmd == 'n':
                        if dy - dy_min > tol:
                            dy -= ddy
                else:
                    # if in balancing
                    # g - com x forward scale
                    if cmd == 'g':
                        if px_max - px_scale > tol:
                            px_scale += dpx

                    # f - com x backward scale
                    if cmd == 'f':
                        if px_max + px_scale > tol:
                            px_scale -= dpx

                    # j - com y left scale
                    if cmd == 'j':
                        if py_max - py_scale > tol:
                            py_scale += dpy

                    # h - com y right scale
                    if cmd == 'h':
                        if py_max + py_scale > tol:
                            py_scale -= dpy

                # l - com z upward scale
                if cmd == 'l':
                    if pz_max - pz > tol:
                        pz += dpz

                # k - com z downward scale
                if cmd == 'k':
                    if pz - pz_min > tol:
                        pz -= dpz

            # set user input command
            input_data['walk'] = np.array([int(walk)])
            input_data['com_xy_velocity'] = np.array([vx, vy])
            input_data['yaw_rate']        = np.array([yaw_rate])

            input_data['right_foot_yaw_angle_change'] = np.array([yaw_r])
            input_data['left_foot_yaw_angle_change']  = np.array([yaw_l])

            input_data['com_position_change_scaled'] = np.array([px_scale, py_scale, pz])

            input_data['body_euler_angle_change'] = np.array([rx, ry, rz])

            input_data['foot_lateral_clearance'] = np.array([dy])

            MM.USER_COMMAND.set(input_data)

            # print out states
            if time.time() - display_last_time > display_duration:
                # get BEAR info
                leg_data = MM.LEG_STATE.get()
                vol_msg = colored('  LOW!!!', 'red') if leg_data['voltage'][0] < 14.5 else ''
                tem_msg = colored('HIGH!!!', 'red') if leg_data['temperature'][0] > 60.0 else ''

                os.system('clear')
                print('====== The User Input Thread is running at', loop_freq, 'Hz... ======')
                print('')
                print('BEAR Voltage:    ', float2str(leg_data['voltage'][0]), '[V]', vol_msg)
                print('BEAR Temperature:', float2str(leg_data['temperature'][0]), '[°C]', tem_msg)
                print('_________')
                print('')
                if input_data['walk'][0] == 0.:
                    print('BALANCING (press spacebar twice to change mode)')
                    print('_________')
                    print('')
                    print('Body Orientation')
                    print('-      roll (t/y):', float2str(rx, 'ang'), '[deg]')
                    print('-     pitch (u/i):', float2str(ry, 'ang'), '[deg]')
                    print('-       yaw (o/p):', float2str(rz, 'ang'), '[deg]')
                    print('')
                    print('CoM Position')
                    print('-  sagittal (f/g):', float2str(px_scale, 'len'), '[%]')
                    print('-   lateral (h/j):', float2str(py_scale, 'len'), '[%]')
                    print('-  vertical (k/l):', float2str(pz,       'len'), '[cm]')
                else:
                    print('WALKING   (press spacebar twice to change mode)')
                    print('_________')
                    print('')
                    print('Body Orientation')
                    print('-      roll (t/y):', float2str(rx, 'ang'), '[deg]')
                    print('-     pitch (u/i):', float2str(ry, 'ang'), '[deg]')
                    print('-       yaw (o/p):', float2str(rz, 'ang'), '[deg]')
                    print('')
                    print('CoM Position')
                    print('-  vertical (k/l):', float2str(pz, 'len'), '[cm]')
                    print('')
                    print('CoM Velocity')
                    print('-  sagittal (w/s):', float2str(vx,        'len'), '[cm/s]')
                    print('-   lateral (a/d):', float2str(vy,        'len'), '[cm/s]')
                    print('-       yaw (q/e):', float2str(yaw_rate,  'ang'), '[deg/s]')
                    print('')
                    print('Swing Foot')
                    print('-  left yaw (z/x):', float2str(yaw_l, 'ang'), '[deg]')
                    print('- right yaw (c/v):', float2str(yaw_r, 'ang'), '[deg]')
                    print('- clearance (n/m):', float2str(dy,    'len'), '[cm]')
                print('')
                if screen_flag:
                    print(colored('Press Ctrl+A+D to go back to the terminal.', 'yellow'))

                display_last_time = time.time()

            # loop time
            loop_target_time = loop_start_time + loop_duration
            while time.time() < loop_target_time:
                pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, filedes)


if __name__ == '__main__':
    try:
        MM.THREAD_STATE.set({'top_level': np.array([1.0])}, opt='only')  # thread is running
        main_loop()
    except:
        time.sleep(0.1)
        MM.THREAD_STATE.set({'top_level': np.array([2.0])}, opt='only')  # thread is stopped
