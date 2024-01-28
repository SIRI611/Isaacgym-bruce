#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2023 Westwood Robotics Corporation"
__date__      = "September 6, 2023"
__version__   = "0.0.3"
__status__    = "Production"

'''
Script usage:
1. estimate robot states, e.g., body orientation, angular velocity, position, velocity, foot position, velocity, etc.
2. calculate robot model, e.g., kinematics (Jacobian and its derivative) and dynamics (equations of motion)
'''

import time
import Settings.BRUCE_data as RDS
import Startups.memory_manager as MM
import Library.ROBOT_MODEL.BRUCE_dynamics as dyn
import Library.ROBOT_MODEL.BRUCE_kinematics as kin
import Library.STATE_ESTIMATION.BRUCE_estimation_CF as est_CF
import Library.STATE_ESTIMATION.BRUCE_estimation_KF as est_KF
from termcolor import colored
from Settings.BRUCE_macros import *
from Library.BRUCE_SENSE.sense import Manager as sense


def IMU_check():
    import statistics

    print('Checking IMU ...')
    IMU_error = True
    while IMU_error:
        data  = []
        count = 0
        while count < 1000:
            # get sensor data
            rtn = s.get_dump_YOLO()
            if rtn is not None:
                data.append(rtn[0][0])
                count += 1

        data_std = statistics.stdev(data)
        if data_std > 0.5:
            print(data_std)
            print(colored('IMU Error! Resetting ...', 'red'))
            s.send_data(mode='reset')
            time.sleep(0.5)
            print('Checking IMU Again ...')
        else:
            print('IMU OK!')
            IMU_error = False


def contact_check():
    print('Checking Foot Contact ...')
    foot_contacts = np.zeros(4)
    foot_contacts_seq = [1, 0, 2, 3]
    count = 0
    while count < 1000:

        # get sensor data
        rtn = s.get_dump_YOLO()
        if rtn is not None:
            contact_data = rtn[1]

            for idx in range(4):
                foot_contacts[idx] = 1 - (contact_data >> foot_contacts_seq[idx] & 1)

            if np.any(foot_contacts[0:2]) or np.any(foot_contacts[2:4]):
                count += 1

    print('Foot On Ground!')


def main_loop():
    # BRUCE Setup
    Bruce = RDS.BRUCE()

    # Parameters
    loop_freq            = 500  # run at 500 Hz
    temp_update_freq     = 1    # update temperature at 1 Hz
    loop_duration        = 1. / loop_freq
    temp_update_duration = 1. / temp_update_freq

    # IMU Measurements
    gravity_accel = 9.81                           # gravity  acceleration
    accel = np.array([0., 0., gravity_accel])      # filtered accelerometer reading
    omega = np.array([0., 0., 0.])                 # filtered gyroscope     reading
    accel_new = np.array([0., 0., gravity_accel])  # new      accelerometer reading
    omega_new = np.array([0., 0., 0.])             # new      gyroscope     reading

    # Timing Mechanism (to check if IMU is static for orientation estimation)
    IMU_start_time = 0.     # start timer
    IMU_diff_max   = 0.2    # maximum acceptable difference between accelerometer reading and gravity
    IMU_period     = 1.0    # IMU considered static after some period of consistent reading, e.g., 1.0 seconds
    IMU_static     = False  # IMU considered static or not

    # Foot Contacts
    foot_contacts       = np.zeros(4)    # 0/1 indicate in air/contact (for right/left toe/heel)
    foot_contacts_seq   = [1, 0, 2, 3]   # foot contact signal sequence [rt, rh, lt, lh]
    foot_contacts_CF    = np.zeros(2)    # for complementary filter
    foot_contacts_count = np.zeros(4)    # indicate how long the foot is in contact

    # Initial Guess
    p_wb  = np.array([-hx, 0., 0.38])    # body position         - in world frame
    v_wb  = np.array([0., 0., 0.])       # body velocity         - in world frame
    a_wb  = np.array([0., 0., 0.])       # body acceleration     - in world frame
    R_wb  = np.eye(3)                    # body orientation      - in world frame
    v_bb  = R_wb.T @ v_wb                # body velocity         - in  body frame
    w_bb  = np.array([0., 0., 0.])       # body angular velocity - in  body frame
    b_acc = np.array([0., 0., 0.])       # accelerometer bias    - in   IMU frame

    p_wt_r = np.array([+at, -0.05, 0.])  # right toe   position  - in world frame
    p_wh_r = np.array([-ah, -0.05, 0.])  # right heel  position  - in world frame
    p_wf_r = np.array([0.0, -0.05, 0.])  # right foot  position  - in world frame

    c_wt_r = np.array([+at, -0.05, 0.])  # right toe   position if in contact
    c_wh_r = np.array([-ah, -0.05, 0.])  # right heel  position if in contact
    c_wf_r = np.array([0.0, -0.05, 0.])  # right foot  position if in contact

    p_wt_l = np.array([+at,  0.05, 0.])  # left  toe   position  - in world frame
    p_wh_l = np.array([-ah,  0.05, 0.])  # left  heel  position  - in world frame
    p_wf_l = np.array([0.0,  0.05, 0.])  # left  foot  position  - in world frame

    c_wt_l = np.array([+at,  0.05, 0.])  # left  toe   position if in contact
    c_wh_l = np.array([-ah,  0.05, 0.])  # left  heel  position if in contact
    c_wf_l = np.array([0.0,  0.05, 0.])  # left  foot  position if in contact

    Po     = np.eye(21) * 1e-2           # Kalman filter state covariance matrix

    # Shared Memory Data
    estimation_data = {'body_rot_matrix': np.zeros((3, 3))}

    # Start Estimation
    estimator = input('Choose complementary filter (CF) or Kalman filter (KF)? ')
    if estimator != 'CF' or 'KF':
        estimator = 'KF'

    print("====== The State Estimation Thread is running at", loop_freq, "Hz... ======")

    t0 = Bruce.get_time()
    last_temp_update_time = t0
    while True:
        loop_start_time = Bruce.get_time()
        elapsed_time    = loop_start_time - t0

        # check threading error
        if Bruce.thread_error():
            Bruce.stop_threading()

        # update plan status
        Bruce.update_plan_status()

        # get sensor data
        rtn = s.get_dump_YOLO()
        if rtn is not None:
            imu_data     = rtn[0]
            contact_data = rtn[1]

            # IMU
            accel_new = np.array([imu_data[0], imu_data[1], imu_data[2]])
            omega_new = np.array([imu_data[3], imu_data[4], imu_data[5]])
            for idx in range(3):
                accel[idx] = MF.exp_filter(accel[idx], accel_new[idx], 0.50)
                omega[idx] = MF.exp_filter(omega[idx], omega_new[idx], 0.50)

            # IMU static check
            IMU_static = False
            if Bruce.state == 0:
                if np.abs(MF.norm(accel_new) - gravity_accel) > IMU_diff_max:
                    IMU_start_time = Bruce.get_time()
                IMU_static = True if Bruce.get_time() - IMU_start_time > IMU_period else False

            # foot contacts
            for idx in range(4):
                foot_contacts[idx] = 1 - (contact_data >> foot_contacts_seq[idx] & 1)
                foot_contacts_count[idx] = foot_contacts_count[idx] + 1 if foot_contacts[idx] else 0

            # for complementary filter
            foot_contacts_CF[0] = 1 if foot_contacts[0] or foot_contacts[1] else 0
            foot_contacts_CF[1] = 1 if foot_contacts[2] or foot_contacts[3] else 0

        # for walking purpose
        if Bruce.state == 1:
            foot_contacts_count[2:4] = np.zeros(2)
            foot_contacts_CF[1]      = 0
        elif Bruce.state == 2:
            foot_contacts_count[0:2] = np.zeros(2)
            foot_contacts_CF[0]      = 0

        # get BEAR info from shared memory
        leg_data = MM.LEG_STATE.get()

        # send temperature info to Pico
        if loop_start_time - last_temp_update_time > temp_update_duration:
            s.send_data(temperature=leg_data['temperature'])
            last_temp_update_time = loop_start_time

        # get leg joint states
        q  = leg_data['joint_positions']
        dq = leg_data['joint_velocities']
        r1, r2, r3, r4, r5 = q[0], q[1], q[2], q[3], q[4]
        l1, l2, l3, l4, l5 = q[5], q[6], q[7], q[8], q[9]
        dr1, dr2, dr3, dr4, dr5 = dq[0], dq[1], dq[2], dq[3], dq[4]
        dl1, dl2, dl3, dl4, dl5 = dq[5], dq[6], dq[7], dq[8], dq[9]

        # compute leg forward kinematics
        p_bt_r, v_bt_r, Jv_bt_r, dJv_bt_r, \
        p_bh_r, v_bh_r, Jv_bh_r, dJv_bh_r, \
        p_ba_r, v_ba_r, Jv_ba_r, dJv_ba_r, \
        p_bf_r, v_bf_r,  R_bf_r,  Jw_bf_r, dJw_bf_r, \
        p_bt_l, v_bt_l, Jv_bt_l, dJv_bt_l, \
        p_bh_l, v_bh_l, Jv_bh_l, dJv_bh_l, \
        p_ba_l, v_ba_l, Jv_ba_l, dJv_ba_l, \
        p_bf_l, v_bf_l,  R_bf_l,  Jw_bf_l, dJw_bf_l = kin.legFK(r1, r2, r3, r4, r5,
                                                                l1, l2, l3, l4, l5,
                                                                dr1, dr2, dr3, dr4, dr5,
                                                                dl1, dl2, dl3, dl4, dl5)

        # state estimation
        if estimator == 'CF':
            kR, kp, kv = 0.001, np.array([0.1, 0.1, 0.1]), np.array([0.1, 0.1, 0.1])
            R_wb, w_bb, yaw_angle, \
            p_wb, v_wb, v_bb, a_wb, \
            c_wf_r, c_wf_l = est_CF.run(R_wb, p_wb, v_wb, a_wb,
                                        c_wf_r, c_wf_l,
                                        p_bf_r, p_bf_l,
                                        v_bf_r, v_bf_l,
                                        p_wf_r, p_wf_l,
                                        omega, accel, foot_contacts_CF,
                                        kR, kp, kv, gravity_accel)
        elif estimator == 'KF':
            R_wb, w_bb, yaw_angle, \
            p_wb, v_wb, v_bb, a_wb, b_acc, \
            c_wt_r, c_wh_r, c_wt_l, c_wh_l, Po = est_KF.run(R_wb, w_bb,
                                                            p_wb, v_wb, a_wb, b_acc,
                                                            c_wt_r, c_wh_r, c_wt_l, c_wh_l,
                                                            Po,
                                                            p_bt_r, p_bh_r, p_bt_l, p_bh_l,
                                                            v_bt_r, v_bh_r, v_bt_l, v_bh_l,
                                                            p_wt_r, p_wh_r, p_wt_l, p_wh_l,
                                                            omega_new, accel_new, foot_contacts_count,
                                                            IMU_static, gravity_accel)

        # compute robot forward kinematics
        p_wt_r, v_wt_r, Jv_wt_r, dJvdq_wt_r, \
        p_wh_r, v_wh_r, Jv_wh_r, dJvdq_wh_r, \
        p_wa_r, v_wa_r, Jv_wa_r, dJvdq_wa_r, \
        p_wf_r, v_wf_r,  \
        R_wf_r, w_ff_r, Jw_ff_r, dJwdq_ff_r, \
        p_wt_l, v_wt_l, Jv_wt_l, dJvdq_wt_l, \
        p_wh_l, v_wh_l, Jv_wh_l, dJvdq_wh_l, \
        p_wa_l, v_wa_l, Jv_wa_l, dJvdq_wa_l, \
        p_wf_l, v_wf_l,  \
        R_wf_l, w_ff_l, Jw_ff_l, dJwdq_ff_l = kin.robotFK(R_wb, p_wb, w_bb, v_bb,
                                                          p_bt_r, Jv_bt_r, dJv_bt_r,
                                                          p_bh_r, Jv_bh_r, dJv_bh_r,
                                                          p_ba_r, Jv_ba_r, dJv_ba_r, R_bf_r, Jw_bf_r, dJw_bf_r,
                                                          p_bt_l, Jv_bt_l, dJv_bt_l,
                                                          p_bh_l, Jv_bh_l, dJv_bh_l,
                                                          p_ba_l, Jv_ba_l, dJv_ba_l, R_bf_l, Jw_bf_l, dJw_bf_l,
                                                          dr1, dr2, dr3, dr4, dr5,
                                                          dl1, dl2, dl3, dl4, dl5)

        # calculate robot dynamics
        H, CG, AG, dAGdq, p_wg, v_wg, k_wg = dyn.robotID(R_wb, p_wb, w_bb, v_bb,
                                                         r1, r2, r3, r4, r5,
                                                         l1, l2, l3, l4, l5,
                                                         dr1, dr2, dr3, dr4, dr5,
                                                         dl1, dl2, dl3, dl4, dl5)

        # save data
        estimation_data['time_stamp']        = np.array([elapsed_time])
        estimation_data['body_position']     = p_wb
        estimation_data['body_velocity']     = v_wb
        estimation_data['body_acceleration'] = a_wb
        estimation_data['body_rot_matrix']   = R_wb
        estimation_data['body_ang_rate']     = w_bb
        estimation_data['body_yaw_ang']      = np.array([yaw_angle])
        estimation_data['com_position']      = p_wg
        estimation_data['com_velocity']      = v_wg
        estimation_data['ang_momentum']      = k_wg
        estimation_data['H_matrix']          = H
        estimation_data['CG_vector']         = CG
        estimation_data['AG_matrix']         = AG
        estimation_data['dAGdq_vector']      = dAGdq
        estimation_data['foot_contacts']     = foot_contacts

        estimation_data['right_foot_rot_matrix'] = R_wf_r
        estimation_data['right_foot_ang_rate']   = w_ff_r
        estimation_data['right_foot_Jw']         = Jw_ff_r
        estimation_data['right_foot_dJwdq']      = dJwdq_ff_r
        estimation_data['right_foot_position']   = p_wf_r
        estimation_data['right_foot_velocity']   = v_wf_r
        estimation_data['right_toe_position']    = p_wt_r
        estimation_data['right_toe_velocity']    = v_wt_r
        estimation_data['right_toe_Jv']          = Jv_wt_r
        estimation_data['right_toe_dJvdq']       = dJvdq_wt_r
        estimation_data['right_heel_position']   = p_wh_r
        estimation_data['right_heel_velocity']   = v_wh_r
        estimation_data['right_heel_Jv']         = Jv_wh_r
        estimation_data['right_heel_dJvdq']      = dJvdq_wh_r
        estimation_data['right_ankle_position']  = p_wa_r
        estimation_data['right_ankle_velocity']  = v_wa_r
        estimation_data['right_ankle_Jv']        = Jv_wa_r
        estimation_data['right_ankle_dJvdq']     = dJvdq_wa_r

        estimation_data['left_foot_rot_matrix']  = R_wf_l
        estimation_data['left_foot_ang_rate']    = w_ff_l
        estimation_data['left_foot_Jw']          = Jw_ff_l
        estimation_data['left_foot_dJwdq']       = dJwdq_ff_l
        estimation_data['left_foot_position']    = p_wf_l
        estimation_data['left_foot_velocity']    = v_wf_l
        estimation_data['left_toe_position']     = p_wt_l
        estimation_data['left_toe_velocity']     = v_wt_l
        estimation_data['left_toe_Jv']           = Jv_wt_l
        estimation_data['left_toe_dJvdq']        = dJvdq_wt_l
        estimation_data['left_heel_position']    = p_wh_l
        estimation_data['left_heel_velocity']    = v_wh_l
        estimation_data['left_heel_Jv']          = Jv_wh_l
        estimation_data['left_heel_dJvdq']       = dJvdq_wh_l
        estimation_data['left_ankle_position']   = p_wa_l
        estimation_data['left_ankle_velocity']   = v_wa_l
        estimation_data['left_ankle_Jv']         = Jv_wa_l
        estimation_data['left_ankle_dJvdq']      = dJvdq_wa_l

        MM.ESTIMATOR_STATE.set(estimation_data)

        # check time to ensure that the state estimator stays at a consistent running loop.
        loop_end_time = loop_start_time + loop_duration
        present_time  = Bruce.get_time()
        if present_time > loop_end_time:
            delay_time = 1000 * (present_time - loop_end_time)
            if delay_time > 1.:
                print(colored('Delayed ' + str(delay_time)[0:5] + ' ms at Te = ' + str(elapsed_time)[0:5] + ' s', 'yellow'))
        else:
            while Bruce.get_time() < loop_end_time:
                pass


if __name__ == '__main__':
    # Pico Setup
    s = sense.SENSOR(port=PICO_port, baudrate=PICO_baudrate)
    s.send_data(mode='run')  # run Pico

    IMU_check()
    contact_check()
    try:
        MM.THREAD_STATE.set({'estimation': np.array([1.0])}, opt='only')  # thread is running
        main_loop()
    except:
        s.send_data(mode='idle')  # set Pico to idle mode
        time.sleep(0.1)
        MM.THREAD_STATE.set({'estimation': np.array([2.0])}, opt='only')  # thread is stopped
