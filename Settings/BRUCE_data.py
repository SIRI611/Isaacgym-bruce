#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2023 Westwood Robotics Corporation"
__date__      = "September 6, 2023"
__version__   = "0.0.3"
__status__    = "Production"

"""
Script that holds useful robot info
"""

import time
import collections
import Startups.memory_manager as MM
from termcolor import colored
from Settings.BRUCE_macros import *
from Play.Walking.walking_macros import *


class BRUCE:
    def __init__(self):
        # PyBEAR settings
        self.BEAR_modes = {'torque': 0, 'velocity': 1, 'position': 2, 'force': 3}

        # DXL settings
        self.DXL_modes = {'position': 3, 'velocity': 1, 'extended position': 4, 'PWM': 16}

        # Joint info
        self.joint = collections.defaultdict(lambda: collections.defaultdict(int))

    def update_robot_status(self):
        """
        Get robot states from shared memory.
        """
        estimation_data = MM.ESTIMATOR_STATE.get()

        # time
        self.t = estimation_data['time_stamp'][0]

        # body
        self.p_wb = estimation_data['body_position']
        self.v_wb = estimation_data['body_velocity']
        self.a_wb = estimation_data['body_acceleration']
        self.R_wb = estimation_data['body_rot_matrix']
        self.w_bb = estimation_data['body_ang_rate']
        self.yaw  = estimation_data['body_yaw_ang'][0]

        # center of mass
        self.p_wg = estimation_data['com_position']
        self.v_wg = estimation_data['com_velocity']
        self.k_wg = estimation_data['ang_momentum']

        # dynamics
        self.H     = estimation_data['H_matrix']
        self.CG    = estimation_data['CG_vector']
        self.AG    = estimation_data['AG_matrix']
        self.dAGdq = estimation_data['dAGdq_vector']

        # foot contacts
        self.foot_contacts = estimation_data['foot_contacts']

        # right foot
        self.R_wf_r     = estimation_data['right_foot_rot_matrix']
        self.w_ff_r     = estimation_data['right_foot_ang_rate']
        self.Jw_ff_r    = estimation_data['right_foot_Jw']
        self.dJwdq_ff_r = estimation_data['right_foot_dJwdq']
        self.p_wf_r     = estimation_data['right_foot_position']
        self.v_wf_r     = estimation_data['right_foot_velocity']
        self.p_wt_r     = estimation_data['right_toe_position']
        self.v_wt_r     = estimation_data['right_toe_velocity']
        self.Jv_wt_r    = estimation_data['right_toe_Jv']
        self.dJvdq_wt_r = estimation_data['right_toe_dJvdq']
        self.p_wh_r     = estimation_data['right_heel_position']
        self.v_wh_r     = estimation_data['right_heel_velocity']
        self.Jv_wh_r    = estimation_data['right_heel_Jv']
        self.dJvdq_wh_r = estimation_data['right_heel_dJvdq']
        self.p_wa_r     = estimation_data['right_ankle_position']
        self.v_wa_r     = estimation_data['right_ankle_velocity']
        self.Jv_wa_r    = estimation_data['right_ankle_Jv']
        self.dJvdq_wa_r = estimation_data['right_ankle_dJvdq']

        # left foot
        self.R_wf_l     = estimation_data['left_foot_rot_matrix']
        self.w_ff_l     = estimation_data['left_foot_ang_rate']
        self.Jw_ff_l    = estimation_data['left_foot_Jw']
        self.dJwdq_ff_l = estimation_data['left_foot_dJwdq']
        self.p_wf_l     = estimation_data['left_foot_position']
        self.v_wf_l     = estimation_data['left_foot_velocity']
        self.p_wt_l     = estimation_data['left_toe_position']
        self.v_wt_l     = estimation_data['left_toe_velocity']
        self.Jv_wt_l    = estimation_data['left_toe_Jv']
        self.dJvdq_wt_l = estimation_data['left_toe_dJvdq']
        self.p_wh_l     = estimation_data['left_heel_position']
        self.v_wh_l     = estimation_data['left_heel_velocity']
        self.Jv_wh_l    = estimation_data['left_heel_Jv']
        self.dJvdq_wh_l = estimation_data['left_heel_dJvdq']
        self.p_wa_l     = estimation_data['left_ankle_position']
        self.v_wa_l     = estimation_data['left_ankle_velocity']
        self.Jv_wa_l    = estimation_data['left_ankle_Jv']
        self.dJvdq_wa_l = estimation_data['left_ankle_dJvdq']

    def update_robot_DCM_status(self):
        estimation_data = MM.ESTIMATOR_STATE.get()

        self.yaw   = estimation_data['body_yaw_ang'][0]
        self.R_yaw = MF.Rz(1e-6 + self.yaw)

        self.p_wb = estimation_data['body_position']
        self.R_wb = estimation_data['body_rot_matrix']

        self.p_wg = estimation_data['com_position']
        self.v_wg = estimation_data['com_velocity']

        self.foot_contacts = estimation_data['foot_contacts']

        self.p_wt_r = estimation_data['right_toe_position']
        self.p_wh_r = estimation_data['right_heel_position']
        self.p_wf_r = estimation_data['right_foot_position']
        self.p_wa_r = estimation_data['right_ankle_position']

        self.p_wt_l = estimation_data['left_toe_position']
        self.p_wh_l = estimation_data['left_heel_position']
        self.p_wf_l = estimation_data['left_foot_position']
        self.p_wa_l = estimation_data['left_ankle_position']

    def update_plan_status(self):
        plan_data = MM.PLANNER_COMMAND.get()

        self.state    = int(plan_data['robot_state'][0])
        self.des_p_wb = plan_data['body_position']
        self.des_v_wb = plan_data['body_velocity']
        self.des_R_wb = plan_data['body_rot_matrix']
        self.des_w_bb = plan_data['body_ang_rate']
        self.des_p_wg = plan_data['com_position']
        self.des_v_wg = plan_data['com_velocity']

        self.des_p_wf_r = plan_data['right_foot_position']
        self.des_v_wf_r = plan_data['right_foot_velocity']
        self.des_R_wf_r = plan_data['right_foot_rot_matrix']
        self.des_w_ff_r = plan_data['right_foot_ang_rate']

        self.des_p_wf_l = plan_data['left_foot_position']
        self.des_v_wf_l = plan_data['left_foot_velocity']
        self.des_R_wf_l = plan_data['left_foot_rot_matrix']
        self.des_w_ff_l = plan_data['left_foot_ang_rate']

    def update_input_status(self):
        input_data = MM.USER_COMMAND.get()

        self.walk             = input_data['walk'][0]
        self.cmd_vxy_wg       = input_data['com_xy_velocity']
        self.cmd_yaw_rate     = input_data['yaw_rate'][0]
        self.cmd_p_wg_change  = input_data['com_position_change_scaled']
        self.cmd_euler_change = input_data['body_euler_angle_change']
        self.cmd_R_change     = MF.Rx(self.cmd_euler_change[0]) @ MF.Ry(self.cmd_euler_change[1]) @ MF.Rz(self.cmd_euler_change[2])
        self.cmd_yaw_r_change = input_data['right_foot_yaw_angle_change'][0]
        self.cmd_yaw_l_change = input_data['left_foot_yaw_angle_change'][0]
        self.cmd_dy           = input_data['foot_lateral_clearance'][0]

    def update_leg_status(self):
        """
        Get leg states from shared memory.
        """
        leg_data = MM.LEG_STATE.get()
        q  = leg_data['joint_positions']
        dq = leg_data['joint_velocities']

        # right leg
        self.joint[HIP_YAW_R]['q']     = q[0]
        self.joint[HIP_ROLL_R]['q']    = q[1]
        self.joint[HIP_PITCH_R]['q']   = q[2]
        self.joint[KNEE_PITCH_R]['q']  = q[3]
        self.joint[ANKLE_PITCH_R]['q'] = q[4]

        self.joint[HIP_YAW_R]['dq']     = dq[0]
        self.joint[HIP_ROLL_R]['dq']    = dq[1]
        self.joint[HIP_PITCH_R]['dq']   = dq[2]
        self.joint[KNEE_PITCH_R]['dq']  = dq[3]
        self.joint[ANKLE_PITCH_R]['dq'] = dq[4]

        # left leg
        self.joint[HIP_YAW_L]['q']     = q[5]
        self.joint[HIP_ROLL_L]['q']    = q[6]
        self.joint[HIP_PITCH_L]['q']   = q[7]
        self.joint[KNEE_PITCH_L]['q']  = q[8]
        self.joint[ANKLE_PITCH_L]['q'] = q[9]

        self.joint[HIP_YAW_L]['dq']     = dq[5]
        self.joint[HIP_ROLL_L]['dq']    = dq[6]
        self.joint[HIP_PITCH_L]['dq']   = dq[7]
        self.joint[KNEE_PITCH_L]['dq']  = dq[8]
        self.joint[ANKLE_PITCH_L]['dq'] = dq[9]

    def set_command_leg_positions(self):
        """
        Set command leg joint positions to shared memory.
        """
        commands = {'BEAR_enable': np.array([1.0]),
                    'BEAR_mode': np.array([self.BEAR_modes['position']]),
                    'goal_positions': np.array([self.joint[HIP_YAW_R]['q_goal'],
                                                self.joint[HIP_ROLL_R]['q_goal'],
                                                self.joint[HIP_PITCH_R]['q_goal'],
                                                self.joint[KNEE_PITCH_R]['q_goal'],
                                                self.joint[ANKLE_PITCH_R]['q_goal'],
                                                self.joint[HIP_YAW_L]['q_goal'],
                                                self.joint[HIP_ROLL_L]['q_goal'],
                                                self.joint[HIP_PITCH_L]['q_goal'],
                                                self.joint[KNEE_PITCH_L]['q_goal'],
                                                self.joint[ANKLE_PITCH_L]['q_goal'],
                                                ])
                    }
        MM.LEG_COMMAND.set(commands)

    def set_command_leg_torques(self):
        """
        Set command leg joint torques to shared memory.
        """
        commands = {'BEAR_enable': np.array([1.0]),
                    'BEAR_mode': np.array([self.BEAR_modes['torque']]),
                    'goal_torques': np.array([self.joint[HIP_YAW_R]['tau_goal'],
                                              self.joint[HIP_ROLL_R]['tau_goal'],
                                              self.joint[HIP_PITCH_R]['tau_goal'],
                                              self.joint[KNEE_PITCH_R]['tau_goal'],
                                              self.joint[ANKLE_PITCH_R]['tau_goal'],
                                              self.joint[HIP_YAW_L]['tau_goal'],
                                              self.joint[HIP_ROLL_L]['tau_goal'],
                                              self.joint[HIP_PITCH_L]['tau_goal'],
                                              self.joint[KNEE_PITCH_L]['tau_goal'],
                                              self.joint[ANKLE_PITCH_L]['tau_goal'],
                                              ])
                    }
        MM.LEG_COMMAND.set(commands)

    def set_command_leg_values(self):
        """
        Set command leg joint values to shared memory.
        """
        commands = {'BEAR_enable': np.array([1.0]),
                    'BEAR_mode': np.array([self.BEAR_modes['force']]),
                    'goal_torques': np.array([self.joint[HIP_YAW_R]['tau_goal'],
                                              self.joint[HIP_ROLL_R]['tau_goal'],
                                              self.joint[HIP_PITCH_R]['tau_goal'],
                                              self.joint[KNEE_PITCH_R]['tau_goal'],
                                              self.joint[ANKLE_PITCH_R]['tau_goal'],
                                              self.joint[HIP_YAW_L]['tau_goal'],
                                              self.joint[HIP_ROLL_L]['tau_goal'],
                                              self.joint[HIP_PITCH_L]['tau_goal'],
                                              self.joint[KNEE_PITCH_L]['tau_goal'],
                                              self.joint[ANKLE_PITCH_L]['tau_goal'],
                                              ]),
                    'goal_positions': np.array([self.joint[HIP_YAW_R]['q_goal'],
                                                self.joint[HIP_ROLL_R]['q_goal'],
                                                self.joint[HIP_PITCH_R]['q_goal'],
                                                self.joint[KNEE_PITCH_R]['q_goal'],
                                                self.joint[ANKLE_PITCH_R]['q_goal'],
                                                self.joint[HIP_YAW_L]['q_goal'],
                                                self.joint[HIP_ROLL_L]['q_goal'],
                                                self.joint[HIP_PITCH_L]['q_goal'],
                                                self.joint[KNEE_PITCH_L]['q_goal'],
                                                self.joint[ANKLE_PITCH_L]['q_goal'],
                                                ]),
                    'goal_velocities': np.array([self.joint[HIP_YAW_R]['dq_goal'],
                                                 self.joint[HIP_ROLL_R]['dq_goal'],
                                                 self.joint[HIP_PITCH_R]['dq_goal'],
                                                 self.joint[KNEE_PITCH_R]['dq_goal'],
                                                 self.joint[ANKLE_PITCH_R]['dq_goal'],
                                                 self.joint[HIP_YAW_L]['dq_goal'],
                                                 self.joint[HIP_ROLL_L]['dq_goal'],
                                                 self.joint[HIP_PITCH_L]['dq_goal'],
                                                 self.joint[KNEE_PITCH_L]['dq_goal'],
                                                 self.joint[ANKLE_PITCH_L]['dq_goal'],
                                                 ])
                    }
        MM.LEG_COMMAND.set(commands)

    def update_arm_status(self):
        arm_data = MM.ARM_STATE.get()
        q  = arm_data['joint_positions']
        dq = arm_data['joint_velocities']

        # right arm
        self.joint[SHOULDER_PITCH_R]['q'] = q[0]
        self.joint[SHOULDER_ROLL_R]['q']  = q[1]
        self.joint[ELBOW_YAW_R]['q']      = q[2]

        self.joint[SHOULDER_PITCH_R]['dq'] = dq[0]
        self.joint[SHOULDER_ROLL_R]['dq']  = dq[1]
        self.joint[ELBOW_YAW_R]['dq']      = dq[2]

        # left arm
        self.joint[SHOULDER_PITCH_L]['q'] = q[3]
        self.joint[SHOULDER_ROLL_L]['q']  = q[4]
        self.joint[ELBOW_YAW_L]['q']      = q[5]

        self.joint[SHOULDER_PITCH_L]['dq'] = dq[3]
        self.joint[SHOULDER_ROLL_L]['dq']  = dq[4]
        self.joint[ELBOW_YAW_L]['dq']      = dq[5]

    def set_command_arm_positions(self):
        """
        Set command arm joint positions to shared memory.
        """
        commands = {'DXL_enable': np.array([1.0]),
                    'DXL_mode': np.array([self.DXL_modes['position']]),
                    'goal_positions': np.array([self.joint[SHOULDER_PITCH_R]['q_goal'],
                                                self.joint[SHOULDER_ROLL_R]['q_goal'],
                                                self.joint[ELBOW_YAW_R]['q_goal'],
                                                self.joint[SHOULDER_PITCH_L]['q_goal'],
                                                self.joint[SHOULDER_ROLL_L]['q_goal'],
                                                self.joint[ELBOW_YAW_L]['q_goal'],
                                                ])
                    }
        MM.ARM_COMMAND.set(commands)

    @staticmethod
    def thread_error():
        thread_data = MM.THREAD_STATE.get()
        if thread_data['bear'][0] == 2.0:
            print(colored('BEAR Thread Error! Terminate Now!', 'red'))
            return True
        if thread_data['estimation'][0] == 2.0:
            print(colored('Estimation Thread Error! Terminate Now!', 'red'))
            return True
        if thread_data['low_level'][0] == 2.0:
            print(colored('Low-Level Thread Error! Terminate Now!', 'red'))
            return True
        if thread_data['high_level'][0] == 2.0:
            print(colored('High-Level Thread Error! Terminate Now!', 'red'))
            return True
        if thread_data['top_level'][0] == 2.0:
            print(colored('Top-Level Thread Error! Terminate Now!', 'red'))
            return True
        return False

    @staticmethod
    def stop_robot():
        BEAR_commands = {'BEAR_enable': np.array([0.0])}
        MM.LEG_COMMAND.set(BEAR_commands)

        DXL_commands = {'DXL_enable': np.array([0.0])}
        MM.ARM_COMMAND.set(DXL_commands)

    @staticmethod
    def damping_robot():
        commands = {'BEAR_enable': np.array([1.0]),
                    'damping': np.array([1.0])}
        MM.LEG_COMMAND.set(commands)

    @staticmethod
    def is_damping():
        leg_data = MM.LEG_STATE.get()
        return False if leg_data['damping'][0] == 0.0 else True

    @staticmethod
    def get_time():
        if sim:
            estimation_data = MM.ESTIMATOR_STATE.get()
            return estimation_data['time_stamp'][0]
        else:
            return time.time()

    @staticmethod
    def sleep(dt):
        if sim:
            t0 = BRUCE.get_time()
            while BRUCE.get_time() - t0 < dt:
                pass
        else:
            time.sleep(dt)

    @staticmethod
    def stop_threading():
        THIS_IS_AN_INTENTIONAL_ERROR  # raise a stupid error to terminate the thread
