import time
import Settings.BRUCE_data as RDS
import Startups.memory_manager as MM
import Library.ROBOT_MODEL.BRUCE_kinematics as kin
import Library.ROBOT_MODEL.BRUCE_dynamics as dyn
from Settings.BRUCE_macros import *
from Library.BRUCE_GYM.GAZEBO_INTERFACE import Manager as gazint
from sim_bruce import GazeboSimulator as gs

class SimEnv():
    def __init__(self) :
        self.simulator = gazint.GazeboInterface(robot_name='bruce', num_joints=self.num_joints, num_contact_sensors=self.num_contact_sensors)
        self.num_legs = 2
        self.num_joints_per_leg = 5
        self.num_arms = 2
        self.num_joints_per_arms = 3
        self.num_joints = self.num_legs * self.num_joints_per_leg + self.num_arms * self.num_joints_per_arms
        self.num_contact_sensors = 4
        
        self.leg_p_gains = [265, 150,  80,  80,    30]
        self.leg_i_gains = [  0,   0,   0,   0,     0]
        self.leg_d_gains = [ 1., 2.3, 0.8, 0.8, 0.003]

        self.arm_p_gains = [ 1.6,  1.6,  1.6]
        self.arm_i_gains = [   0,    0,    0]
        self.arm_d_gains = [0.03, 0.03, 0.03]

        self.p_gains = self.leg_p_gains * 2 + self.arm_p_gains * 2  # the joint order matches the robot's sdf file
        self.i_gains = self.leg_i_gains * 2 + self.arm_i_gains * 2
        self.d_gains = self.leg_d_gains * 2 + self.arm_d_gains * 2
        
        # simulator info
        self.simulator = None
        self.simulation_frequency = 1000  # Hz
        self.simulation_modes = {'torque': 0, 'position': 2}
        self.simulation_mode = self.simulation_modes['position']
        self.simulator.set_step_size(1. / self.simulation_frequency)
        self.simulator.set_operating_mode(self.simulation_mode)
        self.simulator.set_all_position_pid_gains(self.p_gains, self.i_gains, self.d_gains)

    def reset(self):
        # arm pose
        ar1, ar2, ar3 = -0.7,  1.3,  2.0
        al1, al2, al3 =  0.7, -1.3, -2.0

        # leg pose
        bpr = np.array([0.04, -0.07, -0.42])  # right foot position  in body frame
        bpl = np.array([0.04, +0.07, -0.42])  # left  foot position  in body frame
        bxr = np.array([1., 0., 0.])          # right foot direction in body frame
        bxl = np.array([1., 0., 0.])          # left  foot direction in body frame
        lr1, lr2, lr3, lr4, lr5 = kin.legIK_foot(bpr, bxr, +1.)
        ll1, ll2, ll3, ll4, ll5 = kin.legIK_foot(bpl, bxl, -1.)
        initial_pose = [lr1+PI_2, lr2-PI_2, lr3, lr4, lr5,
                        ll1+PI_2, ll2-PI_2, ll3, ll4, ll5,
                        ar1, ar2, ar3,
                        al1, al2, al3]
        self.simulator.reset_simulation(initial_pose=initial_pose)

    def step(self, action):
        
        self.simulator.step_simulation()