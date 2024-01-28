#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2023 Westwood Robotics Corporation"
__date__      = "September 6, 2023"
__version__   = "0.0.3"
__status__    = "Production"

"""
Script that holds useful macros for DCM walking tuning
"""

sim = True     # if in simulation or not

# WALKING
# foot swing trajectory
Ts     = 0.22   # desired stance phase duration [s]
Ts_min = 0.18   # minimum stance duration       [s]
Ts_max = 0.30   # maximum stance duration       [s]
T_buff = 0.05   # stop plan before T - T_buff   [s]

Txi = 0.00      # x stay before Txi
Txn = 0.05      # x go to nominal before Txn
Txf = 0.00      # x arrive before T - Txf

Tyi = 0.00      # y stay before Tyi
Tyn = 0.05      # y go to nominal before Tyn
Tyf = 0.00      # y arrive before T - Tyf

Tzm = 0.08      # desired swing apex time [s]
Tzf = 0.00      # z arrive before T - Tzf

zm_l = 0.035    # left  swing apex height [m]
zm_r = 0.035    # right swing apex height [m]

zf_l = -0.002   # left  swing final height [m]
zf_r = -0.002   # right swing final height [m]

hz = 0.355      # desired CoM height [m]

yaw_f_offset = 0.02  # foot yaw offset [rad]

# kinematic reachability [m]
lx  = 0.20      # max longitudinal step length
lyi = 0.05      # min lateral distance between feet
lyo = 0.25      # max lateral distance between feet

# velocity offset compensation [m]
bx_offset = 0.00  # set to negative if BRUCE tends to go forward
by_offset = 0.00  # set to negative if BRUCE tends to go left

# STANCE
ka = 0.00       # x position of CoM from the center of foot, in scale of 1/2 foot length
                # ka = 1 puts CoM at the front tip of foot
