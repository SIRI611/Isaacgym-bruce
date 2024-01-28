#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2023 Westwood Robotics Corporation"
__date__      = "September 6, 2023"
__version__   = "0.0.3"
__status__    = "Production"

"""
Compile BRUCE state estimation (complementary filter version) ahead of time (AOT) using Numba
"""

from numba.pycc import CC
from Settings.BRUCE_macros import *


cc = CC('BRUCE_estimation_CF')


# FREQUENCY SETTING
freq  = 500.  # run at 500 Hz
dt    = 1. / freq
dt2   = dt * dt
dt2_2 = dt2 / 2.


@cc.export('run', '(f8[:,:], f8[:], f8[:], f8[:],'
                  'f8[:], f8[:],'
                  'f8[:], f8[:], '
                  'f8[:], f8[:],'
                  'f8[:], f8[:],'
                  'f8[:], f8[:], f8[:],'
                  'f8, f8[:], f8[:], f8)')
def run(R0, p0, v0, a0,
        crm0, clm0,
        prm, plm,
        vrm, vlm,
        crm, clm,
        omg, acc, foot_contacts,
        kR, kp, kv, g):

    # ORIENTATION ESTIMATE
    # predict
    R1 = np.copy(R0) @ MF.hatexp(omg * dt)
    w1 = omg

    # correct
    gm  = R1 @ np.copy(acc)
    gmn = MF.norm(gm)
    if gmn > 1e-10:
        gmu = gm / gmn
        dphi = np.arccos(gmu[2] * np.sign(g))
        if np.abs(dphi) > 1e-10:
            nv = MF.hat(gmu) @ np.array([0., 0., np.sign(g)]) / np.sin(dphi)  # rotation axis
            R1 = MF.hatexp(kR * dphi * nv) @ R1

    yaw = np.arctan2(R1[1, 0], R1[0, 0])
    a1  = R1 @ np.copy(acc) - np.array([0., 0., g])

    # POSITION AND VELOCITY ESTIMATE
    # predict
    p1 = p0 + v0 * dt + a0 * dt2_2
    v1 = v0 + a0 * dt

    # correct
    pc   = np.zeros(3)
    vc   = np.zeros(3)
    what = MF.hat(omg)
    total_contacts = 0
    if foot_contacts[0]:
        total_contacts += 1
        prm = np.copy(prm)
        pc += crm0 - R1 @ prm
        vc -= R1 @ (what @ prm + vrm)
    else:
        crm0 = crm

    if foot_contacts[1]:
        total_contacts += 1
        plm = np.copy(plm)
        pc += clm0 - R1 @ plm
        vc -= R1 @ (what @ plm + vlm)
    else:
        clm0 = clm

    if total_contacts == 0:  # do nothing if lose all contacts to prevent divergence
        p1 = p0
        v1 = v0
    else:
        pc /= total_contacts
        vc /= total_contacts
        for i in range(3):
            p1[i] = kp[i] * pc[i] + (1. - kp[i]) * p1[i]

        for i in range(3):
            v1[i] = kv[i] * vc[i] + (1. - kv[i]) * v1[i]
    bv1 = R1.T @ np.copy(v1)

    return R1, w1, yaw, \
           p1, v1, bv1, a1, \
           crm0, clm0


if __name__ == '__main__':
    cc.compile()
