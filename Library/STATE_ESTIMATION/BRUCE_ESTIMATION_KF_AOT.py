#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2023 Westwood Robotics Corporation"
__date__      = "September 6, 2023"
__version__   = "0.0.3"
__status__    = "Production"

"""
Compile BRUCE state estimation (Kalman filter version) ahead of time (AOT) using Numba
"""

import scipy.linalg
from numba.pycc import CC
from Settings.BRUCE_macros import *


cc = CC('BRUCE_estimation_KF')


# FREQUENCY SETTING
freq  = 500     # run at 500 Hz
dt    = 1. / freq
dt2   = dt * dt
dt2_2 = dt2 / 2.

# BODY ORIENTATION
acc_thr = 0.10  # acceleration threshold
acc_dec = 0.05  # acceleration decaying factor

# BODY POSITION AND VELOCITY
# constant
ns = 3 * 7      # number of states
nn = 3 * 6      # number of process noises
I3 = np.eye(3)
Is = np.eye(ns)

# model
Phi = np.copy(Is)
Phi[0:3, 3:6] = I3 * dt

B = np.zeros((ns, 3))
B[0:3, 0:3] = I3 * dt2_2
B[3:6, 0:3] = I3 * dt

Gam = np.zeros((ns, nn))
Gam[6:ns, 3:nn] = np.eye(15) * dt

# covariance
Qa  = 1e-1**2 * np.diag([1., 1., 1.])  # accelerometer noise
Qba = 1e-4**2 * np.diag([1., 1., 1.])  # accelerometer bias noise
Qc0 = 1.e5**2 * np.diag([1., 1., 1.])  # new foot contact noise
Qc1 = 1e-2**2 * np.diag([1., 1., 1.])  # foot contact noise
Q   = scipy.linalg.block_diag(Qa, Qba, Qc1, Qc1, Qc1, Qc1)

Vp  = 1e-3**2 * np.diag([1., 1., 1.])  # foot position FK noise
Vv  = 1e-2**2 * np.diag([1., 1., 1.])  # foot velocity FK noise


@cc.export('run', '(f8[:,:], f8[:],'
                  'f8[:], f8[:], f8[:], f8[:],'
                  'f8[:], f8[:], f8[:], f8[:],'
                  'f8[:,:],'
                  'f8[:], f8[:], f8[:], f8[:],'
                  'f8[:], f8[:], f8[:], f8[:],'
                  'f8[:], f8[:], f8[:], f8[:],'
                  'f8[:], f8[:], f8[:], f8, f8)')
def run(R0, w0,
        p0, v0, a0, ba0,
        crt0, crh0, clt0, clh0,
        P0,
        prt, prh, plt, plh,
        vrt, vrh, vlt, vlh,
        crt, crh, clt, clh,
        omg, acc, contacts_count, imu_static, g):

    # SETTING
    # constant
    gv = np.array([0., 0., g])
    contacts_num = int(np.sum(contacts_count > 0))  # number of foot contacts

    # previous a posteriori state estimate
    x0 = np.hstack((p0, v0, ba0, crt0, crh0, clt0, clh0))

    # current available measurements
    z1 = np.hstack((prt, prh, plt, plh,
                    vrt, vrh, vlt, vlh,
                    crt, crh, clt, clh))

    # covariance reset for new foot contacts
    for i in range(4):
        if contacts_count[i] == 1:
            id1 = 3 * i + 9
            id2 = id1 + 3
            id3 = id1 + 15
            id4 = id3 + 3

            P0[id1:id2,    0:ns] = np.zeros((3, ns))
            P0[   0:ns, id1:id2] = P0[id1:id2, 0:ns].T
            P0[id1:id2, id1:id2] = Qc0

            x0[id1:id2] = z1[id3:id4]

    # ORIENTATION ESTIMATE
    # predict
    R1 = np.copy(R0) @ MF.hatexp(w0 * dt)
    w1 = omg

    # update
    if imu_static:  # only when IMU is static
        am   = np.abs(MF.norm(acc) - np.abs(g))
        alp  = 0.005 * (1. if am < acc_thr else np.exp(-((am - acc_thr) / acc_dec)**2))  # smoothing function
        gm   = R1 @ np.copy(acc)
        gmu  = gm / MF.norm(gm)
        dphi = np.arccos(gmu[2] * np.sign(g))
        if np.abs(dphi) > 1e-10:
            nv = MF.hat(gmu) @ np.array([0., 0., np.sign(g)]) / np.sin(dphi)  # rotation axis
            R1 = MF.hatexp(alp * dphi * nv) @ R1

    yaw = np.arctan2(R1[1, 0], R1[0, 0])
    RT  = R1.T
    wRT = MF.hat(w1) @ RT

    # POSITION AND VELOCITY ESTIMATE
    # predict
    Phik = np.copy(Phi)
    Phik[0:3, 6:9] = -R0 * dt2_2
    Phik[3:6, 6:9] = -R0 * dt

    Gamk = np.copy(Gam)
    Gamk[0:6, 0:3] = Phik[0:6, 6:9]

    P1 = Phik @ np.copy(P0) @ Phik.T + Gamk @ Q @ Gamk.T
    x1 = np.copy(x0)
    x1[0:3] = p0 + v0 * dt + a0 * dt2_2
    x1[3:6] = v0 + a0 * dt

    # update
    if contacts_num > 0:
        nm = 6 * contacts_num  # current number of measurements
        Hk = np.zeros((nm, ns))
        Vk = np.zeros((nm, nm))
        yk = np.zeros(nm)      # innovation
        j  = 0
        for i in range(4):
            if contacts_count[i] > 0:
                id1 = 3 * j
                id2 = id1 + 3
                id3 = id1 + 3 * contacts_num
                id4 = id3 + 3

                id5 = 3 * i
                id6 = id5 + 3
                id7 = id5 + 9
                id8 = id7 + 3
                id9 = id8 + 3

                Hk[id1:id2,     0:3] = -RT
                Hk[id1:id2, id7:id8] =  RT
                Hk[id3:id4,     0:3] =  wRT
                Hk[id3:id4,     3:6] = -RT
                Hk[id3:id4, id7:id8] = -wRT

                Vk[id1:id2, id1:id2] = Vp
                Vk[id3:id4, id3:id4] = Vv

                ci_p = x1[id7:id8] - x1[0:3]
                yk[id1:id2] = z1[id5:id6] -  RT @ ci_p
                yk[id3:id4] = z1[id8:id9] + wRT @ ci_p + RT @ x1[3:6]

                j += 1

        PHT    = P1 @ Hk.T
        Sk     = Hk @ PHT + Vk
        Sk_inv = np.linalg.pinv(Sk)
        Kk     = PHT @ Sk_inv
        IKH    = Is - Kk @ Hk
        P1     = IKH @ P1 @ IKH.T + Kk @ Vk @ Kk.T
        # P1     = IKH @ P1
        x1    += Kk @ yk

    p1   = x1[0:3]
    v1   = x1[3:6]
    ba1  = x1[6:9]
    crt1 = x1[9:12]
    crh1 = x1[12:15]
    clt1 = x1[15:18]
    clh1 = x1[18:21]

    a1  = R1 @ (acc - ba1) - gv  # body acceleration excluding gravity
    bv1 = RT @ v1

    return R1, w1, yaw, \
           p1, v1, bv1, a1, ba1,\
           crt1, crh1, clt1, clh1, P1


if __name__ == '__main__':
    cc.compile()
