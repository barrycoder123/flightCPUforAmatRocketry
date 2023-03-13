# Libs
import numpy as np
import quaternions as qt
import earth_model as em


# strapdown
#
# Single iteration of a 1st-Order IMU Strapdown
# Credit: Tyler Klein
def strapdown(r_ecef, v_ecef, q_e2b, dV_b_imu, dTh_b_imu, dt):
    # Position Update
    r_ecef_new = r_ecef + dt * v_ecef

    # Attitude Update
    q_bOld2bNew = qt.deltaAngleToDeltaQuat(-dTh_b_imu)
    q_i2bNew = qt.quatMultiply(q_bOld2bNew, q_e2b)
    q_e2i = qt.deltaAngleToDeltaQuat(em.WE * dt * np.array([0, 0, 1]))
    q_e2b_new = qt.quatMultiply(q_i2bNew, q_e2i)

    # Velocity Update
    x, y, z = r_ecef
    g = em.xyz2grav(x, y, z)

    T_i2b = qt.quat2dcm(q_e2b)
    dV_e_imu = np.dot(dV_b_imu, T_i2b.conj())

    acc_cor = np.cross(2 * em.omega, v_ecef)
    acc_cen = np.cross(em.omega, np.cross(em.omega, r_ecef))
    dV_e = dV_e_imu - (acc_cor + acc_cen - g) * dt

    v_ecef_new = v_ecef + dV_e

    return r_ecef_new, v_ecef_new, q_e2b_new
