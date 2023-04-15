# Libs
import numpy as np
import quaternions as qt
import earth_model as em


def strapdown(r_ecef, v_ecef, q_e2b, dV_b_imu, dTh_b_imu, dt):
    """
    Runs one single iteration of a 1-st order IMU Strapdown
    Credit: Tyler Klein

    Parameters
    ----------
    r_ecef : (3,)
        DESCRIPTION.
    v_ecef : (3,)
        DESCRIPTION.
    q_e2b : (4,)
        DESCRIPTION.
    dV_b_imu : (3,)
        DESCRIPTION.
    dTh_b_imu : (3,)
        DESCRIPTION.
    dt : (3,)
        DESCRIPTION.

    Returns
    -------
    r_ecef_new : (3,)
        DESCRIPTION.
    v_ecef_new : (3,)
        DESCRIPTION.
    q_e2b_new : (4,)
        DESCRIPTION.

    """
    
    
    # Position Update
    r_ecef_new = r_ecef + dt * v_ecef

    # Attitude Update
    q_bOld2bNew = qt.deltaAngleToDeltaQuat(-dTh_b_imu)
    q_i2bNew = qt.quatMultiply(q_bOld2bNew, q_e2b)
    q_e2i = qt.deltaAngleToDeltaQuat(em.WE * dt * np.array([0, 0, 1]))
    q_e2b_new = qt.quatMultiply(q_i2bNew, q_e2i)

    # Velocity Update
    g = em.xyz2grav(r_ecef)

    T_i2b = qt.quat2dcm(q_e2b)
    dV_e_imu = np.dot(dV_b_imu, T_i2b.conj())

    acc_cor = np.cross(2 * em.omega, v_ecef)
    acc_cen = np.cross(em.omega, np.cross(em.omega, r_ecef))
    dV_e = dV_e_imu - (acc_cor + acc_cen - g) * dt

    v_ecef_new = v_ecef + dV_e

    return r_ecef_new, v_ecef_new, q_e2b_new
