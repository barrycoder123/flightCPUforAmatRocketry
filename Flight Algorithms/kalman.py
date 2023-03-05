import numpy as np

import strapdown as sd
import data_collection as dc
import earth_model as em


class EKF:
    def __init__(self, x, P, Q, R, f, F, h, H):
        self.x = x
        self.P = P
        self.Q = Q
        self.R = R
        self.f = f
        self.F = F
        self.h = h
        self.H = H

    def predict(self):
        # predict state estimate
        self.x, dt = self.f(self.x)
        # predict state covariance
        self.P = self.F * self.P * self.F.transpose() + self.Q * dt
        
        return dt

    def update(self, z):
        # compute innovation
        y = z - self.h(self.x)
        # compute innovation covariance
        H = self.H(x[:3]) # compute H matrix
        S = H @ self.P @ H.transpose() + self.R
        # compute Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
        # update state estimate
        self.x = self.x + K @ y
        # update state covariance
        self.P = (np.eye(self.P.shape[0]) - K @ self.H) @ self.P


""" IMU CONVERSION EQUATION """
def f(x):
    
    r_ecef, v_ecef, q_e2b = x[0:3], x[3:6], x[6:10]
    
    # grab next IMU reading
    accel, gyro, dt = dc.get_next_imu_reading()
    dV_b_imu = accel * dt
    dTh_b_imu = gyro * dt
    
    # iterate a strapdown
    r_ecef_new, v_ecef_new, q_e2b_new = sd.strapdown(r_ecef, v_ecef, q_e2b, dV_b_imu, dTh_b_imu, dt);

    return np.concatenate((r_ecef_new, v_ecef_new, q_e2b_new)), dt


""" GPS CONVERSION EQUATION """
def h(x):
    # Unpack state variables (position)
    p = x[:3]

    # Convert position (xyz) to GPS coordinates (lla)
    lat, lon, alt = em.ecef2lla(p[0], p[1], p[2])

    # TODO: Placeholders for when we add barometer
    baro1 = 0
    baro2 = 0
    baro3 = 0 

    return np.array([lat, lon, alt, baro1, baro2, baro3])


""" H """
def H(x):
    
    # TODO: This may need some barometer stuff
    return em.lla_jacobian(x[:3], HAE=True)


if __name__ == "__main__":

    omega_cross = np.vstack([[0, -em.omega[2], em.omega[1]],
                   [em.omega[2], 0, -em.omega[0]],
                   [-em.omega[1], em.omega[0], 0]])
    am_cross = omega_cross
    
    # state vector: [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, quat-e2bi, quat-e2bj, quat-e2bk, mag(quat)]
    x = np.array([0, 0, 0, 0, 0, 0, 1, 0, 0, 0])
    
    # smeasurement vector: [lat, lon, atti, baro1, baro2, baro3]
    z = np.array([0, 0, 0, 0, 0, 0])

    # Initialize P, Q, R
    # P: predicted covariance matrix, can be random, 10x10, maybe I * .1
    # Q: measurement noise matrix, 10x10, I * 0.001
    # R: 
    # F: 10x10, given by Tyler (add a row of 0's)
    P = np.zeros((10,10))
    Q = np.zeros((10,10))
    R = np.zeros((10,10))
    F = np.zeros((10,10))
    
# =============================================================================
# 
#     # TODO: DEFINE THE F MATRIX
#     grav_gradient = np.zeros((3,3))
#     ang_rot_cross = np.zeros((3,3))
#     T_b2i = np.zeros((3,3))
#     
#     drdr = np.zeros((3,3))
#     drdv = np.identity(3)
#     drdo = np.zeros((3,3))
#     dvdr = grav_gradient - np.square(omega_cross)
#     dvdv = -2 * omega_cross
#     dvdo = -T_b2i * am_cross
#     dodr = np.zeros((3,3))
#     dodv = np.zeros((3,3))
#     dodo = -ang_rot_cross
# 
#     F = [np.hstack([drdr, drdv, drdo]),
#          np.hstack([dvdr, dvdv, dvdo]),
#          np.hstack([dodr, dodv, dodo])]
#     
#     F = np.ravel(F)
# =============================================================================
    
    # Initialize EKF
    ekf = EKF(x, P, Q, R, f, F, h, H)

    while 1:
        # Predict state
        ekf.predict()
        
        # If new GPS reading, update state
        if dc.gps_is_ready():
            lat, long, atti, dt = dc.get_next_gps_reading()
            baro1, baro2, baro3 = dc.get_netx_barometer_reading() # TODO
            
            z = [lat, long, atti, baro1, baro2, baro3]
            ekf.update(z) 
            
            
            