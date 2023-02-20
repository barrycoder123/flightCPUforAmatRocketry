import numpy as np

class EKF:
    def __init__(self, x, P, Q, R, f, h, H):
        self.x = x
        self.P = P
        self.Q = Q
        self.R = R
        self.f = f
        self.h = h
        self.H = H

    def predict(self, f, u, dt):
        # predict state estimate
        self.x = f(self.x, u, dt)
        # predict state covariance
        self.P = self.P + self.Q * dt

    def update(self, z, h):
        # compute innovation
        y = z - h(self.x)
        # compute innovation covariance
        S = self.H @ self.P @ self.H.T + self.R
        # compute Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
        # update state estimate
        self.x = self.x + K @ y
        # update state covariance
        self.P = (np.eye(self.P.shape[0]) - K @ self.H) @ self.P


""" IMU CONVERSION EQUATION """
def f(x, u, dt, imu_data):
    # Unpack state variables
    p, v, q = x[:3], x[3:6], x[6:]

    # Unpack IMU data
    acc, gyro = imu_data[:3], imu_data[3:]

    # Compute body frame acceleration
    acc_body = q_inv(q) @ (acc - u[:3])

    # Compute new position and velocity
    p_new = p + v * dt + 0.5 * acc_body * dt**2
    v_new = v + acc_body * dt

    # Compute new quaternion
    q_dot = 0.5 * quat_multiply(q, np.hstack((0, gyro)))
    q_new = q + q_dot * dt

    # Normalize quaternion
    q_new /= np.linalg.norm(q_new)

    # Pack state variables into new state vector
    x_new = np.hstack((p_new, v_new, q_new))

    return x_new


""" GPS CONVERSION EQUATION """
def h(x, gps_data, baro_data):
    # Unpack state variables
    p, v, q = x[:3], x[3:6], x[6:]

    # Unpack GPS data
    gps = gps_data

    # Unpack barometer data
    baro = baro_data

    # Convert position to GPS coordinates
    lat, lon, alt = ecef2lla(p)

    # Pack GPS and barometer measurements into measurement vector
    z = np.hstack((lat, lon, alt, baro))

    return z



def H(x):
    # unpack state
    q = x[6:]

    # calculate rotation matrix
    C = quat2rotmat(q)

    # calculate measurement Jacobian
    H = np.zeros((3, 7))
    H[:, :3] = np.eye(3)
    H[:, 6] = C.T @ np.array([0, 0, 1])

    return H


def quat2rotmat(q):
    """Convert quaternion to rotation matrix."""
    q0, q1, q2, q3 = q
    return np.array([
        [1 - 2*q2**2 - 2*q3**2, 2*q1*q2 - 2*q0*q3, 2*q1*q3 + 2*q0*q2],
        [2*q1*q2 + 2*q0*q3, 1 - 2*q1**2 - 2*q3**2, 2*q2*q3 - 2*q0*q1],
        [2*q1*q3 - 2*q0*q2, 2*q2*q3 + 2*q0*q1, 1 - 2*q1**2 - 2*q2**2]
    ])


if __name__ == "__main__":
    
    # step 1 - state vector: [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, quat-e2bi, quat-e2bj, quat-e2bk, mag(quat)]
    x = np.array([0, 0, 0, 0, 0, 0, 1, 0, 0, 0])
    
    # step 2 - measurement vector: [pos_x_imu, pos_y_imu, pos_z_imu, vel_x_imu, vel_y_imu, vel_z_imu, ..., altitude]
    z = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    
    
    

    # # Define the state covariance matrix P
    # P = np.diag([100, 100, 100, 10, 10, 10, 0.1, 0.1, 0.1, 0.1])**2
    
    # # Define the process noise covariance matrix Q
    # Q = np.diag([1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2, 1e-5, 1e-5, 1e-5, 1e-5])**2
    
    # # Define the measurement noise covariance matrix R
    # R_gps = np.diag([1e-5, 1e-5, 1])**2
    # R_baro = np.array([1])**2
    # R = np.block([[R_gps, np.zeros((3,1))],
    #               [np.zeros((1,3)), R_baro]])
    
    # # Define inputs
    # u = imu_data
    # dt = time_delta
    
    # ekf = EKF(x, P, Q, R, f, )

    # # Predict state
    # ekf.predict(f, u, dt, imu_data)
    
    # # Define measurements
    # z = np.hstack((gps_data, baro_data))

    # # Update state
    # ekf.update(z, h, gps_data, baro_data)
