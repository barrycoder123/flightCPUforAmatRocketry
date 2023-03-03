import numpy as np
#import strapdown as sd

class EKF:
    def __init__(self, x, z, P, Q, R, f, F, h, H):
        self.x = x
        self.z = z
        self.P = P
        self.Q = Q
        self.R = R
        self.f = f
        self.F = F
        self.h = h
        self.H = H

    def predict(self, f):
        # predict state estimate
        self.x, dt = f(self.x)
        # predict state covariance
        self.P = self.F * self.P * self.F.conv() + self.Q * dt

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
def f(x):
    
    r_ecef, v_ecef, q_e2b = x[0:3], x[3:6], x[6:10]
    dV_b_imu, dTh_b_imu, dt = get_next_imu_reading() # TODO
    
    r_ecef_new, v_ecef_new, q_e2b_new = sd.strapdown(r_ecef, v_ecef, q_e2b, dV_b_imu, dTh_b_imu, dt);

    return np.concatenate((r_ecef_new, v_ecef_new, q_e2b_new)), dt


""" GPS CONVERSION EQUATION """
def h(x):
    # Unpack state variables
    p, v, q = x[:3], x[3:6], x[6:]

    # Convert position (xyz) to GPS coordinates (lla)
    lat, lon, alt = ecef2lla(p) # TODO

    return np.array([lat, lon, alt])


""" H """
def H(r_ecef, HAE=True):
    """
    Computes the Jacobian of (lat, lon, alt) in [rad, rad, m (HAE)] w.r.t ECEF position in [meters]. This uses the Geodetic WGS-84 ellipsoid,
    which is the general standard defining lat/lon.

    It may be useful to allow the altitude to be w.r.t standard pressure, but that has not been added at this point

    The Jacobian is defined such that :math:`\\nabla_{ecef} f = \\nabla_{lla} f * J`

    Parameters
    ----------
    r_ecef : (3,) array-like
        ECEF position vector in meters

    HAE : bool, default=True
        if True, the altitude will be the height above the elipsoid. If False, the height will be the radius from the center of the Earth or height above
        mean sea level (the jacobian will be the same) todo:

    Returns
    -------
    J : (3, 3) ndarray
        Jacobian of (lat, lon, alt) in [rad, rad, m (HAE)] w.r.t ECEF position

    Notes
    -----
    The partials were verified by sweeping over random ECEF coordinates and taking the numerical derivative
    The phase shifts arctan2 were omitted as they didn't seem to matter

    Derived by Tyler Klein, 06/2022
    """
    
    x = r_ecef[0]
    y = r_ecef[1]
    z = r_ecef[2]

    # looking at eceftolla, we can calculate the partials
    grad_lon = np.array([[-y, x, 0]]) / (x * x + y * y)  # gradient of longitude w.r.t (x,y,z)

    b = np.sqrt((a ** 2) * (1 - e ** 2))
    ep = np.sqrt((a ** 2 - b ** 2) / b ** 2)
    p = np.sqrt(x ** 2 + y ** 2)
    grad_p = np.array([x, y, 0]) / p

    th = np.arctan2(a * z, b * p)
    dth_dp_coeff = - a * z / (b * p * p)
    grad_th = np.array([[dth_dp_coeff * grad_p[0], dth_dp_coeff * grad_p[1], a / (b * p)]]) / (1.0 + ((a * z) / (b * p)) ** 2)

    # the latitude is a PAIN
    num = z + ep ** 2 * b * np.sin(th) ** 3
    grad_num = np.array([0, 0, 1.0]) + 3 * ep ** 2 * b * np.sin(th) ** 2 * np.cos(th) * grad_th
    den = p - e ** 2 * a * np.cos(th) ** 3
    grad_den = grad_p + 3 * e ** 2 * a * np.cos(th) ** 2 * np.sin(th) * grad_th
    lat = np.arctan2(num, den)
    grad_lat = ((1.0 / den) * grad_num - (num / den ** 2) * grad_den) * 1.0 / (1.0 + (num / den) ** 2)

    if HAE:
        grad_N = grad_lat * a * (e ** 2 * np.sin(lat) * np.cos(lat)) / (1 - (e ** 2) * np.sin(lat) ** 2) ** (3 / 2)
        grad_alt = grad_p / np.cos(lat) + grad_lat * p * np.sin(lat) / np.cos(lat) ** 2 - grad_N
    else:
        raise NotImplementedError('Need to subtract out ellipsoid height')

    J = np.vstack((grad_lat, grad_lon, grad_alt))
    return J


def quat2rotmat(q):
    """Convert quaternion to rotation matrix."""
    q0, q1, q2, q3 = q
    return np.array([
        [1 - 2*q2**2 - 2*q3**2, 2*q1*q2 - 2*q0*q3, 2*q1*q3 + 2*q0*q2],
        [2*q1*q2 + 2*q0*q3, 1 - 2*q1**2 - 2*q3**2, 2*q2*q3 - 2*q0*q1],
        [2*q1*q3 - 2*q0*q2, 2*q2*q3 + 2*q0*q1, 1 - 2*q1**2 - 2*q2**2]
    ])


if __name__ == "__main__":
    WE = 7.2921151467e-05;
    omega = np.array([0, 0, WE]);
    omega_cross = np.vstack([[0, -omega[2], omega[1]],
                   [omega[2], 0, -omega[0]],
                   [-omega[1], omega[0], 0]])
    am_cross = omega_cross
    
    # step 1 - state vector: [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, quat-e2bi, quat-e2bj, quat-e2bk, mag(quat)]
    x = np.array([0, 0, 0, 0, 0, 0, 1, 0, 0, 0])
    
    # step 2 - measurement vector holds GPS measurement (and evnetually barometer)
    z = np.array([0, 0, 0, 0, 0, 0]) # [lat, lon, atti, baro1, baro2, baro3]

    # Initialize P, Q, R
    # P: predicted covariance matrix, can be random, 10x10, maybe I * .1
    # Q: measurement noise matrix, 10x10, I * 0.001
    # R: 
    # F: 10x10, given by Tyler (add a row of 0's)

    grav_gradient = np.zeros((3,3))
    ang_rot_cross = np.zeros((3,3))
    T_b2i = np.zeros((3,3))
    
    drdr = np.zeros((3,3))
    drdv = np.identity(3)
    drdo = np.zeros((3,3))
    dvdr = grav_gradient - np.square(omega_cross)
    dvdv = -2 * omega_cross
    dvdo = -T_b2i * am_cross
    dodr = np.zeros((3,3))
    dodv = np.zeros((3,3))
    dodo = -ang_rot_cross

    F = [np.hstack([drdr, drdv, drdo]),
         np.hstack([dvdr, dvdv, dvdo]),
         np.hstack([dodr, dodv, dodo])]
    
    F = np.ravel(F)
    

    # Initialize EKF
    ekf = EKF(x, P, Q, R, f, F, h, H)

    while 1:
        
        # Predict state
        ekf.predict(f)
        
        # get GPS data
        z = ping_gps_for_reading() # TODO
        
        # If new GPS reading, update state
        ekf.update(z, h) 