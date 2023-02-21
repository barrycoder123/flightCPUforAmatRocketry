#Libs
import numpy as np


# strapdown
#
# Single iteration of a 1st-Order IMU Strapdown
def strapdown(r_ecef, v_ecef, q_e2b, dV_b_imu, dTh_b_imu, dt):
    WE = 7.2921151467e-05;
    omega = np.array([0, 0, WE]);

    # Position Update
    r_ecef_new = r_ecef + dt * v_ecef;

    # Attitude Update
    q_bOld2bNew = deltaAngleToDeltaQuat(-dTh_b_imu)
    q_i2bNew = quatMultiply(q_bOld2bNew, q_e2b);
    q_e2i = q_e2i = deltaAngleToDeltaQuat(WE * dt * np.array([0, 0, 1]));
    q_e2b_new = quatMultiply(q_i2bNew, q_e2i);

    # Velocity Update
    g = xyz2grav(r_ecef[0], r_ecef[1], r_ecef[2]);
    
    T_i2b = quat2dcm(q_e2b);
    dV_e_imu = np.dot(dV_b_imu, T_i2b.conj())
    
    acc_cor = np.cross(2 * omega, v_ecef);
    acc_cen = np.cross(omega, np.cross(omega, r_ecef));
    dV_e = dV_e_imu - (acc_cor + acc_cen - g) * dt;

    v_ecef_new = v_ecef + dV_e;

    return r_ecef_new, v_ecef_new, q_e2b_new;


# xyz2grav
#
# Ellipsoid Earth gravity model
def xyz2grav(x, y, z):
    j2 = 0.00108263
    mu = 3.986004418e14
    R = 6378137
    r = np.sqrt(x**2 + y**2 + z**2)
    sub1 = 1.5*j2*((R/r)**2)
    sub2 = (5*z**2)/(r**2)
    sub3 = (-mu)/(r**3)
    sub4 = sub3 * (1 - sub1 * (sub2 - 1));
    gx = x * (sub4)
    gy = y * (sub4);
    gz = z * (sub3) * (1 - sub1 * (sub2 - 3));
    g = np.array([gx, gy, gz]);
    g = g[:] #Force column
    return g


# deltaAngleToDeltaQuat
#
# Converts a vector of rotations to a Quaternion
#     Parameters
#     ----------
#     dTheta : (3,) vector
#       delta angles [rad]
#
#     Returns
#     -------
#     q : (4,) vector
#       Quaternion
def deltaAngleToDeltaQuat(dTheta):
    
    mag = norm(dTheta); # norm
    axis = dTheta / mag; # axis of rotation
    
    # See https://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/index.html
    theta = mag / 2; # by definition of Quaternion
    q = np.concatenate(([np.cos(theta)], np.sin(theta) * axis));
    
    return q
  
def norm(v):
    sum_squares = sum([vi**2 for vi in v])
    return np.sqrt(sum_squares)


# quatMultiply
#
# Multiply two quaternions  
def quatMultiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    
    return [w, x, y, z]

# quat2dcm
#
# Convert quaternion to a 3x3 discrete cosine matrix (dcm)
def quat2dcm(q):
    q0, q1, q2, q3 = q
    dcm = [[q0**2 + q1**2 - q2**2 - q3**2, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
           [2*(q1*q2 + q0*q3), q0**2 - q1**2 + q2**2 - q3**2, 2*(q2*q3 - q0*q1)],
           [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), q0**2 - q1**2 - q2**2 + q3**2]]
    return np.array(dcm)
    