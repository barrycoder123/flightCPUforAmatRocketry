#Libs
import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt
#from pyquaternion import Quaternion

#import and format data
data = genfromtxt('traj_raster_30mins_20221115_160156.csv', delimiter=',');
data = data[1:][:] #remove NaN
IMU_data = data[:, 11:17];
dt = 0.1 #Manually setting this for now. Might need to change for RT implementation
PVA_truth = data[:, 1:11];

#Initialize arrays
PVA_est = np.zeros((data.shape[0], 10))
x0 = PVA_truth[1, 0:11];
x0 = x0[:]; # Force column vector
PVA_est[0] = x0 # Store initial conditions in first col of estimate

def strapdown(r_ecef, v_ecef, q_e2b, dV_b_imu, dTh_b_imu, dt):
    WE = 7.2921151467e-05;
    omega = np.array([0, 0, WE]);

    #position update
    r_ecef_new = r_ecef + dt * v_ecef;

    
    q_bOld2bNew = deltaAngleToDeltaQuat(-dTh_b_imu)
    q_i2bNew = quatMultiply(q_bOld2bNew, q_e2b);
    q_e2i = q_e2i = deltaAngleToDeltaQuat(WE * dt * np.array([0, 0, 1]));
    q_e2b_new = quatMultiply(q_i2bNew, q_e2i);

    #Velocity Update
    g = xyz2grav(r_ecef[0], r_ecef[1], r_ecef[2]);
    
    T_i2b = quat2dcm(q_e2b);
    dV_e_imu = np.dot(dV_b_imu, T_i2b.conj())
    

    acc_cor = np.cross(2 * omega, v_ecef);
    acc_cen = np.cross(omega, np.cross(omega, r_ecef));
    dV_e = dV_e_imu - (acc_cor + acc_cen - g) * dt;

    v_ecef_new = v_ecef + dV_e;

    return r_ecef_new, v_ecef_new, q_e2b_new;

def normVec(thetas):
    mag = np.linalg.norm(thetas);
    return thetas/mag

def qAngle(thetas):
    return np.linalg.norm(thetas)/2;

# Ellipsoid earth model
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

def deltaAngleToDeltaQuat(dTheta):
# =============================================================================
#     %DELTAANGLETODELTAQUAT Converts a vector of rotations to a Quaternion
#     %
#     % Parameters
#     % ----------
#     % dTheta : (3,) vector
#     %   delta angles [rad]
#     %
#     % Returns
#     % -------
#     % q : (4,) vector
#     %   Quaternion
# =============================================================================
    
    mag = norm(dTheta); # norm
    axis = dTheta / mag; # axis of rotation
    
    # See https://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/index.htm
    theta = mag / 2; # by definition of Quaternion
    q = np.concatenate(([np.cos(theta)], np.sin(theta) * axis));
    
    return q
  
    
def norm(v):
    sum_squares = sum([vi**2 for vi in v])
    return np.sqrt(sum_squares)

    
def quatMultiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    
    return [w, x, y, z]

def quat2dcm(q):
    q0, q1, q2, q3 = q
    dcm = [[q0**2 + q1**2 - q2**2 - q3**2, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
           [2*(q1*q2 + q0*q3), q0**2 - q1**2 + q2**2 - q3**2, 2*(q2*q3 - q0*q1)],
           [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), q0**2 - q1**2 - q2**2 + q3**2]]
    return np.array(dcm)
    

for i in range(data.shape[0] - 1): #data.shape[0] - 1

    # Extract values from IMU
    dV_b_imu = dt * IMU_data[i, 0:3]; # measured delta-V in the body frame [m/s]
    dTh_b_imu = dt * IMU_data[i, 3:6]; # measured delta-theta in the body frame [rad]

    r_ecef = PVA_est[i, 0:3]; # ECEF position [m]
    v_ecef = PVA_est[i, 3:6]; # ECEF velocity [m/s]
    q_e2b = PVA_est[i, 6:10]; # ECEF-to-body Quaternion

    # r correct, v incorrect, q not sure
    r_ecef_new, v_ecef_new, q_e2b_new = strapdown(r_ecef, v_ecef, q_e2b, dV_b_imu, dTh_b_imu, dt);

    PVA_est[i + 1, 0:3] = r_ecef_new;
    PVA_est[i + 1, 3:6] = v_ecef_new;
    PVA_est[i + 1, 6:10] = q_e2b_new;

#rint(PVA_est[:, 0:3])
print("DONE!")

## PLOT POSITION
plt.figure()
plt.plot(PVA_truth[:, 0])
plt.plot(PVA_est[:, 0])
plt.title("X POSITION")
plt.legend(["Truth","Estimation"])
plt.figure()
plt.plot(PVA_truth[:, 1])
plt.plot(PVA_est[:, 1])
plt.title("Y POSITION")
plt.legend(["Truth","Estimation"])
plt.figure()
plt.plot(PVA_truth[:, 2])
plt.plot(PVA_est[:, 2])
plt.title("Z POSITION")
plt.legend(["Truth","Estimation"])


## PLOT VELOCITY
plt.figure()
plt.plot(PVA_truth[:, 3])
plt.plot(PVA_est[:, 3])
plt.title("X VELOCITY")
plt.legend(["Truth","Estimation"])
plt.figure()
plt.plot(PVA_truth[:, 4])
plt.plot(PVA_est[:, 4])
plt.title("Y VELOCITY")
plt.legend(["Truth","Estimation"])
plt.figure()
plt.plot(PVA_truth[:, 5])
plt.plot(PVA_est[:, 5])
plt.title("Z VELOCITY")
plt.legend(["Truth","Estimation"])

## PLOT ATTITUDE
plt.figure()
plt.plot(PVA_truth[:, 6])
plt.plot(PVA_est[:, 6])
plt.title("SCALAR QUATERNION")
plt.legend(["Truth","Estimation"])
plt.figure()
plt.plot(PVA_truth[:, 7])
plt.plot(PVA_est[:, 7])
plt.title("I QUATERNION")
plt.legend(["Truth","Estimation"])
plt.figure()
plt.plot(PVA_truth[:, 8])
plt.plot(PVA_est[:, 8])
plt.title("J QUATERNION")
plt.legend(["Truth","Estimation"])
plt.figure()
plt.plot(PVA_truth[:, 9])
plt.plot(PVA_est[:, 9])
plt.title("K QUATERNION")
plt.legend(["Truth","Estimation"])