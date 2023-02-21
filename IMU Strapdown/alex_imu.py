#Libs
import numpy as np
from numpy import genfromtxt
from pyquaternion import Quaternion

#import and format data
data = genfromtxt('traj_raster_30mins_20221115_160156.csv', delimiter=',');
IMU_data = data[:, 11:17];
dt = 0.1 #Manually setting this for now. Might need to change for RT implementation
PVA_truth = data[:, 0:11];

#Initialize arrays 
PVA_est = np.zeros((data.shape[0], 10))
x0 = PVA_truth[1, 1:11];
x0 = x0[:]; # Force column vector
PVA_est[1] = x0 # Store initial conditions in first col of estimate

def strapdown(r_ecef, v_ecef, q_e2b, dV_b_imu, dTh_b_imu, dt):
    WE = 7.2921151467e-05;
    omega = np.array([0, 0, WE]);

    #position update
    r_ecef_new = r_ecef + dt * v_ecef;

    
    q_bOld2bNew = Quaternion(axis=normVec(-dTh_b_imu), angle=qAngle(-dTh_b_imu));
    q_i2bNew = q_bOld2bNew * q_e2b;
    q_e2i = Quaternion(Quaternion(a=np.cos(qAngle(omega*dt)), b=0.0, c=0.0, d=np.sin(qAngle(omega*dt))));
    q_e2b_new = q_i2bNew * q_e2i;

    #Velocity Update
    g = xyz2grav(r_ecef[0], r_ecef[1], r_ecef[2]);
    
    T_i2b = q_e2b.rotation_matrix;
    dV_e_imu = np.matmul(dV_b_imu, T_i2b.transpose());
    

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


for i in range(1, data.shape[0] - 1): #data.shape[0] - 1

    # Extract values from IMU
    dV_b_imu = dt * IMU_data[i, 0:3]; # measured delta-V in the body frame [m/s]
    dTh_b_imu = dt * IMU_data[i, 3:6]; # measured delta-theta in the body frame [rad]

    r_ecef = PVA_est[i, 0:3]; # ECEF position [m]
    v_ecef = PVA_est[i, 3:6]; # ECEF velocity [m/s]
    q_e2b = PVA_est[i, 6:10]; # ECEF-to-body Quaternion
    q_e2b = Quaternion(axis = q_e2b[1:4], angle=q_e2b[0]);
    r_ecef_new, v_ecef_new, q_e2b_new = strapdown(r_ecef, v_ecef, q_e2b, dV_b_imu, dTh_b_imu, dt);

    PVA_est[i + 1, 0:3] = r_ecef_new;
    PVA_est[i + 1, 3:6] = v_ecef_new;
    print(v_ecef_new)
    PVA_est[i + 1, 6:10] = q_e2b_new;

#print(PVA_est[:, 0:3])