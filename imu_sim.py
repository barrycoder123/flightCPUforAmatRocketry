import scipy as sc
import numpy as np
from navpy import quat2dcm 

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

def quat2dcm(norm_quat):
    '''
    read: 
    quaternion to dcm given a n x 4 quaternion matrix
    returns 3 x 3 dcm matrix
    dcm used to change coordinates from inertial to body
    use navpy's quat to dcm
    quat2dcm(q0, qvec)
    q0 is the scalar component of the quaternion
    qvec is the vector component of the quaternion
    '''


if __name__ == "__main__":
    ## add alex's code here
