#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar  4 17:54:05 2023

@author: zrummler

PURPOSE: Library for ellipsoid Earth model and refernce frames

FUNCTIONS:
    ecef2lla
    lla2ecef
    alt2pres
    xyz2grav
    lla2quat
    lla_jacobian

"""

import numpy as np
import pandas as pd
import data_collection as dc

WE = 7.2921151467e-05  # rad/sec
omega = np.array([0, 0, WE])

# Define the WGS84 ellipsoid parameters
a = 6378137.0  # semi-major axis of Earth, in meters
b = 6356752.314245  # semi-minor axis of Earth, in meters
e = 0.0818191908426  # eccentricity of Earth


# ecef2lla
#
# generate GPS [lat, long, h] from ECEF (x, y, z)
# h is height above sea level
# https://stackoverflow.com/questions/56945401/converting-xyz-coordinates-to-longitutde-latitude-in-python
def ecef2lla(xyz):
    """
    Converts xyz position to GPS position

    Args:
    - xyz: (3,N) ECEF xyz position [meters]

    Returns:
    - lla: (3,N) dimensional GPS lla position [lat (deg), lon (deg), alt (meters)]
    - alt is height above sea level, as opposed to center of earth

    """
    if xyz.ndim < 2:  # force 2D
        xyz = xyz.reshape(-1, 1)  # forces vector to a column if 1D

    x = xyz[0, :]
    y = xyz[1, :]
    z = xyz[2, :]

    f = (a - b) / a

    e_sq = f * (2 - f)
    eps = e_sq / (1.0 - e_sq)

    p = np.sqrt(np.multiply(x, x) + np.multiply(y, y))
    q = np.arctan2((z * a), (p * b))
    phi = np.arctan2((z + eps * b * np.sin(q) ** 3), (p - e_sq * a * np.cos(q) ** 3))
    lam = np.arctan2(y, x)

    v = a / np.sqrt(1.0 - e_sq * np.sin(phi) * np.sin(phi))
    h = (p / np.cos(phi)) - v

    lat = np.degrees(phi)
    lon = np.degrees(lam)

    return np.vstack([lat, lon, h])


def lla2ecef(lla):
    """
    Converts xyz position to GPS position

    Args:
    - lla: 3-dimensional GPS lla position [lat (deg), lon (deg), alt (meters)]
    - atti is height above sea level, as opposed to center of earth

    Returns:
    - xyz: 3-dimensional ECEF xyz position [x, y, z]
    """

    lat, lon, h = lla

    f = (a - b) / a

    e_sq = f * (2 - f)
    eps = e_sq / (1.0 - e_sq)

    # Convert latitude and longitude to radians
    phi = np.radians(lat)
    lam = np.radians(lon)

    # Compute geocentric latitude
    sin_phi = np.sin(phi)
    cos_phi = np.cos(phi)
    N = a / np.sqrt(1 - e_sq * sin_phi ** 2)
    x = (N + h) * cos_phi * np.cos(lam)
    y = (N + h) * cos_phi * np.sin(lam)
    z = (N * (1 - e_sq) + h) * sin_phi

    return np.array([x, y, z])


# alt2pres
#
# determine pressure from altitude

def alt2pres(altitude):
    """
    Determine site pressure from altitude.

    Parameters
    ----------
    altitude : scalar or Series
        Altitude in meters above sea level

    Returns
    -------
    Pressure : scalar or Series
        Atmospheric pressure (Pascals)

    Notes
    ------
    The following assumptions are made

    ============================   ================
    Parameter                      Value
    ============================   ================
    Base pressure                  101325 Pa
    Temperature at zero altitude   288.15 K
    Gravitational acceleration     9.80665 m/s^2
    Lapse rate                     -6.5E-3 K/m
    Gas constant for air           287.053 J/(kgK)
    Relative Humidity              0%
    ============================   ================

    References
    -----------

    1. "A Quick Derivation relating altitude to air pressure" from Portland
    State Aerospace Society, Version 1.03, 12/22/2004.

    2. https://pvlib-python.readthedocs.io/en/v0.2.2/_modules/pvlib/atmosphere.html
    """

    press = 100 * ((44331.514 - altitude) / 11880.516) ** (1 / 0.1902632)

    return press


def xyz2grav(x, y, z):
    """
    Ellipsoid Earth gravity model

    Args:
    - x, y, z: three-dimensional ECEF position
    
    Returns:
    - g: gravity vector [gx, gy, gz]
    """

    j2 = 0.00108263
    mu = 3.986004418e14
    R = 6378137
    r = np.sqrt(x ** 2 + y ** 2 + z ** 2)
    sub1 = 1.5 * j2 * ((R / r) ** 2)
    sub2 = (5 * z ** 2) / (r ** 2)
    sub3 = (-mu) / (r ** 3)
    sub4 = sub3 * (1 - sub1 * (sub2 - 1))
    gx = x * (sub4)
    gy = y * (sub4)
    gz = z * (sub3) * (1 - sub1 * (sub2 - 3))
    g = np.array([gx, gy, gz])
    g = g[:]  # Force column
    return g


# grav_gradient
#
# Calculate the 3x3 gradient of gravity
def grav_gradient(r_ecef, eps=1e-6):
    """
    Ellipsoid Earth gravity model

    Args:
    - r_ecef: 3-D ECEF position, [x, y, z]
    
    Returns:
    - gradient: 3-D gravity gradient [gx, gy, gz]
    """

    x, y, z = r_ecef

    # Initialize the gradient vector
    gradient = np.zeros((3, 3))

    # Compute the partial derivatives using finite differences
    gradient[:, 0] = (xyz2grav(x + eps, y, z) - xyz2grav(x - eps, y, z)) / (2 * eps)
    gradient[:, 1] = (xyz2grav(x, y + eps, z) - xyz2grav(x, y - eps, z)) / (2 * eps)
    gradient[:, 2] = (xyz2grav(x, y, z + eps) - xyz2grav(x, y, z - eps)) / (2 * eps)

    return gradient


# lla2quat
#
# Compute the quaternion of a body pointing straight upwards
# Used to initialize the rocket's quaternion
def lla2quat(lla):
    """
    Convert GPS position to a quaternion of a body pointing straight up

    Args:
    - lat, lon, atti: 3 GPS coordinates
    
    Returns:
    - quat: 4-dimensional quaternion, [qs, qi, qj, qk]
    """
    
    lat, lon, alt = lla

    # Convert LLA coordinates to ECEF coordinates
    x, y, z = lla2ecef(lat, lon, alt)

    # Calculate the gravitational acceleration vector in the ECEF frame
    grav = xyz2grav(x, y, z)

    # Assume that the body is pointing straight away from the gravity vector
    # Body-frame x-axis is aligned with negative gravity vector in ECEF frame
    body_x = -grav / np.linalg.norm(grav)
    ecef_x = np.array([1, 0, 0])  # ECEF x-axis

    # Calculate the rotation axis and angle between ECEF x-axis and body x-axis
    axis = np.cross(ecef_x, body_x)
    angle = np.arccos(np.dot(ecef_x, body_x))

    # Construct the quaternion that rotates ECEF x-axis to body x-axis
    s = np.cos(angle / 2)
    v = np.sin(angle / 2) * axis
    quat = np.array([s, v[0], v[1], v[2]])

    return quat


# lla_jacobian
#
# Computes the Jacobian of (lat, lon, alt) in [rad, rad, m (HAE)] w.r.t ECEF position in [meters]
# Used to implement our "h" function for Kalman Filtering
# Credit: Tyler Klein
def lla_jacobian(r_ecef, HAE=True):
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


if __name__ == "__main__":
    print("MAIN")

    """
    file_data = pd.read_csv("../Data Generation/traj_raster_30mins_20221115_160156.csv").to_numpy()
    
    gps_data, dt = dc.get_next_gps_reading()
    first_quat = file_data[0, 7:11]
    
    test_quat = lla2quat(gps_data[0], gps_data[1], gps_data[2])
    
    print(first_quat, test_quat)
    """

    """
    xyz = [6000000, 6100000, 6200000]
    lla = ecef2lla(xyz)
    print(lla)
    back = lla2ecef(lla)
    print(xyz,back)
    """
