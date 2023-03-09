#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar  9 11:02:54 2023

@author: zrummler
"""

import sys
import numpy as np

sys.path.append('../Flight Algorithms')

import earth_model as em

def test_ecef2lla():
    
    # Test conversion for random input values
    for i in range(10):
        # Generate random ECEF coordinates
        x = np.random.rand() * 10000 - 5000
        y = np.random.rand() * 10000 - 5000
        z = np.random.rand() * 10000 - 5000
        xyz = np.array([x, y, z])

        # Convert ECEF coordinates to GPS coordinates
        lla = em.ecef2lla(xyz)

        # Convert GPS coordinates back to ECEF coordinates
        xyz2 = em.lla2ecef(lla)

        # Check that the original ECEF coordinates and the converted ECEF coordinates are equal
        print(xyz, xyz2)
        assert np.allclose(xyz, xyz2)

        # Check that the converted GPS coordinates are within a tolerance of 1e-6 of the original GPS coordinates
        #assert_array_almost_equal(xyz, lla2ecef(ecef2lla(xyz)))
    
def test_ecef_lla_conversions():
   
    # Test conversion in both directions for a set of known coordinates
    coordinates = [
        # lat, lon, alt
        (42.3465, -71.0975, 10),  # Boston, MA
        (37.7749, -122.4194, 100),  # San Francisco, CA
        (35.6895, 139.6917, 500),  # Tokyo, Japan
        (-33.8568, 151.2153, 1000),  # Sydney, Australia
        (51.5074, -0.1278, 200),  # London, UK
    ]
    for lat, lon, alt in coordinates:
        lla = np.array([lat, lon, alt])
        ecef = em.lla2ecef(lla)
        lla2 = em.ecef2lla(ecef)
        #print(lla, lla2)
        # Check that the output matches the input, within a small tolerance
        assert np.allclose(lla, lla2, atol=1e-6), f"Failed for coordinates: {lla}"
        # Check that the ECEF output is finite
        assert np.isfinite(ecef).all(), f"Invalid ECEF output for coordinates: {lla}"
        # Check that the LLA output is finite
        assert np.isfinite(lla2).all(), f"Invalid LLA output for coordinates: {ecef}"


if __name__ == "__main__":
    
    test_ecef_lla_conversions()
    #test_ecef2lla()