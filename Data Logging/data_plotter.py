#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 16 15:01:54 2023

@author: zrummler
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from misc import plotDataAndError
from data_logging_wrapper import DataLogger

class DataPlotter(DataLogger):
    
    def __init__(self, num_points):
        
        self.PVA_est = np.zeros((10, num_points))
        
        data = pd.read_csv("../Test Data/traj_raster_30mins_20221115_160156.csv").to_numpy()
        self.PVA_truth = data[:, 1:11].T
        
        self.write_pos = 0
        
        
    def save(self, state_vector, q_truth):
        
        self.PVA_est[:6, self.write_pos] = state_vector[:6]  # store ECEF position and velocity
        self.PVA_est[6:10, self.write_pos] = q_truth
        
        self.write_pos += 1
        
    def show(self):
        
        tplot = np.arange(self.PVA_est.shape[1])
        plotDataAndError(self.PVA_est[6:10, ::10], self.PVA_truth[6:10, ::10], tplot[::10], axes=['A', 'B', 'C', 'D'], name='Quaternion', unit=None)
        plotDataAndError(self.PVA_est[3:6, :], self.PVA_truth[3:6, :], tplot, name='Velocity', subx0=True)
        plotDataAndError(self.PVA_est[:3, :], self.PVA_truth[:3, :], tplot, subx0=True)
        
        plt.show()  # needed to display the figures