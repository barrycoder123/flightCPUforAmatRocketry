#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 16 15:01:54 2023

@author: zrummler
"""

import numpy as np
import matplotlib.pyplot as plt
from data_logging_wrapper import DataLogger

class DataPlotter(DataLogger):
    
    def __init__(self, num_points):
        
        self.PVA_est = np.zeros((10, num_points))
        
        self.write_pos = 0
        
        
    def save(self, state_vector, q_truth):
        
        self.PVA_est[:6, self.write_pos] = state_vector[:6]  # store ECEF position and velocity
        self.PVA_est[6:10, self.write_pos] = q_truth
        
        self.write_pos += 1
        
    def show(self):
        
        tplot = np.arange(self.PVA_est.shape[1])
        self.__plotData(self.PVA_est[6:10, ::10], tplot[::10], axes=['A', 'B', 'C', 'D'], name='Quaternion', unit=None)
        self.__plotData(self.PVA_est[3:6, :], tplot, name='Velocity', subx0=True)
        self.__plotData(self.PVA_est[:3, :], tplot, subx0=True)
        
        plt.show()  # needed to display the figures
        
    def __plotData(self, x, t, cov=None, name='Position', unit='m', subx0=False, decim_fact=1, tEnd=None, axes=None):
        """
        TODO

        Parameters
        ----------
        x : (M,N) ndarray
            matrix of measured values, where ``x[:,i]`` is the vector of measured quantities at time `i`
        x_true : (M,N) ndarray
            matrix of truth values, where ``x_true[:,i]`` is the vector of true values at time `i`. Same shape as ``x``
        t : (N,) ndarray
            time vector [seconds]
        cov : (M, M, N) ndarray
            if provided, specifies the covariance for the data. This will be plotted in the error graphs
        name : str, default='Position'
            plot name to appear in titles and axis labels
        unit : str, default='m'
            unit string for axis labels
        subx0 : bool, default=False
            if True, the true value at the first time will be subtracted from both vectors. This is useful when plotting quantities with a large offset,
            such as ECEF position
        decim_fact : int, default=1
            decimation factor for plotting
        tEnd : float
            if provided, the xlim will be set to end at this time
        axes : list
            axis names. Default is ['x', 'y', 'z']. Length match x.shape[0]
        """
        
        
        if axes is None:
            axes = ['X', 'Y', 'Z']

        if len(axes) != x.shape[0]:
            raise ValueError('len(axes) != x.shape[0]')

        x0_use = x[:, 0] if subx0 else np.zeros((len(axes),))

        fig, axs = plt.subplots(len(axes), 1, sharex=True, figsize=(13, 8))
        plt.suptitle(name, fontsize='xx-large', fontweight='black')

        tplot = t[::decim_fact]
        for i, row in enumerate(axs):
            data = x[i, ::decim_fact]
            row.plot(tplot, data - x0_use[i])
            row.set_xlabel('Time [sec]')
            row.set_ylabel(f'[{unit}]')
            row.set_title(f"{axes[i]}")
            row.grid(True)


            if tEnd is not None:
                row.set_xlim([0, tEnd])

        plt.tight_layout()
        return fig, axs