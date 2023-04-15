"""
Created on Thu Mar 16 15:16:51 2023

@author: zrummler

If num_points is None, then 

"""

import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class DataLogger:
        
    def __init__(self, initial_state_vector, initial_quaternion, colnames, num_points=None):
        """
        initializes the DataLogger class
        
        Arguments:
            initial_state_vector: array (10,1) of the form [pos, vel, atti_error]
            initial_quaternion: array (4,1)
            colnames: array (11,1) column names for the log file
            num_points: int (optional) number of rows of the data file
        """
        
        if num_points is not None:   
            self.PVA_est = np.zeros((11, num_points+1))
        else:
            big_number = 18000 # TODO
            self.PVA_est = np.zeros((11, big_number))
            
        self.colnames = colnames   
        
        self.PVA_est[1:7, 0] = initial_state_vector[:6]
        self.PVA_est[7:11, 0] = initial_quaternion
        
        self.write_pos = 1  
           
        
    def start_timer(self):
        """
        Starts the timer. Must call this right before you start saving data
        
        No arguments, no return value
        """
        
        self.t_initial = time.perf_counter()
        self.PVA_est[0, 0] = 0
            
    def save_state_to_buffer(self, state_vector, quaternion):
        """
        Saves one line to a buffer
        
        Arguments:
            state_vector: array (10,1)
            quaternion: array (4,1)
            
        Returns:
            none
        """
        state_vector.flatten()
        quaternion.flatten()
        
        self.PVA_est[0, self.write_pos] = time.perf_counter() - self.t_initial
        self.PVA_est[1:7, self.write_pos] = state_vector[:6]  # store position and velocity only
        self.PVA_est[7:11, self.write_pos] = quaternion # store the quaternion
    
        self.write_pos += 1
        
    def print_buffer_contents(self):
        
        num_points = self.PVA_est.shape[1]
        
        print("POSITION DRIFT:")
        
        for i in range(num_points):
            curr_time = self.PVA_est[0,i]
            drift_xyz = self.PVA_est[1:4,i] - self.PVA_est[1:4,0]
            
            print(curr_time, drift_xyz)
        

    def write_buffer_to_file(self):
        """
        Writes the buffer to a .csv file. Must call this after you finish saving data
        """
        
        df_new = pd.DataFrame(self.PVA_est.T, columns=self.colnames)
        df_new.to_csv("PVA_est.csv", index=False)
        
    def plot_file_contents(self):
        """
        Reads the log file and plots the data in the form PVA. Helpful for debugging
        """
        
        df = pd.read_csv("PVA_est.csv")
        PVA_est = df.to_numpy().T
        
        tplot = np.arange(PVA_est.shape[1])
        self.__plotData(PVA_est[7:11, ::10], tplot[::10], axes=['A', 'B', 'C', 'D'], name='Quaternion', unit=None)
        self.__plotData(PVA_est[4:7, :], tplot, name='Velocity', subx0=True)
        self.__plotData(PVA_est[1:4, :], tplot, subx0=True)
        
        plt.show()  # needed to display the figures
     
    def print_file_contents(self):
        
        df = pd.read_csv("PVA_est.csv")
        PVA_est = df.to_numpy().T
        num_points = PVA_est.shape[1]
        
        print("POSITION:")
        
        for i in range(num_points):
            print(PVA_est[0, i], PVA_est[1:4, i])
        
        
     
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
        
    
    
