"""
Created on Thu Mar 16 15:16:51 2023

@author: zrummler

If num_points is None, then 

"""

import time
import datetime
import numpy as np
import matplotlib.pyplot as plt

colnames = [
    "t [sec]",
    "r_ecef_X",
    "r_ecef_Y",
    "r_ecef_Z",
    "v_ecef_X",
    "v_ecef_Y",
    "v_ecef_Z",
    "q_e2b_scalar",
    "q_e2b_i",
    "q_e2b_j",
    "q_e2b_k",
    "accel_X [m/s^2]",
    "accel_Y [m/s^2]",
    "accel_Z [m/s^2]",
    "gyro_X [rad/s]",
    "gyro_Y [rad/s]",
    "gyro_Z [rad/s]",
    "lat [deg]",
    "long [deg]",
    "alt [m]"]

class DataLogger:
        
    def __init__(self, initial_state_vector, initial_quaternion, num_points=None):
        """
        initializes the DataLogger class
        
        Arguments:
            initial_state_vector: array (10,1) of the form [pos, vel, atti_error]
            initial_quaternion: array (4,1)
            colnames: array (11,1) column names for the log file
            num_points: int (optional) number of rows of the data file
        """
        width = len(colnames)
        
        if num_points is not None:   
            self.PVA_est = np.zeros((width, num_points+1)) # save room for columns
        else:
            big_number = 18000 # TODO
            self.PVA_est = np.zeros((width, big_number))
        
        
        # write initial conditions
        self.PVA_est[1:7, 0] = initial_state_vector[:6]
        self.PVA_est[7:11, 0] = initial_quaternion
        
        # first position to write to
        self.write_pos = 1
           
        
    def start_timer(self):
        """
        Starts the timer. Must call this right before you start saving data
        
        No arguments, no return value
        """
        
        self.t_initial = time.perf_counter()
        self.PVA_est[0, 1] = 0
            
    def save_state_to_buffer(self, state_vector, quaternion, imu_reading, gps_reading):
        """
        Saves one line to a buffer
        
        Arguments:
            state_vector: array (10,1)
            quaternion: array (4,1)
            
        Returns:
            none
        """
        
        self.PVA_est[0, self.write_pos] = time.perf_counter() - self.t_initial
        self.PVA_est[1:7, self.write_pos] = state_vector[:6].flatten()  # store position and velocity only
        self.PVA_est[7:11, self.write_pos] = quaternion.flatten() # store the quaternion
        self.PVA_est[11:17, self.write_pos] = imu_reading
        self.PVA_est[17:, self.write_pos] = gps_reading
    
        self.write_pos += 1     
        
        
    def write_buffer_to_file(self):
        """
        Writes the buffer to a .csv file. Must call this after you finish saving data
        
        Returns:
            filename: (str) string of the file
        """
        
        # create a formatted filename string with the date and time
        now = datetime.datetime.now()
        filename = now.strftime("data_%Y-%m-%d_%H-%M-%S.csv")
        
        # write the data to the file
        col_str = ','.join(colnames)
        np.savetxt(filename, self.PVA_est.T, delimiter=',', fmt='%s', header=col_str)
        
        return filename
        
    
    def print_position_drift(self):
        """
        Reads the log file and plots the data in the form PVA. Helpful for debugging
        """
        
        num_points = self.PVA_est.shape[1]
        
        print("POSITION DRIFT:")
        
        for i in range(num_points):
            curr_time = self.PVA_est[0,i]
            drift_xyz = self.PVA_est[1:4,i] - self.PVA_est[1:4,0]
            
            print(curr_time, drift_xyz)
 
"""
The following files are separate from the DataLogger class but are helpful for 
viewing the data after it has been written to a file
"""

def read_file_to_buffer(filename):
    """
    Reads the file into a numpy array
    
    Parameters:
        filename: (str) name of file, returned by DataLogger.write_buffer_to_file()
    """
    
    PVA_est = np.loadtxt(filename, delimiter=',', dtype=str).T
    PVA_est = PVA_est.astype(float)
    
    return PVA_est
        
def plot_file_contents(filename):
    """
    Plots the position, velocity, and attitude contents from the file
    
    Parameters:
        filename: (str) name of file, returned by DataLogger.write_buffer_to_file()
    """
    
    PVA_est = read_file_to_buffer(filename)
    
    tplot = np.arange(PVA_est.shape[1])
    _plotData(PVA_est[7:11, ::10], tplot[::10], axes=['A', 'B', 'C', 'D'], name='Quaternion', unit=None)
    _plotData(PVA_est[4:7, :], tplot, name='Velocity', subx0=True)
    _plotData(PVA_est[1:4, :], tplot, subx0=True)
    
    plt.show()  # needed to display the figures
 
def print_file_contents(filename):
    """
    Prints the entire scontents from the file
    
    Parameters:
        filename: (str) name of file, returned by DataLogger.write_buffer_to_file()
    """
    
    PVA_est = read_file_to_buffer(filename)
    num_points = PVA_est.shape[1]
    
    print("POSITION:")
    
    for i in range(num_points):
        print(PVA_est[0, i], PVA_est[1:4, i])
        
        
     
def _plotData(x, t, cov=None, name='Position', unit='m', subx0=False, decim_fact=1, tEnd=None, axes=None):
    """
    Private helper function which plots everything very nicely
    Credit: Tyler Klein

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