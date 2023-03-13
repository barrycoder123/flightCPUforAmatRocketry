import sys
import os
from matplotlib import pyplot as plt
import numpy as np
import time


def get_imu_column_names():
    """
    Gets the column names to be used for IMU/sensor .csv files

    Returns
    -------
    t_col : str
        time column name
    accel_cols : list
        list of [x,y,z] acceleration column names
    gyro_cols : list
        list of [x,y,z] angular rate column names
    mag_cols : list
        list of [x,y,z] magnetic field column names
    """

    t_col = 't [sec]'
    accel_cols = [f'accel_{axis} [m/s^2]' for axis in ['X', 'Y', 'Z']]
    gyro_cols = [f'gyro_{axis} [rad/s]' for axis in ['X', 'Y', 'Z']]
    mag_cols = [f'mag_{axis} [uT]' for axis in ['X', 'Y', 'Z']]
    return t_col, accel_cols, gyro_cols, mag_cols


def get_ecef_column_names():
    """
    Gets the column names to be used for trajectory .csv files

    Returns
    -------
    pos_cols : list
        list of [x,y,z] ecef position column names
    vel_cols : list
        list of [x,y,z] ecef velocity column names
    quat_cols : list
        list of [scalar, i, j, k] ECEF-to-Body Quaternion column names
    """

    pos_cols = [f'r_ecef_{v}' for v in ['X', 'Y', 'Z']]  # position
    vel_cols = [f'v_ecef_{v}' for v in ['X', 'Y', 'Z']]  # velocity
    quat_cols = [f'q_e2b_{v}' for v in ['scalar', 'i', 'j', 'k']]  # quaternion
    return pos_cols, vel_cols, quat_cols


def plotDataAndError(x, x_true, t, cov=None, name='Position', unit='m', subx0=False, decim_fact=1, tEnd=None, axes=None):
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

    fig, axs = plt.subplots(len(axes), 2, sharex=True, figsize=(13, 8))
    plt.suptitle(name, fontsize='xx-large', fontweight='black')

    tplot = t[::decim_fact]
    for i, row in enumerate(axs):
        data = x[i, ::decim_fact]
        truth = x_true[i, ::decim_fact]
        row[0].plot(tplot, data - x0_use[i])
        row[0].plot(tplot, truth - x0_use[i])
        row[0].set_xlabel('Time [sec]')
        row[0].set_ylabel(f'[{unit}]')
        row[0].set_title(f"{axes[i]}")
        row[0].grid(True)

        row[1].plot(tplot, data - truth)
        if cov is not None:
            this_std = np.sqrt(cov[i, i, ::decim_fact])  # pull out the standard deviation along the current axis
            row[1].plot(tplot, 3 * this_std, color='red')
            row[1].plot(tplot, -3 * this_std, color='red')
        row[1].set_xlabel('Time [sec]')
        row[1].set_ylabel(f'Error [{unit}]')
        row[1].set_title(f"{axes[i]} Error")
        row[1].grid(True)

        if tEnd is not None:
            row[0].set_xlim([0, tEnd])
            row[1].set_xlim([0, tEnd])

    plt.tight_layout()
    return fig, axs
