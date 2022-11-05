# pylint: disable=invalid-name
"""Import and plot flux maps from the SyR-e project."""

# %%
import numpy as np
import matplotlib.pyplot as plt
from cycler import cycler
from scipy.interpolate import griddata
from scipy.io import loadmat
from motulator.helpers import Bunch

# Plotting parameters
plt.rcParams['axes.prop_cycle'] = cycler(color='brgcmyk')
plt.rcParams['lines.linewidth'] = 1.
plt.rcParams['axes.grid'] = True
plt.rcParams.update({"text.usetex": False})


# %%
def import_syre_data(fname='THOR.mat'):
    """
    Import a flux map from the MATLAB data file in the SyR-e format.

    For more information on the SyR-e project and the MATLAB file format,
    please visit:

        https://github.com/SyR-e/syre_public

    The default data is for a 7.5-kW PM-SyRM, know as THOR. The imported data
    is converted to the PMSM coordinate convention, in which the PM flux is
    along the d axis.

    Parameters
    ----------
    fname : str, optional
        MATLAB file name. The default is 'THOR.mat'.

    Returns
    -------
    data : Bunch
        Flux map data in rotor (dq) coordinates.

    Notes
    -----
    The THOR data has been licensed under the Apache License, Version 2.0. We
    thank the developers of SyR-e for giving a permission to include this data
    file as a part of the motulator repository.

    """
    # Read the data from mat-file
    data = loadmat(fname)

    # Use the PMSM convention in coordinates
    i_d = -data['motorModel'][0, 0]["FluxMap_dq"][0, 0]["Iq"]
    i_q = data['motorModel'][0, 0]["FluxMap_dq"][0, 0]["Id"]
    i_s = i_d + 1j*i_q
    psi_d = -data['motorModel'][0, 0]["FluxMap_dq"][0, 0]["Fq"]
    psi_q = data['motorModel'][0, 0]["FluxMap_dq"][0, 0]["Fd"]
    psi_s = psi_d + 1j*psi_q
    tau_M = data['motorModel'][0, 0]["FluxMap_dq"][0, 0]["T"]

    data = Bunch(i_s=i_s, psi_s=psi_s, tau_M=tau_M)
    return data


# %%
def plot_flux_map(data):
    """
    Plot the flux linkage as function of the current.

    Parameters
    ----------
    data : Bunch
        Flux map data.

    """
    fig = plt.figure(figsize=(10, 5))
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    ax2 = fig.add_subplot(1, 2, 2, projection='3d')
    ax1.plot_surface(data.i_s.real, data.i_s.imag, data.psi_s.real)
    ax2.plot_surface(data.i_s.real, data.i_s.imag, data.psi_s.imag)
    ax1.set_xlabel(r'$i_\mathrm{d}$ (A)')
    ax1.set_ylabel(r'$i_\mathrm{q}$ (A)')
    ax1.set_zlabel(r'$\psi_\mathrm{d}$ (Vs)')
    ax2.set_xlabel(r'$i_\mathrm{d}$ (A)')
    ax2.set_ylabel(r'$i_\mathrm{q}$ (A)')
    ax2.set_zlabel(r'$\psi_\mathrm{q}$ (Vs)')


# %%
def plot_torque_map(data):
    """
    Plot the torque as function of the current.

    Parameters
    ----------
    data : Bunch
        Flux map data.

    """
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.plot_surface(data.i_s.real, data.i_s.imag, data.tau_M)
    ax.set_xlabel(r'$i_\mathrm{d}$ (A)')
    ax.set_ylabel(r'$i_\mathrm{q}$ (A)')
    ax.set_zlabel(r'$\tau_\mathrm{M}$ (Nm)')


# %%
def plot_flux_vs_current(data):
    """
    Plot the flux vs. current characteristics.

    Parameters
    ----------
    data : Bunch
        Flux map data.

    """
    # Indices corresponding to i_d = 0 and i_q = 0
    ind_d_0 = np.argmin(np.abs(data.i_s.real[:, 0]))
    ind_q_0 = np.argmin(np.abs(data.i_s.imag[0, :]))
    # Indices corresponding to min(i_d) and max(i_d)
    ind_d_min = np.argmin(data.i_s.real[:, 0])
    ind_d_max = np.argmax(data.i_s.real[:, 0])

    fig = plt.figure()
    ax = fig.add_subplot()
    ax.plot(
        data.i_s.real[:, ind_q_0],
        data.psi_s.real[:, ind_q_0],
        color='r',
        linestyle='-',
        label=r'$\psi_\mathrm{d}(i_\mathrm{d}, 0)$')
    ax.plot(
        data.i_s.real[:, -1],
        data.psi_s.real[:, -1],
        color='r',
        linestyle='--',
        label=r'$\psi_\mathrm{d}(i_\mathrm{d}, i_\mathrm{q,max})$')
    ax.plot(
        data.i_s.imag[ind_d_0, :],
        data.psi_s.imag[ind_d_0, :],
        color='b',
        linestyle='-',
        label=r'$\psi_\mathrm{q}(0, i_\mathrm{q})$')
    ax.plot(
        data.i_s.imag[ind_d_min, :],
        data.psi_s.imag[ind_d_min, :],
        color='b',
        linestyle=':',
        label=r'$\psi_\mathrm{q}(i_\mathrm{d,min}, i_\mathrm{q})$')
    ax.plot(
        data.i_s.imag[ind_d_max, :],
        data.psi_s.imag[ind_d_max, :],
        color='b',
        linestyle='--',
        label=r'$\psi_\mathrm{q}(i_\mathrm{d,max}, i_\mathrm{q})$')

    ax.set_xlabel(r'$i_\mathrm{d}$, $i_\mathrm{q}$ (A)')
    ax.set_ylabel(r'$\psi_\mathrm{d}$, $\psi_\mathrm{q}$ (Vs)')
    ax.legend()


# %%
def downsample_flux_map(data, N_d=32, N_q=32):
    """
    Downsample the flux map by means of linear interpolation.

    Parameters
    ----------
    data : Bunch
        Flux map data.
    N_d : int, optional
        Number of interpolated samples in the d axis. The default is 32.
    N_q : int, optional
        Number of interpolated samples in the q axis. The default is 32.

    Returns
    -------
    Bunch object with the following fields defined:
    i_s : complex ndarray, shape (N_d, N_q)
        Stator current data.
    psi_s : complex ndarray, shape (N_d, N_q)
        Stator flux linkage data.
    tau_M : ndarray, shape (N_d, N_q)

    """
    # Get the range
    i_d_max = np.max(data.i_s.real)
    i_d_min = np.min(data.i_s.real)
    i_q_max = np.max(data.i_s.imag)
    i_q_min = np.min(data.i_s.imag)

    # Points at which to interpolate data
    i_d, i_q = np.mgrid[i_d_min:i_d_max:1j*N_d, i_q_min:i_q_max:1j*N_q]

    # Interpolate data
    points = (np.ravel(data.i_s.real), np.ravel(data.i_s.imag))
    psi_s = griddata(points, np.ravel(data.psi_s), (i_d, i_q), method='linear')
    tau_M = griddata(points, np.ravel(data.tau_M), (i_d, i_q), method='linear')
    i_s = i_d + 1j*i_q

    downsampled_data = Bunch(
        i_s=i_s,
        psi_s=psi_s,
        tau_M=tau_M,
    )
    return downsampled_data


# %%
def invert_flux_map(data, N_d=32, N_q=32):
    """
    Compute the inverse flux map by means of linear interpolation.

    Parameters
    ----------
    data : Bunch
        Flux map data.
    N_d : int, optional
        Number of interpolated samples in the d axis. The default is 32.
    N_q : int, optional
        Number of interpolated samples in the q axis. The default is 32.

    Returns
    -------
    Bunch object with the following fields defined:
    psi_s : complex ndarray, shape (N_d, N_q)
        Stator flux linkage data.
    i_s : complex ndarray, shape (N_d, N_q)
        Stator current data.
    tau_M : ndarray, shape (N_d, N_q)

    """
    # Get the range
    psi_d_max = np.min(np.max(data.psi_s.real, axis=0))
    psi_d_min = np.max(np.min(data.psi_s.real, axis=0))
    psi_q_max = np.min(np.max(data.psi_s.imag, axis=1))
    psi_q_min = np.max(np.min(data.psi_s.imag, axis=1))

    # Points at which to interpolate data
    psi_d, psi_q = np.mgrid[psi_d_min:psi_d_max:1j*N_d,
                            psi_q_min:psi_q_max:1j*N_q]

    # Interpolate data
    points = (np.ravel(data.psi_s.real), np.ravel(data.psi_s.imag))
    i_s = griddata(points, np.ravel(data.i_s), (psi_d, psi_q), method='linear')
    tau_M = griddata(
        points, np.ravel(data.tau_M), (psi_d, psi_q), method='linear')
    psi_s = psi_d + 1j*psi_q

    inv_data = Bunch(
        psi_s=psi_s,
        i_s=i_s,
        tau_M=tau_M,
    )
    return inv_data


# %%
if __name__ == '__main__':
    motor_data = import_syre_data(fname='THOR.mat')
    # motor_data = downsample_flux_map(data)
    plot_flux_vs_current(motor_data)
    plot_flux_map(motor_data)
