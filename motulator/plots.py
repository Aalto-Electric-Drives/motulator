# pylint: disable=invalid-name
"""Example plotting scripts."""

# %%
import numpy as np
import matplotlib.pyplot as plt
from cycler import cycler

from motulator.helpers import Bunch, complex2abc

# Plotting parameters
plt.rcParams['axes.prop_cycle'] = cycler(color='brgcmyk')
plt.rcParams['lines.linewidth'] = 1.
plt.rcParams['axes.grid'] = True
plt.rcParams.update({"text.usetex": False})


# %%
def plot(sim, t_span=None, base=None):
    """
    Plot example figures.

    Plots figures in per-unit values, if the base values are given. Otherwise
    SI units are used.

    Parameters
    ----------
    sim : Simulation object
        Should contain the simulated data.
    t_span : 2-tuple, optional
        Time span. The default is (0, sim.ctrl.t[-1]).
    base : BaseValues, optional
        Base values for scaling the waveforms.

    """
    # pylint: disable=too-many-statements
    mdl = sim.mdl.data  # Continuous-time data
    ctrl = sim.ctrl.data  # Discrete-time data

    # Check if the time span was given
    if t_span is None:
        t_span = (0, ctrl.t[-1])

    # Check if the base values were given
    if base is None:
        pu_vals = False
        base = Bunch(w=1, u=1, i=1, psi=1, tau=1)  # Unity base values
    else:
        pu_vals = True

    # Recognize the motor type by checking if the rotor flux data exist
    try:
        if mdl.psi_Rs is not None:
            motor_type = 'im'
    except AttributeError:
        motor_type = 'sm'

    fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, 1, figsize=(8, 10))

    # Subplot 1: angular speeds
    ax1.step(ctrl.t, ctrl.w_m_ref/base.w, '--', where='post')
    ax1.plot(mdl.t, mdl.w_m/base.w)
    try:
        ax1.step(ctrl.t, ctrl.w_m/base.w, where='post')
    except AttributeError:
        pass
    ax1.legend([
        r'$\omega_\mathrm{m,ref}$',
        r'$\omega_\mathrm{m}$',
        r'$\hat \omega_\mathrm{m}$',
    ])
    ax1.set_xlim(t_span)
    ax1.set_xticklabels([])

    # Subplot 2: torques
    ax2.plot(mdl.t, mdl.tau_L/base.tau, '--')
    ax2.plot(mdl.t, mdl.tau_M/base.tau)
    try:
        ax2.step(ctrl.t, ctrl.tau_M_ref_lim/base.tau)  # Limited torque ref
    except AttributeError:
        pass
    ax2.legend([
        r'$\tau_\mathrm{L}$',
        r'$\tau_\mathrm{M}$',
        r'$\tau_\mathrm{M,ref}$',
    ])
    ax2.set_xlim(t_span)
    ax2.set_xticklabels([])

    # Subplot 3: currents
    ax3.step(ctrl.t, ctrl.i_s.real/base.i, where='post')
    ax3.step(ctrl.t, ctrl.i_s.imag/base.i, where='post')
    try:
        ax3.step(ctrl.t, ctrl.i_s_ref.real/base.i, '--', where='post')
        ax3.step(ctrl.t, ctrl.i_s_ref.imag/base.i, '--', where='post')
    except AttributeError:
        pass
    ax3.legend([
        r'$i_\mathrm{sd}$',
        r'$i_\mathrm{sq}$',
        r'$i_\mathrm{sd,ref}$',
        r'$i_\mathrm{sq,ref}$',
    ])
    ax3.set_xlim(t_span)
    ax3.set_xticklabels([])

    # Subplot 4: voltages
    ax4.step(ctrl.t, np.abs(ctrl.u_s)/base.u, where='post')
    ax4.step(ctrl.t, ctrl.u_dc/np.sqrt(3)/base.u, '--', where='post')
    ax4.legend([r'$u_\mathrm{s}$', r'$u_\mathrm{dc}/\sqrt{3}$'])
    ax4.set_xlim(t_span)
    ax4.set_xticklabels([])

    # Subplot 5: flux linkages
    if motor_type == 'sm':
        ax5.plot(mdl.t, np.abs(mdl.psi_s)/base.psi)
        try:
            ax5.step(ctrl.t, np.abs(ctrl.psi_s)/base.psi, where='post')
        except AttributeError:
            pass
        ax5.legend([r'$\psi_\mathrm{s}$', r'$\hat\psi_\mathrm{s}$'])
    else:
        ax5.plot(mdl.t, np.abs(mdl.psi_ss)/base.psi)
        ax5.plot(mdl.t, np.abs(mdl.psi_Rs)/base.psi)
        try:
            ax5.plot(ctrl.t, np.abs(ctrl.psi_s)/base.psi)
        except AttributeError:
            pass
        ax5.legend([
            r'$\psi_\mathrm{s}$',
            r'$\psi_\mathrm{R}$',
            r'$\hat \psi_\mathrm{s}$',
        ])
    ax5.set_xlim(t_span)

    # Add axis labels
    if pu_vals:
        ax1.set_ylabel('Speed (p.u.)')
        ax2.set_ylabel('Torque (p.u.)')
        ax3.set_ylabel('Current (p.u.)')
        ax4.set_ylabel('Voltage (p.u.)')
        ax5.set_ylabel('Flux linkage (p.u.)')
    else:
        ax1.set_ylabel('Speed (rad/s)')
        ax2.set_ylabel('Torque (Nm)')
        ax3.set_ylabel('Current (A)')
        ax4.set_ylabel('Voltage (V)')
        ax5.set_ylabel('Flux linkage (Vs)')
    ax5.set_xlabel('Time (s)')
    fig.align_ylabels()

    plt.tight_layout()
    plt.show()


# %%
def plot_extra(sim, t_span=(1.1, 1.125), base=None):
    """
    Plot extra waveforms for a motor drive with a diode bridge.

    Parameters
    ----------
    sim : Simulation object
        Should contain the simulated data.
    t_span : 2-tuple, optional
        Time span. The default is (1.1, 1.125).
    base : BaseValues, optional
        Base values for scaling the waveforms.

    """
    mdl = sim.mdl.data  # Continuous-time data
    ctrl = sim.ctrl.data  # Discrete-time data

    # Check if the base values were iven
    if base is not None:
        pu_vals = True
    else:
        pu_vals = False
        base = Bunch(w=1, u=1, i=1, psi=1, tau=1)  # Unity base values

    # Quantities in stator coordinates
    ctrl.u_ss = np.exp(1j*ctrl.theta_s)*ctrl.u_s
    ctrl.i_ss = np.exp(1j*ctrl.theta_s)*ctrl.i_s

    fig1, (ax1, ax2) = plt.subplots(2, 1)

    # Subplot 1: voltages
    ax1.plot(mdl.t, mdl.u_ss.real/base.u)
    ax1.plot(ctrl.t, ctrl.u_ss.real/base.u)
    ax1.set_xlim(t_span)
    ax1.legend([r'$u_\mathrm{sa}$', r'$\hat u_\mathrm{sa}$'])
    ax1.set_xticklabels([])

    # Subplot 2: currents
    ax2.plot(mdl.t, complex2abc(mdl.i_ss).T/base.i)
    ax2.step(ctrl.t, ctrl.i_ss.real/base.i, where='post')
    ax2.set_xlim(t_span)
    ax2.legend([r'$i_\mathrm{sa}$', r'$i_\mathrm{sb}$', r'$i_\mathrm{sc}$'])

    # Add axis labels
    if pu_vals:
        ax1.set_ylabel('Voltage (p.u.)')
        ax2.set_ylabel('Current (p.u.)')
    else:
        ax1.set_ylabel('Voltage (V)')
        ax2.set_ylabel('Current (A)')
    ax2.set_xlabel('Time (s)')
    fig1.align_ylabels()

    # Plots the DC bus and grid-side variables (if exist)
    try:
        mdl.i_L
    except AttributeError:
        mdl.i_L = None

    if mdl.i_L is not None:

        fig2, (ax1, ax2) = plt.subplots(2, 1)

        # Subplot 1: voltages
        ax1.plot(mdl.t, mdl.u_di/base.u)
        ax1.plot(mdl.t, mdl.u_dc/base.u)
        ax1.plot(mdl.t, complex2abc(mdl.u_g).T/base.u)
        ax1.legend(
            [r'$u_\mathrm{di}$', r'$u_\mathrm{dc}$', r'$u_\mathrm{ga}$'])
        ax1.set_xlim(t_span)
        ax1.set_xticklabels([])

        # Subplot 2: currents
        ax2.plot(mdl.t, mdl.i_L/base.i)
        ax2.plot(mdl.t, mdl.i_dc/base.i)
        ax2.plot(mdl.t, mdl.i_g.real/base.i)
        ax2.legend([r'$i_\mathrm{L}$', r'$i_\mathrm{dc}$', r'$i_\mathrm{ga}$'])
        ax2.set_xlim(t_span)

    # Add axis labels
    if pu_vals:
        ax1.set_ylabel('Voltage (p.u.)')
        ax2.set_ylabel('Current (p.u.)')
    else:
        ax1.set_ylabel('Voltage (V)')
        ax2.set_ylabel('Current (A)')
    ax2.set_xlabel('Time (s)')

    try:
        fig2.align_ylabels()
    except UnboundLocalError:
        pass

    plt.tight_layout()
    plt.show()

    
    
 # %%
 def nfft(sim,t_span):
    """
    Non-uniformed fast fourier transformer
    Reference: https://github.com/jakevdp/nfft.git

    Parameters
    ----------
    sim : Simulation object
        Should contain the simulated data.
    t_span : 2-tuple, optional
        Time span. The default is (2, 2.36).
    """
    mdl = sim.mdl.data  # Continuous-time data
    ctrl = sim.ctrl.data  # Discrete-time data

    # Check if the time span was given
    if t_span is None:
        t_span = (2, 2.36)

    # Quantities in stator coordinates
    mdl.u_ss = np.exp(1j*mdl.theta_s)*mdl.u_s
    mdl.i_ss = np.exp(1j*mdl.theta_s)*mdl.i_s

    from numpy.fft import fft, ifft, fftshift, ifftshift
    from scipy.sparse import csr_matrix
    def phi(x, n, m, sigma):
        b = (2 * sigma * m) / ((2 * sigma - 1) * np.pi)
        return np.exp(-(n * x) ** 2 / b) / np.sqrt(np.pi * b)

    def C_phi(m, sigma):
        return 4 * np.exp(-m * np.pi * (1 - 1. / (2 * sigma - 1)))

    def m_from_C_phi(C, sigma):
        return np.ceil(-np.log(0.25 * C) / (np.pi * (1 - 1 / (2 * sigma - 1))))

    def phi_hat(k, n, m, sigma):
        b = (2 * sigma * m) / ((2 * sigma - 1) * np.pi)
        return np.exp(-b * (np.pi * k / n) ** 2)
    
    def nfft(x, f, N, sigma=2, tol=1E-8):
       """Alg 3 from https://www-user.tu-chemnitz.de/~potts/paper/nfft3.pdf"""
        indx_dc = N
        N = 2 * N
        n = N * sigma  # size of oversampled grid
        m = m_from_C_phi(tol / N, sigma)

        # 1. Express f(x) in terms of basis functions phi
        shift_to_range = lambda x: -0.5 + (x + 0.5) % 1
        col_ind = np.floor(n * x[:, np.newaxis]).astype(int) + np.arange(-m, m)
        vals = phi(shift_to_range(x[:, None] - col_ind / n), n, m, sigma)
        col_ind = (col_ind + n // 2) % n
        indptr = np.arange(len(x) + 1) * col_ind.shape[1]
        mat = csr_matrix((vals.ravel(), col_ind.ravel(), indptr), shape=(len(x), n))
        g = mat.T.dot(f)

        # 2. Compute the Fourier transform of g on the oversampled grid
        k = -(N // 2) + np.arange(N)
        g_k_n = fftshift(ifft(ifftshift(g)))  # K has to be symmetrical with the origin
        g_k = n * g_k_n[(n - N) // 2: (n + N) // 2]

        # 3. Divide by the Fourier transform of the convolution kernel
        f_k = g_k / phi_hat(k, n, m, sigma)
        f_k = f_k[indx_dc:]
        f_k = np.conj(f_k)
        return f_k

    def find_nearest(array, value):
        array = np.asarray(array)
        idx = (np.abs(array - value)).argmin()
        return idx
        
    indx1 = find_nearest(self.t, t_span[0])
    indx2 = find_nearest(self.t, t_span[-1])
    N = indx2 - indx1

    u_fft = mdl.u_ss.real[indx1:indx2]
    i_fft = mdl.i_ss.real[indx1:indx2]
    t_fft = mdl.t[indx1:indx2]
    plt.figure(1)
    k = np.arange(1700)
    ua_k = nfft(self.t_fft, self.u_fft,len(k))
    ua_k_abs    = abs(ua_k)/(N/2)
    ua_k_abs[0] = ua_k_abs[0]/2
    plt.bar(k,ua_k_abs)
    import pandas as pd
    data_df1 = pd.DataFrame(ua_k_abs)
    data_df2 = pd.DataFrame(ua_k)
    data_df1.columns = ['absolute values']
    data_df2.columns = ['complex values']
    writer = pd.ExcelWriter('u_fft.xlsx')
    data_df1.to_excel(writer,'page_1',float_format='%.5f')
    data_df2.to_excel(writer,'page_2', float_format='%.5f')
    writer.save()
    plt.legend()
    plt.show()
    
    
    

# %%
def save_plot(name):
    """
    Save figures.

    This saves figures in a folder "figures" in the current directory. If the
    folder doesn't exist, it is created.

    Parameters
    ----------
    name : string
        Name for the figure
    plt : object
        Handle for the figure to be saved

    """
    plt.savefig(name + '.pdf')
