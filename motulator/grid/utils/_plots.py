"""Example plotting scripts."""

# %%
import numpy as np
import matplotlib.pyplot as plt
from cycler import cycler

from motulator.common.utils import (complex2abc, wrap)
from types import SimpleNamespace

# Plotting parameters
plt.rcParams['axes.prop_cycle'] = cycler(color='brgcmyk')
plt.rcParams['lines.linewidth'] = 1.
plt.rcParams['axes.grid'] = True
plt.rcParams.update({"text.usetex": False})


# %%
# pylint: disable=too-many-branches
def plot_grid(
        sim, t_range=None, base=None, plot_pcc_voltage=False, plot_w=False):
    """
    Plot example figures of grid converter simulations.

    Plots figures in per-unit values, if the base values are given. Otherwise
    SI units are used.

    Parameters
    ----------
    sim : Simulation object
        Should contain the simulated data.
    t_range : 2-tuple, optional
        Time range. The default is (0, sim.ctrl.t[-1]).
    base : BaseValues, optional
        Base values for scaling the waveforms.
    plot_pcc_voltage : Boolean, optional
        'True' if the user wants to plot the 3-phase waveform at the PCC. This
        is an optional feature and the grid voltage is plotted by default.
    plot_w : Boolean, optional
        'True' if the user wants to plot the grid frequency instead of the
        phase angles (by default).

    """
    FS = 16 # Font size of the plots axis
    FL = 16 # Font size of the legends only
    LW = 3 # Line width in plots


    mdl = sim.mdl      # Continuous-time data
    ctrl = sim.ctrl.data    # Discrete-time data

    # Check if the time span was given
    if t_range is None:
        t_range = (0, mdl.converter.data.t[-1]) # Time span

    # Check if the base values were given
    if base is None:
        pu_vals = False
        # Scaling with unity base values except for power use kW
        base = SimpleNamespace(w=1, u=1, i=1, p=1000)
    else:
        pu_vals = True

    # 3-phase quantities
    i_c_abc = complex2abc(mdl.converter.data.i_cs).T
    e_g_abc = complex2abc(mdl.grid_model.data.e_gs).T

    # Calculation of active and reactive powers
    #p_g = 1.5*np.asarray(np.real(ctrl.u_g*np.conj(ctrl.i_c)))
    #q_g = 1.5*np.asarray(np.imag(ctrl.u_g*np.conj(ctrl.i_c)))
    p_g = 1.5*np.asarray(np.real(mdl.grid_filter.data.u_gs*np.conj(mdl.grid_filter.data.i_gs)))
    q_g = 1.5*np.asarray(np.imag(mdl.grid_filter.data.u_gs*np.conj(mdl.grid_filter.data.i_gs)))
    p_g_ref = np.asarray(ctrl.p_g_ref)
    q_g_ref = np.asarray(ctrl.q_g_ref)

    # %%
    fig, (ax1, ax2,ax3) = plt.subplots(3, 1, figsize=(10, 7))

    if sim.ctrl.on_v_dc==False:
        if plot_pcc_voltage == False:
            # Subplot 1: Grid voltage
            ax1.plot(mdl.grid_model.data.t, e_g_abc/base.u, linewidth=LW)
            ax1.legend([r'$e_g^a$',r'$e_g^b$',r'$e_g^c$'],
                       prop={'size': FL}, loc= 'upper right')
            ax1.set_xlim(t_range)
            ax1.set_xticklabels([])
        else:
            # Subplot 1: PCC voltage
            ax1.plot(ctrl.t, ctrl.u_g_abc/base.u, linewidth=LW)
            ax1.legend([r'$u_g^a$',r'$u_g^b$',r'$u_g^c$'],
                       prop={'size': FL}, loc= 'upper right')
            ax1.set_xlim(t_range)
            ax1.set_xticklabels([])
            #ax1.set_ylabel('Grid voltage (V)')
    else:
        # Subplot 1: DC-bus voltage
        ax1.plot(mdl.converter.data.t, mdl.converter.data.u_dc/base.u, linewidth=LW)
        ax1.plot(ctrl.t, ctrl.u_dc_ref/base.u, '--', linewidth=LW)
        ax1.legend([r'$u_{dc}$',r'$u_{dc,ref}$'],
                   prop={'size': FL}, loc= 'upper right')
        ax1.set_xlim(t_range)
        ax1.set_xticklabels([])
        #ax1.set_ylabel('DC-bus voltage (V)')

    # Subplot 2: Converter currents
    ax2.plot(mdl.converter.data.t, i_c_abc/base.i, linewidth=LW)
    ax2.legend([r'$i_c^a$',r'$i_c^b$',r'$i_c^c$']
               ,prop={'size': FL}, loc= 'upper right')
    ax2.set_xlim(t_range)
    ax2.set_xticklabels([])

    if plot_w:
        # Subplot 3: Grid and converter frequencies
        ax3.plot(mdl.grid_model.data.t, mdl.grid_model.par.w_N/base.w, linewidth=LW)
        ax3.plot(ctrl.t, ctrl.w_c/base.w, '--', linewidth=LW)
        ax3.legend([r'$\omega_{g}$',r'$\omega_{c}$']
                   ,prop={'size': FL}, loc= 'upper right')
        ax3.set_xlim(t_range)
    else:
        # Subplot 3: Phase angles
        ax3.plot(mdl.grid_model.data.t, mdl.grid_model.data.theta_g, linewidth=LW)
        ax3.plot(ctrl.t, ctrl.theta_c, '--', linewidth=LW)
        ax3.legend([r'$\theta_{g}$',r'$\theta_{c}$']
                   ,prop={'size': FL}, loc= 'upper right')
        ax3.set_xlim(t_range)

    # Add axis labels
    if pu_vals:
        ax1.set_ylabel('Voltage (p.u.)')
        ax2.set_ylabel('Current (p.u.)')
    else:
        ax1.set_ylabel('Voltage (V)')
        ax2.set_ylabel('Current (A)')
    if plot_w==False: 
        ax3.set_ylabel('Angle (rad)')
    elif plot_w and pu_vals:
        ax3.set_ylabel('Frequency (p.u.)')
    elif pu_vals == False:
        ax3.set_ylabel('Frequency (rad/s)')
    ax3.set_xlabel('Time (s)')

    # Change font size
    for item in ([ax1.title, ax1.xaxis.label, ax1.yaxis.label] +
             ax1.get_xticklabels() + ax1.get_yticklabels()):
        item.set_fontsize(FS)

    for item in ([ax2.title, ax2.xaxis.label, ax2.yaxis.label] +
             ax2.get_xticklabels() + ax2.get_yticklabels()):
        item.set_fontsize(FS)

    for item in ([ax3.title, ax3.xaxis.label, ax3.yaxis.label] +
             ax3.get_xticklabels() + ax3.get_yticklabels()):
        item.set_fontsize(FS)

    fig.align_ylabels()
    plt.tight_layout()
    plt.grid()
    ax3.grid()
    #plt.show()


    # %%
    # Second figure
    fig, (ax1, ax2,ax3) = plt.subplots(3, 1, figsize=(10, 7))

    # Subplot 1: Active and reactive power
    ax1.plot(mdl.grid_filter.data.t, p_g/base.p, linewidth=LW)
    ax1.plot(mdl.grid_filter.data.t, q_g/base.p, linewidth=LW)
    ax1.plot(ctrl.t, (p_g_ref/base.p), '--', linewidth=LW)
    ax1.plot(ctrl.t, (q_g_ref/base.p), '--', linewidth=LW)
    ax1.legend([r'$p_{g}$',r'$q_{g}$',r'$p_{g,ref}$',r'$q_{g,ref}$'],
               prop={'size': FL}, loc= 'upper right')
    ax1.set_xlim(t_range)
    ax1.set_xticklabels([])

    # Subplot 2: Converter currents
    ax2.plot(ctrl.t, np.real(ctrl.i_c/base.i), linewidth=LW)
    ax2.plot(ctrl.t, np.imag(ctrl.i_c/base.i), linewidth=LW)
    ax2.plot(ctrl.t, np.real(ctrl.i_c_ref/base.i), '--', linewidth=LW)
    ax2.plot(ctrl.t, np.imag(ctrl.i_c_ref/base.i), '--', linewidth=LW)
    #ax2.plot(mdl.t, mdl.iL, linewidth=LW) converter-side dc current for debug
    ax2.legend([r'$i_{c}^d$',r'$i_{c}^q$',r'$i_{c,ref}^d$',r'$i_{c,ref}^q$'],
               prop={'size': FL}, loc= 'upper right')
    ax2.set_xlim(t_range)
    ax2.set_xticklabels([])

    # Subplot 3: Converter voltage reference and grid voltage
    ax3.plot(ctrl.t,np.real(ctrl.u_c/base.u),
            ctrl.t,np.imag(ctrl.u_c/base.u), linewidth=LW)
    ax3.plot(mdl.grid_model.data.t, np.absolute(mdl.grid_model.data.e_gs/base.u),'k--',
             linewidth=LW)
    ax3.legend([r'$u_{c}^d$', r'$u_{c}^q$', r'$|e_{g}|$'],
                prop={'size': FS}, loc= 'upper right')
    ax3.set_xlim(t_range)

    # Change font size
    for item in ([ax2.title, ax2.xaxis.label, ax2.yaxis.label] +
             ax2.get_xticklabels() + ax2.get_yticklabels()):
        item.set_fontsize(FS)

    for item in ([ax1.title, ax1.xaxis.label, ax1.yaxis.label] +
             ax1.get_xticklabels() + ax1.get_yticklabels()):
        item.set_fontsize(FS)

    for item in ([ax3.title, ax3.xaxis.label, ax3.yaxis.label] +
             ax3.get_xticklabels() + ax3.get_yticklabels()):
        item.set_fontsize(FS)

    # Add axis labels
    if pu_vals:
        ax1.set_ylabel('Power (p.u.)')
        ax2.set_ylabel('Current (p.u.)')
        ax3.set_ylabel('Voltage (p.u.)')
    else:
        ax1.set_ylabel('Power (kW, kVar)')
        ax2.set_ylabel('Current (A)')
        ax3.set_ylabel('Voltage (V)')
    ax3.set_xlabel('Time (s)')

    fig.align_ylabels()
    plt.tight_layout()
    plt.grid()
    ax3.grid()
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
