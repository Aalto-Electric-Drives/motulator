"""Example plotting scripts for grid converters."""

# %%
from types import SimpleNamespace
import numpy as np
import matplotlib.pyplot as plt
from cycler import cycler

from motulator.common.utils import (complex2abc)

# Plotting parameters
plt.rcParams['axes.prop_cycle'] = cycler(color='brgcmyk')
plt.rcParams['lines.linewidth'] = 1.
plt.rcParams['axes.grid'] = True
plt.rcParams.update({"text.usetex": False})


# %%
# pylint: disable=too-many-branches
def plot_grid(
        sim, base=None, plot_pcc_voltage=False, plot_w=False, t_span=None):
    """
    Plot example figures of grid converter simulations.

    Plots figures in per-unit values, if the base values are given. Otherwise
    SI units are used.

    Parameters
    ----------
    sim : Simulation
        Should contain the simulated data.
    base : BaseValues, optional
        Base values for scaling the waveforms.
    plot_pcc_voltage : bool, optional
        'True' if the user wants to plot the 3-phase waveform at the PCC. This
        is an optional feature and the grid voltage is plotted by default.
    plot_w : bool, optional
        'True' if the user wants to plot the grid frequency instead of the
        phase angles (by default).
    t_span : 2-tuple, optional
        Time span. The default is (0, sim.ctrl.ref.t[-1]).

    """
    FS = 16  # Font size of the plots axis
    FL = 16  # Font size of the legends only
    LW = 3  # Line width in plots

    mdl = sim.mdl  # Continuous-time data
    ctrl = sim.ctrl.data  # Discrete-time data
    ctrl.t = ctrl.ref.t  # Discrete time

    # Check if the time span was given
    if t_span is None:
        t_span = (0, ctrl.t[-1])  # Time span

    # Check if the base values were given
    if base is None:
        pu_vals = False
        # Scaling with unity base values except for power use kW
        base = SimpleNamespace(w=1, u=1, i=1, p=1000)
    else:
        pu_vals = True

    # 3-phase quantities
    #i_c_abc = complex2abc(mdl.converter.data.i_cs).T
    i_g_abc = complex2abc(mdl.grid_filter.data.i_gs).T
    e_g_abc = complex2abc(mdl.grid_model.data.e_gs).T
    u_g_abc = complex2abc(mdl.grid_filter.data.u_gs).T

    # Calculation of active and reactive powers
    #p_g = 1.5*np.asarray(np.real(ctrl.fbk.u_g*np.conj(ctrl.fbk.i_c)))
    #q_g = 1.5*np.asarray(np.imag(ctrl.fbk.u_g*np.conj(ctrl.fbk.i_c)))
    p_g = 1.5*np.asarray(
        np.real(mdl.grid_filter.data.u_gs*np.conj(mdl.grid_filter.data.i_gs)))
    q_g = 1.5*np.asarray(
        np.imag(mdl.grid_filter.data.u_gs*np.conj(mdl.grid_filter.data.i_gs)))
    p_g_ref = np.asarray(ctrl.ref.p_g)
    q_g_ref = np.asarray(ctrl.ref.q_g)

    # %%
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 7))

    if not sim.ctrl.dc_bus_volt_ctrl:
        if not plot_pcc_voltage:
            # Subplot 1: Grid voltage
            ax1.plot(mdl.grid_model.data.t, e_g_abc/base.u, linewidth=LW)
            ax1.legend([r'$e_g^a$', r'$e_g^b$', r'$e_g^c$'],
                       prop={'size': FL},
                       loc='upper right')
            ax1.set_xlim(t_span)
            ax1.set_xticklabels([])
        else:
            # Subplot 1: PCC voltage
            ax1.plot(mdl.grid_filter.data.t, u_g_abc/base.u, linewidth=LW)
            ax1.legend([r'$u_g^a$', r'$u_g^b$', r'$u_g^c$'],
                       prop={'size': FL},
                       loc='upper right')
            ax1.set_xlim(t_span)
            ax1.set_xticklabels([])
            #ax1.set_ylabel('Grid voltage (V)')
    else:
        # Subplot 1: DC-bus voltage
        ax1.plot(
            mdl.converter.data.t,
            mdl.converter.data.u_dc.T/base.u,
            linewidth=LW)
        ax1.plot(ctrl.t, ctrl.ref.u_dc/base.u, '--', linewidth=LW)
        ax1.legend([r'$u_{dc}$', r'$u_{dc,ref}$'],
                   prop={'size': FL},
                   loc='upper right')
        ax1.set_xlim(t_span)
        ax1.set_xticklabels([])
        #ax1.set_ylabel('DC-bus voltage (V)')

    # Subplot 2: Grid currents
    ax2.plot(mdl.grid_filter.data.t, i_g_abc/base.i, linewidth=LW)
    ax2.legend([r'$i_g^a$', r'$i_g^b$', r'$i_g^c$'],
               prop={'size': FL},
               loc='upper right')
    ax2.set_xlim(t_span)
    ax2.set_xticklabels([])

    if plot_w:
        # Subplot 3: Grid and converter frequencies
        ax3.plot(
            mdl.grid_model.data.t,
            mdl.grid_model.data.w_g/base.w,
            linewidth=LW)
        ax3.plot(ctrl.t, ctrl.fbk.w_c/base.w, '--', linewidth=LW)
        ax3.legend([r'$\omega_{g}$', r'$\omega_{c}$'],
                   prop={'size': FL},
                   loc='upper right')
        ax3.set_xlim(t_span)
    else:
        # Subplot 3: Phase angles
        ax3.plot(
            mdl.grid_model.data.t, mdl.grid_model.data.theta_g, linewidth=LW)
        ax3.plot(ctrl.t, ctrl.fbk.theta_c, '--', linewidth=LW)
        ax3.legend([r'$\theta_{g}$', r'$\theta_{c}$'],
                   prop={'size': FL},
                   loc='upper right')
        ax3.set_xlim(t_span)

    # Add axis labels
    if pu_vals:
        ax1.set_ylabel('Voltage (p.u.)')
        ax2.set_ylabel('Current (p.u.)')
    else:
        ax1.set_ylabel('Voltage (V)')
        ax2.set_ylabel('Current (A)')
    if not plot_w:
        ax3.set_ylabel('Angle (rad)')
    elif pu_vals:
        ax3.set_ylabel('Frequency (p.u.)')
    else:
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
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 7))

    # Subplot 1: Active and reactive power
    ax1.plot(mdl.grid_filter.data.t, p_g/base.p, linewidth=LW)
    ax1.plot(mdl.grid_filter.data.t, q_g/base.p, linewidth=LW)
    ax1.plot(ctrl.t, (p_g_ref/base.p), '--', linewidth=LW)
    ax1.plot(ctrl.t, (q_g_ref/base.p), '--', linewidth=LW)
    ax1.legend([r'$p_{g}$', r'$q_{g}$', r'$p_{g,ref}$', r'$q_{g,ref}$'],
               prop={'size': FL},
               loc='upper right')
    ax1.set_xlim(t_span)
    ax1.set_xticklabels([])

    # Subplot 2: Converter currents
    ax2.plot(ctrl.t, np.real(ctrl.fbk.i_c/base.i), linewidth=LW)
    ax2.plot(ctrl.t, np.imag(ctrl.fbk.i_c/base.i), linewidth=LW)
    ax2.plot(ctrl.t, np.real(ctrl.ref.i_c/base.i), '--', linewidth=LW)
    ax2.plot(ctrl.t, np.imag(ctrl.ref.i_c/base.i), '--', linewidth=LW)
    #ax2.plot(mdl.t, mdl.iL, linewidth=LW) converter-side dc current for debug
    ax2.legend(
        [r'$i_{c}^d$', r'$i_{c}^q$', r'$i_{c,ref}^d$', r'$i_{c,ref}^q$'],
        prop={'size': FL},
        loc='upper right')
    ax2.set_xlim(t_span)
    ax2.set_xticklabels([])

    # Subplot 3: Converter voltage reference and grid voltage
    ax3.plot(
        ctrl.t,
        np.real(ctrl.fbk.u_c/base.u),
        ctrl.t,
        np.imag(ctrl.fbk.u_c/base.u),
        linewidth=LW)
    ax3.plot(
        mdl.grid_model.data.t,
        np.absolute(mdl.grid_model.data.e_gs/base.u),
        'k--',
        linewidth=LW)
    ax3.legend([r'$u_{c}^d$', r'$u_{c}^q$', r'$|e_{g}|$'],
               prop={'size': FS},
               loc='upper right')
    ax3.set_xlim(t_span)

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


def plot_voltage_vector(sim, base=None):
    """
    Plot locus of the grid voltage vector.

    Parameters
    ----------
    sim : Simulation
        Should contain the simulated data.
    base : BaseValues, optional
        Base values for scaling the waveforms.

    """
    FS = 16  # Font size of the plots axis
    FL = 16  # Font size of the legends only
    LW = 3  # Line width in plots

    mdl = sim.mdl  # Continuous-time data

    # Check if the base values were given
    if base is None:
        pu_vals = False
        # Scaling with unity base values except for power use kW
        base = SimpleNamespace(w=1, u=1, i=1, p=1000)
    else:
        pu_vals = True

    # Plot the grid voltage vector in the complex plane
    e_g_alpha = mdl.grid_model.data.e_gs.real
    e_g_beta = mdl.grid_model.data.e_gs.imag

    _, ax = plt.subplots()
    ax.plot(e_g_alpha/base.u, e_g_beta/base.u, linewidth=LW)
    ax.axhline(0, color="k")
    ax.axvline(0, color="k")
    ticks = [-1.5, -1, -0.5, 0, 0.5, 1, 1.5]
    if pu_vals:
        ax.set_xlabel("Real part (p.u.)")
        ax.set_ylabel("Imaginary part (p.u.)")
        ax.set_xticks(ticks)
        ax.set_yticks(ticks)
    else:
        ax.set_xlabel("Real part (V)")
        ax.set_ylabel("Imaginary part (V)")
    ax.legend([r"$\boldsymbol{e}_\mathrm{g}^\mathrm{s}$"], prop={'size': FL})
    ax.set_aspect("equal")
    for item in ([ax.title, ax.xaxis.label, ax.yaxis.label] +
                 ax.get_xticklabels() + ax.get_yticklabels()):
        item.set_fontsize(FS)


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

    """
    plt.savefig(name + '.pdf')
