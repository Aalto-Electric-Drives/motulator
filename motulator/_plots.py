"""Example plotting scripts."""

# %%
import numpy as np
import matplotlib.pyplot as plt
from cycler import cycler
from motulator._helpers import complex2abc
from motulator._utils import Bunch

# Plotting parameters
plt.rcParams["axes.prop_cycle"] = cycler(color="brgcmyk")
plt.rcParams["lines.linewidth"] = 1.
plt.rcParams["axes.grid"] = True
plt.rcParams.update({"text.usetex": False})


# %%
# pylint: disable=too-many-branches
def plot(sim, base=None, t_span=None):
    """
    Plot example figures.

    Plots figures in per-unit values, if the base values are given. Otherwise
    SI units are used.

    Parameters
    ----------
    sim : Simulation
        Should contain the simulated data.
    base : BaseValues, optional
        Base values for scaling the waveforms.
    t_span : 2-tuple, optional
        Time span. The default is (0, sim.ctrl.t[-1]).

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
            motor_type = "im"
    except AttributeError:
        motor_type = "sm"

    fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, 1, figsize=(8, 10))

    # Subplot 1: angular speeds
    ax1.step(ctrl.t, ctrl.w_m_ref/base.w, "--", where="post")
    ax1.plot(mdl.t, mdl.w_m/base.w)
    try:
        ax1.step(ctrl.t, ctrl.w_m/base.w, where="post")
    except AttributeError:
        pass
    ax1.legend([
        r"$\omega_\mathrm{m,ref}$",
        r"$\omega_\mathrm{m}$",
        r"$\hat \omega_\mathrm{m}$",
    ])
    ax1.set_xlim(t_span)
    ax1.set_xticklabels([])

    # Subplot 2: torques
    ax2.plot(mdl.t, mdl.tau_L/base.tau, "--")
    ax2.plot(mdl.t, mdl.tau_M/base.tau)
    try:
        ax2.step(ctrl.t, ctrl.tau_M_ref_lim/base.tau)  # Limited torque ref
    except AttributeError:
        pass
    ax2.legend([
        r"$\tau_\mathrm{L}$",
        r"$\tau_\mathrm{M}$",
        r"$\tau_\mathrm{M,ref}$",
    ])
    ax2.set_xlim(t_span)
    ax2.set_xticklabels([])

    # Subplot 3: currents
    ax3.step(ctrl.t, ctrl.i_s.real/base.i, where="post")
    ax3.step(ctrl.t, ctrl.i_s.imag/base.i, where="post")
    try:
        ax3.step(ctrl.t, ctrl.i_s_ref.real/base.i, "--", where="post")
        ax3.step(ctrl.t, ctrl.i_s_ref.imag/base.i, "--", where="post")
    except AttributeError:
        pass
    ax3.legend([
        r"$i_\mathrm{sd}$",
        r"$i_\mathrm{sq}$",
        r"$i_\mathrm{sd,ref}$",
        r"$i_\mathrm{sq,ref}$",
    ])
    ax3.set_xlim(t_span)
    ax3.set_xticklabels([])

    # Subplot 4: voltages
    ax4.step(ctrl.t, np.abs(ctrl.u_s)/base.u, where="post")
    ax4.step(ctrl.t, ctrl.u_dc/np.sqrt(3)/base.u, "--", where="post")
    ax4.legend([r"$u_\mathrm{s}$", r"$u_\mathrm{dc}/\sqrt{3}$"])
    ax4.set_xlim(t_span)
    ax4.set_xticklabels([])

    # Subplot 5: flux linkages
    if motor_type == "sm":
        ax5.plot(mdl.t, np.abs(mdl.psi_s)/base.psi)
        try:
            ax5.step(ctrl.t, np.abs(ctrl.psi_s)/base.psi, where="post")
        except AttributeError:
            pass
        ax5.legend([r"$\psi_\mathrm{s}$", r"$\hat\psi_\mathrm{s}$"])
    else:
        ax5.plot(mdl.t, np.abs(mdl.psi_ss)/base.psi)
        ax5.plot(mdl.t, np.abs(mdl.psi_Rs)/base.psi)
        try:
            ax5.plot(ctrl.t, np.abs(ctrl.psi_s)/base.psi)
        except AttributeError:
            pass
        ax5.legend([
            r"$\psi_\mathrm{s}$",
            r"$\psi_\mathrm{R}$",
            r"$\hat \psi_\mathrm{s}$",
        ])
    ax5.set_xlim(t_span)

    # Add axis labels
    if pu_vals:
        ax1.set_ylabel("Speed (p.u.)")
        ax2.set_ylabel("Torque (p.u.)")
        ax3.set_ylabel("Current (p.u.)")
        ax4.set_ylabel("Voltage (p.u.)")
        ax5.set_ylabel("Flux linkage (p.u.)")
    else:
        ax1.set_ylabel("Speed (rad/s)")
        ax2.set_ylabel("Torque (Nm)")
        ax3.set_ylabel("Current (A)")
        ax4.set_ylabel("Voltage (V)")
        ax5.set_ylabel("Flux linkage (Vs)")
    ax5.set_xlabel("Time (s)")
    fig.align_ylabels()

    plt.tight_layout()
    plt.show()


# %%
# pylint: disable=too-many-statements
def plot_extra(sim, base=None, t_span=None):
    """
    Plot extra waveforms for a motor drive with a diode bridge.

    Parameters
    ----------
    sim : Simulation
        Should contain the simulated data.
    base : BaseValues, optional
        Base values for scaling the waveforms.
    t_span : 2-tuple, optional
        Time span. The default is (0, sim.ctrl.t[-1]).

    """
    mdl = sim.mdl.data  # Continuous-time data
    ctrl = sim.ctrl.data  # Discrete-time data

    # Check if the time span was given
    if t_span is None:
        t_span = (0, ctrl.t[-1])

    # Check if the base values were given
    if base is not None:
        pu_vals = True
    else:
        pu_vals = False
        base = Bunch(w=1, u=1, i=1, psi=1, tau=1)  # Unity base values

    # Angle of synchronous coordinates
    try:
        theta = ctrl.theta_s  # Induction machine
    except AttributeError:
        theta = ctrl.theta_m  # Synchronous machine

    # Quantities in stator coordinates
    ctrl.u_ss = np.exp(1j*theta)*ctrl.u_s
    ctrl.i_ss = np.exp(1j*theta)*ctrl.i_s

    fig1, (ax1, ax2) = plt.subplots(2, 1)

    # Subplot 1: voltages
    ax1.plot(mdl.t, mdl.u_ss.real/base.u)
    ax1.plot(ctrl.t, ctrl.u_ss.real/base.u)
    ax1.set_xlim(t_span)
    ax1.legend([r"$u_\mathrm{sa}$", r"$\hat u_\mathrm{sa}$"])
    ax1.set_xticklabels([])

    # Subplot 2: currents
    ax2.plot(mdl.t, complex2abc(mdl.i_ss).T/base.i)
    ax2.step(ctrl.t, ctrl.i_ss.real/base.i, where="post")
    ax2.set_xlim(t_span)
    ax2.legend([r"$i_\mathrm{sa}$", r"$i_\mathrm{sb}$", r"$i_\mathrm{sc}$"])

    # Add axis labels
    if pu_vals:
        ax1.set_ylabel("Voltage (p.u.)")
        ax2.set_ylabel("Current (p.u.)")
    else:
        ax1.set_ylabel("Voltage (V)")
        ax2.set_ylabel("Current (A)")
    ax2.set_xlabel("Time (s)")
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
            [r"$u_\mathrm{di}$", r"$u_\mathrm{dc}$", r"$u_\mathrm{ga}$"])
        ax1.set_xlim(t_span)
        ax1.set_xticklabels([])

        # Subplot 2: currents
        ax2.plot(mdl.t, mdl.i_L/base.i)
        ax2.plot(mdl.t, mdl.i_dc/base.i)
        ax2.plot(mdl.t, mdl.i_g.real/base.i)
        ax2.legend([r"$i_\mathrm{L}$", r"$i_\mathrm{dc}$", r"$i_\mathrm{ga}$"])
        ax2.set_xlim(t_span)

        # Add axis labels
        if pu_vals:
            ax1.set_ylabel("Voltage (p.u.)")
            ax2.set_ylabel("Current (p.u.)")
        else:
            ax1.set_ylabel("Voltage (V)")
            ax2.set_ylabel("Current (A)")
        ax2.set_xlabel("Time (s)")

        try:
            fig2.align_ylabels()
        except UnboundLocalError:
            pass

    plt.tight_layout()
    plt.show()


# %%
def save_plot(name):
    """
    Save figures.

    This saves figures in a folder "figures" in the current directory. If the
    folder does not exist, it is created.

    Parameters
    ----------
    name : string
        Name for the figure

    """
    plt.savefig(name + ".pdf")
