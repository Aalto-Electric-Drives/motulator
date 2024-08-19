"""Example plotting scripts for machine drives."""

# %%
from types import SimpleNamespace

import numpy as np
import matplotlib.pyplot as plt
from cycler import cycler

from motulator.common.utils import complex2abc

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
    #mdl = sim.mdl.data  # Continuous-time data
    mdl = sim.mdl  # Continuous-time data
    ctrl = sim.ctrl.data  # Discrete-time data
    ctrl.t = ctrl.ref.t  # Discrete time

    # Check if the time span was given
    if t_span is None:
        t_span = (0, ctrl.t[-1])

    # Check if the base values were given
    if base is None:
        pu_vals = False
        base = SimpleNamespace(w=1, u=1, i=1, psi=1, tau=1)
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
    try:
        ax1.plot(
            ctrl.t,
            ctrl.ref.w_m/base.w,
            "--",
            ds="steps-post",
            label=r"$\omega_\mathrm{m,ref}$")
    except (AttributeError, TypeError):
        pass
    ax1.plot(
        mdl.machine.data.t,
        mdl.machine.data.w_m/base.w,
        label=r"$\omega_\mathrm{m}$")
    try:
        ax1.plot(
            ctrl.t,
            ctrl.fbk.w_m/base.w,
            label=r"$\hat \omega_\mathrm{m}$",
            ds="steps-post")
    except AttributeError:
        pass
    ax1.legend()
    ax1.set_xlim(t_span)
    ax1.set_xticklabels([])

    # Subplot 2: torques
    try:
        ax2.plot(
            mdl.mechanics.data.t,
            mdl.mechanics.data.tau_L_tot/base.tau,
            ":",
            label=r"$\tau_\mathrm{L,tot}$")
    except AttributeError:
        pass
    ax2.plot(
        mdl.machine.data.t,
        mdl.machine.data.tau_M/base.tau,
        label=r"$\tau_\mathrm{M}$")
    try:
        ax2.plot(
            ctrl.t,
            ctrl.ref.tau_M/base.tau,
            "--",
            label=r"$\tau_\mathrm{M,ref}$",
            ds="steps-post")
    except AttributeError:
        pass
    ax2.legend()
    ax2.set_xlim(t_span)
    ax2.set_xticklabels([])

    # Subplot 3: currents
    ax3.plot(
        ctrl.t,
        ctrl.fbk.i_s.real/base.i,
        label=r"$i_\mathrm{sd}$",
        ds="steps-post")
    ax3.plot(
        ctrl.t,
        ctrl.fbk.i_s.imag/base.i,
        label=r"$i_\mathrm{sq}$",
        ds="steps-post")
    try:
        ax3.plot(
            ctrl.t,
            ctrl.ref.i_s.real/base.i,
            "--",
            label=r"$i_\mathrm{sd,ref}$",
            ds="steps-post")
        ax3.plot(
            ctrl.t,
            ctrl.ref.i_s.imag/base.i,
            "--",
            label=r"$i_\mathrm{sq,ref}$",
            ds="steps-post")
    except (AttributeError, TypeError):
        pass
    ax3.legend()
    ax3.set_xlim(t_span)
    ax3.set_xticklabels([])

    # Subplot 4: voltages
    ax4.plot(
        ctrl.t,
        np.abs(ctrl.fbk.u_s)/base.u,
        label=r"$u_\mathrm{s}$",
        ds="steps-post")
    ax4.plot(
        ctrl.t,
        ctrl.fbk.u_dc/np.sqrt(3)/base.u,
        "--",
        label=r"$u_\mathrm{dc}/\sqrt{3}$",
        ds="steps-post")
    ax4.legend()
    ax4.set_xlim(t_span)
    ax4.set_xticklabels([])

    # Subplot 5: flux linkages
    if motor_type == "sm":
        ax5.plot(
            mdl.machine.data.t,
            np.abs(mdl.machine.data.psi_ss)/base.psi,
            label=r"$\psi_\mathrm{s}$")
        try:
            ax5.plot(
                ctrl.t,
                np.abs(ctrl.fbk.psi_s)/base.psi,
                label=r"$\hat\psi_\mathrm{s}$",
                ds="steps-post")
        except (AttributeError, TypeError):
            pass
        try:
            ax5.plot(
                ctrl.t,
                np.abs(ctrl.ref.psi_s)/base.psi,
                "--",
                label=r"$\psi_\mathrm{s,ref}$",
                ds="steps-post")
        except (AttributeError, TypeError):
            pass

    else:
        ax5.plot(
            mdl.machine.data.t,
            np.abs(mdl.machine.data.psi_ss)/base.psi,
            label=r"$\psi_\mathrm{s}$")
        ax5.plot(
            mdl.t, np.abs(mdl.psi_Rs)/base.psi, label=r"$\psi_\mathrm{R}$")
        try:
            ax5.plot(
                ctrl.t,
                np.abs(ctrl.fbk.psi_s)/base.psi,
                label=r"$\hat\psi_\mathrm{s}$",
                ds="steps-post")
        except AttributeError:
            pass
    ax5.legend()
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
    mdl = sim.mdl  # Continuous-time data
    ctrl = sim.ctrl.data  # Discrete-time data
    ctrl.t = ctrl.ref.t  # Discrete time

    # Check if the time span was given
    if t_span is None:
        t_span = (0, ctrl.t[-1])

    # Check if the base values were given
    if base is not None:
        pu_vals = True
    else:
        pu_vals = False
        base = SimpleNamespace(w=1, u=1, i=1, psi=1, tau=1)

    # Angle of synchronous coordinates
    try:
        theta = ctrl.fbk.theta_s  # Induction machine
    except AttributeError:
        theta = ctrl.fbk.theta_m  # Synchronous machine

    # Quantities in stator coordinates
    ctrl.fbk.u_ss = np.exp(1j*theta)*ctrl.fbk.u_s
    ctrl.fbk.i_ss = np.exp(1j*theta)*ctrl.fbk.i_s

    fig1, (ax1, ax2) = plt.subplots(2, 1)

    # Subplot 1: voltages
    ax1.plot(
        mdl.converter.data.t,
        mdl.converter.data.u_cs.real/base.u,
        label=r"$u_\mathrm{sa}$")
    ax1.plot(
        ctrl.t,
        ctrl.fbk.u_ss.real/base.u,
        label=r"$\hat u_\mathrm{sa}$",
        ds="steps-post")
    ax1.set_xlim(t_span)
    ax1.legend()
    ax1.set_xticklabels([])

    # Subplot 2: currents
    ax2.plot(
        mdl.machine.data.t,
        complex2abc(mdl.machine.data.i_ss).T/base.i,
        label=[r"$i_\mathrm{sa}$", r"$i_\mathrm{sb}$", r"$i_\mathrm{sc}$"])
    ax2.plot(
        ctrl.t,
        ctrl.fbk.i_ss.real/base.i,
        label=r"$\hat i_\mathrm{sa}$",
        ds="steps-post")
    ax2.set_xlim(t_span)
    ax2.legend()

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
        mdl.converter.data.i_L
    except AttributeError:
        mdl.converter.data.i_L = None

    if mdl.converter.data.i_L is not None:
        fig2, (ax1, ax2) = plt.subplots(2, 1)

        # Subplot 1: voltages
        ax1.plot(
            mdl.converter.data.t,
            mdl.converter.data.u_di/base.u,
            label=r"$u_\mathrm{di}$")
        ax1.plot(
            mdl.converter.data.t,
            mdl.converter.data.u_dc/base.u,
            label=r"$u_\mathrm{dc}$")
        ax1.plot(
            mdl.converter.data.t,
            mdl.converter.data.u_g_abc.T/base.u,
            label=r"$u_\mathrm{ga}$")
        ax1.legend()
        ax1.set_xlim(t_span)
        ax1.set_xticklabels([])

        # Subplot 2: currents
        ax2.plot(
            mdl.converter.data.t,
            mdl.converter.data.i_L/base.i,
            label=r"$i_\mathrm{L}$")
        ax2.plot(
            mdl.converter.data.t,
            mdl.converter.data.i_dc/base.i,
            label=r"$i_\mathrm{dc}$")
        ax2.plot(
            mdl.converter.data.t,
            mdl.converter.data.i_gs.real/base.i,
            label=r"$i_\mathrm{ga}$")
        ax2.legend()
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
