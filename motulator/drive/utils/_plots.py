"""Example plotting scripts for machine drives."""

from typing import Any, Literal

import matplotlib.pyplot as plt
import numpy as np

from motulator.common.model._simulation import SimulationResults
from motulator.common.utils._utils import (
    BaseValues,
    complex2abc,
    set_latex_style,
    set_screen_style,
)


def _get_machine_type(mdl: Any) -> Literal["im", "sm"]:
    return "im" if hasattr(mdl.machine, "psi_R_ab") else "sm"


# %%
def plot(
    res: SimulationResults,
    base: BaseValues | None = None,
    t_span: tuple[float, float] | None = None,
    latex: bool = False,
) -> None:
    """
    Plot example figures.

    Parameters
    ----------
    res : SimulationResults
        Simulation results.
    base : BaseValues, optional
        Base values for scaling the waveforms. If not given, the waveforms are plotted
        in SI units.
    t_span : 2-tuple, optional
        Time span. If not given, the whole simulation time is plotted.
    latex : bool, optional
        Use LaTeX fonts for the labels. Enabling this option requires a working LaTeX
        installation, defaults to False.

    """
    # ruff: noqa: PLR0912
    if latex:
        set_latex_style()
        height = plt.rcParams["figure.figsize"][1] * 2.2
    else:
        set_screen_style()
        height = plt.rcParams["figure.figsize"][1] * 1.8

    width = plt.rcParams["figure.figsize"][0]
    fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, 1, figsize=(width, height))

    # For brevity
    mdl = res.mdl  # Continuous-time data
    ctrl = res.ctrl  # Discrete-time data

    # Check if the time span was given
    if t_span is None:
        t_span = (0, mdl.t[-1])

    # Check if the base values were given
    if base is None:
        pu_vals = False
        base = BaseValues.unity()
    else:
        pu_vals = True

    # Subplot 1: Angular speeds
    if hasattr(ctrl.ref, "w_M") and np.any(ctrl.ref.w_M):
        ax1.plot(
            ctrl.t,
            ctrl.ref.w_M / base.w_M,
            "--",
            ds="steps-post",
            label=r"$\omega_\mathrm{M,ref}$",
        )
    ax1.plot(mdl.t, mdl.machine.w_M / base.w_M, label=r"$\omega_\mathrm{M}$")
    ax1.plot(
        ctrl.t,
        ctrl.fbk.w_M / base.w_M,
        label=r"$\hat \omega_\mathrm{M}$",
        ds="steps-post",
    )
    ax1.legend()
    ax1.set_xlim(t_span)
    ax1.set_xticklabels([])

    # Subplot 2: torques
    # Plot torque reference only if it is nonzero
    if hasattr(ctrl.ref, "tau_M") and np.any(ctrl.ref.tau_M):
        ax2.plot(
            ctrl.t,
            ctrl.ref.tau_M / base.tau,
            "--",
            label=r"$\tau_\mathrm{M,ref}$",
            ds="steps-post",
        )
    ax2.plot(mdl.t, mdl.machine.tau_M / base.tau, label=r"$\tau_\mathrm{M}$")
    # Plot torque estimated only if it is nonzero
    if hasattr(ctrl.fbk, "tau_M") and np.any(ctrl.fbk.tau_M):
        ax2.plot(
            ctrl.t,
            ctrl.fbk.tau_M / base.tau,
            label=r"$\hat{\tau}_\mathrm{M}$",
            ds="steps-post",
        )
    if hasattr(mdl.mechanics, "tau_L_tot"):
        ax2.plot(
            mdl.t,
            mdl.mechanics.tau_L_tot / base.tau,
            ":",
            label=r"$\tau_\mathrm{L,tot}$",
        )
    ax2.legend(loc="upper right")
    ax2.set_xlim(t_span)
    ax2.set_xticklabels([])

    # Subplot 3: Currents
    if hasattr(ctrl.ref, "i_s") and np.any(ctrl.ref.i_s):
        ax3.plot(
            ctrl.t,
            ctrl.ref.i_s.real / base.i,
            "--",
            label=r"$i_\mathrm{d,ref}$",
            ds="steps-post",
        )
    ax3.plot(
        ctrl.t, ctrl.fbk.i_s.real / base.i, label=r"$i_\mathrm{d}$", ds="steps-post"
    )
    if hasattr(ctrl.ref, "i_s") and np.any(ctrl.ref.i_s):
        ax3.plot(
            ctrl.t,
            ctrl.ref.i_s.imag / base.i,
            "--",
            label=r"$i_\mathrm{q,ref}$",
            ds="steps-post",
        )
    ax3.plot(
        ctrl.t, ctrl.fbk.i_s.imag / base.i, label=r"$i_\mathrm{q}$", ds="steps-post"
    )
    ax3.legend(loc="upper right")
    ax3.set_xlim(t_span)
    ax3.set_xticklabels([])

    # Subplot 4: Voltages
    ax4.plot(
        ctrl.t, np.abs(ctrl.fbk.u_s) / base.u, label=r"$u_\mathrm{s}$", ds="steps-post"
    )
    ax4.plot(
        ctrl.t,
        ctrl.fbk.u_dc / np.sqrt(3) / base.u,
        "--",
        label=r"$u_\mathrm{dc}/\sqrt{3}$",
        ds="steps-post",
    )
    ax4.legend(loc="lower right")
    ax4.set_xlim(t_span)
    _, ymax = ax4.get_ylim()
    ax4.set_ylim(0, ymax)
    ax4.set_xticklabels([])

    # Subplot 5: Flux linkages
    if hasattr(ctrl.ref, "psi_s"):
        ax5.plot(
            ctrl.t,
            np.abs(ctrl.ref.psi_s) / base.psi,
            "--",
            label=r"$\psi_\mathrm{s,ref}$",
            ds="steps-post",
        )
    if _get_machine_type(mdl) == "sm":
        ax5.plot(
            mdl.t, np.abs(mdl.machine.psi_s_ab) / base.psi, label=r"$\psi_\mathrm{s}$"
        )
    else:
        ax5.plot(
            mdl.t, np.abs(mdl.machine.psi_s_ab) / base.psi, label=r"$\psi_\mathrm{s}$"
        )
        ax5.plot(
            mdl.t,
            np.abs(mdl.machine.psi_R_ab) / base.psi,
            "-.",
            label=r"$\psi_\mathrm{R}$",
        )
    if hasattr(ctrl.fbk, "psi_s"):
        ax5.plot(
            ctrl.t,
            np.abs(ctrl.fbk.psi_s) / base.psi,
            label=r"$\hat\psi_\mathrm{s}$",
            ds="steps-post",
        )
    ax5.legend(loc="upper right")
    ax5.set_xlim(t_span)
    _, ymax = ax5.get_ylim()
    ax5.set_ylim(0, ymax)

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

    plt.show()


# %%
def plot_extra(
    res: SimulationResults,
    base: BaseValues | None = None,
    t_span: tuple[float, float] | None = None,
    latex: bool = False,
) -> None:
    """
    Plot extra waveforms for a motor drive with a diode bridge.

    Parameters
    ----------
    res : Simulation
        Should contain the simulated data.
    base : BaseValues, optional
        Base values for scaling the waveforms.
    t_span : 2-tuple, optional
        Time span, defaults to (0, sim.ctrl.t[-1]).
    latex : bool, optional
        Use LaTeX fonts for the labels, requires a working LaTeX installation.

    """
    # ruff: noqa: PLR0915
    if latex:
        set_latex_style()
    else:
        set_screen_style()

    # For brevity
    mdl = res.mdl  # Continuous-time data
    ctrl = res.ctrl  # Discrete-time data

    # Check if the time span was given
    if t_span is None:
        t_span = (0, mdl.t[-1])

    # Check if the base values were given
    if base is None:
        pu_vals = False
        base = BaseValues.unity()
    else:
        pu_vals = True

    # Angle of synchronous coordinates
    if _get_machine_type(mdl) == "sm":
        theta = ctrl.fbk.theta_m
    else:
        theta = ctrl.fbk.theta_s

    # Quantities in stator coordinates
    ctrl.fbk.u_s_ab = np.exp(1j * theta) * ctrl.fbk.u_s
    ctrl.fbk.i_s_ab = np.exp(1j * theta) * ctrl.fbk.i_s

    fig1, (ax1, ax2) = plt.subplots(2, 1)

    # Subplot 1: Voltages
    ax1.plot(mdl.t, mdl.machine.u_s_ab.real / base.u, label=r"$u_\mathrm{sa}$")
    ax1.plot(
        ctrl.t,
        ctrl.fbk.u_s_ab.real / base.u,
        label=r"$\hat u_\mathrm{sa}$",
        ds="steps-post",
    )
    ax1.set_xlim(t_span)
    ax1.legend()
    ax1.set_xticklabels([])

    # Subplot 2: currents
    ax2.plot(
        mdl.t,
        complex2abc(mdl.machine.i_s_ab).T / base.i,
        label=[r"$i_\mathrm{sa}$", r"$i_\mathrm{sb}$", r"$i_\mathrm{sc}$"],
    )
    ax2.plot(
        ctrl.t,
        ctrl.fbk.i_s_ab.real / base.i,
        label=r"$\hat i_\mathrm{sa}$",
        ds="steps-post",
    )
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
    if hasattr(mdl.converter, "i_L"):
        fig2, (ax1, ax2) = plt.subplots(2, 1)

        # Subplot 1: Voltages
        ax1.plot(mdl.t, mdl.converter.u_di / base.u, label=r"$u_\mathrm{di}$")
        ax1.plot(mdl.t, mdl.converter.u_dc / base.u, label=r"$u_\mathrm{dc}$")
        ax1.plot(
            mdl.t, mdl.converter.u_g_abc.T[:, 0] / base.u, label=r"$u_\mathrm{ga}$"
        )
        ax1.legend()
        ax1.set_xlim(t_span)
        ax1.set_xticklabels([])

        # Subplot 2: Currents
        ax2.plot(mdl.t, mdl.converter.i_L / base.i, label=r"$i_\mathrm{L}$")
        ax2.plot(mdl.t, mdl.converter.i_dc_int / base.i, label=r"$i_\mathrm{dc}$")
        ax2.plot(mdl.t, mdl.converter.i_g_ab.real / base.i, label=r"$i_\mathrm{ga}$")
        ax2.legend()
        ax2.set_xlim(t_span)

        # Add axis labels
        if pu_vals:
            ax1.set_ylabel("Voltage (p.u.)")
            ax2.set_ylabel("Current (p.u.)")
        else:
            ax1.set_ylabel("Voltage (V)")
            ax2.set_ylabel("Current (A)")

        fig2.align_ylabels()

    plt.show()
