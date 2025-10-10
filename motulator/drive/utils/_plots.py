"""Example plotting scripts for machine drives."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from numpy.typing import ArrayLike

from motulator.common.model._simulation import SimulationResults
from motulator.common.utils._plotting import (
    configure_axes,
    save_and_show,
    set_latex_style,
    set_screen_style,
)
from motulator.common.utils._utils import BaseValues, complex2abc

# LaTeX symbol definition for easier configuration. When per-unit values are used,
# sometimes subscript m is preferred instead of M.
M = r"\mathrm{M}"


# %%
def _setup_plot(latex: bool) -> tuple[float, float]:
    """Setup plot style and return figure dimensions."""
    if latex:
        set_latex_style()
        height = plt.rcParams["figure.figsize"][1] * 2.2
    else:
        set_screen_style()
        height = plt.rcParams["figure.figsize"][1] * 1.8

    width = plt.rcParams["figure.figsize"][0]
    return width, height


# %%
def _plot_speeds(ax, ctrl, mdl, base: BaseValues) -> None:
    """Plot angular speeds."""
    if hasattr(ctrl.ref, "w_M") and np.any(ctrl.ref.w_M):
        ax.plot(
            ctrl.t,
            ctrl.ref.w_M / base.w_M,
            "--",
            ds="steps-post",
            label=rf"$\omega_{M}^\mathrm{{ref}}$",
        )
    ax.plot(mdl.t, mdl.machine.w_M / base.w_M, label=rf"$\omega_{M}$")
    ax.plot(
        ctrl.t, ctrl.fbk.w_M / base.w_M, label=rf"$\hat{{\omega}}_{M}$", ds="steps-post"
    )
    ax.legend()


def _plot_torques(ax, ctrl, mdl, base: BaseValues) -> None:
    """Plot torques."""
    # Plot torque reference only if it is nonzero
    if hasattr(ctrl.ref, "tau_M") and np.any(ctrl.ref.tau_M):
        ax.plot(
            ctrl.t,
            ctrl.ref.tau_M / base.tau,
            "--",
            label=rf"$\tau_{M}^\mathrm{{ref}}$",
            ds="steps-post",
        )
    ax.plot(mdl.t, mdl.machine.tau_M / base.tau, label=rf"$\tau_{M}$")
    # Plot torque estimated only if it is nonzero
    if hasattr(ctrl.fbk, "tau_M") and np.any(ctrl.fbk.tau_M):
        ax.plot(
            ctrl.t,
            ctrl.fbk.tau_M / base.tau,
            label=rf"$\hat{{\tau}}_{M}$",
            ds="steps-post",
        )
    if hasattr(mdl.mechanics, "tau_L_tot"):
        ax.plot(
            mdl.t,
            mdl.mechanics.tau_L_tot / base.tau,
            ":",
            label=r"$\tau_\mathrm{L}^\mathrm{tot}$",
        )
    ax.legend(loc="upper right")


def _plot_currents(ax, ctrl, base: BaseValues) -> None:
    """Plot currents."""
    if hasattr(ctrl.ref, "i_s") and np.any(ctrl.ref.i_s):
        ax.plot(
            ctrl.t,
            ctrl.ref.i_s.real / base.i,
            "--",
            label=r"$i_\mathrm{d}^\mathrm{ref}$",
            ds="steps-post",
        )
    ax.plot(
        ctrl.t, ctrl.fbk.i_s.real / base.i, label=r"$i_\mathrm{d}$", ds="steps-post"
    )
    if hasattr(ctrl.ref, "i_s") and np.any(ctrl.ref.i_s):
        ax.plot(
            ctrl.t,
            ctrl.ref.i_s.imag / base.i,
            "--",
            label=r"$i_\mathrm{q}^\mathrm{ref}$",
            ds="steps-post",
        )
    ax.plot(
        ctrl.t, ctrl.fbk.i_s.imag / base.i, label=r"$i_\mathrm{q}$", ds="steps-post"
    )
    ax.legend(loc="upper right")


def _plot_voltages(ax, ctrl, base: BaseValues) -> None:
    """Plot voltages."""
    ax.plot(
        ctrl.t,
        np.abs(ctrl.fbk.u_s) / base.u,
        label=r"$\hat{u}_\mathrm{s}$",
        ds="steps-post",
    )
    ax.plot(
        ctrl.t,
        ctrl.fbk.u_dc / np.sqrt(3) / base.u,
        "--",
        label=r"$u_\mathrm{dc}/\sqrt{3}$",
        ds="steps-post",
    )
    ax.legend(loc="lower right")
    _, ymax = ax.get_ylim()
    ax.set_ylim(0, ymax)


def _plot_flux_linkages(ax, ctrl, mdl, base: BaseValues) -> None:
    """Plot flux linkages."""
    if hasattr(ctrl.ref, "psi_s"):
        ax.plot(
            ctrl.t,
            np.abs(ctrl.ref.psi_s) / base.psi,
            "--",
            label=r"$\psi_\mathrm{s}^\mathrm{ref}$",
            ds="steps-post",
        )
    ax.plot(mdl.t, np.abs(mdl.machine.psi_s_ab) / base.psi, label=r"$\psi_\mathrm{s}$")

    if hasattr(mdl.machine, "psi_R_ab"):
        ax.plot(
            mdl.t,
            np.abs(mdl.machine.psi_R_ab) / base.psi,
            "-.",
            label=r"$\psi_\mathrm{R}$",
        )
    if hasattr(ctrl.fbk, "psi_s"):
        ax.plot(
            ctrl.t,
            np.abs(ctrl.fbk.psi_s) / base.psi,
            label=r"$\hat{\psi}_\mathrm{s}$",
            ds="steps-post",
        )
    ax.legend(loc="upper right")
    _, ymax = ax.get_ylim()
    ax.set_ylim(0, ymax)


def plot(
    res: SimulationResults,
    base: BaseValues | None = None,
    t_lims: tuple[float, float] | None = None,
    t_ticks: ArrayLike | None = None,
    y_lims: list[tuple[float, float] | None] | None = None,
    y_ticks: list[ArrayLike | None] | None = None,
    latex: bool = False,
    save_path: str | Path | None = None,
    **savefig_kwargs,
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
    t_lims : tuple[float, float], optional
        Time axis limits. If None, uses full time range.
    t_ticks : ArrayLike, optional
        Time axis tick locations.
    y_lims : list[tuple[float, float] | None], optional
        y-axis limits for each subplot.
    y_ticks : list[ArrayLike | None], optional
        y-axis tick locations for each subplot.
    latex : bool, optional
        Use LaTeX fonts for the labels. Enabling this option requires a working LaTeX
        installation, defaults to False.
    save_path : str | Path, optional
        Path to save the figure. If None, the figure is not saved.
    **savefig_kwargs
        Additional keyword arguments passed to plt.savefig().

    """
    # Setup plot style and dimensions
    width, height = _setup_plot(latex)

    # Process base values
    if base is None:
        pu_vals = False
        base = BaseValues.unity()
    else:
        pu_vals = True

    # Set time limits
    if t_lims is None:
        t_lims = (0, res.mdl.t[-1])

    # Create figure
    fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(
        5, 1, figsize=(width, height), sharex=True
    )
    axes = [ax1, ax2, ax3, ax4, ax5]

    # Plot subplots
    _plot_speeds(ax1, res.ctrl, res.mdl, base)
    _plot_torques(ax2, res.ctrl, res.mdl, base)
    _plot_currents(ax3, res.ctrl, base)
    _plot_voltages(ax4, res.ctrl, base)
    _plot_flux_linkages(ax5, res.ctrl, res.mdl, base)

    # Configure all axes
    configure_axes(axes, t_lims, t_ticks, y_lims, y_ticks)

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
    fig.align_ylabels()
    axes[-1].set_xlabel("Time (s)")

    save_and_show(save_path, **savefig_kwargs)


# %%
def _plot_phase_voltages(ax, mdl, ctrl, base: BaseValues) -> None:
    """Plot phase voltages."""
    u_s_ab = np.exp(1j * ctrl.fbk.theta_c) * ctrl.fbk.u_s

    ax.plot(mdl.t, mdl.machine.u_s_ab.real / base.u, label=r"$u_\mathrm{sa}$")
    ax.plot(
        ctrl.t, u_s_ab.real / base.u, label=r"$\hat{u}_\mathrm{sa}$", ds="steps-post"
    )
    ax.legend()


def _plot_phase_currents(ax, mdl, ctrl, base: BaseValues) -> None:
    """Plot phase currents."""
    i_s_ab = np.exp(1j * ctrl.fbk.theta_c) * ctrl.fbk.i_s

    ax.plot(
        mdl.t,
        complex2abc(mdl.machine.i_s_ab).T / base.i,
        label=[r"$i_\mathrm{sa}$", r"$i_\mathrm{sb}$", r"$i_\mathrm{sc}$"],
    )
    ax.plot(ctrl.t, i_s_ab.real / base.i, label=r"$i_\mathrm{sa}(k)$", ds="steps-post")
    ax.legend()


def plot_stator_waveforms(
    res: SimulationResults,
    base: BaseValues | None = None,
    t_lims: tuple[float, float] | None = None,
    t_ticks: ArrayLike | None = None,
    y_lims: list[tuple[float, float] | None] | None = None,
    y_ticks: list[ArrayLike | None] | None = None,
    latex: bool = False,
    save_path: str | Path | None = None,
    **savefig_kwargs,
) -> None:
    """
    Plot stator voltage and current waveforms.

    Parameters
    ----------
    res : SimulationResults
        Should contain the simulated data.
    base : BaseValues, optional
        Base values for scaling the waveforms.
    t_lims : tuple[float, float], optional
        Time axis limits. If None, uses full time range.
    t_ticks : ArrayLike, optional
        Time axis tick locations.
    y_lims : list[tuple[float, float] | None], optional
        y-axis limits for each subplot.
    y_ticks : list[ArrayLike | None], optional
        y-axis tick locations for each subplot.
    latex : bool, optional
        Use LaTeX fonts for the labels, requires a working LaTeX installation.
    save_path : str | Path, optional
        Path to save the figure. If None, the figure is not saved.
    **savefig_kwargs
        Additional keyword arguments passed to plt.savefig().

    """
    # Setup plot style
    if latex:
        set_latex_style()
    else:
        set_screen_style()

    # Process base values
    if base is None:
        pu_vals = False
        base = BaseValues.unity()
    else:
        pu_vals = True

    # Set time limits
    if t_lims is None:
        t_lims = (0, res.mdl.t[-1])

    # Create figure
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
    axes = [ax1, ax2]

    # Plot subplots
    _plot_phase_voltages(ax1, res.mdl, res.ctrl, base)
    _plot_phase_currents(ax2, res.mdl, res.ctrl, base)

    # Configure all axes
    configure_axes(axes, t_lims, t_ticks, y_lims, y_ticks)

    # Add axis labels
    if pu_vals:
        ax1.set_ylabel("Voltage (p.u.)")
        ax2.set_ylabel("Current (p.u.)")
    else:
        ax1.set_ylabel("Voltage (V)")
        ax2.set_ylabel("Current (A)")
    fig.align_ylabels()
    axes[-1].set_xlabel("Time (s)")

    save_and_show(save_path, **savefig_kwargs)


# %%
def _plot_dc_bus_voltages(ax, mdl, base: BaseValues) -> None:
    """Plot DC bus and grid voltages."""
    ax.plot(mdl.t, mdl.converter.u_di / base.u, label=r"$u_\mathrm{di}$")
    ax.plot(mdl.t, mdl.converter.u_dc / base.u, label=r"$u_\mathrm{dc}$")
    ax.plot(mdl.t, mdl.converter.u_g_abc.T[:, 0] / base.u, label=r"$u_\mathrm{ga}$")
    ax.legend()


def _plot_dc_bus_currents(ax, mdl, base: BaseValues) -> None:
    """Plot DC bus and grid currents."""
    ax.plot(mdl.t, mdl.converter.i_L / base.i, label=r"$i_\mathrm{L}$")
    ax.plot(mdl.t, mdl.converter.i_dc_int / base.i, label=r"$i_\mathrm{dc}$")
    ax.plot(mdl.t, mdl.converter.i_g_ab.real / base.i, label=r"$i_\mathrm{ga}$")
    ax.legend()


def plot_dc_bus_waveforms(
    res: SimulationResults,
    base: BaseValues | None = None,
    t_lims: tuple[float, float] | None = None,
    t_ticks: ArrayLike | None = None,
    y_lims: list[tuple[float, float] | None] | None = None,
    y_ticks: list[ArrayLike | None] | None = None,
    latex: bool = False,
    save_path: str | Path | None = None,
    **savefig_kwargs,
) -> None:
    """
    Plot DC bus and grid-side waveforms.

    Parameters
    ----------
    res : SimulationResults
        Should contain the simulated data.
    base : BaseValues, optional
        Base values for scaling the waveforms.
    t_lims : tuple[float, float], optional
        Time axis limits. If None, uses full time range.
    t_ticks : ArrayLike, optional
        Time axis tick locations.
    y_lims : list[tuple[float, float] | None], optional
        y-axis limits for each subplot.
    y_ticks : list[ArrayLike | None], optional
        y-axis tick locations for each subplot.
    latex : bool, optional
        Use LaTeX fonts for the labels, requires a working LaTeX installation.
    save_path : str | Path, optional
        Path to save the figure. If None, the figure is not saved.
    **savefig_kwargs
        Additional keyword arguments passed to plt.savefig().

    """
    # Check if DC bus data exists
    if not hasattr(res.mdl.converter, "i_L"):
        print("DC bus data not available in simulation results.")
        return

    # Setup plot style
    if latex:
        set_latex_style()
    else:
        set_screen_style()

    # Process base values
    if base is None:
        pu_vals = False
        base = BaseValues.unity()
    else:
        pu_vals = True

    # Set time limits
    if t_lims is None:
        t_lims = (0, res.mdl.t[-1])

    # Create figure
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
    axes = [ax1, ax2]

    # Plot subplots
    _plot_dc_bus_voltages(ax1, res.mdl, base)
    _plot_dc_bus_currents(ax2, res.mdl, base)

    # Configure all axes
    configure_axes(axes, t_lims, t_ticks, y_lims, y_ticks)

    # Add axis labels
    if pu_vals:
        ax1.set_ylabel("Voltage (p.u.)")
        ax2.set_ylabel("Current (p.u.)")
    else:
        ax1.set_ylabel("Voltage (V)")
        ax2.set_ylabel("Current (A)")
    fig.align_ylabels()
    axes[-1].set_xlabel("Time (s)")

    save_and_show(save_path, **savefig_kwargs)
