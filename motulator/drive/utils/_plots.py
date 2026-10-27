"""Example plotting scripts for machine drives."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from numpy.typing import ArrayLike

from motulator.common.model._simulation import SimulationResults
from motulator.common.utils._plotting import (
    configure_axes,
    save_and_show,
    set_style,
    setup_time_series_plot,
)
from motulator.common.utils._utils import BaseValues, complex2abc


# %%
def _has(ctrl, group: str, name: str) -> bool:
    """Check if the controller signal is available and not identically zero."""
    return ctrl is not None and bool(np.any(getattr(getattr(ctrl, group), name, 0)))


def _plot_speeds(ax, ctrl, mdl, base: BaseValues, M: str) -> None:
    """Plot angular speeds, using the LaTeX subscript `M` of mechanical quantities."""
    if _has(ctrl, "ref", "w_M"):
        ax.plot(
            ctrl.t,
            ctrl.ref.w_M / base.w_M,
            "--",
            ds="steps-post",
            label=rf"$\omega_{M}^\mathrm{{ref}}$",
        )
    ax.plot(mdl.t, mdl.machine.w_M / base.w_M, label=rf"$\omega_{M}$")
    if ctrl is not None:
        ax.plot(
            ctrl.t,
            ctrl.fbk.w_M / base.w_M,
            label=rf"$\hat{{\omega}}_{M}$",
            ds="steps-post",
        )
    ax.legend()


def _plot_torques(ax, ctrl, mdl, base: BaseValues, M: str) -> None:
    """Plot torques, using the LaTeX subscript `M` of mechanical quantities."""
    # Plot torque reference only if it is nonzero
    if _has(ctrl, "ref", "tau_M"):
        ax.plot(
            ctrl.t,
            ctrl.ref.tau_M / base.tau,
            "--",
            label=rf"$\tau_{M}^\mathrm{{ref}}$",
            ds="steps-post",
        )
    ax.plot(mdl.t, mdl.machine.tau_M / base.tau, label=rf"$\tau_{M}$")
    # Plot estimated torque only if it is nonzero
    if _has(ctrl, "fbk", "tau_M"):
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


def _plot_currents(ax, ctrl, mdl, base: BaseValues) -> None:
    """Plot currents, the magnitude of the stator current without control signals."""
    if ctrl is None:
        i_s = np.abs(mdl.machine.i_s_ab) / base.i
        ax.plot(mdl.t, i_s, label=r"$i_\mathrm{s}$")
        ax.legend(loc="upper right")
        return
    if _has(ctrl, "ref", "i_s"):
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
    if _has(ctrl, "ref", "i_s"):
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


def _plot_voltages(ax, ctrl, mdl, base: BaseValues) -> None:
    """Plot voltages, the magnitude of the stator voltage without control signals."""
    if ctrl is None:
        u_s = np.abs(mdl.machine.u_s_ab) / base.u
        ax.plot(mdl.t, u_s, label=r"$u_\mathrm{s}$")
        ax.legend(loc="lower right")
        ax.set_ylim(0, 1.2 * np.max(u_s, initial=1e-3))
        return
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
    if _has(ctrl, "ref", "psi_s"):
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
    if ctrl is not None and hasattr(ctrl.fbk, "psi_s"):
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
    subplots: list[str] | None = None,
    t_lims: tuple[float, float] | None = None,
    t_ticks: ArrayLike | None = None,
    y_lims: list[tuple[float, float] | None] | None = None,
    y_ticks: list[ArrayLike | None] | None = None,
    latex: bool = False,
    save_path: str | Path | None = None,
    **savefig_kwargs,
) -> None:
    """
    Plot example figures with selectable subplots.

    Parameters
    ----------
    res : ~motulator.common.model.SimulationResults
        Simulation results. Without a control system, the magnitudes of the stator
        current and voltage are plotted instead of the controller signals.
    base : BaseValues, optional
        Base values for scaling the waveforms. If not given, the waveforms are plotted
        in SI units.
    subplots : list[str], optional
        List of subplot names to include. Valid names: "speed", "torque", "current",
        "voltage", "flux". Defaults to all.
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
    # Process base values and set the LaTeX subscript of mechanical quantities
    if base is None:
        pu_vals = False
        base = BaseValues.unity()
        M = r"\mathrm{M}"
    else:
        pu_vals = True
        M = r"\mathrm{m}"

    # Setup plot style and dimensions
    width, height = setup_time_series_plot(latex)

    # Set time limits
    if t_lims is None:
        t_lims = (0, res.mdl.t[-1])

    # Subplot mapping: name -> (function, SI label, p.u. label)
    subplot_map = {
        "speed": (_plot_speeds, "Speed (rad/s)", "Speed (p.u.)"),
        "torque": (_plot_torques, "Torque (Nm)", "Torque (p.u.)"),
        "current": (_plot_currents, "Current (A)", "Current (p.u.)"),
        "voltage": (_plot_voltages, "Voltage (V)", "Voltage (p.u.)"),
        "flux": (_plot_flux_linkages, "Flux linkage (Vs)", "Flux linkage (p.u.)"),
    }

    if subplots is None:
        subplots = ["speed", "torque", "current", "voltage", "flux"]

    # Validate subplot names
    for name in subplots:
        if name not in subplot_map:
            raise ValueError(f"Unknown subplot name: {name}")

    n_subplots = len(subplots)
    base_height = height
    height = base_height * n_subplots / 5
    fig, axes = plt.subplots(n_subplots, 1, figsize=(width, height), sharex=True)
    if n_subplots == 1:
        axes = [axes]

    # Plot selected subplots
    for ax, name in zip(axes, subplots, strict=True):
        plot_func, label_si, label_pu = subplot_map[name]
        if name in ["speed", "torque"]:
            plot_func(ax, res.ctrl, res.mdl, base, M)
        else:
            plot_func(ax, res.ctrl, res.mdl, base)
        ax.set_ylabel(label_pu if pu_vals else label_si)

    # Configure all axes
    configure_axes(axes, t_lims, t_ticks, y_lims, y_ticks)

    fig.align_ylabels()
    axes[-1].set_xlabel("Time (s)")

    save_and_show(save_path, **savefig_kwargs)


# %%
def _plot_phase_voltages(ax, mdl, ctrl, base: BaseValues) -> None:
    """Plot phase voltages."""
    ax.plot(mdl.t, mdl.machine.u_s_ab.real / base.u, label=r"$u_\mathrm{sa}$")
    if ctrl is not None:
        u_s_ab = np.exp(1j * ctrl.fbk.theta_c) * ctrl.fbk.u_s
        ax.plot(
            ctrl.t,
            u_s_ab.real / base.u,
            label=r"$\hat{u}_\mathrm{sa}$",
            ds="steps-post",
        )
    ax.legend()


def _plot_phase_currents(ax, mdl, ctrl, base: BaseValues) -> None:
    """Plot phase currents."""
    ax.plot(
        mdl.t,
        complex2abc(mdl.machine.i_s_ab).T / base.i,
        label=[r"$i_\mathrm{sa}$", r"$i_\mathrm{sb}$", r"$i_\mathrm{sc}$"],
    )
    if ctrl is not None:
        i_s_ab = np.exp(1j * ctrl.fbk.theta_c) * ctrl.fbk.i_s
        ax.plot(
            ctrl.t, i_s_ab.real / base.i, label=r"$i_\mathrm{sa}(k)$", ds="steps-post"
        )
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
    res : ~motulator.common.model.SimulationResults
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
    set_style(latex)

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
