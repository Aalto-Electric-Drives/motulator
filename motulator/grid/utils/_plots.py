"""Example plotting scripts for grid converters."""

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
def _plot_powers(ax, ctrl, mdl, base: BaseValues) -> None:
    """Plot active and reactive powers."""
    # Calculate powers
    p_g = 1.5 * np.real(mdl.ac_filter.e_g_ab * np.conj(mdl.ac_filter.i_g_ab))
    q_g = 1.5 * np.imag(mdl.ac_filter.e_g_ab * np.conj(mdl.ac_filter.i_g_ab))

    ax.plot(
        ctrl.t,
        ctrl.ref.p_g / base.p,
        "--",
        label=r"$p_\mathrm{g}^\mathrm{ref}$",
        ds="steps-post",
    )
    ax.plot(mdl.t, p_g / base.p, label=r"$p_\mathrm{g}$")

    if hasattr(ctrl.ref, "q_g") and np.any(ctrl.ref.q_g):
        ax.plot(
            ctrl.t,
            ctrl.ref.q_g / base.p,
            "--",
            label=r"$q_\mathrm{g}^\mathrm{ref}$",
            ds="steps-post",
        )
    ax.plot(mdl.t, q_g / base.p, label=r"$q_\mathrm{g}$")
    ax.legend()


def _plot_currents(ax, ctrl, base: BaseValues) -> None:
    """Plot converter currents."""
    ax.plot(
        ctrl.t,
        np.real(ctrl.ref.i_c / base.i),
        "--",
        label=r"$i_\mathrm{cd}^\mathrm{ref}$",
        ds="steps-post",
    )
    ax.plot(
        ctrl.t,
        np.real(ctrl.fbk.i_c / base.i),
        label=r"$i_\mathrm{cd}$",
        ds="steps-post",
    )
    ax.plot(
        ctrl.t,
        np.imag(ctrl.ref.i_c / base.i),
        "--",
        label=r"$i_\mathrm{cq}^\mathrm{ref}$",
        ds="steps-post",
    )
    ax.plot(
        ctrl.t,
        np.imag(ctrl.fbk.i_c / base.i),
        label=r"$i_\mathrm{cq}$",
        ds="steps-post",
    )
    ax.legend()


def _plot_voltages(ax, ctrl, mdl, base: BaseValues) -> None:
    """Plot voltages."""
    ax.plot(
        ctrl.t,
        np.abs(ctrl.fbk.u_c / base.u),
        label=r"$\hat{u}_\mathrm{c}$",
        ds="steps-post",
    )

    if hasattr(ctrl.ref, "v_c"):
        ax.plot(
            ctrl.t,
            np.abs(ctrl.ref.v_c / base.u),
            label=r"$v_\mathrm{c}^\mathrm{ref}$",
            ds="steps-post",
        )

    if hasattr(ctrl.fbk, "v_c"):
        ax.plot(
            ctrl.t,
            np.abs(ctrl.fbk.v_c / base.u),
            label=r"$\hat{v}_\mathrm{c}$",
            ds="steps-post",
        )

    if np.any(ctrl.ref.u_dc):
        ax.plot(
            ctrl.t,
            ctrl.ref.u_dc / np.sqrt(3) / base.u,
            "--",
            label=r"$u_\mathrm{dc}^\mathrm{ref}/\sqrt{3}$",
            ds="steps-post",
        )

    ax.plot(
        ctrl.t,
        ctrl.fbk.u_dc / np.sqrt(3) / base.u,
        "--",
        label=r"$u_\mathrm{dc}/\sqrt{3}$",
        ds="steps-post",
    )
    ax.plot(mdl.t, np.abs(mdl.ac_source.e_g_ab / base.u), "--", label=r"$e_\mathrm{g}$")

    _, ymax = ax.get_ylim()
    ax.set_ylim(0, ymax)
    ax.legend()


def plot_control_signals(
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
    Plot control signals and converter voltages.

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
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(width, height))
    axes = [ax1, ax2, ax3]

    # Plot subplots
    _plot_powers(ax1, res.ctrl, res.mdl, base)
    _plot_currents(ax2, res.ctrl, base)
    _plot_voltages(ax3, res.ctrl, res.mdl, base)

    # Configure all axes
    configure_axes(axes, t_lims, t_ticks, y_lims, y_ticks)

    # Remove xticklabels for all but the last subplot
    for ax in axes[:-1]:
        ax.set_xticklabels([])
    ax3.set_xlabel("Time (s)")

    # Add axis labels
    if pu_vals:
        ax1.set_ylabel("Power (p.u.)")
        ax2.set_ylabel("Current (p.u.)")
        ax3.set_ylabel("Voltage (p.u.)")
    else:
        ax1.set_ylabel("Power (W)")
        ax2.set_ylabel("Current (A)")
        ax3.set_ylabel("Voltage (V)")
    fig.align_ylabels()

    save_and_show(save_path, **savefig_kwargs)


# %%
def _plot_three_phase_voltages(
    ax, mdl, base: BaseValues, plot_pcc_voltage: bool
) -> None:
    """Plot three-phase voltages."""
    if plot_pcc_voltage:
        u_g_abc = complex2abc(mdl.ac_filter.u_g_ab).T
        ax.plot(
            mdl.t,
            u_g_abc / base.u,
            label=[r"$u_\mathrm{ga}$", r"$u_\mathrm{gb}$", r"$u_\mathrm{gc}$"],
        )
    else:
        e_g_abc = complex2abc(mdl.ac_source.e_g_ab).T
        ax.plot(
            mdl.t,
            e_g_abc / base.u,
            label=[r"$e_\mathrm{ga}$", r"$e_\mathrm{gb}$", r"$e_\mathrm{gc}$"],
        )
    ax.legend()


def _plot_three_phase_currents(ax, mdl, base: BaseValues) -> None:
    """Plot three-phase currents."""
    i_g_abc = complex2abc(mdl.ac_filter.i_g_ab).T
    ax.plot(
        mdl.t,
        i_g_abc / base.i,
        label=[r"$i_\mathrm{ga}$", r"$i_\mathrm{gb}$", r"$i_\mathrm{gc}$"],
    )
    ax.legend()


def _plot_phase_angles(ax, mdl, ctrl) -> None:
    """Plot phase angles."""
    ax.plot(mdl.t, 180 / np.pi * mdl.ac_source.theta_g, label=r"$\theta_\mathrm{g}$")
    ax.plot(
        ctrl.t,
        180 / np.pi * ctrl.fbk.theta_c,
        "--",
        label=r"$\theta_\mathrm{c}$",
        ds="steps-post",
    )
    ax.legend()
    ax.set_ylim(-180, 180)
    ax.set_yticks([-180, -90, 0, 90, 180])


def plot_grid_waveforms(
    res: SimulationResults,
    base: BaseValues | None = None,
    t_lims: tuple[float, float] | None = None,
    t_ticks: ArrayLike | None = None,
    y_lims: list[tuple[float, float] | None] | None = None,
    y_ticks: list[ArrayLike | None] | None = None,
    latex: bool = False,
    plot_pcc_voltage: bool = True,
    save_path: str | Path | None = None,
    **savefig_kwargs,
) -> None:
    """
    Plot grid waveforms and phase angles.

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
    plot_pcc_voltage : bool, optional
        If True, plot the phase voltage waveforms at the point of common coupling (PCC).
        Otherwise, plot the grid voltage waveforms, defaults to True.
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
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(width, height))
    axes = [ax1, ax2, ax3]

    # Plot subplots
    _plot_three_phase_voltages(ax1, res.mdl, base, plot_pcc_voltage)
    _plot_three_phase_currents(ax2, res.mdl, base)
    _plot_phase_angles(ax3, res.mdl, res.ctrl)

    # Configure all axes
    configure_axes(axes, t_lims, t_ticks, y_lims, y_ticks)

    # Remove xticklabels for all but the last subplot
    for ax in axes[:-1]:
        ax.set_xticklabels([])
    ax3.set_xlabel("Time (s)")

    # Add axis labels
    if pu_vals:
        ax1.set_ylabel("Voltage (p.u.)")
        ax2.set_ylabel("Current (p.u.)")
    else:
        ax1.set_ylabel("Voltage (V)")
        ax2.set_ylabel("Current (A)")
    ax3.set_ylabel("Angle (deg)")
    fig.align_ylabels()

    save_and_show(save_path, **savefig_kwargs)


# %%
def plot_voltage_vector(
    res: SimulationResults,
    base: BaseValues | None = None,
    save_path: str | Path | None = None,
    **savefig_kwargs,
) -> None:
    """
    Plot locus of the grid voltage vector.

    Parameters
    ----------
    res : SimulationResults
        Simulation results.
    base : BaseValues, optional
        Base values for scaling the waveforms. If not given, the waveforms are plotted
        in SI units.
    save_path : str | Path, optional
        Path to save the figure. If None, the figure is not saved.
    **savefig_kwargs
        Additional keyword arguments passed to plt.savefig().

    """
    mdl = res.mdl  # For brevity

    # Process base values
    if base is None:
        pu_vals = False
        base = BaseValues.unity()
        base.p = 1000  # For power use kW
    else:
        pu_vals = True

    # Plot the grid voltage vector in the complex plane
    _, ax = plt.subplots()
    ax.plot(
        mdl.ac_source.e_g_ab.real / base.u,
        mdl.ac_source.e_g_ab.imag / base.u,
        label="Grid voltage",
    )
    ax.axhline(0, color="k")
    ax.axvline(0, color="k")
    if pu_vals:
        ax.set_xlabel("Real (p.u.)")
        ax.set_ylabel("Imaginary (p.u.)")
        ticks = [-1.5, -1, -0.5, 0, 0.5, 1, 1.5]
        ax.set_xticks(ticks)
        ax.set_yticks(ticks)
    else:
        ax.set_xlabel("Real (V)")
        ax.set_ylabel("Imaginary (V)")
    ax.legend()
    ax.set_aspect("equal")

    save_and_show(save_path, **savefig_kwargs)
