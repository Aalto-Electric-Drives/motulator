"""Example plotting scripts for grid converters."""

import matplotlib.pyplot as plt
import numpy as np

from motulator.common.model._simulation import SimulationResults
from motulator.common.utils._utils import (
    BaseValues,
    complex2abc,
    set_latex_style,
    set_screen_style,
)


# %%
def plot(
    res: SimulationResults,
    base: BaseValues | None,
    t_span: tuple[float, float] | None = None,
    latex: bool = False,
    plot_pcc_voltage: bool = True,
) -> None:
    """
    Plot example figures.

    Parameters
    ----------
    res : SimulationResults
        Should contain the simulated data.
    base : BaseValues, optional
        Base values for scaling the waveforms. If not given, the waveforms are plotted
        in SI units.
    t_span : 2-tuple, optional
        Time span. If not given, the whole simulation time is plotted.
    plot_pcc_voltage : bool, optional
        If True, plot the phase voltage waveforms at the point of common coupling (PCC).
        Otherwise, plot the grid voltage waveforms, defaults to True.

    """
    # ruff: noqa: PLR0912, PLR0915, PLR0913
    if latex:
        set_latex_style()
        height = plt.rcParams["figure.figsize"][1] * 2.2
    else:
        set_screen_style()
        height = plt.rcParams["figure.figsize"][1] * 1.8

    width = plt.rcParams["figure.figsize"][0]

    # For brevity
    mdl = res.mdl  # Continuous-time data
    ctrl = res.ctrl  # Discrete-time data

    # Check if the time span was given
    if t_span is None:
        t_span = (0, mdl.t[-1])  # Time span

    # Check if the base values were given
    if base is None:
        pu_vals = False
        base = BaseValues.unity()
    else:
        pu_vals = True

    # Three-phase quantities
    i_g_abc = complex2abc(mdl.ac_filter.i_g_ab).T
    e_g_abc = complex2abc(mdl.ac_source.e_g_ab).T
    u_g_abc = complex2abc(mdl.ac_filter.u_g_ab).T

    # Calculation of active and reactive powers
    p_g = 1.5 * np.real(mdl.ac_filter.e_g_ab * np.conj(mdl.ac_filter.i_g_ab))
    q_g = 1.5 * np.imag(mdl.ac_filter.e_g_ab * np.conj(mdl.ac_filter.i_g_ab))

    # Figure 1
    fig1, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(width, height))

    # Subplot 1: Active and reactive power
    ax1.plot(
        ctrl.t,
        ctrl.ref.p_g / base.p,
        "--",
        label=r"$p_\mathrm{g,ref}$",
        ds="steps-post",
    )
    ax1.plot(mdl.t, p_g / base.p, label=r"$p_\mathrm{g}$")
    if hasattr(ctrl.ref, "q_g") and np.any(ctrl.ref.q_g):
        ax1.plot(
            ctrl.t,
            ctrl.ref.q_g / base.p,
            "--",
            label=r"$q_\mathrm{g,ref}$",
            ds="steps-post",
        )
    ax1.plot(mdl.t, q_g / base.p, label=r"$q_\mathrm{g}$")
    ax1.legend()
    ax1.set_xlim(t_span)
    ax1.set_xticklabels([])

    # Subplot 2: Converter currents
    ax2.plot(
        ctrl.t,
        np.real(ctrl.ref.i_c / base.i),
        "--",
        label=r"$i_\mathrm{cd,ref}$",
        ds="steps-post",
    )
    ax2.plot(
        ctrl.t,
        np.real(ctrl.fbk.i_c / base.i),
        label=r"$i_\mathrm{cd}$",
        ds="steps-post",
    )
    ax2.plot(
        ctrl.t,
        np.imag(ctrl.ref.i_c / base.i),
        "--",
        label=r"$i_\mathrm{cq,ref}$",
        ds="steps-post",
    )
    ax2.plot(
        ctrl.t,
        np.imag(ctrl.fbk.i_c / base.i),
        label=r"$i_\mathrm{cq}$",
        ds="steps-post",
    )
    ax2.legend()
    ax2.set_xlim(t_span)
    ax2.set_xticklabels([])

    # Subplot 3: Converter voltage reference, quasi-static converter voltage, and grid
    # voltage magnitudes
    ax3.plot(
        ctrl.t, np.abs(ctrl.fbk.u_c / base.u), label=r"$u_\mathrm{c}$", ds="steps-post"
    )
    if hasattr(ctrl.ref, "v_c"):
        ax3.plot(
            ctrl.t,
            np.abs(ctrl.ref.v_c / base.u),
            label=r"$v_\mathrm{c,ref}$",
            ds="steps-post",
        )
    if hasattr(ctrl.fbk, "v_c"):
        ax3.plot(
            ctrl.t,
            np.abs(ctrl.fbk.v_c / base.u),
            label=r"$\hat{v}_\mathrm{c}$",
            ds="steps-post",
        )
    if np.any(ctrl.ref.u_dc):
        ax3.plot(
            ctrl.t,
            ctrl.ref.u_dc / np.sqrt(3) / base.u,
            "--",
            label=r"$u_\mathrm{dc,ref}/\sqrt{3}$",
            ds="steps-post",
        )
    ax3.plot(
        ctrl.t,
        ctrl.fbk.u_dc / np.sqrt(3) / base.u,
        "--",
        label=r"$u_\mathrm{dc}/\sqrt{3}$",
        ds="steps-post",
    )
    ax3.plot(
        mdl.t, np.abs(mdl.ac_source.e_g_ab / base.u), "--", label=r"$e_\mathrm{g}$"
    )
    _, ymax = ax3.get_ylim()
    ax3.set_ylim(0, ymax)
    ax3.legend()
    ax3.set_xlim(t_span)

    # Add axis labels
    if pu_vals:
        ax1.set_ylabel("Power (p.u.)")
        ax2.set_ylabel("Current (p.u.)")
        ax3.set_ylabel("Voltage (p.u.)")
    else:
        ax1.set_ylabel("Power (kW, kVAr)")
        ax2.set_ylabel("Current (A)")
        ax3.set_ylabel("Voltage (V)")
    ax3.set_xlabel("Time (s)")

    fig1.align_ylabels()

    plt.show()

    # %%
    # Figure 2
    fig2, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(width, height))

    if not plot_pcc_voltage:
        # Subplot 1: Grid voltage
        ax1.plot(
            mdl.t,
            e_g_abc / base.u,
            label=[r"$e_\mathrm{ga}$", r"$e_\mathrm{gb}$", r"$e_\mathrm{gc}$"],
        )
    else:
        # Subplot 1: PCC voltage
        ax1.plot(
            mdl.t,
            u_g_abc / base.u,
            label=[r"$u_\mathrm{ga}$", r"$u_\mathrm{gb}$", r"$u_\mathrm{gc}$"],
        )
    ax1.legend()
    ax1.set_xlim(t_span)
    ax1.set_xticklabels([])

    # Subplot 2: Grid currents
    ax2.plot(
        mdl.t,
        i_g_abc / base.i,
        label=[r"$i_\mathrm{ga}$", r"$i_\mathrm{gb}$", r"$i_\mathrm{gc}$"],
    )
    ax2.legend()
    ax2.set_xlim(t_span)
    ax2.set_xticklabels([])

    # Subplot 3: Phase angles
    ax3.plot(mdl.t, 180 / np.pi * mdl.ac_source.theta_g, label=r"$\theta_\mathrm{g}$")
    ax3.plot(
        ctrl.t,
        180 / np.pi * ctrl.fbk.theta_c,
        "--",
        label=r"$\theta_\mathrm{c}$",
        ds="steps-post",
    )
    ax3.legend()
    ax3.set_xlim(t_span)
    ax3.set_ylim(-180, 180)
    ax3.set_yticks([-180, -90, 0, 90, 180])

    # Add axis labels
    if pu_vals:
        ax1.set_ylabel("Voltage (p.u.)")
        ax2.set_ylabel("Current (p.u.)")
    else:
        ax1.set_ylabel("Voltage (V)")
        ax2.set_ylabel("Current (A)")
    ax3.set_ylabel("Angle (deg)")
    ax3.set_xlabel("Time (s)")

    fig2.align_ylabels()

    plt.show()


# %%
def plot_voltage_vector(res: SimulationResults, base: BaseValues | None) -> None:
    """
    Plot locus of the grid voltage vector.

    Parameters
    ----------
    res : SimulationResults
        Simulation results.
    base : BaseValues, optional
        Base values for scaling the waveforms.

    """

    mdl = res.mdl  # For brevity

    # Check if the base values were given
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
    ticks = [-1.5, -1, -0.5, 0, 0.5, 1, 1.5]
    if pu_vals:
        ax.set_xlabel("Real (p.u.)")
        ax.set_ylabel("Imaginary (p.u.)")
        ax.set_xticks(ticks)
        ax.set_yticks(ticks)
    else:
        ax.set_xlabel("Real (V)")
        ax.set_ylabel("Imaginary (V)")
    ax.legend()
    ax.set_aspect("equal")

    plt.show()
