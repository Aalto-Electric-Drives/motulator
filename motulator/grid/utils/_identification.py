"""Functions and classes for converter output admittance identification."""

import multiprocessing as mp
from collections.abc import Iterable
from contextlib import nullcontext
from copy import deepcopy
from dataclasses import dataclass, field
from functools import partial
from os import environ
from time import time
from typing import Literal

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle
from scipy.integrate import trapezoid

from motulator.common.model import SolverCfg
from motulator.common.utils._plotting import set_style
from motulator.common.utils._utils import empty_array, get_value
from motulator.grid import control, model, utils

STEPS_PER_PERIOD = 10  # Minimum number of solver steps per excitation period


# %%
@dataclass
class IdentificationCfg:
    """
    Configuration parameters for converter output admittance identification.

    Parameters
    ----------
    abs_u_e : float
        Magnitude of the voltage excitation (V) at the first frequency.
    freqs : ndarray, optional
        Excitation frequencies (Hz, in dq-coordinates), defaults to 100 logarithmically
        spaced frequencies from 1 Hz to 10 kHz.
    amplitude_multiplier : float, optional
        Excitation magnitude at the last frequency relative to the first one, defaults
        to 5. The magnitude changes linearly over the frequency array. Use 1 for a
        constant magnitude.
    t0 : float, optional
        Stop time for initial simulating to the operating point (s), defaults to 1.0.
        Should be set large enough to reach steady-state.
    t1 : float, optional
        Additional simulation time for reaching sinusoidal steady-state during signal
        injection (s), defaults to 0.05.
    N_eval : int, optional
        Number of evenly spaced data points the solver should return for each
        integration interval, i.e., for each sampling period or, if carrier comparison
        is used, for each switching interval, defaults to 10.
    n_periods_excitation : int, optional
        Number of excitation signal periods used in calculating the DFT, defaults to 4.
    n_periods_init : int, optional
        Number of fundamental periods to include for calculating operating-point
        vectors, defaults to 1.
    multiprocess : bool, optional
        If True, the frequencies are simulated in parallel using all available CPU
        cores, defaults to True.
    use_window : bool, optional
        Whether to use window function for calculating DFT, defaults to True.

    """

    abs_u_e: float
    freqs: np.ndarray = field(default_factory=lambda: np.geomspace(1, 10e3, 100))
    amplitude_multiplier: float = 5.0
    t0: float = 1.0
    t1: float = 0.05
    N_eval: int = 10
    n_periods_excitation: int = 4
    n_periods_init: int = 1
    multiprocess: bool = True
    use_window: bool = True


@dataclass
class IdentificationResults:
    """
    Container for identification results.

    Contains fields for excitation signal frequency `f_e`, elements of the output
    admittance matrix `[Y_dd, Y_qd; Y_dq, Y_qq]`, and the following operating-point
    vectors (in synchronous coordinates aligned with the PCC voltage): grid current
    `i_g0`, grid voltage `e_g0`, PCC voltage `u_g0` and converter voltage `u_c0`. The
    results can be saved, e.g., by ``scipy.io.savemat(path, dataclasses.asdict(res))``.

    """

    f_e: np.ndarray = field(default_factory=empty_array)
    Y_dd: np.ndarray = field(default_factory=empty_array)
    Y_qd: np.ndarray = field(default_factory=empty_array)
    Y_dq: np.ndarray = field(default_factory=empty_array)
    Y_qq: np.ndarray = field(default_factory=empty_array)
    i_g0: complex = 0j
    e_g0: complex = 0j
    u_g0: complex = 0j
    u_c0: complex = 0j


# %%
def dft(
    cfg: IdentificationCfg,
    t: np.ndarray,
    u: np.ndarray,
    f: float,
    initial_simulation: bool = False,
) -> complex:
    """
    Single-frequency Fourier transform.

    Calculates the complex amplitude of the frequency component f of the signal u over
    the last periods of the signal. The transform is computed with the trapezoidal rule
    from the time instants t, which need not be evenly spaced.

    """
    n_periods = cfg.n_periods_init if initial_simulation else cfg.n_periods_excitation
    T = n_periods / f
    t_start = t[-1] - T

    # Window of the last n_periods, starting from the value interpolated at t_start
    k = np.searchsorted(t, t_start, side="right")
    u_start = np.interp(t_start, t, np.real(u)) + 1j * np.interp(t_start, t, np.imag(u))
    t = np.concatenate(([t_start], t[k:]))
    u = np.concatenate(([u_start], u[k:]))

    if cfg.use_window and not initial_simulation:
        x = 2 * np.pi * (t - t_start) / T
        u = u * (0.42 - 0.5 * np.cos(x) + 0.08 * np.cos(2 * x))  # Blackman window

    return 2 / T * trapezoid(u * np.exp(-2j * np.pi * f * t), t)


def pre_process(
    cfg: IdentificationCfg,
    mdl: model.GridConverterSystem,
    ctrl: control.GridConverterControlSystem,
) -> tuple[
    model.GridConverterSystem, control.GridConverterControlSystem, IdentificationResults
]:
    """Simulate the system to the operating point."""
    # Simulate the original system to the operating point
    w_g = get_value(mdl.ac_source.w_g, 0)
    T_nom = 2 * np.pi / w_g
    t_stop = np.ceil(cfg.t0 / T_nom) * T_nom + cfg.n_periods_init * T_nom
    sim = model.Simulation(deepcopy(mdl), deepcopy(ctrl), show_progress=False)
    res = sim.simulate(t_stop=t_stop, N_eval=cfg.N_eval)

    # Fundamental-frequency vectors in coordinates aligned with the PCC voltage
    f_nom = w_g * 0.5 / np.pi
    t, filt = res.mdl.t, res.mdl.ac_filter
    vectors = [
        0.5 * dft(cfg, t, x, f_nom, initial_simulation=True)
        for x in (filt.i_g_ab, filt.e_g_ab, filt.u_g_ab, filt.u_c_ab)
    ]
    i_g0, e_g0, u_g0, u_c0 = np.exp(-1j * np.angle(vectors[2])) * np.array(vectors)
    operating_point = IdentificationResults(i_g0=i_g0, e_g0=e_g0, u_g0=u_g0, u_c0=u_c0)

    # The PCC voltage is imposed by a stiff source with signal injection at the
    # operating point: the grid impedance is removed, and the source has the PCC
    # voltage magnitude and the initial frequency of the original source
    ac_filter = deepcopy(mdl.ac_filter)
    ac_filter.L_g, ac_filter.R_g = 0.0, 0.0
    ac_source = model.ThreePhaseSourceWithSignalInjection(w_g=w_g, e_g=np.abs(u_g0))
    mdl_inj = model.GridConverterSystem(deepcopy(mdl.converter), ac_filter, ac_source)
    mdl_inj.pwm, mdl_inj.delay = deepcopy(mdl.pwm), deepcopy(mdl.delay)
    ctrl_inj = deepcopy(ctrl)
    sim = model.Simulation(mdl_inj, ctrl_inj, show_progress=False)
    sim.simulate(t_stop=t_stop, N_eval=cfg.N_eval)

    return mdl_inj, ctrl_inj, operating_point


def identify(
    cfg: IdentificationCfg,
    mdl0: model.GridConverterSystem,
    ctrl0: control.GridConverterControlSystem,
    excitation: tuple[float, float],
) -> np.ndarray:
    """Compute the output admittance matrix at a single frequency."""
    f_e, abs_u_e = excitation
    U, I = np.zeros((2, 2), complex), np.zeros((2, 2), complex)  # noqa: E741
    # Inject the excitation in the d- and q-axes, one at a time
    for k, u_e in enumerate((abs_u_e, 1j * abs_u_e)):
        mdl, ctrl = deepcopy(mdl0), deepcopy(ctrl0)
        source = mdl.ac_source
        assert isinstance(source, model.ThreePhaseSourceWithSignalInjection)
        source.f_e, source.u_e = f_e, u_e
        t_stop = mdl.t0 + cfg.t1 + cfg.n_periods_excitation / f_e
        # The excitation is computed analytically, so the step size is limited to
        # resolve its response, also at frequencies near the sampling frequency
        solver_cfg = SolverCfg(max_step=1 / (STEPS_PER_PERIOD * f_e))
        sim = model.Simulation(mdl, ctrl, show_progress=False, cfg=solver_cfg)
        res = sim.simulate(t_stop=t_stop, N_eval=cfg.N_eval)
        # Voltage and current in synchronous coordinates
        exp_j_theta_g = res.mdl.ac_source.exp_j_theta_g
        u_g = np.conj(exp_j_theta_g) * res.mdl.ac_filter.u_g_ab
        i_g = np.conj(exp_j_theta_g) * res.mdl.ac_filter.i_g_ab
        t = res.mdl.t
        U[:, k] = dft(cfg, t, u_g.real, f_e), dft(cfg, t, u_g.imag, f_e)
        I[:, k] = dft(cfg, t, i_g.real, f_e), dft(cfg, t, i_g.imag, f_e)
    return -I @ np.linalg.inv(U)


# %%
def run_identification(
    cfg: IdentificationCfg,
    mdl: model.GridConverterSystem,
    ctrl: control.GridConverterControlSystem,
) -> IdentificationResults:
    """
    Run the identification.

    The system is first simulated to the operating point. Then, the output admittance
    of the converter is identified at the PCC: the PCC voltage is imposed by a stiff
    voltage source, whose magnitude and frequency equal those of the operating point,
    and small-signal voltage excitations are injected in the d- and q-axes. The grid
    impedance of `mdl` as well as any time variation and negative-sequence component of
    its source only affect the operating point.

    Parameters
    ----------
    cfg : IdentificationCfg
        Identification configuration.
    mdl : GridConverterSystem
        Continuous-time system model.
    ctrl : GridConverterControlSystem
        Discrete-time control system.

    Returns
    -------
    IdentificationResults
        Admittance matrix elements and the operating point.

    Notes
    -----
    With carrier-comparison PWM, background switching harmonics can bias the identified
    admittance at the excitation frequency. Results near switching harmonics and their
    sidebands may depend strongly on the excitation amplitude and its phase relative to
    the carrier, even when the numerical solution has converged.

    """
    mdl_inj, ctrl_inj, res = pre_process(cfg, mdl, ctrl)
    show_progress = environ.get("BUILDING_DOCS") != "1"
    if show_progress:
        print("Start identification...")
    t_start = time()

    n_freqs = np.size(cfg.freqs)
    amplitudes = cfg.abs_u_e * np.linspace(1.0, cfg.amplitude_multiplier, n_freqs)
    excitations = zip(cfg.freqs, amplitudes, strict=True)
    task = partial(identify, cfg, mdl_inj, ctrl_inj)
    Y = []
    with mp.Pool() if cfg.multiprocess else nullcontext() as pool:
        # Results in the order of the frequencies
        results: Iterable[np.ndarray] = (
            pool.imap(task, excitations) if pool else map(task, excitations)
        )
        for k, Y_k in enumerate(results, start=1):
            Y.append(Y_k)
            if show_progress:
                print(f"\rFrequencies simulated: {k}/{n_freqs}", end="")
    if show_progress:
        print(f"\nExecution time: {(time() - t_start):.2f} s")

    Y_c = np.array(Y)
    res.f_e = np.array(cfg.freqs, dtype=float)  # A copy, independent of cfg
    res.Y_dd, res.Y_qd = Y_c[:, 0, 0], Y_c[:, 0, 1]
    res.Y_dq, res.Y_qq = Y_c[:, 1, 0], Y_c[:, 1, 1]
    return res


# %%
def plot_identification(
    res: IdentificationResults,
    plot_style: Literal["polar", "re_im"] = "re_im",
    plot_passivity_index: bool = True,
    latex: bool = False,
) -> None:
    """
    Plot the identification results

    Parameters
    ----------
    res : IdentificationResults
        Should contain the results from the identification.
    plot_style : Literal["polar", "re_im"], optional
        Style for plotting of identification results, defaults to "re_im". Options are:
        - "polar": plot magnitude and phase
        - "re_im": plot real and imaginary parts
    plot_passivity_index : bool, optional
        Plot input feedforward passivity index calculated from the identification
        results, defaults to True.
    latex : bool, optional
        Use latex for plots, defaults to False.

    """
    # ruff: noqa: PLR0915
    set_style(latex)

    # First figure: elements of output admittance matrix
    if plot_style == "polar":
        fig, ((ax1, ax5), (ax2, ax6), (ax3, ax7), (ax4, ax8)) = plt.subplots(
            4, 2, sharey="row"
        )

        ax1.loglog(res.f_e, np.abs(res.Y_dd))
        ax2.semilogx(
            res.f_e, np.unwrap(np.angle(res.Y_dd, deg=True), discont=180, period=360)
        )
        ax3.loglog(res.f_e, np.abs(res.Y_dq))
        ax4.semilogx(
            res.f_e, np.unwrap(np.angle(res.Y_dq, deg=True), discont=180, period=360)
        )
        ax5.loglog(res.f_e, np.abs(res.Y_qd))
        ax6.semilogx(
            res.f_e, np.unwrap(np.angle(res.Y_qd, deg=True), discont=180, period=360)
        )
        ax7.loglog(res.f_e, np.abs(res.Y_qq))
        ax8.semilogx(
            res.f_e, np.unwrap(np.angle(res.Y_qq, deg=True), discont=180, period=360)
        )

        ax1.set_ylabel(r"$|Y_\mathrm{dd}|\ (\mathrm{S})$")
        ax2.set_ylabel(r"$\angle Y_\mathrm{dd}\ (\mathrm{deg})$")
        ax3.set_ylabel(r"$|Y_\mathrm{dq}|\ (\mathrm{S})$")
        ax4.set_ylabel(r"$\angle Y_\mathrm{dq}\ (\mathrm{deg})$")
        ax5.set_ylabel(r"$|Y_\mathrm{qd}|\ (\mathrm{S})$")
        ax6.set_ylabel(r"$\angle Y_\mathrm{qd}\ (\mathrm{deg})$")
        ax7.set_ylabel(r"$|Y_\mathrm{qq}|\ (\mathrm{S})$")
        ax8.set_ylabel(r"$\angle Y_\mathrm{qq}\ (\mathrm{deg})$")

        ax5.tick_params(axis="y", labelleft=True)
        ax6.tick_params(axis="y", labelleft=True)
        ax7.tick_params(axis="y", labelleft=True)
        ax8.tick_params(axis="y", labelleft=True)

        ymin1, ymax1 = ax1.get_ylim()
        ymin2, ymax2 = ax2.get_ylim()
        ymin3, ymax3 = ax3.get_ylim()
        ymin4, ymax4 = ax4.get_ylim()

        ylim_magn = (min(ymin1, ymin3), max(ymax1, ymax3))
        ax1.set_ylim(ylim_magn)
        ax3.set_ylim(ylim_magn)

        ylim_phase = (min(ymin2, ymin4), max(ymax2, ymax4))
        ax2.set_ylim(ylim_phase)
        ax4.set_ylim(ylim_phase)

    else:
        fig, ((ax1, ax5), (ax2, ax6), (ax3, ax7), (ax4, ax8)) = plt.subplots(
            4, 2, sharex=True, sharey=True
        )

        ax1.semilogx(res.f_e, np.real(res.Y_dd))
        ax2.semilogx(res.f_e, np.imag(res.Y_dd))
        ax3.semilogx(res.f_e, np.real(res.Y_dq))
        ax4.semilogx(res.f_e, np.imag(res.Y_dq))
        ax5.semilogx(res.f_e, np.real(res.Y_qd))
        ax6.semilogx(res.f_e, np.imag(res.Y_qd))
        ax7.semilogx(res.f_e, np.real(res.Y_qq))
        ax8.semilogx(res.f_e, np.imag(res.Y_qq))

        ax1.set_ylabel(r"$\mathrm{Re}\{Y_\mathrm{dd}\}\ (\mathrm{S})$")
        ax2.set_ylabel(r"$\mathrm{Im}\{Y_\mathrm{dd}\}\ (\mathrm{S})$")
        ax3.set_ylabel(r"$\mathrm{Re}\{Y_\mathrm{dq}\}\ (\mathrm{S})$")
        ax4.set_ylabel(r"$\mathrm{Im}\{Y_\mathrm{dq}\}\ (\mathrm{S})$")
        ax5.set_ylabel(r"$\mathrm{Re}\{Y_\mathrm{qd}\}\ (\mathrm{S})$")
        ax6.set_ylabel(r"$\mathrm{Im}\{Y_\mathrm{qd}\}\ (\mathrm{S})$")
        ax7.set_ylabel(r"$\mathrm{Re}\{Y_\mathrm{qq}\}\ (\mathrm{S})$")
        ax8.set_ylabel(r"$\mathrm{Im}\{Y_\mathrm{qq}\}\ (\mathrm{S})$")

        ax5.tick_params(axis="y", labelleft=True)
        ax6.tick_params(axis="y", labelleft=True)
        ax7.tick_params(axis="y", labelleft=True)
        ax8.tick_params(axis="y", labelleft=True)

    ax4.set_xlabel(r"Frequency (Hz)")
    ax8.set_xlabel(r"Frequency (Hz)")

    ax1.margins(x=0)
    ax2.margins(x=0)
    ax3.margins(x=0)
    ax4.margins(x=0)
    ax5.margins(x=0)
    ax6.margins(x=0)
    ax7.margins(x=0)
    ax8.margins(x=0)

    fig.align_ylabels()
    plt.show()

    # Second figure: passivity index
    if plot_passivity_index:
        _, ax1 = plt.subplots(1, 1, figsize=(4, 3))

        Y_c = np.array([[res.Y_dd, res.Y_qd], [res.Y_dq, res.Y_qq]])
        Y_c = np.moveaxis(Y_c, -1, 0)
        nu_F = 0.5 * np.min(np.linalg.eigvals(Y_c + np.matrix_transpose(Y_c.conj())), 1)

        ax1.axhline(0, linestyle="--", color="k", linewidth="1")
        ax1.semilogx(res.f_e, nu_F.real)
        ax1.set_xlabel("Frequency (Hz)")
        ax1.set_ylabel("Passivity index")
        ax1.margins(x=0)

        plt.show()


def plot_vector_identification(
    res: IdentificationResults, base: utils.BaseValues, latex: bool = False
) -> None:
    """
    Plot the converter voltage, PCC voltage, grid voltage, and grid current vectors in
    the operating point.

    Parameters
    ----------
    res : IdentificationResults
        Should contain the results from the identification.
    base : BaseValues
        Base values for per-unit conversion.
    latex : bool, optional
        Use latex for plots, defaults to False.

    """
    set_style(latex)

    _, ax = plt.subplots()
    ax.grid(True, zorder=0)
    circle = Circle((0, 0), 1, fill=False, edgecolor="gray", zorder=1)
    ax.add_patch(circle)

    ax.quiver(
        0,
        0,
        res.e_g0.real / base.u,
        res.e_g0.imag / base.u,
        angles="xy",
        scale_units="xy",
        scale=1,
        color="blue",
        label=r"$\mathbf{e}_\mathrm{g0}$",
        zorder=2,
    )
    ax.quiver(
        0,
        0,
        res.u_g0.real / base.u,
        res.u_g0.imag / base.u,
        angles="xy",
        scale_units="xy",
        scale=1,
        color="red",
        label=r"$\mathbf{u}_\mathrm{g0}$",
        zorder=2,
    )
    ax.quiver(
        0,
        0,
        res.u_c0.real / base.u,
        res.u_c0.imag / base.u,
        angles="xy",
        scale_units="xy",
        scale=1,
        color="black",
        label=r"$\mathbf{u}_\mathrm{c0}$",
        zorder=2,
    )
    ax.quiver(
        0,
        0,
        res.i_g0.real / base.i,
        res.i_g0.imag / base.i,
        angles="xy",
        scale_units="xy",
        scale=1,
        color="green",
        label=r"$\mathbf{i}_\mathrm{g0}$",
        zorder=2,
    )
    ticks = [-1, -0.5, 0, 0.5, 1]

    ax.set_xlabel("d (p.u.)")
    ax.set_ylabel("q (p.u.)")
    ax.set_xticks(ticks)
    ax.set_yticks(ticks)
    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(-1.2, 1.2)

    ax.legend(loc="upper left")
    ax.set_aspect("equal")

    plt.show()
