"""Functions and classes for converter output admittance identification."""

import multiprocessing as mp
from copy import deepcopy
from dataclasses import dataclass, field
from datetime import datetime
from os import environ, makedirs
from os.path import exists, join
from time import time
from typing import Any, Literal

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle
from scipy.io import savemat
from scipy.signal.windows import blackman

from motulator.common.model._pwm import CarrierComparison
from motulator.common.utils._plotting import set_latex_style, set_screen_style
from motulator.common.utils._utils import empty_array, get_value
from motulator.grid import control, model, utils


# %%
@dataclass
class IdentificationCfg:
    """
    Configuration parameters for converter output admittance identification.

    Parameters
    ----------
    abs_u_e : float
        Magnitude of the voltage excitation (V).
    f_start : float, optional
        Starting frequency of the voltage excitation (Hz, in dq-coordinates), defaults
        to 1.
    f_stop : float, optional
        End frequency of the voltage excitation (Hz, in dq-coordinates), defaults to
        10e3.
    n_freqs : int, optional
        Number of frequencies for measurement, defaults to 100.
    spacing : Literal["log", "lin"], optional
        The spacing used for creating the array of measurement frequencies, defaults to
        "log". Valid options are:
        - "log": logarithmic spacing
        - "lin": linear spacing
    manual_freqs : ndarray | None, optional
        Manually specified array of frequencies (Hz) to measure admittance at. If set to
        None, f_start, f_stop and n_freqs parameters are used to create the array of
        frequencies. Defaults to None.
    t0 : float, optional
        Stop time for initial simulating to the operating point (s), defaults to 1.0.
        Should be set large enough to reach steady-state.
    t1 : float, optional
        Additional simulation time for reaching sinusoidal steady-state during signal
        injection (s), defaults to 0.05.
    T_s : float, optional
        Sampling period of the control system (s), defaults to 125e-6.
    N_eval : int, optional
        Number of evenly spaced data points the solver should return for each controller
        sampling period, defaults to 10.
    n_periods_excitation : int, optional
        Number of excitation signal periods used in calculating the DFT, defaults to 4.
    n_periods_init : int, optional
        Number of fundamental periods to include for calculating operating-point
        vectors, defaults to 1.
    multiprocess : bool, optional
        If set to True, multiprocessing.Pool() is used to run the identification using
        all available CPU cores in the system. Defaults to True.
    filename : str | None, optional
        If given, the identification result is saved in
        ``data/{date}_{time}_{filename}`` (relative to the project root directory),
        defaults to None. The file format is set by the `filetype` parameter.
    filetype : Literal["csv", "mat"], optional
        The filetype for saving identification results, defaults to "csv". Valid options
        are:
        - "csv": save results in .csv-format
        - "mat": save results in MATLAB .mat-format
    delay : int, optional
        Number of samples for modeling the computational delay, defaults to 1.
    use_window : bool, optional
        Whether to use window function for calculating DFT, defaults to True.
    variable_amplitude : bool, optional
        Whether to increase the excitation signal amplitude with the frequency, defaults
        to True.
    amplitude_multiplier : float, optional
        Gain value to set how much the excitation signal amplitude is increased at the
        highest frequency compared to the lowest if `variable_amplitude` is set to True.
        Defaults to 5.0.

    """

    abs_u_e: float
    f_start: float = 1
    f_stop: float = 10e3
    n_freqs: int = 100
    spacing: Literal["log", "lin"] = "log"
    manual_freqs: np.ndarray | None = None
    t0: float = 1.0
    t1: float = 0.05
    T_s: float = 125e-6
    N_eval: int = 10
    n_periods_excitation: int = 4
    n_periods_init: int = 1
    multiprocess: bool = True
    filename: str | None = None
    filetype: Literal["csv", "mat"] = "csv"
    delay: int = 1
    use_window: bool = True
    variable_amplitude: bool = True
    amplitude_multiplier: float = 5.0

    def __post_init__(self) -> None:
        # Create array of frequencies if not specified
        if self.manual_freqs is None:
            if self.spacing == "lin":
                self.freqs = np.linspace(self.f_start, self.f_stop, self.n_freqs)
            else:
                self.freqs = np.geomspace(self.f_start, self.f_stop, self.n_freqs)
        else:
            self.freqs = self.manual_freqs
        # Increase excitation signal amplitude with the frequency if configured
        if self.variable_amplitude:
            self.amplitudes = self.abs_u_e * np.linspace(
                1.0, self.amplitude_multiplier, np.size(self.freqs)
            )
        else:
            self.amplitudes = np.full(np.size(self.freqs), self.abs_u_e)


@dataclass
class IdentificationResults:
    """
    Container for identification results.

    Contains fields for excitation signal frequency `f_e`, elements of the output
    admittance matrix `[Y_dd, Y_qd; Y_dq, Y_qq]`, and the following operating-point
    vectors (in synchronous coordinates aligned with the PCC voltage): grid current
    `i_g0`, grid voltage `e_g0`, PCC voltage `u_g0` and converter voltage `u_c0`.

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
def save_csv(data: IdentificationResults, filename: str) -> None:
    """Save the identification results in a .csv-file."""
    try:
        # Create the data directory if it doesn't exist
        makedirs("data", exist_ok=True)

        # Create the file path
        timestamp = datetime.now().strftime("%Y%m%d_%H.%M_")
        filepath = join("data", timestamp + filename + ".csv")

        # Check if file already exists
        if exists(filepath):
            print("Warning: Overwriting already existing file")

        # Get field names from the IdentificationResults dataclass
        field_names = list(data.__dataclass_fields__.keys())

        # Save data to CSV file
        with open(filepath, "w", encoding="utf-8") as f:
            f.write(",".join(field_names) + "\n")
            n_rows = len(data.f_e)

            for i in range(n_rows):
                row = []
                for field_name in field_names:
                    value = getattr(data, field_name)

                    # Only write scalar values once in the first row
                    if isinstance(value, np.ndarray):
                        row.append(str(value[i]))
                    else:
                        row.append(str(value) if i == 0 else "")

                f.write(",".join(row) + "\n")

        print(f"Data successfully exported to {timestamp + filename}")

    except Exception as error:
        print(f"Error saving data: {str(error)}")


def save_mat(data: IdentificationResults, filename: str) -> None:
    """Save the identification results in a .mat-file."""
    try:
        # Create the data directory if it doesn't exist
        makedirs("data", exist_ok=True)

        # Convert the data class to dict
        data_dict = dict(data.__dict__.items())
        # Create the file path
        timestamp = datetime.now().strftime("%Y%m%d_%H.%M_")
        filepath = join("data", timestamp + filename + ".mat")

        # Check if file already exists
        if exists(filepath):
            print("Warning: Overwriting already existing file")

        savemat(filepath, data_dict)
        print(f"Data successfully exported to {timestamp + filename}")

    except Exception as error:
        print(f"Error saving data: {str(error)}")


def dft(
    cfg: IdentificationCfg, u: np.ndarray, f: float, initial_simulation: bool = False
) -> complex:
    """
    Single-frequency discrete Fourier transform.

    Calculates the frequency component y at frequency f from input signal u, using the
    discrete Fourier transform algorithm.

    """
    n_periods = cfg.n_periods_init if initial_simulation else cfg.n_periods_excitation
    n = int(n_periods * cfg.N_eval / (f * cfg.T_s))

    if cfg.use_window and not initial_simulation:
        u = u[-n:] * blackman(n, False)
    else:
        u = u[-n:]

    w = 2 * np.pi * f
    y = 2 / n * np.sum(u * np.exp(-1j * w * cfg.T_s / cfg.N_eval * np.arange(n)))
    return y


def copy_state(sim: model.Simulation) -> tuple[Any, Any]:
    """Make a copy of the simulation state."""
    mdl = deepcopy(sim.mdl)
    ctrl = deepcopy(sim.ctrl)
    return mdl, ctrl


def pre_process(
    cfg: IdentificationCfg,
    mdl: model.GridConverterSystem,
    ctrl: control.GridConverterControlSystem,
) -> tuple[model.Simulation, list[Any]]:
    """Simulate the system to the desired operating point."""
    # Create Simulation object and simulate
    w_g = get_value(mdl.ac_source.w_g, 0)
    T_nom = 2 * np.pi / w_g
    t_stop = np.ceil(cfg.t0 / T_nom) * T_nom + cfg.n_periods_init * T_nom
    sim = model.Simulation(deepcopy(mdl), deepcopy(ctrl), show_progress=False)
    res = sim.simulate(t_stop=t_stop, N_eval=cfg.N_eval)

    # Calculate fundamental-frequency quantities in the operating point
    f_nom = w_g * 0.5 / np.pi
    i_g0 = 0.5 * (dft(cfg, res.mdl.ac_filter.i_g_ab, f_nom, initial_simulation=True))
    e_g0 = 0.5 * (dft(cfg, res.mdl.ac_filter.e_g_ab, f_nom, initial_simulation=True))
    u_g0 = 0.5 * (dft(cfg, res.mdl.ac_filter.u_g_ab, f_nom, initial_simulation=True))
    u_c0 = 0.5 * (dft(cfg, res.mdl.ac_filter.u_c_ab, f_nom, initial_simulation=True))

    # Align coordinates with PCC voltage vector
    theta = np.angle(u_g0)
    i_g0 = i_g0 * np.exp(-1j * theta)
    e_g0 = e_g0 * np.exp(-1j * theta)
    u_g0 = u_g0 * np.exp(-1j * theta)
    u_c0 = u_c0 * np.exp(-1j * theta)

    # Create new system model and simulate to steady state
    converter = deepcopy(mdl.converter)
    ac_filter = deepcopy(mdl.ac_filter)
    ac_filter.L_g = 0.0
    ac_filter.R_g = 0.0
    ac_source = model.ThreePhaseSourceWithSignalInjection(w_g=w_g, e_g=np.abs(u_g0))
    pwm = isinstance(mdl.pwm, CarrierComparison)
    mdl = model.GridConverterSystem(
        converter, ac_filter, ac_source, pwm=pwm, delay=cfg.delay
    )
    sim = model.Simulation(mdl, ctrl, show_progress=False)
    res = sim.simulate(t_stop=t_stop, N_eval=cfg.N_eval)

    operating_point = [i_g0, e_g0, u_g0, u_c0]

    return sim, operating_point


def identify(
    cfg: IdentificationCfg, sim: model.Simulation, i: int, f_e: float
) -> list[Any]:
    """Calculate the output admittance at a single frequency."""
    # 1: d-axis injection
    mdl, ctrl = copy_state(sim)
    mdl.ac_source.f_e = f_e
    mdl.ac_source.u_ed = cfg.amplitudes[i]
    # Set new stop time and simulate
    t_stop = mdl.t0 + cfg.t1 + cfg.n_periods_excitation / f_e
    sim_d = model.Simulation(mdl, ctrl, show_progress=False)
    res_d = sim_d.simulate(t_stop=t_stop, N_eval=cfg.N_eval)

    # Transform the voltage and current to synchronous coordinates and
    # calculate the DFT
    u_g1 = np.conj(res_d.mdl.ac_source.exp_j_theta_g) * res_d.mdl.ac_filter.u_g_ab
    u_gd1 = dft(cfg, u_g1.real, f_e)
    u_gq1 = dft(cfg, u_g1.imag, f_e)
    i_g1 = np.conj(res_d.mdl.ac_source.exp_j_theta_g) * res_d.mdl.ac_filter.i_g_ab
    i_gd1 = dft(cfg, i_g1.real, f_e)
    i_gq1 = dft(cfg, i_g1.imag, f_e)

    # 2: q-axis injection
    mdl, ctrl = copy_state(sim)
    mdl.ac_source.f_e = f_e
    mdl.ac_source.u_eq = cfg.amplitudes[i]
    sim_q = model.Simulation(mdl, ctrl, show_progress=False)
    res_q = sim_q.simulate(t_stop=t_stop, N_eval=cfg.N_eval)

    # DFT
    u_g2 = np.conj(res_q.mdl.ac_source.exp_j_theta_g) * res_q.mdl.ac_filter.u_g_ab
    u_gd2 = dft(cfg, u_g2.real, f_e)
    u_gq2 = dft(cfg, u_g2.imag, f_e)
    i_g2 = np.conj(res_q.mdl.ac_source.exp_j_theta_g) * res_q.mdl.ac_filter.i_g_ab
    i_gd2 = dft(cfg, i_g2.real, f_e)
    i_gq2 = dft(cfg, i_g2.imag, f_e)

    # Calculate the elements of the output admittance matrix
    I = np.array([[i_gd1, i_gd2], [i_gq1, i_gq2]])  # noqa: E741
    U = np.array([[u_gd1, u_gd2], [u_gq1, u_gq2]])
    inv_U = np.linalg.inv(U)
    Y_c = -I @ inv_U

    Y_dd = Y_c[0, 0]
    Y_qd = Y_c[0, 1]
    Y_dq = Y_c[1, 0]
    Y_qq = Y_c[1, 1]
    return [i, f_e, Y_dd, Y_qd, Y_dq, Y_qq]


def post_process(
    results: list[list[Any]], operating_point: list[Any]
) -> IdentificationResults:
    """Save the identification results and information about the operating point."""
    results_array = np.vstack(results)
    res = IdentificationResults(
        f_e=np.real(results_array[:, 1]),
        Y_dd=results_array[:, 2],
        Y_qd=results_array[:, 3],
        Y_dq=results_array[:, 4],
        Y_qq=results_array[:, 5],
        i_g0=operating_point[0],
        e_g0=operating_point[1],
        u_g0=operating_point[2],
        u_c0=operating_point[3],
    )
    return res


# %%
def run_identification(
    cfg: IdentificationCfg,
    mdl: model.GridConverterSystem,
    ctrl: control.GridConverterControlSystem,
) -> IdentificationResults:
    """
    Run the identification.

    Parameters
    ----------
    cfg : IdentificationCfg
        Dataclass object containing the identification configuration.
    mdl : GridConverterSystem
        Continuous-time system model object.
    ctrl : GridConverterControlSystem
        Discrete-time control system object.

    Returns
    -------
    res : IdentificationResults
        Dataclass object containing the results from the identification, along with
        information about the operating point.

    """
    results = []
    sim_op, operating_point = pre_process(cfg, mdl, ctrl)
    t_start = time()

    show_progress = False if environ.get("BUILDING_DOCS") == "1" else True
    if show_progress:
        print("Start identification...")

    index = 1
    freqs = np.size(cfg.freqs)

    def collect_result(result: list[Any]) -> None:
        nonlocal index
        results.append(result)
        if show_progress:
            print(f"\rFrequencies simulated: {index}/{freqs}", end="")
        index += 1

    def custom_error_callback(error: Any) -> None:
        print(f"Error during multiprocessing: {str(error)}")

    if cfg.multiprocess:
        # Create the multiprocessing pool using all available CPUs
        with mp.Pool(mp.cpu_count()) as pool:
            async_results = []
            for i, f_e in enumerate(cfg.freqs):
                async_result = pool.apply_async(
                    identify,
                    args=[cfg, sim_op, i, f_e],
                    error_callback=custom_error_callback,
                    callback=collect_result,
                )
                async_results.append(async_result)
            # Wait for all tasks to complete
            [result.get() for result in async_results]
        # Sort the results list along the first element (index i)
        results.sort(key=lambda x: x[0])
    else:
        # Run only in single thread
        for i, f_e in enumerate(cfg.freqs):
            result = identify(cfg, sim_op, i=i, f_e=f_e)
            collect_result(result)

    if show_progress:
        print(f"\nExecution time: {(time() - t_start):.2f} s")
    res = post_process(results, operating_point)
    if cfg.filename is not None:
        if cfg.filetype == "csv":
            save_csv(res, cfg.filename)
        elif cfg.filetype == "mat":
            save_mat(res, cfg.filename)

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
    if latex:
        set_latex_style()
    else:
        set_screen_style()

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
    if latex:
        set_latex_style()
    else:
        set_screen_style()

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
