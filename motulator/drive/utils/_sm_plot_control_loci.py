"""Visualize optimal control loci of synchronous machines."""

from collections.abc import Callable, Sequence
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from cycler import cycler
from matplotlib.axes import Axes

from motulator.common.utils._plotting import save_and_show, set_style
from motulator.common.utils._utils import BaseValues
from motulator.drive.utils._parameters import BaseSynchronousMachineControlPars
from motulator.drive.utils._sm_control_loci import ControlLoci, Locus


# %%
class MachineCharacteristics:
    """
    Visualize the optimal control loci of synchronous machines.

    The MTPA locus and the MTPV locus (if it exists) are computed up to the largest
    given current magnitude, and the constant-current loci between them at each given
    current magnitude. The methods plot these loci in different planes.

    Parameters
    ----------
    par : ~motulator.drive.control.sm.BaseSynchronousMachineControlPars
        Machine parameters.
    i_s_vals : float | Sequence[float]
        Current magnitudes (A) of the constant-current loci, in ascending order. If
        `base` is given, the values are per-unit values.
    base : BaseValues, optional
        Base values. If given, the loci are plotted in per-unit values, otherwise in SI
        units.
    num : int, optional
        Number of points of each locus, defaults to 16.

    """

    def __init__(
        self,
        par: BaseSynchronousMachineControlPars,
        i_s_vals: float | Sequence[float],
        base: BaseValues | None = None,
        num: int = 16,
    ) -> None:
        self.par = par
        self.pu = base is not None
        self.base = base if base is not None else BaseValues.unity()
        loci = ControlLoci(par)
        values = np.array(i_s_vals, ndmin=1)
        self.i_s_max = float(values[-1])  # In the units of the plots
        i_s_max = values[-1] * self.base.i
        self.mtpa = loci.compute_mtpa_locus(i_s_max, num)
        i_s_mtpv = loci.compute_mtpv_current(i_s_max)
        self.mtpv = None
        if not np.isnan(i_s_mtpv):  # MTPV within the current range
            psi_s_max = float(abs(par.psi_s_dq(i_s_mtpv)))
            self.mtpv = loci.compute_mtpv_locus(psi_s_max, num)
        unit = "p.u." if self.pu else "A"
        self.current_loci: list[tuple[str, Locus]] = []
        for value in values:
            i_s_abs = value * self.base.i
            gamma_min = np.angle(loci.compute_mtpv_current(i_s_abs))
            gamma_max = loci.compute_mtpa_current_angle(i_s_abs)
            locus = loci.compute_const_current_locus(
                i_s_abs, (gamma_min, gamma_max), num
            )
            label_value = int(value) if float(value).is_integer() else f"{value:.1f}"
            label = rf"$i_\mathrm{{s}}=$ {label_value} {unit}"
            self.current_loci.append((label, locus))

    def _plot_loci(
        self,
        ax: Axes,
        x: Callable[[Locus], np.ndarray],
        y: Callable[[Locus], np.ndarray],
    ) -> None:
        """Plot the loci in the plane given by the functions `x` and `y` of a locus."""
        for label, locus in (("MTPA", self.mtpa), ("MTPV", self.mtpv)):
            if locus is not None:
                ax.plot(x(locus), y(locus), linewidth=1.5, label=label)
        ax.set_prop_cycle(
            cycler(color=["b", "r", "b", "r"])
            + cycler(linestyle=["--", "--", "-.", "-."])
        )
        for label, locus in self.current_loci:
            ax.plot(x(locus), y(locus), label=label)
        ax.legend()

    def _label(self, symbol: str, si_unit: str) -> str:
        """Axis label of the quantity in per-unit or SI units."""
        return f"{symbol} ({'p.u.' if self.pu else si_unit})"

    def _torque_label(self) -> str:
        """Axis label of the torque."""
        return r"$\tau_\mathrm{m}$ (p.u.)" if self.pu else r"$\tau_\mathrm{M}$ (Nm)"

    def plot_flux_loci(
        self, latex: bool = False, save_path: str | Path | None = None, **savefig_kwargs
    ) -> None:
        """
        Plot the loci in the flux linkage plane.

        Parameters
        ----------
        latex : bool, optional
            Use LaTeX fonts for the labels. Enabling this option requires a working
            LaTeX installation, defaults to False.
        save_path : str | Path, optional
            Path to save the figure. If None, the figure is not saved.
        **savefig_kwargs
            Additional keyword arguments passed to plt.savefig().

        """
        set_style(latex)
        _, ax = plt.subplots()
        psi = self.base.psi
        self._plot_loci(
            ax,
            lambda locus: np.real(locus.psi_s_dq) / psi,
            lambda locus: np.imag(locus.psi_s_dq) / psi,
        )
        # Square plot containing the MTPA locus
        x, y = np.real(self.mtpa.psi_s_dq) / psi, np.imag(self.mtpa.psi_s_dq) / psi
        x_center = 0.5 * (np.max(x) + np.min(x))
        size = max(np.max(x) - np.min(x), np.max(y))
        ax.set_ylim(0, size)
        ax.set_xlim(x_center - 0.5 * size, x_center + 0.5 * size)
        ax.set_aspect("equal")
        ax.set_xlabel(self._label(r"$\psi_\mathrm{d}$", "Vs"))
        ax.set_ylabel(self._label(r"$\psi_\mathrm{q}$", "Vs"))
        save_and_show(save_path, **savefig_kwargs)

    def plot_current_loci(
        self, latex: bool = False, save_path: str | Path | None = None, **savefig_kwargs
    ) -> None:
        """
        Plot the loci in the current plane.

        Parameters
        ----------
        latex : bool, optional
            Use LaTeX fonts for the labels. Enabling this option requires a working
            LaTeX installation, defaults to False.
        save_path : str | Path, optional
            Path to save the figure. If None, the figure is not saved.
        **savefig_kwargs
            Additional keyword arguments passed to plt.savefig().

        """
        set_style(latex)
        _, ax = plt.subplots()
        i = self.base.i
        self._plot_loci(
            ax,
            lambda locus: np.real(locus.i_s_dq) / i,
            lambda locus: np.imag(locus.i_s_dq) / i,
        )
        ax.set_xlabel(self._label(r"$i_\mathrm{d}$", "A"))
        ax.set_ylabel(self._label(r"$i_\mathrm{q}$", "A"))
        i_max = self.i_s_max
        ax.set_xlim((0, i_max) if self.par.psi_f == 0 else (-i_max, 0))
        ax.set_ylim(0, i_max)
        ax.set_aspect("equal")
        save_and_show(save_path, **savefig_kwargs)

    def plot_flux_vs_torque(
        self, latex: bool = False, save_path: str | Path | None = None, **savefig_kwargs
    ) -> None:
        """
        Plot the flux linkage magnitude as a function of the torque.

        Parameters
        ----------
        latex : bool, optional
            Use LaTeX fonts for the labels. Enabling this option requires a working
            LaTeX installation, defaults to False.
        save_path : str | Path, optional
            Path to save the figure. If None, the figure is not saved.
        **savefig_kwargs
            Additional keyword arguments passed to plt.savefig().

        """
        set_style(latex)
        _, ax = plt.subplots()
        tau, psi = self.base.tau, self.base.psi
        self._plot_loci(
            ax,
            lambda locus: locus.tau_M / tau,
            lambda locus: np.abs(locus.psi_s_dq) / psi,
        )
        ax.set_xlabel(self._torque_label())
        ax.set_ylabel(self._label(r"$\psi_\mathrm{s}$", "Vs"))
        ax.set_xlim(0, self.mtpa.tau_M[-1] / tau)
        ax.set_ylim(0, abs(self.mtpa.psi_s_dq[-1]) / psi)
        save_and_show(save_path, **savefig_kwargs)

    def plot_current_vs_torque(
        self, latex: bool = False, save_path: str | Path | None = None, **savefig_kwargs
    ) -> None:
        """
        Plot the current components as functions of the torque.

        Parameters
        ----------
        latex : bool, optional
            Use LaTeX fonts for the labels. Enabling this option requires a working
            LaTeX installation, defaults to False.
        save_path : str | Path, optional
            Path to save the figure. If None, the figure is not saved.
        **savefig_kwargs
            Additional keyword arguments passed to plt.savefig().

        """
        set_style(latex)
        width, height = plt.rcParams["figure.figsize"]
        figsize = (width, 1.6 * height) if latex else None
        _, (ax1, ax2) = plt.subplots(2, 1, figsize=figsize, sharex=True)
        tau, i = self.base.tau, self.base.i
        self._plot_loci(
            ax1,
            lambda locus: locus.tau_M / tau,
            lambda locus: np.real(locus.i_s_dq) / i,
        )
        self._plot_loci(
            ax2,
            lambda locus: locus.tau_M / tau,
            lambda locus: np.imag(locus.i_s_dq) / i,
        )
        ax1.set_xlim(0, self.mtpa.tau_M[-1] / tau)
        i_s_mtpa = self.mtpa.i_s_dq[-1] / i
        if self.par.psi_f == 0:
            ax1.set_ylim(0, i_s_mtpa.real)
            ax2.set_ylim(0, self.i_s_max)
        else:
            ax1.set_ylim(-self.i_s_max, 0)
            ax2.set_ylim(0, i_s_mtpa.imag)
        ax1.set_ylabel(self._label(r"$i_\mathrm{d}$", "A"))
        ax2.set_ylabel(self._label(r"$i_\mathrm{q}$", "A"))
        ax2.set_xlabel(self._torque_label())
        save_and_show(save_path, **savefig_kwargs)
