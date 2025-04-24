"""Visualize optimal control loci of synchronous machines."""

from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from cycler import cycler

from motulator.common.utils._utils import BaseValues, set_latex_style, set_screen_style
from motulator.drive.utils._parameters import (
    SaturatedSynchronousMachinePars,
    SynchronousMachinePars,
)
from motulator.drive.utils._sm_control_loci import ControlLoci


def _setup_plot(
    i_s_vals: Any, latex: bool, base: BaseValues | None = None
) -> tuple[np.ndarray, bool, BaseValues]:
    """Setup common plot parameters."""
    if latex:
        set_latex_style()
    else:
        set_screen_style()
    pu_vals = base is not None
    # Use SI units if no base values are given
    if base is None:
        base = BaseValues.unity()
    # Convert to SI units
    i_s_vals = np.array(i_s_vals, ndmin=1) * base.i
    return i_s_vals, pu_vals, base


# %%
class MachineCharacteristics:
    """
    Analyze and visualize control loci for synchronous machines.

    This class provides a unified interface for plotting different characteristics of
    synchronous machines directly from machine parameters.

    Parameters
    ----------
    par : SynchronousMachinePars
        Machine parameters.

    """

    def __init__(
        self, par: SynchronousMachinePars | SaturatedSynchronousMachinePars
    ) -> None:
        self.par = par
        self._loci = ControlLoci(par)

    # %%
    def plot_flux_loci(
        self,
        i_s_vals: Any,
        base: BaseValues | None = None,
        num: int = 16,
        latex: bool = False,
    ) -> None:
        """
        Plot the flux linkage loci.

        Parameters
        ----------
        i_s_vals : float or list
            Current magnitudes (A) at which the loci are evaluated. If `base` is given,
            the values are interpreted as per-unit values.
        base : BaseValues, optional
            Base values for scaling the loci.
        num : int, optional
            Amount of points to be evaluated, defaults to 16.
        latex : bool, optional
            Use LaTeX fonts for the labels, requires a working LaTeX installation.

        """
        i_s_vals, pu_vals, base = _setup_plot(i_s_vals, latex, base)

        fig, ax = plt.subplots()

        # MTPA locus
        mtpa = self._loci.compute_mtpa_locus(i_s_vals[-1], num)
        ax.plot(
            mtpa.psi_s_dq.real / base.psi,
            mtpa.psi_s_dq.imag / base.psi,
            linewidth=1.5,
            label="MTPA",
        )

        # MTPV locus
        mtpv_i_s_dq_max = self._loci.compute_mtpv_current(i_s_vals[-1])
        if not np.isnan(mtpv_i_s_dq_max):
            # MTPV exists within the current range
            mtpv_psi_s_max = float(abs(self.par.psi_s_dq(mtpv_i_s_dq_max)))
            mtpv = self._loci.compute_mtpv_locus(mtpv_psi_s_max, num)
            ax.plot(
                mtpv.psi_s_dq.real / base.psi,
                mtpv.psi_s_dq.imag / base.psi,
                linewidth=1.5,
                label="MTPV",
            )

        # Constant current loci
        ax.set_prop_cycle(
            cycler(color=["b", "r", "b", "r"])
            + cycler(linestyle=["--", "--", "-.", "-."])
        )

        for i_s_abs in i_s_vals:
            gamma_min = np.angle(self._loci.compute_mtpv_current(i_s_abs))
            gamma_max = self._loci.compute_mtpa_current_angle(i_s_abs)
            gamma_range = (gamma_min, gamma_max)
            current_lim = self._loci.compute_const_current_locus(
                i_s_abs, gamma_range, num
            )
            label_unit = "p.u." if pu_vals else "A"
            i_pu = i_s_abs / base.i
            value = int(i_pu) if i_pu.is_integer() else f"{i_pu:.1f}"
            label = rf"$i_\mathrm{{s}}=$ {value} {label_unit}"
            ax.plot(
                current_lim.psi_s_dq.real / base.psi,
                current_lim.psi_s_dq.imag / base.psi,
                label=label,
            )

        ax.legend()

        # Find centers and ranges
        x_data = ax.get_lines()[0].get_xdata()
        y_data = ax.get_lines()[0].get_ydata()
        x_center = 0.5 * (np.max(x_data) + np.min(x_data))
        x_range = np.max(x_data) - np.min(x_data)
        y_max = np.max(y_data)
        # Use larger dimension for square plot
        plot_size = max(x_range, y_max)
        # Set limits
        ax.set_ylim(0, plot_size)
        ax.set_xlim(x_center - 0.5 * plot_size, x_center + 0.5 * plot_size)
        ax.set_aspect("equal")

        if pu_vals:
            ax.set_xlabel(r"$\psi_\mathrm{d}$ (p.u.)")
            ax.set_ylabel(r"$\psi_\mathrm{q}$ (p.u.)")
        else:
            ax.set_xlabel(r"$\psi_\mathrm{d}$ (Vs)")
            ax.set_ylabel(r"$\psi_\mathrm{q}$ (Vs)")

        plt.show()

    # %%
    def plot_current_loci(
        self,
        i_s_vals: Any,
        base: BaseValues | None = None,
        num: int = 16,
        latex: bool = False,
    ) -> None:
        """
        Plot the current loci.

        Parameters
        ----------
        i_s_vals : float or list
            Current magnitudes (A) at which the loci are evaluated. If `base` is given,
            the values are interpreted as per-unit values.
        base : BaseValues, optional
            Base values for scaling the loci.
        num : int, optional
            Amount of points to be evaluated, defaults to 16.
        latex : bool, optional
            Use LaTeX fonts for the labels, requires a working LaTeX installation.

        """
        i_s_vals, pu_vals, base = _setup_plot(i_s_vals, latex, base)

        fig, ax = plt.subplots()

        # MTPA locus
        mtpa = self._loci.compute_mtpa_locus(i_s_vals[-1], num)
        ax.plot(
            mtpa.i_s_dq.real / base.i,
            mtpa.i_s_dq.imag / base.i,
            linewidth=1.5,
            label="MTPA",
        )

        # MTPV locus
        mtpv_i_s_dq_max = self._loci.compute_mtpv_current(i_s_vals[-1])
        if not np.isnan(mtpv_i_s_dq_max):  # MTPV exists within the current range
            mtpv_psi_s_max = float(abs(self.par.psi_s_dq(mtpv_i_s_dq_max)))
            mtpv = self._loci.compute_mtpv_locus(mtpv_psi_s_max, num)
            ax.plot(
                mtpv.i_s_dq.real / base.i,
                mtpv.i_s_dq.imag / base.i,
                linewidth=1.5,
                label="MTPV",
            )

        # Constant current loci
        ax.set_prop_cycle(
            cycler(color=["b", "r", "b", "r"])
            + cycler(linestyle=["--", "--", "-.", "-."])
        )

        for i_s_abs in i_s_vals:
            gamma_min = np.angle(self._loci.compute_mtpv_current(i_s_abs))
            gamma_max = self._loci.compute_mtpa_current_angle(i_s_abs)
            gamma_range = (gamma_min, gamma_max)
            const_current = self._loci.compute_const_current_locus(
                i_s_abs, gamma_range, num
            )
            label_unit = "p.u." if pu_vals else "A"
            i_pu = i_s_abs / base.i
            value = int(i_pu) if i_pu.is_integer() else f"{i_pu:.1f}"
            label = rf"$i_\mathrm{{s}}=$ {value} {label_unit}"
            ax.plot(
                const_current.i_s_dq.real / base.i,
                const_current.i_s_dq.imag / base.i,
                label=label,
            )

        # Labels and limits
        ax.legend()
        if pu_vals:
            ax.set_xlabel(r"$i_\mathrm{d}$ (p.u.)")
            ax.set_ylabel(r"$i_\mathrm{q}$ (p.u.)")
        else:
            ax.set_xlabel(r"$i_\mathrm{d}$ (A)")
            ax.set_ylabel(r"$i_\mathrm{q}$ (A)")

        match self.par.kind:
            case "rel":
                ax.axis((0, i_s_vals[-1] / base.i, 0, i_s_vals[-1] / base.i))
            case "pm":
                ax.axis((-i_s_vals[-1] / base.i, 0, 0, i_s_vals[-1] / base.i))

        ax.set_aspect("equal")

        plt.show()

    # %%
    def plot_flux_vs_torque(
        self,
        i_s_vals: Any,
        base: BaseValues | None = None,
        num: int = 16,
        latex: bool = False,
    ) -> None:
        """
        Plot flux magnitude vs. torque characteristics.

        Parameters
        ----------
        i_s_vals : float or list
            Current magnitudes (A) at which the loci are evaluated. If `base` is given,
            the values are interpreted as per-unit values.
        base : BaseValues, optional
            Base values for scaling the loci.
        num : int, optional
            Amount of points to be evaluated, defaults to 16.
        latex : bool, optional
            Use LaTeX fonts for the labels, requires a working LaTeX installation.

        """
        i_s_vals, pu_vals, base = _setup_plot(i_s_vals, latex, base)

        fig, ax = plt.subplots(1, 1)

        # MTPA locus
        mtpa = self._loci.compute_mtpa_locus(i_s_vals[-1], num)

        ax.plot(
            mtpa.tau_M / base.tau,
            np.abs(mtpa.psi_s_dq) / base.psi,
            linewidth=1.5,
            label="MTPA",
        )

        # MTPV locus
        mtpv_i_s_dq_max = self._loci.compute_mtpv_current(i_s_vals[-1])
        if not np.isnan(mtpv_i_s_dq_max):  # MTPV exists within the current range
            mtpv_psi_s_max = float(abs(self.par.psi_s_dq(mtpv_i_s_dq_max)))
            mtpv = self._loci.compute_mtpv_locus(mtpv_psi_s_max, num)
            ax.plot(
                mtpv.tau_M / base.tau,
                np.abs(mtpv.psi_s_dq) / base.psi,
                linewidth=1.5,
                label="MTPV",
            )

        # Constant current loci
        ax.set_prop_cycle(
            cycler(color=["b", "r", "b", "r"])
            + cycler(linestyle=["--", "--", "-.", "-."])
        )

        for i_s_abs in i_s_vals:
            gamma_min = np.angle(self._loci.compute_mtpv_current(i_s_abs))
            gamma_max = self._loci.compute_mtpa_current_angle(i_s_abs)
            gamma_range = (gamma_min, gamma_max)
            const_current = self._loci.compute_const_current_locus(
                i_s_abs, gamma_range, num
            )
            label_unit = "p.u." if pu_vals else "A"
            i_pu = i_s_abs / base.i
            value = int(i_pu) if i_pu.is_integer() else f"{i_pu:.1f}"
            label = rf"$i_\mathrm{{s}}=$ {value} {label_unit}"
            ax.plot(
                const_current.tau_M / base.tau,
                np.abs(const_current.psi_s_dq) / base.psi,
                label=label,
            )

        ax.legend()
        if pu_vals:
            ax.set_xlabel(r"$\tau_\mathrm{M}$ (p.u.)")
            ax.set_ylabel(r"$\psi_\mathrm{s}$ (p.u.)")
        else:
            ax.set_xlabel(r"$\tau_\mathrm{M}$ (Nm)")
            ax.set_ylabel(r"$\psi_\mathrm{s}$ (Vs)")

        ax.set_xlim(0, mtpa.tau_M[-1] / base.tau)
        ax.set_ylim(0, abs(mtpa.psi_s_dq[-1]) / base.psi)

        plt.show()

    # %%
    def plot_current_vs_torque(
        self,
        i_s_vals: Any,
        base: BaseValues | None = None,
        num: int = 16,
        latex: bool = False,
    ) -> None:
        """
        Plot current vs. torque characteristics.

        Parameters
        ----------
        i_s_vals : float or list
            Current magnitudes (A) at which the loci are evaluated. If `base` is given,
            the values are interpreted as per-unit values.
        base : BaseValues, optional
            Base values for scaling the loci.
        num : int, optional
            Amount of points to be evaluated, defaults to 16.
        latex : bool, optional
            Use LaTeX fonts for the labels, requires a working LaTeX installation.

        """
        i_s_vals, pu_vals, base = _setup_plot(i_s_vals, latex, base)
        if latex:
            width = plt.rcParams["figure.figsize"][0]
            height = plt.rcParams["figure.figsize"][1] * 1.6
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(width, height))
        else:
            fig, (ax1, ax2) = plt.subplots(2, 1)

        # MTPA locus
        mtpa = self._loci.compute_mtpa_locus(i_s_vals[-1], num)

        ax1.plot(
            mtpa.tau_M / base.tau,
            mtpa.i_s_dq.real / base.i,
            "b-",
            linewidth=1.5,
            label="MTPA",
        )

        ax2.plot(
            mtpa.tau_M / base.tau,
            mtpa.i_s_dq.imag / base.i,
            "b-",
            linewidth=1.5,
            label="MTPA",
        )

        # MTPV locus
        mtpv_i_s_dq_max = self._loci.compute_mtpv_current(i_s_vals[-1])
        if not np.isnan(mtpv_i_s_dq_max):  # MTPV exists within the current range
            max_mtpv_psi_s = float(abs(self.par.psi_s_dq(mtpv_i_s_dq_max)))
            mtpv = self._loci.compute_mtpv_locus(max_mtpv_psi_s, num)

            ax1.plot(
                mtpv.tau_M / base.tau,
                mtpv.i_s_dq.real / base.i,
                "r-",
                linewidth=1.5,
                label="MTPV",
            )

            ax2.plot(
                mtpv.tau_M / base.tau,
                mtpv.i_s_dq.imag / base.i,
                "r-",
                linewidth=1.5,
                label="MTPV",
            )

        # Constant current loci
        cycle = cycler(color=["b", "r", "b", "r"]) + cycler(
            linestyle=["--", "--", "-.", "-."]
        )
        ax1.set_prop_cycle(cycle)
        ax2.set_prop_cycle(cycle)

        for i_s_abs in i_s_vals:
            gamma_min = np.angle(self._loci.compute_mtpv_current(i_s_abs))
            gamma_max = self._loci.compute_mtpa_current_angle(i_s_abs)
            gamma_range = (gamma_min, gamma_max)
            const_current = self._loci.compute_const_current_locus(
                i_s_abs, gamma_range, num
            )
            label_unit = "p.u." if pu_vals else "A"
            i_pu = i_s_abs / base.i
            value = int(i_pu) if i_pu.is_integer() else f"{i_pu:.1f}"
            label = rf"$i_\mathrm{{s}}=$ {value} {label_unit}"
            ax1.plot(
                const_current.tau_M / base.tau,
                const_current.i_s_dq.real / base.i,
                label=label,
            )

            ax2.plot(
                const_current.tau_M / base.tau,
                const_current.i_s_dq.imag / base.i,
                label=label,
            )

        ax1.legend()
        ax2.legend()

        # Limits
        ax1.set_xlim(0, mtpa.tau_M[-1] / base.tau)
        ax2.set_xlim(0, mtpa.tau_M[-1] / base.tau)

        match self.par.kind:
            case "rel":
                ax1.set_ylim(0, mtpa.i_s_dq.real[-1] / base.i)
                ax2.set_ylim(0, i_s_vals[-1] / base.i)
            case "pm":
                ax1.set_ylim(-i_s_vals[-1] / base.i, 0)
                ax2.set_ylim(0, mtpa.i_s_dq.imag[-1] / base.i)

        if pu_vals:
            ax1.set_ylabel(r"$i_\mathrm{d}$ (p.u.)")
            ax2.set_ylabel(r"$i_\mathrm{q}$ (p.u.)")
            ax2.set_xlabel(r"$\tau_\mathrm{M}$ (p.u.)")
        else:
            ax1.set_ylabel(r"$i_\mathrm{d}$ (A)")
            ax2.set_ylabel(r"$i_\mathrm{q}$ (A)")
            ax2.set_xlabel(r"$\tau_\mathrm{M}$ (Nm)")

    plt.show()
