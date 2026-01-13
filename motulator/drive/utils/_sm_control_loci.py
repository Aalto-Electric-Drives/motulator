"""Computation of optimal control loci for synchronous machines."""

from dataclasses import dataclass
from typing import Any, Callable

import numpy as np
from scipy.optimize import root_scalar

from motulator.drive.utils._parameters import (
    SaturatedSynchronousMachinePars,
    SynchronousMachinePars,
)

NUM = 16


# %%
@dataclass
class MTPALocus:
    """Maximum-torque-per-ampere (MTPA) locus data."""

    i_s_dq: Any
    psi_s_dq: Any
    tau_M: Any
    i_s_dq_vs_tau_M: Callable[[float], complex]


@dataclass
class MTPVLocus:
    """Maximum-torque-per-volt (MTPV) locus data."""

    psi_s_dq: Any
    i_s_dq: Any
    tau_M: Any
    tau_M_vs_psi_s_abs: Callable[[float], float]
    i_s_dq_vs_psi_s_abs: Callable[[float], complex]


@dataclass
class CurrentLimitLocus:
    """Constant current limit locus data."""

    psi_s_dq: Any
    i_s_dq: Any
    tau_M: Any
    i_s_dq_vs_psi_s_abs: Callable[[float], complex]


# %%
class ControlLoci:
    """
    Compute MTPA and MTPV loci based on the machine parameters.

    This class computes optimal control loci for synchronous machines, including the
    maximum-torque-per-ampere (MTPA), maximum-torque-per-volt (MTPV), and current limit
    loci [#Mor1994]_. The magnetic saturation is taken into account. The methods can be
    used to precompute lookup tables for control and to analyze the machine
    characteristics.

    Parameters
    ----------
    par : SynchronousMachinePars | SaturatedSynchronousMachinePars
        Machine model parameters.

    Notes
    -----
    The MTPA and MTPV conditions are expressed in terms of the auxiliary flux and the
    auxiliary current, respectively [#Var2022]_, allowing a compact representation of
    the conditions. Notice that we define these auxiliary vectors 90 degrees rotated as
    compared to [#Var2022]_, but otherwise the concepts are equivalent.

    References
    ----------
    .. [#Mor1994] Morimoto, Sanada, Takeda, "Wide-speed operation of interior permanent
       magnet synchronous motors with high-performance current regulator," IEEE Trans.
       Ind. Appl., https://doi.org/10.1109/28.297908

    .. [#Var2022] Varatharajan, Pellegrino, Armando, "Direct flux vector control of
       synchronous motor drives: Accurate decoupled control with online adaptive maximum
       torque per ampere and maximum torque per volts evaluation," IEEE Trans. Ind.
       Electron., 2022, https://doi.org/10.1109/TIE.2021.3060665

    """

    def __init__(
        self, par: SynchronousMachinePars | SaturatedSynchronousMachinePars
    ) -> None:
        self.par = par

    def compute_mtpa_current_angle(self, i_s_abs: float) -> float:
        """MTPA current angle (rad) at given current magnitude (A)."""

        def mtpa_cond(gamma: float) -> float:
            i_s_dq = i_s_abs * np.exp(1j * gamma)
            psi_a_dq = self.par.aux_flux(i_s_dq)
            return np.real(psi_a_dq * np.conj(i_s_dq))

        if self.par.psi_f == 0:
            gamma_range = (0, 0.5 * np.pi)
        else:
            gamma_range = (0.5 * np.pi, np.pi)

        if mtpa_cond(gamma_range[0]) * mtpa_cond(gamma_range[1]) > 0:
            return 0.0  # No root in the range
        return root_scalar(mtpa_cond, bracket=gamma_range, method="brentq").root

    def compute_mtpa_locus(self, i_s_max: float, num: int = NUM) -> MTPALocus:
        """
        Compute the MTPA locus.

        Parameters
        ----------
        i_s_max : float
            Maximum current magnitude (A) at which the locus is computed.
        num : int, optional
            Number of points.

        Returns
        -------
        MTPALocus
            MTPA locus data.

        """
        current_magnitudes = np.linspace(0, i_s_max, num)

        # Calculate MTPA points iteratively for each current magnitude
        gamma = np.zeros_like(current_magnitudes)
        for idx, i_s_abs in enumerate(current_magnitudes):
            gamma[idx] = self.compute_mtpa_current_angle(float(i_s_abs))

        # MTPA locus expressed with different quantities
        i_s_dq = current_magnitudes * np.exp(1j * gamma)
        psi_s_dq = self.par.psi_s_dq(i_s_dq)
        tau_M = 1.5 * self.par.n_p * np.imag(i_s_dq * np.conj(psi_s_dq))

        return MTPALocus(
            i_s_dq=i_s_dq,
            psi_s_dq=psi_s_dq,
            tau_M=tau_M,
            i_s_dq_vs_tau_M=lambda x: np.interp(x, tau_M, i_s_dq),
        )

    def compute_mtpv_flux_angle(self, psi_s_abs: float) -> float:
        """MTPV flux angle (rad) at given flux magnitude (Vs)."""

        def mtpv_cond(delta: float) -> float:
            psi_s_dq = psi_s_abs * np.exp(1j * delta)
            i_s_dq = self.par.iterate_i_s_dq(psi_s_dq)
            i_a_dq = self.par.aux_current(i_s_dq)
            return np.real(i_a_dq * np.conj(psi_s_dq))

        if self.par.psi_f == 0:
            delta_range = (0, 0.5 * np.pi)
        else:
            delta_range = (0.5 * np.pi, np.pi)

        if mtpv_cond(delta_range[0]) * mtpv_cond(delta_range[1]) > 0:
            return np.nan  # No root in the range
        return root_scalar(mtpv_cond, bracket=delta_range, method="brentq").root

    def compute_mtpv_locus(self, psi_s_max: float, num: int = NUM) -> MTPVLocus:
        """
        Compute the MTPV locus.

        Parameters
        ----------
        psi_s_max : float
            Maximum flux linkage (Vs) at which the locus is computed.
        num : int, optional
            Number of points.

        Returns
        -------
        MTPVLocus
            MTPV locus data.

        """
        # Calculate MTPV points iteratively for each flux magnitude
        flux_magnitudes = np.linspace(0, psi_s_max, num)
        delta = np.zeros_like(flux_magnitudes)
        for idx, psi_s_abs in enumerate(flux_magnitudes):
            delta[idx] = self.compute_mtpv_flux_angle(float(psi_s_abs))

        # MTPV locus expressed with different quantities
        psi_s_dq = flux_magnitudes * np.exp(1j * delta)
        i_s_dq = np.array([self.par.iterate_i_s_dq(psi) for psi in psi_s_dq])
        tau_M = 1.5 * self.par.n_p * np.imag(i_s_dq * np.conj(psi_s_dq))

        return MTPVLocus(
            psi_s_dq=psi_s_dq,
            i_s_dq=i_s_dq,
            tau_M=tau_M,
            tau_M_vs_psi_s_abs=lambda x: np.interp(x, abs(psi_s_dq), tau_M),
            i_s_dq_vs_psi_s_abs=lambda x: np.interp(x, abs(psi_s_dq), i_s_dq),
        )

    def compute_const_current_locus(
        self,
        i_s_max: float,
        gamma_range: tuple[Any, Any] = (np.pi, 0.5 * np.pi),
        num: int = NUM,
    ) -> CurrentLimitLocus:
        """
        Compute the constant current locus.

        Parameters
        ----------
        i_s_max : float
            Current limit (A).
        gamma_range : tuple, optional
            Range of the current angle (electrical rad), defaults to (pi, pi/2).
        num : int, optional
            Number of points.

        Returns
        -------
        CurrentLimitLocus
            Constant current locus data.

        """
        if np.isnan(gamma_range[0]):  # No MTPV
            gamma_range = (np.pi, gamma_range[-1])

        gamma = np.linspace(*gamma_range, num)

        # Current limit expressed with different quantities
        i_s_dq = i_s_max * np.exp(1j * gamma)
        psi_s_dq = self.par.psi_s_dq(i_s_dq)
        tau_M = 1.5 * self.par.n_p * np.imag(i_s_dq * np.conj(psi_s_dq))

        return CurrentLimitLocus(
            psi_s_dq=psi_s_dq,
            i_s_dq=i_s_dq,
            tau_M=tau_M,
            i_s_dq_vs_psi_s_abs=lambda x: np.interp(x, abs(psi_s_dq), i_s_dq),
        )

    def compute_mtpv_current(self, i_s_abs: float) -> complex:
        """
        MTPV current at given current magnitude.

        Parameters
        ----------
        i_s_abs : float
            Current magnitude (A).

        Returns
        -------
        complex
            MTPV current (A). If no MTPV exists, returns np.nan.

        """

        def mtpv_cond(gamma: float) -> float:
            i_s_dq = i_s_abs * np.exp(1j * gamma)
            psi_s_dq = self.par.psi_s_dq(i_s_dq)
            i_a_dq = self.par.aux_current(i_s_dq)
            return float(np.real(i_a_dq * np.conj(psi_s_dq)))

        if self.par.psi_f == 0:
            gamma_range = (0, 0.5 * np.pi)
        else:
            gamma_range = (0.5 * np.pi, np.pi)

        if mtpv_cond(gamma_range[0]) * mtpv_cond(gamma_range[1]) >= 0:
            return np.nan  # No MTPV for this current

        gamma = root_scalar(mtpv_cond, bracket=gamma_range, method="brentq").root

        return complex(i_s_abs * np.exp(1j * gamma))
