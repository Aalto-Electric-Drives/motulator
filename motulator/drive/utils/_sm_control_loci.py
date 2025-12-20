"""Computation of optimal control loci for synchronous machines."""

from dataclasses import dataclass
from typing import Any, Callable

import numpy as np
from scipy.optimize import root_scalar

from motulator.drive.utils._parameters import (
    SaturatedSynchronousMachinePars,
    SynchronousMachinePars,
)

EPS: float = 1e-6


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

        if mtpa_cond(gamma_range[0]) * mtpa_cond(gamma_range[1]) >= 0:
            return 0.0  # No root in the range
        return root_scalar(mtpa_cond, bracket=gamma_range, method="brentq").root

    def compute_mtpa_locus(self, i_s_max: float, num: int = 16) -> MTPALocus:
        """
        Compute the MTPA locus.

        Parameters
        ----------
        i_s_max : float
            Maximum current magnitude (A) at which the locus is computed.
        num : int, optional
            Number of points, defaults to 16.

        Returns
        -------
        MTPALocus
            MTPA locus data.

        """
        current_magnitudes = np.linspace(0, i_s_max, num)

        # Calculate MTPA points iteratively for each current magnitude
        gamma = np.zeros_like(current_magnitudes)
        for idx, i_s_mag in enumerate(current_magnitudes):
            gamma[idx] = self.compute_mtpa_current_angle(float(i_s_mag))

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

    def compute_mtpv_current_angle(self, i_s_abs: float) -> float:
        """MTPV current angle (rad) at given current magnitude (A)."""

        def mtpv_cond(delta: float) -> float:
            i_s_dq = i_s_abs * np.exp(1j * delta)
            psi_s_dq = self.par.psi_s_dq(i_s_dq)
            i_a_dq = self.par.aux_current(i_s_dq)
            return float(np.real(i_a_dq * np.conj(psi_s_dq)))

        if self.par.psi_f == 0:
            delta_range = (0, 0.5 * np.pi)
        else:
            delta_range = (0.5 * np.pi, np.pi)

        if mtpv_cond(delta_range[0]) * mtpv_cond(delta_range[1]) >= 0:
            return np.nan  # No root in the range
        return root_scalar(mtpv_cond, bracket=delta_range, method="brentq").root

    def solve_current_for_mtpv_torque(self, tau_M: float, i_s_abs0: float) -> float:
        """
        Solve for the current yielding the given MTPV torque.

        Parameters
        ----------
        tau_M : float
            Target torque (Nm).
        i_s_abs0 : float
            Initial guess for the current magnitude (A).

        Returns
        -------
        float
            Stator current magnitude (A) that yields the target torque.

        """

        # Solve for the current yielding the target torque
        def torque_error(i_s_abs: float) -> float:
            angle = self.compute_mtpv_current_angle(i_s_abs)
            i_s = i_s_abs * np.exp(1j * angle)
            psi_s = self.par.psi_s_dq(i_s)
            return tau_M - 1.5 * self.par.n_p * (i_s * psi_s.conjugate()).imag

        # Find MTPV starting point
        i_d_mtpv = root_scalar(
            lambda i_d: np.real(self.par.psi_s_dq(i_d)), x0=0, method="newton"
        ).root
        if i_s_abs0 < abs(i_d_mtpv):
            return np.nan

        return root_scalar(torque_error, x0=i_s_abs0, method="newton").root

    def compute_mtpv_locus(self, i_s_max: float, num: int = 16) -> MTPVLocus:
        """
        Compute the MTPV locus.

        Parameters
        ----------
        i_s_max : float
            Maximum current (A) at which the locus is computed.
        num : int, optional
            Number of points, defaults to 16.

        Returns
        -------
        MTPVLocus
            MTPV locus data.

        """
        # Find MTPV starting point
        i_d_mtpv = root_scalar(
            lambda i_d: np.real(self.par.psi_s_dq(i_d)), x0=0, method="newton"
        ).root
        i_d0 = abs(i_d_mtpv) + EPS

        # Check if MTPV exists
        if i_d0 >= i_s_max:
            return MTPVLocus(
                psi_s_dq=np.array([np.nan]),
                i_s_dq=np.array([np.nan]),
                tau_M=np.array([np.nan]),
                tau_M_vs_psi_s_abs=lambda x: np.nan,
                i_s_dq_vs_psi_s_abs=lambda x: np.nan,
            )

        # Calculate MTPV points iteratively for each current magnitude
        current_magnitudes = np.linspace(i_d0, i_s_max, num)
        delta = np.zeros_like(current_magnitudes)
        for idx, i_s_abs in enumerate(current_magnitudes):
            delta[idx] = self.compute_mtpv_current_angle(float(i_s_abs))

        # MTPV locus expressed with different quantities
        i_s_dq = current_magnitudes * np.exp(1j * delta)
        psi_s_dq = self.par.psi_s_dq(i_s_dq)
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
        num: int = 16,
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
            Number of points, defaults to 16.

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
