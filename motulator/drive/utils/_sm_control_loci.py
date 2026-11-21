"""Computation of optimal control loci for synchronous machines."""

from dataclasses import dataclass
from typing import Any

import numpy as np
from scipy.optimize import root_scalar

from motulator.drive.utils._parameters import (
    BaseSynchronousMachineControlPars,
    aux_current,
    aux_flux,
)

NUM = 16


# %%
@dataclass
class Locus:
    """
    Control locus of a synchronous machine.

    Parameters
    ----------
    i_s_dq : ndarray
        Stator current (A) in rotor coordinates.
    psi_s_dq : ndarray
        Stator flux linkage (Vs) in rotor coordinates.
    tau_M : ndarray
        Electromagnetic torque (Nm).

    """

    i_s_dq: np.ndarray
    psi_s_dq: np.ndarray
    tau_M: np.ndarray


# %%
class ControlLoci:
    """
    Compute the control loci of a synchronous machine.

    The class computes the maximum-torque-per-ampere (MTPA), maximum-torque-per-volt
    (MTPV), and constant-current loci [#Mor1994]_, taking the magnetic saturation into
    account. The loci can be used to precompute lookup tables for control and to analyze
    the machine characteristics.

    Parameters
    ----------
    par : ~motulator.drive.control.sm.BaseSynchronousMachineControlPars
        Machine model parameters.

    Notes
    -----
    The MTPA and MTPV conditions are expressed in terms of the auxiliary flux and the
    auxiliary current, respectively [#Var2022]_, allowing a compact representation of
    the conditions. The auxiliary vectors are rotated by 90° compared to [#Var2022]_.

    The d axis is assumed to be aligned with the PM flux, in which case `L_q >= L_d`,
    or, in machines without PMs, with the maximum inductance, i.e., `L_d > L_q`.

    References
    ----------
    .. [#Mor1994] Morimoto, Sanada, Takeda, "Wide-speed operation of interior permanent
       magnet synchronous motors with high-performance current regulator," IEEE Trans.
       Ind. Appl., 1994, https://doi.org/10.1109/28.297908

    .. [#Var2022] Varatharajan, Pellegrino, Armando, "Direct flux vector control of
       synchronous motor drives: Accurate decoupled control with online adaptive maximum
       torque per ampere and maximum torque per volts evaluation," IEEE Trans. Ind.
       Electron., 2022, https://doi.org/10.1109/TIE.2021.3060665

    """

    def __init__(self, par: BaseSynchronousMachineControlPars) -> None:
        self.par = par
        # Range of the current and flux angles (rad) containing the loci
        self._angles = (0, 0.5 * np.pi) if par.psi_f == 0 else (0.5 * np.pi, np.pi)

    def _locus(self, i_s_dq: Any, psi_s_dq: Any) -> Locus:
        """Locus with the torque computed from the current and the flux linkage."""
        tau_M = 1.5 * self.par.n_p * np.imag(i_s_dq * np.conj(psi_s_dq))
        return Locus(i_s_dq, psi_s_dq, tau_M)

    def compute_mtpa_current_angle(self, i_s_abs: float) -> float:
        """MTPA current angle (rad) at given current magnitude (A)."""

        def mtpa_cond(gamma: float) -> float:
            i_s_dq = i_s_abs * np.exp(1j * gamma)
            psi_s_dq = self.par.psi_s_dq(i_s_dq)
            L_s = self.par.incr_ind_mat(i_s_dq)
            psi_a_dq = aux_flux(complex(i_s_dq), complex(psi_s_dq), L_s)
            return np.real(psi_a_dq * np.conj(i_s_dq))

        if mtpa_cond(self._angles[0]) * mtpa_cond(self._angles[1]) > 0:
            raise ValueError(
                "No MTPA solution found. Check the alignment of the d axis, see Notes."
            )
        return root_scalar(mtpa_cond, bracket=self._angles, method="brentq").root

    def compute_mtpa_locus(self, i_s_max: float, num: int = NUM) -> Locus:
        """
        Compute the MTPA locus.

        Parameters
        ----------
        i_s_max : float
            Maximum current magnitude (A), up to which the locus is computed.
        num : int, optional
            Number of points, defaults to 16.

        Returns
        -------
        Locus
            MTPA locus.

        """
        current_magnitudes = np.linspace(0, i_s_max, num)

        # Calculate MTPA points iteratively for each current magnitude
        gamma = np.zeros_like(current_magnitudes)
        for idx, i_s_abs in enumerate(current_magnitudes):
            gamma[idx] = self.compute_mtpa_current_angle(float(i_s_abs))

        i_s_dq = current_magnitudes * np.exp(1j * gamma)
        return self._locus(i_s_dq, self.par.psi_s_dq(i_s_dq))

    def compute_mtpv_flux_angle(self, psi_s_abs: float) -> tuple[float, complex]:
        """
        MTPV flux angle (rad) and current (A) at given flux linkage magnitude (Vs).

        NaN values are returned if no MTPV exists in the range of the flux angle.

        """

        def mtpv_cond(delta: float) -> float:
            psi_s_dq = psi_s_abs * np.exp(1j * delta)
            i_s_dq = self.par.iterate_i_s_dq(psi_s_dq)
            L_s = self.par.incr_ind_mat(i_s_dq)
            i_a_dq = aux_current(complex(psi_s_dq), complex(i_s_dq), L_s)
            return np.real(i_a_dq * np.conj(psi_s_dq))

        if mtpv_cond(self._angles[0]) * mtpv_cond(self._angles[1]) > 0:
            return np.nan, np.nan  # No root in the range
        delta = root_scalar(mtpv_cond, bracket=self._angles, method="brentq").root
        return delta, self.par.iterate_i_s_dq(psi_s_abs * np.exp(1j * delta))

    def compute_mtpv_locus(self, psi_s_max: float, num: int = NUM) -> Locus:
        """
        Compute the MTPV locus.

        Parameters
        ----------
        psi_s_max : float
            Maximum flux linkage magnitude (Vs), up to which the locus is computed.
        num : int, optional
            Number of points, defaults to 16.

        Returns
        -------
        Locus
            MTPV locus.

        """
        # Calculate MTPV points iteratively for each flux magnitude
        flux_magnitudes = np.linspace(0, psi_s_max, num)
        delta = np.zeros_like(flux_magnitudes)
        i_s_dq = np.zeros_like(flux_magnitudes, dtype=complex)
        for idx, psi_s_abs in enumerate(flux_magnitudes):
            delta[idx], i_s_dq[idx] = self.compute_mtpv_flux_angle(float(psi_s_abs))

        return self._locus(i_s_dq, flux_magnitudes * np.exp(1j * delta))

    def compute_const_current_locus(
        self, i_s_max: float, gamma_range: tuple[Any, Any] | None = None, num: int = NUM
    ) -> Locus:
        """
        Compute the constant-current locus.

        Parameters
        ----------
        i_s_max : float
            Current limit (A).
        gamma_range : tuple, optional
            Range of the current angle (electrical rad), defaults to (pi, pi/2) for PM
            machines and (pi/2, 0) for machines without PMs. If the first element is NaN
            (no MTPV), the range starts from pi or pi/2, respectively.
        num : int, optional
            Number of points, defaults to 16.

        Returns
        -------
        Locus
            Constant-current locus.

        """
        if gamma_range is None:
            gamma_range = (self._angles[1], self._angles[0])
        elif np.isnan(gamma_range[0]):  # No MTPV
            gamma_range = (self._angles[1], gamma_range[-1])
        i_s_dq = i_s_max * np.exp(1j * np.linspace(*gamma_range, num))
        return self._locus(i_s_dq, self.par.psi_s_dq(i_s_dq))

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
            L_s = self.par.incr_ind_mat(i_s_dq)
            i_a_dq = aux_current(complex(psi_s_dq), complex(i_s_dq), L_s)
            return float(np.real(i_a_dq * np.conj(psi_s_dq)))

        if mtpv_cond(self._angles[0]) * mtpv_cond(self._angles[1]) >= 0:
            return np.nan  # No MTPV for this current
        gamma = root_scalar(mtpv_cond, bracket=self._angles, method="brentq").root
        return complex(i_s_abs * np.exp(1j * gamma))
