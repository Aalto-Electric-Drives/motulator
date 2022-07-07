# pylint: disable=C0103
"""
This module contains continuous-time models for induction motors.

Peak-valued complex space vectors are used. The space vector models are
implemented in stator coordinates. The default values correspond to a 2.2-kW
induction motor.

"""
from __future__ import annotations
from collections.abc import Callable
from dataclasses import dataclass, field
import numpy as np
from motulator.helpers import complex2abc


# %%
@dataclass
class InductionMotor:
    """
    Induction motor model.

    An induction motor is modeled using the Gamma-equivalent model [1]_. The
    model is implemented in stator coordinates.

    Parameters
    ----------
    p : int
        Number of pole pairs.
    R_s : float
        Stator resistance.
    R_r : float
        Rotor resistance.
    L_ell : float
        Leakage inductance.
    L_s : float
        Stator inductance.

    Notes
    -----
    The Gamma model is chosen here since it can be extended with the magnetic
    saturation model in a staightforward manner. If the magnetic saturation is
    omitted, the Gamma model is mathematically identical to the inverse-Gamma
    and T models [1]_.

    References
    ----------
    .. [1] Slemon, "Modelling of induction machines for electric drives," IEEE
       Trans. Ind. Appl., 1989, https://doi.org/10.1109/28.44251.

    """
    p: int = 2
    # Gamma parameters
    R_s: float = 3.7
    R_r: float = 2.5
    L_ell: float = .023
    L_s: float = .245
    # Initial values
    psi_ss0: complex = field(repr=False, default=0j)
    psi_rs0: complex = field(repr=False, default=0j)

    def currents(self, psi_ss, psi_rs):
        """
        Compute the stator and rotor currents.

        Parameters
        ----------
        psi_ss : complex
            Stator flux linkage.
        psi_rs : complex
            Rotor flux linkage.

        Returns
        -------
        i_ss : complex
            Stator current.
        i_rs : complex
            Rotor current.

        """
        i_rs = (psi_rs - psi_ss)/self.L_ell
        i_ss = psi_ss/self.L_s - i_rs
        return i_ss, i_rs

    def torque(self, psi_ss, i_ss):
        """
        Compute the electromagnetic torque.

        Parameters
        ----------
        psi_ss : complex
            Stator flux linkage.
        i_ss : complex
            Stator current.

        Returns
        -------
        tau_M : float
            Electromagnetic torque.

        """
        tau_M = 1.5*self.p*np.imag(i_ss*np.conj(psi_ss))
        return tau_M

    def f(self, psi_ss, psi_rs, u_ss, w_M):
        # pylint: disable=R0913
        """
        Compute the state derivatives.

        Parameters
        ----------
        psi_ss : complex
            Stator flux linkage.
        psi_rs : complex
            Rotor flux linkage.
        u_ss : complex
            Stator voltage.
        w_M : float
            Rotor angular speed (in mechanical rad/s).

        Returns
        -------
        complex list, length 2
            Time derivative of the state vector, [dpsi_ss, dpsi_rs]

        """
        i_ss, i_rs = self.currents(psi_ss, psi_rs)
        dpsi_ss = u_ss - self.R_s*i_ss
        dpsi_rs = -self.R_r*i_rs + 1j*self.p*w_M*psi_rs
        return [dpsi_ss, dpsi_rs]

    def meas_currents(self):
        """
        Measure the phase currents at the end of the sampling period.

        Returns
        -------
        i_s_abc : 3-tuple of floats
            Phase currents.

        """
        # Stator current space vector in stator coordinates
        i_ss, _ = self.currents(self.psi_ss0, self.psi_rs0)
        # Phase currents
        i_s_abc = complex2abc(i_ss)  # + noise + offset ...
        return i_s_abc


# %%
@dataclass
class SaturableStatorInductance:
    """
    Stator inductance saturation model.

    This saturation model is given by [2]_::

        L_s(psi_s) = L_su/(1 + (beta*abs(psi_s)**S)

    Parameters
    ----------
    L_su : float
        Unsaturated stator inductance.
    beta : float
        Positive coefficient.
    S : float
        Positive coefficient.

    References
    ----------
    .. [2] Qu, Ranta, Hinkkanen, Luomi, "Loss-minimizing flux level control of
       induction motor drives," IEEE Trans. Ind. Appl., 2021,
       https://doi.org/10.1109/TIA.2012.2190818

    """
    L_su: float = .34
    beta: float = .84
    S: float = 7.

    def __call__(self, psi_s):
        """
        Parameters
        ----------
        psi_s : float or complex
            Stator flux linkage.

        Returns
        -------
        L_s : float
            Saturated value of the stator inductance.

        """
        L_s = self.L_su/(1. + (self.beta*np.abs(psi_s))**self.S)
        return L_s


# %%
@dataclass
class InductionMotorSaturated(InductionMotor):
    """
    Induction motor model with main-flux saturation.

    This extends the InductionMotor class with a main-flux magnetic saturation
    model.

    Parameters
    ----------
    L_s : Callable[[float], float]
        Saturable stator inductance, L_s = L_s(psi_s)

    """
    L_s: Callable[[float], float] = SaturableStatorInductance()

    def currents(self, psi_ss, psi_rs):
        """
        This method overrides the base class method.

        """
        L_s = self.L_s(psi_ss)
        i_rs = (psi_rs - psi_ss)/self.L_ell
        i_ss = psi_ss/L_s - i_rs
        return i_ss, i_rs
