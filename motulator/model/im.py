# pylint: disable=C0103
"""
Continuous-time models for induction motors.

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
    Γ-equivalent model of an induction motor.

    An induction motor is modeled using the Γ-equivalent model [1]_. The model
    is implemented in stator coordinates.

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
    The Γ model is chosen here since it can be extended with the magnetic
    saturation model in a staightforward manner. If the magnetic saturation is
    omitted, the Γ model is mathematically identical to the inverse-Γ and T
    models [1]_.

    References
    ----------
    .. [1] Slemon, "Modelling of induction machines for electric drives," IEEE
       Trans. Ind. Appl., 1989, https://doi.org/10.1109/28.44251.

    """
    p: int = 2
    # Γ parameters
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
    Γ-equivalent model of an induction motor model with main-flux saturation.

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


# %%
@dataclass
class InductionMotorInvGamma(InductionMotor):
    """
    Inverse-Γ model of an induction motor.

    This extends the InductionMotor class (based on the Γ model) by providing
    the interface for the inverse-Γ model parameters. Linear magnetics are
    assumed. If magnetic saturation is to be modeled, the Γ model is preferred.

    Parameters
    ----------
    p : int
        Number of pole pairs.
    R_s : float
        Stator resistance.
    R_R : float
        Rotor resistance.
    L_sgm : float
        Leakage inductance.
    L_M : float
        Magnetizing inductance.

    """
    # Inverse-Γ parameters
    R_R: float = 2.1
    L_sgm: float = .021
    L_M: float = .224
    # Γ parameters to be computed in the post init
    R_r: float = field(repr=False, default=None)
    L_ell: float = field(repr=False, default=None)
    L_s: float = field(repr=False, default=None)
    # Initial value
    # psi_Rs0: complex = field(repr=False, default=0j)

    def __post_init__(self):
        # Convert the inverse-Γ parameters to the Γ parameters
        gamma = self.L_M/(self.L_M + self.L_sgm)  # Magnetic coupling factor
        self.L_s = self.L_M + self.L_sgm
        self.L_ell = self.L_sgm/gamma
        self.R_r = self.R_R/gamma**2
        # self.psi_rs0 = self.psi_Rs0/gamma
