# pylint: disable=invalid-name
"""
Continuous-time models for induction motors.

Peak-valued complex space vectors are used. The space vector models are
implemented in stator coordinates. The default values correspond to a 2.2-kW
induction motor.

"""
from __future__ import annotations
import numpy as np

from motulator.helpers import complex2abc


# %%
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

    def __init__(self, p=2, R_s=3.7, R_r=2.5, L_ell=.023, L_s=.245):

        # pylint: disable=too-many-arguments
        self.p = p
        self.R_s, self.R_r = R_s, R_r
        self.L_ell, self.L_s = L_ell, L_s
        # Initial values
        self.psi_ss0, self.psi_rs0 = 0j, 0j

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
class InductionMotorSaturated(InductionMotor):
    """
    Γ-equivalent model of an induction motor model with main-flux saturation.

    This extends the InductionMotor class with a main-flux magnetic saturation
    model [2]_::

        L_s(psi_ss) = L_su/(1 + (beta*abs(psi_ss)**S)

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
    L_su : float
        Unsaturated stator inductance.
    beta : float
        Positive coefficient.
    S : float
        Positive coefficient.

    References
    ----------
    .. [2] Qu, Ranta, Hinkkanen, Luomi, "Loss-minimizing flux level control of
       induction motor drives," IEEE Trans. Ind. Appl., 2012,
       https://doi.org/10.1109/TIA.2012.2190818

    """

    def __init__(self,
                 p=2, R_s=3.7, R_r=2.5, L_ell=.023, L_su=.34, beta=.84, S=7):

        # pylint: disable=too-many-arguments
        super().__init__(p=p, R_s=R_s, R_r=R_r, L_ell=L_ell)
        # Saturation model
        self.L_s = lambda psi: L_su/(1. + (beta*np.abs(psi))**S)

    def currents(self, psi_ss, psi_rs):
        """
        This method overrides the base class method.

        """
        # Saturated value of the stator inductance.
        L_s = self.L_s(psi_ss)
        # Currents
        i_rs = (psi_rs - psi_ss)/self.L_ell
        i_ss = psi_ss/L_s - i_rs
        return i_ss, i_rs


# %%
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

    def __init__(self, p=2, R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224):

        # pylint: disable=too-many-arguments, disable=super-init-not-called
        # Convert the inverse-Γ parameters to the Γ parameters
        gamma = L_M/(L_M + L_sgm)  # Magnetic coupling factor
        self.p = p
        self.R_s = R_s
        self.L_s = L_M + L_sgm
        self.L_ell = L_sgm/gamma
        self.R_r = R_R/gamma**2
        # Initial values
        self.psi_ss0 = 0j
        self.psi_rs0 = 0j  # self.psi_rs0 = self.psi_Rs0/gamma
