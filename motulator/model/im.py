"""
Continuous-time models for induction motors.

Peak-valued complex space vectors are used. The space vector models are
implemented in stator coordinates. 

"""
import numpy as np
from motulator.helpers import complex2abc


# %%
class InductionMotor:
    """
    Γ-equivalent model of an induction motor.

    An induction motor is modeled using the Γ-equivalent model [1]_. The model
    is implemented in stator coordinates. The flux linkages are used as state
    variables.

    Parameters
    ----------
    n_p : int
        Number of pole pairs.
    R_s : float
        Stator resistance (Ohm).
    R_r : float
        Rotor resistance (Ohm).
    L_ell : float
        Leakage inductance (H).
    L_s : float
        Stator inductance (H).

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

    def __init__(self, n_p, R_s, R_r, L_ell, L_s):
        # pylint: disable=too-many-arguments
        self.n_p = n_p
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
            Stator flux linkage (Vs).
        psi_rs : complex
            Rotor flux linkage (Vs).

        Returns
        -------
        i_ss : complex
            Stator current (A).
        i_rs : complex
            Rotor current (A).

        """
        i_rs = (psi_rs - psi_ss)/self.L_ell
        i_ss = psi_ss/self.L_s - i_rs

        return i_ss, i_rs

    def magnetic(self, psi_ss, psi_rs):
        """
        Magnetic model.

        Parameters
        ----------
        psi_ss : complex
            Stator flux linkage (Vs).
        psi_rs : complex
            Rotor flux linkage (Vs).

        Returns
        -------
        i_ss : complex
            Stator current (A).
        i_rs : complex
            Rotor current (A).
        tau_M : float
            Electromagnetic torque (Nm).

        """
        i_ss, i_rs = self.currents(psi_ss, psi_rs)
        tau_M = 1.5*self.n_p*np.imag(i_ss*np.conj(psi_ss))

        return i_ss, i_rs, tau_M

    def f(self, psi_ss, psi_rs, u_ss, w_M):
        """
        Compute the state derivatives.

        Parameters
        ----------
        psi_ss : complex
            Stator flux linkage (Vs).
        psi_rs : complex
            Rotor flux linkage (Vs).
        u_ss : complex
            Stator voltage (V).
        w_M : float
            Rotor angular speed (mechanical rad/s).

        Returns
        -------
        complex list, length 2
            Time derivative of the state vector, [dpsi_ss, dpsi_rs]
        i_ss : complex
            Stator current (A).
        tau_M : float
            Electromagnetic torque (Nm).

        Notes
        -----
        In addition to the state derivatives, this method also returns the
        output signals (stator current `i_ss` and torque `tau_M`) needed for
        interconnection with other subsystems. This avoids overlapping
        computation in simulation.

        """
        i_ss, i_rs, tau_M = self.magnetic(psi_ss, psi_rs)
        dpsi_ss = u_ss - self.R_s*i_ss
        dpsi_rs = -self.R_r*i_rs + 1j*self.n_p*w_M*psi_rs

        return [dpsi_ss, dpsi_rs], i_ss, tau_M

    def meas_currents(self):
        """
        Measure the phase currents at the end of the sampling period.

        Returns
        -------
        i_s_abc : 3-tuple of floats
            Phase currents (A).

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
    model::

        L_s = L_s(abs(psi_ss))

    Parameters
    ----------
    n_p : int
        Number of pole pairs.
    R_s : float
        Stator resistance (Ohm).
    R_r : float
        Rotor resistance (Ohm).
    L_ell : float
        Leakage inductance (H).
    L_s : Callable[[float], float]
        Stator inductance (H) as a function of the stator-flux magnitude.

    """

    def currents(self, psi_ss, psi_rs):
        """Override the base class method."""
        # Saturated value of the stator inductance.
        L_s = self.L_s(np.abs(psi_ss))
        # Currents
        i_rs = (psi_rs - psi_ss)/self.L_ell
        i_ss = psi_ss/L_s - i_rs
        return i_ss, i_rs


# %%
class InductionMotorInvGamma(InductionMotor):
    """
    Inverse-Γ model of an induction motor.

    This extends the InductionMotor class (based on the Γ model) by providing
    an interface for the inverse-Γ model parameters. Linear magnetics are
    assumed. If magnetic saturation is to be modeled, the Γ model is preferred.

    Parameters
    ----------
    n_p : int
        Number of pole pairs.
    R_s : float
        Stator resistance (Ohm).
    R_R : float
        Rotor resistance (Ohm).
    L_sgm : float
        Leakage inductance (H).
    L_M : float
        Magnetizing inductance (H).

    """

    def __init__(self, n_p, R_s, R_R, L_sgm, L_M):
        # pylint: disable=too-many-arguments, disable=super-init-not-called
        # Convert the inverse-Γ parameters to the Γ parameters
        gamma = L_M/(L_M + L_sgm)  # Magnetic coupling factor
        self.n_p = n_p
        self.R_s = R_s
        self.L_s = L_M + L_sgm
        self.L_ell = L_sgm/gamma
        self.R_r = R_R/gamma**2
        # Initial values
        self.psi_ss0 = 0j
        self.psi_rs0 = 0j  # self.psi_rs0 = self.psi_Rs0/gamma
