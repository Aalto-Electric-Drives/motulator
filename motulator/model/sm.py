# pylint: disable=C0103
"""
This module contains continuous-time models for synchronous motors.

The motor model can be parametrized to represent permanent-magnet synchronous
motors and synchronous reluctance motors. Peak-valued complex space vectors are
used. The default values correspond to a 2.2-kW permanent-magnet synchronous
motor.

"""
from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np
from helpers import complex2abc
from model.mech import Mechanics


# %%
@dataclass
class SynchronousMotor:
    """
    Synchronous motor model.

    This models a synchronous motor in rotor coordinates.

    Parameters
    ----------
    p : int
        Number of pole pairs.
    R_s : float
        Stator resistance.
    L_d : float
        d-axis inductance.
    L_q : float
        q-axis inductance.
    psi_f : float
        PM flux linkage.

    """
    p: int = 3
    R_s: float = 3.6
    L_d: float = .036
    L_q: float = .051
    psi_f: float = .545
    # The rotor position from the mechanics subsystem is needed only for the
    # coordinate transformation in the measure_currents method
    _mech: Mechanics = field(repr=False, default=None)
    # Initial value
    psi_s0: complex = field(repr=False, init=False)

    def __post_init__(self):
        self.psi_s0 = self.psi_f + 0j

    def current(self, psi_s):
        """
        Compute the stator current.

        Parameters
        ----------
        psi_s : complex
            Stator flux linkage.

        Returns
        -------
        i_s : complex
            Stator current.

        """
        i_s = (psi_s.real - self.psi_f)/self.L_d + 1j*psi_s.imag/self.L_q
        return i_s

    def torque(self, psi_s, i_s):
        """
        Compute the electromagnetic torque.

        Parameters
        ----------
        psi_s : complex
            Stator flux linkage.
        i_s : complex
            Stator current.

        Returns
        -------
        tau_M : float
            Electromagnetic torque.

        """
        tau_M = 1.5*self.p*np.imag(i_s*np.conj(psi_s))
        return tau_M

    def f(self, psi_s, i_s, u_s, w_M):
        """
        Compute the state derivative.

        Parameters
        ----------
        psi_s : complex
            Stator flux linkage.
        u_s : complex
            Stator voltage.
        w_M : float
            Rotor angular speed (in mechanical rad/s).

        Returns
        -------
        dpsi_s : complex
            Time derivative of the stator flux linkage.

        """
        dpsi_s = u_s - self.R_s*i_s - 1j*self.p*w_M*psi_s
        return [dpsi_s]

    def meas_currents(self):
        """
        Measure the phase currents at the end of the sampling period.

        Returns
        -------
        i_s_abc : 3-tuple of floats
            Phase currents.

        """
        i_s0 = self.current(self.psi_s0)
        theta_m0 = self.p*self._mech.theta_M0
        i_s_abc = complex2abc(np.exp(1j*theta_m0)*i_s0)
        return i_s_abc
