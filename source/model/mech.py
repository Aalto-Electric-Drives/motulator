# pylint: disable=C0103
"""
This module contains continuous-time models for mechanical subsystems.

"""

from __future__ import annotations
from dataclasses import dataclass, field


# %%
@dataclass
class Mechanics:
    """
    Mechanics subsystem.

    This models an equation of motion for stiff mechanics.

    Parameters
    ----------
    J : float
        Total moment of inertia.
    B : float
        Viscous damping coefficient.
    tau_L_ext : function
        External load torque as a function of time, `tau_L_ext(t)`.

    """
    J: float = .015
    B: float = 0
    # Initial values
    w_M0: float = field(repr=False, default=0)
    theta_M0: float = field(repr=False, default=0)
    # Default: tau_L_ext(t) = 0
    tau_L_ext: float = field(repr=False, default=lambda t: 0)

    def f(self, t, w_M, tau_M):
        """
        Compute the state derivative.

        Parameters
        ----------
        t : float
            Time.
        w_M : float
            Rotor angular speed (in mechanical rad/s).
        tau_M : float
            Electromagnetic torque.

        Returns
        -------
        list, length 2
            Time derivative of the state vector.

        """
        dw_M = (tau_M - self.B*w_M - self.tau_L_ext(t))/self.J
        dtheta_M = w_M
        return [dw_M, dtheta_M]

    def meas_speed(self):
        """
        Measure the rotor speed.

        This returns the rotor speed at the end of the sampling period.

        Returns
        -------
        w_M0 : float
            Rotor angular speed (in mechanical rad/s).

        """
        # The quantization noise of an incremental encoder could be modeled
        # by means of the rotor angle, to be done later.
        return self.w_M0

    def meas_position(self):
        """
        Measure the rotor angle.

        This returns the rotor angle at the end of the sampling period.

        Returns
        -------
        theta_M0 : float
            Rotor angle (in mechanical rad).

        """
        return self.theta_M0
