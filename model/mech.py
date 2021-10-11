# pylint: disable=C0103
"""
This module contains continuous-time models for mechanical subsystems.

"""
from helpers import Step


# %%
class Mechanics:
    """
    This class represents the mechanical subsystem.

    """

    def __init__(self, J=.015, B=0, tau_L_ext=Step(.8, 14.6)):
        """
        Parameters
        ----------
        J : float, optional
            Total moment of inertia. The default is .015.
        B : float, optional
            Viscous damping coefficient. The default is 0.
        tau_L_ext : function, optional
            External load torque as a function of time, tau_L_ext(t). The
            default is Step(.8, 14.6).

        """
        self.J, self.B, self.tau_L_ext = J, B, tau_L_ext
        # Initial state
        self.w_M0 = 0
        self.theta_M0 = 0

    def f(self, t, w_M, tau_M):
        """
        Computes the state derivative.

        Parameters
        ----------
        t : float
            Time.
        w_M : float
            Rotor speed (in mechanical rad/s).
        tau_M : float
            Electromagnetic torque.

        Returns
        -------
        list, length 2
            Time derivative of the state vector.

        """
        dtheta_M = w_M
        dw_M = (tau_M - self.B*w_M - self.tau_L_ext(t))/self.J
        return [dtheta_M, dw_M]

    def meas_speed(self):
        """
        Returns
        -------
        w_M0 : float
            Rotor speed (in mechanical rad/s) at the end of the integration
            period.

        """
        # The quantization noise of an incremental encoder could be modeled
        # by means of the rotor angle, to be done later.
        return self.w_M0

    def meas_position(self):
        """
        Returns
        -------
        theta_M0 : float
            Rotor angle (in mechanical rad) at the end of the integration
            period.

        """
        return self.theta_M0

    def __str__(self):
        desc = ('Mechanics:\n'
                '    J={:.4f}  B={}')
        return desc.format(self.J, self.B)
