# pylint: disable=C0103
"""
This module contains continuous-time models for mechanical subsystems.

"""
from helpers import Step


# %%
class Mechanics:
    """
    Mechanical subsystem.

    This models an equation of motion for stiff mechanics.

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

    def __str__(self):
        desc = (('Mechanics:\n'
                 '    J={:.4f}  B={}  w_M0={}  theta_M0={}\n')
                .format(self.J, self.B, self.w_M0, self.theta_M0))
        return desc
