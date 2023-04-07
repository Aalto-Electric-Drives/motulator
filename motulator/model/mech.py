"""Continuous-time models for mechanical subsystems."""


# %%
class Mechanics:
    """
    Mechanics subsystem.

    This models an equation of motion for stiff mechanics.

    Parameters
    ----------
    J : float
        Total moment of inertia.
    tau_L_w : function
        Load torque as function of speed, `tau_L_w(w_M)`. For example,
        tau_L_w = b*w_M, where b is the viscous friction coefficient.
    tau_L_t : function
        Load torque as a function of time, `tau_L_t(t)`.

    """

    def __init__(
            self, J=.015, tau_L_w=lambda w_M: 0*w_M, tau_L_t=lambda t: 0*t):
        self.J = J
        self.tau_L_t = tau_L_t
        self.tau_L_w = tau_L_w
        # Initial values
        self.w_M0, self.theta_M0 = 0, 0

    def f(self, t, w_M, tau_M):
        """
        Compute the state derivatives.

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
            Time derivatives of the state vector.

        """
        # Total load torque
        tau_L = self.tau_L_w(w_M) + self.tau_L_t(t)
        # Time derivatives
        dw_M = (tau_M - tau_L)/self.J
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


# %%
class MechanicsTwoMass(Mechanics):
    """
    Two-mass mechanics subsystem.

    This models an equation of motion for two-mass mechanics.

    Parameters
    ----------
    J_M : float
        Moment of inertia of the motor.
    J_L : float
        Moment of inertia of the load.
    K_S : float
        Torsional stiffness of the shaft.
    C_S : float
        Torsional damping of the shaft.
    tau_L_w : function
        Load torque as function of the load speed, `tau_L_w(w_L)`. For example,
        tau_L_w = b*w_L, where b is the viscous friction coefficient.
    tau_L_t : function
        Load torque as a function of time, `tau_L_t(t)`.

    """

    # pylint: disable=too-many-instance-attributes
    def __init__(
            self,
            J_M=.005,
            J_L=.005,
            K_S=700.,
            C_S=.13,
            tau_L_w=lambda w_M: 0*w_M,
            tau_L_t=lambda t: 0*t):
        # pylint: disable=too-many-arguments
        # pylint: disable=super-init-not-called
        self.J_M = J_M
        self.J_L = J_L
        self.K_S = K_S
        self.C_S = C_S
        self.tau_L_t = tau_L_t
        self.tau_L_w = tau_L_w
        # Initial values
        self.w_M0, self.theta_M0, self.w_L0, self.theta_ML0 = 0, 0, 0, 0

    def f(self, t, w_M, w_L, theta_ML, tau_M):
        # pylint: disable=too-many-arguments
        # pylint: disable=arguments-differ
        """
        Compute the state derivatives.

        Parameters
        ----------
        t : float
            Time.
        w_M : float
            Rotor angular speed (in mechanical rad/s).
        w_L : float
            Load angular speed (in mechanical rad/s).
        theta_ML : float
            Twist angle, theta_M - theta_L (in mechanical rad).
        tau_M : float
            Electromagnetic torque.

        Returns
        -------
        list, length 4
            Time derivatives of the state vector.

        """
        # Total load torque
        tau_L = self.tau_L_w(w_L) + self.tau_L_t(t)
        # Shaft torque
        tau_S = self.K_S*theta_ML + self.C_S*(w_M - w_L)
        # Time derivatives
        dw_M = (tau_M - tau_S)/self.J_M
        dtheta_M = w_M
        dw_L = (tau_S - tau_L)/self.J_L
        dtheta_ML = w_M - w_L

        return [dw_M, dtheta_M, dw_L, dtheta_ML]

    def meas_load_speed(self):
        """
        Measure the load speed.

        This returns the load speed at the end of the sampling period.

        Returns
        -------
        w_L0 : float
            Load angular speed (in mechanical rad/s).

        """
        return self.w_L0

    def meas_load_position(self):
        """
        Measure the load angle.

        This returns the load angle at the end of the sampling period.

        Returns
        -------
        theta_L0 : float
            Rotor angle (in mechanical rad).

        """
        theta_L0 = self.theta_M0 - self.theta_ML0
        return theta_L0
