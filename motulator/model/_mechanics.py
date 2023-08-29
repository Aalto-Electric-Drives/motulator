"""Continuous-time models for mechanical subsystems.

"""


# %%
class Mechanics:
    """
    Mechanics subsystem.

    This models an equation of motion for stiff mechanics.

    Parameters
    ----------
    J : float
        Total moment of inertia (kgm²).
    tau_L_w : callable
        Load torque (Nm) as a function of speed, `tau_L_w(w_M)`. For example,
        ``tau_L_w = b*w_M``, where `b` is the viscous friction coefficient. The
        default is zero, ``lambda w_M: 0*w_M``.
    tau_L_t : callable
        Load torque (Nm) as a function of time, `tau_L_t(t)`. The default is 
        zero, ``lambda t: 0*t``.

    Methods
    -------
    f(t, w_M, tau_M)
        Compute the state derivatives.
    meas_speed()   
        Measure the rotor speed.
    meas_position()
        Measure the rotor angle.
        
    """

    def __init__(self, J, tau_L_w=lambda w_M: 0*w_M, tau_L_t=lambda t: 0*t):
        self.J = J
        self.tau_L_t, self.tau_L_w = tau_L_t, tau_L_w
        # Initial values
        self.w_M0, self.theta_M0 = 0, 0

    def f(self, t, w_M, tau_M):
        """
        Compute the state derivatives.

        Parameters
        ----------
        t : float
            Time (s).
        w_M : float
            Rotor angular speed (mechanical rad/s).
        tau_M : float
            Electromagnetic torque (Nm).

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
            Rotor angular speed (mechanical rad/s).

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
            Rotor angle (mechanical rad).

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
        Motor moment of inertia (kgm²).
    J_L : float
        Load moment of inertia (kgm²).
    K_S : float
        Shaft torsional stiffness (Nm).
    C_S : float
        Shaft torsional damping (Nms).
    tau_L_w : callable
        Load torque (Nm) as a function of the load speed, `tau_L_w(w_L)`, e.g., 
        ``tau_L_w = B*w_L``, where `B` is the viscous friction coefficient. The
        default is zero, ``lambda w_L: 0*w_L``.
    tau_L_t : callable
        Load torque (Nm) as a function of time, `tau_L_t(t)`. The default is
        zero, ``lambda t: 0*t``.
    
    Methods
    -------
    f(t, w_M, w_L, theta_ML, tau_M)
        Compute the state derivatives.
    meas_load_speed()
        Measure the load speed.
    meas_load_position()
        Measure the load angle.

    """

    # pylint: disable=too-many-instance-attributes
    def __init__(self, J_M, J_L, K_S, C_S, tau_L_w=None, tau_L_t=None):
        # pylint: disable=too-many-arguments
        # pylint: disable=super-init-not-called
        self.J_M, self.J_L, self.K_S, self.C_S = J_M, J_L, K_S, C_S
        self.tau_L_w = tau_L_w if tau_L_w is not None else lambda w_L: 0*w_L
        self.tau_L_t = tau_L_t if tau_L_t is not None else lambda t: 0*t
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
            Time (s).
        w_M : float
            Rotor angular speed (mechanical rad/s).
        w_L : float
            Load angular speed (mechanical rad/s).
        theta_ML : float
            Twist angle, theta_M - theta_L (mechanical rad).
        tau_M : float
            Electromagnetic torque (Nm).

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
            Load angular speed (mechanical rad/s).

        """
        return self.w_L0

    def meas_load_position(self):
        """
        Measure the load angle.

        This returns the load angle at the end of the sampling period.

        Returns
        -------
        theta_L0 : float
            Rotor angle (mechanical rad).

        """
        theta_L0 = self.theta_M0 - self.theta_ML0
        return theta_L0
