"""
Continuous-time models for synchronous motors.

The motor models can be parametrized to represent permanent-magnet synchronous
motors and synchronous reluctance motors. Peak-valued complex space vectors are
used.

"""
import numpy as np
from motulator.helpers import complex2abc


# %%
class SynchronousMotor:
    """
    Synchronous motor model.

    This models a synchronous motor in rotor coordinates. The stator flux
    linkage is the state variable. 

    Parameters
    ----------
    n_p : int
        Number of pole pairs.
    R_s : float
        Stator resistance (Ohm).
    L_d : float
        d-axis inductance (H).
    L_q : float
        q-axis inductance (H).
    psi_f : float
        PM-flux linkage (Vs).

    """

    def __init__(self, n_p, R_s, L_d, L_q, psi_f):
        # pylint: disable=too-many-arguments
        self.n_p, self.R_s = n_p, R_s
        self.L_d, self.L_q, self.psi_f = L_d, L_q, psi_f
        # Initial value
        self.psi_s0 = complex(psi_f)
        # For the coordinate transformation
        self._mech = None

    def current(self, psi_s):
        """
        Compute the stator current.

        Parameters
        ----------
        psi_s : complex
            Stator flux linkage (Vs).

        Returns
        -------
        i_s : complex
            Stator current (A).

        """
        i_s = (psi_s.real - self.psi_f)/self.L_d + 1j*psi_s.imag/self.L_q
        return i_s

    def magnetic(self, psi_s):
        """
        Magnetic model.

        Parameters
        ----------
        psi_s : complex
            Stator flux linkage (Vs).

        Returns
        -------
        i_s : complex
            Stator current (A).
        tau_M : float
            Electromagnetic torque (Nm).

        """
        i_s = self.current(psi_s)
        tau_M = 1.5*self.n_p*np.imag(i_s*np.conj(psi_s))
        return i_s, tau_M

    def f(self, psi_s, u_s, w_M):
        """
        Compute the state derivative.

        Parameters
        ----------
        psi_s : complex
            Stator flux linkage (Vs).
        u_s : complex
            Stator voltage (V).
        w_M : float
            Rotor angular speed (mechanical rad/s).

        Returns
        -------
        dpsi_s : complex list
            Time derivative of the stator flux linkage (V).
        i_s : complex
            Stator current (A).
        tau_M : float
            Electromagnetic torque (Nm).

        Notes
        -----
        In addition to the state derivative, this method also returns the
        output signals (stator current `i_ss` and torque `tau_M`) needed for
        interconnection with other subsystems. This avoids overlapping
        computation in simulation.

        """
        i_s, tau_M = self.magnetic(psi_s)
        dpsi_s = u_s - self.R_s*i_s - 1j*self.n_p*w_M*psi_s
        return [dpsi_s], i_s, tau_M

    def meas_currents(self):
        """
        Measure the phase currents at the end of the sampling period.

        Returns
        -------
        i_s_abc : 3-tuple of floats
            Phase currents (A).

        """
        i_s0 = self.current(self.psi_s0)
        theta_m0 = self.n_p*self._mech.theta_M0
        i_s_abc = complex2abc(np.exp(1j*theta_m0)*i_s0)
        return i_s_abc


# %%
class SynchronousMotorSaturated(SynchronousMotor):
    """
    Model of a saturated synchronous motor.

    This overrides the linear magnetics model of the SynchronousMotor class
    with a generic saturation model::

        i_s = i_s(psi_s)
        
    The saturation model could be an analytical function or a look-up table. 

    Parameters
    ----------
    n_p : int
        Number of pole pairs.
    R_s : float
        Stator resistance (Ohm).
    current : callable
        Function that computes the stator current `i_s` as a function of the 
        stator flux linkage `psi_s`. 
    psi_s0 : complex, optional
        Initial value of the stator flux linkage (Vs). The default is 0j. For
        PM motors, this should be solved from the the saturation model.

    """

    # pylint: disable=too-many-instance-attributes
    def __init__(self, n_p, R_s, current, psi_s0=0j):
        # pylint: disable=too-many-arguments, disable=super-init-not-called
        self.n_p, self.R_s = n_p, R_s
        self.current = current
        # For the coordinate transformation
        self._mech = None
        # Initial value of the stator flux linkage
        self.psi_s0 = complex(psi_s0)
