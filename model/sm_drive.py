# pylint: disable=C0103
"""
This module includes continuous-time models for a permmanent-magnet
synchronous motor drive. The space-vector model is implemented in rotor
coordinates.

"""
import numpy as np
from sklearn.utils import Bunch
from helpers import complex2abc


# %%
class Drive:
    """
    This class interconnects the subsystems of a PMSM drive and provides an
    interface to the solver.

    """

    def __init__(self, motor, mech, converter, delay, pwm, datalog):
        """
        Instantiate the classes.

        """
        self.motor = motor
        self.mech = mech
        self.converter = converter
        self.delay = delay
        self.pwm = pwm
        self.datalog = datalog
        self.q = 0                  # Switching-state space vector
        self.t0 = 0                 # Initial simulation time

    def get_initial_values(self):
        """
        Returns
        -------
        x0 : complex list, length 2
            Initial values of the state variables.

        """
        x0 = [self.motor.psi_s0, self.mech.w_M0, self.mech.theta_M0]
        return x0

    def set_initial_values(self, t0, x0):
        """
        Parameters
        ----------
        x0 : complex ndarray
            Initial values of the state variables.

        """
        self.t0 = t0
        self.motor.psi_s0 = x0[0]
        self.mech.w_M0 = x0[1].real         # x0[2].imag is always zero
        self.mech.theta_M0 = x0[2].real     # x0[1].imag is always zero
        # Limit the angle [0, 2*pi]
        self.mech.theta_M0 = np.mod(self.mech.theta_M0, 2*np.pi)

    def f(self, t, x):
        """
        Compute the complete state derivative list for the solver.

        Parameters
        ----------
        t : float
            Time.
        x : complex ndarray
            State vector.

        Returns
        -------
        complex list
            State derivatives.

        """
        # Unpack the states
        psi_s, w_M, theta_M = x
        theta_m = self.motor.p*theta_M
        # Interconnections: outputs for computing the state derivatives
        u_ss = self.converter.ac_voltage(self.q, self.converter.u_dc0)
        u_s = np.exp(-1j*theta_m)*u_ss  # Stator voltage in rotor coordinates
        i_s = self.motor.current(psi_s)
        tau_M = self.motor.torque(psi_s, i_s)
        # State derivatives
        motor_f = self.motor.f(psi_s, i_s, u_s, w_M)
        mech_f = self.mech.f(t, w_M, tau_M)
        # List of state derivatives
        return motor_f + mech_f

    def __str__(self):
        desc = ('System: Synchronous motor drive\n'
                '-------------------------------\n')
        desc += (self.delay.__str__() + self.pwm.__str__()
                 + self.converter.__str__() + self.motor.__str__()
                 + self.mech.__str__())
        return desc


# %%
class Motor:
    """
    This class represents a permanent-magnet synchronous motor. The
    peak-valued complex space vectors are used.

    """

    def __init__(self, mech, R_s=3.6, L_d=.036, L_q=.051, psi_f=.545, p=3):
        """
        The default values correspond to the 2.2-kW PMSM.

        Parameters
        ----------
        mech : object
            Mechanics, needed for computing the measured phase currents.
        R_s : float, optional
            Stator resistance. The default is 3.6.
        L_d : float, optional
            d-axis inductance. The default is .036.
        L_q : float, optional
            q-axis inductance. The default is .051.
        psi_f : float, optional
            PM-flux linkage. The default is .545.
        p : int, optional
            Number of pole pairs. The default is 3.

        """
        self.R_s, self.L_d, self.L_q, self.psi_f = R_s, L_d, L_q, psi_f
        self.p = p
        self.mech = mech
        self.psi_s0 = psi_f + 0j

    def current(self, psi_s):
        """
        Compute the stator current.

        Parameters
        ----------
        psi_s : complex
            Stator flux linkage in rotor coordinates.

        Returns
        -------
        i_s : complex
            Stator current in rotor coordinates.

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
            Stator flux linkage in rotor coordinates.
        u_s : complex
            Stator voltage in rotor coordinates.
        w_M : float
            Rotor speed (in mechanical rad/s).

        Returns
        -------
        dpsi_s : complex
            Time derivative of the stator flux linkage.

        """
        dpsi_s = u_s - self.R_s*i_s - 1j*self.p*w_M*psi_s
        return [dpsi_s]

    def meas_currents(self):
        """
        Return the phase currents at the end of the sampling period.

        Returns
        -------
        i_s_abc : 3-tuple of floats
            Phase currents.

        """
        i_s0 = self.current(self.psi_s0)
        theta_m0 = self.p*self.mech.theta_M0
        i_s_abc = complex2abc(np.exp(1j*theta_m0)*i_s0)
        return i_s_abc

    def __str__(self):
        desc = (('Synchronous motor:\n'
                 '    p={}  R_s={}  L_d={}  L_q={}  psi_f={}\n')
                .format(self.p, self.R_s, self.L_d, self.L_q, self.psi_f))
        return desc


# %%
class Datalogger:
    """
    This class contains a datalogger. Here, stator coordinates are marked
    with additional s, e.g. i_ss is the stator current in stator coordinates.

    """

    def __init__(self):
        """
        Initialize the attributes.

        """
        self.data = Bunch()
        self.data.t, self.data.q = [], []
        self.data.psi_s, self.data.theta_M, self.data.w_M = [], [], []

    def save(self, sol):
        """
        Save the solution.

        Parameters
        ----------
        sol : bunch object
            Solution from the solver.

        """
        self.data.t.extend(sol.t)
        self.data.q.extend(sol.q)
        self.data.psi_s.extend(sol.y[0])
        self.data.w_M.extend(sol.y[1].real)
        self.data.theta_M.extend(sol.y[2].real)

    def post_process(self, mdl):
        """
        Transform the lists to the ndarray format and post-process them.

        """
        # From lists to the ndarray
        for key in self.data:
            self.data[key] = np.asarray(self.data[key])

        # Compute some useful quantities
        self.data.i_s = mdl.motor.current(self.data.psi_s)
        self.data.w_m = mdl.motor.p*self.data.w_M
        self.data.tau_M = mdl.motor.torque(self.data.psi_s, self.data.i_s)
        self.data.tau_L = (mdl.mech.tau_L_ext(self.data.t)
                           + mdl.mech.B*self.data.w_M)
        self.data.u_ss = mdl.converter.ac_voltage(self.data.q,
                                                  mdl.converter.u_dc0)
        self.data.theta_m = mdl.motor.p*self.data.theta_M
        self.data.theta_m = np.mod(self.data.theta_m, 2*np.pi)
