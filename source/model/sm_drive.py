# pylint: disable=C0103
"""
This module contains continuous-time models for synchronous motor drives.

The motor model can be parametrized to represent a permanent-magnet synchronous
motor and synchronous reluctance motor. Peak-valued complex space vectors are
used.

"""
from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np
from sklearn.utils import Bunch
from helpers import complex2abc
from model.mech import Mechanics
from model.converter import Inverter, PWMInverter


# %%
@dataclass
class SynchronousMotorDrive:
    """
    Model of a synchronous motor drive.

    This interconnects the subsystems of a synchronous motor drive and provides
    the interface to the solver. More complicated systems could be simulated
    using a similar template.

    """
    motor: SynchronousMotor = None
    mech: Mechanics = None
    conv: Inverter | PWMInverter = None
    data: Bunch = field(repr=False, default_factory=Bunch)
    t0: float = field(repr=False, default=0)

    def __post_init__(self):
        # Store the solution in these lists
        self.data.t, self.data.q = [], []
        self.data.psi_s, self.data.theta_M, self.data.w_M = [], [], []

    def get_initial_values(self):
        """
        Get the initial values.

        Returns
        -------
        x0 : complex list, length 2
            Initial values of the state variables.

        """
        x0 = [self.motor.psi_s0, self.mech.w_M0, self.mech.theta_M0]
        return x0

    def set_initial_values(self, t0, x0):
        """
        Set the initial values.

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
        u_ss = self.conv.ac_voltage(self.conv.q, self.conv.u_dc0)
        u_s = np.exp(-1j*theta_m)*u_ss  # Stator voltage in rotor coordinates
        i_s = self.motor.current(psi_s)
        tau_M = self.motor.torque(psi_s, i_s)

        # State derivatives
        motor_f = self.motor.f(psi_s, i_s, u_s, w_M)
        mech_f = self.mech.f(t, w_M, tau_M)

        # List of state derivatives
        return motor_f + mech_f

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

    def post_process(self):
        """
        Transform the lists to the ndarray format and post-process them.

        """
        # From lists to the ndarray
        for key in self.data:
            self.data[key] = np.asarray(self.data[key])

        # Compute some useful quantities
        self.data.i_s = self.motor.current(self.data.psi_s)
        self.data.w_m = self.motor.p*self.data.w_M
        self.data.tau_M = self.motor.torque(self.data.psi_s, self.data.i_s)
        self.data.tau_L = (self.mech.tau_L_ext(self.data.t)
                           + self.mech.B*self.data.w_M)
        self.data.u_ss = self.conv.ac_voltage(self.data.q,
                                              self.conv.u_dc0)
        self.data.theta_m = self.motor.p*self.data.theta_M
        self.data.theta_m = np.mod(self.data.theta_m, 2*np.pi)


# %%
@dataclass
class SynchronousMotor:
    """
    Synchronous motor.

    This models a synchronous motor. The model is implemented in rotor
    coordinates.

    Parameters
    ----------
    mech : Mechanics object
        Mechanical subsystem model.

    Attributes
    ----------
    R_s : float
        Stator resistance.
    L_d : float
        d-axis inductance.
    L_q : float
        q-axis inductance.
    psi_f : float
        PM-flux linkage.
    p : int
        Number of pole pairs.
    psi_s0 : complex
        Initial value of the stator flux linkage.

    """

    p: int = 3
    R_s: float = 3.6
    L_d: float = .036
    L_q: float = .051
    psi_f: float = .545
    # Initial value
    psi_s0: complex = field(repr=False, init=False)
    # Needed for coordinate transformations
    mech: Mechanics = None

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
        theta_m0 = self.p*self.mech.theta_M0
        i_s_abc = complex2abc(np.exp(1j*theta_m0)*i_s0)
        return i_s_abc

