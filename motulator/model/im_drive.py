# pylint: disable=C0103
"""
This module contains continuous-time models for induction motor drives.

Peak-valued complex space vectors are used. The space vector models are
implemented in stator coordinates. The default values correspond to a 2.2-kW
induction motor.

"""
from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np

from motulator.helpers import abc2complex, Bunch
from motulator.model.im import InductionMotor, InductionMotorSaturated
from motulator.model.mech import Mechanics
from motulator.model.converter import Inverter, PWMInverter, FrequencyConverter


# %%
@dataclass
class InductionMotorDrive:
    """
    Continuous-time model for an induction motor drive.

    This interconnects the subsystems of an induction motor drive and provides
    an interface to the solver. More complicated systems could be modeled using
    a similar template.

    Parameters
    ----------
    motor : InductionMotor | InductionMotorSaturated
        Induction motor model.
    mech : Mechanics
        Mechanics model.
    conv : Inverter | PWMInverter
        Inverter model.

    """
    motor: InductionMotor | InductionMotorSaturated = InductionMotor()
    mech: Mechanics = Mechanics()
    conv: Inverter | PWMInverter = Inverter()
    # Stores the solution data
    data: Bunch = field(repr=False, default_factory=Bunch)
    # Initial time
    t0: float = field(repr=False, default=0)

    def __post_init__(self):
        # Store the solution in these lists
        self.data.t, self.data.q = [], []
        self.data.psi_ss, self.data.psi_rs = [], []
        self.data.theta_M, self.data.w_M = [], []

    def get_initial_values(self):
        """
        Get the initial values.

        Returns
        -------
        x0 : complex list, length 4
            Initial values of the state variables.

        """
        x0 = [self.motor.psi_ss0, self.motor.psi_rs0,
              self.mech.w_M0, self.mech.theta_M0]
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
        self.motor.psi_ss0 = x0[0]
        self.motor.psi_rs0 = x0[1]
        # x0[2].imag and x0[3].imag are always zero
        self.mech.w_M0 = x0[2].real
        self.mech.theta_M0 = x0[3].real
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
        psi_ss, psi_rs, w_M, _ = x
        # Interconnections: outputs for computing the state derivatives
        u_ss = self.conv.ac_voltage(self.conv.q, self.conv.u_dc0)
        i_ss, _ = self.motor.currents(psi_ss, psi_rs)
        tau_M = self.motor.torque(psi_ss, i_ss)
        # State derivatives
        motor_f = self.motor.f(psi_ss, psi_rs, u_ss, w_M)
        mech_f = self.mech.f(t, w_M, tau_M)
        # List of state derivatives
        return motor_f + mech_f

    def save(self, sol):
        """
        Save the solution.

        Parameters
        ----------
        sol : Bunch object
            Solution from the solver.

        """
        self.data.t.extend(sol.t)
        self.data.q.extend(sol.q)
        self.data.psi_ss.extend(sol.y[0])
        self.data.psi_rs.extend(sol.y[1])
        self.data.w_M.extend(sol.y[2].real)
        self.data.theta_M.extend(sol.y[3].real)

    def post_process(self):
        """
        Transform the lists to the ndarray format and post-process them.

        """
        # From lists to the ndarray
        for key in self.data:
            self.data[key] = np.asarray(self.data[key])
        # Some useful variables
        self.data.i_ss, _ = self.motor.currents(self.data.psi_ss,
                                                self.data.psi_rs)
        self.data.theta_m = self.motor.p*self.data.theta_M
        self.data.theta_m = np.mod(self.data.theta_m, 2*np.pi)
        self.data.w_m = self.motor.p*self.data.w_M
        self.data.tau_M = self.motor.torque(self.data.psi_ss, self.data.i_ss)
        self.data.tau_L = (self.mech.tau_L_ext(self.data.t)
                           + self.mech.B*self.data.w_M)
        self.data.u_ss = self.conv.ac_voltage(self.data.q,
                                              self.conv.u_dc0)
        # Compute the inverse-Gamma rotor flux
        try:
            # Saturable stator inductance
            L_s = self.motor.L_s(np.abs(self.data.psi_ss))
        except TypeError:
            # Constant stator inductance
            L_s = self.motor.L_s
        gamma = L_s/(L_s + self.motor.L_ell)  # Magnetic coupling factor
        self.data.psi_Rs = gamma*self.data.psi_rs


# %%
@dataclass
class InductionMotorDriveDiode(InductionMotorDrive):
    """
    Induction motor drive equipped with a diode bridge.

    This models extends the InductionMotorDrive class with a model for a
    three-phase diode bridge fed from stiff supply voltages. The DC bus is
    modeled as an inductor and a capacitor.

    Parameters
    ----------

    conv : FrequencyConverter
        Frequency converter model.

    """
    conv: FrequencyConverter = None

    def __post_init__(self):
        # Extends the base class, store the solution in these lists
        self.data.t, self.data.q = [], []
        self.data.psi_ss, self.data.psi_rs = [], []
        self.data.theta_M, self.data.w_M = [], []
        self.data.u_dc, self.data.i_L = [], []

    def get_initial_values(self):
        # Extends the base class
        x0 = super().get_initial_values() + [self.conv.u_dc0, self.conv.i_L0]
        return x0

    def set_initial_values(self, t0, x0):
        # Extends the base class
        super().set_initial_values(t0, x0[0:4])
        self.conv.u_dc0 = x0[4].real
        self.conv.i_L0 = x0[5].real

    def f(self, t, x):
        # Overrides the base class, unpack the states for better readability
        psi_ss, psi_rs, w_M, _, u_dc, i_L = x
        # Interconnections: outputs for computing the state derivatives
        i_ss, _ = self.motor.currents(psi_ss, psi_rs)
        u_ss = self.conv.ac_voltage(self.conv.q, u_dc)
        i_dc = self.conv.dc_current(self.conv.q, i_ss)
        tau_M = self.motor.torque(psi_ss, i_ss)
        # Returns the list of state derivatives
        return (self.motor.f(psi_ss, psi_rs, u_ss, w_M) +
                self.mech.f(t, w_M, tau_M) +
                self.conv.f(t, u_dc, i_L, i_dc))

    def save(self, sol):
        # Extends the base class
        super().save(sol)
        self.data.u_dc.extend(sol.y[4].real)
        self.data.i_L.extend(sol.y[5].real)

    def post_process(self):
        # Extends the base class
        super().post_process()
        # From lists to the ndarray
        self.data.u_dc = np.asarray(self.data.u_dc)
        self.data.i_L = np.asarray(self.data.i_L)
        # Some useful variables
        self.data.u_ss = self.conv.ac_voltage(self.data.q, self.data.u_dc)
        self.data.i_dc = self.conv.dc_current(self.data.q, self.data.i_ss)
        u_g_abc = self.conv.grid_voltages(self.data.t)
        self.data.u_g = abc2complex(u_g_abc)
        # Voltage at the output of the diode bridge
        self.data.u_di = np.amax(u_g_abc, 0) - np.amin(u_g_abc, 0)
        # Diode briddge switching states (-1, 0, 1)
        q_g_abc = ((np.amax(u_g_abc, 0) == u_g_abc).astype(int) -
                   (np.amin(u_g_abc, 0) == u_g_abc).astype(int))
        # Grid current space vector
        self.data.i_g = abc2complex(q_g_abc)*self.data.i_L
