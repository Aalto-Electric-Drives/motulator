"""
Continuous-time models for synchronous motor drives.

Peak-valued complex space vectors are used.

"""
import numpy as np
from motulator.helpers import Bunch


# %%
class SynchronousMotorDrive:
    """
    Continuous-time model for a synchronous motor drive.

    This interconnects the subsystems of a synchronous motor drive and provides
    an interface to the solver. More complicated systems could be modeled using
    a similar template.

    Parameters
    ----------
    motor : SynchronousMotor
        Synchronous motor model.
    mech : Mechanics
        Mechanics model.
    conv : Inverter
        Inverter model.

    """

    def __init__(self, motor, mech, conv):
        self.motor = motor
        self.motor._mech = mech
        self.mech = mech
        self.conv = conv

        # Initial time
        self.t0 = 0

        # Store the solution in these lists
        self.data = Bunch()
        self.data.t, self.data.q = [], []
        self.data.psi_s, self.data.theta_M, self.data.w_M = [], [], []

    def get_initial_values(self):
        """
        Get the initial values.

        Returns
        -------
        x0 : complex list, length 3
            Initial values of the state variables.

        """
        x0 = [
            self.motor.psi_s0,
            self.mech.w_M0,
            self.mech.theta_M0,
        ]
        return x0

    def set_initial_values(self, t0, x0):
        """
        Set the initial values.

        Parameters
        ----------
        t0 : float
            Initial time (s).
        x0 : complex ndarray
            Initial values of the state variables.

        """
        self.t0 = t0
        self.motor.psi_s0 = x0[0]
        # x0[1].imag and x0[2].imag are always zero
        self.mech.w_M0 = x0[1].real
        # Limit theta_M0 = x0[2].real into [-pi, pi)
        self.mech.theta_M0 = np.mod(x0[2].real + np.pi, 2*np.pi) - np.pi

    def f(self, t, x):
        """
        Compute the complete state derivative list for the solver.

        Parameters
        ----------
        t : float
            Time (s).
        x : complex ndarray
            State vector.

        Returns
        -------
        complex list
            State derivatives.

        """
        # Unpack the states
        psi_s, w_M, theta_M = x
        theta_m = self.motor.n_p*theta_M

        # Interconnections: outputs for computing the state derivatives
        u_ss = self.conv.ac_voltage(self.conv.q, self.conv.u_dc0)
        u_s = np.exp(-1j*theta_m)*u_ss  # Stator voltage in rotor coordinates

        # State derivatives
        motor_f, _, tau_M = self.motor.f(psi_s, u_s, w_M)
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
        self.data.psi_s.extend(sol.y[0])
        self.data.w_M.extend(sol.y[1].real)
        self.data.theta_M.extend(sol.y[2].real)

    def post_process(self):
        """Transform the lists to the ndarray format and post-process them."""
        # From lists to the ndarray
        for key in self.data:
            self.data[key] = np.asarray(self.data[key])

        # Compute some useful quantities
        self.data.i_s, self.data.tau_M = self.motor.magnetic(self.data.psi_s)
        self.data.w_m = self.motor.n_p*self.data.w_M
        self.data.tau_L = (
            self.mech.tau_L_t(self.data.t) + self.mech.tau_L_w(self.data.w_M))
        self.data.u_ss = self.conv.ac_voltage(self.data.q, self.conv.u_dc0)
        self.data.theta_M = np.mod(  # Limit into [-pi, pi)
            self.data.theta_M + np.pi, 2*np.pi) - np.pi
        self.data.theta_m = np.mod(  # Limit into [-pi, pi)
            self.motor.n_p*self.data.theta_M + np.pi, 2*np.pi) - np.pi
        self.data.i_ss = self.data.i_s*np.exp(1j*self.data.theta_m)


# %%
class SynchronousMotorDriveTwoMass(SynchronousMotorDrive):
    """
    Synchronous motor drive with two-mass mechanics.

    This interconnects the subsystems of a synchronous motor drive and provides
    an interface to the solver.

    Parameters
    ----------
    motor : SynchronousMotor
        Synchronous motor model.
    mech : MechanicsTwoMass
        Mechanics model.
    conv : Inverter
        Inverter model.

    """

    def __init__(self, motor, mech, conv):
        super().__init__(motor, mech, conv)
        self.data.w_L, self.data.theta_ML = [], []

    def get_initial_values(self):
        """Extend the base class."""
        x0 = super().get_initial_values() + [
            self.mech.w_L0, self.mech.theta_ML0
        ]
        return x0

    def set_initial_values(self, t0, x0):
        """Extend the base class."""
        super().set_initial_values(t0, x0[0:3])
        self.mech.w_L0 = x0[3].real
        self.mech.theta_ML0 = np.mod(x0[4].real + np.pi, 2*np.pi) - np.pi

    def f(self, t, x):
        """Override the base class."""
        # Unpack the states
        psi_s, w_M, theta_M, w_L, theta_ML = x
        theta_m = self.motor.n_p*theta_M
        # Interconnections: outputs for computing the state derivatives
        u_ss = self.conv.ac_voltage(self.conv.q, self.conv.u_dc0)
        u_s = np.exp(-1j*theta_m)*u_ss  # Stator voltage in rotor coordinates
        # State derivatives plus the outputs for interconnections
        motor_f, _, tau_M = self.motor.f(psi_s, u_s, w_M)
        mech_f = self.mech.f(t, w_M, w_L, theta_ML, tau_M)
        # List of state derivatives
        return motor_f + mech_f

    def save(self, sol):
        """Extend the base class."""
        super().save(sol)
        self.data.w_L.extend(sol.y[3].real)
        self.data.theta_ML.extend(sol.y[4].real)

    def post_process(self):
        """Extend the base class."""
        super().post_process()
        # From lists to the ndarray
        self.data.w_L = np.asarray(self.data.w_L)
        self.data.theta_ML = np.asarray(self.data.theta_ML)
