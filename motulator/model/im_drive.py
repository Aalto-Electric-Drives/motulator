"""
Continuous-time models for induction motor drives.

Peak-valued complex space vectors are used. The space vector models are
implemented in stator coordinates. The default values correspond to a 2.2-kW
induction motor.

"""
import numpy as np

from motulator.helpers import abc2complex, Bunch


# %%
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
    conv : Inverter
        Inverter model.

    """

    def __init__(self, motor=None, mech=None, conv=None):
        self.motor = motor
        self.mech = mech
        self.conv = conv
        self.t0 = 0  # Initial time
        # Store the solution in these lists
        self.data = Bunch()  # Stores the solution data
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
        x0 = [
            self.motor.psi_ss0,
            self.motor.psi_rs0,
            self.mech.w_M0,
            self.mech.theta_M0,
        ]
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
        # Limit theta_M0 = x0[3].real into [-pi, pi)
        self.mech.theta_M0 = np.mod(x0[3].real + np.pi, 2*np.pi) - np.pi

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
        # State derivatives plus the outputs for interconnections
        motor_f, _, tau_M = self.motor.f(psi_ss, psi_rs, u_ss, w_M)
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
        """Transform the lists to the ndarray format and post-process them."""
        # From lists to the ndarray
        for key in self.data:
            self.data[key] = np.asarray(self.data[key])

        # Some useful variables
        self.data.i_ss, _, self.data.tau_M = self.motor.magnetic(
            self.data.psi_ss, self.data.psi_rs)
        self.data.theta_M = np.mod(  # Limit into [-pi, pi)
            self.data.theta_M + np.pi, 2*np.pi) - np.pi
        self.data.theta_m = np.mod(  # Limit into [-pi, pi)
            self.motor.p*self.data.theta_M + np.pi, 2*np.pi) - np.pi
        self.data.w_m = self.motor.p*self.data.w_M
        self.data.tau_L = (
            self.mech.tau_L_t(self.data.t) + self.mech.tau_L_w(self.data.w_M))
        self.data.u_ss = self.conv.ac_voltage(self.data.q, self.conv.u_dc0)

        # Compute the inverse-Γ rotor flux
        try:
            # Saturable stator inductance
            L_s = self.motor.L_s(np.abs(self.data.psi_ss))
        except TypeError:
            # Constant stator inductance
            L_s = self.motor.L_s
        gamma = L_s/(L_s + self.motor.L_ell)  # Magnetic coupling factor
        self.data.psi_Rs = gamma*self.data.psi_rs


# %%
class InductionMotorDriveDiode(InductionMotorDrive):
    """
    Induction motor drive equipped with a diode bridge.

    This model extends the InductionMotorDrive class with a model for a
    three-phase diode bridge fed from stiff supply voltages. The DC bus is
    modeled as an inductor and a capacitor.

    Parameters
    ----------
    motor : InductionMotor | InductionMotorSaturated
        Induction motor model.
    mech : Mechanics
        Mechanics model.
    conv : FrequencyConverter
        Frequency converter model.

    """

    def __init__(self, motor=None, mech=None, conv=None):
        super().__init__(motor=motor, mech=mech)
        self.conv = conv
        self.data.u_dc, self.data.i_L = [], []

    def get_initial_values(self):
        """Extend the base class."""
        x0 = super().get_initial_values() + [self.conv.u_dc0, self.conv.i_L0]
        return x0

    def set_initial_values(self, t0, x0):
        """Extend the base class."""
        super().set_initial_values(t0, x0[0:4])
        self.conv.u_dc0 = x0[4].real
        self.conv.i_L0 = x0[5].real

    def f(self, t, x):
        """Override the base class."""
        # Unpack the states for better readability
        psi_ss, psi_rs, w_M, _, u_dc, i_L = x

        # Interconnections: outputs for computing the state derivatives
        u_ss = self.conv.ac_voltage(self.conv.q, u_dc)
        motor_f, i_ss, tau_M = self.motor.f(psi_ss, psi_rs, u_ss, w_M)
        i_dc = self.conv.dc_current(self.conv.q, i_ss)

        # Return the list of state derivatives
        return (
            motor_f + self.mech.f(t, w_M, tau_M) +
            self.conv.f(t, u_dc, i_L, i_dc))

    def save(self, sol):
        """Extend the base class."""
        super().save(sol)
        self.data.u_dc.extend(sol.y[4].real)
        self.data.i_L.extend(sol.y[5].real)

    def post_process(self):
        """Extend the base class."""
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


# %%
class InductionMotorDriveTwoMass(InductionMotorDrive):
    """
    Induction motor drive with two-mass mechanics.

    This interconnects the subsystems of an induction motor drive and provides
    an interface to the solver.

    Parameters
    ----------
    motor : InductionMotor | InductionMotorSaturated
        Induction motor model.
    mech : MechanicsTwoMass
        Mechanics model.
    conv : Inverter
        Inverter model.

    """

    def __init__(self, motor=None, mech=None, conv=None):
        super().__init__(motor=motor, mech=mech, conv=conv)
        self.data.w_L, self.data.theta_ML = [], []

    def get_initial_values(self):
        """Extend the base class."""
        x0 = super().get_initial_values() + [
            self.mech.w_L0, self.mech.theta_ML0
        ]
        return x0

    def set_initial_values(self, t0, x0):
        """Extend the base class."""
        super().set_initial_values(t0, x0[0:4])
        self.mech.w_L0 = x0[4].real
        self.mech.theta_ML0 = np.mod(x0[5].real + np.pi, 2*np.pi) - np.pi

    def f(self, t, x):
        """Override the base class."""
        # Unpack the states
        psi_ss, psi_rs, w_M, _, w_L, theta_ML = x
        # Interconnections: outputs for computing the state derivatives
        u_ss = self.conv.ac_voltage(self.conv.q, self.conv.u_dc0)
        # State derivatives plus the outputs for interconnections
        motor_f, _, tau_M = self.motor.f(psi_ss, psi_rs, u_ss, w_M)
        mech_f = self.mech.f(t, w_M, w_L, theta_ML, tau_M)
        # List of state derivatives
        return motor_f + mech_f

    def save(self, sol):
        """Extend the base class."""
        super().save(sol)
        self.data.w_L.extend(sol.y[4].real)
        self.data.theta_ML.extend(sol.y[5].real)

    def post_process(self):
        """Extend the base class."""
        super().post_process()
        # From lists to the ndarray
        self.data.w_L = np.asarray(self.data.w_L)
        self.data.theta_ML = np.asarray(self.data.theta_ML)
