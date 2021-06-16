# pylint: disable=C0103
"""
This module includes a continuous-time model for an induction motor drive. The
space vector models are implemented in stator coordinates, but this is not
shown in the variable naming for simplicity.

"""
import numpy as np
from helpers import abc2complex, complex2abc


# %%
class Motor:
    """
    This class represents an induction motor. The inverse-Gamma model and
    peak-valued complex space vectors are used.

    """

    def __init__(self, R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224, p=2):
        # pylint: disable=R0913
        """
        The default values correspond to the 2.2-kW induction motor.

        Parameters
        ----------
        R_s : float, optional
            Stator resistance. The default is 3.7.
        R_R : float, optional
            Rotor resistance. The default is 2.1.
        L_sgm : float, optional
            Leakage inductance. The default is .021.
        L_M : float, optional
            Magnetizing inductance. The default is .224.
        p : int, optional
            Number of pole pairs. The default is 2.

        """
        self.R_s, self.R_R, self.L_sgm, self.L_M = R_s, R_R, L_sgm, L_M
        self.p = p
        self.psi_s0, self.psi_R0 = 0j, 0j

    def currents(self, psi_s, psi_R):
        """
        Computes the stator and rotor currents.

        Parameters
        ----------
        psi_s : complex
            Stator flux linkage.
        psi_R : complex
            Rotor flux linkage.

        Returns
        -------
        i_s : complex
            Stator current.
        i_R : complex
            Rotor current.

        """
        i_s = (psi_s - psi_R)/self.L_sgm
        i_R = psi_R/self.L_M - i_s
        return i_s, i_R

    def torque(self, psi_s, i_s):
        """
        Computes the electromagnetic torque.

        Parameters
        ----------
        psi_s : complex
            Stator flux linkage.
        i_s : complex
            Stator current.

        Returns
        -------
        T_M : float
            Electromagnetic torque.

        """
        T_M = 1.5*self.p*np.imag(i_s*np.conj(psi_s))
        return T_M

    def f(self, psi_R, i_s, i_R, u_s, w_M):
        # pylint: disable=R0913
        # To avoid overlapping computations, i_s and i_R are the inputs.
        """
        Computes the state derivatives.

        Parameters
        ----------
        psi_s : complex
            Stator flux linkage.
        psi_R : complex
            Rotor flux linkage.
        i_s : complex
            Stator current.
        i_R : complex
            Rotor current.
        u_s : complex
            Stator voltage.
        w_M : float
            Rotor speed (in mechanical rad/s).

        Returns
        -------
        complex list, length 2
            Time derivative of the state vector, [dpsi_s, dpsi_R]

        """
        dpsi_s = u_s - self.R_s*i_s
        dpsi_R = -self.R_R*i_R + 1j*self.p*w_M*psi_R
        return [dpsi_s, dpsi_R]

    def meas_currents(self):
        """
        Returns the phase currents at the end of the sampling period.

        Returns
        -------
        i_s_abc : 3-tuple of floats
            Phase currents.

        """
        i_s, _ = self.currents(self.psi_s0, self.psi_R0)
        i_s_abc = complex2abc(i_s)  # + noise + offset ...
        return i_s_abc

    def __str__(self):
        desc = ('Induction motor (inverse-Gamma model):\n'
                '    p={}  R_s={}  R_R={}  L_sgm={}  L_M={}')
        return desc.format(self.p, self.R_s, self.R_R, self.L_sgm, self.L_M)


# %%
class SaturationModel:
    """
    This data class contains a saturation model based on a simple power
    function.

    """

    def __init__(self, L_unsat=.34, beta=.84, S=7):
        self.L_unsat, self.beta, self.S = L_unsat, beta, S

    def __call__(self, psi):
        """
        The default values correspond to the stator inductance of the 2.2-kW
        induction motor.

        Parameters
        ----------
        psi : complex
            Flux linkage. If the value is complex, its magnitude is used.

        Returns
        -------
        L_sat : float
            Instantatenous saturated value of the inductance.

        """
        L_sat = self.L_unsat/(1 + (self.beta*np.abs(psi))**self.S)
        return L_sat

    def __str__(self):
        desc = ('L_sat(psi)=L_unsat/(1+(beta*abs(psi))**S):'
                '  L_unsat={}  beta={}  S={}')
        return desc.format(self.L_unsat, self.beta, self.S)


# %%
class MotorSaturated(Motor):
    """
    This subclass represents the Gamma model of a saturated induction motor.
    The Gamma model suits better for modeling the magnetic saturation.

    """

    def __init__(self,
                 R_s=3.7, R_R=2.5, L_sgm=.023, L_M=SaturationModel(), p=2):
        # pylint: disable=R0913
        """
        The default values correspond to the 2.2-kW induction motor.

        Parameters
        ----------
        R_s : float, optional
            Stator resistance. The default is 3.7.
        R_R : float, optional
            Rotor resistance. The default is 2.1.
        L_sgm : float, optional
            Leakage inductance. The default is .021.
        L_M : function, optional
            Stator inductance function L_M(psi_s). The default is
            SaturationModel().
        p : int, optional
            Number of pole pairs. The default is 2.

        """
        super().__init__(R_s=R_s, R_R=R_R, L_sgm=L_sgm, L_M=L_M, p=p)

    def currents(self, psi_s, psi_R):
        """
        This method overrides the base class method.

        """
        L_M = self.L_M(psi_s)
        i_R = (psi_R - psi_s)/self.L_sgm
        i_s = psi_s/L_M - i_R
        return i_s, i_R

    def __str__(self):
        desc = ('Saturated induction motor (Gamma model):\n'
                '    p={}  R_s={}  R_R={}  L_sgm={}\n'
                '    L_M={}')
        return desc.format(self.p, self.R_s, self.R_R, self.L_sgm, self.L_M)


# %%
class Drive:
    """
    This class interconnects the subsystems of an induction motor drive
    and provides the interface to the solver. More complicated systems
    could be simulated using a similar template.

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
        x0 : complex list, length 3
            Initial values of the state variables.

        """
        x0 = [self.motor.psi_s0, self.motor.psi_R0,
              self.mech.theta_M0, self.mech.w_M0]
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
        self.motor.psi_R0 = x0[1]
        # x0[2].imag and x0[3].imag are always zero
        self.mech.theta_M0 = x0[2].real
        self.mech.w_M0 = x0[3].real
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
        psi_s, psi_R, _, w_M = x
        # Interconnections: outputs for computing the state derivatives
        u_s = self.converter.ac_voltage(self.q, self.converter.u_dc0)
        i_s, i_R = self.motor.currents(psi_s, psi_R)
        T_M = self.motor.torque(psi_s, i_s)
        # State derivatives
        motor_f = self.motor.f(psi_R, i_s, i_R, u_s, w_M)
        mech_f = self.mech.f(t, w_M, T_M)
        # List of state derivatives
        return motor_f + mech_f


# %%
class Datalogger:
    """
    This class contains a default datalogger. Here, stator coordinates are
    marked with extra s, e.g. i_ss is the stator current in stator
    coordinates.

    """

    def __init__(self):
        """
        Initialize the attributes.

        """
        # pylint: disable=too-many-instance-attributes
        self.t, self.q = [], []
        self.psi_ss, self.psi_Rs = [], []
        self.theta_M, self.w_M = [], []
        self.u_ss, self.i_ss = 0j, 0j
        self.w_m, self.theta_m = 0, 0
        self.T_M, self.T_L = 0, 0

    def save(self, mdl, sol):
        """
        Saves the solution.

        Parameters
        ----------
        mdl : instance of a class
            Continuous-time model.
        sol : bunch object
            Solution from the solver.

        """
        self.t.extend(sol.t)
        self.q.extend(len(sol.t)*[mdl.q])
        self.psi_ss.extend(sol.y[0])
        self.psi_Rs.extend(sol.y[1])
        self.theta_M.extend(sol.y[2].real)
        self.w_M.extend(sol.y[3].real)

    def post_process(self, mdl):
        """
        Transforms the lists to the ndarray format and post-process them.

        """
        # From lists to the ndarray
        self.t = np.asarray(self.t)
        self.q = np.asarray(self.q)
        self.psi_ss = np.asarray(self.psi_ss)
        self.psi_Rs = np.asarray(self.psi_Rs)
        self.theta_M = np.asarray(self.theta_M)
        self.w_M = np.asarray(self.w_M)
        # Some useful variables
        self.i_ss, _ = mdl.motor.currents(self.psi_ss, self.psi_Rs)
        self.theta_m = mdl.motor.p*self.theta_M
        self.theta_m = np.mod(self.theta_m, 2*np.pi)
        self.w_m = mdl.motor.p*self.w_M
        self.T_M = mdl.motor.torque(self.psi_ss, self.i_ss)
        self.T_L = mdl.mech.T_L_ext(self.t) + mdl.mech.B*self.w_M
        self.u_ss = mdl.converter.ac_voltage(self.q, mdl.converter.u_dc0)


# %%
class DriveWithDiodeBridge(Drive):
    """
    This subclass models an induction motor drive, equipped with a three-phase
    diode bridge.

    """

    def get_initial_values(self):
        """
        Extends the base class.

        """
        x0 = super().get_initial_values() + [self.converter.u_dc0,
                                             self.converter.i_L0]
        return x0

    def set_initial_values(self, t0, x0):
        """
        Extends the base class.

        """
        super().set_initial_values(t0, x0[0:4])
        self.converter.u_dc0 = x0[4].real
        self.converter.i_L0 = x0[5].real

    def f(self, t, x):
        """
        Overrides the base class.

        """
        # Unpack the states for better readability
        psi_s, psi_R, _, w_M, u_dc, i_L = x
        # Interconnections: outputs for computing the state derivatives
        i_s, i_R = self.motor.currents(psi_s, psi_R)
        u_s = self.converter.ac_voltage(self.q, u_dc)
        i_dc = self.converter.dc_current(self.q, i_s)
        T_M = self.motor.torque(psi_s, i_s)
        # Returns the list of state derivatives
        return (self.motor.f(psi_R, i_s, i_R, u_s, w_M) +
                self.mech.f(t, w_M, T_M) +
                self.converter.f(t, u_dc, i_L, i_dc))


# %%
class DataloggerExtended(Datalogger):
    """
    Extends the default data logger for the model with the DC-bus dynamics.

    """

    def __init__(self):
        """
        Initialize the attributes.

        """
        # pylint: disable=too-many-instance-attributes
        super().__init__()
        self.u_dc, self.i_L = [], []
        self.i_dc, self.u_di, self.u_g, self.i_g = 0, 0, 0, 0

    def save(self, mdl, sol):
        """
        Extends the base class.

        """
        super().save(mdl, sol)
        self.u_dc.extend(sol.y[4].real)
        self.i_L.extend(sol.y[5].real)

    def post_process(self, mdl):
        """
        Extends the base class.

        """
        super().post_process(mdl)
        # From lists to the ndarray
        self.u_dc = np.asarray(self.u_dc)
        self.i_L = np.asarray(self.i_L)
        # Some useful variables
        self.u_ss = mdl.converter.ac_voltage(self.q, self.u_dc)
        self.i_dc = mdl.converter.dc_current(self.q, self.i_ss)
        u_g_abc = mdl.converter.grid_voltages(self.t)
        self.u_g = abc2complex(u_g_abc)
        # Voltage at the output of the diode bridge
        self.u_di = np.amax(u_g_abc, 0) - np.amin(u_g_abc, 0)
        # Diode briddge switching states (-1, 0, 1)
        q_g_abc = ((np.amax(u_g_abc, 0) == u_g_abc).astype(int) -
                   (np.amin(u_g_abc, 0) == u_g_abc).astype(int))
        # Grid current space vector
        self.i_g = abc2complex(q_g_abc)*self.i_L
