# pylint: disable=C0103
"""
This module includes a continuous-time model for an induction motor drive. The
space vector models are implemented in stator coordinates.

"""
import numpy as np
from sklearn.utils import Bunch
from helpers import abc2complex, complex2abc


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
        x0 : complex list, length 4
            Initial values of the state variables.

        """
        x0 = [self.motor.psi_ss0, self.motor.psi_Rs0,
              self.mech.w_M0, self.mech.theta_M0]
        return x0

    def set_initial_values(self, t0, x0):
        """
        Parameters
        ----------
        x0 : complex ndarray
            Initial values of the state variables.

        """
        self.t0 = t0
        self.motor.psi_ss0 = x0[0]
        self.motor.psi_Rs0 = x0[1]
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
        psi_ss, psi_Rs, w_M, _ = x
        # Interconnections: outputs for computing the state derivatives
        u_ss = self.converter.ac_voltage(self.q, self.converter.u_dc0)
        i_ss, i_Rs = self.motor.currents(psi_ss, psi_Rs)
        tau_M = self.motor.torque(psi_ss, i_ss)
        # State derivatives
        motor_f = self.motor.f(psi_Rs, i_ss, i_Rs, u_ss, w_M)
        mech_f = self.mech.f(t, w_M, tau_M)
        # List of state derivatives
        return motor_f + mech_f

    def __str__(self):
        desc = ('\nSystem: Induction motor drive\n'
                '-----------------------------\n')
        desc += (self.delay.__str__() + self.pwm.__str__()
                 + self.converter.__str__() + self.motor.__str__()
                 + self.mech.__str__())
        return desc


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
        self.psi_ss0, self.psi_Rs0 = 0j, 0j

    def currents(self, psi_ss, psi_Rs):
        """
        Computes the stator and rotor currents.

        Parameters
        ----------
        psi_ss : complex
            Stator flux linkage in stator coordinates.
        psi_Rs : complex
            Rotor flux linkage in stator coordinates.

        Returns
        -------
        i_ss : complex
            Stator current in stator coordinates.
        i_Rs : complex
            Rotor current in stator coordinates.

        """
        i_ss = (psi_ss - psi_Rs)/self.L_sgm
        i_Rs = psi_Rs/self.L_M - i_ss
        return i_ss, i_Rs

    def torque(self, psi_ss, i_ss):
        """
        Computes the electromagnetic torque.

        Parameters
        ----------
        psi_ss : complex
            Stator flux linkage.
        i_ss : complex
            Stator current.

        Returns
        -------
        tau_M : float
            Electromagnetic torque.

        """
        tau_M = 1.5*self.p*np.imag(i_ss*np.conj(psi_ss))
        return tau_M

    def f(self, psi_Rs, i_ss, i_Rs, u_ss, w_M):
        # pylint: disable=R0913
        # To avoid overlapping computations, i_s and i_R are the inputs.
        """
        Computes the state derivatives. All space vectors are in stator
        coordinates.

        Parameters
        ----------
        psi_ss : complex
            Stator flux linkage.
        psi_Rs : complex
            Rotor flux linkage.
        i_ss : complex
            Stator current.
        i_Rs : complex
            Rotor current.
        u_ss : complex
            Stator voltage.
        w_M : float
            Rotor speed (in mechanical rad/s).

        Returns
        -------
        complex list, length 2
            Time derivative of the state vector, [dpsi_ss, dpsi_Rs]

        """
        dpsi_ss = u_ss - self.R_s*i_ss
        dpsi_Rs = -self.R_R*i_Rs + 1j*self.p*w_M*psi_Rs
        return [dpsi_ss, dpsi_Rs]

    def meas_currents(self):
        """
        Returns the phase currents at the end of the sampling period.

        Returns
        -------
        i_s_abc : 3-tuple of floats
            Phase currents.

        """
        # Stator current space vector in stator coordinates
        i_ss, _ = self.currents(self.psi_ss0, self.psi_Rs0)
        # Phase currents
        i_s_abc = complex2abc(i_ss)  # + noise + offset ...
        return i_s_abc

    def __str__(self):
        desc = (('Induction motor (inverse-Gamma model):\n'
                 '    p={}  R_s={}  R_R={}  L_sgm={}  L_M={}\n')
                .format(self.p, self.R_s, self.R_R, self.L_sgm, self.L_M))
        return desc


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
        desc = (('L_sat(psi)=L_unsat/(1+(beta*abs(psi))**S):'
                 '  L_unsat={}  beta={}  S={}')
                .format(self.L_unsat, self.beta, self.S))
        return desc


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

    def currents(self, psi_ss, psi_Rs):
        """
        This method overrides the base class method.

        """
        L_M = self.L_M(psi_ss)
        i_Rs = (psi_Rs - psi_ss)/self.L_sgm
        i_ss = psi_ss/L_M - i_Rs
        return i_ss, i_Rs

    def __str__(self):
        desc = (('Saturated induction motor (Gamma model):\n'
                 '    p={}  R_s={}  R_R={}  L_sgm={}\n'
                 '    L_M={}\n')
                .format(self.p, self.R_s, self.R_R, self.L_sgm, self.L_M))
        return desc


# %%
class Datalogger:
    """
    This class contains a datalogger.

    """

    def __init__(self):
        """
        Initialize the attributes.

        """
        self.data = Bunch()
        self.data.t, self.data.q = [], []
        self.data.psi_ss, self.data.psi_Rs = [], []
        self.data.theta_M, self.data.w_M = [], []

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
        self.data.psi_ss.extend(sol.y[0])
        self.data.psi_Rs.extend(sol.y[1])
        self.data.w_M.extend(sol.y[2].real)
        self.data.theta_M.extend(sol.y[3].real)

    def post_process(self, mdl):
        """
        Transform the lists to the ndarray format and post-process them.

        """
        # From lists to the ndarray
        for key in self.data:
            self.data[key] = np.asarray(self.data[key])

        # Some useful variables
        self.data.i_ss, _ = mdl.motor.currents(self.data.psi_ss,
                                               self.data.psi_Rs)
        self.data.theta_m = mdl.motor.p*self.data.theta_M
        self.data.theta_m = np.mod(self.data.theta_m, 2*np.pi)
        self.data.w_m = mdl.motor.p*self.data.w_M
        self.data.tau_M = mdl.motor.torque(self.data.psi_ss, self.data.i_ss)
        self.data.tau_L = (mdl.mech.tau_L_ext(self.data.t)
                           + mdl.mech.B*self.data.w_M)
        self.data.u_ss = mdl.converter.ac_voltage(self.data.q,
                                                  mdl.converter.u_dc0)


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
        psi_ss, psi_Rs, _, w_M, u_dc, i_L = x
        # Interconnections: outputs for computing the state derivatives
        i_ss, i_Rs = self.motor.currents(psi_ss, psi_Rs)
        u_ss = self.converter.ac_voltage(self.q, u_dc)
        i_dc = self.converter.dc_current(self.q, i_ss)
        tau_M = self.motor.torque(psi_ss, i_ss)
        # Returns the list of state derivatives
        return (self.motor.f(psi_Rs, i_ss, i_Rs, u_ss, w_M) +
                self.mech.f(t, w_M, tau_M) +
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
        self.data.u_dc, self.data.i_L = [], []
        # self.i_dc, self.u_di, self.u_g, self.i_g = 0, 0, 0, 0

    def save(self, sol):
        """
        Extends the base class.

        """
        super().save(sol)
        self.data.u_dc.extend(sol.y[4].real)
        self.data.i_L.extend(sol.y[5].real)

    def post_process(self, mdl):
        """
        Extends the base class.

        """
        super().post_process(mdl)
        # From lists to the ndarray
        self.data.u_dc = np.asarray(self.data.u_dc)
        self.data.i_L = np.asarray(self.data.i_L)
        # Some useful variables
        self.data.u_ss = mdl.converter.ac_voltage(self.data.q, self.data.u_dc)
        self.data.i_dc = mdl.converter.dc_current(self.data.q, self.data.i_ss)
        u_g_abc = mdl.converter.grid_voltages(self.data.t)
        self.data.u_g = abc2complex(u_g_abc)
        # Voltage at the output of the diode bridge
        self.data.u_di = np.amax(u_g_abc, 0) - np.amin(u_g_abc, 0)
        # Diode briddge switching states (-1, 0, 1)
        q_g_abc = ((np.amax(u_g_abc, 0) == u_g_abc).astype(int) -
                   (np.amin(u_g_abc, 0) == u_g_abc).astype(int))
        # Grid current space vector
        self.data.i_g = abc2complex(q_g_abc)*self.data.i_L
