"""
Continuous-time models for synchronous motor drives.

Peak-valued complex space vectors are used.

"""
import numpy as np
from motulator._helpers import complex2abc
from motulator._utils import Bunch


# %%
class SynchronousMachine:
    """
    Synchronous machine model.

    This models a synchronous machine in rotor coordinates. The stator flux 
    linkage and the electrical angle of the rotor are the state variables. 

    Parameters
    ----------
    n_p : int
        Number of pole pairs.
    R_s : float
        Stator resistance (Ω).
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
        # Initial values
        self.psi_s0 = complex(psi_f)
        self.theta_m0 = 0

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
        complex list, length 2
            Time derivative of the state vector, [dpsi_s, dtheta_m0]
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
        dtheta_m = self.n_p*w_M
        return [dpsi_s, dtheta_m], i_s, tau_M

    def meas_currents(self):
        """
        Measure the phase currents at the end of the sampling period.

        Returns
        -------
        i_s_abc : 3-tuple of floats
            Phase currents (A).

        """
        i_s0 = self.current(self.psi_s0)
        i_s_abc = complex2abc(np.exp(1j*self.theta_m0)*i_s0)
        return i_s_abc


# %%
class SynchronousMachineSaturated(SynchronousMachine):
    """
    Model of a saturated synchronous machine.

    This overrides the linear magnetics model of the SynchronousMachine class
    with a generic saturation model::

        i_s = i_s(psi_s)
        
    The saturation model could be an analytical function or a look-up table. 

    Parameters
    ----------
    n_p : int
        Number of pole pairs.
    R_s : float
        Stator resistance (Ω).
    current : callable
        Function that computes the stator current `i_s` as a function of the 
        stator flux linkage `psi_s`. 
    psi_s0 : complex, optional
        Initial value of the stator flux linkage (Vs). For PM machines, this 
        should be solved from the the saturation model. The default is 0j. 

    """

    def __init__(self, n_p, R_s, current, psi_s0=0j):
        # pylint: disable=super-init-not-called
        self.n_p, self.R_s = n_p, R_s
        self.current = current
        # Initial values
        self.psi_s0 = complex(psi_s0)
        self.theta_m0 = 0


# %%
class Drive:
    """
    Continuous-time model for a synchronous machine drive.

    This interconnects the subsystems of a synchronous machine drive and 
    provides an interface to the solver. More complicated systems could be 
    modeled using a similar template.

    Parameters
    ----------
    machine : SynchronousMachine
        Synchronous machine model.
    mechanics : Mechanics
        Mechanics model.
    converter : Inverter
        Inverter model.

    """

    def __init__(self, machine=None, mechanics=None, converter=None):
        self.machine = machine
        self.mechanics = mechanics
        self.converter = converter
        self.t0 = 0
        self.clear()

    def clear(self):
        """
        Clear the simulation data of the system model.
        
        This method is automatically run when the instance for the system model
        is created. It can also be used in the case of repeated simulations to 
        clear the data from the previous simulation run.
        
        """
        self.t0 = 0
        # Solution will be stored in the following lists
        self.data = Bunch()
        self.data.t, self.data.q = [], []
        self.data.psi_s, self.data.theta_m = [], []
        self.data.w_M, self.data.theta_M = [], []

    def get_initial_values(self):
        """
        Get the initial values.

        Returns
        -------
        x0 : complex list, length 4
            Initial values of the state variables.

        """
        x0 = [
            self.machine.psi_s0,
            self.machine.theta_m0,
            self.mechanics.w_M0,
            self.mechanics.theta_M0,
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
        self.machine.psi_s0 = x0[0]
        # x0[1:3].imag are always zero
        self.machine.theta_m0 = x0[1].real
        self.mechanics.w_M0 = x0[2].real
        # Limit the angles into [-pi, pi)
        self.mechanics.theta_m0 = np.mod(x0[1].real + np.pi, 2*np.pi) - np.pi
        self.mechanics.theta_M0 = np.mod(x0[3].real + np.pi, 2*np.pi) - np.pi

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
        psi_s, theta_m, w_M, _ = x

        # Interconnections: outputs for computing the state derivatives
        u_ss = self.converter.ac_voltage(
            self.converter.q, self.converter.u_dc0)
        u_s = np.exp(-1j*theta_m)*u_ss  # Stator voltage in rotor coordinates

        # State derivatives
        machine_f, _, tau_M = self.machine.f(psi_s, u_s, w_M)
        mechanics_f = self.mechanics.f(t, w_M, tau_M)

        # List of state derivatives
        return machine_f + mechanics_f

    def save(self, sol):
        """
        Save the solution.

        Parameters
        ----------
        sol : Bunch
            Solution from the solver.

        """
        self.data.t.extend(sol.t)
        self.data.q.extend(sol.q)
        self.data.psi_s.extend(sol.y[0])
        self.data.theta_m.extend(sol.y[1].real)
        self.data.w_M.extend(sol.y[2].real)
        self.data.theta_M.extend(sol.y[3].real)

    def post_process(self):
        """Transform the lists to the ndarray format and post-process them."""
        # From lists to the ndarray
        for key in self.data:
            self.data[key] = np.asarray(self.data[key])

        # Compute some useful quantities
        self.data.i_s, self.data.tau_M = self.machine.magnetic(self.data.psi_s)
        self.data.w_m = self.machine.n_p*self.data.w_M
        self.data.tau_L = (
            self.mechanics.tau_L_t(self.data.t) +
            self.mechanics.tau_L_w(self.data.w_M))
        self.data.u_ss = self.converter.ac_voltage(
            self.data.q, self.converter.u_dc0)
        self.data.theta_m = np.mod(  # Limit into [-pi, pi)
            self.data.theta_m + np.pi, 2*np.pi) - np.pi
        self.data.theta_M = np.mod(  # Limit into [-pi, pi)
            self.data.theta_M + np.pi, 2*np.pi) - np.pi
        self.data.i_ss = self.data.i_s*np.exp(1j*self.data.theta_m)


# %%
class DriveTwoMassMechanics(Drive):
    """
    Synchronous machine drive with two-mass mechanics.

    This interconnects the subsystems of a synchronous machine drive and
    provides an interface to the solver.

    Parameters
    ----------
    machine : SynchronousMachine
        Synchronous machine model.
    mechanics : MechanicsTwoMass
        Mechanics model.
    converter : Inverter
        Inverter model.

    """

    def __init__(self, machine=None, mechanics=None, converter=None):
        super().__init__(machine, mechanics, converter)
        self.clear()

    def clear(self):
        """Extend the base class."""
        super().clear()
        self.data.w_L, self.data.theta_ML = [], []

    def get_initial_values(self):
        """Extend the base class."""
        x0 = super().get_initial_values() + [
            self.mechanics.w_L0, self.mechanics.theta_ML0
        ]
        return x0

    def set_initial_values(self, t0, x0):
        """Extend the base class."""
        super().set_initial_values(t0, x0[0:4])
        self.mechanics.w_L0 = x0[4].real
        self.mechanics.theta_ML0 = np.mod(x0[5].real + np.pi, 2*np.pi) - np.pi

    def f(self, t, x):
        """Override the base class."""
        # Unpack the states
        psi_s, theta_m, w_M, _, w_L, theta_ML = x
        # Interconnections: outputs for computing the state derivatives
        u_ss = self.converter.ac_voltage(
            self.converter.q, self.converter.u_dc0)
        u_s = np.exp(-1j*theta_m)*u_ss  # Stator voltage in rotor coordinates
        # State derivatives plus the outputs for interconnections
        machine_f, _, tau_M = self.machine.f(psi_s, u_s, w_M)
        mechanics_f = self.mechanics.f(t, w_M, w_L, theta_ML, tau_M)
        # List of state derivatives
        return machine_f + mechanics_f

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
