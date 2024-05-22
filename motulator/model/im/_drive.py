"""
Continuous-time models for induction machine drives.

Peak-valued complex space vectors are used. The space vector models are
implemented in stator coordinates. 

"""
import numpy as np
from motulator._helpers import abc2complex, complex2abc, wrap
from motulator.model._simulation import Model


# %%
class InductionMachine:
    """
    Γ-equivalent model of an induction machine.

    An induction machine is modeled using the Γ-equivalent model [#Sle1989]_. 
    The model is implemented in stator coordinates. The flux linkages are used 
    as state variables.

    Parameters
    ----------
    n_p : int
        Number of pole pairs.
    R_s : float
        Stator resistance (Ω).
    R_r : float
        Rotor resistance (Ω).
    L_ell : float
        Leakage inductance (H).
    L_s : float
        Stator inductance (H).

    Notes
    -----
    The Γ model is chosen here since it can be extended with the magnetic
    saturation model in a straightforward manner. If the magnetic saturation is
    omitted, the Γ model is mathematically identical to the inverse-Γ and T
    models [#Sle1989]_.

    References
    ----------
    .. [#Sle1989] Slemon, "Modelling of induction machines for electric 
       drives," IEEE Trans. Ind. Appl., 1989, https://doi.org/10.1109/28.44251

    """

    def __init__(self, n_p, R_s, R_r, L_ell, L_s):
        # pylint: disable=too-many-arguments
        self.n_p = n_p
        self.R_s, self.R_r = R_s, R_r
        self.L_ell, self.L_s = L_ell, L_s
        # Initial values
        self.psi_ss0, self.psi_rs0 = 0j, 0j

    def currents(self, psi_ss, psi_rs):
        """
        Compute the stator and rotor currents.

        Parameters
        ----------
        psi_ss : complex
            Stator flux linkage (Vs).
        psi_rs : complex
            Rotor flux linkage (Vs).

        Returns
        -------
        i_ss : complex
            Stator current (A).
        i_rs : complex
            Rotor current (A).

        """
        i_rs = (psi_rs - psi_ss)/self.L_ell
        i_ss = psi_ss/self.L_s - i_rs

        return i_ss, i_rs

    def magnetic(self, psi_ss, psi_rs):
        """
        Magnetic model.

        Parameters
        ----------
        psi_ss : complex
            Stator flux linkage (Vs).
        psi_rs : complex
            Rotor flux linkage (Vs).

        Returns
        -------
        i_ss : complex
            Stator current (A).
        i_rs : complex
            Rotor current (A).
        tau_M : float
            Electromagnetic torque (Nm).

        """
        i_ss, i_rs = self.currents(psi_ss, psi_rs)
        tau_M = 1.5*self.n_p*np.imag(i_ss*np.conj(psi_ss))

        return i_ss, i_rs, tau_M

    def f(self, psi_ss, psi_rs, u_ss, w_M):
        """
        Compute the state derivatives.

        Parameters
        ----------
        psi_ss : complex
            Stator flux linkage (Vs).
        psi_rs : complex
            Rotor flux linkage (Vs).
        u_ss : complex
            Stator voltage (V).
        w_M : float
            Rotor angular speed (mechanical rad/s).

        Returns
        -------
        complex list, length 2
            Time derivative of the state vector, [d_psi_ss, d_psi_rs]
        i_ss : complex
            Stator current (A).
        tau_M : float
            Electromagnetic torque (Nm).

        Notes
        -----
        In addition to the state derivatives, this method also returns the
        output signals (stator current `i_ss` and torque `tau_M`) needed for
        interconnection with other subsystems. This avoids overlapping
        computation in simulation.

        """
        i_ss, i_rs, tau_M = self.magnetic(psi_ss, psi_rs)
        d_psi_ss = u_ss - self.R_s*i_ss
        d_psi_rs = -self.R_r*i_rs + 1j*self.n_p*w_M*psi_rs

        return [d_psi_ss, d_psi_rs], i_ss, tau_M

    def meas_currents(self):
        """
        Measure the phase currents at the end of the sampling period.

        Returns
        -------
        i_s_abc : 3-tuple of floats
            Phase currents (A).

        """
        # Stator current space vector in stator coordinates
        i_ss, _ = self.currents(self.psi_ss0, self.psi_rs0)
        # Phase currents
        i_s_abc = complex2abc(i_ss)  # + noise + offset ...
        return i_s_abc


# %%
class InductionMachineSaturated(InductionMachine):
    """
    Γ-equivalent model of an induction machine model with main-flux saturation.

    This extends the InductionMachine class with a main-flux magnetic 
    saturation model::

        L_s = L_s(abs(psi_ss))

    Parameters
    ----------
    n_p : int
        Number of pole pairs.
    R_s : float
        Stator resistance (Ω).
    R_r : float
        Rotor resistance (Ω).
    L_ell : float
        Leakage inductance (H).
    L_s : callable
        Stator inductance (H) as a function of the stator-flux magnitude.

    """

    def currents(self, psi_ss, psi_rs):
        """Override the base class method."""
        # Saturated value of the stator inductance.
        L_s = self.L_s(np.abs(psi_ss))
        # Currents
        i_rs = (psi_rs - psi_ss)/self.L_ell
        i_ss = psi_ss/L_s - i_rs
        return i_ss, i_rs


# %%
class InductionMachineInvGamma(InductionMachine):
    """
    Inverse-Γ model of an induction machine.

    This extends the InductionMachine class (based on the Γ model) by providing
    an interface for the inverse-Γ model parameters. Linear magnetics are 
    assumed. If magnetic saturation is to be modeled, the Γ model is preferred.

    Parameters
    ----------
    n_p : int
        Number of pole pairs.
    R_s : float
        Stator resistance (Ω).
    R_R : float
        Rotor resistance (Ω).
    L_sgm : float
        Leakage inductance (H).
    L_M : float
        Magnetizing inductance (H).

    """

    def __init__(self, n_p, R_s, R_R, L_sgm, L_M):
        # pylint: disable=too-many-arguments
        # Convert the inverse-Γ parameters to the Γ parameters
        gamma = L_M/(L_M + L_sgm)  # Magnetic coupling factor
        super().__init__(n_p, R_s, R_R/gamma**2, L_sgm/gamma, L_M + L_sgm)
        # Initial values
        self.psi_ss0 = 0j
        self.psi_rs0 = 0j  # self.psi_rs0 = self.psi_Rs0/gamma


# %%
class Drive(Model):
    """
    Continuous-time model for an induction machine drive.

    This interconnects the subsystems of an induction machine drive and 
    provides an interface to the solver. 

    Parameters
    ----------
    machine : InductionMachine | InductionMachineSaturated
        Induction machine model.
    mechanics : Mechanics
        Mechanics model.
    converter : Inverter
        Inverter model.

    """

    def __init__(self, machine=None, mechanics=None, converter=None):
        super().__init__()
        self.machine = machine
        self.mechanics = mechanics
        self.converter = converter
        self._data.psi_ss, self._data.psi_rs = [], []
        self._data.theta_M, self._data.w_M = [], []

    def get_initial_values(self):
        """
        Get the initial values.

        Returns
        -------
        x0 : complex list, length 4
            Initial values of the state variables.

        """
        x0 = [
            self.machine.psi_ss0,
            self.machine.psi_rs0,
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
        self.machine.psi_ss0 = x0[0]
        self.machine.psi_rs0 = x0[1]
        # x0[2].imag and x0[3].imag are always zero
        self.mechanics.w_M0 = x0[2].real
        self.mechanics.theta_M0 = wrap(x0[3].real)

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
        psi_ss, psi_rs, w_M, _ = x
        # Interconnections: outputs for computing the state derivatives
        u_cs = self.converter.ac_voltage(self.q, self.converter.u_dc0)
        # State derivatives plus the outputs for interconnections
        machine_f, _, tau_M = self.machine.f(psi_ss, psi_rs, u_cs, w_M)
        mechanics_f = self.mechanics.f(t, w_M, tau_M)
        # List of state derivatives
        return machine_f + mechanics_f

    def save(self, sol):
        """Save the solution."""
        super().save(sol)
        self._data.psi_ss.extend(sol.y[0])
        self._data.psi_rs.extend(sol.y[1])
        self._data.w_M.extend(sol.y[2].real)
        self._data.theta_M.extend(sol.y[3].real)

    def post_process(self):
        """Transform the lists to the ndarray format and post-process them."""
        super().post_process()
        # Some useful variables
        self.data.i_ss, _, self.data.tau_M = self.machine.magnetic(
            self.data.psi_ss, self.data.psi_rs)
        self.data.theta_M = wrap(self.data.theta_M)
        self.data.theta_m = wrap(self.machine.n_p*self.data.theta_M)
        self.data.w_m = self.machine.n_p*self.data.w_M
        self.data.tau_L = (
            self.mechanics.tau_L_t(self.data.t) +
            self.mechanics.tau_L_w(self.data.w_M))
        self.data.u_cs = self.converter.ac_voltage(
            self.data.q, self.converter.u_dc0)

        # Compute the inverse-Γ rotor flux
        try:
            # Saturable stator inductance
            L_s = self.machine.L_s(np.abs(self.data.psi_ss))
        except TypeError:
            # Constant stator inductance
            L_s = self.machine.L_s
        gamma = L_s/(L_s + self.machine.L_ell)  # Magnetic coupling factor
        self.data.psi_Rs = gamma*self.data.psi_rs


# %%
class DriveWithDiodeBridge(Drive):
    """
    Induction machine drive equipped with a diode bridge.

    This model extends the Drive class with a model for a three-phase diode 
    bridge fed from stiff supply voltages. The DC bus is modeled as an inductor 
    and a capacitor.

    Parameters
    ----------
    machine : InductionMachine | InductionMachineSaturated
        Induction machine model.
    mechanics : Mechanics
        Mechanics model.
    converter : FrequencyConverter
        Frequency converter model.

    """

    def __init__(self, machine=None, mechanics=None, converter=None):
        super().__init__(machine, mechanics, converter)
        self._data.u_dc, self._data.i_L = [], []

    def get_initial_values(self):
        """Extend the base class."""
        x0 = super().get_initial_values() + [
            self.converter.u_dc0, self.converter.i_L0
        ]
        return x0

    def set_initial_values(self, t0, x0):
        """Extend the base class."""
        super().set_initial_values(t0, x0[0:4])
        self.converter.u_dc0 = x0[4].real
        self.converter.i_L0 = x0[5].real

    def f(self, t, x):
        """Override the base class."""
        # Unpack the states for better readability
        psi_ss, psi_rs, w_M, _, u_dc, i_L = x

        # Interconnections: outputs for computing the state derivatives
        u_cs = self.converter.ac_voltage(self.q, u_dc)
        machine_f, i_ss, tau_M = self.machine.f(psi_ss, psi_rs, u_cs, w_M)
        i_dc = self.converter.dc_current(self.q, i_ss)

        # Return the list of state derivatives
        return (
            machine_f + self.mechanics.f(t, w_M, tau_M) +
            self.converter.f(t, u_dc, i_L, i_dc))

    def save(self, sol):
        """Extend the base class."""
        super().save(sol)
        self._data.u_dc.extend(sol.y[4].real)
        self._data.i_L.extend(sol.y[5].real)

    def post_process(self):
        """Extend the base class."""
        super().post_process()
        # From lists to the ndarray
        self.data.u_dc = np.asarray(self.data.u_dc)
        self.data.i_L = np.asarray(self.data.i_L)
        # Some useful variables
        self.data.u_cs = self.converter.ac_voltage(self.data.q, self.data.u_dc)
        self.data.i_dc = self.converter.dc_current(self.data.q, self.data.i_ss)
        u_g_abc = self.converter.grid_voltages(self.data.t)
        self.data.u_g = abc2complex(u_g_abc)
        # Voltage at the output of the diode bridge
        self.data.u_di = np.amax(u_g_abc, axis=0) - np.amin(u_g_abc, axis=0)
        # Diode bridge switching states (-1, 0, 1)
        q_g_abc = ((np.amax(u_g_abc, axis=0) == u_g_abc).astype(int) -
                   (np.amin(u_g_abc, axis=0) == u_g_abc).astype(int))
        # Grid current space vector
        self.data.i_g = abc2complex(q_g_abc)*self.data.i_L


# %%
class DriveTwoMassMechanics(Drive):
    """
    Induction machine drive with two-mass mechanics.

    Parameters
    ----------
    machine : InductionMachine | InductionMachineSaturated
        Induction machine model.
    mechanics : MechanicsTwoMass
        Mechanics model.
    converter : Inverter
        Inverter model.

    """

    def __init__(self, machine=None, mechanics=None, converter=None):
        super().__init__(machine, mechanics, converter)
        self._data.w_L, self._data.theta_ML = [], []

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
        self.mechanics.theta_ML0 = wrap(x0[5].real)

    def f(self, t, x):
        """Override the base class."""
        # Unpack the states
        psi_ss, psi_rs, w_M, _, w_L, theta_ML = x
        # Interconnections: outputs for computing the state derivatives
        u_cs = self.converter.ac_voltage(self.q, self.converter.u_dc0)
        # State derivatives plus the outputs for interconnections
        machine_f, _, tau_M = self.machine.f(psi_ss, psi_rs, u_cs, w_M)
        mechanics_f = self.mechanics.f(t, w_M, w_L, theta_ML, tau_M)
        # List of state derivatives
        return machine_f + mechanics_f

    def save(self, sol):
        """Extend the base class."""
        super().save(sol)
        self._data.w_L.extend(sol.y[4].real)
        self._data.theta_ML.extend(sol.y[5].real)

    def post_process(self):
        """Extend the base class."""
        super().post_process()
        # From lists to the ndarray
        self.data.w_L = np.asarray(self.data.w_L)
        self.data.theta_ML = np.asarray(self.data.theta_ML)


# %%
class DriveWithLCFilter(Drive):
    """
    Induction machine drive with an output LC filter.

    Parameters
    ----------
    machine : InductionMachine | InductionMachineSaturated
        Induction machine model.
    mechanics : Mechanics
        Mechanics model.
    converter : Inverter
        Inverter model.
    lc_filter : LCFilter
        LC-filter model.

    """

    def __init__(
            self,
            machine=None,
            mechanics=None,
            converter=None,
            lc_filter=None):
        super().__init__(machine, mechanics, converter)
        self.lc_filter = lc_filter
        self._data.i_cs, self._data.u_ss = [], []

    def get_initial_values(self):
        """Extend the base class."""
        x0 = super().get_initial_values() + [
            self.lc_filter.i_cs0, self.lc_filter.u_ss0
        ]
        return x0

    def set_initial_values(self, t0, x0):
        """Extend the base class."""
        super().set_initial_values(t0, x0[0:4])
        self.lc_filter.i_cs0 = x0[4]
        self.lc_filter.u_ss0 = x0[5]

    def f(self, t, x):
        """Override the base class."""
        # Unpack the states
        psi_ss, psi_rs, w_M, _, i_cs, u_ss = x
        # Interconnections: outputs for computing the state derivatives
        u_cs = self.converter.ac_voltage(self.q, self.converter.u_dc0)
        # State derivatives plus the outputs for interconnections
        machine_f, i_ss, tau_M = self.machine.f(psi_ss, psi_rs, u_ss, w_M)
        mechanics_f = self.mechanics.f(t, w_M, tau_M)
        lc_filter_f = self.lc_filter.f(i_cs, u_ss, u_cs, i_ss)
        # List of state derivatives
        return machine_f + mechanics_f + lc_filter_f

    def save(self, sol):
        """Extend the base class."""
        super().save(sol)
        self._data.i_cs.extend(sol.y[4])
        self._data.u_ss.extend(sol.y[5])

    def post_process(self):
        """Extend the base class."""
        super().post_process()
        # From lists to the ndarray
        self.data.i_cs = np.asarray(self.data.i_cs)
        self.data.u_ss = np.asarray(self.data.u_ss)
