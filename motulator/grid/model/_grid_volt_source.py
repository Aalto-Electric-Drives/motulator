"""
3-phase AC voltage source models.

Peak-valued complex space vectors are used.

"""
from types import SimpleNamespace

import numpy as np

from motulator.common.model import Subsystem
from motulator.common.utils import complex2abc


# %%
class StiffSource(Subsystem):
    """
    3-phase voltage source model.

    This model is a 3-phase voltage source for the AC grid. A stiff grid is
    modeled, where the frequency is given by the user either as a constant
    or a time-dependent function. The grid voltage magnitude e_g_abs can also
    be a function, to simulate voltage dips and symmetrical short circuits. 
    
    In addition, nonsymmetric faults can be modeled by adding a negative
    sequence component to the grid voltage, in the form of the magnitude
    e_g_neg_abs and the phase shift phi_neg.
    
    A phase shift angle for the grid voltage vector can also be given, to
    simulate phase angle jumps. The complex form of the phase A voltage angle
    (exp_j_theta_p) is used as a state variable.

    Parameters
    ----------
    w_g : float | callable
        Grid angular frequency (rad/s). 
    e_g_abs : float | callable
        Grid voltage magnitude (positive sequence), phase-to-ground peak
        value (V).
    phi : callable, optional
        Phase shift of the grid voltage (rad). Default is 0.
    e_g_neg_abs : float | callable, optional
        Negative sequence voltage magnitude, phase-to-ground peak value (V).
        Default is None.
    phi_neg : float, optional
        Phase shift of the negative sequence component (rad). Default is 0.

    """

    def __init__(
        self,
        w_g,
        e_g_abs,
        phi=lambda t: 0,
        e_g_neg_abs=0,
        phi_neg=0,
    ):
        super().__init__()
        self.par = SimpleNamespace(
            w_g=w_g,
            e_g_abs=e_g_abs,
            phi=phi,
            e_g_neg_abs=e_g_neg_abs,
            phi_neg=phi_neg,
        )
        # states
        self.state = SimpleNamespace(exp_j_theta_p=complex(1))
        # Store the solutions in these lists
        self.sol_states = SimpleNamespace(exp_j_theta_p=[])

    def voltages(self, t, theta_p):
        """
        Compute the grid voltage in stationary frame.
        
        Parameters
        ----------
        t : float
            Time (s).
        theta_p : float
            Phase A voltage angle (rad).

        Returns
        -------
        e_gs : complex
            Grid complex voltage (V).

        """
        par = self.par
        e_g_abs = par.e_g_abs(t) if callable(par.e_g_abs) else par.e_g_abs
        e_gs = e_g_abs*np.exp(1j*(theta_p - par.phi(t)))

        # Adding negative sequence component if given
        if par.e_g_neg_abs is not None:
            e_g_neg_abs = par.e_g_neg_abs(t) if callable(
                par.e_g_neg_abs) else par.e_g_neg_abs
            e_gs += e_g_neg_abs*np.exp(
                -1j*(theta_p + par.phi_neg + par.phi(t)))

        return e_gs

    def set_outputs(self, t):
        """Set output variables."""
        self.out.e_gs = self.voltages(t, np.angle(self.state.exp_j_theta_p))

    def set_inputs(self, t):
        """Set input variables."""
        self.inp.w_g = self.par.w_g(t) if callable(
            self.par.w_g) else self.par.w_g

    def rhs(self):
        """
        Compute the state derivatives.
        
        Returns
        -------
        Complex list, length 1
            Time derivative of the complex state vector, [d_exp_j_theta_p].
            
        """
        d_exp_j_theta_p = 1j*self.inp.w_g*self.state.exp_j_theta_p
        return [d_exp_j_theta_p]

    def meas_voltages(self, t):
        """
        Measure the grid phase voltages.
        
        Parameters
        ----------
        t : float
            Time (s).

        Returns
        -------
        e_g_abc : 3-tuple of floats
            Phase voltages (V).

        """
        e_gs = self.voltages(t, np.angle(self.state.exp_j_theta_p))
        return complex2abc(e_gs)

    def post_process_states(self):
        """Post-process the solution."""
        if callable(self.par.w_g):
            self.data.w_g = self.par.w_g(self.data.t)
        else:
            self.data.w_g = np.full(np.size(self.data.t), self.par.w_g)
        self.data.theta_p = np.angle(self.data.exp_j_theta_p)
        self.data.e_gs = self.voltages(self.data.t, self.data.theta_p)
        self.data.theta_g = np.angle(self.data.e_gs)
