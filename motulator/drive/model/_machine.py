"""
Continuous-time models for electric machines.

Peak-valued complex space vectors are used. 

"""
from types import SimpleNamespace

import numpy as np

from motulator.common.model import Subsystem
from motulator.common.utils import complex2abc


# %%
class InductionMachine(Subsystem):
    """
    Γ-equivalent model of an induction machine.

    An induction machine is modeled using the Γ-equivalent model [#Sle1989]_. 
    The stator inductance `L_s` can either be constant or a function of the 
    stator flux magnitude::

        L_s = L_s(abs(psi_ss))

    Parameters
    ----------
    par : InductionMachinePars

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

    def __init__(self, par):
        super().__init__()
        self.par = par
        # States
        self.state = SimpleNamespace(psi_ss=0j, psi_rs=0j)
        # Store the solutions in these lists
        self.sol_states = SimpleNamespace(psi_ss=[], psi_rs=[])

    @property
    def L_s(self):
        """Stator inductance (H)."""
        if callable(self.par.L_s):
            return self.par.L_s(np.abs(self.state.psi_ss))
        return self.par.L_s

    @property
    def i_rs(self):
        """Rotor current (A)."""
        return (self.state.psi_rs - self.state.psi_ss)/self.par.L_ell

    @property
    def i_ss(self):
        """Stator current (A)."""
        return self.state.psi_ss/self.L_s - self.i_rs

    @property
    def tau_M(self):
        """Electromagnetic torque (Nm)."""
        return 1.5*self.par.n_p*np.imag(self.i_ss*np.conj(self.state.psi_ss))

    def set_outputs(self, _):
        """Set output variables."""
        out = self.out
        out.i_ss, out.i_rs, out.tau_M = self.i_ss, self.i_rs, self.tau_M

    def rhs(self):
        """Compute state derivatives."""
        # Flux linkages are used as state variables
        state, inp, out, par = self.state, self.inp, self.out, self.par
        d_psi_ss = inp.u_ss - par.R_s*out.i_ss
        d_psi_rs = -par.R_r*out.i_rs + 1j*par.n_p*inp.w_M*state.psi_rs

        return [d_psi_ss, d_psi_rs]

    def meas_currents(self):
        """Measure the phase currents."""
        i_s_abc = complex2abc(self.i_ss)  # + noise + offset ...
        return i_s_abc

    def post_process_states(self):
        """Post-process the solution."""
        data = self.data
        L_s = self.par.L_s(np.abs(data.psi_ss)) if callable(
            self.par.L_s) else self.par.L_s
        gamma = L_s/(L_s + self.par.L_ell)
        data.psi_Rs = gamma*data.psi_rs
        data.i_rs = (data.psi_rs - data.psi_ss)/self.par.L_ell
        data.i_ss = data.psi_ss/L_s - data.i_rs
        data.tau_M = 1.5*self.par.n_p*np.imag(data.i_ss*np.conj(data.psi_ss))

    def post_process_with_inputs(self):
        """Post-process the solution."""
        self.data.w_m = self.par.n_p*self.data.w_M


# %%
class SynchronousMachine(Subsystem):
    """
    Synchronous machine model.

    Parameters
    ----------
    par : SynchronousMachinePars
        Machine parameters.
    i_s : callable, optional
        Stator current (A) as a function of the stator flux linkage (A) in 
        order to model the magnetic saturation. If this function is given, the
        stator current is computed using this function instead of constants
        `par.L_d`, `par.L_q`, and `par.psi_f`.  
    psi_s0 : float, optional
        Initial stator flux linkage (Vs). If not given, `par.psi_f` is used.

    """

    def __init__(self, par, i_s=None, psi_s0=None):
        super().__init__()
        self.par = par
        self._i_s = i_s
        psi_s_initial = psi_s0 if psi_s0 is not None else par.psi_f
        self.state = SimpleNamespace(
            psi_s=complex(psi_s_initial), exp_j_theta_m=complex(1))
        self.sol_states = SimpleNamespace(psi_s=[], exp_j_theta_m=[])

    @property
    def i_s(self):
        """Stator current (A)."""
        if callable(self._i_s):
            return self._i_s(self.state.psi_s)
        return ((self.state.psi_s.real - self.par.psi_f)/self.par.L_d +
                1j*self.state.psi_s.imag/self.par.L_q)

    @property
    def tau_M(self):
        """Electromagnetic torque (Nm)."""
        return 1.5*self.par.n_p*np.imag(self.i_s*np.conj(self.state.psi_s))

    def set_outputs(self, _):
        """Set output variables."""
        self.out.i_s = self.i_s
        self.out.tau_M = self.tau_M
        self.out.i_ss = self.i_s*self.state.exp_j_theta_m

    def rhs(self):
        """Compute state derivatives."""
        # Rotor coordinates are internally used
        state, inp, out, par = self.state, self.inp, self.out, self.par
        inp.u_s = inp.u_ss*np.conj(state.exp_j_theta_m)
        d_psi_s = inp.u_s - par.R_s*out.i_s - 1j*par.n_p*inp.w_M*state.psi_s
        d_exp_j_theta_m = 1j*par.n_p*inp.w_M*state.exp_j_theta_m

        return [d_psi_s, d_exp_j_theta_m]

    def meas_currents(self):
        """Measure the phase currents."""
        i_s_abc = complex2abc(self.state.exp_j_theta_m*self.i_s)
        return i_s_abc

    def post_process_states(self):
        """Post-process the solution."""
        data, par = self.data, self.par
        if callable(self._i_s):
            data.i_s = self._i_s(data.psi_s)
        else:
            data.i_s = ((data.psi_s.real - par.psi_f)/par.L_d +
                        1j*data.psi_s.imag/par.L_q)
        data.i_ss = data.i_s*data.exp_j_theta_m
        data.psi_ss = data.psi_s*data.exp_j_theta_m
        data.tau_M = 1.5*self.par.n_p*np.imag(data.i_s*np.conj(data.psi_s))
        data.theta_m = np.angle(data.exp_j_theta_m)

    def post_process_with_inputs(self):
        """Post-process the solution."""
        self.data.w_m = self.par.n_p*self.data.w_M
