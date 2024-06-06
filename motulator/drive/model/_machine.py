"""
Continuous-time models for electric machines.

Peak-valued complex space vectors are used. 

"""
from types import SimpleNamespace

import numpy as np

from motulator._common._model import Subsystem
from motulator.utils import complex2abc, wrap


# %%
class InductionMachine(Subsystem):
    """
    Γ-equivalent model of an induction machine.

    An induction machine is modeled using the Γ-equivalent model [#Sle1989]_. 
    The model is implemented in stator coordinates. The flux linkages are used 
    as state variables. The stator inductance `L_s` can either be constant or
    a function of the stator flux magnitude::

        L_s = L_s(abs(psi_ss))

    Parameters
    ----------
    R_s : float
        Stator resistance (Ω).
    R_r : float
        Rotor resistance (Ω).
    L_ell : float
        Leakage inductance (H).
    L_s : float | callable
        Stator inductance (H) or a callable L_s = L_s(abs(psi_ss)).
    n_p : int
        Number of pole pairs.

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
        super().__init__()
        self.par = SimpleNamespace(n_p=n_p, R_s=R_s, R_r=R_r, L_ell=L_ell)
        self._L_s = L_s
        # States
        self.state = SimpleNamespace(psi_ss=0j, psi_rs=0j)
        # Store the solutions in these lists
        self.sol_states = SimpleNamespace(psi_ss=[], psi_rs=[])

    @property
    def L_s(self):
        """Stator inductance (H)."""
        if callable(self._L_s):
            return self._L_s(np.abs(self.state.psi_ss))
        return self._L_s

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
        state, inp, out, par = self.state, self.inp, self.out, self.par
        d_psi_ss = inp.u_ss - par.R_s*out.i_ss
        d_psi_rs = (-par.R_r*out.i_rs + 1j*par.n_p*inp.w_M*state.psi_rs)

        return [d_psi_ss, d_psi_rs]

    def meas_currents(self):
        """Measure the phase currents."""
        i_s_abc = complex2abc(self.i_ss)  # + noise + offset ...
        return i_s_abc

    def post_process_states(self):
        """Post-process the solution."""
        data = self.data
        L_s = self._L_s(np.abs(data.psi_ss)) if callable(
            self._L_s) else self._L_s
        gamma = L_s/(L_s + self.par.L_ell)
        data.psi_Rs = gamma*data.psi_rs
        data.i_rs = (data.psi_rs - data.psi_ss)/self.par.L_ell
        data.i_ss = data.psi_ss/L_s - data.i_rs
        data.tau_M = 1.5*self.par.n_p*np.imag(data.i_ss*np.conj(data.psi_ss))

    def post_process_with_inputs(self):
        """Post-process the solution."""
        self.data.w_m = self.par.n_p*self.data.w_M


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
        gamma = L_M/(L_M + L_sgm)
        super().__init__(n_p, R_s, R_R/gamma**2, L_sgm/gamma, L_M + L_sgm)


# %%
class SynchronousMachine(Subsystem):
    """
    Synchronous machine model.

    This models a synchronous machine in rotor coordinates. The stator flux 
    linkage and the electrical angle of the rotor are the state variables. 

    Parameters
    ----------
    R_s : float
        Stator resistance (Ω).
    L_d : float
        d-axis inductance (H).
    L_q : float
        q-axis inductance (H).
    psi_f : float
        PM-flux linkage (Vs).
    n_p : int
        Number of pole pairs.

    """

    def __init__(
            self,
            n_p,
            R_s,
            L_d=None,
            L_q=None,
            psi_f=None,
            i_s=None,
            psi_s0=None):
        super().__init__()
        self.par = SimpleNamespace(
            n_p=n_p, R_s=R_s, L_d=L_d, L_q=L_q, psi_f=psi_f)
        self._i_s = i_s
        # Initial values
        if psi_s0 is not None:
            self.state = SimpleNamespace(psi_s=complex(psi_s0), theta_m=0)
        else:
            self.state = SimpleNamespace(psi_s=complex(psi_f), theta_m=0)
        # Store the solutions in these lists
        self.sol_states = SimpleNamespace(psi_s=[], theta_m=[])

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
        out = self.out
        out.i_s, out.tau_M = self.i_s, self.tau_M
        out.i_ss = self.i_s*np.exp(1j*self.state.theta_m)

    def rhs(self):
        """Compute state derivatives."""
        state, inp, out, par = self.state, self.inp, self.out, self.par
        inp.u_s = inp.u_ss*np.exp(-1j*state.theta_m)
        d_psi_s = inp.u_s - par.R_s*out.i_s - 1j*par.n_p*inp.w_M*state.psi_s
        d_theta_m = par.n_p*inp.w_M
        # TODO: Wrap the angle

        return [d_psi_s, d_theta_m]

    def meas_currents(self):
        """Measure the phase currents."""
        i_s_abc = complex2abc(np.exp(1j*self.state.theta_m)*self.i_s)
        return i_s_abc

    def post_process_states(self):
        """Post-process the solution."""
        data, par = self.data, self.par
        if callable(self._i_s):
            data.i_s = self._i_s(data.psi_s)
        else:
            data.i_s = ((data.psi_s.real - par.psi_f)/par.L_d +
                        1j*data.psi_s.imag/par.L_q)
        data.i_ss = data.i_s*np.exp(1j*data.theta_m)
        data.psi_ss = data.psi_s*np.exp(1j*data.theta_m)
        data.tau_M = 1.5*self.par.n_p*np.imag(data.i_s*np.conj(data.psi_s))
        data.theta_m = wrap(data.theta_m.real)

    def post_process_with_inputs(self):
        """Post-process the solution."""
        self.data.w_m = self.par.n_p*self.data.w_M
