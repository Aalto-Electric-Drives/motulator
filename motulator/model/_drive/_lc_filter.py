"""
Continuous-time model for an output LC filter.

The space vector model is implemented in stator coordinates. 

"""
from types import SimpleNamespace
from motulator._helpers import complex2abc
from motulator.model._simulation import Subsystem


# %%
class LCFilter(Subsystem):
    """
    LC-filter model.

    Parameters
    ----------
    L : float
        Inductance (H).
    C : float
        Capacitance (F). 
    R : float, optional
        Series resistance (Ω) of the inductor. The default is 0.
   
    """

    def __init__(self, L, C, R=0):
        super().__init__()
        self.par = SimpleNamespace(L=L, C=C, R=R)
        self.state = SimpleNamespace(i_cs=0, u_fs=0)
        self.sol_states = SimpleNamespace(i_cs=[], u_fs=[])

    def set_outputs(self, _):
        """Set output variables."""
        state, out = self.state, self.out
        out.i_cs, out.u_fs = state.i_cs, state.u_fs

    def rhs(self):
        """Compute the state derivative."""
        state, inp, par = self.state, self.inp, self.par
        d_i_cs = (inp.u_cs - state.u_fs - par.R*state.i_cs)/par.L
        d_u_fs = (state.i_cs - inp.i_fs)/par.C

        return [d_i_cs, d_u_fs]

    def meas_currents(self):
        """Measure the converter phase currents."""
        i_c_abc = complex2abc(self.state.i_cs)
        return i_c_abc

    def meas_voltages(self):
        """Measure the capacitor phase voltages."""
        u_s_abc = complex2abc(self.state.u_fs)
        return u_s_abc
