"""
Continuous-time model for an output LC filter.

The space vector model is implemented in stator coordinates.

"""
from types import SimpleNamespace

from motulator.common.model import Subsystem
from motulator.common.utils import complex2abc


# %%
class LCFilter(Subsystem):
    """
    LC-filter model.

    Parameters
    ----------
    L_f : float
        Filter inductance (H).
    C_f : float
        Filter capacitance (F).
    R_f : float, optional
        Series resistance (Ω) of the inductor. The default is 0.
    """

    def __init__(self, L_f, C_f, R_f=0):
        super().__init__()
        self.par = SimpleNamespace(
            L_f=L_f,
            C_f=C_f,
            R_f=R_f,
        )
        self.state = SimpleNamespace(i_cs=0, u_fs=0)
        self.sol_states = SimpleNamespace(i_cs=[], u_fs=[])

    def set_outputs(self, _):
        """Set output variables."""
        state, out = self.state, self.out
        out.i_cs, out.u_fs = state.i_cs, state.u_fs

    def rhs(self):
        """Compute state derivatives."""
        state, inp, par = self.state, self.inp, self.par
        d_i_cs = (inp.u_cs - state.u_fs - par.R_f*state.i_cs)/par.L_f
        d_u_fs = (state.i_cs - inp.i_fs)/par.C_f

        return [d_i_cs, d_u_fs]

    def meas_currents(self):
        """Measure the converter phase currents."""
        i_c_abc = complex2abc(self.state.i_cs)
        return i_c_abc

    def meas_capacitor_voltages(self):
        """Measure the capacitor phase voltages."""
        u_f_abc = complex2abc(self.state.u_fs)
        return u_f_abc
