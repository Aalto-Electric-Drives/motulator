"""
Continuous-time models for converters.

A three-phase voltage-source inverter with optional DC-bus dynamics is
modelled, along with a six-pulse diode bridge rectifier supplied from a stiff
grid. Complex space vectors are used also for duty ratios and switching states,
wherever applicable. 

"""

from types import SimpleNamespace

import numpy as np

from motulator.common.model._simulation import Subsystem
from motulator.common.utils import abc2complex, complex2abc
from motulator.grid.model import StiffSource


# %%
class Inverter(Subsystem):
    """
    Lossless three-phase voltage source inverter.

    Capacitive DC-bus dynamics are modeled if C_dc is given as
    a parameter. In this case, the capacitor voltage u_dc is used as
    a state variable. Otherwise the DC voltage is constant or a function of
    time depending on the type of parameter u_dc.

    Parameters
    ----------
    u_dc : float | callable
        DC-bus voltage (V).
    C_dc : float, optional
        DC-bus capacitance (F). Default is None.
    i_ext : callable, optional
        External DC current, seen as disturbance, `i_ext(t)`. Default is zero,
        ``lambda t: 0``.
    
    """

    def __init__(self, u_dc, C_dc=None, i_ext=lambda t: 0):
        super().__init__()
        self.i_ext = i_ext
        self.par = SimpleNamespace(
            u_dc=u_dc,
            C_dc=C_dc,
        )
        # Initial values
        self.u_dc0 = self.par.u_dc(0) if callable(
            self.par.u_dc) else self.par.u_dc
        # Only initialize states if dynamic DC model is used
        if self.par.C_dc is not None:
            self.state = SimpleNamespace(u_dc=self.u_dc0)
            self.sol_states = SimpleNamespace(u_dc=[])
        self.inp = SimpleNamespace(
            u_dc=self.u_dc0,
            i_ext=i_ext(0),
            q_cs=None,
            i_cs=0j,
        )
        self.sol_q_cs = []

    @property
    def u_dc(self):
        """DC-bus voltage (V)."""
        if self.par.C_dc is not None:
            return self.state.u_dc.real
        return self.inp.u_dc

    @property
    def u_cs(self):
        """AC-side voltage (V)."""
        return self.inp.q_cs*self.u_dc

    @property
    def i_dc(self):
        """DC-side current (A)."""
        return 1.5*np.real(self.inp.q_cs*np.conj(self.inp.i_cs))

    def set_outputs(self, _):
        """Set output variables."""
        self.out.u_cs = self.u_cs
        self.out.u_dc = self.u_dc

    def set_inputs(self, t):
        """Set input variables."""
        self.inp.u_dc = self.par.u_dc(t) if callable(
            self.par.u_dc) else self.par.u_dc
        self.inp.i_ext = self.i_ext(t)

    def rhs(self):
        """
        Compute the state derivatives.

        Returns
        -------
        complex list, length 1
            Time derivative of the complex state vector, [d_u_dc].

        """
        inp, par = self.inp, self.par
        if par.C_dc is None:  # Check whether dynamic DC model is used
            return None
        d_u_dc = (inp.i_ext - self.i_dc)/par.C_dc
        return [d_u_dc]

    def meas_dc_voltage(self):
        """
        Measure the converter DC-bus voltage.

        Returns
        -------
        u_dc : float
            DC-bus voltage (V).

        """
        return self.u_dc

    def post_process_states(self):
        """Post-process data."""
        data = self.data
        if self.par.C_dc is None:
            if callable(self.u_dc):
                self.data.u_dc = self.u_dc(self.data.t)
            else:
                self.data.u_dc = np.full(np.size(self.data.t), self.u_dc)
        else:
            data.u_dc = data.u_dc.real
        data.u_cs = data.q_cs*data.u_dc

    def post_process_with_inputs(self):
        """Post-process data with inputs."""
        data = self.data
        data.i_dc = 1.5*np.real(data.q_cs*np.conj(data.i_cs))


# %%
class DiodeBridge(StiffSource):
    """
    Three-phase diode bridge supplied from a stiff grid.

    A three-phase diode bridge rectifier with a DC-side inductor is modeled.
    The supply voltage is an ideal three-phase voltage source with constant
    frequency and voltage magnitude. The inductor current i_L is used as a
    state variable.

    Parameters
    ----------
    L_dc : float
        DC-bus inductance (H).
    U_g : float
        Grid voltage (V, line-line, rms).
    f_g : float
        Grid frequency (Hz).

    """

    def __init__(self, L_dc, U_g, f_g):
        super().__init__(w_g=2*np.pi*f_g, e_g_abs=np.sqrt(2/3)*U_g)
        self.par.L = L_dc
        self.state = SimpleNamespace(i_L=0)
        self.sol_states = SimpleNamespace(i_L=[])

    def set_outputs(self, t):
        """Set output variables."""
        self.out.u_gs = self.voltages(t, self.par.w_g*t)
        self.out.i_L = self.state.i_L.real

    def rhs(self):
        """
        Compute the state derivatives.

        Returns
        -------
        complex list, length 1
            Time derivative of the complex state vector, [d_i_L].

        """
        state, inp, out, par = self.state, self.inp, self.out, self.par
        # Output voltage of the diode bridge
        u_g_abc = complex2abc(out.u_gs)
        u_di = np.amax(u_g_abc, axis=0) - np.amin(u_g_abc, axis=0)
        # State derivatives
        d_i_L = (u_di - inp.u_dc)/par.L
        # The inductor current cannot be negative due to the diode bridge
        if state.i_L < 0 and d_i_L < 0:
            d_i_L = 0

        return [d_i_L]

    def post_process_states(self):
        """Post-process data."""
        self.data.i_L = self.data.i_L.real
        self.data.u_gs = self.voltages(self.data.t, self.par.w_g*self.data.t)

    def post_process_with_inputs(self):
        """Post-process data with inputs."""
        data = self.data
        data.u_g_abc = complex2abc(data.u_gs)
        # Voltage at the output of the diode bridge
        data.u_di = (
            np.amax(data.u_g_abc, axis=0) - np.amin(data.u_g_abc, axis=0))
        # Diode bridge switching states (-1, 0, 1)
        data.q_g_abc = (
            (np.amax(data.u_g_abc, axis=0) == data.u_g_abc).astype(int) -
            (np.amin(data.u_g_abc, axis=0) == data.u_g_abc).astype(int))
        # Grid current space vector
        data.i_gs = abc2complex(data.q_g_abc)*data.i_L
