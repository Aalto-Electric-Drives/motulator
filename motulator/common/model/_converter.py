"""
Continuous-time models for converters.

A three-phase voltage-source inverter with optional DC-bus dynamics is
modelled, along with a six-pulse diode bridge rectifier supplied from a stiff
grid. Complex space vectors are used also for duty ratios and switching states,
wherever applicable.

"""
from types import SimpleNamespace

import numpy as np

from motulator.common.model import Subsystem
from motulator.common.utils import abc2complex, complex2abc


# %%
class VoltageSourceConverter(Subsystem):
    """
    Lossless three-phase voltage-source converter.

    Parameters
    ----------
    u_dc : float
        DC-bus voltage (V). If the DC-bus capacitor is modeled, this value is
        used as the initial condition.
    C_dc : float, optional
        DC-bus capacitance (F). The default is None.
    i_dc : callable, optional
        External current (A) fed to the DC bus. Needed if `C_dc` is not None.

    """

    def __init__(self, u_dc, C_dc=None, i_dc=lambda t: None):
        super().__init__()
        self.par = SimpleNamespace(u_dc=u_dc, C_dc=C_dc)
        self.inp = SimpleNamespace(q_cs=None, i_cs=0j)
        if C_dc is not None:
            self.state = SimpleNamespace(u_dc=u_dc)
            self.sol_states = SimpleNamespace(u_dc=[])
            self.i_dc = i_dc
            self.inp.i_dc = i_dc(0)
        self.sol_q_cs = []

    @property
    def u_dc(self):
        """DC-bus voltage (V)."""
        if self.par.C_dc is not None:
            return self.state.u_dc.real
        return self.par.u_dc

    @property
    def u_cs(self):
        """AC-side voltage (V)."""
        return self.inp.q_cs*self.u_dc

    @property
    def i_dc_int(self):
        """Converter-side DC current (A)."""
        return 1.5*np.real(self.inp.q_cs*np.conj(self.inp.i_cs))

    def set_outputs(self, _):
        """Set output variables."""
        self.out.u_cs = self.u_cs
        self.out.u_dc = self.u_dc

    def set_inputs(self, t):
        """Set input variables."""
        if self.par.C_dc is not None:
            self.inp.i_dc = self.i_dc(t)

    def rhs(self):
        """Compute the state derivatives."""
        if self.par.C_dc is not None:
            d_u_dc = (self.inp.i_dc - self.i_dc_int)/self.par.C_dc
            return [d_u_dc]
        return []

    def meas_dc_voltage(self):
        """Measure the converter DC-bus voltage (V)."""
        return self.u_dc

    def post_process_states(self):
        """Post-process data."""
        data = self.data
        if self.par.C_dc is not None:
            data.u_dc = data.u_dc.real
        else:
            if callable(self.u_dc):
                self.data.u_dc = self.u_dc(self.data.t)
            else:
                self.data.u_dc = np.full(np.size(self.data.t), self.u_dc)
        data.u_cs = data.q_cs*data.u_dc

    def post_process_with_inputs(self):
        """Post-process data with inputs."""
        data = self.data
        data.i_dc_int = 1.5*np.real(data.q_cs*np.conj(data.i_cs))


# %%
class FrequencyConverter(VoltageSourceConverter):
    """
    Frequency converter with a six-pulse diode bridge.

    A three-phase diode bridge rectifier with a DC-bus inductor is modeled. The
    diode bridge is connected to the voltage-source inverter. The inductance of
    the grid is omitted.

    Parameters
    ----------
    C_dc : float
        DC-bus capacitance (F).
    L_dc : float
        DC-bus inductance (H).
    U_g : float
        Grid voltage (V, line-line, rms).
    f_g : float
        Grid frequency (Hz).

    """

    def __init__(self, C_dc, L_dc, U_g, f_g):
        super().__init__(np.sqrt(2)*U_g, C_dc)
        self.par = SimpleNamespace(
            L_dc=L_dc, C_dc=C_dc, w_g=2*np.pi*f_g, u_g=np.sqrt(2/3)*U_g)
        self.state.i_L = 0
        self.state.exp_j_theta_g = complex(1)
        self.sol_states.i_L, self.sol_states.exp_j_theta_g = [], []

    def set_outputs(self, t):
        """Set output variables."""
        super().set_outputs(t)
        self.out.i_L = self.state.i_L.real

    def set_inputs(self, t):
        """Set output variables."""
        self.inp.i_dc = self.out.i_L
        self.inp.u_dc = self.out.u_dc

    def rhs(self):
        """Compute the state derivatives."""
        # Grid voltage
        u_gs = self.par.u_g*self.state.exp_j_theta_g
        u_g_abc = complex2abc(u_gs)
        # Output voltage of the diode bridge
        u_di = np.amax(u_g_abc, axis=0) - np.amin(u_g_abc, axis=0)
        # State derivatives
        d_exp_j_theta_g = 1j*self.par.w_g*self.state.exp_j_theta_g
        d_i_L = (u_di - self.inp.u_dc)/self.par.L_dc
        # The inductor current cannot be negative due to the diode bridge
        if self.state.i_L < 0 and d_i_L < 0:
            d_i_L = 0
        return super().rhs() + [d_i_L, d_exp_j_theta_g]

    def post_process_states(self):
        """Post-process data."""
        super().post_process_states()
        self.data.i_L = self.data.i_L.real

    def post_process_with_inputs(self):
        """Post-process data with inputs."""
        super().post_process_with_inputs()
        data = self.data
        data.u_gs = self.par.u_g*data.exp_j_theta_g
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
