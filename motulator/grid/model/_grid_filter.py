"""
Grid and AC filter impedance models.

This module contains continuous-time models for subsystems comprising an AC 
filter and a grid impedance between the converter and grid voltage sources. The 
models are implemented with space vectors in stationary coordinates.

"""
from types import SimpleNamespace

import numpy as np
from motulator.common.utils._utils import complex2abc
from motulator.common.model import Subsystem


# %%
class LFilter(Subsystem):
    """
    Dynamic model for an inductive L filter and an inductive-resistive grid.

    An L filter and an inductive-resistive grid impedance, between the converter 
    and grid voltage sources, are modeled combining their inductances and series
    resistances in a state equation. The grid current is used as a state 
    variable. The point-of-common-coupling (PCC) voltage between the L filter 
    and the grid impedance is separately calculated.

    Parameters
    ----------
    L_f : float
        Filter inductance (H)
    R_f : float
        Filter series resistance (Ω)
    L_g : float
        Grid inductance (H)
    R_g : float
        Grid resistance (Ω)

    """
    def __init__(self, U_gN, L_f, R_f=0, L_g=0, R_g=0):
        super().__init__()
        self.par = SimpleNamespace(L_f=L_f, R_f=R_f, L_g=L_g, R_g=R_g)
        self.inp = SimpleNamespace(u_cs=0+0j, e_gs=U_gN+0j)
        self.out = SimpleNamespace(u_gs=U_gN+0j)  # Needed for direct feedthrough
        self.state = SimpleNamespace(i_gs=0+0j)
        self.sol_states = SimpleNamespace(i_gs=[])

    def set_outputs(self, _):
        """Set output variables."""
        state, out = self.state, self.out
        u_gs = (self.par.L_g*self.inp.u_cs + self.par.L_f*self.inp.e_gs +
            (self.par.R_g*self.par.L_f - self.par.R_f*self.par.L_g)*
            self.state.i_gs)/(self.par.L_g+self.par.L_f)
        out.i_gs, out.i_cs, out.u_gs = state.i_gs, state.i_gs, u_gs

    def rhs(self):
        # pylint: disable=R0913
        """
        Compute the state derivatives.

        Returns
        -------
        complex list, length 1
            Time derivative of the complex state vector, [di_gs]

        """
        state, inp, par = self.state, self.inp, self.par
        L_t = par.L_f + par.L_g
        R_t = par.R_f + par.R_g
        di_gs = (inp.u_cs - inp.e_gs - R_t*state.i_gs)/L_t
        return [di_gs]

    def meas_currents(self):
        """
        Measure the phase currents at the end of the sampling period.

        Returns
        -------
        i_g_abc : 3-tuple of floats
            Grid phase currents (A).

        """
        # Grid phase currents from the corresponding space vector
        i_g_abc = complex2abc(self.state.i_gs)
        return i_g_abc

    def meas_pcc_voltage(self):
        """
        Measure the phase voltages at PCC at the end of the sampling period.

        Returns
        -------
        u_g_abc : 3-tuple of floats
            Phase voltage at the point of common coupling (V).

        """
        # PCC phase voltages from the corresponding space vector
        u_g_abc = complex2abc(self.out.u_gs)
        return u_g_abc

    def post_process_states(self):
        """Post-process data."""
        self.data.i_cs=self.data.i_gs

    def post_process_with_inputs(self):
        """Post-process data with inputs."""
        data=self.data
        data.u_gs=(self.par.L_g*data.u_cs + self.par.L_f*data.e_gs +
            (self.par.R_g*self.par.L_f - self.par.R_f*self.par.L_g)*
            data.i_gs)/(self.par.L_g+self.par.L_f)


# %%
class LCLFilter:
    """
    Dynamic model for an inductive-capacitive-inductive (LCL) filter and a grid.

    An LCL filter and an inductive-resistive grid impedance, between the 
    converter and grid voltage sources, are modeled using converter current, 
    LCL-filter capacitor voltage and grid current as state variables. The grid 
    inductance and resistance are included in the state equation of the grid 
    current. The point-of-common-coupling (PCC) voltage between the LCL filter 
    and the grid impedance is separately calculated.

    Parameters
    ----------
    L_fc : float
        Converter-side inductance of the LCL filter (H)
    R_fc : float
        Converter-side series resistance (Ω)
    L_fg : float
        Grid-side inductance of the LCL filter (H)
    R_fg : float
        Grid-side series resistance (Ω)
    C_f : float
        Capacitance of the LCL Filter (F)
    G_f : float
        Conductance of a resistor in parallel with the LCL-filter capacitor (S)
    L_g : float
        Grid inductance (H)
    R_g : float
        Grid resistance (Ω)


    """
    def __init__(
            self, U_gN=400*np.sqrt(2/3), L_fc=6e-3, R_fc=0, L_fg=3e-3,
            R_fg=0, C_f=10e-6, G_f=0, L_g=0, R_g=0):
        self.L_fc = L_fc
        self.R_fc = R_fc
        self.L_fg = L_fg
        self.R_fg = R_fg
        self.C_f = C_f
        self.G_f = G_f
        self.L_g = L_g
        self.R_g = R_g
        # Storing useful variables
        self.u_gs0 = U_gN + 0j
        # Initial values
        self.i_cs0 = 0j
        self.i_gs0 = 0j
        self.u_fs0 = U_gN + 0j


    def pcc_voltages(self, i_gs, u_fs, e_gs):
        """
        Compute the PCC voltage between the LCL filter and the grid impedance.

        Parameters
        ----------
        i_gs : complex
            Grid current (A).
        u_fs : complex
            LCL-filter capacitor voltage (V).
        e_gs : complex
            Grid voltage (V).

        Returns
        -------
        u_gs : complex
            Voltage at the point of common coupling (V).

        """
        # PCC voltage in alpha-beta coordinates
        u_gs = (self.L_fg*e_gs + self.L_g*u_fs + 
            (self.R_g*self.L_fg-self.R_fg*self.L_g)*i_gs)/(self.L_g+self.L_fg)

        return u_gs

    def f(self, i_cs, u_fs, i_gs, u_cs, e_gs):
        # pylint: disable=R0913
        """
        Compute the state derivatives.

        Parameters
        ----------
        i_cs : complex
            Converter current (A).
        u_fs : complex
            LCL-filter capacitor voltage (V).
        i_gs : complex
            Grid current (A).
        u_cs : complex
            Converter voltage (V).
        e_gs : complex
            Grid voltage (V).

        Returns
        -------
        complex list, length 3
            Time derivative of the complex state vector, [di_cs, du_fs, di_gs]

        """
        # Converter current dynamics
        di_cs = (u_cs - u_fs - self.R_fc*i_cs)/self.L_fc
        # Capacitor voltage dynamics
        du_fs = (i_cs - i_gs - self.G_f*u_fs)/self.C_f
        # Calculation of the total grid-side impedance
        L_t = self.L_fg + self.L_g
        R_t = self.R_fg + self.R_g
        # Grid current dynamics
        di_gs = (u_fs - e_gs - R_t*i_gs)/L_t

        return [di_cs, du_fs, di_gs]

    def meas_currents(self):
        """
        Measure the converter phase currents at the end of the sampling period.

        Returns
        -------
        i_c_abc : 3-tuple of floats
            Converter phase currents (A).

        """
        # Converter phase currents from the corresponding space vector
        i_c_abc = complex2abc(self.i_cs0)

        return i_c_abc

    def meas_grid_currents(self):
        """
        Measure the grid phase currents at the end of the sampling period.

        Returns
        -------
        i_g_abc : 3-tuple of floats
            Grid phase currents (A).

        """
        # Grid phase currents from the corresponding space vector
        i_g_abc = complex2abc(self.i_gs0)
        return i_g_abc

    def meas_cap_voltage(self):
        """
        Measure the capacitor phase voltages at the end of the sampling period.

        Returns
        -------
        u_f_abc : 3-tuple of floats
            Phase voltages of the LCL filter capacitor (V).

        """  
        # Capacitor phase voltages from the corresponding space vector
        u_f_abc = complex2abc(self.u_fs0)
        return u_f_abc

    def meas_pcc_voltage(self):
        """
        Measure the PCC voltages at the end of the sampling period.

        Returns
        -------
        u_g_abc : 3-tuple of floats
            Phase voltages at the point of common coupling (V).

        """
        # PCC phase voltages from the corresponding space vector
        u_g_abc = complex2abc(self.u_gs0)
        return u_g_abc
