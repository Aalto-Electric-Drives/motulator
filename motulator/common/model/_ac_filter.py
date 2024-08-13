"""Continuous-time models for converter AC-side filters."""

from types import SimpleNamespace

from motulator.common.model._model import Subsystem
from motulator.common.utils import complex2abc, FilterPars
from motulator.grid.utils import GridPars


# %%
class ACFilter(Subsystem):
    """
    Base class for converter AC-side filters.

    This provides a base class and wrapper for converter AC-side filters
    (LFilter, LCLFilter, LCFilter). Calling this class returns one of the three
    subclasses depending on whether values for filter capacitance C_f and
    filter grid-side inductance L_fg are included in the FilterPars object.

    Parameters
    ----------
    filter_par : FilterPars
        Filter model parameters.
    grid_par : GridPars, optional
        Grid model parameters. Default is None.

    """

    def __new__(cls, filter_par: FilterPars, grid_par: GridPars = None):
        if filter_par.C_f != 0:
            if filter_par.L_fg != 0:
                return super().__new__(LCLFilter)
            if grid_par is not None:
                raise SyntaxError(
                    "Could not create filter with given parameters. Did you " +
                    "forget to specify L_fg?")
            return super().__new__(LCFilter)
        return super().__new__(LFilter)

    def meas_currents(self):
        """
        Measure the converter phase currents.

        Returns
        -------
        i_c_abc : 3-tuple of floats
            Converter phase currents (A).

        """
        # Converter phase currents from the corresponding space vector
        i_c_abc = complex2abc(self.state.i_cs)

        return i_c_abc

    def meas_grid_currents(self):
        """
        Measure the grid phase currents.

        Returns
        -------
        i_g_abc : 3-tuple of floats
            Grid phase currents (A).

        """
        # Grid phase currents from the corresponding space vector
        i_g_abc = complex2abc(self.state.i_gs)
        return i_g_abc

    def meas_cap_voltage(self):
        """
        Measure the capacitor phase voltages.

        Returns
        -------
        u_f_abc : 3-tuple of floats
            Phase voltages of the filter capacitor (V).

        """
        # Capacitor phase voltages from the corresponding space vector
        u_f_abc = complex2abc(self.state.u_fs)
        return u_f_abc

    def meas_pcc_voltage(self):
        """
        Measure the phase voltages at the point of common coupling (PCC).

        Returns
        -------
        u_g_abc : 3-tuple of floats
            Phase voltages at the PCC (V).

        """
        # PCC phase voltages from the corresponding space vector
        u_g_abc = complex2abc(self.out.u_gs)
        return u_g_abc


# %%
class LFilter(ACFilter):
    """
    Dynamic model for an inductive L filter and an inductive-resistive grid.

    An L filter and an inductive-resistive grid impedance, between the
    converter and grid voltage sources, are modeled combining their inductances
    and series resistances in a state equation. The grid current is used as a
    state variable. The point-of-common-coupling (PCC) voltage between the L
    filter and the grid impedance is separately calculated.

    Parameters
    ----------
    grid_par : GridPars
        Grid model parameters. Needed to set the initial value of PCC voltage.
    filter_par : FilterPars
        Filter model parameters.
        L-Filter model uses only the following FilterPars parameters:

            L_fc : float
                Converter-side inductance of the filter (H).
            R_fc : float (optional)
                Converter-side series resistance (Ω). The default is 0.

    """

    def __init__(self, filter_par: FilterPars, grid_par: GridPars):
        super().__init__()
        self.par = SimpleNamespace(
            L_f=filter_par.L_fc,
            R_f=filter_par.R_fc,
            L_g=grid_par.L_g,
            R_g=grid_par.R_g,
        )
        self.inp = SimpleNamespace(u_cs=0 + 0j, e_gs=grid_par.u_gN + 0j)
        self.out = SimpleNamespace(
            u_gs=grid_par.u_gN + 0j,  # Needed for direct feedthrough
        )
        self.state = SimpleNamespace(i_cs=0 + 0j)
        self.sol_states = SimpleNamespace(i_cs=[])

    def set_outputs(self, _):
        """Set output variables."""
        state, par, inp, out = self.state, self.par, self.inp, self.out
        u_gs = (
            par.L_g*inp.u_cs + par.L_f*inp.e_gs +
            (par.R_g*par.L_f - par.R_f*par.L_g)*state.i_cs)/(
                par.L_g + par.L_f)
        out.i_cs, out.i_gs, out.u_gs = state.i_cs, state.i_cs, u_gs

    def rhs(self):
        """
        Compute the state derivatives.

        Returns
        -------
        complex list, length 1
            Time derivative of the complex state vector, [d_i_cs].

        """
        state, inp, par = self.state, self.inp, self.par
        L_t = par.L_f + par.L_g
        R_t = par.R_f + par.R_g
        d_i_cs = (inp.u_cs - inp.e_gs - R_t*state.i_cs)/L_t
        return [d_i_cs]

    def post_process_states(self):
        """Post-process data."""
        self.data.i_gs = self.data.i_cs

    def post_process_with_inputs(self):
        """Post-process data with inputs."""
        data = self.data
        data.u_gs = (
            self.par.L_g*data.u_cs + self.par.L_f*data.e_gs +
            (self.par.R_g*self.par.L_f - self.par.R_f*self.par.L_g)*
            data.i_cs)/(self.par.L_g + self.par.L_f)


# %%
class LCLFilter(ACFilter):
    """
    Dynamic model for an inductive-capacitive-inductive (LCL) filter and grid.

    An LCL filter and an inductive-resistive grid impedance, between the 
    converter and grid voltage sources, are modeled using converter current, 
    LCL-filter capacitor voltage and grid current as state variables. The grid 
    inductance and resistance are included in the state equation of the grid 
    current. The point-of-common-coupling (PCC) voltage between the LCL filter 
    and the grid impedance is separately calculated.

    Parameters
    ----------
    grid_par : GridPars
        Grid model parameters. Needed to set the initial value of PCC voltage.
    filter_par : FilterPars
        Filter model parameters.
    """

    def __init__(self, filter_par: FilterPars, grid_par: GridPars):
        super().__init__()
        self.par = SimpleNamespace(
            L_fc=filter_par.L_fc,
            R_fc=filter_par.R_fc,
            L_fg=filter_par.L_fg,
            R_fg=filter_par.R_fg,
            C_f=filter_par.C_f,
            G_f=filter_par.G_f,
            L_g=grid_par.L_g,
            R_g=grid_par.R_g,
        )
        self.inp = SimpleNamespace(u_cs=0 + 0j, e_gs=grid_par.u_gN + 0j)
        self.out = SimpleNamespace(u_gs=grid_par.u_gN + 0j)
        self.state = SimpleNamespace(
            i_cs=0 + 0j,
            u_fs=grid_par.u_gN + 0j,
            i_gs=0 + 0j,
        )
        self.sol_states = SimpleNamespace(i_cs=[], u_fs=[], i_gs=[])

    def set_outputs(self, _):
        """Set output variables."""
        state, par, inp, out = self.state, self.par, self.inp, self.out
        u_gs = (
            par.L_fg*inp.e_gs + par.L_g*state.u_fs +
            (par.R_g*par.L_fg - par.R_fg*par.L_g)*state.i_gs)/(
                par.L_g + par.L_fg)
        out.i_cs, out.u_fs, out.i_gs, out.u_gs = (
            state.i_cs, state.u_fs, state.i_gs, u_gs)

    def rhs(self):
        """
        Compute the state derivatives.

        Returns
        -------
        complex list, length 3
            Time derivative of the complex state vector,
            [d_i_cs, d_u_fs, d_i_gs].

        """
        state, par, inp = self.state, self.par, self.inp
        # Converter current dynamics
        d_i_cs = (inp.u_cs - state.u_fs - par.R_fc*state.i_cs)/par.L_fc
        # Capacitor voltage dynamics
        d_u_fs = (state.i_cs - state.i_gs - par.G_f*state.u_fs)/par.C_f
        # Calculation of the total grid-side impedance
        L_t = par.L_fg + par.L_g
        R_t = par.R_fg + par.R_g
        # Grid current dynamics
        d_i_gs = (state.u_fs - inp.e_gs - R_t*state.i_gs)/L_t

        return [d_i_cs, d_u_fs, d_i_gs]

    def post_process_with_inputs(self):
        """Post-process data with inputs."""
        data, par = self.data, self.par
        data.u_gs = (
            par.L_fg*data.e_gs + par.L_g*data.u_fs +
            (par.R_g*par.L_fg - par.R_fg*par.L_g)*data.i_gs)/(
                par.L_g + par.L_fg)


# %%
class LCFilter(ACFilter):
    """
    LC-filter model.

    Parameters
    ----------
    filter_pars : FilterPars
        Filter parameters. Machine drive LC-filter uses the following parameters:
    
            filter_pars.L_fc : float
                Converter-side inductance of the filter (H).
            filter_pars.C_f : float
                Filter capacitance (F).
            filter_pars.G_f : float, optional
                Filter conductance (S). The default is 0.
            filter_pars.R_fc : float, optional
                Converter-side series resistance (Ω). The default is 0.
   
    """

    def __init__(self, filter_par: FilterPars):
        super().__init__()
        self.par = SimpleNamespace(
            L_fc=filter_par.L_fc,
            C_f=filter_par.C_f,
            R_fc=filter_par.R_fc,
            G_f=filter_par.G_f,
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
        d_i_cs = (inp.u_cs - state.u_fs - par.R_fc*state.i_cs)/par.L_fc
        d_u_fs = (state.i_cs - inp.i_fs - par.G_f*state.u_fs)/par.C_f

        return [d_i_cs, d_u_fs]
