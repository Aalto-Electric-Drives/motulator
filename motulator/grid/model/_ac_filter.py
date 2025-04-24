"""
AC filter and grid impedance models.

This module contains continuous-time models for subsystems comprising an AC filter and a
grid impedance between the converter and grid voltage sources. The models are
implemented with space vectors in stationary coordinates.

"""

from dataclasses import InitVar, dataclass, field
from typing import Any

import numpy as np

from motulator.common.model import Subsystem, SubsystemTimeSeries
from motulator.common.utils import complex2abc, empty_array


# %%
@dataclass
class Inputs:
    """AC filter inputs."""

    u_c_ab: complex = 0j
    e_g_ab: complex = 0j


# %%
@dataclass
class LFilterOutputs:
    """Base class for outputs."""

    i_c_ab: complex


@dataclass
class LFilterStates:
    """State variables."""

    i_c_ab: complex = 0j


@dataclass
class LFilterStateHistory:
    """State history."""

    i_c_ab: list[complex] = field(default_factory=list)


class LFilter(Subsystem):
    """
    Model of an L filter and an inductive-resistive grid.

    An L filter and an inductive-resistive grid, between the converter and grid voltage
    sources, are modeled combining their inductances and series resistances. The point-
    of-common-coupling (PCC) voltage between the L filter and the grid impedance is
    calculated.

    Parameters
    ----------
    L_f : float
        Filter inductance (H).
    R_f : float, optional
        Series resistance (Ω) of the filter inductor, defaults to 0.
    L_g : float, optional
        Grid inductance (H), defaults to 0.
    R_g : float, optional
        Grid resistance (Ω), defaults to 0.

    """

    def __init__(
        self, L_f: float, R_f: float = 0.0, L_g: float = 0.0, R_g: float = 0.0
    ) -> None:
        self.L_f = L_f
        self.R_f = R_f
        self.L_g = L_g
        self.R_g = R_g
        # The following initial conditions are needed for computing the PCC voltage,
        # which has direct feedthrough. The PCC voltage is used only as a feedback
        # signal for the control system (but not coupled to the solution of the
        # continuous-time system). If the transients during the very first time steps
        # are important, these initial conditions can be set to appropriate values.
        self.inp: Inputs = Inputs()
        self.state: LFilterStates = LFilterStates()
        self.out: LFilterOutputs = LFilterOutputs(self.state.i_c_ab)
        self._history: LFilterStateHistory = LFilterStateHistory()

    def pcc_voltage(self, state: Any, inp: Any) -> Any:
        """Compute the voltage at the point of common coupling (PCC)."""
        L_t = self.L_f + self.L_g
        u_g_ab = (
            self.L_g * (inp.u_c_ab - self.R_f * state.i_c_ab)
            + self.L_f * (inp.e_g_ab + self.R_g * state.i_c_ab)
        ) / L_t
        return u_g_ab

    def set_outputs(self, t: float) -> None:
        """Set output variables."""
        self.out.i_c_ab = self.state.i_c_ab

    def rhs(self, t: float) -> list[complex]:
        """Compute the state derivatives."""
        state = self.state
        inp = self.inp
        L_t = self.L_f + self.L_g
        R_t = self.R_f + self.R_g
        d_i_c_ab = (inp.u_c_ab - inp.e_g_ab - R_t * state.i_c_ab) / L_t
        return [d_i_c_ab]

    def meas_currents(self) -> Any:
        """Measure the converter phase currents (A)."""
        return complex2abc(self.state.i_c_ab)

    def meas_pcc_voltages(self) -> Any:
        """Measure the phase voltages (V) at the PCC."""
        u_g_ab = self.pcc_voltage(self.state, self.inp)
        return complex2abc(u_g_ab)

    def create_time_series(self, t: np.ndarray) -> tuple[str, "LFilterTimeSeries"]:
        """Create time series from state list."""
        return "ac_filter", LFilterTimeSeries(t, self)


@dataclass
class LFilterTimeSeries(SubsystemTimeSeries):
    """Continuous time series."""

    t: InitVar[np.ndarray]
    subsystem: InitVar[LFilter]
    # State
    i_c_ab: np.ndarray = field(default_factory=empty_array)
    # Inputs
    u_c_ab: np.ndarray = field(default_factory=empty_array)
    e_g_ab: np.ndarray = field(default_factory=empty_array)
    # Outputs
    u_g_ab: np.ndarray = field(default_factory=empty_array)
    i_g_ab: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: LFilter) -> None:
        """Compute output time series from the states."""
        self.i_c_ab = np.array(subsystem._history.i_c_ab)
        self.i_g_ab = self.i_c_ab

    def compute_input_derived_signals(self, t: np.ndarray, subsystem: LFilter) -> None:
        """Compute direct feedthrough time series."""
        self.u_g_ab = subsystem.pcc_voltage(self, self)


# %%
@dataclass
class LCLFilterStates:
    """LCL filter states."""

    i_c_ab: complex = 0j
    u_f_ab: complex = 0j
    i_g_ab: complex = 0j


@dataclass
class LCLFilterOutputs:
    """Base class for outputs."""

    i_c_ab: complex
    u_f_ab: complex
    i_g_ab: complex


@dataclass
class LCLFilterStateHistory:
    """LCL filter state history."""

    i_c_ab: list[complex] = field(default_factory=list)
    u_f_ab: list[complex] = field(default_factory=list)
    i_g_ab: list[complex] = field(default_factory=list)


class LCLFilter(Subsystem):
    """
    Model of an LCL filter and an inductive-resistive grid.

    An LCL filter and an inductive-resistive grid impedance, between the converter and
    grid voltage sources, are modeled. The point-of-common-coupling (PCC) voltage
    between the LCL filter and the grid impedance is also calculated.

    Parameters
    ----------
    L_fc : float
        Converter-side filter inductance (H).
    L_fg : float
        Grid-side filter inductance (H).
    C_f : float
        Filter capacitance (F).
    R_fc : float, optional
        Series resistance (Ω) of the converter-side inductor, defaults to 0.
    R_fg : float, optional
        Series resistance (Ω) of the grid-side inductor, defaults to 0.
    L_g : float, optional
        Grid inductance (H), defaults to 0.
    R_g : float, optional
        Grid resistance (Ω), defaults to 0.
    u_f0_ab : complex, optional
        Initial value of the filter capacitor voltage (V), defaults to 0.
    """

    def __init__(
        self,
        L_fc: float,
        L_fg: float,
        C_f: float,
        R_fc: float = 0.0,
        R_fg: float = 0.0,
        L_g: float = 0.0,
        R_g: float = 0.0,
        u_f0_ab: complex = 0j,
    ) -> None:
        self.L_fc = L_fc
        self.L_fg = L_fg
        self.C_f = C_f
        self.R_fc = R_fc
        self.R_fg = R_fg
        self.L_g = L_g
        self.R_g = R_g
        self.state: LCLFilterStates = LCLFilterStates(0j, u_f0_ab, 0j)
        # The following initial conditions are needed for computing the PCC voltage,
        # which has direct feedthrough. The PCC voltage is used only as a feedback
        # signal for the control system.
        self.inp: Inputs = Inputs(u_f0_ab, u_f0_ab)
        self.out: LCLFilterOutputs = LCLFilterOutputs(
            self.state.i_c_ab, self.state.u_f_ab, self.state.i_g_ab
        )
        self._history: LCLFilterStateHistory = LCLFilterStateHistory()

    def pcc_voltage(self, state: Any, inp: Any) -> Any:
        """Compute the voltage at the point of common coupling (PCC)."""
        L_t = self.L_fg + self.L_g
        u_g_ab = (
            self.L_fg * inp.e_g_ab
            + self.L_g * state.u_f_ab
            + (self.R_g * self.L_fg - self.R_fg * self.L_g) * state.i_g_ab
        ) / L_t
        return u_g_ab

    def set_outputs(self, t: float) -> None:
        """Set output variables."""
        state, out = self.state, self.out
        out.i_c_ab = state.i_c_ab
        out.u_f_ab = state.u_f_ab
        out.i_g_ab = state.i_g_ab

    def rhs(self, t: float) -> list[complex]:
        """Compute the state derivatives."""
        state = self.state
        inp = self.inp
        # Total inductance and resistance
        L_t = self.L_fg + self.L_g
        R_t = self.R_fg + self.R_g
        # State equations
        d_i_c_ab = (inp.u_c_ab - state.u_f_ab - self.R_fc * state.i_c_ab) / self.L_fc
        d_u_f_ab = (state.i_c_ab - state.i_g_ab) / self.C_f
        d_i_g_ab = (state.u_f_ab - inp.e_g_ab - R_t * state.i_g_ab) / L_t
        return [d_i_c_ab, d_u_f_ab, d_i_g_ab]

    def meas_currents(self) -> Any:
        """Measure the converter phase currents (A)."""
        return complex2abc(self.state.i_c_ab)

    def meas_pcc_voltages(self) -> Any:
        """Measure the phase voltages (V) at point of common coupling (PCC)."""
        u_g_ab = self.pcc_voltage(self.state, self.inp)
        return complex2abc(u_g_ab)

    def meas_grid_currents(self) -> Any:
        """Measure the grid phase currents (A)."""
        return complex2abc(self.state.i_g_ab)

    def meas_capacitor_voltages(self) -> Any:
        """Measure the capacitor phase voltages (V)."""
        return complex2abc(self.state.u_f_ab)

    def create_time_series(self, t: np.ndarray) -> tuple[str, "LCLFilterTimeSeries"]:
        """Create time series from state list."""
        return "ac_filter", LCLFilterTimeSeries(t, self)


@dataclass
class LCLFilterTimeSeries(SubsystemTimeSeries):
    """Continuous time series for AC filters."""

    t: InitVar[np.ndarray]
    subsystem: InitVar[LCLFilter]
    # States
    i_c_ab: np.ndarray = field(default_factory=empty_array)
    u_f_ab: np.ndarray = field(default_factory=empty_array)
    i_g_ab: np.ndarray = field(default_factory=empty_array)
    # Inputs
    u_c_ab: np.ndarray = field(default_factory=empty_array)
    e_g_ab: np.ndarray = field(default_factory=empty_array)
    # Outputs
    u_g_ab: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: LCLFilter) -> None:
        """Compute output time series from the states."""
        self.i_c_ab = np.array(subsystem._history.i_c_ab)
        self.i_g_ab = np.array(subsystem._history.i_g_ab)
        self.u_f_ab = np.array(subsystem._history.u_f_ab)

    def compute_input_derived_signals(
        self, t: np.ndarray, subsystem: LCLFilter
    ) -> None:
        """Process input time series."""
        self.u_g_ab = subsystem.pcc_voltage(self, self)
