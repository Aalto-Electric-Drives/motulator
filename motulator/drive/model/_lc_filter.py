"""Continuous-time model for an output LC filter."""

from dataclasses import InitVar, dataclass, field
from typing import Any

import numpy as np

from motulator.common.model import Subsystem, SubsystemTimeSeries
from motulator.common.utils import complex2abc, empty_array


# %%
@dataclass
class Inputs:
    """Input variables."""

    u_c_ab: complex = 0j
    i_f_ab: complex = 0j


@dataclass
class Outputs:
    """Output variables for interconnection."""

    i_c_ab: complex
    u_f_ab: complex


@dataclass
class States:
    """State variables."""

    i_c_ab: complex = 0j
    u_f_ab: complex = 0j


@dataclass
class StateHistory:
    """State history."""

    i_c_ab: list[complex] = field(default_factory=list)
    u_f_ab: list[complex] = field(default_factory=list)


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
        Series resistance (Ω) of the inductor, defaults to 0.

    """

    def __init__(self, L_f: float, C_f: float, R_f: float = 0.0) -> None:
        self.L_f = L_f
        self.C_f = C_f
        self.R_f = R_f
        self.state: States = States()
        self.inp: Inputs = Inputs()
        self.out: Outputs = Outputs(self.state.i_c_ab, self.state.u_f_ab)
        self._history: StateHistory = StateHistory()

    def set_outputs(self, t: float) -> None:
        """Set output variables."""
        self.out.i_c_ab = self.state.i_c_ab
        self.out.u_f_ab = self.state.u_f_ab

    def rhs(self, t: float) -> list[complex]:
        """Compute state derivatives."""
        state = self.state
        inp = self.inp

        d_i_c_ab = (inp.u_c_ab - state.u_f_ab - self.R_f * state.i_c_ab) / self.L_f
        d_u_f_ab = (state.i_c_ab - inp.i_f_ab) / self.C_f

        return [d_i_c_ab, d_u_f_ab]

    def meas_currents(self) -> Any:
        """Measure the converter phase currents."""
        return complex2abc(self.out.i_c_ab)

    def meas_capacitor_voltages(self) -> Any:
        """Measure the capacitor phase voltages."""
        return complex2abc(self.out.u_f_ab)

    def create_time_series(self, t: np.ndarray) -> tuple[str, "LCFilterTimeSeries"]:
        """Create time series from state list."""
        return "lc_filter", LCFilterTimeSeries(t, self)


@dataclass
class LCFilterTimeSeries(SubsystemTimeSeries):
    """Continuous-time series."""

    t: InitVar[np.ndarray]
    subsystem: InitVar[LCFilter]
    i_c_ab: np.ndarray = field(default_factory=empty_array)
    u_f_ab: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: LCFilter) -> None:
        """Compute output time series from the states."""
        self.i_c_ab = np.array(subsystem._history.i_c_ab)
        self.u_f_ab = np.array(subsystem._history.u_f_ab)
