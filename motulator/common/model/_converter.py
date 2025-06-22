"""
Continuous-time models for converters.

A three-phase voltage-source inverter with optional DC-bus dynamics is modeled, along
with a six-pulse diode bridge rectifier supplied from a stiff grid. Complex space
vectors are used also for duty ratios and switching states, wherever applicable.

"""

from dataclasses import InitVar, dataclass, field
from math import sqrt
from typing import Any, Callable

import numpy as np

from motulator.common.model._base import Subsystem
from motulator.common.utils._utils import abc2complex, complex2abc, empty_array


# %%
@dataclass
class Inputs:
    """Input variables."""

    q_c_ab: complex = 0j
    i_c_ab: complex = 0j
    i_dc: float | Callable[[float], float] | None = None


@dataclass
class Outputs:
    """Output variables for interconnection."""

    u_c_ab: complex
    u_dc: float


class VoltageSourceConverter(Subsystem):
    """
    Lossless three-phase voltage-source converter with constant DC-bus voltage.

    Parameters
    ----------
    u_dc : float
        DC-bus voltage (V).

    """

    def __init__(self, u_dc: float) -> None:
        self.u_dc = u_dc
        self.inp: Inputs = Inputs()
        self.out: Outputs = Outputs(u_c_ab=0j, u_dc=u_dc)
        self.state = None
        self._history = None

    def set_external_dc_current(self, i_dc: Callable[[float], float]) -> None:
        """Set external DC current (A)."""
        raise NotImplementedError

    def compute_internal_dc_current(self, inp: Any) -> Any:
        """Compute the internal DC current (A)."""
        return 1.5 * np.real(inp.q_c_ab * np.conj(inp.i_c_ab))

    def set_outputs(self, t: float) -> None:
        """Set output variables."""
        self.out.u_c_ab = self.inp.q_c_ab * self.out.u_dc

    def meas_dc_voltage(self) -> float:
        """Measure converter DC-bus voltage (V)."""
        return self.out.u_dc

    def rhs(self, t: float) -> list[complex]:
        """Default empty implementation."""
        return []

    def create_time_series(
        self, t: np.ndarray
    ) -> tuple[str, "VoltageSourceConverterTimeSeries"]:
        """Create time series."""
        return "converter", VoltageSourceConverterTimeSeries(t, self)


@dataclass
class VoltageSourceConverterTimeSeries[T: VoltageSourceConverter]:
    """Continuous time series."""

    t: InitVar[np.ndarray]
    subsystem: InitVar[T]
    u_dc: np.ndarray = field(default_factory=empty_array)
    q_c_ab: np.ndarray = field(default_factory=empty_array)
    i_c_ab: np.ndarray = field(default_factory=empty_array)
    u_c_ab: np.ndarray = field(default_factory=empty_array)
    i_dc_int: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: T) -> None:
        self.u_dc = np.full(np.size(t), subsystem.u_dc)

    def compute_zoh_input_derived_signals(self, t: np.ndarray, subsystem: T) -> None:
        """Compute zero-order hold derived signals."""
        self.u_c_ab = self.q_c_ab * self.u_dc

    def compute_input_derived_signals(self, t: np.ndarray, subsystem: T) -> None:
        """Process input time series."""
        self.i_dc_int = subsystem.compute_internal_dc_current(self)


# %%
@dataclass
class CapacitiveDCBusConverterStates:
    """State variables."""

    u_dc: float


@dataclass
class CapacitiveDCBusConverterStateHistory:
    """State history."""

    u_dc: list[complex] = field(default_factory=list)


class CapacitiveDCBusConverter(VoltageSourceConverter):
    """
    Lossless voltage-source converter with capacitive DC-bus dynamics.

    Parameters
    ----------
    u_dc : float
        DC-bus voltage (V).
    C_dc : float
        DC-bus capacitance (F).

    """

    def __init__(self, u_dc: float, C_dc: float) -> None:
        super().__init__(u_dc)
        self.C_dc = C_dc
        self.state: CapacitiveDCBusConverterStates = CapacitiveDCBusConverterStates(
            self.u_dc
        )
        self._history: CapacitiveDCBusConverterStateHistory = (
            CapacitiveDCBusConverterStateHistory()
        )

    def set_external_dc_current(self, i_dc: Callable[[float], float]) -> None:
        """Set external DC current (A)."""
        self.inp.i_dc = i_dc

    def set_outputs(self, t: float) -> None:
        """Set output variables for interconnection."""
        self.out.u_dc = self.state.u_dc.real
        super().set_outputs(t)

    def rhs(self, t: float) -> list[complex]:
        """Compute state derivatives for DC-bus voltage."""
        if callable(self.inp.i_dc):
            i_dc = self.inp.i_dc(t)
        elif isinstance(self.inp.i_dc, (int, float)):
            i_dc = self.inp.i_dc
        else:
            i_dc = 0.0
        i_dc_int = self.compute_internal_dc_current(self.inp)
        d_u_dc = (i_dc - i_dc_int) / self.C_dc
        return [d_u_dc]

    def create_time_series(
        self, t: np.ndarray
    ) -> tuple[str, "CapacitiveDCBusConverterTimeSeries"]:
        """Create time series from state list."""
        return "converter", CapacitiveDCBusConverterTimeSeries(t, self)


@dataclass
class CapacitiveDCBusConverterTimeSeries(
    VoltageSourceConverterTimeSeries[CapacitiveDCBusConverter]
):
    """Continuous time series."""

    subsystem: InitVar[CapacitiveDCBusConverter]

    def __post_init__(self, t: np.ndarray, subsystem: CapacitiveDCBusConverter) -> None:
        self.u_dc = np.array(subsystem._history.u_dc)


# %%


@dataclass
class FrequencyConverterStates:
    """State variables."""

    u_dc: complex  # Imaginary part is always zero
    i_L: complex = 0j  # Imaginary part is always zero
    exp_j_theta_g: complex = complex(1)


@dataclass
class FrequencyConverterStateHistory:
    """State history."""

    u_dc: list[complex] = field(default_factory=list)
    i_L: list[complex] = field(default_factory=list)
    exp_j_theta_g: list[complex] = field(default_factory=list)


class FrequencyConverter(VoltageSourceConverter):
    """
    Frequency converter with a six-pulse diode bridge.

    A three-phase diode bridge rectifier with a DC-bus inductor is modeled. The diode
    bridge is connected to the voltage-source inverter. The grid inductance is zero.

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

    def __init__(self, C_dc: float, L_dc: float, U_g: float, f_g: float) -> None:
        u_dc = sqrt(2) * U_g
        super().__init__(u_dc)
        self.C_dc = C_dc
        self.L_dc = L_dc
        self.w_g = 2 * np.pi * f_g
        self.u_g = sqrt(2 / 3) * U_g
        self.state: FrequencyConverterStates = FrequencyConverterStates(u_dc)
        self._history: FrequencyConverterStateHistory = FrequencyConverterStateHistory()

    def compute_voltages(self, state: Any) -> tuple[Any, Any]:
        """Compute grid and rectified voltages."""
        # Grid voltage
        u_g_ab = self.u_g * state.exp_j_theta_g
        u_g_abc = complex2abc(u_g_ab)
        # Output voltage of the diode bridge
        u_di = np.amax(u_g_abc, axis=0) - np.amin(u_g_abc, axis=0)
        return u_g_ab, u_di

    def set_outputs(self, t: float) -> None:
        """Set output variables for interconnection."""
        self.out.u_dc = self.state.u_dc.real
        super().set_outputs(t)

    def rhs(self, t: float) -> list[complex]:
        """Compute state derivatives."""
        # Rectified voltage and internal DC current
        _, u_di = self.compute_voltages(self.state)
        i_dc_int = self.compute_internal_dc_current(self.inp)
        # State derivatives
        d_u_dc = (self.state.i_L.real - i_dc_int) / self.C_dc
        d_i_L = (u_di - self.state.u_dc.real) / self.L_dc
        d_exp_j_theta_g = 1j * self.w_g * self.state.exp_j_theta_g
        # Inductor current cannot be negative due to the diode bridge
        if self.state.i_L.real < 0 and d_i_L < 0:
            d_i_L = 0
        return [d_u_dc, d_i_L, d_exp_j_theta_g]

    def create_time_series(
        self, t: np.ndarray
    ) -> tuple[str, "FrequencyConverterTimeSeries"]:
        """Time series."""
        return "converter", FrequencyConverterTimeSeries(t, self)


@dataclass
class FrequencyConverterTimeSeries(
    VoltageSourceConverterTimeSeries[FrequencyConverter]
):
    """Continuous time series."""

    subsystem: InitVar[FrequencyConverter]

    # State variables
    i_L: np.ndarray = field(default_factory=empty_array)
    exp_j_theta_g: np.ndarray = field(default_factory=empty_array)
    # Derived signals
    u_g_ab: np.ndarray = field(default_factory=empty_array)
    u_di: np.ndarray = field(default_factory=empty_array)
    u_g_abc: np.ndarray = field(default_factory=empty_array)
    q_g_abc: np.ndarray = field(default_factory=empty_array)
    i_g_ab: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: FrequencyConverter) -> None:
        self.u_dc = np.real(np.array(subsystem._history.u_dc))
        self.i_L = np.real(np.array(subsystem._history.i_L))
        self.exp_j_theta_g = np.array(subsystem._history.exp_j_theta_g)
        self.u_g_ab, self.u_di = subsystem.compute_voltages(self)
        self.u_g_abc = complex2abc(self.u_g_ab)
        # Diode bridge switching states (-1, 0, 1)
        self.q_g_abc = (np.amax(self.u_g_abc, axis=0) == self.u_g_abc).astype(int) - (
            np.amin(self.u_g_abc, axis=0) == self.u_g_abc
        ).astype(int)
        # Grid current space vector
        self.i_g_ab = abc2complex(self.q_g_abc) * self.i_L
