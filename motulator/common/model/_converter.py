"""
Continuous-time models for converters and DC buses.

A three-phase voltage-source converter is modeled, along with a capacitive DC bus that
can be shared by several converters and a DC bus fed from a six-pulse diode bridge
rectifier supplied from a stiff grid. Complex space vectors are used also for duty
ratios and switching states, wherever applicable.

"""

from collections.abc import Callable, Sequence
from dataclasses import InitVar, dataclass, field
from math import sqrt
from typing import Any

import numpy as np

from motulator.common.model._base import Subsystem, SubsystemTimeSeries
from motulator.common.model._sensor import Sensor
from motulator.common.utils._utils import (
    abc2complex,
    complex2abc,
    empty_array,
    get_value,
)


# %%
@dataclass
class Inputs:
    """Input variables."""

    q_c_ab: complex = 0j  # Switching state
    s_c_ab: complex = 0j  # Current-direction vector at the latest switching instant
    i_c_ab: complex = 0j
    u_dc: float | None = None  # DC-bus voltage (V) from a DC-bus model, if connected


@dataclass
class Outputs:
    """Output variables for interconnection."""

    u_c_ab: complex
    u_dc: float


class VoltageSourceConverter(Subsystem):
    """
    Three-phase voltage-source converter.

    The switches are ideal, except for the dead time and the threshold voltage. When a
    leg is blanked, i.e., neither of its switches conducts, the current direction
    determines which diode conducts. The leg state is then `(1 - sign(i))/2`, where
    `i` is the phase current in the beginning of the blanking interval. The threshold
    voltage of the conducting device, assumed equal for the switches and the diodes,
    reduces each phase voltage by `u_th*sign(i)`, where the current direction is also
    determined at the latest switching instant. The DC-bus voltage is either constant
    or given by a DC-bus model connected to the input `u_dc` (see `DCBus` and
    `Model`).

    Parameters
    ----------
    u_dc : float, optional
        Constant DC-bus voltage (V). Defaults to None, in which case the converter must
        be fed from a DC-bus model.
    t_d : float, optional
        Dead time (s), defaults to 0.
    u_th : float, optional
        Threshold voltage (V) of the power devices, defaults to 0.
    sign : Callable[[np.ndarray], np.ndarray], optional
        Function of the phase currents (A) determining the leg states during blanking,
        defaults to `np.sign`. A smooth function, such as `2/pi*arctan(i/i_d)`, can be
        used to model the effect of parasitic capacitances, for example.
    u_dc_sensor : Sensor, optional
        DC-bus voltage sensor, defaults to an ideal measurement.

    """

    n_legs: int = 3

    def __init__(
        self,
        u_dc: float | None = None,
        t_d: float = 0.0,
        u_th: float = 0.0,
        sign: Callable[[np.ndarray], np.ndarray] = np.sign,
        u_dc_sensor: Sensor | None = None,
    ) -> None:
        self.u_dc = u_dc
        self.t_d = t_d
        self.u_th = u_th
        self.sign = sign
        self.u_dc_sensor = u_dc_sensor
        self.inp: Inputs = Inputs()
        self.out: Outputs = Outputs(u_c_ab=0j, u_dc=0.0 if u_dc is None else u_dc)
        self.state = None

    def set_gate_signals(self, q_abc: np.ndarray, b_abc: np.ndarray) -> None:
        """Set the switching state based on the gate and blanking signals."""
        s_abc = self.sign(complex2abc(self.inp.i_c_ab))
        self.inp.q_c_ab = abc2complex(q_abc + 0.5 * b_abc * (1 - s_abc))
        self.inp.s_c_ab = abc2complex(s_abc)

    def compute_internal_dc_current(self, inp: Any) -> Any:
        """Compute the internal DC current (A)."""
        return 1.5 * np.real(inp.q_c_ab * np.conj(inp.i_c_ab))

    def set_outputs(self, t: float) -> None:
        """Set output variables."""
        if self.inp.u_dc is not None:
            self.out.u_dc = self.inp.u_dc
        elif self.u_dc is None:
            raise ValueError(
                "The DC-bus voltage must be given, unless the converter is fed from a "
                "DC-bus model"
            )
        self.out.u_c_ab = self.inp.q_c_ab * self.out.u_dc - self.u_th * self.inp.s_c_ab

    def meas_dc_voltage(self) -> float:
        """Measure converter DC-bus voltage (V)."""
        if self.u_dc_sensor is None:
            return self.out.u_dc
        return float(self.u_dc_sensor(self.out.u_dc))

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
    s_c_ab: np.ndarray = field(default_factory=empty_array)
    i_c_ab: np.ndarray = field(default_factory=empty_array)
    u_c_ab: np.ndarray = field(default_factory=empty_array)
    i_dc_int: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: T) -> None:
        # Constant DC-bus voltage, replaced by the DC-bus model time series if connected
        u_dc = np.nan if subsystem.u_dc is None else subsystem.u_dc
        self.u_dc = np.full(np.size(t), u_dc)

    def compute_zoh_input_derived_signals(self, t: np.ndarray, subsystem: T) -> None:
        """Compute zero-order hold derived signals."""
        self.u_c_ab = self.q_c_ab * self.u_dc - subsystem.u_th * self.s_c_ab

    def compute_input_derived_signals(self, t: np.ndarray, subsystem: T) -> None:
        """Process input time series."""
        self.i_dc_int = subsystem.compute_internal_dc_current(self)


# %%
@dataclass
class DCBusInputs:
    """Input variables."""

    i_dc: float | Callable[[float], float] | None = None  # External current (A)


@dataclass
class DCBusOutputs:
    """Output variables for interconnection."""

    u_dc: float


@dataclass
class DCBusStates:
    """State variables."""

    u_dc: float


class DCBus(Subsystem):
    """
    Capacitive DC bus feeding converters.

    The DC bus feeds the given converters, whose DC currents are computed from their
    switching states and AC currents. The DC-bus voltage is connected to the input
    `u_dc` of each converter by the model (see `Model`). The DC bus can be shared by
    the converters of several models (see `CompositeModel`). An external current,
    e.g., from a battery or a braking chopper, can also be fed to the DC bus.

    Parameters
    ----------
    C_dc : float
        DC-bus capacitance (F).
    u_dc : float
        Initial DC-bus voltage (V).
    converters : Sequence[VoltageSourceConverter]
        Converters fed from the DC bus.

    """

    def __init__(
        self, C_dc: float, u_dc: float, converters: Sequence[VoltageSourceConverter]
    ) -> None:
        if len({id(c) for c in converters}) < len(converters):
            raise ValueError("The converters must be distinct")
        self.C_dc = C_dc
        self.converters = list(converters)
        self.inp: DCBusInputs = DCBusInputs()
        self.out: DCBusOutputs = DCBusOutputs(u_dc)
        self.state: DCBusStates = DCBusStates(u_dc)

    def set_external_dc_current(self, i_dc: float | Callable[[float], float]) -> None:
        """Set external DC current (A), constant or a function of time."""
        self.inp.i_dc = i_dc

    def compute_dc_current(self, t: float) -> float:
        """Compute the net current (A) fed to the DC-bus capacitor."""
        i_dc = 0.0 if self.inp.i_dc is None else get_value(self.inp.i_dc, t)
        return i_dc - sum(c.compute_internal_dc_current(c.inp) for c in self.converters)

    def set_outputs(self, t: float) -> None:
        """Set output variables for interconnection."""
        self.out.u_dc = self.state.u_dc.real

    def rhs(self, t: float) -> list[complex]:
        """Compute state derivatives."""
        return [self.compute_dc_current(t) / self.C_dc]

    def create_time_series(self, t: np.ndarray) -> tuple[str, "DCBusTimeSeries"]:
        """Create time series from state list."""
        return "dc_bus", DCBusTimeSeries(t, self)


@dataclass
class DCBusTimeSeries[T: DCBus](SubsystemTimeSeries):
    """Continuous time series."""

    t: InitVar[np.ndarray]
    subsystem: InitVar[T]
    u_dc: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: T) -> None:
        self.u_dc = np.real(np.array(subsystem._history["u_dc"]))


# %%
@dataclass
class DiodeBridgeDCBusStates(DCBusStates):
    """State variables."""

    i_L: complex = 0j  # Imaginary part is always zero
    exp_j_theta_g: complex = complex(1)


class DiodeBridgeDCBus(DCBus):
    """
    DC bus fed from a six-pulse diode bridge.

    A three-phase diode bridge rectifier with a DC-bus inductor is modeled, supplied
    from a stiff grid, i.e., the grid inductance is zero. The DC bus feeds the given
    converters, see `DCBus`.

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
    converters : Sequence[VoltageSourceConverter]
        Converters fed from the DC bus.

    """

    def __init__(
        self,
        C_dc: float,
        L_dc: float,
        U_g: float,
        f_g: float,
        converters: Sequence[VoltageSourceConverter],
    ) -> None:
        u_dc = sqrt(2) * U_g
        super().__init__(C_dc, u_dc, converters)
        self.L_dc = L_dc
        self.w_g = 2 * np.pi * f_g
        self.u_g = sqrt(2 / 3) * U_g
        self.state = DiodeBridgeDCBusStates(u_dc)

    def compute_voltages(self, state: Any) -> tuple[Any, Any]:
        """Compute grid and rectified voltages."""
        # Grid voltage
        u_g_ab = self.u_g * state.exp_j_theta_g
        u_g_abc = complex2abc(u_g_ab)
        # Output voltage of the diode bridge
        u_di = np.ptp(u_g_abc, axis=0)
        return u_g_ab, u_di

    def compute_derivatives(self, t: float, state: Any) -> list[complex]:
        """Compute the state derivatives."""
        _, u_di = self.compute_voltages(state)
        d_u_dc = (state.i_L.real + self.compute_dc_current(t)) / self.C_dc
        d_i_L = (u_di - state.u_dc.real) / self.L_dc
        d_exp_j_theta_g = 1j * self.w_g * state.exp_j_theta_g
        # Inductor current cannot be negative due to the diode bridge
        if state.i_L.real < 0 and d_i_L < 0:
            d_i_L = 0
        return [d_u_dc, d_i_L, d_exp_j_theta_g]

    def rhs(self, t: float) -> list[complex]:
        """Compute state derivatives."""
        return self.compute_derivatives(t, self.state)

    def create_time_series(
        self, t: np.ndarray
    ) -> tuple[str, "DiodeBridgeDCBusTimeSeries"]:
        """Create time series from state list."""
        return "dc_bus", DiodeBridgeDCBusTimeSeries(t, self)


@dataclass
class DiodeBridgeDCBusTimeSeries(DCBusTimeSeries[DiodeBridgeDCBus]):
    """Continuous time series."""

    subsystem: InitVar[DiodeBridgeDCBus]
    # State variables
    i_L: np.ndarray = field(default_factory=empty_array)
    exp_j_theta_g: np.ndarray = field(default_factory=empty_array)
    # Derived signals
    u_g_ab: np.ndarray = field(default_factory=empty_array)
    u_di: np.ndarray = field(default_factory=empty_array)
    u_g_abc: np.ndarray = field(default_factory=empty_array)
    q_g_abc: np.ndarray = field(default_factory=empty_array)
    i_g_ab: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: DiodeBridgeDCBus) -> None:
        super().__post_init__(t, subsystem)
        self.i_L = np.real(np.array(subsystem._history["i_L"]))
        self.exp_j_theta_g = np.array(subsystem._history["exp_j_theta_g"])
        self.u_g_ab, self.u_di = subsystem.compute_voltages(self)
        self.u_g_abc = complex2abc(self.u_g_ab)
        # Diode bridge switching states (-1, 0, 1)
        self.q_g_abc = (self.u_g_abc.max(axis=0) == self.u_g_abc).astype(int) - (
            self.u_g_abc.min(axis=0) == self.u_g_abc
        ).astype(int)
        # Grid current space vector
        self.i_g_ab = abc2complex(self.q_g_abc) * self.i_L
