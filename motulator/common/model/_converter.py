"""
Continuous-time models for converters and DC buses.

A three-phase six-pulse diode bridge and a three-phase voltage-source converter are
modeled, along with a capacitive DC bus that can be shared by several converters.
Complex space vectors are used also for duty ratios and switching states, wherever
applicable.

"""

from collections.abc import Callable, Sequence
from dataclasses import InitVar, dataclass, field
from typing import Any

import numpy as np

from motulator.common.model._ac_port import (
    ACPort,
    diode_margin,
    leg_voltages,
    update_diodes,
)
from motulator.common.model._base import Converter, Subsystem, SubsystemTimeSeries
from motulator.common.model._sensor import Sensor
from motulator.common.utils._utils import (
    abc2complex,
    complex2abc,
    complex2line,
    empty_array,
    get_value,
)


# %%
@dataclass
class DiodeBridgeInputs:
    """Input variables."""

    i_c_ab: complex = 0j
    u_dc: float | None = None  # DC-bus voltage (V), constant without a DC bus


@dataclass
class Outputs:
    """Output variables for interconnection."""

    u_c_ab: complex
    u_dc: float


class DiodeBridge(Subsystem):
    """
    Three-phase six-pulse diode bridge.

    The diodes conduct according to the currents of the load `ac_load`: a leg carrying
    current is clamped to a rail, and a leg without current floats at the voltage
    keeping its current at zero. The voltage is computed from the AC port of the load,
    see `ACPort`. The diode states are updated at their transitions, which the
    simulation locates as events, see `Simulation`. Hence, the commutation of the
    current between the legs follows from the inductance of the load, e.g., an AC
    filter and the grid. The threshold voltage of the diodes is included in the diode
    drops. The DC-bus voltage is the input `u_dc`, which is constant unless a DC-bus
    model is connected to it (see `DCBus` and `Model`).

    Parameters
    ----------
    u_dc : float, optional
        Constant DC-bus voltage (V). Defaults to None, in which case the bridge must be
        fed from a DC-bus model.
    u_th : float, optional
        Threshold voltage (V) of the diodes, defaults to 0.

    """

    def __init__(self, u_dc: float | None = None, u_th: float = 0.0) -> None:
        self.u_th = u_th
        # The inputs are extended by `VoltageSourceConverter`, so their type is not
        # declared here
        self.inp = DiodeBridgeInputs(u_dc=u_dc)
        self.out: Outputs = Outputs(u_c_ab=0j, u_dc=0.0 if u_dc is None else u_dc)
        self.state = None
        self.ac_load: Subsystem | None = None  # Set by the model
        # Diode states of the legs, latched from the current directions at the first
        # evaluation, see `update_diodes`
        self._diodes: np.ndarray | None = None

    @property
    def blocked(self) -> bool:
        """Only the diodes conduct, as the bridge has no switches."""
        return True

    @property
    def margin(self) -> float:
        """Margin of the diode states at the latest evaluation, see `diode_margin`."""
        return diode_margin(*self._latest)

    def _ac_port(self) -> ACPort:
        """Get the AC port of the load."""
        port = None if self.ac_load is None else self.ac_load.compute_ac_port()
        if port is None:
            raise ValueError("The diodes require a load with an AC port")
        return port

    def _rail_voltage(self) -> float:
        """Voltage (V) between the rails, including the drops of the diodes."""
        return self.out.u_dc + 2 * self.u_th

    def update_diodes(self) -> None:
        """Update the diode states at a transition, see `update_diodes`."""
        self._diodes = update_diodes(
            self._ac_port(), self._rail_voltage(), self._diodes
        )

    def compute_internal_dc_current(self, inp: Any) -> Any:
        """Compute the internal DC current (A), rectified by the diodes."""
        return -0.5 * np.sum(np.abs(complex2abc(inp.i_c_ab)), axis=0)

    def _set_dc_voltage(self) -> None:
        """Set the DC-bus voltage from the input."""
        if self.inp.u_dc is None:
            raise ValueError(
                "The DC-bus voltage must be given, unless the converter is fed from a "
                "DC-bus model"
            )
        self.out.u_dc = self.inp.u_dc

    def set_outputs(self, t: float) -> None:
        """Set output variables."""
        self._set_dc_voltage()
        port, u_dc = self._ac_port(), self._rail_voltage()
        if self._diodes is None:
            self._diodes = update_diodes(port, u_dc)
        u_abc = leg_voltages(port, u_dc, self._diodes)
        self.out.u_c_ab = abc2complex(u_abc)
        # The margin is computed on demand, only for locating the transitions
        self._latest = (port, u_dc, self._diodes, u_abc)

    def rhs(self, t: float) -> list[complex]:
        """Default empty implementation."""
        return []

    def init_state_history(self) -> None:
        """Initialize the history of the voltage, which is evaluated by the model."""
        super().init_state_history()
        self._history["u_c_ab"] = []

    def extend_history(self, n: int, u_c_ab: Sequence[complex] | None) -> None:
        """
        Extend the history over the solution points of an integration interval.

        Parameters
        ----------
        n : int
            Number of solution points.
        u_c_ab : Sequence[complex] | None
            Voltages (V) evaluated at the solution points if only the diodes conduct,
            otherwise None.

        """
        self._history["u_c_ab"].extend([np.nan] * n if u_c_ab is None else u_c_ab)

    def create_time_series(self, t: np.ndarray) -> tuple[str, "DiodeBridgeTimeSeries"]:
        """Create time series."""
        return "converter", DiodeBridgeTimeSeries(t, self)


@dataclass
class DiodeBridgeTimeSeries[T: DiodeBridge](SubsystemTimeSeries):
    """Continuous time series."""

    t: InitVar[np.ndarray]
    subsystem: InitVar[T]
    u_dc: np.ndarray = field(default_factory=empty_array)
    i_c_ab: np.ndarray = field(default_factory=empty_array)
    u_c_ab: np.ndarray = field(default_factory=empty_array)
    i_dc_int: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: T) -> None:
        # Constant DC-bus voltage, replaced by the DC-bus time series if connected
        self.u_dc = np.full(np.size(t), subsystem.inp.u_dc, dtype=float)
        self.u_c_ab = np.array(subsystem._history["u_c_ab"], dtype=complex)

    def compute_derived_signals(self, t: np.ndarray, subsystem: T) -> None:
        """Compute the internal DC current."""
        self.i_dc_int = subsystem.compute_internal_dc_current(self)


# %%
@dataclass
class VoltageSourceConverterInputs(DiodeBridgeInputs):
    """Input variables."""

    q_c_ab: complex = 0j  # Switching state
    s_c_ab: complex = 0j  # Current-direction vector at the latest switching instant
    enabled: bool = True  # Gate enable


class VoltageSourceConverter(DiodeBridge):
    """
    Three-phase voltage-source converter.

    The switches are ideal, except for the dead time and the threshold voltage. When a
    leg is blanked, i.e., neither of its switches conducts, the current direction
    determines which diode conducts. The leg state is then `(1 - sign(i))/2`, where
    `i` is the phase current in the beginning of the blanking interval. The threshold
    voltage of the conducting device, assumed equal for the switches and the diodes,
    reduces each phase voltage by `u_th*sign(i)`, where the current direction is also
    determined at the latest switching instant. The DC-bus voltage is the input
    `u_dc`, which is constant unless a DC-bus model is connected to it (see `DCBus`
    and `Model`).

    When the gates are blocked (`inp.enabled` is False), all the switches are open, and
    the converter operates as a diode bridge, see `DiodeBridge`. The diode states are
    latched from the current directions when the gates are blocked.

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
        super().__init__(u_dc, u_th)
        self.t_d = t_d
        self.sign = sign
        self.u_dc_sensor = u_dc_sensor
        self.inp: VoltageSourceConverterInputs = VoltageSourceConverterInputs(u_dc=u_dc)

    @property
    def blocked(self) -> bool:
        """Only the diodes conduct while the gates are blocked."""
        return not self.inp.enabled

    def set_enable(self, enabled: bool) -> None:
        """Set the gate enable."""
        if enabled:
            self._diodes = None  # Latched again when the gates are blocked
        self.inp.enabled = enabled

    def set_gate_signals(self, q_abc: np.ndarray, b_abc: np.ndarray) -> None:
        """Set the switching state based on the gate and blanking signals."""
        s_abc = self.sign(complex2abc(self.inp.i_c_ab))
        self.inp.q_c_ab = abc2complex(q_abc + 0.5 * b_abc * (1 - s_abc))
        self.inp.s_c_ab = abc2complex(s_abc)

    def compute_internal_dc_current(self, inp: Any) -> Any:
        """Compute the internal DC current (A)."""
        i_dc = 1.5 * (inp.q_c_ab * inp.i_c_ab.conjugate()).real
        if inp.enabled is True:  # A single state, in the simulation
            return i_dc
        return np.where(inp.enabled, i_dc, super().compute_internal_dc_current(inp))

    def set_outputs(self, t: float) -> None:
        """Set output variables."""
        if self.blocked:
            super().set_outputs(t)
        else:
            self._set_dc_voltage()
            inp = self.inp
            self.out.u_c_ab = inp.q_c_ab * self.out.u_dc - self.u_th * inp.s_c_ab

    def meas_dc_voltage(self) -> float:
        """Measure converter DC-bus voltage (V)."""
        if self.u_dc_sensor is None:
            return self.out.u_dc
        return float(self.u_dc_sensor(self.out.u_dc))

    def meas_voltages(self) -> np.ndarray:
        """Measure the converter line-to-line voltages u_ab and u_bc (V)."""
        return complex2line(self.out.u_c_ab)

    def init_state_history(self) -> None:
        """Initialize the history of the switching state and the voltage."""
        super().init_state_history()
        self._history |= {"q_c_ab": [], "s_c_ab": [], "enabled": []}

    def extend_history(self, n: int, u_c_ab: Sequence[complex] | None) -> None:
        """Extend the history, the switching state held constant, see `DiodeBridge`."""
        super().extend_history(n, u_c_ab)
        inp = self.inp
        self._history["q_c_ab"].extend([inp.q_c_ab] * n)
        self._history["s_c_ab"].extend([inp.s_c_ab] * n)
        self._history["enabled"].extend([inp.enabled] * n)

    def create_time_series(
        self, t: np.ndarray
    ) -> tuple[str, "VoltageSourceConverterTimeSeries"]:
        """Create time series."""
        return "converter", VoltageSourceConverterTimeSeries(t, self)


@dataclass
class VoltageSourceConverterTimeSeries(DiodeBridgeTimeSeries[VoltageSourceConverter]):
    """Continuous time series."""

    subsystem: InitVar[VoltageSourceConverter]
    q_c_ab: np.ndarray = field(default_factory=empty_array)
    s_c_ab: np.ndarray = field(default_factory=empty_array)
    enabled: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: VoltageSourceConverter) -> None:
        super().__post_init__(t, subsystem)
        self.q_c_ab = np.array(subsystem._history["q_c_ab"])
        self.s_c_ab = np.array(subsystem._history["s_c_ab"])
        self.enabled = np.array(subsystem._history["enabled"])

    def set_outputs(self, t: np.ndarray, subsystem: VoltageSourceConverter) -> None:
        """Compute the voltage, which is saved by the model with blocked gates."""
        u_c_ab = self.q_c_ab * self.u_dc - subsystem.u_th * self.s_c_ab
        self.u_c_ab = np.where(self.enabled, u_c_ab, self.u_c_ab)


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

    The DC bus feeds the converters of the models it is given to, see `Model`. The
    models connect the DC-bus voltage to the input `u_dc` of their converters and
    register the converters in `converters`, whose DC currents the DC bus computes from
    their switching states and AC currents. The DC bus can be shared by the converters
    of several models, see `CompositeModel`. An external current, e.g., from a battery
    or a braking chopper, can also be fed to the DC bus.

    Parameters
    ----------
    C_dc : float
        DC-bus capacitance (F).
    u_dc : float
        Initial DC-bus voltage (V).

    """

    def __init__(self, C_dc: float, u_dc: float) -> None:
        self.C_dc = C_dc
        self.converters: list[Converter] = []  # Registered by the models
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
class DCBusTimeSeries(SubsystemTimeSeries):
    """Continuous time series."""

    t: InitVar[np.ndarray]
    subsystem: InitVar[DCBus]
    u_dc: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: DCBus) -> None:
        self.u_dc = np.real(np.array(subsystem._history["u_dc"]))
