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
    DCPort,
    current_direction,
    dc_voltage,
    diode_margin,
    leg_voltages,
    update_diodes,
)
from motulator.common.model._base import (
    ControlledConverter,
    Converter,
    Subsystem,
    SubsystemTimeSeries,
)
from motulator.common.model._pwm import (
    ZOH,
    GateStates,
    SwitchingTimes,
    TwoLevelModulator,
)
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
    i_dc: float = 0.0  # DC current (A) fed to the DC load of a current-fed bridge


class DiodeBridge(Converter):
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
    model is connected to it (see `DCBus` and `Model`). Alternatively, the bridge
    feeds a DC load `dc_load` (e.g., an EMF through a DC choke), whose DC port gives
    the relation between the DC current and the voltage, see `DCPort`. The bridge then
    solves its DC voltage together with the leg voltages, and the DC current is its
    output `i_dc`. The devices of the legs can be gated through `gated`, which models
    thyristors: a floating leg is clamped to a rail only by a gated device.

    Parameters
    ----------
    u_dc : float, optional
        Constant DC-bus voltage (V). Defaults to None, in which case the bridge must be
        fed from a DC-bus model or feed a DC load.
    u_th : float, optional
        Threshold voltage (V) of the diodes, defaults to 0.

    """

    # The converter voltage depends directly on the DC-bus voltage
    feedthrough = frozenset({"u_dc"})

    def __init__(self, u_dc: float | None = None, u_th: float = 0.0) -> None:
        self.u_th = u_th
        # The inputs are extended by `VoltageSourceConverter`, so their type is not
        # declared here
        self.inp = DiodeBridgeInputs(u_dc=u_dc)
        self.out: Outputs = Outputs(u_c_ab=0j, u_dc=0.0 if u_dc is None else u_dc)
        self.state = None
        self.ac_load = None  # Set by the model
        # DC load of a current-fed bridge, set by the model
        self.dc_load: Subsystem | None = None
        # Gate signals of the devices, shape (2, 3), None for diodes
        self.gated: np.ndarray | None = None
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
        return diode_margin(*self._latest, self.gated)

    def _ac_port(self) -> ACPort:
        """Get the AC port of the load."""
        port = None if self.ac_load is None else self.ac_load.compute_ac_port()
        if port is None:
            raise ValueError("The diodes require a load with an AC port")
        return port

    def _dc_side(self) -> float | DCPort:
        """Voltage (V) between the rails or the DC port, including the diode drops."""
        if self.dc_load is None:
            return self.out.u_dc + 2 * self.u_th
        dc = self.dc_load.compute_dc_port()
        if dc is None:
            raise ValueError("The current-fed bridge requires a load with a DC port")
        return DCPort(dc.e + 2 * self.u_th, dc.R, dc.L)

    def update_diodes(self) -> None:
        """Update the diode states at a transition, see `update_diodes`."""
        self._diodes = update_diodes(
            self._ac_port(), self._dc_side(), self._diodes, self.gated
        )

    @staticmethod
    def _rectified_current(i_c_ab: Any) -> Any:
        """Current (A) rectified by the diodes from the AC current."""
        return 0.5 * np.sum(np.abs(complex2abc(i_c_ab)), axis=0)

    def compute_internal_dc_current(self, inp: Any) -> Any:
        """Compute the internal DC current (A), rectified by the diodes."""
        return -self._rectified_current(inp.i_c_ab)

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
        if self.dc_load is None:
            self._set_dc_voltage()
        port, dc = self._ac_port(), self._dc_side()
        if self._diodes is None:
            self._diodes = update_diodes(port, dc, None, self.gated)
        u_dc = dc_voltage(port, dc, self._diodes) if isinstance(dc, DCPort) else dc
        u_abc = leg_voltages(port, u_dc, self._diodes)
        self.out.u_c_ab = abc2complex(u_abc)
        if self.dc_load is not None:
            # The DC current is computed from the current of the port, which comes
            # from the states of the load, independently of the evaluation order
            self.out.u_dc = u_dc - 2 * self.u_th
            self.out.i_dc = self._rectified_current(port.i)
        # The margin is computed on demand, only for locating the transitions
        self._latest = (port, u_dc, self._diodes, u_abc)

    def rhs(self, t: float) -> list[complex]:
        """Default empty implementation."""
        return []

    def init_state_history(self) -> None:
        """Initialize the history of the voltages, which are evaluated by the model."""
        super().init_state_history()
        self._history |= {"u_c_ab": [], "u_dc": [], "i_dc": []}

    def sample(self) -> Any:
        """Voltage, and the DC voltage and current of a current-fed bridge."""
        if self.dc_load is None:
            return self.out.u_c_ab
        return self.out.u_c_ab, self.out.u_dc, self.out.i_dc

    def extend_history(self, n: int, samples: Sequence[Any] | None) -> None:
        """
        Extend the history over the solution points of an integration interval.

        Parameters
        ----------
        n : int
            Number of solution points.
        samples : Sequence[Any] | None
            Voltages (V) evaluated at the solution points if only the diodes conduct,
            otherwise None. For a current-fed bridge, each sample consists of the
            converter voltage, the DC voltage, and the DC current, see `sample`.

        """
        if samples is None:
            self._history["u_c_ab"].extend([np.nan] * n)
        elif self.dc_load is None:
            self._history["u_c_ab"].extend(samples)
        else:
            u_c_ab, u_dc, i_dc = zip(*samples, strict=True)
            self._history["u_c_ab"].extend(u_c_ab)
            self._history["u_dc"].extend(u_dc)
            self._history["i_dc"].extend(i_dc)

    def create_time_series(self, t: np.ndarray) -> "DiodeBridgeTimeSeries":
        """Create time series."""
        return DiodeBridgeTimeSeries(t, self)


@dataclass
class DiodeBridgeTimeSeries[T: DiodeBridge](SubsystemTimeSeries):
    """Continuous time series."""

    t: InitVar[np.ndarray]
    subsystem: InitVar[T]
    u_dc: np.ndarray = field(default_factory=empty_array)
    i_c_ab: np.ndarray = field(default_factory=empty_array)
    u_c_ab: np.ndarray = field(default_factory=empty_array)
    i_dc: np.ndarray = field(default_factory=empty_array)
    i_dc_int: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: T) -> None:
        # Constant DC-bus voltage, replaced by the DC-bus time series if connected,
        # or the DC voltage of a current-fed bridge
        if subsystem._history["u_dc"]:
            self.u_dc = np.array(subsystem._history["u_dc"], dtype=float)
            self.i_dc = np.array(subsystem._history["i_dc"], dtype=float)
        else:
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


class VoltageSourceConverter(DiodeBridge, ControlledConverter):
    """
    Three-phase voltage-source converter.

    The modulator converts the duty ratios of the control system into the gate states
    of the legs, i.e., the gate signals of the upper switches and the blanking signals,
    including the computational delay and the dead time, see `ZOH` and
    `CarrierComparison`. The switches are ideal, except for the dead time and the
    threshold voltage. When a leg is blanked, i.e., neither of its switches conducts,
    the current direction determines which diode conducts. The leg state is then
    `(1 - sign(i))/2`, where `i` is the phase current in the beginning of the blanking
    interval. The threshold voltage of the conducting device, assumed equal for the
    switches and the diodes, reduces each phase voltage by `u_th*sign(i)`, where the
    current direction is also determined at the latest switching instant. A current
    within the tolerance of the diode-state transitions has no direction, see
    `current_direction`. The DC-bus voltage is the input `u_dc`, which is constant
    unless a DC-bus model is connected to it (see `DCBus` and `Model`).

    When the gates are blocked (`inp.enabled` is False), all the switches are open, and
    the converter operates as a diode bridge, see `DiodeBridge`. The diode states are
    latched from the current directions when the gates are blocked.

    Parameters
    ----------
    u_dc : float, optional
        Constant DC-bus voltage (V). Defaults to None, in which case the converter must
        be fed from a DC-bus model.
    modulator : TwoLevelModulator, optional
        Modulator converting the command of the control system into the gate states,
        see `TwoLevelModulator`. Defaults to `ZOH()`, i.e., the switching-cycle
        averaged model of the duty ratios without the dead time and with the
        computational delay of one sampling period. A modulator keeps its state, so
        each converter needs its own.
    u_th : float, optional
        Threshold voltage (V) of the power devices, defaults to 0.
    sign : Callable[[np.ndarray], np.ndarray], optional
        Function of the phase currents (A) determining the leg states during blanking,
        defaults to `np.sign`. A smooth function, such as `2/pi*arctan(i/i_d)`, can be
        used to model the effect of parasitic capacitances, for example.
    u_dc_sensor : Sensor, optional
        DC-bus voltage sensor, defaults to an ideal measurement.

    """

    def __init__(
        self,
        u_dc: float | None = None,
        modulator: TwoLevelModulator[Any] | None = None,
        u_th: float = 0.0,
        sign: Callable[[np.ndarray], np.ndarray] = np.sign,
        u_dc_sensor: Sensor | None = None,
    ) -> None:
        super().__init__(u_dc, u_th)
        self.modulator = ZOH() if modulator is None else modulator
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

    def modulate(self, T_s: float, command: Any) -> tuple[SwitchingTimes, GateStates]:
        """
        Convert the command to the gate states and their durations.

        The command passes through the modulator also while the gates are blocked,
        but the gate states are then zero for the whole sampling period.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        command : Any
            Command computed at this sampling instant, the duty ratios for `ZOH` and
            `CarrierComparison`.

        Returns
        -------
        t_steps : SwitchingTimes, shape (N,)
            Durations of the intervals (s), summing to `T_s`.
        gates : GateStates, shape (N, 2, 3)
            Gate signals of the upper switches and blanking signals in each interval.

        """
        t_steps, gates = self.modulator(T_s, command)
        if self.inp.enabled:
            return t_steps, gates
        return np.array([T_s]), np.zeros_like(gates[:1])

    def set_gates(self, gates: np.ndarray) -> None:
        """
        Set the switching state based on the gate state of an interval.

        Parameters
        ----------
        gates : ndarray, shape (2, 3)
            Gate signals of the upper switches and blanking signals.

        """
        q_abc, b_abc = gates
        s_abc = current_direction(complex2abc(self.inp.i_c_ab), self.sign)
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

    def extend_history(self, n: int, samples: Sequence[Any] | None) -> None:
        """Extend the history, the switching state held constant, see `DiodeBridge`."""
        super().extend_history(n, samples)
        inp = self.inp
        self._history["q_c_ab"].extend([inp.q_c_ab] * n)
        self._history["s_c_ab"].extend([inp.s_c_ab] * n)
        self._history["enabled"].extend([inp.enabled] * n)

    def create_time_series(self, t: np.ndarray) -> "VoltageSourceConverterTimeSeries":
        """Create time series."""
        return VoltageSourceConverterTimeSeries(t, self)


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
    model connects its converter to the DC bus through `connect`, which registers the
    converter in `converters` and gives the connections of its DC terminals, i.e., the
    DC-bus voltage to the input `u_dc` of the converter. The DC bus draws the DC
    currents of the registered converters, computed from their switching states and
    AC currents. The DC bus can be shared by the converters of several models, see
    `CompositeModel`. An external current, e.g., from a battery or a braking chopper,
    can also be fed to the DC bus.

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

    def connect(
        self, converter: Converter
    ) -> dict[tuple[Subsystem, str], tuple[Subsystem, str]]:
        """
        Register a converter and get the connections of its DC terminals.

        Parameters
        ----------
        converter : Converter
            Converter fed from the DC bus.

        Returns
        -------
        dict[tuple[Subsystem, str], tuple[Subsystem, str]]
            Connections from the DC bus to the converter, see `Model`.

        """
        if converter not in self.converters:
            self.converters.append(converter)
        return {(converter, "u_dc"): (self, "u_dc")}

    def set_external_dc_current(self, i_dc: float | Callable[[float], float]) -> None:
        """Set external DC current (A), constant or a function of time."""
        self.inp.i_dc = i_dc

    def _compute_dc_current(self, t: float) -> float:
        """Compute the net current (A) fed to the DC-bus capacitor."""
        i_dc = 0.0 if self.inp.i_dc is None else get_value(self.inp.i_dc, t)
        return i_dc - sum(c.compute_internal_dc_current(c.inp) for c in self.converters)

    def set_outputs(self, t: float) -> None:
        """Set output variables for interconnection."""
        self.out.u_dc = self.state.u_dc.real

    def rhs(self, t: float) -> list[complex]:
        """Compute state derivatives."""
        return [self._compute_dc_current(t) / self.C_dc]

    def create_time_series(self, t: np.ndarray) -> "DCBusTimeSeries":
        """Create time series from state list."""
        return DCBusTimeSeries(t, self)


@dataclass
class DCBusTimeSeries(SubsystemTimeSeries):
    """Continuous time series."""

    t: InitVar[np.ndarray]
    subsystem: InitVar[DCBus]
    u_dc: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: DCBus) -> None:
        self.u_dc = np.real(np.array(subsystem._history["u_dc"]))
