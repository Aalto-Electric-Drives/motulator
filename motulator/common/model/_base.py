"""Base classes for models."""

from collections.abc import Callable, Sequence
from dataclasses import InitVar, dataclass, field
from math import inf
from typing import TYPE_CHECKING, Any, Protocol, runtime_checkable

import numpy as np
from scipy.integrate._ivp.ivp import OdeResult

from motulator.common.model._ac_port import ACPort
from motulator.common.model._pwm import GateStates, SwitchingTimes

if TYPE_CHECKING:
    from motulator.common.model._converter import DCBus


# %%
class SubsystemInputs(Protocol):
    """Protocol for subsystem inputs."""


class SubsystemOutputs(Protocol):
    """Protocol for subsystem outputs."""


class SubsystemStates(Protocol):
    """Protocol for subsystem states."""


class Subsystem[Inp: SubsystemInputs, Out: SubsystemOutputs, State: SubsystemStates](
    Protocol
):
    """
    Protocol defining the interface for all subsystems.

    This class defines the interface for all subsystems. It is a generic class that can
    be used with different input, output, and state types. The class provides methods
    for setting states, outputs, and computing state derivatives. It also provides
    methods for extending the state history and creating time-series representations
    of the subsystem. The state history is a dict of lists, keyed by the names of the
    state fields. The model initializes it by `init_state_history` and extends it by
    `extend_state_history`. A subclass can keep its other signals in the history as
    well, under other keys, as a converter does (see `Converter`).

    """

    inp: Inp  # None if the subsystem has no inputs
    out: Out
    state: State | None
    _history: dict[str, list[complex]]

    def set_states(self, state_list: list[complex], index: int) -> int:
        """Set states from the state list."""
        if self.state is not None:
            for attr in vars(self.state):
                setattr(self.state, attr, state_list[index])
                index += 1
        return index

    def set_outputs(self, t: float) -> None:
        """Set output variables."""
        ...

    def rhs(self, t: float) -> list[complex]:
        """Compute state derivatives."""
        ...

    def compute_ac_port(self) -> ACPort | None:
        """Compute the AC port for a blocked converter, see `ACPort`."""
        return None  # Not available by default

    def init_state_history(self) -> None:
        """Initialize the state history with an empty list for each state field."""
        states = vars(self.state) if self.state is not None else {}
        self._history = {name: [] for name in states}

    def extend_state_history(self, sol_y: list[list[float]], index: int) -> int:
        """Extend the state history with values from solver output."""
        states = vars(self.state) if self.state is not None else {}
        for name in states:
            self._history[name].extend(sol_y[index])
            index += 1
        return index

    def create_time_series(self, t: np.ndarray) -> tuple[str, "SubsystemTimeSeries"]:
        """Create time-series representation of this subsystem."""
        ...


class Converter(Subsystem, Protocol):
    """
    Protocol defining the interface for converters.

    In addition to the subsystem interface, a converter provides its internal DC
    current, which a DC bus draws (see `DCBus`), and the states of its diodes. While
    only the diodes conduct (`blocked` is True), the converter voltage `out.u_c_ab` is
    determined by the AC port of the load `ac_load` and the states of the diodes, see
    `ACPort`. The simulation then locates the transitions of the diode states as
    events, using the margin of the states, and updates the states at the transitions.
    Since the voltage varies within the integration interval, the model evaluates it
    at the solution points and passes it to `extend_history`, which also saves the
    other signals of the converter that are not states.

    The diodes and the DC bus are optional capabilities: by default, the gates are
    never blocked, so there are no diode-state transitions, and the converter cannot
    be fed from a DC bus. A converter subclassing this protocol implements only the
    capabilities it has.

    """

    ac_load: Subsystem | None = None  # Load providing the AC port

    @property
    def margin(self) -> float:
        """Margin of the diode states, zero at their transition."""
        return inf  # No transitions

    @property
    def blocked(self) -> bool:
        """Only the diodes conduct."""
        return False

    def update_diodes(self) -> None:
        """Update the diode states at a transition, nothing by default."""
        return

    def compute_internal_dc_current(self, inp: Any) -> Any:
        """Compute the internal DC current (A) from the inputs."""
        raise TypeError(
            f"{type(self).__name__} must override compute_internal_dc_current to be "
            "fed from a DC bus"
        )

    def extend_history(self, n: int, u_c_ab: Sequence[Any] | None) -> None:
        """Extend the history over `n` solution points, `u_c_ab` if blocked."""
        ...


@runtime_checkable
class ControlledConverter(Protocol):
    """
    Protocol defining the gate control of converters.

    A controlled converter is a converter (see `Converter`) whose switches are driven by
    a control system. Its modulator converts the command of the control system (e.g.,
    the duty ratios) into the gate states and their durations within the sampling
    period, see `Modulator`. The simulation sets the gate states at the switching
    instants, from which the converter resolves its switching state (e.g., from the
    gate signals and the phase currents during the blanking). The format of the
    command and the gate states is up to the converter and its modulator. The control
    system can block the gates through `set_enable` at its sampling instants, after
    which only the diodes conduct (see `Converter.blocked`). By default, blocking the
    gates is not supported.

    """

    def modulate(self, T_s: float, command: Any) -> tuple[SwitchingTimes, GateStates]:
        """Convert the command to the gate states and their durations."""
        ...

    def set_gates(self, gates: np.ndarray) -> None:
        """Set the switching state based on the gate states."""
        ...

    def set_enable(self, enabled: bool) -> None:
        """Set the gate enable, blocking the gates not supported by default."""
        if not enabled:
            raise TypeError(f"{type(self).__name__} does not support gate blocking")


# %%
class Model[C: Converter | None]:
    """
    Base class for continuous-time system models.

    A model consists of subsystems and connections between them. The outputs are
    computed in the order of the `subsystems` list and passed to the connected inputs
    immediately. Hence, a subsystem whose outputs depend directly on its inputs must
    come after the subsystems providing these inputs. The voltage of a blocked
    converter depends on its load, so it is computed after the other subsystems.

    A model can have a converter, either a passive one (e.g., `DiodeBridge`) or a
    controlled one (see `ControlledConverter`), which is driven by a control system, see
    `Simulation`. The modulator of the controlled converter includes the computational
    delay and the PWM model, see `Modulator`. The control system can also block the
    gates of the converter, see `ControlSystem.set_enable`.

    Parameters
    ----------
    converter : Converter | None
        Converter, or None if the model has no converter.
    subsystems : list[Subsystem]
        All subsystems, including the converter.
    connections : dict[tuple[Subsystem, str], tuple[Subsystem, str]]
        Connections as `{(target, input_name): (source, output_name)}`.
    dc_bus : DCBus, optional
        DC bus feeding the converter. The DC bus is added to the subsystems, and the
        converter is connected to it, see `DCBus.connect`. The DC bus can be shared
        with other models, see `CompositeModel`. Defaults to None, in which case the
        converter has a constant DC-bus voltage.

    """

    def __init__(
        self,
        converter: C,
        subsystems: list[Subsystem],
        connections: dict[tuple[Subsystem, str], tuple[Subsystem, str]],
        dc_bus: "DCBus | None" = None,
    ) -> None:
        if converter is not None and converter not in subsystems:
            raise ValueError("The converter must be included in the subsystems")
        if dc_bus is not None:
            if converter is None:
                raise ValueError("A DC bus requires a converter")
            if dc_bus in subsystems:
                raise ValueError("The DC bus must not be included in the subsystems")
            # The DC bus comes first, since the converter voltage depends directly on
            # the DC-bus voltage
            subsystems = [dc_bus, *subsystems]
            connections = {**connections, **dc_bus.connect(converter)}
        # Group the connections by the source for passing the outputs to the inputs
        self._outgoing: dict[Subsystem, list[tuple[Subsystem, str, str]]] = {
            s: [] for s in subsystems
        }
        for (target, inp), (src, out) in connections.items():
            if not (hasattr(target.inp, inp) and hasattr(src.out, out)):
                raise ValueError(
                    f"Invalid connection {type(src).__name__}.{out} -> "
                    f"{type(target).__name__}.{inp}"
                )
            self._outgoing[src].append((target, inp, out))
        self.t0: float = 0.0
        self.subsystems = subsystems
        self.connections = connections
        self.converter: C = converter
        self.converters: list[Converter] = [] if converter is None else [converter]
        self.dc_bus = dc_bus
        self._t: list[float] = []  # Time history
        for subsystem in subsystems:
            subsystem.init_state_history()

    def get_initial_values(self) -> list[complex]:
        """Get initial values of all subsystems before the solver."""
        state0: list[complex] = []
        for subsystem in self.subsystems:
            if subsystem.state is not None:
                state0.extend(vars(subsystem.state).values())
        return state0

    def set_states(self, state_list: list[complex]) -> None:
        """Set states in all subsystems."""
        index = 0
        for subsystem in self.subsystems:
            index = subsystem.set_states(state_list, index)

    def set_outputs(self, t: float) -> None:
        """Compute the outputs and pass them to the connected inputs."""
        # The voltage of a blocked converter depends on the outputs of its load
        order = self.subsystems
        if blocked := [c for c in self.converters if c.blocked]:
            order = [s for s in self.subsystems if s not in blocked] + blocked
        for subsystem in order:
            subsystem.set_outputs(t)
            for target, inp, out in self._outgoing[subsystem]:
                setattr(target.inp, inp, getattr(subsystem.out, out))

    def rhs(self, t: float, state_list: list[complex]) -> list[complex]:
        """Compute complete state derivative list for the solver."""
        self.set_states(state_list)
        self.set_outputs(t)
        rhs_list: list[complex] = []
        for subsystem in self.subsystems:
            if derivatives := subsystem.rhs(t):
                rhs_list.extend(derivatives)
        return rhs_list

    def save(self, sol: OdeResult) -> None:
        """Save the solution and the signals of the converters."""
        self._t.extend(sol.t)
        index = 0
        for subsystem in self.subsystems:
            index = subsystem.extend_state_history(sol.y, index)
        # The voltage of a blocked converter varies within the integration interval,
        # so it is evaluated at the solution points
        u_c_ab: dict[Converter, Sequence[Any]] = {}
        if blocked := [c for c in self.converters if c.blocked]:
            values = self._evaluate(sol, lambda: [c.out.u_c_ab for c in blocked])
            u_c_ab = dict(zip(blocked, zip(*values, strict=True), strict=True))
        for converter in self.converters:
            converter.extend_history(len(sol.t), u_c_ab.get(converter))

    def _evaluate[T](self, sol: OdeResult, fcn: Callable[[], T]) -> list[T]:
        """Evaluate `fcn` at the solution points, restoring the final state after."""
        state = self.get_initial_values()
        values: list[T] = []
        for t, y in zip(sol.t, sol.y.T, strict=True):
            self.set_states(y)
            self.set_outputs(t)
            values.append(fcn())
        self.set_states(state)
        self.set_outputs(self.t0)
        return values


# %%
class SubsystemTimeSeries[S: Subsystem](Protocol):
    """
    Base class for subsystem time series.

    The outputs are computed when the time series is created or, if they depend
    directly on the inputs, in `set_outputs`. The signals computed in
    `compute_derived_signals` are not passed to other subsystems.

    """

    t: np.ndarray

    def set_outputs(self, t: np.ndarray, subsystem: S) -> None:
        """Compute the outputs that depend directly on the inputs."""
        pass

    def compute_derived_signals(self, t: np.ndarray, subsystem: S) -> None:
        """Compute the other signals that depend on the inputs."""
        pass


@dataclass
class ModelTimeSeries:
    """
    Time series of the simulation results.

    The time series of each subsystem is stored as an attribute (e.g., `machine`). It
    also contains the time series of the subsystem inputs, which are used for plotting
    and for computing the signals that depend directly on the inputs.

    """

    mdl: InitVar[Model]
    t: np.ndarray = field(default_factory=lambda: np.array([]), init=False)

    def __post_init__(self, mdl: Model) -> None:
        self.t = np.array(mdl._t)
        ts = {}
        for subsystem in mdl.subsystems:
            name, ts[subsystem] = subsystem.create_time_series(self.t)
            setattr(self, name, ts[subsystem])
        # Outputs in the evaluation order of the simulation, passed to the inputs
        # immediately, see `Model.set_outputs`
        for subsystem in mdl.subsystems:
            ts[subsystem].set_outputs(self.t, subsystem)
            for target, inp, out in mdl._outgoing[subsystem]:
                setattr(ts[target], inp, np.array(getattr(ts[subsystem], out)))
        # Signals derived from the inputs, which are all available now
        for subsystem in mdl.subsystems:
            ts[subsystem].compute_derived_signals(self.t, subsystem)

    def __getattr__(self, name: str) -> Any:
        """Support type checking for dynamic attributes."""
        # This helps type checkers understand dynamic attributes
        error_msg = f"'{self.__class__.__name__}' has no attribute '{name}'"
        raise AttributeError(error_msg)
