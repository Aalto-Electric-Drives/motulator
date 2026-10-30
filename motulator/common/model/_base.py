"""Base classes for models."""

from collections.abc import Callable, Sequence
from dataclasses import InitVar, dataclass, field
from math import inf
from typing import TYPE_CHECKING, Any, Protocol, runtime_checkable

import numpy as np
from scipy.integrate._ivp.ivp import OdeResult

from motulator.common.model._ac_port import ACPort, DCPort
from motulator.common.model._pwm import GateStates, SwitchingTimes

if TYPE_CHECKING:
    from motulator.common.model._converter import DCBus


# %%
class Subsystem(Protocol):
    """
    Protocol defining the interface for all subsystems.

    A subsystem has the inputs `inp`, the outputs `out`, and the states `state`,
    typically dataclasses, whose attributes the model reads by name: the connections
    pass the outputs to the inputs (see `Model`), and the states are gathered into the
    state vector of the solver in their declaration order. The class provides methods
    for setting states, outputs, and computing state derivatives. It also provides
    methods for extending the state history and creating the time-series
    representation of the subsystem, which the model names in the results (see
    `ModelTimeSeries`). A subsystem whose outputs depend directly on some of its
    inputs lists these inputs in `feedthrough`, from which the model derives the
    evaluation order of the outputs. The state history is a dict of lists, keyed by
    the names of the state fields. The model initializes it by `init_state_history`
    and extends it by `extend_state_history`. A subclass can keep its other signals
    in the history as well, under other keys, as a converter does (see `Converter`).

    """

    inp: Any  # None if the subsystem has no inputs
    out: Any
    state: Any  # None if the subsystem has no states
    feedthrough: frozenset[str] = frozenset()  # Inputs the outputs depend on directly
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

    def compute_dc_port(self) -> DCPort | None:
        """Compute the DC port for a current-fed converter, see `DCPort`."""
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

    def create_time_series(self, t: np.ndarray) -> "SubsystemTimeSeries":
        """Create time-series representation of this subsystem."""
        ...


class Converter(Subsystem, Protocol):
    """
    Protocol defining the interface for converters.

    In addition to the subsystem interface, a converter provides its internal DC
    current, which a DC bus draws (see `DCBus`), and the states of its diodes. While
    only the diodes conduct (`blocked` is True), the converter voltage `out.u_c_ab` is
    determined by the AC port of the load `ac_load` and the states of the diodes, see
    `ACPort`. A current-fed converter (e.g., a diode bridge with a DC choke) also
    solves its DC voltage from the DC port of its DC load `dc_load`, see `DCPort`, and
    the model evaluates the DC load after the converter. The simulation locates the
    transitions of the diode states as events, using the margin of the states, and
    updates the states at the transitions.
    Since the voltage varies within the integration interval, the model evaluates the
    signals given by `sample` (the voltage by default) at the solution points and
    passes them to `extend_history`, which also saves the other signals of the
    converter that are not states.

    The diodes and the DC bus are optional capabilities: by default, the gates are
    never blocked, so there are no diode-state transitions, and the converter cannot
    be fed from a DC bus. A converter subclassing this protocol implements only the
    capabilities it has.

    """

    ac_load: Subsystem | None = None  # Load providing the AC port
    dc_load: Subsystem | None = None  # Load providing the DC port, if current-fed

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

    def sample(self) -> Any:
        """Signals evaluated at the solution points while blocked, the voltage."""
        return self.out.u_c_ab

    def extend_history(self, n: int, samples: Sequence[Any] | None, /) -> None:
        """Extend the history over `n` solution points, the `samples` if blocked."""
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
Wiring = dict[tuple[Subsystem, str], tuple[Subsystem, str]]


def check_name(name: str) -> None:
    """Check that `name` is a valid attribute name of the results, see `Model`."""
    if not name.isidentifier() or name.startswith("_") or name == "t":
        raise ValueError(f"Invalid name {name!r}")


class Model[C: Converter | None]:
    """
    Base class for continuous-time system models.

    A model consists of named subsystems and connections between them. The outputs are
    computed in an order that respects the direct dependencies of the subsystems (see
    `Subsystem.feedthrough`) and passed to the connected inputs immediately, so that
    the outputs are consistent with the inputs of the same evaluation. The order
    follows the order of the subsystems where the dependencies allow, and a loop of
    direct dependencies is rejected. The voltage of a blocked converter depends on
    the port of its load, which depends on the inputs of the load, so the converter is
    evaluated after the sources of these inputs. The names of the subsystems name
    their time series in the results, see `ModelTimeSeries`.

    A model can have a converter, either a passive one (e.g., `DiodeBridge`) or a
    controlled one (see `ControlledConverter`), which is driven by a control system, see
    `Simulation`. The modulator of the controlled converter includes the computational
    delay and the PWM model, see `Modulator`. The control system can also block the
    gates of the converter, see `ControlSystem.set_enable`.

    Parameters
    ----------
    converter : Converter | None
        Converter, or None if the model has no converter.
    subsystems : dict[str, Subsystem]
        All subsystems by name, including the converter. The names must be valid
        attribute names (not starting with an underscore, and not `t`).
    connections : dict[tuple[str, str], tuple[str, str]]
        Connections as `{(target, input_name): (source, output_name)}`, where `target`
        and `source` are the names of the subsystems.
    dc_bus : DCBus, optional
        DC bus feeding the converter. The DC bus is added to the subsystems under the
        name `dc_bus`, and the converter is connected to it, see `DCBus.connect`. The
        DC bus can be shared with other models, see `CompositeModel`. Defaults to None,
        in which case the converter has a constant DC-bus voltage.

    """

    def __init__(
        self,
        converter: C,
        subsystems: dict[str, Subsystem],
        connections: dict[tuple[str, str], tuple[str, str]],
        dc_bus: "DCBus | None" = None,
    ) -> None:
        if dc_bus is not None:
            if converter is None:
                raise ValueError("A DC bus requires a converter")
            if "dc_bus" in subsystems or dc_bus in subsystems.values():
                raise ValueError("The DC bus must not be included in the subsystems")
            subsystems = {"dc_bus": dc_bus, **subsystems}
        for name in subsystems:
            check_name(name)
        if len({id(s) for s in subsystems.values()}) < len(subsystems):
            raise ValueError("The subsystems must be distinct")
        if converter is not None and converter not in subsystems.values():
            raise ValueError("The converter must be included in the subsystems")
        wiring: Wiring = {}
        for (target, inp), (src, out) in connections.items():
            if target not in subsystems or src not in subsystems:
                raise ValueError(
                    f"Connection {src}.{out} -> {target}.{inp} to an unknown subsystem"
                )
            wiring[subsystems[target], inp] = (subsystems[src], out)
        if dc_bus is not None and converter is not None:
            wiring |= dc_bus.connect(converter)
        self.converter: C = converter
        self.converters: list[Converter] = [] if converter is None else [converter]
        self.dc_bus = dc_bus
        self._wire(subsystems, wiring)

    def _wire(self, subsystems: dict[str, Subsystem], wiring: Wiring) -> None:
        """Set the subsystems and group the connections by the source."""
        self.subsystems = subsystems
        self._outgoing: dict[Subsystem, list[tuple[Subsystem, str, str]]] = {
            s: [] for s in subsystems.values()
        }
        for (target, inp), (src, out) in wiring.items():
            if not (hasattr(target.inp, inp) and hasattr(src.out, out)):
                raise ValueError(
                    f"Invalid connection {type(src).__name__}.{out} -> "
                    f"{type(target).__name__}.{inp}"
                )
            self._outgoing[src].append((target, inp, out))
        self.t0: float = 0.0
        self._t: list[float] = []  # Time history
        for subsystem in subsystems.values():
            subsystem.init_state_history()
        # Evaluation orders of the outputs by the blocked converters, see `_order`
        self._orders: dict[tuple[bool, ...], list[Subsystem]] = {}
        self._order((False,) * len(self.converters))

    def _order(self, blocked: tuple[bool, ...]) -> list[Subsystem]:
        """Evaluation order of the outputs, given which converters are blocked."""
        if blocked not in self._orders:
            converters = [c for c, b in zip(self.converters, blocked, strict=True) if b]
            self._orders[blocked] = self._sort(converters)
        return self._orders[blocked]

    def _sort(self, blocked: list[Converter]) -> list[Subsystem]:
        """Sort the subsystems by their direct dependencies, keeping their order
        otherwise."""
        subsystems = list(self.subsystems.values())
        # Subsystems whose outputs each subsystem depends on directly
        deps: dict[Subsystem, set[Subsystem]] = {s: set() for s in subsystems}
        for src, targets in self._outgoing.items():
            for target, inp, _ in targets:
                if inp in target.feedthrough:
                    deps[target].add(src)
        # The voltage of a blocked converter depends on the ports of its loads, which
        # depend on the inputs of the loads
        for converter in blocked:
            loads = [converter.ac_load, converter.dc_load]
            for src, targets in self._outgoing.items():
                if src is not converter and any(t in loads for t, _, _ in targets):
                    deps[converter].add(src)
        order: list[Subsystem] = []
        while len(order) < len(subsystems):
            ready = [s for s in subsystems if s not in order and deps[s] <= set(order)]
            if not ready:
                raise ValueError("Loop in the direct dependencies of the subsystems")
            order.append(ready[0])
        return order

    def get_initial_values(self) -> list[complex]:
        """Get initial values of all subsystems before the solver."""
        state0: list[complex] = []
        for subsystem in self.subsystems.values():
            if subsystem.state is not None:
                state0.extend(vars(subsystem.state).values())
        return state0

    def set_states(self, state_list: list[complex]) -> None:
        """Set states in all subsystems."""
        index = 0
        for subsystem in self.subsystems.values():
            index = subsystem.set_states(state_list, index)

    def set_outputs(self, t: float) -> None:
        """Compute the outputs and pass them to the connected inputs."""
        for subsystem in self._order(tuple(c.blocked for c in self.converters)):
            subsystem.set_outputs(t)
            for target, inp, out in self._outgoing[subsystem]:
                setattr(target.inp, inp, getattr(subsystem.out, out))

    def rhs(self, t: float, state_list: list[complex]) -> list[complex]:
        """Compute complete state derivative list for the solver."""
        self.set_states(state_list)
        self.set_outputs(t)
        rhs_list: list[complex] = []
        for subsystem in self.subsystems.values():
            if derivatives := subsystem.rhs(t):
                rhs_list.extend(derivatives)
        return rhs_list

    def save(self, sol: OdeResult) -> None:
        """Save the solution and the signals of the converters."""
        self._t.extend(sol.t)
        index = 0
        for subsystem in self.subsystems.values():
            index = subsystem.extend_state_history(sol.y, index)
        # The voltage of a blocked converter varies within the integration interval,
        # so its samples are evaluated at the solution points
        samples: dict[Converter, Sequence[Any]] = {}
        if blocked := [c for c in self.converters if c.blocked]:
            values = self._evaluate(sol, lambda: [c.sample() for c in blocked])
            samples = dict(zip(blocked, zip(*values, strict=True), strict=True))
        for converter in self.converters:
            converter.extend_history(len(sol.t), samples.get(converter))

    def time_series(self, t: np.ndarray) -> dict[Subsystem, "SubsystemTimeSeries"]:
        """Compute the time series of the subsystems, connected as in the model."""
        ts = {s: s.create_time_series(t) for s in self.subsystems.values()}
        # Outputs in the evaluation order of the simulation (the voltage of a blocked
        # converter is in its history), passed to the inputs immediately, see
        # `set_outputs`
        for subsystem in self._order((False,) * len(self.converters)):
            ts[subsystem].set_outputs(t, subsystem)
            for target, inp, out in self._outgoing[subsystem]:
                setattr(ts[target], inp, np.array(getattr(ts[subsystem], out)))
        # Signals derived from the inputs, which are all available now
        for subsystem in self.subsystems.values():
            ts[subsystem].compute_derived_signals(t, subsystem)
        return ts

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

    The time series of each subsystem is stored as an attribute by the name of the
    subsystem in the model (e.g., `machine`). It also contains the time series of the
    subsystem inputs, which are used for plotting and for computing the signals that
    depend directly on the inputs.

    Parameters
    ----------
    mdl : Model
        Model whose subsystems the time series cover.
    ts : dict[Subsystem, SubsystemTimeSeries], optional
        Time series of the subsystems, computed with the wiring of a composite model
        (see `CompositeModelTimeSeries`). Defaults to None, in which case they are
        computed from the model, see `Model.time_series`.

    """

    mdl: InitVar[Model]
    ts: InitVar[dict[Subsystem, SubsystemTimeSeries] | None] = None
    t: np.ndarray = field(default_factory=lambda: np.array([]), init=False)

    def __post_init__(
        self, mdl: Model, ts: dict[Subsystem, SubsystemTimeSeries] | None
    ) -> None:
        self.t = np.array(mdl._t)
        if ts is None:
            ts = mdl.time_series(self.t)
        for name, subsystem in mdl.subsystems.items():
            setattr(self, name, ts[subsystem])

    def __getattr__(self, name: str) -> Any:
        """Support type checking for dynamic attributes."""
        # This helps type checkers understand dynamic attributes
        error_msg = f"'{self.__class__.__name__}' has no attribute '{name}'"
        raise AttributeError(error_msg)
