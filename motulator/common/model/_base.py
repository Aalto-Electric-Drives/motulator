"""Base classes for models."""

from collections.abc import Sequence
from dataclasses import InitVar, dataclass, field
from typing import TYPE_CHECKING, Any, Protocol

import numpy as np
from scipy.integrate._ivp.ivp import OdeResult

from motulator.common.model._pwm import (
    ZOH,
    BlankingSignals,
    CarrierComparison,
    GateSignals,
    SwitchingTimes,
)

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
    be used with different input, output, state, and history types. The class provides
    methods for setting states, outputs, and computing state derivatives. It also
    provides methods for extending the state history and creating time-series
    representations of the subsystem. The state history is a dict of lists, keyed by
    the names of the state fields. The model initializes it by `init_state_history`
    and extends it by `extend_state_history`.

    """

    inp: Inp | None
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

    def init_state_history(self) -> None:
        """Initialize the state history with an empty list for each state field."""
        states = vars(self.state) if self.state is not None else {}
        self._history = {name: [] for name in states}

    def extend_state_history(self, sol_y: list[list[float]], index: int) -> int:
        """Extend the state history with values from solver output."""
        for name in self._history:
            self._history[name].extend(sol_y[index])
            index += 1
        return index

    def create_time_series(self, t: np.ndarray) -> tuple[str, "SubsystemTimeSeries"]:
        """Create time-series representation of this subsystem."""
        ...


class ConverterInputs(Protocol):
    """Protocol for converter inputs."""

    q_c_ab: Any  # Switching state, held constant over each integration interval
    s_c_ab: Any  # Current-direction vector at the latest switching instant


class Converter(Subsystem, Protocol):
    """
    Protocol defining the interface for converters.

    In addition to the subsystem interface, a converter provides the number of legs,
    the dead time, and a method for setting the switching state `inp.q_c_ab` and the
    current-direction vector `inp.s_c_ab` based on the gate and blanking signals from
    the PWM model and the phase currents. The format of these is up to the converter
    (e.g., complex space vectors in a three-phase converter).

    """

    n_legs: int  # Number of legs
    t_d: float  # Dead time (s)

    @property
    def inp(self) -> ConverterInputs:
        """Input variables."""
        ...

    def set_gate_signals(self, q_abc: np.ndarray, b_abc: np.ndarray) -> None:
        """Set the switching state based on the gate and blanking signals."""
        ...


# %%
@dataclass
class ModelStateHistory:
    """Temporary storage."""

    t: list[float] = field(default_factory=list)
    q_c_ab: list[complex] = field(default_factory=list)
    s_c_ab: list[complex] = field(default_factory=list)


class System:
    """
    Base class for continuous-time system models.

    A system consists of subsystems and connections between them. The outputs are
    computed in the order of the `subsystems` list and passed to the connected inputs
    immediately. Hence, a subsystem whose outputs depend directly on its inputs must
    come after the subsystems providing these inputs.

    Parameters
    ----------
    subsystems : list[Subsystem]
        All subsystems.
    connections : dict[tuple[Subsystem, str], tuple[Subsystem, str]]
        Connections as `{(target, input_name): (source, output_name)}`.

    """

    def __init__(
        self,
        subsystems: list[Subsystem],
        connections: dict[tuple[Subsystem, str], tuple[Subsystem, str]],
    ) -> None:
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
        self._history = ModelStateHistory()
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
        for subsystem in self.subsystems:
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
        """Save the solution."""
        self._history.t.extend(sol.t)
        index = 0
        for subsystem in self.subsystems:
            index = subsystem.extend_state_history(sol.y, index)


class Model[C: Converter](System):
    """
    Base class for system models with a converter driven by a control system.

    The converter subsystem gets the switching state `q_c_ab` and the current-direction
    vector `s_c_ab`, which are held constant over each integration interval. The PWM
    model includes the dead time `converter.t_d`, and the computational delay is sized
    according to the number of legs `converter.n_legs`. The control systems of this
    package compensate for a computational delay of one sampling period, i.e.,
    `delay=1` with double update and `delay=2` with single update.

    Parameters
    ----------
    converter : Converter
        Converter model.
    subsystems : list[Subsystem]
        All subsystems, including the converter.
    connections : dict[tuple[Subsystem, str], tuple[Subsystem, str]]
        Connections as `{(target, input_name): (source, output_name)}`.
    dc_bus : DCBus, optional
        DC bus feeding the converter, whose input `u_dc` is connected to the DC-bus
        voltage. The DC bus is added to the subsystems, and it can be shared with other
        models, see `CompositeModel`. Defaults to None, in which case the converter has
        its own DC-bus model (or a constant DC-bus voltage).
    pwm : bool, optional
        Enable PWM model, defaults to False.
    delay : int, optional
        Computational delay in half carrier periods from the sampling instant to the
        loading of the duty ratios, defaults to 0. With double update, this equals
        the delay in sampling periods.
    single_update : bool, optional
        Sample and update the duty ratios once per carrier period, i.e., the sampling
        period of the control system equals the carrier period, defaults to False
        (double update, the sampling period is half the carrier period). With single
        update, `delay=2` loads the duty ratios at the next sampling instant and
        `delay=1` at the carrier extremum between the sampling instants.

    """

    def __init__(
        self,
        converter: C,
        subsystems: list[Subsystem],
        connections: dict[tuple[Subsystem, str], tuple[Subsystem, str]],
        dc_bus: "DCBus | None" = None,
        pwm: bool = False,
        delay: int = 0,
        single_update: bool = False,
    ) -> None:
        if converter not in subsystems:
            raise ValueError("The converter must be included in the subsystems")
        if dc_bus is not None:
            if converter not in dc_bus.converters:
                raise ValueError("The converter must be given to the DC bus")
            if dc_bus in subsystems:
                raise ValueError("The DC bus must not be included in the subsystems")
            # The DC bus comes first, since the converter voltage depends directly on
            # the DC-bus voltage
            subsystems = [dc_bus, *subsystems]
            connections = {**connections, (converter, "u_dc"): (dc_bus, "u_dc")}
        super().__init__(subsystems, connections)
        self.converter: C = converter
        self.dc_bus = dc_bus
        self.delay = Delay(delay, converter.n_legs)
        self.single_update = single_update
        t_d = converter.t_d
        self.pwm = CarrierComparison(t_d=t_d) if pwm else ZOH(t_d=t_d)

    def modulate(
        self, T_s: float, d_abc: Sequence[float]
    ) -> tuple[SwitchingTimes, GateSignals, BlankingSignals]:
        """
        Delay the duty ratios and convert them to gate signals.

        The sampling period consists of one half carrier period with double update
        and of two with single update. The duty ratios pass through the
        computational delay once per half carrier period, and the PWM model is
        called for each half carrier period.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        d_abc : Sequence[float]
            Duty ratios computed at this sampling instant.

        Returns
        -------
        t_steps : SwitchingTimes, shape (N,)
            Durations of the intervals (s), summing to `T_s`.
        q_abc : GateSignals, shape (N, n_legs)
            Gate signals of the upper switches in each interval.
        b_abc : BlankingSignals, shape (N, n_legs)
            Blanking signals in each interval.

        """
        n = 2 if self.single_update else 1  # Half carrier periods per sampling period
        parts = [self.pwm(T_s / n, self.delay(d_abc)) for _ in range(n)]
        t_steps = np.concatenate([p[0] for p in parts])
        q_abc = np.vstack([p[1] for p in parts])
        b_abc = np.vstack([p[2] for p in parts])
        return t_steps, q_abc, b_abc

    def save(self, sol: OdeResult) -> None:
        """Save the solution and the switching state."""
        super().save(sol)
        self._history.q_c_ab.extend([self.converter.inp.q_c_ab] * len(sol.t))
        self._history.s_c_ab.extend([self.converter.inp.s_c_ab] * len(sol.t))


# %%
class SubsystemTimeSeries[S: Subsystem](Protocol):
    """Base class for subsystem time series."""

    t: np.ndarray

    def compute_zoh_input_derived_signals(self, t: np.ndarray, subsystem: S) -> None:
        """Compute additional time series using subsystem's ZOH inputs."""
        pass

    def compute_input_derived_signals(self, t: np.ndarray, subsystem: S) -> None:
        """Compute additional time series using subsystem's regular inputs."""
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
        self.t = np.array(mdl._history.t)
        ts = {}
        for subsystem in mdl.subsystems:
            name, ts[subsystem] = subsystem.create_time_series(self.t)
            setattr(self, name, ts[subsystem])

        def pass_inputs() -> None:
            for (target, inp), (src, out) in mdl.connections.items():
                setattr(ts[target], inp, np.array(getattr(ts[src], out)))

        # Switching states and the signals derived from them. The inputs are passed
        # first, since the derived signals can depend on the inputs that are given by
        # the states (e.g., the converter voltage on the DC-bus voltage).
        ts[mdl.converter].q_c_ab = np.array(mdl._history.q_c_ab)
        ts[mdl.converter].s_c_ab = np.array(mdl._history.s_c_ab)
        pass_inputs()
        for subsystem in mdl.subsystems:
            ts[subsystem].compute_zoh_input_derived_signals(self.t, subsystem)
        # Inputs from the connections and the signals derived from them. The second
        # pass propagates the signals corrected in the first pass (e.g., the stator
        # current with core losses).
        for _ in range(2):
            pass_inputs()
            for subsystem in mdl.subsystems:
                ts[subsystem].compute_input_derived_signals(self.t, subsystem)

    def __getattr__(self, name: str) -> Any:
        """Support type checking for dynamic attributes."""
        # This helps type checkers understand dynamic attributes
        error_msg = f"'{self.__class__.__name__}' has no attribute '{name}'"
        raise AttributeError(error_msg)


# %%
class Delay:
    """
    Computational delay modeled as a ring buffer.

    Parameters
    ----------
    length : int, optional
        Length of the buffer, i.e., the delay in calls, defaults to 1.
    elem : int, optional
        Number of elements in each sample, defaults to 3.

    """

    def __init__(self, length: int = 1, elem: int = 3) -> None:
        self.data = [elem * [0] for _ in range(length)]  # Creates zero lists

    def __call__(self, u: Any) -> Any:
        """
        Delay the input.

        Parameters
        ----------
        u : array_like, shape (elem,)
            Input array.

        Returns
        -------
        array_like, shape (elem,)
            Output array.

        """
        # Add the latest value to the end of the list
        self.data.append(u)
        # Pop the first element and return it
        return self.data.pop(0)
