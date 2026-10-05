"""Base classes for models."""

from dataclasses import InitVar, dataclass, field
from typing import Any, Protocol

import numpy as np
from scipy.integrate._ivp.ivp import OdeResult

from motulator.common.model._pwm import ZOH, CarrierComparison


# %%
class SubsystemInputs(Protocol):
    """Protocol for subsystem inputs."""


class SubsystemOutputs(Protocol):
    """Protocol for subsystem outputs."""


class SubsystemStates(Protocol):
    """Protocol for subsystem states."""


class SubsystemStateHistory(Protocol):
    """Protocol for subsystem state histories."""


class Subsystem[
    Inp: SubsystemInputs,
    Out: SubsystemOutputs,
    State: SubsystemStates,
    History: SubsystemStateHistory,
](Protocol):
    """
    Protocol defining the interface for all subsystems.

    This class defines the interface for all subsystems. It is a generic class that can
    be used with different input, output, state, and history types. The class provides
    methods for setting states, outputs, and computing state derivatives. It also
    provides methods for extending the state history and creating time-series
    representations of the subsystem.

    """

    inp: Inp | None
    out: Out
    state: State | None
    _history: History | None

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

    def extend_state_history(self, sol_y: list[list[float]], index: int) -> int:
        """Extend the state history with values from solver output."""
        if self._history is not None:
            for attr, value in vars(self._history).items():
                if isinstance(value, list):
                    value.extend(sol_y[index])
                else:
                    setattr(self._history, attr, list(sol_y[index]))
                index += 1
        return index

    def create_time_series(self, t: np.ndarray) -> tuple[str, "SubsystemTimeSeries"]:
        """Create time-series representation of this subsystem."""
        ...


class ConverterInputs(Protocol):
    """Protocol for converter inputs."""

    q_c_ab: Any  # Switching state, held constant over each integration interval


class Converter(Subsystem, Protocol):
    """
    Protocol defining the interface for converters.

    In addition to the subsystem interface, a converter provides the number of legs,
    the dead time, and a method for setting the switching state `inp.q_c_ab` based on
    the gate and blanking signals from the PWM model. The format of the switching state
    is up to the converter (e.g., a complex space vector in a three-phase converter).

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


class Model[C: Converter]:
    """
    Base class for continuous-time system models.

    A model consists of subsystems and connections between them. The converter
    subsystem gets the switching state `q_c_ab`, which is held constant over each
    integration interval. The PWM model includes the dead time `converter.t_d`, and the
    computational delay is sized according to the number of legs `converter.n_legs`. The
    outputs are computed in the order of the `subsystems` list and passed to the
    connected inputs immediately. Hence, a subsystem whose outputs depend directly on
    its inputs must come after the subsystems providing these inputs.

    Parameters
    ----------
    converter : Converter
        Converter model.
    subsystems : list[Subsystem]
        All subsystems, including the converter.
    connections : dict[tuple[Subsystem, str], tuple[Subsystem, str]]
        Connections as `{(target, input_name): (source, output_name)}`.
    pwm : bool, optional
        Enable PWM model, defaults to False.
    delay : int, optional
        Computational delay (samples), defaults to 0.

    """

    def __init__(
        self,
        converter: C,
        subsystems: list[Subsystem],
        connections: dict[tuple[Subsystem, str], tuple[Subsystem, str]],
        pwm: bool = False,
        delay: int = 0,
    ) -> None:
        if converter not in subsystems:
            raise ValueError("The converter must be included in the subsystems")
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
        self.delay = Delay(delay, converter.n_legs)
        t_d = converter.t_d
        self.pwm = CarrierComparison(t_d=t_d) if pwm else ZOH(t_d=t_d)
        self.converter: C = converter
        self.subsystems = subsystems
        self.connections = connections
        self._history = ModelStateHistory()

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
        """Save the solution and the switching state."""
        self._history.t.extend(sol.t)
        self._history.q_c_ab.extend([self.converter.inp.q_c_ab] * len(sol.t))
        # Save states
        index = 0
        for subsystem in self.subsystems:
            index = subsystem.extend_state_history(sol.y, index)


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
        # Switching states and the signals derived from them
        ts[mdl.converter].q_c_ab = np.array(mdl._history.q_c_ab)
        for subsystem in mdl.subsystems:
            ts[subsystem].compute_zoh_input_derived_signals(self.t, subsystem)
        # Inputs from the connections and the signals derived from them. The second
        # pass propagates the signals corrected in the first pass (e.g., the stator
        # current with core losses).
        for _ in range(2):
            for (target, inp), (src, out) in mdl.connections.items():
                setattr(ts[target], inp, np.array(getattr(ts[src], out)))
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
        Length of the buffer in samples, defaults to 1.
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
